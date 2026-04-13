#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <math.h>
#include "hwa_gnss_data_navglo.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_base_typeconv.h"

using namespace hwa_base;

namespace hwa_gnss
{
    gnss_data_navglo::gnss_data_navglo()
        : gnss_data_nav(),
          _iodc(0),
          _toc(base_time::UTC)
    {
        id_type(base_data::EPHGLO);
        id_group(base_data::GRP_EPHEM);
        _maxEphAge = MAX_GLO_TIMEDIFF; // a bit above 900; //[s]
        _min_step = 10;
    }
    gnss_data_navglo::gnss_data_navglo(base_log spdlog)
        : gnss_data_nav(spdlog),
          _iodc(0),
          _toc(base_time::UTC)
    {

        id_type(base_data::EPHGLO);
        id_group(base_data::GRP_EPHEM);

        _maxEphAge = MAX_GLO_TIMEDIFF; // a bit above 900; //[s]
        _min_step = 10;
    }

    /* --- */
    gnss_data_navglo::~gnss_data_navglo()
    {
    }

    // check message // NEED TO IMPROVE !
    // ----------
    int gnss_data_navglo::chk(std::set<std::string> &msg)
    {
        _min_step = 225; // to speed up

        if (!_healthy())
        {
            msg.insert("Mesg: " + _sat + " unhealthy satellite " + _toc.str_ymdhms());
        }

        double clkB[1] = {0.0};
        double clkF[1] = {0.0};
        if (_clk(_toc - _maxEphAge, clkB) < 0 ||
            _clk(_toc + _maxEphAge, clkF) < 0 ||
            fabs(*clkF - *clkB) * CLK_GLO_FACTOR > MAX_GLO_CLKDIF)
        {
            msg.insert("Issue: " + _sat + " nav not correct [clk] " + _toc.str_ymdhms() + base_type_conv::dbl2str(fabs(*clkF - *clkB) * CLK_GLO_FACTOR));
            _validity = false;
        }
        *clkB *= CLK_GLO_FACTOR;
        *clkF *= CLK_GLO_FACTOR;

        double xyzB[3] = {0.0, 0.0, 0.0};
        double xyzF[3] = {0.0, 0.0, 0.0};
        if (_pos(_toc - _maxEphAge, xyzB) < 0 ||
            _pos(_toc + _maxEphAge, xyzF) < 0)
        {
            msg.insert("Issue: " + _sat + " nav not correct [pos] " + _toc.str_ymdhms() + base_type_conv::dbl2str(xyzF[0] - xyzB[0]));
            _validity = false;
        }

        double radB2 = xyzB[0] * xyzB[0] + xyzB[1] * xyzB[1] + xyzB[2] * xyzB[2];
        double radF2 = xyzF[0] * xyzF[0] + xyzF[1] * xyzF[1] + xyzF[2] * xyzF[2];
        if (radB2 < 0 || radF2 < 0)
        {
            msg.insert("Issue: " + _sat + " nav not correct [rad2] " + _toc.str_ymdhms());
            _validity = false;
        }

        double radB = sqrt(radB2) * RAD_GLO_FACTOR;
        double radF = sqrt(radF2) * RAD_GLO_FACTOR;
        if (radB < MIN_GLO_RADIUS || radB > MAX_GLO_RADIUS ||
            radF < MIN_GLO_RADIUS || radF > MAX_GLO_RADIUS)
        {
            msg.insert("Issue: " + _sat + " nav not correct [rad] " + _toc.str_ymdhms() + base_type_conv::dbl2str(radF) + base_type_conv::dbl2str(radB));
            _validity = false;
        }

        if (fabs(radF - radB) > MAX_GLO_RADDIF)
        {
            msg.insert("Issue: " + _sat + " nav not correct [rad-diff] " + _toc.str_ymdhms() + base_type_conv::dbl2str(fabs(radF - radB)));
            _validity = false;
        }

        int sodfrac = _toc.sod() % 3600;
        if (sodfrac == 900 ||
            sodfrac == 2700)
        {
        }
        else
        {
            msg.insert("Issue: " + _sat + " nav unexpected [toc] " + _toc.str_ymdhms());
            _validity = false;
        }

        if (_freq_num > 240)
        { // 255 = -1, 254 = -2, 253 = -3 atd
            _freq_num -= 256;
            msg.insert("Warn: " + _sat + " corrected [channel] " + base_type_conv::int2str(_freq_num + 256) + " -> " + base_type_conv::int2str(_freq_num));
        }

#ifdef DEBUG
        std::cerr << "ok: navglo " + _toc.str_ymdhms(_sat + " ") << std::fixed << std::setprecision(3)
             << " radB: " << radB << " radF: " << radF << " diff: " << fabs(radF - radB)
             << " clkB: " << *clkB << " clkF: " << *clkF << " diff: " << fabs(*clkF - *clkB)
             << std::endl;
#endif
        return 0;
    }

    /* --- */
    int gnss_data_navglo::channel() const
    {

        if (_freq_num < -7 || _freq_num > 13)
        {
            return 256;
        }

        /* if( nav.iodc < 0 || 1023 < nav.iodc ){     */
        /*   std::cout << "rinex nav invalid iodc: iodc" << nav.iodc << std::endl; */
        /*   return -1; */
        /* } */

        return _freq_num;
    }

    /* --- */
    /*int gnss_data_navglo::ura( double acc ) const
{
   int i; 
   for( i = 0; i < (int)sizeof(ura_eph); i++)
     if(acc>ura_eph[i]) break;
   
   return i;
}
*/

    /* ---
   t is glo time of transmission, 
   i.e. glo time corrected for transit time (range/speed of light)

*/

    // local function
    // ----------
    int gnss_data_navglo::pos(const base_time &t, double xyz[], double var[], double vel[], bool chk_health)
    {
        _min_step = 10;

        int i = _pos(t, xyz, var, vel, chk_health);
        return i;
    }

    // local function
    // ----------
    int gnss_data_navglo::nav(const base_time &t, double xyz[], double var[], double vel[], bool chk_health)
    {
        _min_step = 225;

        int i = _pos(t, xyz, var, vel, chk_health);
        return i;
    }

    // local function
    // ----------
    int gnss_data_navglo::clk(const base_time &t, double *clk, double *var, double *dclk, bool chk_health)
    {
        int i = _clk(t, clk, var, dclk, chk_health);
        return i;
    }

    // local function
    // ----------
    int gnss_data_navglo::_pos(const base_time &t, double xyz[], double var[], double vel[], bool chk_health)
    {

        if (sat().empty())
            return -1; // not valid !!!
        if (chk_health && _healthy() == false)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "not healthy sat " + sat() + " excluded from pos calculation " + t.str_ymdhms());
            return -1; // HEALTH NOT OK
        }

        xyz[0] = xyz[1] = xyz[2] = 0.0;
        if (var)
            var[0] = var[1] = var[2] = 0.0;
        if (vel)
            vel[0] = vel[1] = vel[2] = 0.0;

        //  std::cout << "t = " << t.str_hms() << " toc = " << _toc.str_hms("", true) << std::endl;
        double Tk = t.diff(_toc); // T - toc difference
                                  //  std::cout << " t: "   <<    t.str_ymdhms() << " " <<    t.sys()
                                  //       << " toc: " << _toc.str_ymdhms() << " " << _toc.sys()
                                  //       << " Tk: "  << Tk << std::endl;

        if (fabs(Tk) > _maxEphAge)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "CRD " + _sat + ": The user time and GLONASS ephemerides epoch differ too much. TOC: " + _toc.str_ymdhms() + " T: " + t.str_ymdhms());
            return -1;
        }

        // init state std::vector (crd, vel)
        Vector yy(6);

        if (double_eq(_x, 0.0) || double_eq(_y, 0.0) || double_eq(_z, 0.0))
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Zero ephemerides:" + t.str_ymdhms(sat() + " "));
            return -1;
        }

        yy << _x, _y, _z, _x_d, _y_d, _z_d;
        //std::cout << _x << " " << _y << " " << _z << std::endl;
        // acceleration
        Triple acc(_x_dd, _y_dd, _z_dd);

        int nsteps = (int)fabs(floor(Tk / _min_step + 0.5)); // step number for RungeKutta
        double step = fabs(Tk) / nsteps;                     // step length for RungeKutta, length is cca 10s
                                                             //  std::cout << Tk << " " << nsteps << " " << step << std::endl;
        if (Tk < 0)
            step = -step;
        //  std::cout << "nstep: " << nsteps << " step: " << step << std::endl;
        Vector yy_integr(6);
        yy_integr = _RungeKutta(step, nsteps, yy, acc);

        xyz[0] = yy_integr(1);
        xyz[1] = yy_integr(2);
        xyz[2] = yy_integr(3);
        //  std::cout << t.str_hms() << std::endl << "PZ90: " << std::fixed << std::setprecision(3) << std::setw(15) << xyz[0] << std::setw(15) << xyz[1] << std::setw(15) << xyz[2] << std::endl;

        // PZ_90.11 to ITRF_2008 transformation
        Vector T(3);
        T << -0.003, -0.001, 0.000;
        Matrix R(3, 3);
        R.setZero();
        double mas = (1 / (36e5)) * D2R;
        R << 1, 0.002 * mas, 0.042 * mas
            , -0.002 * mas, 1, 0.019 * mas
            , -0.042 * mas, -0.019 * mas, 1;
        Vector xyzWGS;
        xyzWGS = T + R * yy_integr.segment(0, 3);

        // position at time t
        xyz[0] = xyzWGS(0);
        xyz[1] = xyzWGS(1);
        xyz[2] = xyzWGS(2);

        //std::cout << "gallnav:";
        //std::cout  << "ITRF: " << std::fixed << std::setprecision(3) << std::setw(15) << xyz[0] << std::setw(15) << xyz[1] << std::setw(15) << xyz[2] << std::endl;

        // velocity at positon t
        if (vel)
        {
            vel[0] = yy_integr(3);
            vel[1] = yy_integr(4);
            vel[2] = yy_integr(5);
        }

        return 0;
    }

    /* ---
   t is glo time of transmission, 
   i.e. glo time corrected for transit time (range/speed of light)

*/
    int gnss_data_navglo::_clk(const base_time &t, double *clk, double *var, double *dclk, bool chk_health)
    {

        if (sat().empty())
            return -1; // not valid !!!
        if (chk_health && _healthy() == false)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "not healthy sat " + sat() + " excluded from clk calculation " + t.str_ymdhms());
            return -1; // HEALTH NOT OK
        }

        //  std::cout << "gnavglo: CLK " << _sat << ": The user time and GLONASS ephemerides epoch differs too much. TOC: " << _toc.str_ymdhms() << " T: " << t.str_ymdhms() << std::endl;

        double Tk = t.diff(_toc); // T - toc difference

        // std::cout << "Tk = " << Tk << " maxEphAge = " << _maxEphAge << std::endl;
        if (fabs(Tk) > _maxEphAge)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "CLK " + _sat + ": The user time and GLONASS ephemerides epoch differs too much. TOC: " + _toc.str_ymdhms() + " T: " + t.str_ymdhms());
            return -1;
        }

        for (int i = 0; i < 2; i++)
        {
            Tk -= -_tau + _gamma * Tk;
        }

        *clk = -_tau + _gamma * Tk;

        // relativity correction is excluded
        // uniquely is applied in gpppfilter
        /* double xyz[3] = {0.0, 0.0, 0.0}; */
        /* double vel[3] = {0.0, 0.0, 0.0}; */
        /* double varPos[3] = {0.0, 0.0, 0.0}; */
        /* if (this->_pos(t, xyz, varPos, vel) < 0){ return -1;} */
        /* double relcor = -2.0 * ( xyz[0]*vel[0] + xyz[1]*vel[1] + xyz[2]*vel[2] ) /CLIGHT /CLIGHT; */
        /* *clk -= relcor; */

        return 0;
    }

    int gnss_data_navglo::data2nav(std::string sat, const base_time &ep, const gnss_data_navDATA &data)
    {
        if (sat.find("R") != std::string::npos)
        {
            _sat = sat;
        }
        else
        {
            std::ostringstream tmp;
            tmp << std::setw(1) << 'R' << std::setfill('0') << std::setw(2) << sat;
            _sat = tmp.str();
        }
        //  std::cout << "prn = " << sat << " " << _sat << std::endl;
        //
        //  if( sat.substr(0,1) == "R" )  _sat =     sat;
        //  else                          _sat = "R"+sat;
        //  if( !strncmp(sat,"R",1) )  _sat = std::string(sat);     // strcpy(_sat,sat);
        //  else                       _sat = "R"+string(sat); // sprintf(_sat,"%c%02i", SYS_GLO, atoi(sat));

        _epoch = ep;
        _toc = ep;
        _iodc = _iod();
        _tau = -data[0]; // in RINEX is stored -tauN
        _gamma = data[1];
        _tki = data[2];
        if (_tki < 0)
            _tki += 86400;

        _x = data[3] * 1.e3;
        _x_d = data[4] * 1.e3;
        _x_dd = data[5] * 1.e3;

        _health = data[6];

        _y = data[7] * 1.e3;
        _y_d = data[8] * 1.e3;
        _y_dd = data[9] * 1.e3;

        _freq_num = (int)data[10];

        _z = data[11] * 1.e3;
        _z_d = data[12] * 1.e3;
        _z_dd = data[13] * 1.e3;

        _E = data[14];
        return 0;
    }

    // convert gnav_glo element to general gnavdata
    // ----------
    int gnss_data_navglo::nav2data(gnss_data_navDATA &data)
    {
        if (!this->_valid())
            return -1;

        data[0] = -_tau; // in RINEX is stored -tauN
        data[1] = _gamma;
        data[2] = _tki; //  if (_tki < 0) _tki += 86400;

        data[3] = _x / 1.e3;
        data[4] = _x_d / 1.e3;
        data[5] = _x_dd / 1.e3;

        data[6] = _health;

        data[7] = _y / 1.e3;
        data[8] = _y_d / 1.e3;
        data[9] = _y_dd / 1.e3;

        data[10] = _freq_num;

        data[11] = _z / 1.e3;
        data[12] = _z_d / 1.e3;
        data[13] = _z_dd / 1.e3;

        data[14] = _E;
        return 0;
    }

    // get parameter value
    // ----------
    hwa_map_time_value gnss_data_navglo::param(const NAVDATA &n)
    {
        hwa_map_time_value tmp;
        switch (n)
        {
        case NAV_X:
            tmp = std::make_pair(_toc, _x * 1e0);
            break; // meters
        case NAV_XD:
            tmp = std::make_pair(_toc, _x_d * 1e0);
            break; // meters
        case NAV_XDD:
            tmp = std::make_pair(_toc, _x_dd * 1e0);
            break; // meters
        case NAV_Y:
            tmp = std::make_pair(_toc, _y * 1e0);
            break; // meters
        case NAV_YD:
            tmp = std::make_pair(_toc, _y_d * 1e0);
            break; // meters
        case NAV_YDD:
            tmp = std::make_pair(_toc, _y_dd * 1e0);
            break; // meters
        case NAV_Z:
            tmp = std::make_pair(_toc, _z * 1e0);
            break; // meters
        case NAV_ZD:
            tmp = std::make_pair(_toc, _z_d * 1e0);
            break; // meters
        case NAV_ZDD:
            tmp = std::make_pair(_toc, _z_dd * 1e0);
            break; // meters

        case NAV_IOD:
            tmp = std::make_pair(_toc, _iodc * 1e0);
            break; //
        case NAV_HEALTH:
            tmp = std::make_pair(_toc, _health * 1e0);
            break; //

        default:
            break;
        }
        return tmp;
    }

    // std::set parameter value
    // ----------
    int gnss_data_navglo::param(const NAVDATA &n, double val)
    {
        switch (n)
        { // SELECTED only, ! use the same MULTIPLICATOR as in param()

        case NAV_IOD:
            _iodc = val / 1.e0;
            break;
        case NAV_HEALTH:
            _health = val / 1.e0;
            break;

        default:
            break;
        }
        return 0;
    }

    // print function
    // ----------
    std::string gnss_data_navglo::line() const
    {

        int w = 20;
        std::ostringstream tmp;

        tmp << " " << std::setw(3) << sat()
            << " " << _toc.str("%Y-%m-%d %H:%M:%S")
            << std::scientific << std::setprecision(12)
            << std::setw(w) << _tau
            << std::setw(w) << _gamma
            << std::setw(w) << _tki
            << std::setw(w) << _x
            << std::setw(w) << _x_d
            << std::setw(w) << _x_dd
            << std::setw(w) << _health
            << std::setw(w) << _y
            << std::setw(w) << _y_d
            << std::setw(w) << _y_dd
            << std::setw(w) << _freq_num
            << std::setw(w) << _z
            << std::setw(w) << _z_d
            << std::setw(w) << _z_dd
            << std::setw(w) << _E;

        return tmp.str();
    }

    // print function
    // ----------
    std::string gnss_data_navglo::linefmt() const
    {

        std::ostringstream tmp;

        tmp << " " << std::setw(3) << sat() << std::fixed
            << " " << _toc.str("%Y-%m-%d %H:%M:%S")
            //     << " " << _toe.str("%Y-%m-%d %H:%M:%S")
            //     << " " << _tot.str("%Y-%m-%d %H:%M:%S")
            << std::fixed << std::setprecision(0)

            << std::setw(8) << _E
            << std::setw(8) << _freq_num
            << std::setw(4) << _iodc
            << std::setw(4) << _health
            << " |"
            << std::setw(12) << std::setprecision(3) << _tau * 1e9  //   [sec]
            << std::setw(8) << std::setprecision(3) << _gamma * 1e9 //   [sec]
            << std::setw(8) << std::setprecision(0) << _tki * 1e0   //
            << " |"
            << std::setw(14) << std::setprecision(3) << _x * 1e0   // 1 [km]
            << std::setw(11) << std::setprecision(3) << _x_d * 1e0 // 2 [km/s]
            << std::setw(9) << std::setprecision(3) << _x_dd * 1e6 // 3 [km/s^2]
            << " |"
            << std::setw(14) << std::setprecision(3) << _y * 1e0   // 1 [km]
            << std::setw(11) << std::setprecision(3) << _y_d * 1e0 // 2 [km/s]
            << std::setw(9) << std::setprecision(3) << _y_dd * 1e6 // 3 [km/s^2]
            << " |"
            << std::setw(14) << std::setprecision(3) << _z * 1e0   // 1 [km]
            << std::setw(11) << std::setprecision(3) << _z_d * 1e0 // 2 [km/s]
            << std::setw(9) << std::setprecision(3) << _z_dd * 1e6 // 3 [km/s^2]
            ;

        return tmp.str();
    }

    // healthy check
    // ----------
    bool gnss_data_navglo::_healthy() const
    {
        if (_health == 0)
            return true;
        return false;
    }

    // six orbital differential equations
    // ------------------------------------
    Vector gnss_data_navglo::_deriv(const Vector&xx, const Triple &acc)
    {

        Triple crd = xx.segment(0, 3);
        Triple vel = xx.segment(3, 3);

        double r = crd.norm();

        double k1 = -GM_PZ90 / (r * r * r);
        double k2 = (3.0 / 2.0) * C20_PZ90 * (GM_PZ90 * Aell_PZ90 * Aell_PZ90) / pow(r, 5);

        Vector xxdot(6);

        xxdot[0] = vel[0];
        xxdot[1] = vel[1];
        xxdot[2] = vel[2];
        xxdot[3] = k1 * crd[0] + k2 * (1.0 - 5.0 * crd[2] * crd[2] / (r * r)) * crd[0] + OMEGA * OMEGA * crd[0] + 2 * OMEGA * vel[1] + acc[0];
        xxdot[4] = k1 * crd[1] + k2 * (1.0 - 5.0 * crd[2] * crd[2] / (r * r)) * crd[1] + OMEGA * OMEGA * crd[1] - 2 * OMEGA * vel[0] + acc[1];
        xxdot[5] = k1 * crd[2] + k2 * (3.0 - 5.0 * crd[2] * crd[2] / (r * r)) * crd[2] + acc[2];

        return xxdot;
    }

    // Runge-Kutta integration
    // -----------------------------------
    Vector gnss_data_navglo::_RungeKutta(double step, int nsteps, const Vector &yy, const Triple &acc)
    {

        Vector yyn = yy;

        for (int i = 1; i <= nsteps; i++)
        {
            Vector k1 = step * _deriv(yyn, acc);
            Vector k2 = step * _deriv(yyn + k1 / 2.0, acc);
            Vector k3 = step * _deriv(yyn + k2 / 2.0, acc);
            Vector k4 = step * _deriv(yyn + k3, acc);

            yyn += k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0;

#ifdef DEBUG
            std::cout << "k1 = " << k1 << std::endl
                 << "k2 = " << k1 << std::endl
                 << "k3 = " << k1 << std::endl
                 << "k4 = " << k1 << std::endl;
#endif
        }

        return yyn;
    }

    // IOD of GLONASS clocks
    // -----------------------------
    int gnss_data_navglo::_iod() const
    {

        base_time gloTime = _toc + 3 * 3600.0; // toc in GLO (if toc in UTC +3*3600.0;)

        int iod = int(gloTime.sod() / 900);

        // doy is added for making iod unique over days
        int doy = _toc.doy();
        iod += doy;

        return iod;
    }

} // namespace
