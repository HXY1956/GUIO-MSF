#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <math.h>
#include "hwa_gnss_data_navsbs.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_base_typeconv.h"

namespace hwa_gnss
{
    gnss_data_navsbs::gnss_data_navsbs()
        : gnss_data_nav(),
          _toc(base_time::GPS)
    {
        id_type(base_data::EPHSBS);
        id_group(base_data::GRP_EPHEM);
    }
    gnss_data_navsbs::gnss_data_navsbs(base_log spdlog)
        : gnss_data_nav(spdlog),
          _toc(base_time::GPS)
    {

        id_type(base_data::EPHSBS);
        id_group(base_data::GRP_EPHEM);
    }

    /* --- */
    gnss_data_navsbs::~gnss_data_navsbs()
    {
    }

    // check message // NEED TO IMPROVE !
    // ----------
    int gnss_data_navsbs::chk(std::set<std::string> &msg)
    {
        if (!_healthy())
        {
            msg.insert("Issue: " + _sat + " nav unhealthy satellite " + _toc.str_ymdhms());
            _validity = false;
        }

        if (_tki >= 9.9999e8)
        {
            msg.insert("Issue: " + _sat + " nav too large value [tki] " + base_type_conv::dbl2str(_tki));
            _validity = false;
        }
        return 0;
    }

    /* --- */
    /*int gnss_data_navsbs::ura( double acc ) const
{
   int i; 
   for( i = 0; i < (int)sizeof(ura_eph); i++)
     if(acc>ura_eph[i]) break;
   
   return i;
}
*/

    // local function
    // ----------
    int gnss_data_navsbs::pos(const base_time &t, double xyz[], double var[], double vel[], bool chk_health)
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
        double Tk = t.diff(_toc); // T - toe difference

        //  std::cout << "t: " << t.str_ymdhms() << " " << t.sys() << " toe: " << _toc.str_ymdhms() << " " << _toc.sys() << std::endl;
        //  std::cout << "Tk: " << Tk << std::endl;

        if (fabs(Tk) > MAX_SBS_TIMEDIFF)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "CRD " + _sat + ": The user time and Glonass ephemerides epoch differ too much. TOE: " + _toc.str_ymdhms() + " T: " + t.str_ymdhms());
            return -1;
        }

        if (double_eq(_x, 0.0) || double_eq(_y, 0.0) || double_eq(_z, 0.0))
        {
            return -1;
        }

        xyz[0] = _x + _x_d * Tk + _x_dd * Tk * Tk / 2.0;
        xyz[1] = _y + _y_d * Tk + _y_dd * Tk * Tk / 2.0;
        xyz[2] = _z + _z_d * Tk + _z_dd * Tk * Tk / 2.0;

        //  std::cout << sat() << " " << t.str_hms() << " " << xyz[0] << " " << xyz[1] << " " << xyz[2] << std::endl;

        // velocity at positon t
        if (vel)
        {
            vel[0] = _x_dd;
            vel[1] = _y_dd;
            vel[2] = _z_dd;
        }
        return 0;
    }

    /* ---
   t is glo time of transmission, 
   i.e. glo time corrected for transit time (range/speed of light)

*/
    int gnss_data_navsbs::clk(const base_time &t, double *clk, double *var, double *dclk, bool chk_health)
    {

        if (sat().empty())
            return -1; // not valid !!!

        if (chk_health && _healthy() == false)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "not healthy sat " + sat() + " excluded from clk calculation " + t.str_ymdhms());
            return -1; // HEALTH NOT OK
        }
        //  std::cout << "gnavsbs: CLK " << _sat << ": The user time and SBAS ephemerides epoch differs too much. TOE: " << _toc.str_ymdhms() << " T: " << t.str_ymdhms() << std::endl;

        double Tk = t.diff(_toc); // T - toe difference

        //std::cout << "Tk = " << Tk << " maxEphAge = " << MAX_SBS_TIMEDIFF << std::endl;
        if (fabs(Tk) > MAX_SBS_TIMEDIFF)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "CLK " + _sat + ": The user time and SBAS ephemerides epoch differs too much. TOE: " + _toc.str_ymdhms() + " T: " + t.str_ymdhms());
            return -1;
        }

        for (int i = 0; i < 2; i++)
        {
            Tk -= _f0 + _f1 * Tk;
        }

        *clk = _f0 + _f1 * Tk;
        return 0;
    }

    int gnss_data_navsbs::data2nav(std::string sat, const base_time &ep, const gnss_data_navDATA &data)
    {
        if (sat.substr(0, 1) == "S")
            _sat = sat;
        else
            _sat = "S" + sat;

        _epoch = ep;
        _toc = ep;
        _f0 = data[0];
        _f1 = data[1];
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

        _C_rms = data[10];

        _z = data[11] * 1.e3;
        _z_d = data[12] * 1.e3;
        _z_dd = data[13] * 1.e3;

        _iod = data[14];
        return 0;
    }

    // convert gnav_sbs element to general gnavdata
    // ----------
    int gnss_data_navsbs::nav2data(gnss_data_navDATA &data)
    {
        data[0] = _f0;
        data[1] = _f1;
        data[2] = _tki; //  if (_tki < 0) _tki += 86400;

        data[3] = _x / 1.e3;
        data[4] = _x_d / 1.e3;
        data[5] = _x_dd / 1.e3;

        data[6] = _health;

        data[7] = _y / 1.e3;
        data[8] = _y_d / 1.e3;
        data[9] = _y_dd / 1.e3;

        data[10] = _C_rms;

        data[11] = _z / 1.e3;
        data[12] = _z_d / 1.e3;
        data[13] = _z_dd / 1.e3;

        data[14] = _iod;
        return 0;
    }

    // get parameter value
    // ----------
    hwa_map_time_value gnss_data_navsbs::param(const NAVDATA &n)
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
            tmp = std::make_pair(_toc, _iod * 1e0);
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
    int gnss_data_navsbs::param(const NAVDATA &n, double val)
    {
        switch (n)
        { // SELECTED only, ! use the same MULTIPLICATOR as in param()

        case NAV_IOD:
            _iod = val / 1.e0;
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
    std::string gnss_data_navsbs::line() const
    {

        int w = 20;
        std::ostringstream tmp;

        tmp << " " << std::setw(3) << sat()
            << " " << _toc.str("%Y-%m-%d %H:%M:%S")
            << std::scientific << std::setprecision(12)
            << std::setw(w) << _f0
            << std::setw(w) << _f1
            << std::setw(w) << _tki
            << std::setw(w) << _x
            << std::setw(w) << _x_d
            << std::setw(w) << _x_dd
            << std::setw(w) << _health
            << std::setw(w) << _y
            << std::setw(w) << _y_d
            << std::setw(w) << _y_dd
            << std::setw(w) << _C_rms
            << std::setw(w) << _z
            << std::setw(w) << _z_d
            << std::setw(w) << _z_dd
            << std::setw(w) << _iod;

        return tmp.str();
    }

    // print function
    // ----------
    std::string gnss_data_navsbs::linefmt() const
    {

        std::ostringstream tmp;

        tmp << " " << std::setw(3) << sat() << std::fixed
            << " " << _toc.str("%Y-%m-%d %H:%M:%S")
            //     << " " << _toe.str("%Y-%m-%d %H:%M:%S")
            //     << " " << _tot.str("%Y-%m-%d %H:%M:%S")
            << std::fixed << std::setprecision(0)

            << std::setw(8) << 0
            << std::setw(8) << _C_rms
            << std::setw(4) << _iod
            << std::setw(4) << _health
            << " |"
            << std::setw(12) << std::setprecision(3) << _f0 * 1e9 //   [sec]
            << std::setw(8) << std::setprecision(3) << _f1 * 1e9  //   [sec]
            << std::setw(8) << std::setprecision(0) << _tki * 1e0 //
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
    bool gnss_data_navsbs::_healthy() const
    {
        if (_health == 0)
            return true;
        return false;
    }

} // namespace
