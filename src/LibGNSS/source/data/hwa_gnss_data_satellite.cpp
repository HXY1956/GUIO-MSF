#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "hwa_gnss_data_satellite.h"
#include "hwa_gnss_model_ephplan.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_satellite::gnss_data_satellite() : base_data()
    {
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }

    gnss_data_satellite::gnss_data_satellite(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }

    // -----------
    gnss_data_satellite::gnss_data_satellite(base_log spdlog, gnss_data_obs_manager *obs)
        : base_data(spdlog),
          _satcrd(0.0, 0.0, 0.0),
          _clk(0.0),
          _dclk(0.0),
          _ele(0.0),
          _azi(0.0),
          _rho(0.0),
          _eclipse(false)
    {
        gobs = obs;
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
        _lastEcl = FIRST_TIME;
    }

    // -----------
    gnss_data_satellite::~gnss_data_satellite()
    {
#ifdef DEBUG
        cout << "gsatgnss - destruct POCAT: " << site() << " " << sat() << " "
             << epoch().str("  %Y-%m-%d %H:%M:%S[%T] ") << fixed << setprecision(3) << endl;

        if (gobs)
        {
            vector<GOBS> v_obs = gobs->obs();
            vector<GOBS>::iterator itOBS = v_obs.begin();
            for (; itOBS != v_obs.end(); ++itOBS)
                cout << " " << t_gobs::gobs2str(*itOBS) << ":" << this->getobs(*itOBS);
            cout << endl;

            cout << "gsatgnss - destruct KONEC: " << site() << " " << sat() << " "
                 << epoch().str("  %Y-%m-%d %H:%M:%S[%T] ");
            cout.flush();
        }
#endif
    }

    // -----------
    void gnss_data_satellite::addcrd(const Triple &crd)
    {
        _satcrd = crd;
        return;
    }

    //----------
    int gnss_data_satellite::addprd(gnss_all_nav *gnav, bool msk_health)
    {

        if (!gobs)
            return -1;

        double P3 = gobs->P3();
        double L3 = gobs->L3();

        if (P3 == 0 || L3 == 0 || gnav == 0)
            return -1;

        string satname(gobs->sat());
        if (satname.substr(0, 1) != "G" &&
            satname.substr(0, 1) != "R")
            return -1;
        double xyz[3] = {0.0, 0.0, 0.0};
        double vel[3] = {0.0, 0.0, 0.0};
        double var[3] = {0.0, 0.0, 0.0};
        double clk = 0.0;
        double dclk = 0.0;
        double clkrms = 0.0;

        base_time epoT(base_time::GPS);
        double satclk = 0.0;
        double satclk2 = 1.0;

        while (fabs(satclk - satclk2) > 1.e-3 / CLIGHT)
        {
            satclk2 = satclk;
            epoT = gobs->epoch() - P3 / CLIGHT - satclk;
            int irc = gnav->clk(satname, epoT, &clk, &clkrms, &dclk, msk_health);

            if (irc < 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + gobs->epoch().str_ymdhms(" clocks not calculated for epoch "));
                //     cout << satname + gobs->epoch().str_ymdhms(" clocks not calculated for epoch ") << endl;
                return -1;
            }
            satclk = clk;
        }

        int irc = gnav->pos(satname, epoT, xyz, var, vel, msk_health);

        if (irc < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + gobs->epoch().str_ymdhms(" coordinates not calculated for epoch "));
            //     cout << satname + gobs->epoch().str_ymdhms(" coordinates not calculated for epoch ") << endl;
            return -1;
        }

        Triple txyz(xyz);
        // phase center ofset correction
        /* if( gobj != 0 ){ */
        /*    t_gobj* obj  = gobj->obj( satname ); */
        /*    t_gpcv* apc  =   obj->pcv( epoT ); */
        /*    if ( apc != 0 ) { */
        /*      const char* s = satname.substr(0,1).c_str(); */
        /*      gnss_sys sys(s[0]); */
        /*      apc->pco(epoT, txyz, sys, "L3");      */
        /*    }       */
        /* } */

        // relativistic correction
        satclk -= 2.0 * (txyz[0] * vel[0] + txyz[1] * vel[1] + txyz[2] * vel[2]) / CLIGHT / CLIGHT;

        // filling gsatgnss
        _satcrd = txyz;
        _clk = satclk * CLIGHT;
        _dclk = dclk * CLIGHT;
        addecl(txyz, epoT);

#ifdef DEBUG
        cout << satname
             << " CRD " << fixed << setprecision(3)
             << "  " << epoT.str_ymdhms()
             << " X " << setw(14) << xyz[0]
             << " Y " << setw(14) << xyz[1]
             << " Z " << setw(14) << xyz[2]
             << " T " << setw(14) << clk * 1000.0 * 1000.0
             << endl;
//   int ooo; cin >> ooo;
#endif

#ifdef BMUTEX
        lock.unlock();
#endif
        return 1;
    }

    // -----------
    void gnss_data_satellite::addclk(double clk)
    {
        _clk = clk;
        return;
    }

    // -----------
    void gnss_data_satellite::addele(double ele)
    {
        _ele = ele;
        return;
    }

    // -----------
    void gnss_data_satellite::addazi(double azi)
    {
        _azi = azi;
        return;
    }

    // -----------
    void gnss_data_satellite::addrho(double rho)
    {
        _rho = rho;
        return;
    }

    // -----------
    Triple gnss_data_satellite::satcrd()
    {
        Triple crd(_satcrd[0], _satcrd[1], _satcrd[2]);
        return crd;
    }

    // -----------
    double gnss_data_satellite::clk()
    {
        double tmp = _clk;
        return tmp;
    }

    double gnss_data_satellite::dclk()
    {
        double tmp = _dclk;
        return tmp;
    }

    // -----------
    double gnss_data_satellite::ele()
    {
        double tmp = _ele;
        return tmp;
    }

    // -----------
    double gnss_data_satellite::ele_deg()
    {
        double tmp = _ele * 180.0 / hwa_pi;
        return tmp;
    }

    // -----------
    double gnss_data_satellite::azi()
    {
        double tmp = _azi;
        return tmp;
    }

    // -----------
    double gnss_data_satellite::rho()
    {
        double tmp = _rho;
        return tmp;
    }

    // valid
    // ----------
    bool gnss_data_satellite::valid()
    {
        bool tmp = _valid();
        return tmp;
    }

    // clean data
    // ----------
    void gnss_data_satellite::clear()
    {
        _clear();
        return;
    }

    // clean internal function
    // ----------
    void gnss_data_satellite::_clear()
    {

        //  gnss_data_obs_manager::_clear(); // DELETE!
        //  _satcrd[0] = 0.0;
        //  _satcrd[1] = 0.0;
        //  _satcrd[2] = 0.0;
        _ele = 0.0;
        _azi = 0.0;
        _rho = 0.0;
        _clk = 0.0;
        _dclk = 0.0;
    }

    // clean internal function
    // ----------
    bool gnss_data_satellite::_valid() const
    {

        // single validity identification for gsatgnss!
        if (_rho == 0.0)
            return false;
        //if( gobs && ! gobs->_valid() ) return false;

        return true;
    }

    // get eclipsing
    // ---------------------
    bool gnss_data_satellite::ecl()
    {

        return _eclipse;
    }

    // determi ne wheather eclipsed or not
    // -------------------------------------
    void gnss_data_satellite::addecl(Triple &sat, base_time &epoch)
    {

        gnss_model_ephplan plan;
        double mjd = epoch.dmjd();

        Triple sun = plan.sunPos(mjd);

        // Unit vector Earth - Sun
        Vector eSun = sun / sun.norm();

        // Unit vector Earth - Satellite
        Vector eSat = sat / sat.norm();

        double cosin = eSun.dot(eSat);
        if (cosin < 0)
        {
            double r = sat.norm();
            double val = r * sqrt(1 - cosin * cosin);
            if (val < Aell)
            {
                _eclipse = true;
                _lastEcl = epoch;
                return;
            }
            else
            {
                _eclipse = false;
            }
        }

        double tdiff = epoch.diff(_lastEcl);
        if (tdiff <= 1800)
            _eclipse = true;
    }

} // namespace
