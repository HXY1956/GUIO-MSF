#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "hwa_base_typeconv.h"
#include "hwa_gnss_all_prec.h"

namespace hwa_gnss
{
    gnss_all_prec::gnss_all_prec() : gnss_all_nav(),
                               _degree_sp3(9),
                               _sec(3600.0 * 6),
                               _ref(base_time::GPS),
                               _clkref(base_time::GPS),
                               _clkrnx(true),
                               _clksp3(false),
                               _clknav(false),
                               _posnav(false),
                               _realtime(false)
    {
        id_type(base_data::ALLPREC);
        id_group(base_data::GRP_EPHEM);
        _com = 1;
        _tend = LAST_TIME;
        _tend_clk = LAST_TIME;
        _tend_sp3 = LAST_TIME;
    }
    gnss_all_prec::gnss_all_prec(base_log spdlog) : gnss_all_nav(spdlog),
                                              _degree_sp3(9),
                                              _sec(3600.0 * 6),
                                              _ref(base_time::GPS),
                                              _clkref(base_time::GPS),
                                              _clkrnx(true),
                                              _clksp3(false),
                                              _clknav(false),
                                              _posnav(false),
                                              _realtime(false)
    {

        id_type(base_data::ALLPREC);
        id_group(base_data::GRP_EPHEM);

        _com = 1;
        _tend = LAST_TIME;
        _tend_clk = LAST_TIME;
        _tend_sp3 = LAST_TIME;
    }

    // destructor
    // ----------
    gnss_all_prec::~gnss_all_prec()
    {

        _mapprec.clear();
    }

    // return satellite health
    // ----------
    bool gnss_all_prec::health(const std::string &sat, const base_time &t)
    {

        if (_mapsat.size() > 0)
            return gnss_all_nav::health(sat, t);

        return true; // default healthy if no presence of NAV
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // ----------
    int gnss_all_prec::pos(const std::string &sat, const base_time &t, double xyz[3], double var[3], double vel[3], const bool &chk_mask)
    {
        std::shared_ptr<gnss_data_eph> tmp;
        if (!_realtime || (_realtime && _ultrasp3)) // modified by glfeng
        {
            tmp = gnss_all_prec::_find(sat, t);
        }
        if (tmp == _null)
        {
            for (int i = 0; i < 3; i++)
            {
                xyz[i] = 0.0;
                if (var)
                    var[i] = 0.0;
                if (vel)
                    vel[i] = 0.0;
            }
            if (_posnav && gnss_all_nav::pos(sat, t, xyz, var, vel, chk_mask) >= 0)
            {
                return 1;
            } // std::cout << "USING POS NAV:\n"; return 1; }
            return -1;
        }

        int irc = tmp->pos(t, xyz, var, vel, _chkHealth && chk_mask);
        return irc;
    }

    bool gnss_all_prec::posnew(const std::string &sat, const base_time &t, double xyz[3])
    {
        std::map<base_time, hwa_map_iv>::iterator itReq = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]
        if (itReq == _mapsp3[sat].end())
        {
            return false;
        }
        hwa_map_iv data_t;
        if (abs(itReq->first - t) < 0.01)
        {
            data_t = itReq->second;
            if (data_t.find("X") != data_t.end())
            {
                xyz[0] = data_t["X"];
                xyz[1] = data_t["Y"];
                xyz[2] = data_t["Z"];
            }
            return true;
        }

        double var[3] = {0.0};
        double vel[3] = {0.0};
        bool chk_mask = false;
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_prec::_find(sat, t);
        if (tmp == _null)
        {
            if (_posnav && gnss_all_nav::pos(sat, t, xyz, var, vel, chk_mask) >= 0)
            {
                return 1;
            } // std::cout << "USING POS NAV:\n"; return 1; }
            return false;
        }

        int irc = tmp->pos(t, xyz, var, vel, _chkHealth && chk_mask);
        return irc == 0;
    }
    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // ----------
    int gnss_all_prec::pos_int(const std::string &sat, const base_time &t, double xyz[3], double var[3], double vel[3])
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_prec::_find(sat, t);

        if (tmp == _null)
        {
            for (int i = 0; i < 3; i++)
            {
                xyz[i] = 0.0;
                if (var)
                    var[i] = 0.0;
                if (vel)
                    vel[i] = 0.0;
            }
            return -1;
        }

        int irc = std::dynamic_pointer_cast<gnss_data_ephprec>(tmp)->pos_int(t, xyz, var, vel);
        return irc;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // POSITION directly interpolated from array of SP3 positions
    // ----------
    int gnss_all_prec::pos_alt(const std::string &sat, const base_time &t, double xyz[3], double var[3], double vel[3])
    {
        if (_get_crddata(sat, t) < 0)
        {
            for (int i = 0; i < 3; i++)
            {
                xyz[i] = 0.0;
                if (var)
                    var[i] = 0.0;
                if (vel)
                    vel[i] = 0.0;
            }
            return -1;
        }

        gnss_model_poly poly;
        poly.interpolate(_PT, _X, t.diff(_ref), xyz[0], var[0]);
        poly.interpolate(_PT, _Y, t.diff(_ref), xyz[1], var[1]);
        poly.interpolate(_PT, _Z, t.diff(_ref), xyz[2], var[2]);
        return 1;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // NOT CORRECTED FOR 2nd relativistic effect !!!
    // ----------
    int gnss_all_prec::clk(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk, const bool &chk_mask)
    {
        if (!_clkrnx || _get_clkdata(sat, t) < 0)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            if (_clksp3 && this->clk_int(sat, t, clk, var, dclk) >= 0)
            {
                return 1;
            }
            if (_clknav && gnss_all_nav::clk(sat, t, clk, var, dclk, chk_mask) >= 0)
            {
                return 1;
            }
            return -1;
        }

        gnss_model_poly poly;
        poly.interpolate(_CT, _C, t.diff(_clkref), *clk, *dclk);
        // added by zhshen
        *dclk = *dclk / (_CT.back() - _CT.front());
        return 1;
    }

    int gnss_all_prec::clk_ifcb(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk, const bool &chk_mask)
    {
        if (!_clkrnx || _get_clkdata(sat, t) < 0)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            return -1;
        }

        gnss_model_poly poly;
        poly.interpolate(_CT, _ifcb_F3, t.diff(_clkref), *clk, *dclk);
        return 1;
    }

    int gnss_all_prec::clk_cdr(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk, double *ifcb, const bool &chk_mask)
    {
        if (_mapclk.find(sat) == _mapclk.end())
            return -1;

        std::map<base_time, hwa_map_iv>::iterator itReq = _mapclk[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itReq == _mapclk[sat].end())
            return -1;

        if (itReq->first > t)
            return -1;
        else
        {
            *clk = itReq->second["C0"];
            *var = 0.0;
            *dclk = 0.0;
            *ifcb = itReq->second["IFCB_F3"];
        }
        return 0;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // NOT CORRECTED FOR 2nd relativistic effect !!!
    // ----------
    int gnss_all_prec::clk_sp3(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk)
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_prec::_find(sat, t);

        if (tmp == _null)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            return -1;
        }

        int irc = tmp->clk(t, clk, var, dclk);
        return irc;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // NOT CORRECTED FOR 2nd relativistic effect !!!
    // ----------
    int gnss_all_prec::clk_int(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk)
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_prec::_find(sat, t);

        if (tmp == _null)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            return -1;
        }
        int irc = std::dynamic_pointer_cast<gnss_data_ephprec>(tmp)->clk_int(t, clk, var, dclk);
        return irc;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // CLOCK CORRECTIONS directly interpolated from array of CLOCK-RINEX
    // NOT CORRECTED FOR 2nd relativistic effect !!!
    // ----------
    int gnss_all_prec::clk_alt(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk)
    {
        if (_get_clkdata(sat, t) < 0)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            return -1;
        }

        gnss_model_poly poly;
        poly.interpolate(_CT, _C, t.diff(_clkref), *clk, *dclk);

        // add relativistic correction ! -2rs/c/c
        //  *clk -= 2.0*;
        return 1;
    }

    //yjqin
    void gnss_all_prec::add_interval(const int &intv)
    {
        _intv = intv;
    }

    void gnss_all_prec::add_interval(const std::string &sat, int intv)
    {
        _intvm[sat] = intv;
    }

    void gnss_all_prec::add_agency(const std::string &agency)
    {
        _agency = agency;
    }

    void gnss_all_prec::add_clk_interval(const double &intv)
    {
        _udclkInt = intv;
    }

    int gnss_all_prec::intv()
    {
        return _intv;
    }

    int gnss_all_prec::intv(const std::string &sat)
    {
        return _intvm[sat];
    }

    std::string gnss_all_prec::get_agency()
    {
        return _agency;
    }

    base_time gnss_all_prec::get_beg()
    {
        return _tbeg;
    }

    base_time gnss_all_prec::get_end()
    {
        return _tend;
    }

    std::vector<std::string> gnss_all_prec::get_sats()
    {
        return _sats;
    }

    std::vector<std::string> gnss_all_prec::get_sat3()
    {
        return _sat3;
    }

    std::string gnss_all_prec::get_data_type()
    {
        return _datatype;
    }

    std::string gnss_all_prec::get_sat_type()
    {
        return _sattype;
    }

    int gnss_all_prec::gegnss_coder_sp3_size()
    {
        return _mapsp3.size();
    }

    bool gnss_all_prec::get_pos_vel(const std::string &sat, const base_time &epoch, double pos[3], double vel[3], int &obs_num)
    {
        if (_mapsp3[sat].size() == 0)
            return false; // std::make_shared<gnss_data_eph>();
        // if not exists satellite not in cache
        hwa_map_iv data = _mapsp3[sat][epoch];
        if (data.size() != 0)
        {
            pos[0] = data["X"];
            pos[1] = data["Y"];
            pos[2] = data["Z"];
            obs_num = data["OBS"];
            if (vel)
            {
                vel[0] = data["VX"];
                vel[1] = data["VY"];
                vel[2] = data["VZ"];
            }

            return true;
        }

        return false;
    }

    int gnss_all_prec::get_ssr_iod(const std::string& sat, const base_time& t, int& pv_iod, int& clk_iod)
    {
        if (_mapsp3.find(sat) == _mapsp3.end() || _mapsp3[sat].size() == 0 ||
            _mapclk.find(sat) == _mapclk.end() || _mapclk[sat].size() == 0) {
            /**/ return -1;
        }

        auto pv_itLast = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]
        auto pv_itPrev = --(_mapsp3[sat].lower_bound(t)); // 1st equal|greater [than t]

        if (pv_itLast != _mapsp3[sat].end())
        {
            pv_iod = int(pv_itLast->second["IOD"]);
        }
        else if (pv_itPrev != _mapsp3[sat].end())
        {
            pv_iod = int(pv_itPrev->second["IOD"]);
        }
        else return -1;

        auto clk_itLast = _mapclk[sat].lower_bound(t); // 1st equal|greater [than t]
        auto clk_itPrev = --(_mapclk[sat].lower_bound(t)); // 1st equal|greater [than t]

        if (clk_itLast != _mapclk[sat].end())
        {
            clk_iod = int(clk_itLast->second["IOD"]);
        }
        else if (clk_itPrev != _mapsp3[sat].end())
        {
            clk_iod = int(clk_itPrev->second["IOD"]);
        }
        else return -1;

        return 1;
    }

    void gnss_all_prec::set_beg(const base_time &beg)
    {
        _tbeg = beg;
    }

    void gnss_all_prec::set_end(const base_time &end, const std::string &mode)
    {
        if (mode == "CLK")
        {
            _tend_clk = end;
            _tend = _tend_clk <= _tend_sp3 ? _tend_clk : _tend_sp3;
        }
        else if (mode == "SP3")
        {
            _tend_sp3 = end;
            _tend = _tend_clk <= _tend_sp3 ? _tend_clk : _tend_sp3;
        }
        else
        {
            _tend = end;
        }
    }

    void gnss_all_prec::set_sat(const std::vector<std::string> &sat)
    {
        _sats = sat;
    }

    void gnss_all_prec::set_sat3(const std::vector<std::string> &sat)
    {
        _sat3 = sat;
    }

    void gnss_all_prec::set_data_type(const std::string &type)
    {
        _datatype = type;
    }

    void gnss_all_prec::set_sat_type(const std::string &type)
    {
        _sattype = type;
    }

    void gnss_all_prec::set_agency(const std::string &agency)
    {
        _agency = agency;
    }

    void gnss_all_prec::add_orb_interval(const double &intv)
    {
        _udorbInt = intv;
    }

    int gnss_all_prec::add_delta_pos_vel(const std::string &sat, const base_time &ep, const int &iod, const Triple &dxyz, const Triple &dvxyz)
    {
        if (_overwrite || _mapsp3[sat].find(ep) == _mapsp3[sat].end())
        {

            _mapsp3[sat][ep]["IOD"] = iod;
            _mapsp3[sat][ep]["DX"] = dxyz[0];
            _mapsp3[sat][ep]["DY"] = dxyz[1];
            _mapsp3[sat][ep]["DZ"] = dxyz[2];

            _mapsp3[sat][ep]["DVX"] = dvxyz[0];
            _mapsp3[sat][ep]["DVY"] = dvxyz[1];
            _mapsp3[sat][ep]["DVZ"] = dvxyz[2];
            _realtime = true;
        }
        else
        {
            return -1;
        }
        return 0;
    }
    //yjqin

    // add position
    // ----------
    //int gnss_all_prec::addpos( std::string sat, const base_time& ep, double xyzt[], double dxyz[] )
    int gnss_all_prec::addpos(const std::string &sat, const base_time &ep, const Triple &xyz, const double &t,
                           const Triple &dxyz, const double &dt)
    {
        if (_overwrite || _mapsp3[sat].find(ep) == _mapsp3[sat].end())
        {

            _mapsp3[sat][ep]["X"] = xyz[0];
            _mapsp3[sat][ep]["Y"] = xyz[1];
            _mapsp3[sat][ep]["Z"] = xyz[2];
            _mapsp3[sat][ep]["C"] = t;

            _mapsp3[sat][ep]["SX"] = dxyz[0];
            _mapsp3[sat][ep]["SY"] = dxyz[1];
            _mapsp3[sat][ep]["SZ"] = dxyz[2];
            _mapsp3[sat][ep]["SC"] = dt;
        }
        else
        {
            return -1;
        }
        return 0;
    }

    // add velocity
    // ----------
    int gnss_all_prec::addvel(const std::string &sat, const base_time &ep, double xyzt[4], double dxyz[4])
    {
        if (_overwrite || _mapsp3[sat].find(ep) == _mapsp3[sat].end())
        {

            _mapsp3[sat][ep]["VX"] = xyzt[0];
            _mapsp3[sat][ep]["VY"] = xyzt[1];
            _mapsp3[sat][ep]["VZ"] = xyzt[2];
            _mapsp3[sat][ep]["VC"] = xyzt[3];

            _mapsp3[sat][ep]["SVX"] = dxyz[0];
            _mapsp3[sat][ep]["SVY"] = dxyz[1];
            _mapsp3[sat][ep]["SVZ"] = dxyz[2];
            _mapsp3[sat][ep]["SVC"] = dxyz[3];
        }
        else
        {
            return -1;
        }
        return 0;
    }

    int gnss_all_prec::add_delta_clk(const std::string &sat, const base_time &ep, const int &iod, const double &dt, const double &dot_dt, const double &dot_dot_dt)
    {
        if (_overwrite || _mapclk[sat].find(ep) == _mapclk[sat].end())
        {

            _mapclk[sat][ep]["IOD"] = iod;
            _mapclk[sat][ep]["DCLK"] = dt;
            _mapclk[sat][ep]["DOTCLK"] = dot_dt;
            _mapclk[sat][ep]["DOTDOTCLK"] = dot_dot_dt;
            _realtime = true;
        }
        else
        {
            return -1;
        }
        return 0;
    }

    int gnss_all_prec::get_pos_clk_correction(const std::string &sat, const base_time &t, double *xyz, double *vxyz, double &clk, double &dclk)
    {
        if (_get_delta_pos_vel(sat, t) < 0)
        {
            xyz[0] = xyz[1] = xyz[2] = 0.0;
        }
        if (_get_delta_clk(sat, t) < 0)
        {
            clk = 0.0;
        }

        gnss_model_poly poly;
        poly.interpolate(_CTCorr, _CCorr, t.diff(_clkref), clk, dclk);

        // the vxyz&dclk may be changed to another way, reserved temporary
        poly.interpolate(_PTCorr, _XCorr, t.diff(_ref), xyz[0], vxyz[0]);
        poly.interpolate(_PTCorr, _YCorr, t.diff(_ref), xyz[1], vxyz[1]);
        poly.interpolate(_PTCorr, _ZCorr, t.diff(_ref), xyz[2], vxyz[2]);
        return 1;
    }

    // added by zhShen
    int gnss_all_prec::get_pos_clk_correction(const std::string &sat, const base_time &t, const int& iod, double *xyz, double *vxyz, double &clk, double &dclk)
    {

        //return -1;
        base_time tRefOrb;
        base_time tRefClk;
        hwa_map_iv orbcorr;
        hwa_map_iv clkcorr;
        int f1 = _get_delta_pos_vel(sat, t, iod, tRefOrb, orbcorr);
        int f2 = _get_delta_clk(sat, t, iod, tRefClk, clkcorr);

        if (f1 < 0)
        {
            xyz[0] = xyz[1] = xyz[2] = 0.0;
        }
        if (f2 < 0)
        {
            clk = 0.0;
            dclk = 0.0;
        }
        if (f1 == -1 && f2 == -1)
        {
            return -1;
        }

        double tdifforb = t.diff(tRefOrb);
        if (_udorbInt)
            tdifforb -= 0.5 * _udorbInt;
        if (fabs(tdifforb) > 5 * 60)
        {
            return -1;
        }
        // get the position correction, no ratation
        xyz[0] = orbcorr["DX"] + orbcorr["DVX"] * tdifforb;
        xyz[1] = orbcorr["DY"] + orbcorr["DVY"] * tdifforb;
        xyz[2] = orbcorr["DZ"] + orbcorr["DVZ"] * tdifforb;

        // get the velocity correction, no ratation
        vxyz[0] = orbcorr["DVX"];
        vxyz[1] = orbcorr["DVY"];
        vxyz[2] = orbcorr["DVZ"];

        double tdiffclk = t.diff(tRefClk);
        if (_udclkInt)
            tdiffclk -= 0.5 * _udclkInt;
        if (fabs(tdiffclk) > 10)
        {
            return -1;
        }
        // get the clock correction
        clk = clkcorr["DCLK"] + clkcorr["DOTCLK"] * tdiffclk + clkcorr["DOTDOCLK"] * tdiffclk * tdiffclk;
        dclk = clkcorr["DOTCLK"] + clkcorr["DOTDOCLK"] * tdiffclk;

#ifdef DEBUG

        std::cout << t.str() << "  " << sat << "  " << tdifforb << "  " << tdiffclk << "  ";

        std::cout << std::fixed << std::setprecision(3) << xyz[0] << "  "
             << xyz[1] << "  "
             << xyz[2] << "  "
             << vxyz[0] << "  "
             << vxyz[1] << "  "
             << vxyz[2] << "  "
             << clk << "  "
             << dclk << "  " << std::endl;

#endif // DEBUG
        return 1;
    }

    int gnss_all_prec::get_clk_correction(const std::string &sat, const base_time &t, const int &iod, double &clk, double &dclk)
    {
        base_time tRefClk;
        hwa_map_iv clkcorr;
        int f = _get_delta_clk(sat, t, iod, tRefClk, clkcorr);
        if (f < 0)
        {
            clk = 0.0;
            dclk = 0.0;
        }
        if (f == -1)
        {
            return -1;
        }

        double tdiffclk = t.diff(tRefClk);
        if (_udclkInt)
            tdiffclk -= 0.5 * _udclkInt;
        if (fabs(tdiffclk) > 10)
        {
            return -1;
        }
        // get the clock correction
        clk = clkcorr["DCLK"] + clkcorr["DOTCLK"] * tdiffclk + clkcorr["DOTDOCLK"] * tdiffclk * tdiffclk;
        dclk = clkcorr["DOTCLK"] + clkcorr["DOTDOCLK"] * tdiffclk;
        return 1;
    }

    bool gnss_all_prec::corr_avali()
    {
        return (_mapsp3.size() > 0 && _mapclk.size() > 0);
    }

    bool gnss_all_prec::corr_avali(const base_time &now)
    {
        if (now < _tend || now.diff(_tend) < _udorbInt)
            return true;
        else
        {
            return false;
        }
    }

    //yjqin
    int gnss_all_prec::add_pos_vel(const std::string &sat, const base_time &ep, const Triple &xyz, const double &t, const Triple &dxyz, const double &dt, double xyzt[4], double var[4], const int &obs_num)
    {
        if (_overwrite || _mapsp3.at(sat).find(ep) == _mapsp3.at(sat).end())
        {

            _mapsp3.at(sat)[ep]["X"] = xyz[0];
            _mapsp3.at(sat)[ep]["Y"] = xyz[1];
            _mapsp3.at(sat)[ep]["Z"] = xyz[2];
            _mapsp3.at(sat)[ep]["C"] = t;

            _mapsp3.at(sat)[ep]["SX"] = dxyz[0];
            _mapsp3.at(sat)[ep]["SY"] = dxyz[1];
            _mapsp3.at(sat)[ep]["SZ"] = dxyz[2];
            _mapsp3.at(sat)[ep]["SC"] = dt;

            _mapsp3.at(sat)[ep]["VX"] = xyzt[0];
            _mapsp3.at(sat)[ep]["VY"] = xyzt[1];
            _mapsp3.at(sat)[ep]["VZ"] = xyzt[2];
            _mapsp3.at(sat)[ep]["VC"] = xyzt[3];

            _mapsp3.at(sat)[ep]["SVX"] = var[0];
            _mapsp3.at(sat)[ep]["SVY"] = var[1];
            _mapsp3.at(sat)[ep]["SVZ"] = var[2];
            _mapsp3.at(sat)[ep]["SVC"] = var[3];
            _mapsp3.at(sat)[ep]["OBS"] = obs_num;
        }
        else
        {
            return -1;
        }
        return 0;
    }
    // add clocks
    // ----------
    int gnss_all_prec::addclk(const std::string &sat, const base_time &ep, double clk[3], double dxyz[3])
    {
        if (_overwrite || _mapclk[sat].find(ep) == _mapclk[sat].end())
        {

            // change by ZhengHJ for slow
            hwa_map_iv data = {
                {"C0", clk[0]},
                {"C1", clk[1]},
                {"C2", clk[2]},
                {"SC0", dxyz[0]},
                {"SC1", dxyz[1]},
                {"SC2", dxyz[2]},
            };
            _mapclk[sat][ep] = data;

            if (sat.length() == 3)
            {
                _clk_type_list.insert(AS);
            }
            else
            {
                _clk_type_list.insert(AR);
            }
        }
        else
        {
            return 1;
        }
        return 0;
    }

    int gnss_all_prec::addclk_tri(const std::string &sat, const base_time &ep, double clk[3], double dxyz[3])
    {
        if (_overwrite || _mapclk[sat].find(ep) == _mapclk[sat].end())
        {

            // change by ZhengHJ for slow
            // add ifcb by xiongyun
            hwa_map_iv data = {
                {"C0", clk[0]},
                {"C1", clk[1]},
                {"C2", clk[2]},
                {"IFCB_F3", clk[3]},
                {"SC0", dxyz[0]},
                {"SC1", dxyz[1]},
                {"SC2", dxyz[2]},
            };
            _mapclk[sat][ep] = data;

            if (sat.length() == 3)
            {
                _clk_type_list.insert(AS);
            }
            else
            {
                _clk_type_list.insert(AR);
            }
        }
        else
        {
            return 1;
        }
        return 0;
    }

    // return number of epochs
    // ----------
    unsigned int gnss_all_prec::nepochs(const std::string &prn)
    {
        unsigned int tmp = 0;
        if (_mapsp3.find(prn) != _mapsp3.end())
            tmp = _mapsp3[prn].size();
        return tmp;
    }

    // return list of available satellites
    // ----------
    std::set<std::string> gnss_all_prec::satellites() const
    {

        std::set<std::string> all_sat = gnss_all_nav::satellites();
        hwa_map_ITIV::const_iterator itSP3 = _mapsp3.begin();
        while (itSP3 != _mapsp3.end())
        {
            all_sat.insert(itSP3->first);
            ++itSP3;
        }
        return all_sat;
    }

    // clean clk function
    // clean sp3 derived from gnav
    // ----------
    void gnss_all_prec::clean_outer(const base_time &beg, const base_time &end)
    {

        if (end < beg)
            return;

        // first clean navigation messages
        gnss_all_nav::clean_outer(beg, end);
        // prec ephemeris - loop over all satellites
        // -----------------------------------------
        std::map<std::string, hwa_map_tiv>::const_iterator itPRN = _mapsp3.begin();
        while (itPRN != _mapsp3.end())
        {
            std::string prn = itPRN->first;

            // find and CLEAN all data (epochs) out of the specified period !
            std::map<base_time, hwa_map_iv>::iterator itFirst = _mapsp3[prn].begin();
            std::map<base_time, hwa_map_iv>::iterator itLast = _mapsp3[prn].end();
            std::map<base_time, hwa_map_iv>::iterator itBeg = _mapsp3[prn].lower_bound(beg); // greater|equal
            std::map<base_time, hwa_map_iv>::iterator itEnd = _mapsp3[prn].upper_bound(end); // greater only!

            // remove before BEGIN request
            if (itBeg != itFirst)
            {

                // begin is last
                if (itBeg == itLast)
                {
                    itBeg--;
                    _mapsp3[prn].erase(itFirst, itLast);

                    // begin is not last
                }
                else
                {
                    _mapsp3[prn].erase(itFirst, itBeg);
                }
            }

            // remove after END request
            if (itEnd != itLast)
            {
                _mapsp3[prn].erase(itEnd, itLast);
            }
            ++itPRN;
        }

        // prec clocks - loop over all satellites
        // --------------------------------------
        itPRN = _mapclk.begin();
        while (itPRN != _mapclk.end())
        {
            std::string prn = itPRN->first;

            // find and CLEAN all data (epochs) out of the specified period !
            std::map<base_time, hwa_map_iv>::iterator itFirst = _mapclk[prn].begin();
            std::map<base_time, hwa_map_iv>::iterator itLast = _mapclk[prn].end();
            std::map<base_time, hwa_map_iv>::iterator itBeg = _mapclk[prn].lower_bound(beg); // greater|equal
            std::map<base_time, hwa_map_iv>::iterator itEnd = _mapclk[prn].upper_bound(end); // greater only!

            // remove before BEGIN request
            if (itBeg != itFirst)
            {

                // begin is last
                if (itBeg == itLast)
                {
                    itBeg--;

                    _mapclk[prn].erase(itFirst, itLast);

                    // begin is not last
                }
                else
                {
                    _mapclk[prn].erase(itFirst, itBeg);
                }
            }

            // remove after END request
            if (itEnd != itLast)
            { // && ++itEnd != itLast ){

                _mapclk[prn].erase(itEnd, itLast);
            }
            itPRN++;
        }
        return;
    }

    // return first epoch of sp3 position/clocks
    // ----------
    base_time gnss_all_prec::beg_data(const std::string &prn)
    {
        base_time tmp = LAST_TIME;
        if (!prn.empty())
        {
            if (_mapsp3.find(prn) != _mapsp3.end())
                tmp = _mapsp3[prn].begin()->first;
        }
        else
        {
            for (auto itSAT = _mapsp3.begin(); itSAT != _mapsp3.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapsp3[itSAT->first].begin()->first < tmp)
                        tmp = _mapsp3[itSAT->first].begin()->first;
                }
            }
        }
        return tmp;
    }

    // return last epoch of sp3 position/clocks
    // ----------
    base_time gnss_all_prec::end_data(const std::string &prn)
    {
        base_time tmp = FIRST_TIME;
        if (!prn.empty())
        {
            if (_mapsp3.find(prn) != _mapsp3.end())
                tmp = _mapsp3[prn].rbegin()->first;
        }
        else
        {
            for (auto itSAT = _mapsp3.begin(); itSAT != _mapsp3.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapsp3[itSAT->first].rbegin()->first > tmp)
                        tmp = _mapsp3[itSAT->first].rbegin()->first;
                }
            }
        }
        return tmp;
    }

    // return first epoch of rinex clocks
    // ----------
    base_time gnss_all_prec::beg_clk(const std::string &prn)
    {
        base_time tmp = LAST_TIME;
        if (!prn.empty())
        {
            if (_mapclk.find(prn) != _mapclk.end())
                tmp = _mapclk[prn].begin()->first;
        }
        else
        {
            for (auto itSAT = _mapclk.begin(); itSAT != _mapclk.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapclk[itSAT->first].begin()->first < tmp)
                        tmp = _mapclk[itSAT->first].begin()->first;
                }
            }
        }
        return tmp;
    }

    // return last epoch of rinex clocks
    // ----------
    base_time gnss_all_prec::end_clk(const std::string &prn)
    {
        base_time tmp = FIRST_TIME;
        if (!prn.empty())
        {
            if (_mapclk.find(prn) != _mapclk.end())
                tmp = _mapclk[prn].rbegin()->first;
        }
        else
        {
            for (auto itSAT = _mapclk.begin(); itSAT != _mapclk.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapclk[itSAT->first].rbegin()->first > tmp)
                        tmp = _mapclk[itSAT->first].rbegin()->first;
                }
            }
        }
        return tmp;
    }

    // return first epoch of polynomials
    // ----------
    base_time gnss_all_prec::beg_prec(const std::string &prn)
    {
        base_time tmp = LAST_TIME;
        if (!prn.empty())
        {
            if (_mapprec.find(prn) != _mapprec.end())
                tmp = _mapprec[prn].begin()->first;
        }
        else
        {
            for (auto itSAT = _mapprec.begin(); itSAT != _mapprec.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapprec[itSAT->first].begin()->first < tmp)
                        tmp = _mapprec[itSAT->first].begin()->first;
                }
            }
        }
        return tmp;
    }

    // return last epoch of polynomials
    // ----------
    base_time gnss_all_prec::end_prec(const std::string &prn)
    {
        base_time tmp = FIRST_TIME;
        if (!prn.empty())
        {
            if (_mapprec.find(prn) != _mapprec.end())
                tmp = _mapprec[prn].rbegin()->first;
        }
        else
        {
            for (auto itSAT = _mapprec.begin(); itSAT != _mapprec.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapprec[itSAT->first].rbegin()->first > tmp)
                        tmp = _mapprec[itSAT->first].rbegin()->first;
                }
            }
        }
        return tmp;
    }

    // Approximative position
    // ----------
    int gnss_all_prec::nav(const std::string &sat, const base_time &t, double xyz[3], double var[3], double vel[3], const bool &chk_mask)
    {

        int fitdat = 24; // fitting samples
        int fitdeg = 12; // fitting degree

        for (int i = 0; i < 3; i++)
        {
            xyz[i] = 0.0;
            if (var)
                var[i] = 0.0;
            if (vel)
                vel[i] = 0.0;
        }

        // alternative use of gnav
        if (_mapsp3[sat].size() == 0)
            return ((_clknav && gnss_all_nav::nav(sat, t, xyz, var, vel, chk_mask) >= 0) ? 1 : -1);
        base_time beg(_mapsp3[sat].begin()->first);
        base_time end(_mapsp3[sat].rbegin()->first);

        if (t < beg - 900 || t > end + 900)
        {
            return ((_clknav && gnss_all_nav::nav(sat, t, xyz, var, vel) >= 0) ? 1 : -1);
        }

        if (_poly_beg.find(sat) == _poly_beg.end())
            _poly_beg[sat] = LAST_TIME;
        if (_poly_end.find(sat) == _poly_end.end())
            _poly_end[sat] = FIRST_TIME;

        // use existing approximative estimates from cached polynomials
        if (!(t > _poly_end[sat] || t < _poly_beg[sat]))
        {
            _sec = _poly_end[sat] - _poly_beg[sat];
            _ref = _poly_beg[sat] + _sec / 2;

            _poly_x[sat].evaluate(t.diff(_ref) / _sec, 0, xyz[0]);
            _poly_y[sat].evaluate(t.diff(_ref) / _sec, 0, xyz[1]);
            _poly_z[sat].evaluate(t.diff(_ref) / _sec, 0, xyz[2]);

#ifdef DEBUG
            std::cout << " PRN: " << sat
                 << " req: " << t.str_ymdhms()
                 << " ref: " << _ref.str_ymdhms()
                 << " sec: " << _sec
                 << " beg: " << _poly_beg[sat].str_ymdhms()
                 << " end: " << _poly_end[sat].str_ymdhms() << std::endl;
#endif
            return 1;
        }

        // prepare approximative estimates
        _PT.clear();
        _X.clear();
        _Y.clear();
        _Z.clear();

        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapsp3[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapsp3[sat].end();
        std::map<base_time, hwa_map_iv>::iterator itReq = _mapsp3[sat].upper_bound(t);

        int dst = distance(itBeg, itEnd); // tab values [#]
        if (dst < fitdeg)
        {
            return ((_clknav && gnss_all_nav::nav(sat, t, xyz, var, vel, chk_mask) >= 0) ? 1 : -1);
        }
        if (dst < fitdat)
        {
            fitdat = dst;
        } // shorten window

        int sign = 1; // towards future
        double diffEnd = t - _poly_end[sat];
        double diffBeg = t - _poly_beg[sat];
        if (_poly_beg[sat] == LAST_TIME)
        {
            itReq = _mapsp3[sat].upper_bound(t);
            --itReq;
        } // towards future, initialize
        else if (diffEnd > 0 && diffEnd < +900 * fitdat)
        {
            sign = 1;
            itReq = _mapsp3[sat].lower_bound(_poly_end[sat]);
        } // towards future
        else if (diffBeg < 0 && diffBeg > -900 * fitdat)
        {
            sign = -1;
            itReq = _mapsp3[sat].lower_bound(_poly_beg[sat]);
        } // towards past

        if (sign > 0 && distance(itReq, itEnd) <= fitdat)
        {
            itReq = itEnd;
            advance(itReq, -fitdat - 1);
        } // towards future, but shift
        else if (sign < 0 && distance(itBeg, itReq) <= fitdat)
        {
            itReq = itBeg;
            advance(itReq, +fitdat);
        } // towards past,   but shift

        _poly_beg[sat] = itReq->first;
        advance(itReq, +fitdat);
        _poly_end[sat] = itReq->first;
        advance(itReq, -fitdat);

        _sec = _poly_end[sat] - _poly_beg[sat];
        _ref = _poly_beg[sat] + _sec / 2;

#ifdef DEBUG
        std::cout << " PRN: " << sat
             << " epo: " << t.str_ymdhms()
             << " req: " << itReq->first.str_ymdhms()
             << " ref: " << _ref.str_ymdhms()
             << " sec: " << _sec
             << " beg: " << _poly_beg[sat].str_ymdhms()
             << " end: " << _poly_end[sat].str_ymdhms() << std::setprecision(0)
             << " dat: " << fitdat << std::endl;
#endif

        while (_PT.size() < static_cast<unsigned int>(fitdat))
        {
            ++itReq;
            base_time tt = itReq->first;
            _PT.push_back(tt.diff(_ref) / _sec);
            _X.push_back(_mapsp3[sat][tt]["X"]);
            _Y.push_back(_mapsp3[sat][tt]["Y"]);
            _Z.push_back(_mapsp3[sat][tt]["Z"]);

#ifdef DEBUG
            std::cout << std::fixed << std::setprecision(3)
                 << " PRN " << sat << tt.str_ymdhms(" epo:") << " " << tt.diff(_ref) / _sec
                 << " " << _mapsp3[sat][tt]["X"]
                 << " " << _mapsp3[sat][tt]["Y"]
                 << " " << _mapsp3[sat][tt]["Z"]
                 << " " << _X.size() << " " << _PT.size() << std::endl;
#endif
        }

        _poly_x[sat].fitpolynom(_PT, _X, fitdeg, _sec, _ref);
        _poly_x[sat].evaluate(t.diff(_ref) / _sec, 0, xyz[0]);
        _poly_y[sat].fitpolynom(_PT, _Y, fitdeg, _sec, _ref);
        _poly_y[sat].evaluate(t.diff(_ref) / _sec, 0, xyz[1]);
        _poly_z[sat].fitpolynom(_PT, _Z, fitdeg, _sec, _ref);
        _poly_z[sat].evaluate(t.diff(_ref) / _sec, 0, xyz[2]);

        //  _poly_x[sat].polynomials( _PT, _X ); _poly_x[sat].evaluate( t.diff(_ref), 0, xyz[0] );
        //  _poly_y[sat].polynomials( _PT, _Y ); _poly_y[sat].evaluate( t.diff(_ref), 0, xyz[1] );
        //  _poly_z[sat].polynomials( _PT, _Z ); _poly_z[sat].evaluate( t.diff(_ref), 0, xyz[2] );

#ifdef DEBUG
        std::vector<double> v_xpoly = _poly_x[sat].polynomials();
        std::cout << " PRN: " << sat << std::fixed << std::setprecision(3)
             << " req: " << t.str_ymdhms()
             << " ref: " << _ref.str_ymdhms()
             << " beg: " << _poly_beg[sat].str_ymdhms()
             << " end: " << _poly_end[sat].str_ymdhms() << std::setprecision(0)
             << " " << t.diff(_ref) << " " << xyz[0] << "  " << xyz[1] << "  " << xyz[2] << "  " << v_xpoly.size() << std::endl;
#endif
        return 1;
    }

    std::set<std::string> gnss_all_prec::clk_objs()
    {
        std::set<std::string> allobj;
        for (auto iter : _mapclk)
        {
            allobj.insert(iter.first);
        }
        return allobj;
    }

    std::set<base_time> gnss_all_prec::clk_epochs()
    {
        std::set<base_time> alltime;
        for (auto iter : _mapclk)
        {
            for (auto time : iter.second)
            {
                alltime.insert(time.first);
            }
        }
        return alltime;
    }

    std::set<gnss_all_prec::clk_type> gnss_all_prec::get_clk_type() const
    {
        return _clk_type_list;
    }

    // lagrange_interpolate
    // -------------------
    int gnss_all_prec::lagrange_pos(const std::string &sat, const base_time &t, Triple &crd)
    {

        _T.clear();
        _PT.clear();
        _X.clear();
        _Y.clear();
        _Z.clear();
        _CT.clear();
        _C.clear();

        if (_mapsp3.find(sat) == _mapsp3.end())
            return -1;

        std::map<base_time, hwa_map_iv>::iterator itReq = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itReq == _mapsp3[sat].end())
            return -1;

        // add for nearest epoch
        std::map<base_time, hwa_map_iv>::iterator itReq_tmp = --(_mapsp3[sat].lower_bound(t));
        double sample = _intvm[sat]; //yjqin
        if (itReq_tmp != _mapsp3[sat].end())
        {
            if (abs(t.diff(itReq_tmp->first)) < abs(t.diff(itReq->first)))
                itReq = itReq_tmp;
        }
        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapsp3[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapsp3[sat].end();
        std::map<base_time, hwa_map_iv>::iterator it = itReq;

        if (itReq == itEnd)
        {
            //    std::cerr << "itReq = " << itReq->first.str("%Y-%m-%d %H:%M:%S " <<
            //         << "itEnd = " << itEnd->first.str("%Y-%m-%d %H:%M:%S " << std::endl;
            return -1;
        }

#ifdef DEBUG
        std::cerr << "EPH FOUND: " << sat << " " << itReq->first.str("%Y-%m-%d %H:%M:%S[%T]")
             << std::fixed
             << std::setprecision(3)
             << "  x= " << std::setw(13) << itReq->second["X"]
             << "  y= " << std::setw(13) << itReq->second["Y"]
             << "  z= " << std::setw(13) << itReq->second["Z"]
             << std::setprecision(6)
             << "  c= " << std::setw(13) << itReq->second["C"] * 1000000
             << std::endl;
#endif

        // DISTANCE() NEED TO BE A POSITIVE DIFFERENCE !!!
        int limit = static_cast<int>(_degree_sp3 / 2); // round (floor)

        bool flag_left = false;
        auto itleft = itReq;
        for (int i = 0; i <= limit; i++)
        {
            itleft--;
            if (itleft == _mapsp3[sat].end())
            {
                flag_left = true;
                break;
            }
        }
        bool flag_right = false;

        if (!flag_left)
        {
            auto itright = itReq;
            for (int i = 0; i < static_cast<int>(_degree_sp3 - limit); i++)
            {
                itright++;
                if (itright == _mapsp3[sat].end())
                {
                    flag_right = true;
                    break;
                }
            }
        }

        // too few data
        //if (distance(itBeg, itEnd) < static_cast<int>(_degree_sp3))
        if (_mapsp3[sat].size() < static_cast<unsigned int>(_degree_sp3))
        {
            return -1;

            // start from the first item
        }
        //else if (distance(itBeg, itReq) <= limit)
        else if (flag_left)
        {
            it = itBeg;

            // start from the last item
        }
        //else if (distance(itReq, itEnd) <= static_cast<int>(_degree_sp3 - limit))
        else if (flag_right)
        {
            it = itEnd;
            for (int i = 0; i <= static_cast<int>(_degree_sp3); i++)
                it--;

            // around requested item (standard case)
        }
        else
        {
            for (int i = 0; i < limit; i++)
                it--;
        }

        //Added by lewen
        if (it == itEnd)
        {
            return -1;
        }
        base_time t_beg = it->first;
        double epo_now = (t - t_beg) / sample + 1;
        if (epo_now > 9) //yjqin
        {
            return -1;
        }
        // std::vector for polynomial
        for (unsigned int i = 0; i < _degree_sp3; it++, i++)
        {

            if (it->second["X"] != UNDEFVAL_POS)
            {

                _T.push_back(it->first);
                _X.push_back(it->second["X"]);
                _Y.push_back(it->second["Y"]);
                _Z.push_back(it->second["Z"]);
                _C.push_back(it->second["C"]);

#ifdef DEBUG
                std::cout << "EPH:" << i << " " << sat
                     << std::fixed
                     << std::setprecision(0)
                     << "  r=" << std::setw(6) << tdiff
                     << "  t=" << it->first.str("%Y-%m-%d %H:%M:%S ")
                     << std::setprecision(3)
                     << "  x=" << std::setw(13) << _X[i] // it->second["X"]
                     << "  y=" << std::setw(13) << _Y[i] // it->second["Y"]
                     << "  z=" << std::setw(13) << _Z[i] // it->second["Z"]
                     << std::setprecision(6)
                     << "  c=" << std::setw(13) << _C[i] * 1000000 // it->second["C"]*10000000
                     << " " << _X.size() << " " << _degree_sp3 * MAXDIFF_EPH
                     << std::endl;
#endif
            }
        }

        if (_X.size() != _degree_sp3)
        {
            return -1;
        }

        std::vector<double> X;
        for (unsigned int i = 1; i <= _degree_sp3; i++)
        {
            X.push_back(i);
        }

        crd[0] = lagrange_interpolate(X, _X, epo_now);
        crd[1] = lagrange_interpolate(X, _Y, epo_now);
        crd[2] = lagrange_interpolate(X, _Z, epo_now);

        return 1;
    }

    int gnss_all_prec::lagrange_vel(const std::string &sat, const base_time &t, Triple &crd)
    {

        _T.clear();
        _PT.clear();
        _X.clear();
        _Y.clear();
        _Z.clear();
        _CT.clear();
        _C.clear();

        if (_mapsp3.find(sat) == _mapsp3.end())
            return -1;

        std::map<base_time, hwa_map_iv>::iterator itReq = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itReq == _mapsp3[sat].end())
            return -1;

        // add for nearest epoch
        std::map<base_time, hwa_map_iv>::iterator itReq_tmp = --(_mapsp3[sat].lower_bound(t));
        double sample = itReq->first.diff(itReq_tmp->first);
        if (abs(t.diff(itReq_tmp->first)) < abs(t.diff(itReq->first)))
            itReq = itReq_tmp;

        //_ref = itReq->first; // get the nearest epoch after t as reference

        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapsp3[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapsp3[sat].end();
        std::map<base_time, hwa_map_iv>::iterator it = itReq;

        if (itReq == itEnd)
        {
            //    std::cerr << "itReq = " << itReq->first.str("%Y-%m-%d %H:%M:%S " <<
            //         << "itEnd = " << itEnd->first.str("%Y-%m-%d %H:%M:%S " << std::endl;
            return -1;
        }

#ifdef DEBUG
        std::cerr << "EPH FOUND: " << sat << " " << itReq->first.str("%Y-%m-%d %H:%M:%S[%T]")
             << std::fixed
             << std::setprecision(3)
             << "  Vx= " << std::setw(13) << itReq->second["VX"]
             << "  Vy= " << std::setw(13) << itReq->second["VY"]
             << "  Vz= " << std::setw(13) << itReq->second["VZ"]
             << std::setprecision(6)
             << "  c= " << std::setw(13) << itReq->second["C"] * 1000000
             << std::endl;
#endif

        // DISTANCE() NEED TO BE A POSITIVE DIFFERENCE !!!
        int limit = static_cast<int>(_degree_sp3 / 2); // round (floor)

        // too few data
        if (distance(itBeg, itEnd) < static_cast<int>(_degree_sp3))
        {
            return -1;

            // start from the first item
        }
        else if (distance(itBeg, itReq) <= limit)
        {
            it = itBeg;

            // start from the last item
        }
        else if (distance(itReq, itEnd) <= static_cast<int>(_degree_sp3 - limit))
        {
            it = itEnd;
            for (int i = 0; i <= static_cast<int>(_degree_sp3); i++)
                it--;

            // around requested item (standard case)
        }
        else
        {
            for (int i = 0; i < limit; i++)
                it--;
        }

        //Added by lewen
        if (it == itEnd)
        {
            return -1;
        }
        base_time t_beg = it->first;
        double epo_now = (t - t_beg) / sample + 1;
        // std::vector for polynomial
        for (unsigned int i = 0; i < _degree_sp3; it++, i++)
        {

            if (it->second["X"] != UNDEFVAL_POS)
            {

                _T.push_back(it->first);
                _X.push_back(it->second["X"]);
                _Y.push_back(it->second["Y"]);
                _Z.push_back(it->second["Z"]);
                _C.push_back(it->second["C"]);

#ifdef DEBUG
                std::cout << "EPH:" << i << " " << sat
                     << std::fixed
                     << std::setprecision(0)
                     << "  r=" << std::setw(6) << tdiff
                     << "  t=" << it->first.str("%Y-%m-%d %H:%M:%S ")
                     << std::setprecision(3)
                     << "  x=" << std::setw(13) << _X[i] // it->second["X"]
                     << "  y=" << std::setw(13) << _Y[i] // it->second["Y"]
                     << "  z=" << std::setw(13) << _Z[i] // it->second["Z"]
                     << std::setprecision(6)
                     << "  c=" << std::setw(13) << _C[i] * 1000000 // it->second["C"]*10000000
                     << " " << _X.size() << " " << _degree_sp3 * MAXDIFF_EPH
                     << std::endl;
#endif
            }
        }

        if (_X.size() != _degree_sp3)
        {
            return -1;
        }

        std::vector<double> X;
        for (unsigned int i = 1; i <= _degree_sp3; i++)
        {
            X.push_back(i);
        }
        crd[0] = lagrange_interpolate(X, _X, epo_now, true);
        crd[1] = lagrange_interpolate(X, _Y, epo_now, true);
        crd[2] = lagrange_interpolate(X, _Z, epo_now, true);
        return 1;
    }
    // find gnss_data_eph element
    // ---------

    std::shared_ptr<gnss_data_eph> gnss_all_prec::_find(const std::string &sat, const base_time &t)
    {

        /*if (_mapsp3[sat].size() == 0)
            return _null;*/
        if (_mapsp3.find(sat) == _mapsp3.end())
            return _null; // std::make_shared<gnss_data_eph>();
        else if (_mapsp3[sat].size() == 0)
            return _null;
        // if not exists satellite not in cache
        hwa_map_EPHPREC::iterator it = _prec.find(sat);
        if (it == _prec.end())
        {
            if (_get_crddata(sat, t) < 0)
                return _null; // std::make_shared<gnss_data_eph>();
                              //    std::cout << " CACHE INIT ! : " << sat << " " << t.str("%Y-%m-%d %H:%M:%S") << std::endl;
        }

        // could not find the data at all --- SHOULDN'T OCCURE, SINCE _get_crddata already return !
        it = _prec.find(sat);
        if (it == _prec.end())
        {
            return _null; //std::make_shared<gnss_data_eph>();
        }

        double t_minus_ref = t - (it->second)->epoch();

        //  std::cout   << " CACHE        : " << sat << " " << t.str("%Y-%m-%d %H:%M:%S") << std::endl;

        // standard case: cache - satellite found and cache still valid!
        if (fabs((float)t_minus_ref) < (it->second)->interval() / _degree_sp3 && // more frequently updated cache
            (it->second)->valid(t)                                               // internal gephprec validity
        )
        {
            //    std::cout << " CACHE USED ! : " << sat << " " << t.str_ymdhms()
            //         << " " << t_minus_ref << " < " << (it->second)->interval()/_degree_sp3 << std::endl;
            return it->second;

            // specific case: cache - satellite is not within its standard validity (try to update cache)
        }
        else
        {

            base_time beg(_mapsp3[sat].begin()->first);
            base_time end(_mapsp3[sat].rbegin()->first);

            // update cache only if not close to the prec data boundaries
            if ((fabs(t.diff(beg)) > (it->second)->interval() / 2 &&
                 fabs(t.diff(end)) > (it->second)->interval() / 2) ||
                !(it->second)->valid(t))
            {
                /*       
      std::cout << " CACHE UPDATE : " << sat << " " << t.str_ymdhms()
//       << " beg: " << beg.str_ymdhms()
//       << " end: " << end.str_ymdhms()
       << "   " << fabs(t.diff(beg))
       << "   " << fabs(t.diff(end))
       << " int1: " << (it->second)->interval()/2
       << " int2: " << (it->second)->interval()/_degree_sp3
       << " val:" << (it->second)->valid(t)
       << " ref:" << t_minus_ref
       << std::endl;
*/
                if (_get_crddata(sat, t) < 0)
                    return _null; //std::make_shared<gnss_data_eph>();
                it = _prec.find(sat);

                //    }else{
                //      std::cout << " CACHE !! UPD : " << sat << " " << t.str("%Y-%m-%d %H:%M:%S") << std::endl;
            }
        }

        return it->second;
    }

    // fill PT,X,Y,Z std::vectors
    // ----------
    int gnss_all_prec::_get_crddata(const std::string &sat, const base_time &t)
    {

        _T.clear();
        _PT.clear();
        _X.clear();
        _Y.clear();
        _Z.clear();
        _CT.clear();
        _C.clear();

        if (_mapsp3.find(sat) == _mapsp3.end())
            return -1;

        std::map<base_time, hwa_map_iv>::iterator itReq = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itReq == _mapsp3[sat].end())
            return -1;

        // add for nearest epoch add by glfeng
        std::map<base_time, hwa_map_iv>::iterator itReq_tmp = --(_mapsp3[sat].lower_bound(t));

        // jdhuang : fix the problem
        if (itReq_tmp != std::end(_mapsp3[sat]))
        {
            if (abs(t.diff(itReq_tmp->first)) < abs(t.diff(itReq->first)))
                itReq = itReq_tmp;
        }

        _ref = itReq->first; // get the nearest epoch after t as reference

        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapsp3[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapsp3[sat].end();
        std::map<base_time, hwa_map_iv>::iterator it = itReq;

        if (itReq == itEnd)
        {
            //    std::cerr << "itReq = " << itReq->first.str("%Y-%m-%d %H:%M:%S " <<
            //         << "itEnd = " << itEnd->first.str("%Y-%m-%d %H:%M:%S " << std::endl;
            return -1;
        }

#ifdef DEBUG
        std::cerr << "EPH FOUND: " << sat << " " << itReq->first.str("%Y-%m-%d %H:%M:%S[%T]")
             << std::fixed
             << std::setprecision(3)
             << "  x= " << std::setw(13) << itReq->second["X"]
             << "  y= " << std::setw(13) << itReq->second["Y"]
             << "  z= " << std::setw(13) << itReq->second["Z"]
             << std::setprecision(6)
             << "  c= " << std::setw(13) << itReq->second["C"] * 1000000
             << std::endl;
#endif

        // DISTANCE() NEED TO BE A POSITIVE DIFFERENCE !!!
        int limit = static_cast<int>(_degree_sp3 / 2); // round (floor)

        // too few data
        if (distance(itBeg, itEnd) < static_cast<int>(_degree_sp3))
        {
            return -1;

            // =============================  modified by glfeng  <= change to < /// add return -1
            // start from the first item
        }
        else if (distance(itBeg, itReq) < limit)
        {
            it = itBeg;
            //return -1; // add by glfeng // jdhuang : fix
            // =============================  modified by glfeng  <= change to < /// add return -1
            // start from the last item
        }
        else if (distance(itReq, itEnd) < static_cast<int>(_degree_sp3 - limit))
        {
            it = itEnd;
            //return -1; // add by glfeng // jdhuang : fix
            for (int i = 0; i <= static_cast<int>(_degree_sp3); i++)
                it--;
            // =============================
            // around requested item (standard case)
        }
        else
        {
            for (int i = 0; i < limit; i++)
                it--;
        }

        //Added by lewen
        if (it == itEnd)
        {
            return -1;
        }

        // std::vector for polynomial
        for (unsigned int i = 0; i <= _degree_sp3; it++, i++)
        {
            double tdiff = it->first - _ref;

            // check maximum interval allowed between reference and sta/end epochs
            if (fabs(tdiff) > static_cast<double>(_degree_sp3 * MAXDIFF_EPH))
                continue;

            if (it->second["X"] != UNDEFVAL_POS)
            {

                _PT.push_back(tdiff);
                _T.push_back(it->first);
                _X.push_back(it->second["X"]);
                _Y.push_back(it->second["Y"]);
                _Z.push_back(it->second["Z"]);
                _CT.push_back(tdiff);
                _C.push_back(it->second["C"]);

#ifdef DEBUG
                std::cout << "EPH:" << i << " " << sat
                     << std::fixed
                     << std::setprecision(0)
                     << "  r=" << std::setw(6) << tdiff
                     << "  t=" << it->first.str("%Y-%m-%d %H:%M:%S ")
                     << std::setprecision(3)
                     << "  x=" << std::setw(13) << _X[i] // it->second["X"]
                     << "  y=" << std::setw(13) << _Y[i] // it->second["Y"]
                     << "  z=" << std::setw(13) << _Z[i] // it->second["Z"]
                     << std::setprecision(6)
                     << "  c=" << std::setw(13) << _C[i] * 1000000 // it->second["C"]*10000000
                     << " " << _X.size() << " " << _degree_sp3 * MAXDIFF_EPH
                     << std::endl;
#endif
            }
        }

        if (_X.size() != _degree_sp3 + 1)
        {
            return -1;
        }

        if (_prec.find(sat) != _prec.end())
        {
            _prec[sat]->degree(_degree_sp3);
            _prec[sat]->add(sat, _T, _X, _Y, _Z, _C);
        }
        else
        {
            std::shared_ptr<gnss_data_ephprec> tmp(new gnss_data_ephprec(_spdlog));
            tmp->spdlog(_spdlog);
            tmp->degree(_degree_sp3);
            tmp->add(sat, _T, _X, _Y, _Z, _C);
            _prec[sat] = tmp;
        }

        return 1;
    }

    // fill CT,C std::vectors
    // ----------
    int gnss_all_prec::_get_clkdata(const std::string &sat, const base_time &t)
    {

        _CT.clear();
        _C.clear();
        _ifcb_F3.clear();

        if (_mapclk.find(sat) == _mapclk.end())
            return -1;
        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapclk[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapclk[sat].end();
        std::map<base_time, hwa_map_iv>::iterator itReq = _mapclk[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itReq == _mapclk[sat].end())
            return -1; // too old products
        if (t < itBeg->first)
            return -1; // too new products

        _clkref = itReq->first; // get the nearest epoch after t as reference

        std::map<base_time, hwa_map_iv>::iterator it = itReq;
        //  std::map<base_time,hwa_map_iv>::iterator itPlus = itReq; itPlus++;
        //  std::map<base_time,hwa_map_iv>::iterator itMin  = itReq; if( itReq != itBeg ) itMin--;

        if (itReq == itEnd)
        {
            return -1;
        }

#ifdef DEBUG
        std::cout << "CLK FOUND: " << itReq->first.str_ymdhms(" ")
             << " c1= " << itReq->second["C0"]
             << " c2= " << itReq->second["C1"]
             << " c3= " << itReq->second["C2"]
             << std::endl;
#endif

        /*
  double dist = 0.0, dist1 = 0.0, dist2 = 0.0; // seconds
  if( itPlus != itEnd ) dist1 = fabs(itReq->first - itPlus->first );
  if( itMin  != itBeg ) dist2 = fabs(itReq->first -  itMin->first );

  if( dist1 != dist2 ){
     std::cout << sat <<  t.str(" %Y-%m-%d %H:%M:%S ")
          << " dist1 = " << dist1
          << " dist2 = " << dist2 
          << " itReq: "  << itReq->first.str(" %Y-%m-%d %H:%M:%S ")
          << " itBeg: "  << itBeg->first.str(" %Y-%m-%d %H:%M:%S ")
          << " itMin: "  << itMin->first.str(" %Y-%m-%d %H:%M:%S ")
          << std::endl;
    if( _log ) _log->comment(1,"gallprec",t.str_ymdhms(sat + " warning - no clocks distance clear for "));
    return -1;
  }else{
    dist = dist1;
  }
*/
        //  std::cerr << sat <<  t.str(" %Y-%m-%d %H:%M:%S  distance = ") << dist << std::endl;

        unsigned int degree_clk = 1; // MOZNA DAT 3 degree polinom kvuli 1Hz datum (ale cekovat ktery 3)

        //  if( dist > 30 ) degree_clk = 7; // use higher polynomial degree

        // DISTANCE() NEED TO BE A POSITIVE DIFFERENCE !!!
        int limit = static_cast<int>(degree_clk / 2); // round (floor)

        // too few data
        // disatance(beg,end) == size () distance is too low! ZHJ
        //if( distance(itBeg,itEnd) < static_cast<int>(degree_clk) ){

        //int distance_left = distance(itBeg, itReq);
        //int distance_right = _mapclk[sat].size() - distance_left;
        bool flag_left = false;
        auto itleft = itReq;
        for (int i = 0; i <= limit; i++)
        {
            itleft--;
            if (itleft == _mapclk[sat].end())
            {
                flag_left = true;
                break;
            }
        }
        bool flag_right = false;
        auto itright = itReq;
        for (int i = 0; i < static_cast<int>(degree_clk - limit); i++)
        {
            itright++;
            if (itright == _mapclk[sat].end())
            {
                flag_right = true;
                break;
            }
        }

        if (_mapclk[sat].size() < static_cast<unsigned int>(degree_clk))
        {
            return -1;

            // start from the first item
            //}else if( distance(itBeg,itReq) <= limit ){
            // change by ZHJ
        }
        else if (flag_left)
        {
            it = itBeg;

            // start from the last item
            //}else if( distance(itReq,itEnd) <= static_cast<int>(degree_clk - limit) ){
            // change by ZHJ
        }
        else if (flag_right)
        {
            it = itEnd;
            for (int i = 0; i <= static_cast<int>(degree_clk); i++)
                it--;

            // around requested item (standard case)
        }
        else
        {
            for (int i = 0; i <= limit; i++)
                it--;
        }

        // calculate
        if (it->second["C1"] < UNDEFVAL_CLK)
        {
            if (it->second["C2"] < UNDEFVAL_CLK)
            {
                // xiongyun:add for ifcb
                if (it->second.find("IFCB_F3") == it->second.end())
                {
                    //      double tdiff = it->first - _clkref;  // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                    double tdiff = it->first - t; // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                    _CT.push_back(tdiff);
                    _C.push_back(it->second["C0"] + it->second["C1"] * tdiff + it->second["C2"] * tdiff * tdiff);
                }
                else
                {
                    it++;
                    double tdiff = it->first - t; // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                    _CT.push_back(tdiff);
                    _C.push_back(it->second["C0"]);
                    _ifcb_F3.push_back(it->second["IFCB_F3"]); //add by xiongyun
                }
                //      std::cout << "POCITAM C0+C1+C2 " << it->second["C0"] << " " <<  it->second["C1"] << " " << tdiff << "\n";
                return 1;
            }
            else
            {
                //      double tdiff = it->first - _clkref;  // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                double tdiff = it->first - t; // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                _CT.push_back(tdiff);
                _C.push_back(it->second["C0"] + it->second["C1"] * tdiff);
                //      std::cout << "POCITAM C0+C1 " << it->second["C0"] << " " <<  it->second["C1"] << " " << tdiff << "\n";
                return 1;
            }

            // interpolate
        }
        else
        {

            // std::vector for polynomial
            for (unsigned int i = 0; i <= degree_clk; it++, i++)
            {
                double tdiff = it->first - _clkref;

                // check maximum interval allowed between reference and sta/end epochs
                if (fabs(tdiff) > static_cast<double>(degree_clk * MAXDIFF_CLK))
                {
                    continue;
                }

                if (it->second["C0"] != UNDEFVAL_CLK)
                {
                    _CT.push_back(tdiff);
                    _C.push_back(it->second["C0"]);

#ifdef DEBUG
                    std::cout << "CLK:" << i
                         << " " << _CT.size()
                         << " " << _C.size()
                         << std::fixed << std::setprecision(0)
                         << "  r=" << std::setw(6) << tdiff
                         << "  t=" << it->first.str("%Y-%m-%d %H:%M:%S ")
                         << scientific << std::setprecision(8)
                         << " c0=" << std::setw(14) << it->second["C0"]
                         << " c1=" << std::setw(14) << it->second["C1"]
                         << " c2=" << std::setw(14) << it->second["C2"]
                         << std::endl;
#endif
                }
            }

            if (_C.size() != degree_clk + 1)
            {

                //Use Liner Poly by ZHJ
                if (itReq != itBeg)
                {
                    _C.clear();
                    _CT.clear();
                    // Add First
                    auto itFirst = itReq;
                    itFirst--;
                    _C.push_back(itFirst->second["C0"]);
                    if (fabs(itFirst->first - _clkref) > static_cast<double>(degree_clk * MAXDIFF_CLK))
                    {
                        return -1;
                    }
                    _CT.push_back(itFirst->first - _clkref);

                    // Add Second
                    _C.push_back(itReq->second["C0"]);
                    _CT.push_back(0.0);

                    return 1;
                }

                return -1;
            }
        }

        return 1;
    }

    int gnss_all_prec::_get_delta_pos_vel(const std::string &sat, const base_time &t)
    {

        _TCorr.clear();
        _PTCorr.clear();
        _XCorr.clear();
        _YCorr.clear();
        _ZCorr.clear();
        _CTCorr.clear();
        _CCorr.clear();

        if (_mapsp3.find(sat) == _mapsp3.end())
            return -1;

        std::map<base_time, hwa_map_iv>::iterator itReq = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]

        // if itReq=end, so the itReq is end--.
        if (itReq == _mapsp3[sat].end())
        {
            itReq--;
        }

        _ref = itReq->first; // get the nearest epoch after t as reference

        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapsp3[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapsp3[sat].end();
        std::map<base_time, hwa_map_iv>::iterator it = itReq;

        if (itReq == itEnd)
        {
            //    std::cerr << "itReq = " << itReq->first.str("%Y-%m-%d %H:%M:%S " <<
            //         << "itEnd = " << itEnd->first.str("%Y-%m-%d %H:%M:%S " << std::endl;
            return -1;
        }

#ifdef DEBUG
        std::cerr << "EPH FOUND: " << sat << " " << itReq->first.str("%Y-%m-%d %H:%M:%S[%T]")
             << std::fixed
             << std::setprecision(3)
             << "  x= " << std::setw(13) << itReq->second["DX"]
             << "  y= " << std::setw(13) << itReq->second["DY"]
             << "  z= " << std::setw(13) << itReq->second["DZ"]
             << std::setprecision(6)
             << "  c= " << std::setw(13) << itReq->second["DCLK"] * 1000000
             << std::endl;
#endif

        // DISTANCE() NEED TO BE A POSITIVE DIFFERENCE !!!
        int limit = static_cast<int>(_degree_sp3 / 2); // round (floor)

        // too few data
        if (distance(itBeg, itEnd) < static_cast<int>(_degree_sp3))
        {
            return -1;

            // =============================  modified by glfeng  <= change to < /// add return -1
            // start from the first item
        }
        else if (distance(itBeg, itReq) < limit)
        {
            it = itBeg;
            return -1; // add by glfeng
                       // =============================  modified by glfeng  <= change to < /// add return -1
                       // start from the last item
        }
        //else if (distance(itReq, itEnd) < static_cast<int>(_degree_sp3 - limit)) {
        //    it = itEnd;
        //    return -1; // add by glfeng
        //    for (int i = 0; i <= static_cast<int>(_degree_sp3); i++) it--;
        //    // =============================
        //    // around requested item (standard case)
        //}
        else
        {
            for (int i = 0; i < limit; i++)
                it--;
        }

        // std::vector for polynomial
        for (unsigned int i = 0; i <= _degree_sp3; it++, i++)
        {
            double tdiff = it->first - _ref;

            // check maximum interval allowed between reference and sta/end epochs
            if (fabs(tdiff) > static_cast<double>(_degree_sp3 * MAXDIFF_EPH))
                continue;

            if (it->second["DX"] != UNDEFVAL_POS)
            {

                _PTCorr.push_back(tdiff);
                _TCorr.push_back(it->first);
                _XCorr.push_back(it->second["DX"]);
                _YCorr.push_back(it->second["DY"]);
                _ZCorr.push_back(it->second["DZ"]);
                _CTCorr.push_back(tdiff);
                _CCorr.push_back(it->second["DCLK"]);

#ifdef DEBUG
                std::cout << "EPH:" << i << " " << sat
                     << std::fixed
                     << std::setprecision(0)
                     << "  r=" << std::setw(6) << tdiff
                     << "  t=" << it->first.str("%Y-%m-%d %H:%M:%S ")
                     << std::setprecision(3)
                     << "  x=" << std::setw(13) << _XCorr[i] // it->second["DX"]
                     << "  y=" << std::setw(13) << _YCorr[i] // it->second["DY"]
                     << "  z=" << std::setw(13) << _ZCorr[i] // it->second["DZ"]
                     << std::setprecision(6)
                     << "  c=" << std::setw(13) << _CCorr[i] * 1000000 // it->second["C"]*10000000
                     << " " << _X.size() << " " << _degree_sp3 * MAXDIFF_EPH
                     << std::endl;
#endif
            }
        }

        if (_XCorr.size() != _degree_sp3 + 1)
        {
            return -1;
        }

        if (_prec.find(sat) != _prec.end())
        {
            _prec[sat]->degree(_degree_sp3);
            _prec[sat]->add(sat, _TCorr, _XCorr, _YCorr, _ZCorr, _CCorr);
        }
        else
        {
            std::shared_ptr<gnss_data_ephprec> tmp(new gnss_data_ephprec(_spdlog));

            tmp->spdlog(_spdlog);
            tmp->degree(_degree_sp3);
            tmp->add(sat, _TCorr, _XCorr, _YCorr, _ZCorr, _CCorr);
            _prec[sat] = tmp;
        }

        return 1;
    }

    int gnss_all_prec::_get_delta_pos_vel(const std::string &sat, const base_time &t, int iod, base_time &tRef, hwa_map_iv &orbcorr)
    {

        /**/
        if (_mapsp3.find(sat) == _mapsp3.end() || _mapsp3[sat].size() == 0)
        {
            /**/ return -1;
        }

        std::map<base_time, hwa_map_iv>::iterator itLast = _mapsp3[sat].lower_bound(t); // 1st equal|greater [than t]

        std::map<base_time, hwa_map_iv>::iterator itPrev = --(_mapsp3[sat].lower_bound(t)); // 1st equal|greater [than t]

        if (itLast != _mapsp3[sat].end() && int(itLast->second["IOD"]) == iod)
        {
            tRef = itLast->first;
            orbcorr = itLast->second;
            /**/ return 1;
        }
        else if (itPrev != _mapsp3[sat].end() && int(itPrev->second["IOD"]) == iod)
        {
            tRef = itPrev->first;
            orbcorr = itPrev->second;
            /**/ return 1;
        }
        else
        {
            /* */ return -1;
        }
    }

    int gnss_all_prec::_get_delta_clk(const std::string &sat, const base_time &t)
    {

        _CTCorr.clear();
        _CCorr.clear();

        if (_mapclk.find(sat) == _mapclk.end())
            return -1;
        std::map<base_time, hwa_map_iv>::iterator itBeg = _mapclk[sat].begin();
        std::map<base_time, hwa_map_iv>::iterator itEnd = _mapclk[sat].end();
        std::map<base_time, hwa_map_iv>::iterator itReq = _mapclk[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itReq == _mapclk[sat].end())
            return -1; // too old products
        if (t < itBeg->first)
            return -1; // too new products

        _clkref = itReq->first; // get the nearest epoch after t as reference

        std::map<base_time, hwa_map_iv>::iterator it = itReq;
        //  std::map<base_time,hwa_map_iv>::iterator itPlus = itReq; itPlus++;
        //  std::map<base_time,hwa_map_iv>::iterator itMin  = itReq; if( itReq != itBeg ) itMin--;

        if (itReq == itEnd)
        {
            return -1;
        }

#ifdef DEBUG
        std::cout << "CLK FOUND: " << itReq->first.str_ymdhms(" ")
             << " c1= " << itReq->second["DCLK"]
             << " c2= " << itReq->second["DOTCLK"]
             << " c3= " << itReq->second["DOTDOTCLK"]
             << std::endl;
#endif

        unsigned int degree_clk = 1; // MOZNA DAT 3 degree polinom kvuli 1Hz datum (ale cekovat ktery 3)

        //  if( dist > 30 ) degree_clk = 7; // use higher polynomial degree

        // DISTANCE() NEED TO BE A POSITIVE DIFFERENCE !!!
        int limit = static_cast<int>(degree_clk / 2); // round (floor)

        // too few data
        // disatance(beg,end) == size () distance is too low! ZHJ
        //if( distance(itBeg,itEnd) < static_cast<int>(degree_clk) ){

        //int distance_left = distance(itBeg, itReq);
        //int distance_right = _mapclk[sat].size() - distance_left;
        bool flag_left = false;
        auto itleft = itReq;
        for (int i = 0; i <= limit; i++)
        {
            itleft--;
            if (itleft == _mapclk[sat].end())
            {
                flag_left = true;
                break;
            }
        }
        bool flag_right = false;
        auto itright = itReq;
        for (int i = 0; i < static_cast<int>(degree_clk - limit); i++)
        {
            itright++;
            if (itright == _mapclk[sat].end())
            {
                flag_right = true;
                break;
            }
        }

        if (_mapclk[sat].size() < static_cast<unsigned int>(degree_clk))
        {
            return -1;
        }
        else if (flag_left)
        {
            it = itBeg;
        }
        else if (flag_right)
        {
            it = itEnd;
            for (int i = 0; i <= static_cast<int>(degree_clk); i++)
                it--;

            // around requested item (standard case)
        }
        else
        {
            for (int i = 0; i <= limit; i++)
                it--;
        }

        // calculate
        if (it->second["DOTCLK"] < UNDEFVAL_CLK)
        {
            if (it->second["DOTDOTCLK"] < UNDEFVAL_CLK)
            {
                //      double tdiff = it->first - _clkref;  // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                double tdiff = it->first - t; // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                _CTCorr.push_back(tdiff);
                _CCorr.push_back(it->second["DCLK"] + it->second["DOTCLK"] * tdiff + it->second["DOTDOTCLK"] * tdiff * tdiff);
                //      std::cout << "POCITAM C0+C1+C2 " << it->second["C0"] << " " <<  it->second["C1"] << " " << tdiff << "\n";
                return 1;
            }
            else
            {
                //      double tdiff = it->first - _clkref;  // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                double tdiff = it->first - t; // seconds !!!!!!!!!!!!!!!!!!!! instead of _clkref should be ReqT = t !!
                _CTCorr.push_back(tdiff);
                _CCorr.push_back(it->second["DCLK"] + it->second["DOTCLK"] * tdiff);
                //      std::cout << "POCITAM C0+C1 " << it->second["C0"] << " " <<  it->second["C1"] << " " << tdiff << "\n";
                return 1;
            }

            // interpolate
        }
        else
        {

            // std::vector for polynomial
            for (unsigned int i = 0; i <= degree_clk; it++, i++)
            {
                double tdiff = it->first - _clkref;

                // check maximum interval allowed between reference and sta/end epochs
                if (fabs(tdiff) > static_cast<double>(degree_clk * MAXDIFF_CLK))
                {
                    continue;
                }

                if (it->second["DCLK"] != UNDEFVAL_CLK)
                {
                    _CTCorr.push_back(tdiff);
                    _CCorr.push_back(it->second["DCLK"]);

#ifdef DEBUG
                    std::cout << "CLK:" << i
                         << " " << _CTCorr.size()
                         << " " << _CCorr.size()
                         << std::fixed << std::setprecision(0)
                         << "  r=" << std::setw(6) << tdiff
                         << "  t=" << it->first.str("%Y-%m-%d %H:%M:%S ")
                         << scientific << std::setprecision(8)
                         << " c0=" << std::setw(14) << it->second["DCLK"]
                         << " c1=" << std::setw(14) << it->second["DOTCLK"]
                         << " c2=" << std::setw(14) << it->second["DOTDOTCLK"]
                         << std::endl;
#endif
                }
            }

            if (_CCorr.size() != degree_clk + 1)
            {
                return -1;
            }
        }

        return 1;
    }

    int gnss_all_prec::_get_delta_clk(const std::string &sat, const base_time &t, int iod, base_time &tRef, hwa_map_iv &clkcorr)
    {

        //

        if (_mapclk.find(sat) == _mapclk.end() || _mapclk[sat].size() == 0)
        {
            /**/ return -1;
        }
        std::map<base_time, hwa_map_iv>::iterator itLast = _mapclk[sat].lower_bound(t); // 1st equal|greater [than t]

        std::map<base_time, hwa_map_iv>::iterator itPrev = --_mapclk[sat].lower_bound(t); // 1st equal|greater [than t]

        if (itLast != _mapclk[sat].end() && int(itLast->second["IOD"]) == iod)
        {
            tRef = itLast->first;
            clkcorr = itLast->second;
            /**/ return 1;
        }
        else if (itPrev != _mapclk[sat].end() && int(itPrev->second["IOD"]) == iod)
        {
            tRef = itPrev->first;
            clkcorr = itPrev->second;
            /**/ return 1;
        }
        else
        {
            /**/ return -1;
        }
    }

} // namespace
