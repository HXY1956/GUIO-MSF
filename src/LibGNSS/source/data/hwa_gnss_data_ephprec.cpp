#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include "hwa_gnss_data_ephprec.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"

using namespace hwa_base;

namespace hwa_gnss
{

    // constructor
    // ----------
    gnss_data_ephprec::gnss_data_ephprec()
        : gnss_data_eph(),
          _degree(0),
          _poly_x(),
          _poly_y(),
          _poly_z(),
          _poly_c()
    {
        id_type(base_data::EPHPREC);
        id_group(base_data::GRP_EPHEM);
    }

    gnss_data_ephprec::gnss_data_ephprec(base_log spdlog)
        : gnss_data_eph(spdlog),
          _degree(0),
          _poly_x(),
          _poly_y(),
          _poly_z(),
          _poly_c()
    {
        id_type(base_data::EPHPREC);
        id_group(base_data::GRP_EPHEM);
    }
    // destructor
    // ----------
    gnss_data_ephprec::~gnss_data_ephprec()
    {
    }

    // add data
    // ---------
    int gnss_data_ephprec::add(std::string sat, std::vector<base_time> t,
                        const std::vector<double> &x, const std::vector<double> &y,
                        const std::vector<double> &z, const std::vector<double> &c)
    {
        // check dimensions
        if (ndata() != t.size() || ndata() != x.size() ||
            ndata() != y.size() || ndata() != z.size() ||
            ndata() != c.size() || sat.empty())
        {
            return -1;
        }

        _clear();
        _sat = sat;
        _epoch = t[0] + t[_degree].diff(t[0]) / 2; // reference epoch always in the mid of interval!!!

        _xcrd = x; //  _xcrd.assign(x[0],x[_degree]);
        _ycrd = y; //  _ycrd.assign(y[0],y[_degree]);
        _zcrd = z; //  _zcrd.assign(z[0],z[_degree]);
        _clkc = c; //  _clkc.assign(c[0],c[_degree]);

        for (unsigned int i = 0; i < ndata(); ++i)
        {
            _dt.push_back(t[i] - _epoch);

#ifdef DEBUG
            std::cout << _sat
                 << " t = "
                 << std::fixed
                 << setprecision(3)
                 << setw(14) << _epoch.str("%Y-%m-%d %H:%M:%S")
                 << setw(14) << _dt[i]
                 << setw(14) << _xcrd[i]
                 << setw(14) << _ycrd[i]
                 << setw(14) << _zcrd[i]
                 << setprecision(6)
                 << setw(14) << _clkc[i] * 1000000
                 << std::endl;
#endif
        }

        _poly_x.polynomials(_dt, _xcrd);
        _poly_y.polynomials(_dt, _ycrd);
        _poly_z.polynomials(_dt, _zcrd);
        _poly_c.polynomials(_dt, _clkc);
        return 0;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // ----------
    int gnss_data_ephprec::pos(const base_time &t, double xyz[], double var[], double vel[], bool chk_health)
    {
        xyz[0] = xyz[1] = xyz[2] = 0.0;
        if (var)
            var[0] = var[1] = var[2] = 0.0;
        if (vel)
            vel[0] = vel[1] = vel[2] = 0.0;

        double tdiff = t - _epoch;

        if (!_valid_crd() || sat().empty() ||
            fabs(_epoch - t) > (fabs(_poly_x.xref() - _poly_x.span() / 2) + 0.25))
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "no ephemeris [" + _sat + t.str("] %Y-%m-%d %H:%M:%S") + " epoch: " + _epoch.str("%Y-%m-%d %H:%M:%S") + " tdiff: " + base_type_conv::dbl2str(tdiff) + " xref: " + base_type_conv::dbl2str(_poly_x.xref()) + " span: " + base_type_conv::dbl2str(_poly_x.span()));
            return -1;
        }

        _poly_x.evaluate(tdiff, 0, xyz[0]); // base_earth-std::fixed coordinates [m]
        _poly_y.evaluate(tdiff, 0, xyz[1]);
        _poly_z.evaluate(tdiff, 0, xyz[2]);

        if (vel)
        {
            _poly_x.evaluate(tdiff, 1, vel[0]);
            _poly_y.evaluate(tdiff, 1, vel[1]);
            _poly_z.evaluate(tdiff, 1, vel[2]);
        }
        return 1;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // NOT CORRECTED FOR 2nd relativistic effect !!!
    // ----------
    int gnss_data_ephprec::clk(const base_time &t, double *clk, double *var, double *dclk, bool chk_health)
    {
        *clk = 0.0;
        if (var)
            *var = 0.0;
        if (dclk)
            *dclk = 0.0;

        if (!_valid_clk() || sat().empty())
        {
            return -1;
        } // check if valid

        double tdiff = t - _epoch;

        if (abs(tdiff) > (abs(_poly_c.xref() - _poly_c.span() / 2) + 0.25))
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "no clock [" + _sat + t.str("] %Y-%m-%d %H:%M:%S"));
            return -1;
        }

        _poly_c.evaluate(tdiff, 0, *clk);

        if (var)
        {
        } // not implemented

        if (dclk)
        {
            _poly_c.evaluate(tdiff, 1, *dclk);
        }
        return 1;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // ----------
    int gnss_data_ephprec::pos_int(const base_time &t, double xyz[], double var[], double vel[])
    {
        xyz[0] = xyz[1] = xyz[2] = 0.0;
        if (var)
            var[0] = var[1] = var[2] = 0.0;
        if (vel)
            vel[0] = vel[1] = vel[2] = 0.0;

        if (!_valid_crd() || sat().empty())
        {
            return -1;
        } // check if valid

        double tdiff = t - _epoch;

        if (abs(tdiff) > (abs(_poly_x.xref() - _poly_x.span() / 2 + 0.25)))
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "no ephemeris [" + _sat + t.str("] %Y-%m-%d %H:%M:%S"));
            return -1;
        }

        for (unsigned int i = 0; i < ndata(); ++i)
        {

#ifdef DEBUG
            std::cout << _sat << t.str(" %Y-%m-%d %H:%M:%S[%T] ")
                 << " t_int = "
                 << tdiff
                 << std::fixed
                 << setprecision(3)
                 << setw(14) << _dt[i]
                 << setw(14) << _xcrd[i]
                 << setw(14) << _ycrd[i]
                 << setw(14) << _zcrd[i]
                 << setprecision(6)
                 << setw(14) << _clkc[i] * 1000000
                 << std::endl;
#endif
        }

        gnss_model_poly poly;
        if (var)
        {
            poly.interpolate(_dt, _xcrd, tdiff, xyz[0], var[0]);
            poly.interpolate(_dt, _ycrd, tdiff, xyz[1], var[1]);
            poly.interpolate(_dt, _zcrd, tdiff, xyz[2], var[2]);
        }
        else
        {
            double dummy[3];
            poly.interpolate(_dt, _xcrd, tdiff, xyz[0], dummy[0]);
            poly.interpolate(_dt, _ycrd, tdiff, xyz[1], dummy[1]);
            poly.interpolate(_dt, _zcrd, tdiff, xyz[2], dummy[2]);
        }

        if (vel)
        {
        } // not implemented
        return 1;
    }

    // t .. GPS time of transmission,
    // i.e. GPS time corrected for transit time (range/speed of light)
    // NOT CORRECTED FOR 2nd relativistic effect !!!
    // ----------
    int gnss_data_ephprec::clk_int(const base_time &t, double *clk, double *var, double *dclk)
    {
        *clk = 0.0;
        if (var)
            *var = 0.0;
        if (dclk)
            *dclk = 0.0;

        if (!_valid_clk() || sat().empty())
        {
            return -1;
        } // check if valid

        double tdiff = t - _epoch;

        if (abs(tdiff) > (abs(_poly_c.xref() - _poly_c.span() / 2) + 0.25))
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "no ephemeris [" + _sat + t.str("] %Y-%m-%d %H:%M:%S"));
            return -1;
        }

        gnss_model_poly poly;

        if (var)
        {
            poly.interpolate(_dt, _clkc, tdiff, *clk, *dclk);
        }
        else
        {
            double dummy;
            poly.interpolate(_dt, _clkc, tdiff, *clk, dummy);
        }

        if (dclk)
        {
        } // not yet implemented
        return 1;
    }

    // validity
    // ----------
    bool gnss_data_ephprec::valid(const base_time &t) const
    {
        if (_valid_crd() && !sat().empty() &&
            fabs(_epoch - t) < fabs(_poly_x.xref() - _poly_x.span() / 2) + 0.25) // add 0.25s!
        {
            //    std::cout << " valid : " << sat() << " " << t.str("%Y-%m-%d %H:%M:%S") << std::endl;
            return true;
        }

        //  std::cout << " ! not valid : " << sat() << " " << t.str("%Y-%m-%d %H:%M:%S") << std::endl;
        return false;
    }

    // clean data
    // ----------
    void gnss_data_ephprec::_clear()
    {
        _epoch = FIRST_TIME;
        _sat.clear();

        _dt.clear();

        _xcrd.clear();
        _ycrd.clear();
        _zcrd.clear();
        _xcrd.clear();

        _poly_x.reset();
        _poly_y.reset();
        _poly_z.reset();
        _poly_c.reset();
    }

    // validity ?
    // ----------
    bool gnss_data_ephprec::_valid_crd() const
    {
        if (gnss_data_eph::_valid() &&
            _poly_x.valid() &&
            _poly_y.valid() &&
            _poly_z.valid())
            return true;

        return false;
    }

    // validity ?
    // ----------
    bool gnss_data_ephprec::_valid_clk() const
    {

        bool undef = false;
        for (unsigned int i = 0; i < _clkc.size(); i++)
        {
            if (_clkc[i] >= 999999)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "clk_int undefined ephemeris [" + _sat + _epoch.str("] %Y-%m-%d %H:%M:%S"));
                undef = true;
            }
        }

        if (gnss_data_eph::_valid() &&
            _poly_c.valid() &&
            !undef)
            return true;

        return false;
    }

} // namespace
