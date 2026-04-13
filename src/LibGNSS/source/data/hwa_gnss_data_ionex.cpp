#include "hwa_gnss_data_ionex.h"
#include "hwa_base_globaltrans.h"

using namespace hwa_base;

namespace hwa_gnss
{
    gnss_data_ionex_head::gnss_data_ionex_head()
    {
    }

    gnss_data_ionex_head::~gnss_data_ionex_head()
    {
    }

    void gnss_data_ionex_head::operator=(const gnss_data_ionex_head &Other)
    {
        this->beg = Other.beg;
        this->end = Other.end;
        this->interval = Other.interval;
        this->exponent = Other.exponent;
        this->base_radius = Other.base_radius;
        this->lats = Other.lats;
        this->lons = Other.lons;
        this->hgts = Other.hgts;
        this->nlon = Other.nlon;
        this->nlat = Other.nlat;
        this->nhgt = Other.nhgt;
        this->ion_mapfunc = Other.ion_mapfunc;
        this->map_dimension = Other.map_dimension;
        this->p1p2_dcb = Other.p1p2_dcb;
    }

    gnss_data_ionex::gnss_data_ionex() : base_data()
    {
        id_type(base_data::IONEX);
    }

    gnss_data_ionex::gnss_data_ionex(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::IONEX);
    }

    gnss_data_ionex::~gnss_data_ionex()
    {
    }

    void gnss_data_ionex::add_head(gnss_data_ionex_head ionex_hd)
    {
        _ionex_hd = ionex_hd;
    }

    bool gnss_data_ionex::getSTEC(const base_time &time, gnss_data_sats &satdata, Triple &site_ell, double &value, double &rms)
    {
        value = 0.0;
        rms = 0.0;
        // Get grid data adjacent in time
        auto it2 = _map_ionex_data.lower_bound(time); // greater|equal  (can be still end())
        auto it1 = it2;
        if (it2 == _map_ionex_data.begin()) // beg
        {
            if (abs(it2->first.diff(time)) > _ionex_hd.interval * 2)
                return false;
        }
        else if (it2 == _map_ionex_data.end()) // end
        {
            if (abs(it2->first.diff(time)) > _ionex_hd.interval * 2)
                return false;
            it2--;
            it1--;
        }
        else
        {
            it1--;
        }

        double base_radius = _ionex_hd.base_radius;
        // loop ionosphere layer according altitude
        for (int ihgt = 0; ihgt < _ionex_hd.nhgt; ihgt++)
        {
            /* ionospheric pierce point position */
            Triple ipp_ell;
            double iono_hgt = std::get<0>(_ionex_hd.hgts) + std::get<2>(_ionex_hd.hgts) * ihgt;
            ell2ipp(&satdata, site_ell, base_radius, iono_hgt, ipp_ell);

            if (std::get<0>(_ionex_hd.lons) < 0.0)
            {
                if (ipp_ell[1] > hwa_pi)
                    ipp_ell[1] -= 2.0 * hwa_pi;
                else if (ipp_ell[1] < -hwa_pi)
                    ipp_ell[1] += 2.0 * hwa_pi;
            }

            /* std::map function factor */
            double mf = 1 / sqrt(1.0 - pow(base_radius / (base_radius + iono_hgt) * sin(hwa_pi / 2.0 - satdata.ele()), 2));

            /* interpolate STEC by grid data and std::map function factor */

            /* first bilinear interpolation/ nearest-neighbour extrapolation by lat&lon */
            double it1_val, it2_val, it1_rms, it2_rms; // Interpolated result

            bool valid1 = this->_interpolate_ion(it1->first, ipp_ell, ihgt + 1, it1_val, it1_rms); // VTEC, unit TECU
            bool valid2 = this->_interpolate_ion(it2->first, ipp_ell, ihgt + 1, it2_val, it2_rms); // VTEC, unit TECU

            /*then interpolation by time*/
            if (valid1 && valid2)
            {
                double dt = time.diff(it1->first) / _ionex_hd.interval;
                value += (it1_val * (1 - dt) + dt * it2_val) * mf;
                rms += SQR(it1_rms * (1 - dt) + dt * it2_rms) * SQR(mf);
            }
            else if (valid1)
            {
                value += it1_val * mf;
                rms += SQR(it1_rms) * SQR(mf);
            }
            else if (valid2)
            {
                value += it2_val * mf;
                rms += SQR(it2_rms) * SQR(mf);
            }
        }

        if (double_eq(value, 0.0) || double_eq(rms, 0.0))
            return false;

        return true;
    }

    void gnss_data_ionex::add_data(const base_time &time, std::string type, int ilat, int ilon, int ihgt, double v)
    {
        if (type == "TEC_MAP")
        {
            _map_ionex_data[time].tec_val[std::make_tuple(ilat, ilon, ihgt)] = v;
        }
        else if (type == "RMS_MAP")
        {
            _map_ionex_data[time].tec_rms[std::make_tuple(ilat, ilon, ihgt)] = v;
        }
    }

    bool gnss_data_ionex::_interpolate_ion(const base_time &time, const Triple &ipp_ell, const int &ihgt, double &value, double &rms)
    {
        double val11, val12, val21, val22; // origin values
        double rms11, rms12, rms21, rms22; // origin rms
        value = 0.0;                       // Interpolated result
        rms = 0.0;                         // Interpolated rms

        double dbl_lat = (ipp_ell[0] * R2D - std::get<0>(_ionex_hd.lats)) / std::get<2>(_ionex_hd.lats) + 1;
        double dbl_lon = (ipp_ell[1] * R2D - std::get<0>(_ionex_hd.lons)) / std::get<2>(_ionex_hd.lons) + 1;

        int ilat_0, ilat_1, ilon_0, ilon_1;
        ilat_0 = floor(dbl_lat);
        ilat_1 = ilat_0 + 1;
        ilon_0 = floor(dbl_lon);
        ilon_1 = ilon_0 + 1;

        val11 = _map_ionex_data[time].tec_val[std::make_tuple(ilat_0, ilon_0, ihgt)];
        val12 = _map_ionex_data[time].tec_val[std::make_tuple(ilat_0, ilon_1, ihgt)];
        val21 = _map_ionex_data[time].tec_val[std::make_tuple(ilat_1, ilon_0, ihgt)];
        val22 = _map_ionex_data[time].tec_val[std::make_tuple(ilat_1, ilon_1, ihgt)];

        rms11 = _map_ionex_data[time].tec_rms[std::make_tuple(ilat_0, ilon_0, ihgt)];
        rms12 = _map_ionex_data[time].tec_rms[std::make_tuple(ilat_0, ilon_1, ihgt)];
        rms21 = _map_ionex_data[time].tec_rms[std::make_tuple(ilat_1, ilon_0, ihgt)];
        rms22 = _map_ionex_data[time].tec_rms[std::make_tuple(ilat_1, ilon_1, ihgt)];

        /* bilinear interpolation (inside of grid) */
        if (val11 > 0.0 && val12 > 0.0 && val21 > 0.0 && val22 > 0.0)
        {
            gnss_data_interp interp(_spdlog);                   // interpolation class
            base_pair req_pos = base_pair(dbl_lat, dbl_lon); // Target point
            std::map<base_pair, double> data;                   // search datastd::set

            data[base_pair(ilat_0, ilon_0)] = val11;
            data[base_pair(ilat_0, ilon_1)] = val12;
            data[base_pair(ilat_1, ilon_0)] = val21;
            data[base_pair(ilat_1, ilon_1)] = val22;

            interp.bilinear(data, req_pos, value);

            data[base_pair(ilat_0, ilon_0)] = rms11;
            data[base_pair(ilat_0, ilon_1)] = rms12;
            data[base_pair(ilat_1, ilon_0)] = rms21;
            data[base_pair(ilat_1, ilon_1)] = rms22;

            interp.bilinear(data, req_pos, rms);
        }
        /* nearest-neighbour extrapolation (outside of grid) */
        else if (dbl_lat - ilat_0 <= 0.5 && dbl_lon - ilon_0 <= 0.5 && val11 > 0.0)
        {
            value = val11;
            rms = rms11;
        }
        else if (dbl_lat - ilat_0 <= 0.5 && dbl_lon - ilon_0 > 0.5 && val12 > 0.0)
        {
            value = val12;
            rms = rms12;
        }
        else if (dbl_lat - ilat_0 > 0.5 && dbl_lon - ilon_0 <= 0.5 && val21 > 0.0)
        {
            value = val21;
            rms = rms21;
        }
        else if (dbl_lat - ilat_0 > 0.5 && dbl_lon - ilon_0 > 0.5 && val22 > 0.0)
        {
            value = val22;
            rms = rms22;
        }
        else
        {
            int n = 0;
            if (!double_eq(val11, 0.0))
            {
                value += val11;
                rms += rms11;
                n++;
            }
            if (!double_eq(val12, 0.0))
            {
                value += val12;
                rms += rms12;
                n++;
            }
            if (!double_eq(val21, 0.0))
            {
                value += val21;
                rms += rms21;
                n++;
            }
            if (!double_eq(val22, 0.0))
            {
                value += val22;
                rms += rms22;
                n++;
            }

            if (n == 0)
                return false;

            value /= n;
            rms /= n;
        }
        return true;
    }

}