#include <cmath>
#include <iomanip>
#include "hwa_gnss_model_iono.h"
#include "hwa_base_const.h"

namespace hwa_gnss
{
    hwa_gnss_model_iono::hwa_gnss_model_iono()
    {
    }

    hwa_gnss_model_iono::~hwa_gnss_model_iono()
    {
    }

    // get ZHD
    // ----------
    double hwa_gnss_model_iono::ionMapFunc(IONMPFUNC mapFunc, double ele) // ell v RADIANECH !! TREBA SJEDNOTIT
    {
        if (mapFunc == IONMPFUNC::ICOSZ)
        {
            double mf = 1 / sqrt(1.0 - pow(R_SPHERE / (R_SPHERE + 450000.0) * sin(hwa_pi / 2.0 - ele), 2));
            return mf;
        }
        else if (mapFunc == IONMPFUNC::QFAC)
        {
            return 0.0;
        }
        else
        {
            return 0.0;
        }
    }

    double hwa_gnss_model_iono_brdc::getIonoDelay(gnss_all_nav *nav, gnss_data_sats &satdata, const base_time &epo, const Triple &site_ell)
    {
        // jdhuang
        if (nav == nullptr)
        {
            return 0.0;
        }

        // pre-Judge
        if (site_ell[2] < -1000 || satdata.ele() <= 0)
            return 0.0;

        gnss_data_iono_corr gps_alpha = nav->gegnss_data_iono_corr(IONO_CORR::IO_GPSA);
        gnss_data_iono_corr gps_beta = nav->gegnss_data_iono_corr(IONO_CORR::IO_GPSB);

        /* base_earth centered angle (semi-circle) */
        double psi = 0.0137 / (satdata.ele() / hwa_pi + 0.11) - 0.022;

        /* subionospheric latitude/longitude (semi-circle) */
        double phi = site_ell[0] / hwa_pi + psi * cos(satdata.azi());
        if (phi > 0.416)
            phi = 0.416;
        else if (phi < -0.416)
            phi = -0.416;
        double lam = site_ell[1] / hwa_pi + psi * sin(satdata.azi()) / cos(phi * hwa_pi);

        /* geomagnetic latitude (semi-circle) */
        phi += 0.064 * cos((lam - 1.617) * hwa_pi);

        /* local time (s) */
        double tt = 43200.0 * lam + epo.sow() + epo.dsec();
        tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

        /* slant factor */
        double f = 1.0 + 16.0 * pow(0.53 - satdata.ele() / hwa_pi, 3.0);

        /* ionospheric delay */
        double amp = gps_alpha.x0 + phi * (gps_alpha.x1 + phi * (gps_alpha.x2 + phi * gps_alpha.x3));
        double per = gps_beta.x0 + phi * (gps_beta.x1 + phi * (gps_beta.x2 + phi * gps_beta.x3));
        amp = amp < 0.0 ? 0.0 : amp;
        per = per < 72000.0 ? 72000.0 : per;
        double x = 2.0 * hwa_pi * (tt - 50400.0) / per;

        return CLIGHT * f * (fabs(x) < 1.57 ? 5e-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5e-9);
    }

    bool hwa_gnss_model_iono_tecgrid::getIonoDelay(gnss_data_ionex *ionexdata, gnss_data_sats &satdata, const base_time &epo, Triple &site_pos, double &value, double &rms)
    {
        value = 0.0;
        rms = 0.0;

        Triple site_ell;
        xyz2ell(site_pos, site_ell, false);

        /*elevation and site height check*/
        if (site_ell[2] < -1000 || satdata.ele() <= 0)
            return false;

        /*Slant total electron content by tec grid data*/
        if (!ionexdata->getSTEC(epo, satdata, site_ell, value, rms))
            return false;

        /*Slant ionospheric delay(L1) (m)*/
        double f1 = satdata.frequency(gnss_sys::band_priority(satdata.gsys(), FREQ_1));
        double fact = 40.30 * 1e16 / f1 / f1;
        value *= fact;
        rms *= SQR(fact);

        return true;
    }

    double hwa_gnss_model_iono_ionf::getIonoDelay(const base_time &epo, const std::string &sat, const std::string &rec)
    {
        return 0.0;
    }

} // namespace
