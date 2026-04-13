#include "hwa_base_globaltrans.h"
#include "hwa_gnss_data_satdata.h"

namespace hwa_base {
    int ell2ipp(gnss_data_sats* satdata, Triple& ell_site, Triple& ell_ipp, bool GPStkflag)
    {
        double radius = R_SPHERE;
        if (!GPStkflag)
        {
            double ippE = acos((radius / (radius + 450000.0)) * cos(satdata->ele()));
            double eps = ippE - satdata->ele();
            double ipp_lat = ell_site[0] + eps * cos(satdata->azi());
            if (fabs(ipp_lat * R2D) > 80)
                return -1;
            double ipp_lon = ell_site[1] + eps * sin(satdata->azi()) / cos(ipp_lat);
            if (ipp_lon < 0)
                ipp_lon += 2.0 * hwa_pi;
            if (ipp_lon > 2.0 * hwa_pi)
                ipp_lon -= 2.0 * hwa_pi;

            ell_ipp[0] = ipp_lat;
            ell_ipp[1] = ipp_lon;
        }
        // GPStk formula
        else if (GPStkflag)
        {
            double p = hwa_pi / 2.0 - satdata->ele() - asin((radius * cos(satdata->ele())) / (radius + 450000.0));
            Triple ell_ipp_g;
            ell_ipp_g[0] = asin(sin(ell_site[0]) * cos(p) + cos(ell_site[0]) * sin(p) * cos(satdata->azi()));
            if ((ell_site[0] > 70 * D2R && tan(p) * cos(satdata->azi()) > tan(hwa_pi / 2.0 - ell_site[0])) ||
                (ell_site[0] < -70 * D2R && -tan(p) * cos(satdata->azi()) > tan(hwa_pi / 2.0 - ell_site[0])))
            {
                ell_ipp_g[1] = ell_site[1] + hwa_pi - asin(sin(p) * sin(satdata->azi()) / cos(ell_ipp_g[0]));
            }
            else
            {
                ell_ipp_g[1] = ell_site[1] + asin(sin(p) * sin(satdata->azi()) / cos(ell_ipp_g[0]));
            }

            if (ell_ipp_g[1] > hwa_pi)
                ell_ipp_g[1] = ell_ipp_g[1] - 2 * hwa_pi;
            else if (ell_ipp_g[1] < -hwa_pi)
                ell_ipp_g[1] = ell_ipp_g[1] + 2 * hwa_pi;
            ell_ipp = ell_ipp_g;
        }

        // -------------

        return 1;
    }

    int ell2ipp(gnss_data_sats* satdata, Triple& ell_site, double radius, double ion_hgt, Triple& ell_ipp)
    {

        double ippE = acos((radius / (radius + ion_hgt)) * cos(satdata->ele()));
        double eps = ippE - satdata->ele();
        double ipp_lat = ell_site[0] + eps * cos(satdata->azi());
        if (fabs(ipp_lat * R2D) > 80)
            return -1;
        double ipp_lon = ell_site[1] + eps * sin(satdata->azi()) / cos(ipp_lat);
        if (ipp_lon < 0)
            ipp_lon += 2.0 * hwa_pi;
        if (ipp_lon > 2.0 * hwa_pi)
            ipp_lon -= 2.0 * hwa_pi;

        ell_ipp[0] = ipp_lat;
        ell_ipp[1] = ipp_lon;

        return 1;
    }
}