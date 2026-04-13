#include "hwa_gnss_model_reldelay.h"
#include "hwa_base_const.h"
#include "math.h"

double hwa_gnss::gnss_model_reldelay::reldelay(Triple &crd_site, Triple &vel_site, Triple &crd_sat, Triple &vel_sat)
{
    double reldelay = 0.0;
    //reldelay = 2.0*(dot(crd_sat, vel_sat) - dot(crd_site, vel_site)) / CLIGHT;
    reldelay = 2.0 * (crd_sat.dot(vel_sat)) / CLIGHT;
    Vector xsat = crd_sat;
    Vector xsite = crd_site;

    double r = xsite.norm() + xsat.norm();
    Vector xsat2site = xsite - xsat;
    double r_site2sat = xsat2site.norm();

    reldelay += 2.0 * GM_CGCS / CLIGHT / CLIGHT * log((r + r_site2sat) / (r - r_site2sat));

    return reldelay;
}
