#ifndef GSATVIEW_H
#define GSATVIEW_H

#include "hwa_set_gtype.h"
#include "hwa_base_const.h"
#include "hwa_gnss_all_Nav.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{

    typedef std::map<base_time, base_time> hwa_map_sky_epo;      ///< sat visibility of a single sat/site
    typedef std::map<std::string, hwa_map_sky_epo> hwa_map_SKY_SAT; ///< sat visibility of a single satellite

    /** 
    *@brief calculate satellite visibility. 
    */
    int sat_view(hwa_map_SKY_SAT &sat_view,
                 Triple xyz_rec,
                 gnss_all_nav *all_nav,
                 const base_time &beg, // should be setup reasonably !
                 const base_time &end, // should be setup reasonably !
                 double mask = 0.0);

    /** 
    *@brief calculate satellite visibility. 
    */
    int sat_elev(hwa_map_sky_epo &sat_elev,
                 Triple xyz_rec,
                 gnss_all_nav *all_nav,
                 const base_time &beg, // should be setup reasonably !
                 const base_time &end, // should be setup reasonably !
                 const std::string &prn,
                 double mask);

} // namespace

#endif // # GCOMMON_H
