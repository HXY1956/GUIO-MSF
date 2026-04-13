#include <math.h>
#include "hwa_gnss_all_nav.h"
#include "hwa_base_time.h"
#include "hwa_gnss_Satview.h"
#include "hwa_base_common.h"

namespace hwa_gnss
{
    // calculate satellite visibility
    // ----------
    int sat_view(hwa_map_SKY_SAT &sat_view,
                 Triple xyz_rec,
                 gnss_all_nav *all_nav,
                 const base_time &beg, // should be setup reasonably !
                 const base_time &end, // should be setup reasonably !
                 double mask)
    {

        if (beg > end || beg == FIRST_TIME || end == LAST_TIME ||
            !all_nav)
            return -1;

        Triple xyz_sat;
        Triple top_sat;
        double sec = 120.0; // add seconds as a function of elevation (120s ~ 1 deg)
        double tdif = 0.0;
        double r_radi, s_radi, s_topo;

        // loop over all satellites
        hwa_map_sats _gnss_sats = gnss_sats();
        hwa_map_sats::const_iterator itSYS;
        std::set<std::string>::const_iterator itSAT;

        for (itSYS = _gnss_sats.begin(); itSYS != _gnss_sats.end(); ++itSYS)
        {
            GSYS gs = itSYS->first;

            for (itSAT = _gnss_sats[gs].begin(); itSAT != _gnss_sats[gs].end(); ++itSAT)
            {
                std::string prn = *itSAT;
#ifdef DEBUG
                base_time beg_eph = all_nav->beg_time(prn);
                base_time end_eph = all_nav->end_time(prn);
                std::cerr << prn << beg_eph.str_ymdhms("  beg: ") << end_eph.str_ymdhms("  end: ") << std::endl;

                if (beg_eph == LAST_TIME || end_eph == FIRST_TIME)
                {
                    std::cerr << prn << " no eph - sat skipped "
                         << " per: " << beg_eph.str_ymdhms()
                         << " - " << end_eph.str_ymdhms() << std::endl;
                    continue;
                }
#endif

                base_time beg_lim = beg;
                base_time end_lim = end;

#ifdef DEBUG
                double dt_lim = end_lim - beg_lim;
                if (dt_lim > 86400 || beg_lim == FIRST_TIME)
                    beg_lim = beg_eph;
                if (dt_lim > 86400 || end_lim == LAST_TIME)
                    end_lim = end_eph;

                std::cout << " sat: " << prn
                     << " per: " << beg_lim.str_ymdhms()
                     << " - " << end_lim.str_ymdhms()
                     << std::endl;
#endif

                base_time tt = beg_lim;
                base_time tt_sav = beg_lim;
                double ele = 0.0;
                double ele_sav = 0.0;
                int direction = 0;
                int iter = 0;
                bool init = true;

                while (tt < end_lim + 1 && ++iter < 75)
                {
                    if (all_nav->pos(prn, tt, xyz_sat.data(), NULL, NULL) < 0)
                    {

                        // TEMPORARY SOLUTION FOR GLONASS (hope this all function will be re-implemented soon)
                        if (gs == GLO)
                        {
                            tt.add_secs(3600);
                            ele = 1.0;
                            continue;
                        }      // risky to skip nav gap
                        break; // better STOP and skip this SAT, but a problem with GLONASS when missing NAV
                    }

                    top_sat = xyz_sat - xyz_rec;

                    r_radi = xyz_rec.norm() / 1000.0; // km
                    s_radi = xyz_sat.norm() / 1000.0; // km
                    s_topo = top_sat.norm() / 1000.0; // km

                    ele = -90 + acos((pow(s_topo, 2.0) + pow(r_radi, 2.0) - pow(s_radi, 2.0)) / (2 * r_radi * s_topo)) / hwa_pi * 180;

                    if (gs == QZS || gs == SBS || gnss_sys::bds_geo(prn))
                    {
                        // initiate new start with end time!
                        if (ele > mask)
                            sat_view[prn][beg_lim] = end_lim;
                        //      std::cerr << prn << " - satview: skipping geostacionary satellites QZS, SBS or BDS, ele: " << ele << std::endl;
                        break;
                    }

                    // gradient
                    tdif = tt - tt_sav;
                    if (tdif != 0.0)
                        sec = tdif / fabs(ele - ele_sav) / 2.0;
                    if (sec > 240.0)
                        sec = 240.0;

                    // horizon control
                    //    bool horizon_found = ( fabs(ele) < 0.01 && !init ); // not below 0.01! for 1s increase at least
                    bool horizon_found = (fabs(tt - tt_sav) < 2 && !init);           // MORE ROBUST, GLONASS Particularly!
                                                                                     //    bool horizon_cross = ((ele > 0) - (ele_sav > 0) && !init ); // alternative to a sign check
                    bool horizon_cross = ((ele > mask) - (ele_sav > mask) && !init); // alternative to a sign check

                    if (!init && !horizon_found) //  !horizon_cross &&
                    {
                        if (ele_sav < ele)
                            direction = +1;
                        if (ele_sav > ele)
                            direction = -1;
                    }

                    init = false;

#ifdef DEBUG
                    std::cerr << std::fixed << setprecision(3)
                         << setw(3) << iter
                         << "  " << prn
                         << "  " << tt.str_ymdhms()
                         << "  " << tt_sav.str_ymdhms()
                         << "  " << tdif
                         << " sec: " << setw(7) << sec
                         << " ele: " << setw(7) << ele
                         << " sav: " << setw(7) << ele_sav
                         << " " << init << ":" << horizon_cross << ":" << horizon_found
                         << " " << setw(2) << direction
                         << "  " << std::endl;
#endif

                    // horizon found
                    if (horizon_found)
                    { // && direction != 0 ){
#ifdef DEBUG
                        std::cerr << "  " << prn << " add H 1800.000";
                        if (direction > 0)
                        {
                            std::cerr << "  ascend horizon\n\n";
                        }
                        else
                        {
                            std::cerr << " descend horizon\n\n";
                        }
#endif
                        // FILL
                        std::map<base_time, base_time>::reverse_iterator itEND = sat_view[prn].rbegin();
                        if (sat_view[prn].size() > 0) // multiple visibility record
                        {
                            if (direction < 0)
                                itEND->second = tt; // add visibility end time
                            else
                                sat_view[prn][tt] = end_lim; // initiate new start with end time!
                        }
                        else // initial visibility record
                        {
                            if (direction < 0)
                                sat_view[prn][beg_lim] = tt; // add visibility end time
                            else
                                sat_view[prn][tt] = end_lim; // initiate new start with end time!
                        }

                        iter = 0;
                        tt_sav = tt;
                        tt.add_secs(300);
                        ele_sav = mask + direction * 0.01;
                        continue;
                    }

                    // horizon roughly crossing
                    else if (horizon_cross)
                    {

                        if (fabs(ele_sav - mask) > fabs(ele - mask) || fabs(ele - mask) < 1.0)
                        {
                            //            std::cerr << "  " << prn << "  add M " << setw(8) << -sec*fabs(ele-mask) << std::endl;
                            tt_sav = tt;
                            tt.add_secs((int)(-sec * fabs(ele - mask)));
                        }
                        else
                        {
                            //            std::cerr << "  " << prn << "  add P " << setw(8) << sec*fabs(ele_sav-mask) << std::endl;
                            if (fabs(ele_sav - mask) < 0.2)
                                tt = tt_sav + sec * fabs((ele_sav - mask) / 2);
                            else
                                tt = tt_sav + sec * fabs((ele_sav - mask));
                        }
                    }

                    else
                    {
                        ele_sav = ele;
                        tt_sav = tt;

                        double add = sec * fabs(ele - mask);
                        if (fabs(ele - mask) > 10)
                            add *= 0.6;

                        //      std::cout << prn << " add A " << setw(8) << add << std::endl;
                        tt.add_secs((int)add);
                    }
                }
            }
        }

#ifdef DEBUG
        hwa_map_SKY_SAT::iterator itPRN;
        for (itPRN = sat_view.begin(); itPRN != sat_view.end(); ++itPRN)
        {
            string prn = itPRN->first;

            hwa_map_sky_epo::iterator itBEG;
            for (itBEG = sat_view[prn].begin(); itBEG != sat_view[prn].end(); ++itBEG)
            {
                std::cerr << " PRN: " << prn
                     << "   " << itBEG->first.str_ymdhms()
                     << " - " << itBEG->second.str_ymdhms()
                     << " time:" << prn << " " << itBEG->second - itBEG->first
                     << std::endl;
            }
            std::cerr << std::endl;
        }
#endif

        return 1;
    }

    // calculate satellite visibility
    // ----------
    int sat_elev(hwa_map_sky_epo &sat_elev,
                 Triple xyz_rec,
                 gnss_all_nav *all_nav,
                 const base_time &beg, // should be setup reasonably !
                 const base_time &end, // should be setup reasonably !
                 const std::string &prn,
                 double mask)
    {

        if (beg > end || beg == FIRST_TIME || end == LAST_TIME ||
            !all_nav)
            return -1;

        Triple xyz_sat;
        Triple top_sat;
        double sec = 120.0; // add seconds as a function of elevation (120s ~ 1 deg)
        double tdif = 0.0;
        double r_radi, s_radi, s_topo;

        // loop over all satellites
        hwa_map_sats _gnss_sats = gnss_sats();
        hwa_map_sats::const_iterator itSYS;
        std::set<std::string>::const_iterator itSAT;

        GSYS gs = gnss_sys::str2gsys(prn);

#ifdef DEBUG
        base_time beg_eph = all_nav->beg_time(prn);
        base_time end_eph = all_nav->end_time(prn);
        std::cerr << prn << beg_eph.str_ymdhms("  beg: ") << end_eph.str_ymdhms("  end: ") << std::endl;

        if (beg_eph == LAST_TIME || end_eph == FIRST_TIME)
        {
            std::cerr << prn << " no eph - sat skipped "
                 << " per: " << beg_eph.str_ymdhms()
                 << " - " << end_eph.str_ymdhms() << std::endl;
            return -1;
        }
#endif

        base_time beg_lim = beg;
        base_time end_lim = end;

#ifdef DEBUG
        double dt_lim = end_lim - beg_lim;
        if (dt_lim > 86400 || beg_lim == FIRST_TIME)
            beg_lim = beg_eph;
        if (dt_lim > 86400 || end_lim == LAST_TIME)
            end_lim = end_eph;

        std::cout << " sat: " << prn
             << " per: " << beg_lim.str_ymdhms()
             << " - " << end_lim.str_ymdhms()
             << std::endl;
#endif

        base_time tt = beg_lim;
        base_time tt_sav = beg_lim;
        double ele = 0.0;
        double ele_sav = 0.0;
        int direction = 0;
        int iter = 0;
        bool init = true;

        while (tt < end_lim + 1 && ++iter < 75)
        {
            if (all_nav->pos(prn, tt, xyz_sat.data(), NULL, NULL) < 0)
            {
                // TEMPORARY SOLUTION FOR GLONASS (hope this all function will be re-implemented soon)
                if (gs == GLO)
                {
                    tt.add_secs(3600);
                    ele = 1.0;
                    continue;
                }      // risky to skip nav gap
                break; // better STOP and skip this SAT, but a problem with GLONASS when missing NAV
            }

            top_sat = xyz_sat - xyz_rec;

            r_radi = xyz_rec.norm() / 1000.0; // km
            s_radi = xyz_sat.norm() / 1000.0; // km
            s_topo = top_sat.norm() / 1000.0; // km

            ele = -90 + acos((pow(s_topo, 2.0) + pow(r_radi, 2.0) - pow(s_radi, 2.0)) / (2 * r_radi * s_topo)) / hwa_pi * 180;

            if (gs == QZS || gs == SBS || gnss_sys::bds_geo(prn))
            {
                // initiate new start with end time!
                if (ele > mask)
                    sat_elev[beg_lim] = end_lim;
                //      std::cerr << prn << " - satview: skipping geostacionary satellites QZS, SBS or BDS, ele: " << ele << std::endl;
                break;
            }

            // gradient
            tdif = tt - tt_sav;
            if (tdif != 0.0)
                sec = tdif / fabs(ele - ele_sav) / 2.0;
            if (sec > 240.0)
                sec = 240.0;

            // horizon control
            //    bool horizon_found = ( fabs(ele) < 0.01 && !init ); // not below 0.01! for 1s increase at least
            bool horizon_found = (fabs(tt - tt_sav) < 2 && !init);           // MORE ROBUST, GLONASS Particularly!
                                                                             //    bool horizon_cross = ((ele > 0) - (ele_sav > 0) && !init ); // alternative to a sign check
            bool horizon_cross = ((ele > mask) - (ele_sav > mask) && !init); // alternative to a sign check

            if (!init && !horizon_found) //  !horizon_cross &&
            {
                if (ele_sav < ele)
                    direction = +1;
                if (ele_sav > ele)
                    direction = -1;
            }

            init = false;

            //#ifdef DEBUG
            std::cerr << std::fixed << std::setprecision(3)
                 << std::setw(3) << iter
                 << "  " << prn
                 << "  " << tt.str_ymdhms()
                 << "  " << tt_sav.str_ymdhms()
                 << "  " << tdif
                 << " sec: " << std::setw(7) << sec
                 << " ele: " << std::setw(7) << ele
                 << " sav: " << std::setw(7) << ele_sav
                 << " " << init << ":" << horizon_cross << ":" << horizon_found
                 << " " << std::setw(2) << direction
                 << "  " << std::endl;
            //#endif

            // horizon found
            if (horizon_found)
            { // && direction != 0 ){
                //#ifdef DEBUG
                std::cerr << "  " << prn << " add H 1800.000";
                if (direction > 0)
                {
                    std::cerr << "  ascend horizon\n\n";
                }
                else
                {
                    std::cerr << " descend horizon\n\n";
                }
                //#endif
                // FILL
                std::map<base_time, base_time>::reverse_iterator itEND = sat_elev.rbegin();
                if (sat_elev.size() > 0) // multiple visibility record
                {
                    if (direction < 0)
                        itEND->second = tt; // add visibility end time
                    else
                        sat_elev[tt] = end_lim; // initiate new start with end time!
                }
                else // initial visibility record
                {
                    if (direction < 0)
                        sat_elev[beg_lim] = tt; // add visibility end time
                    else
                        sat_elev[tt] = end_lim; // initiate new start with end time!
                }

                iter = 0;
                tt_sav = tt;
                tt.add_secs(300);
                ele_sav = mask + direction * 0.01;
                continue;
            }

            // horizon roughly crossing
            else if (horizon_cross)
            {

                if (fabs(ele_sav - mask) > fabs(ele - mask) || fabs(ele - mask) < 1.0)
                {
                    //            std::cerr << "  " << prn << "  add M " << setw(8) << -sec*fabs(ele-mask) << std::endl;
                    tt_sav = tt;
                    tt.add_secs((int)(-sec * fabs(ele - mask)));
                }
                else
                {
                    //            std::cerr << "  " << prn << "  add P " << setw(8) << sec*fabs(ele_sav-mask) << std::endl;
                    if (fabs(ele_sav - mask) < 0.2)
                        tt = tt_sav + sec * fabs((ele_sav - mask) / 2);
                    else
                        tt = tt_sav + sec * fabs((ele_sav - mask));
                }
            }

            else
            {
                ele_sav = ele;
                tt_sav = tt;

                double add = sec * fabs(ele - mask);
                if (fabs(ele - mask) > 10)
                    add *= 0.6;

                //      std::cout << prn << " add A " << setw(8) << add << std::endl;
                tt.add_secs((int)add);
            }
        }

        //#ifdef DEBUG
        hwa_map_sky_epo::iterator itBEG;
        for (itBEG = sat_elev.begin(); itBEG != sat_elev.end(); ++itBEG)
        {
            std::cerr << " PRN: " << prn
                 << "   " << itBEG->first.str_ymdhms()
                 << " - " << itBEG->second.str_ymdhms()
                 << " time:" << prn << " " << itBEG->second - itBEG->first
                 << std::endl;
        }
        std::cerr << std::endl;
        //#endif

        return 1;
    }

} // namespace
