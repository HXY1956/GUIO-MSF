#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include "hwa_gnss_all_obs.h"
#include "hwa_base_log.h"

namespace hwa_gnss
{
    gnss_all_obs::gnss_all_obs() : base_data(),
                             _set(0),
                             _nepoch(0),
                             _overwrite(false)
    {
        id_type(base_data::ALLOBS);
        id_group(base_data::GRP_obsERV);
    }
    gnss_all_obs::gnss_all_obs(base_log spdlog)
        : base_data(spdlog),
          _set(0),
          _nepoch(0),
          _overwrite(false)
    {
        id_type(base_data::ALLOBS);
        id_group(base_data::GRP_obsERV);
    }
    gnss_all_obs::gnss_all_obs(base_log spdlog, set_base *set)
        : base_data(spdlog),
          _set(set),
          _nepoch(0),
          _overwrite(false)
    {
        if (nullptr == set)
        {
            throw_logical_error(spdlog, "your std::set ptr is nullptr");
        }
        id_type(base_data::ALLOBS);
        id_group(base_data::GRP_obsERV);
    }

    // destructor
    // ----------
    gnss_all_obs::~gnss_all_obs()
    {

        _mapobj.clear();
        _filter.clear();
    }

    // settings
    // ----------
    void gnss_all_obs::gset(set_base *gset)
    {
        _set = gset;
        _sys = dynamic_cast<set_gen *>(_set)->sys();
        _smp = dynamic_cast<set_gen *>(_set)->sampling();
        _scl = dynamic_cast<set_gen *>(_set)->sampling_scalefc(); // scaling 10^decimal-digits

        return;
    }

    // return std::set of available GNSS systems
    // --------------------------
    std::set<GSYS> gnss_all_obs::sys(const std::string &site)
    {

        hwa_map_ti_obsspt::const_iterator itEpo;
        hwa_map_id_obsspt::iterator itSat;
        std::set<GSYS> temp;
        base_time t;

        for (itEpo = _mapobj[site].begin(); itEpo != _mapobj[site].end(); ++itEpo) // loop over epochs
        {
            t = itEpo->first;
            for (itSat = _mapobj[site][t].begin(); itSat != _mapobj[site][t].end(); ++itSat) // loop over satellites
            {
                temp.insert(itSat->second->gsys());
            }
        }
        return temp;
    }

    // return used systems for epoch
    // -------------------------------
    std::set<GSYS> gnss_all_obs::sys(const std::string &site, const base_time &t)
    {
        std::set<GSYS> gnss = _gsys(site, t);
        return gnss;
    }

    // protected
    // ---------
    std::set<GSYS> gnss_all_obs::_gsys(const std::string &site, const base_time &t)
    {
        std::set<GSYS> gnss;

        std::vector<gnss_data_sats> satdata = _gobs(site, t);

        std::vector<gnss_data_sats>::iterator it;
        for (it = satdata.begin(); it != satdata.end(); ++it)
        {
            //    std::cout << it->epoch().str_hms() << " " << it->sat() << std::endl;
            GSYS gsys = it->gsys();
            gnss.insert(gsys);
        }

        return gnss;
    }

    // return list of available stations
    // ----------
    std::set<std::string> gnss_all_obs::stations()
    {
        std::set<std::string> all_sites;
        hwa_map_iti_obsspt::const_iterator itSITE = _mapobj.begin();

        while (itSITE != _mapobj.end())
        {
            all_sites.insert(itSITE->first);
            ++itSITE;
        }
        return all_sites;
    }

    // return list of available satellites
    // ----------
    std::set<std::string> gnss_all_obs::sats(const std::string &site,
                                const base_time &t, GSYS gnss)
    {
        std::set<std::string> all_sats = _sats(site, t, gnss);
        return all_sats;
    }

    // return list of available satellites
    // ----------
    std::set<std::string> gnss_all_obs::_sats(const std::string &site,
                                 const base_time &t, GSYS gnss)
    {
        std::set<std::string> all_sats;

        if (_mapobj.find(site) == _mapobj.end() ||
            _mapobj[site].find(t) == _mapobj[site].end())
        {
            return all_sats;
        }

        hwa_map_id_obsspt::const_iterator itSAT = _mapobj[site][t].begin();
        while (itSAT != _mapobj[site][t].end())
        {
            GSYS sys = itSAT->second->gsys();
            if (gnss == GNS)
                all_sats.insert(itSAT->first);
            else if (gnss == GPS && sys == GPS)
                all_sats.insert(itSAT->first);
            else if (gnss == GLO && sys == GLO)
                all_sats.insert(itSAT->first);
            else if (gnss == GAL && sys == GAL)
                all_sats.insert(itSAT->first);
            else if (gnss == BDS && sys == BDS)
                all_sats.insert(itSAT->first);
            else if (gnss == QZS && sys == QZS)
                all_sats.insert(itSAT->first);
            else if (gnss == SBS && sys == SBS)
                all_sats.insert(itSAT->first);
            itSAT++;
        }
        return all_sats;
    }

    // return list of available satellites
    // ----------
    std::vector<gnss_data_sats> gnss_all_obs::obs(const std::string &site,
                                      const base_time &t)
    {
        std::vector<gnss_data_sats> all_obs = _gobs(site, t);
        return all_obs;
    }

    std::vector<gnss_data_obs_manager *> gnss_all_obs::obs(bool isPtr, const std::string &site,
                                        const base_time &t)
    {
        std::vector<gnss_data_obs_manager *> all_obs = _gobs(true, site, t);
        return all_obs;
    }

    // return list of available satellites for all sites
    // ----------
    std::vector<gnss_data_sats> gnss_all_obs::obs(const std::set<std::string> &sites, const base_time &t)
    {
        std::vector<gnss_data_sats> all_obs;
        // std::cerr << sites.size() << std::endl;
        for (auto site_iter = sites.begin(); site_iter != sites.end(); site_iter++)
        {
            std::vector<gnss_data_sats> site_obs = _gobs(*site_iter, t);
            all_obs.insert(all_obs.end(), site_obs.begin(), site_obs.end());
        }
        return all_obs;
    }

    // protected
    // ---------
    std::vector<gnss_data_sats> gnss_all_obs::_gobs(const std::string &site,
                                        const base_time &t)
    {
        std::vector<gnss_data_sats> all_obs;
        base_time tt(base_time::GPS);

        if (_find_epo(site, t, tt) < 0)
        {
            return all_obs;
        }

        hwa_map_id_obsspt::const_iterator itSAT = _mapobj[site][tt].begin();
        while (itSAT != _mapobj[site][tt].end())
        {
            gnss_data_sats tmp(*itSAT->second);
            all_obs.push_back(tmp);
            itSAT++;
        }

        return all_obs;
    }

    std::vector<gnss_data_obs_manager *> gnss_all_obs::_gobs(bool isPtr, const std::string &site,
                                          const base_time &t)
    {
        std::vector<gnss_data_obs_manager *> all_obs;
        base_time tt(base_time::GPS);

        if (_find_epo(site, t, tt) < 0)
        {
            return all_obs;
        }
        hwa_map_id_obsspt::iterator itSAT = _mapobj[site][tt].begin();
        while (itSAT != _mapobj[site][tt].end())
        {
            all_obs.push_back((*itSAT).second.get());
            itSAT++;
        }

        return all_obs;
    }

    // TEMPORARY !!!
    // return list of available satellites (POINTERS!)
    // ----------
    std::vector<hwa_spt_obsmanager> gnss_all_obs::obs_pt(const std::string &site, const base_time &t)
    {
        std::vector<hwa_spt_obsmanager> all_obs;
        base_time tt(base_time::GPS);

        if (_find_epo(site, t, tt) < 0)
        {
            return all_obs;
        }

        hwa_map_id_obsspt::iterator itSAT = _mapobj[site][tt].begin();
        while (itSAT != _mapobj[site][tt].end())
        {

            std::string sat = itSAT->first;

            // TESTING NEW METHOD
            all_obs.push_back(std::dynamic_pointer_cast<gnss_data_obs_manager>(itSAT->second));

            itSAT++;
        }
        return all_obs;
    }

    // return single difference between satellites
    // ---------
    /*t_gsdbs gnss_all_obs::sdbs(const std::string& site,
                        const base_time& t)
{
   t_gsdbs diff;
   
   std::vector<gnss_data_sats> gobs = this->obs(site, t);   
   diff.pivot( *gobs.begin() );
      
   for (unsigned int i = 1; i < gobs.size(); i++){
      gobs[i] = gobs[i] - gobs[0];      
   }        
   
   gobs.erase(gobs.begin()); // erase reference observation

   diff.data(gobs);
   
   return diff;
}
*/

    // return list of available satellites
    // ----------
    std::vector<base_time> gnss_all_obs::epochs(const std::string &site)
    {
        std::vector<base_time> all_epochs;

        if (_mapobj.find(site) == _mapobj.end())
        {
            // std::cout << "site not found: " << site << t.str_ymdhms() << std::endl; std::cout.flush();
            return all_epochs;
        }

        hwa_map_ti_obsspt::iterator it = _mapobj[site].begin();

        while (it != _mapobj[site].end())
        {
            all_epochs.push_back(it->first);
            ++it;
        }
        return all_epochs;
    }

    base_time gnss_all_obs::load(const std::string &site, const double &t)
    {
        std::vector<base_time>::const_iterator it = _allepoches.begin();
        it = upper_bound(_allepoches.begin(), _allepoches.end(), t);
        if (it == _allepoches.end() || it == _allepoches.begin())
            return base_time(0.0);
        std::vector<base_time>::const_iterator it_up = it, it_low = --it;
        base_time crt(it->gwk(), t);
        if (fabs(crt.diff(*it_low)) < fabs(crt.diff(*it_up)))
            return *it_low;
        else
            return *it_up;
    }

    base_time gnss_all_obs::begT()
    {
        base_time tmp = LAST_TIME;

        for (hwa_map_iti_obsspt::const_iterator it = _mapobj.begin(); it != _mapobj.end(); ++it)
        {
            if (it->second.size() != 0)
            {
                tmp = it->second.begin()->first;
                break;
            }
        }

        //if (_mapobj.find(site) != _mapobj.end() &&
        //    _mapobj[site].begin() != _mapobj[site].end()) tmp = _mapobj[site].begin()->first;

        //// get first synchronized obs
        //if (smpl > 0.0) {
        //    auto itEpoB = _mapobj[site].begin();
        //    auto itEpoE = _mapobj[site].end();
        //    int sod = static_cast<int>(dround(itEpoB->first.sod() + itEpoB->first.dsec()));

        //    while (sod % static_cast<int>(smpl) != 0 && ++itEpoB != itEpoE) {
        //        sod = static_cast<int>(dround(itEpoB->first.sod() + itEpoB->first.dsec()));
        //        tmp = itEpoB->first;
        //        tmp.reset_sod();
        //        tmp.add_secs(sod);
        //        //      std::cerr << " sync sod = " << std::fixed << std::setprecision(4) << sod << "  " << itEpoB->first.str_ymdhms() << "  " << tmp.str_ymdhms() << std::endl;
        //    }
        //}
        return tmp;
    }

    // return first position for satellite
    // ----------
    base_time gnss_all_obs::beg_obs(const std::string &site, double smpl)
    {
        base_time tmp = LAST_TIME;

        if (_mapobj.find(site) != _mapobj.end() &&
            _mapobj[site].begin() != _mapobj[site].end())
            tmp = _mapobj[site].begin()->first;

        // get first synchronized obs
        if (smpl > 0.0)
        {
            auto itEpoB = _mapobj[site].begin();
            auto itEpoE = _mapobj[site].end();
            int sod = static_cast<int>(dround(itEpoB->first.sod() + itEpoB->first.dsec()));

            while (sod % static_cast<int>(smpl) != 0 && ++itEpoB != itEpoE)
            {
                sod = static_cast<int>(dround(itEpoB->first.sod() + itEpoB->first.dsec()));
                tmp = itEpoB->first;
                tmp.reset_sod();
                tmp.add_secs(sod);
                //      std::cerr << " sync sod = " << std::fixed << std::setprecision(4) << sod << "  " << itEpoB->first.str_ymdhms() << "  " << tmp.str_ymdhms() << std::endl;
            }
        }
        return tmp;
    }

    // return last position for satellite
    // ----------
    base_time gnss_all_obs::end_obs(const std::string &site)
    {
        base_time tmp = FIRST_TIME;
        if (_mapobj.find(site) != _mapobj.end() &&
            _mapobj[site].begin() != _mapobj[site].end())
            tmp = _mapobj[site].rbegin()->first;
        return tmp;
    }

    // add observations ( both P and L in meters !!!)
    // ----------
    int gnss_all_obs::addobs(hwa_spt_obsmanager obs)
    {
        // repair small out-sync ( < 10 ms )
        double outsync = fmod(obs->epoch().dsec(), _smp) - round(fmod(obs->epoch().dsec(), _smp));
        if (fabs(outsync) < 0.014 && fabs(outsync) > 1e-6)
        {
            obs->epo(obs->epoch() - outsync);
            std::vector<GOBS> v_obs = obs->obs();
            std::vector<GOBS>::iterator itOBS = v_obs.begin();
            for (; itOBS != v_obs.end(); ++itOBS)
            {
                GOBSTYPE obstype = str2gobstype(gobs2str(*itOBS));
                if (obstype == TYPE_P || obstype == TYPE_C)
                    obs->resetobs(*itOBS, obs->getobs(*itOBS) - CLIGHT * outsync);
                else if (obstype == TYPE_L)
                    obs->resetobs(*itOBS, obs->getobs(*itOBS) - CLIGHT * outsync / obs->wavelength(str2gobsband(gobs2str(*itOBS))));
                else if (obstype == TYPE_S) // add wh
                    obs->resetobs(*itOBS, obs->getobs(*itOBS));
            }
        }

        base_time t(obs->epoch()), tt = t;
        std::string site = obs->site();
        std::string sat = obs->sat();
        //  std::cout << "ADDING site: " << site << std::endl;

        if (_map_sites.find(site) == _map_sites.end())
            _map_sites.insert(site);


        int epo_found = _find_epo(site, t, tt);
        auto itSAT = _mapobj[site][tt].find(sat);

        // add new observations (or overwrite)
        // ===================================
        if (_overwrite || epo_found < 0         // epoch exists (smart search)
            || itSAT == _mapobj[site][tt].end() // satellite exists
                                                //                 || itEPO == _mapobj[site].end()
                                                //                 || itSAT == _mapobj[site][tt].end()
        )
        {
#ifdef DEBUG
            std::cout << " ADDING " << site << " " << _mapobj[site].size() << " " << _nepoch // << " " <<  nepochs(site) // MUTEX !!!
                 << " sat " << sat << " " << tt.str_ymdhms() << " "
                 << "\n";
            std::cout.flush();
#endif

            // delete old if exists
            // ====================
            if (_overwrite && epo_found > 0         // epoch exists (smart search)
                && itSAT != _mapobj[site][tt].end() // satellite exists
                                                    //    if( _overwrite && itEPO != _mapobj[site].end()
                                                    //                   && itSAT != _mapobj[site][tt].end()
            )
            {
                _mapobj[site][tt].erase(sat);
            }

            // too many epochs (remove old)
            // ============================
            if (_nepoch > 0 && _mapobj[site].size() > _nepoch + 10)
            { // +10 .. reduce number of removals to every tenths

                auto itEPO = _mapobj[site].begin();
                while (itEPO != _mapobj[site].end())
                {

                    if (_mapobj[site].size() <= _nepoch)
                    {
                        break;
                    }

                    base_time t = itEPO->first;

                    _mapobj[site][t].erase(_mapobj[site][t].begin(), _mapobj[site][t].end());
                    _mapobj[site].erase(itEPO++);
                }
            }

            // distinguish GNSS specific message
            // =================================
            if (obs->id_type() == base_data::OBSGNSS)
            {

                _mapobj[site][tt][sat] = obs;
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "warning: gnss_data_obs_manager record not identified!");
                return 1;
            }
        }
        else
        {
            return 0;
        }

        // comments
        return 0;
    }

    // clean function
    // ----------
    void gnss_all_obs::clean_outer(const std::string &obj,
                                const base_time &beg,
                                const base_time &end)
    {

        if (end < beg)
            return;
        // loop over all object [default]
        hwa_map_iti_obsspt::const_iterator itOBJ = _mapobj.begin();
        hwa_map_iti_obsspt::const_iterator itOBJ2 = _mapobj.end();

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "obs clean request: " + obj + beg.str_ymdhms(": ") + end.str_ymdhms(" - "));

        // loop over a single object (if obj defined !)
        if (!obj.empty())
            itOBJ = itOBJ2 = _mapobj.find(obj);
        if (itOBJ2 != _mapobj.end())
            itOBJ2++;

        for (; itOBJ != itOBJ2; ++itOBJ)
        {

            std::string site = itOBJ->first;

            // find and CLEAN all data (epochs) out of the specified period !
            hwa_map_ti_obsspt::iterator it;
            hwa_map_ti_obsspt::iterator itFirst = _mapobj[site].begin();
            hwa_map_ti_obsspt::iterator itLast = _mapobj[site].end();
            hwa_map_ti_obsspt::iterator itBeg = _mapobj[site].lower_bound(beg); // greater|equal
            hwa_map_ti_obsspt::iterator itEnd = _mapobj[site].upper_bound(end); // greater only!

            hwa_map_id_obsspt::iterator itOBS;

            // delete before BEGIN request
            if (itBeg != itLast && itBeg != itFirst)
            {

                for (it = itFirst; it != itBeg; ++it)
                {
                    base_time tt(it->first);

                    _mapobj[site][tt].erase(_mapobj[site][tt].begin(), _mapobj[site][tt].end());
                }

                _mapobj[site].erase(itFirst, itBeg); // erase all but itBeg
            }

            // delete after END request
            if (itEnd != itLast)
            { // && ++itEnd != itLast ){

                for (it = itEnd; it != itLast; ++it)
                {

                    base_time tt(it->first);

                    _mapobj[site][tt].erase(_mapobj[site][tt].begin(), _mapobj[site][tt].end());
                }
                _mapobj[site].erase(itEnd, itLast);
            }
            //    itOBJ++; // WHILE LOOP ONLY
        }
        return;
    }

    void gnss_all_obs::erase(const std::string &site, const base_time &t)
    {
        double range = 30 * 60;
        hwa_map_ti_obsspt::iterator itFirst = _mapobj[site].begin();
        hwa_map_ti_obsspt::iterator itEnd = _mapobj[site].lower_bound(t - range);
        if (itEnd == itFirst)
        {
            return;
        }
        else
            itEnd--;
        if (fabs(itFirst->first.diff(t)) > range) //modified by liyuhao,only in pce
            _mapobj[site].erase(itFirst, itEnd);
        //_mapobj[site].erase(t);
    }

    void gnss_all_obs::erase(const std::string &site, const base_time &t, const std::string &sat)
    {
        _mapobj[site][t].erase(sat);
    }

    void gnss_all_obs::clear_obj()
    {
        _mapobj.clear();
        _filter.clear();
        _allepoches.clear();
        _mapcrds.clear();
        _map_sites.clear();
    }

    // return list of available satellites
    // ----------
    gnss_all_obs::hwa_map_id_obsspt gnss_all_obs::find(const std::string &site, const base_time &t)
    {
        base_time tt(base_time::GPS);
        hwa_map_id_obsspt tmp;
        if (_find_epo(site, t, tt) < 0)
        {
            return tmp;
        }

        //  std::shared_ptr<hwa_map_id_obsspt> tmp = &_mapobj[site][tt];
        return _mapobj[site][tt]; // tmp;
    }

    // return list of available satellites
    // ----------
    double gnss_all_obs::find(const std::string &site, const base_time &t, const std::string &prn, const GOBS &gobs)
    {
        double obs = 0.0;

        base_time tt(base_time::GPS);
        hwa_map_id_obsspt tmp;
        if (_find_epo(site, t, tt) < 0)
        {
            return obs;
        }

        hwa_map_id_obsspt mdata = _mapobj[site][tt];
        hwa_map_id_obsspt::iterator it = mdata.find(prn);

        if (it != mdata.end())
        {
            hwa_spt_obsmanager satdata = it->second;
            std::vector<GOBS> vobs = satdata->obs();
            for (std::vector<GOBS>::iterator it2 = vobs.begin(); it2 != vobs.end(); it2++)
            {
                if (*it2 == gobs)
                    obs = satdata->getobs(*it2);
                else
                    continue;
            }
        }
        return obs;
    }

    // get number of occurance of individual signals
    // ----------
    gnss_all_obs::hwa_map_freq gnss_all_obs::frqobs(const std::string &site)
    {
        hwa_map_freq mfrq;

        if (_mapobj.find(site) == _mapobj.end())
        {
            return mfrq;
        }

        hwa_map_ti_obsspt::const_iterator itEpo = _mapobj[site].begin();
        hwa_map_id_obsspt::iterator itSat;

        base_time t;

        while (itEpo != _mapobj[site].end())
        {
            t = itEpo->first;
            for (itSat = _mapobj[site][t].begin(); itSat != _mapobj[site][t].end(); itSat++)
            { // loop over satellites
                std::string prn = itSat->first;
                hwa_spt_obsmanager obs = itSat->second;

                std::vector<GOBS> vgobs = obs->obs();
                for (std::vector<GOBS>::iterator it = vgobs.begin(); it != vgobs.end(); ++it)
                {
                    GOBS gobs = *it;
                    int bnd = gobs2band(gobs);
                    GOBSBAND bend = int2gobsband(bnd);
                    mfrq[prn][bend][gobs]++;
                }
            }
            itEpo++;
        }
        return mfrq;
    }

    // add site-specific filtered data/epochs
    // ----------
    void gnss_all_obs::xdata(const std::string &site, const std::string &file, const hwa_stc_xfilter &xflt)
    {
        _filter[site][file].xdat = xflt.xdat;
        _filter[site][file].beg = xflt.beg;
        _filter[site][file].end = xflt.end;
    }

    // get site-specific filtered data/epochs
    // ---------
    gnss_all_obs::hwa_stc_xfilter gnss_all_obs::xdata(const std::string &site, const std::string &file)
    {
        if (_filter.find(site) != _filter.end())
        {
            if (_filter[site].find(file) != _filter[site].end())
            {
                return _filter[site][file];
            }
        }

        hwa_stc_xfilter tmp;
        tmp.xdat[XDATA_BEG] = 0;
        tmp.xdat[XDATA_END] = 0;
        tmp.xdat[XDATA_SMP] = 0;
        tmp.xdat[XDATA_SYS] = 0;

        tmp.beg = LAST_TIME;
        tmp.end = FIRST_TIME;
        return tmp;
    }

    void gnss_all_obs::setepoches(const std::string &site)
    {
        _allepoches = epochs(site);
    }

    // number of epochs for station
    // ----------
    unsigned int gnss_all_obs::nepochs(const std::string &site)
    {
        if (_mapobj.find(site) == _mapobj.end())
        {
            return 0;
        }

        unsigned int tmp = _mapobj[site].size();
        /*
  if( site == "STAN" ){
    hwa_map_ti_obsspt::iterator itEPO = _mapobj[site].begin();
    while( itEPO != _mapobj[site].end() ){
       
      base_time t = itEPO->first;
      std::cout << site << " --- time: " << t.str_ymdhms() << " " << t.dsec() << "  size: " << tmp << std::endl;
      itEPO++;
    }
  }
*/
        return tmp;
    }

    // number of epochs for station, interval and sampling, list of OBS types
    // ----------
    unsigned int gnss_all_obs::nepochs(const std::string &site, const base_time &beg, const base_time &end, double sampl, std::map<GSYS, std::pair<int, int>> &n)
    {

        if (_mapobj.find(site) == _mapobj.end())
            return 0;

        std::set<GSYS>::iterator itS;
        std::map<GSYS, std::pair<int, int>>::iterator it;
        n.clear();

        base_time tt = beg;
        while (tt < end || tt == end)
        {
            std::set<GSYS> gnss = _gsys(site, tt);
            for (itS = gnss.begin(); itS != gnss.end(); itS++)
                n[*itS].second += 1;
            //  tt = tt + sampl;
            tt.add_dsec(sampl);
        }

        for (it = n.begin(); it != n.end(); ++it)
            it->second.first = (int)floor((end - beg) / sampl + DIFF_SEC(sampl)) + 1;
        return 1;
    }

    // return true if std::map contains the site and false if not
    //-----------------------------------------------------
    bool gnss_all_obs::isSite(const std::string &site)
    {
        bool tmp = false;
        if (_mapobj.find(site) != _mapobj.end())
            tmp = true;
        return tmp;
    }

    // get all gnss_data_sats for prn from epoch interval
    // ----------------------------------------------
    std::vector<hwa_spt_obsmanager> gnss_all_obs::obs_prn_pt(const std::string &site, const std::string &prn,
                                             const base_time &beg, const base_time &end)
    {
        std::vector<hwa_spt_obsmanager> all_obs;
        base_time tt(base_time::GPS);

        if (_mapobj.find(site) == _mapobj.end())
        {
#ifdef DEBUG
            std::cout << "site not found: " << site << t.str_ymdhms() << std::endl;
            std::cout.flush();
#endif
            return all_obs;
        }

        hwa_map_ti_obsspt::iterator it1 = _mapobj[site].lower_bound(beg); // greater || equal
        hwa_map_ti_obsspt::iterator it2 = _mapobj[site].lower_bound(end); // greater || equal

        for (; it1 != it2; ++it1)
        {
            tt = it1->first;
            if (_mapobj[site].find(tt) != _mapobj[site].end())
            {
                auto itSAT = _mapobj[site][tt].find(prn);
                if (itSAT != _mapobj[site][tt].end())
                    all_obs.push_back(itSAT->second);
            }
        }
        return all_obs;
    }

    // find epoch from the std::map w.r.t. DIFF_SEC
    // ---------------------------------------
    int gnss_all_obs::_find_epo(const std::string &site, const base_time &epo, base_time &tt)
    {

        if (_mapobj.find(site) == _mapobj.end())
        {
            return -1;
        }

        hwa_map_ti_obsspt::iterator it1 = _mapobj[site].lower_bound(epo); // greater || equal
        hwa_map_ti_obsspt::iterator it0 = it1;                            // previous value

        if (it0 != _mapobj[site].begin())
            it0--; // std::set previous value
        if (it1 == _mapobj[site].end())
            it1 = it0;

        if (it1 == _mapobj[site].end() && it0 == _mapobj[site].end())
        {
            //std::cout << site << " " << epo.str_ymdhms() << " " << epo.dsec() << " no observations !\n";
            return -1;
        }

#ifdef DEBUG
        std::cout << site << " " << epo.str_ymdhms() << " " << epo.dsec() << std::fixed << std::setprecision(6)
             << " dif_sec: " << DIFF_SEC(_smp)
             << " t1_diff: " << std::setw(14) << it1->first.diff(epo)
             << " t0_diff: " << std::setw(14) << it0->first.diff(epo) << std::endl;
#endif

        if (fabs(it1->first - epo) <= DIFF_SEC(_smp))
            tt = it1->first; // it = it1;                 // std::set closest value
        else if (fabs(it0->first - epo) <= DIFF_SEC(_smp))
            tt = it0->first; // it = it0;                 // std::set closest value
        else
        {
            return -1; // not found !
        }

        return 1;
    }

    void gnss_all_obs::exclude_sat_band(const std::string &site, const std::map<GSYS, std::set<std::string>> &sats, const std::map<GSYS, GOBSBAND> &bands)
    {
        auto itEPO = _mapobj[site].begin();
        while (itEPO != _mapobj[site].end())
        {
            base_time t = itEPO->first;
            auto itSAT = itEPO->second.begin();
            while (itSAT != itEPO->second.end())
            {
                std::string sat = itSAT->first;
                GSYS gsys = itSAT->second->gsys();

                if (sats.at(gsys).size() == 0)
                {
                    itSAT++;
                    continue;
                }
                if (sats.at(gsys).find(sat) == sats.at(gsys).end())
                {
                    itSAT++;
                    continue;
                }
                std::vector<GOBS> obs_vec = itSAT->second->obs();

                for (auto it : obs_vec)
                {
                    if (str2gobsband(gobs2str(it)) == bands.at(gsys))
                        _mapobj[site][t][sat]->eraseobs(it);
                }
                itSAT++;
            }
            itEPO++;
        }
    }

} // namespace
