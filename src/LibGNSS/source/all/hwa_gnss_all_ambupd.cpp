#include "hwa_gnss_all_ambupd.h"

namespace hwa_gnss
{

    gnss_all_ambupd::gnss_all_ambupd() : _allambupd(), base_data()
    {
        id_type(base_data::AMBUPD);
    }

    gnss_all_ambupd::gnss_all_ambupd(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::AMBUPD);
    }

    gnss_all_ambupd::~gnss_all_ambupd()
    {
        _allambupd.clear();
    }

    void gnss_all_ambupd::addAmbUpd(const std::string &sta, const gnss_data_ambupd &ambupd)
    {
        _allambupd[sta] = ambupd;
    }

    void gnss_all_ambupd::addAmbUpd(const std::string &sta, const base_time &epoch, const std::string &sat, const gnss_data_ambupd_batch &data)
    {
        _allambupd[sta].addAmbUpd(epoch, sat, data);
    }

    gnss_data_ambupd &gnss_all_ambupd::getOneSiteAmbUpd(const std::string &sta)
    {

        return _allambupd[sta];
    }

    hwa_map_id_ambupd &gnss_all_ambupd::getOneSiteOneEpoAmbUpd(const std::string &sta, const base_time &t)
    {
        return _allambupd[sta].getEpoAmbUpd(t);
    }

    std::map<std::string, gnss_data_ambupd> &gnss_all_ambupd::getAllAmbUpd()
    {
        return _allambupd;
    }

    std::shared_ptr<gnss_data_ambupd_batch> &gnss_all_ambupd::getOneSiteOneEpoOneSatAmbUpd(const base_time &epoch, const std::string &sta, const std::string &sat)
    {
        return _allambupd[sta].getSatAmbUpd(epoch, sat);
    }

    bool gnss_all_ambupd::isAmbUpdValid(const base_time &epoch, const std::string &sta, const std::string &sat)
    {
        if (_allambupd[sta].getSatAmbUpd(epoch, sat) == NULL)
            return false;
        else
            return true;
    }

    std::set<std::string> gnss_all_ambupd::get_sites()
    {
        std::set<std::string> sites;
        for (auto it = _allambupd.begin(); it != _allambupd.end(); it++)
        {
            sites.insert(it->first);
        }
        return sites;
    }

    std::set<std::string> gnss_all_ambupd::get_sats()
    {
        std::set<std::string> tmp, sat_list;
        for (auto it = _allambupd.begin(); it != _allambupd.end(); it++)
        {
            tmp = it->second.getSatList();
            sat_list.insert(tmp.begin(), tmp.end());
        }
        return sat_list;
    }

    std::map<std::string, base_time> gnss_all_ambupd::getSitesEndRecord()
    {
        std::map<std::string, base_time> site_end;
        for (auto it = _allambupd.begin(); it != _allambupd.end(); it++)
        {
            site_end[it->first] = it->second.getSatEndTime();
        }
        return site_end;
    }
}