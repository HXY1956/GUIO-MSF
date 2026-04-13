#include "hwa_gnss_all_ambflag.h"
#include "hwa_base_time.h"
#include <algorithm>

using namespace std;

namespace hwa_gnss
{
    gnss_all_ambflag::gnss_all_ambflag()
    {
        id_type(base_data::AMBFLAG);
    }

    gnss_all_ambflag::gnss_all_ambflag(const ID_TYPE &t)
    {
        if (t != base_data::AMBFLAG && t != base_data::AMBFLAG13 && t != base_data::AMBFLAG14 && t != base_data::AMBFLAG15)
        {
            id_type(base_data::AMBFLAG);
        }
        else
        {
            id_type(t);
        }
    }
    gnss_all_ambflag::gnss_all_ambflag(base_log spdlog, ID_TYPE t) : base_data(spdlog)
    {
        if (t != base_data::AMBFLAG && t != base_data::AMBFLAG13 && t != base_data::AMBFLAG14 && t != base_data::AMBFLAG15)
        {
            id_type(base_data::AMBFLAG);
        }
        else
        {
            id_type(t);
        }
    }

    gnss_all_ambflag::~gnss_all_ambflag()
    {
        _map_allambflag.clear();
    }

    void gnss_all_ambflag::addAmbFlagHead(const string &sta, const gnss_data_ambflag_head &hd)
    {
        _map_allambflag[sta].setAmbFlagHead(hd);
        _map_allambflag[sta].setSite(sta);
    }

    void gnss_all_ambflag::addAmbFlag(const string &sta, const gnss_data_ambflag &ambflag)
    {
        _map_allambflag[sta] = ambflag;
    }

    void gnss_all_ambflag::addAmbFlagData(const string &sta, const string &sat, const gnss_data_ambflag_data &data)
    {
        _map_allambflag[sta].addAmbFlagData(sat, data);
    }

    gnss_data_ambflag &gnss_all_ambflag::getOneAmbFlag(const string &sta)
    {
        string tmp = sta;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
        if (_map_allambflag.find(tmp) != _map_allambflag.end())
        {
            return _map_allambflag[tmp];
        }
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (_map_allambflag.find(tmp) != _map_allambflag.end())
        {
            return _map_allambflag[tmp];
        }
        return _map_allambflag[tmp];
    }

    hwa_map_all_ambflag &gnss_all_ambflag::getAllAmbFlag()
    {
        return _map_allambflag;
    }

    set<string> gnss_all_ambflag::getSatList()
    {
        set<string> satlist;
        set<string> tmp;
        for (auto it_site = _map_allambflag.begin(); it_site != _map_allambflag.end(); it_site++)
        {
            tmp = it_site->second.getAllSatSet();
            satlist.insert(tmp.begin(), tmp.end());
        }
        return satlist;
    }

    set<string> gnss_all_ambflag::getSiteList()
    {
        set<string> sitelist;
        string tmp;
        for (auto it_site = _map_allambflag.begin(); it_site != _map_allambflag.end(); it_site++)
        {
            tmp = it_site->first;
            transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
            sitelist.insert(tmp);
        }
        return sitelist;
    }

} // namespace
