#include "hwa_gnss_prod.h"

namespace hwa_gnss
{
    gnss_prod::gnss_prod(const base_time &t, std::shared_ptr<gnss_data_obj> pt)
        : base_data(),
          _epo(t),
          _obj(pt)
    {
        id_group(GRP_prodUCT);
        id_type(NONE);
    }
    gnss_prod::gnss_prod(base_log spdlog, const base_time &t, std::shared_ptr<gnss_data_obj> pt)
        : base_data(spdlog),
          _epo(t),
          _obj(pt)
    {
        id_group(GRP_prodUCT);
        id_type(NONE);
    }

    // destructor
    // ----------
    gnss_prod::~gnss_prod()
    {
    }

    // std::set value + rms
    // ----------
    int gnss_prod::set_val(const std::string &str, const double &val, const double &rms)
    {
        _prod[str] = std::make_pair(val, rms);
        return 0;
    }

    // get value
    // ----------
    int gnss_prod::get_val(const std::string &str, double &val, double &rms)
    {
        if (_prod.find(str) != _prod.end())
        {
            val = _prod[str].first;
            rms = _prod[str].second;
            return 0;
        }

        return -1;
    }

    // get value
    // ----------
    int gnss_prod::get_val(const std::string &str, double &val)
    {
        double rms;
        int irc = this->get_val(str, val, rms);

        return irc;
    }

    // list id
    // ----------
    std::set<std::string> gnss_prod::list_id()
    {
        std::set<std::string> list_id;
        for (itPROD = _prod.begin(); itPROD != _prod.end(); ++itPROD)
        {
            list_id.insert(itPROD->first);
        }

        return list_id;
    }

} // namespace
