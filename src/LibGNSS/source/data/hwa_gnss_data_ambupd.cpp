#include "hwa_gnss_data_ambupd.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief default constructor. */
    gnss_data_ambupd::gnss_data_ambupd() : _ambupd()
    {

        id_type(hwa_base::base_data::AMBUPD);
        _sat_list.clear();
    }

    gnss_data_ambupd::gnss_data_ambupd(hwa_base::base_log spdlog) : hwa_base::base_data(spdlog)
    {

        id_type(hwa_base::base_data::AMBUPD);
        _sat_list.clear();
    }

    /** @brief default destructor. */
    gnss_data_ambupd::~gnss_data_ambupd()
    {
    }

    /**
    * @brief std::set the station of the class data.
    * @param[in]  sta        station name.
    * @return      void
    */
    //void  gnss_data_ambupd::set_station(std::string sta)
    //{
    //
    //    _station = sta;
    //}

    void gnss_data_ambupd::addAmbUpd(hwa_base::base_time epoch, std::string sat, const gnss_data_ambupd_batch &data)
    {

        if (_ambupd.find(epoch) == _ambupd.end())
        {
            _ambupd[epoch][sat] = std::make_shared<gnss_data_ambupd_batch>(data);
        }
        else if (_ambupd[epoch].find(sat) == _ambupd[epoch].end())
        {
            _ambupd[epoch][sat] = std::make_shared<gnss_data_ambupd_batch>(data);
        }
        else
        { //LX modify the bug from <= to ==
            if (_ambupd[epoch][sat]->amb_if == 0.0 && data.amb_if != 0)
                _ambupd[epoch][sat]->amb_if = data.amb_if;
            if (_ambupd[epoch][sat]->amb_wl == 0.0 && data.amb_wl != 0)
                _ambupd[epoch][sat]->amb_wl = data.amb_wl;
            if (_ambupd[epoch][sat]->amb_l3 == 0.0 && data.amb_l3 != 0)
                _ambupd[epoch][sat]->amb_l3 = data.amb_l3;
            if (_ambupd[epoch][sat]->amb_l4 == 0.0 && data.amb_l4 != 0)
                _ambupd[epoch][sat]->amb_l4 = data.amb_l4;
            if (_ambupd[epoch][sat]->amb_l5 == 0.0 && data.amb_l5 != 0)
                _ambupd[epoch][sat]->amb_l5 = data.amb_l5;
        }

        _sat_list.insert(sat);
    }

    std::shared_ptr<gnss_data_ambupd_batch> &gnss_data_ambupd::getSatAmbUpd(hwa_base::base_time epoch, std::string sat)
    {
        return _ambupd[epoch][sat];
    }

    hwa_map_id_ambupd &gnss_data_ambupd::getEpoAmbUpd(hwa_base::base_time epoch)
    {
        return _ambupd[epoch];
    }

    hwa_map_time_ambupd gnss_data_ambupd::getSatepoAmbUpd(hwa_base::base_time &beg, hwa_base::base_time &end, std::string sat)
    {
        hwa_map_time_ambupd satepo_ambupd;
        for (auto iter = _ambupd.upper_bound(beg); iter != _ambupd.lower_bound(end); ++iter)
        {
            auto iter_sat = iter->second.find(sat);
            if (iter_sat != iter->second.end())
                satepo_ambupd[iter->first] = iter_sat->second;
        }
        return satepo_ambupd;
    }

    hwa_map_ti_ambupd &gnss_data_ambupd::getAllAmbUpd()
    {
        return _ambupd;
    }

    hwa_base::base_time gnss_data_ambupd::getSatEndTime()
    {
        hwa_base::base_time end_time;
        end_time = _ambupd.rbegin()->first;
        return end_time;
    }

}