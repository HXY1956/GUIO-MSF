#include "hwa_gnss_data_ambflag.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_ambflag::gnss_data_ambflag() //:_gnss_data_ambflag_data()
    {
        id_type(base_data::AMBFLAG);
    }
    gnss_data_ambflag::gnss_data_ambflag(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::AMBFLAG);
    }

    /** @brief default destructor. */
    gnss_data_ambflag::~gnss_data_ambflag()
    {
    }

    /**
    * @brief add head data of ambflag file.
    * @param[in]  head        ambflag file head data.
    * @return      void
    */
    void gnss_data_ambflag::setAmbFlagHead(const gnss_data_ambflag_head &head)
    {

        _ambflag_head = std::make_shared<gnss_data_ambflag_head>(head);
    }

    shared_ptr<gnss_data_ambflag_head> &gnss_data_ambflag::getAmbFlagHead()
    {
        return _ambflag_head;
    }

    void gnss_data_ambflag::addAmbFlagData(const string &prn, const gnss_data_ambflag_data &data)
    {

        try
        {
            _all_sats_ambflag[prn].push_back(std::make_shared<gnss_data_ambflag_data>(data));
        }
        catch (exception ex)
        {
            std::cout << ex.what();
        }
    }

    hwa_map_sat_ambflag &gnss_data_ambflag::getAmbFlagData()
    {
        return _all_sats_ambflag;
    }

    hwa_vector_ambflag &gnss_data_ambflag::getSatAmbFlag(const string &sat)
    {
        return _all_sats_ambflag[sat];
    }

    bool gnss_data_ambflag::isValid(const string &prn, const base_time &time, int &pos)
    {
        try
        {
            pos = 0;
            if (_all_sats_ambflag.find(prn) == _all_sats_ambflag.end())
                return false;

            bool is_found = false;
            for (auto iter = _all_sats_ambflag[prn].begin(); iter != _all_sats_ambflag[prn].end(); ++iter)
            {
                if (time >= epoch2time((*iter)->beg_epo) && time <= epoch2time((*iter)->end_epo))
                {
                    if ((*iter)->identify == "BAD" || (*iter)->identify == "DEL")
                    {
                        return false;
                    }
                    else
                    {
                        is_found = true;
                        pos = distance(_all_sats_ambflag[prn].begin(), iter);
                        //break;
                    }
                }
            }
            return is_found;
            //base_time beg;
            //base_time end;

            // find in unused
            //for (auto iter = _all_sats_ambflag[prn].begin(); iter != _all_sats_ambflag[prn].end(); iter++)
            //{
            //    if ((*iter)->identify == "BAD" || (*iter)->identify == "DEL") {
            //        //beg.from_mjd(_ambflag_head->beg_mjd, _ambflag_head->beg_sod + _ambflag_head->intv*((*iter)->beg_epo - 1), 0.0);
            //        //end.from_mjd(_ambflag_head->beg_mjd, _ambflag_head->beg_sod + _ambflag_head->intv*((*iter)->end_epo - 1), 0.0);
            //        if (time >= epoch2time((*iter)->beg_epo) && time <= epoch2time((*iter)->end_epo)) {
            //            return false;
            //        }
            //    }
            //}

            //// find in used
            //for (auto iter = _all_sats_ambflag[prn].begin(); iter != _all_sats_ambflag[prn].end(); iter++)
            //{
            //    if ((*iter)->identify == "AMB") {
            //        //beg.from_mjd(_ambflag_head->beg_mjd, _ambflag_head->beg_sod + _ambflag_head->intv*((*iter)->beg_epo - 1), 0.0);
            //        //end.from_mjd(_ambflag_head->beg_mjd, _ambflag_head->beg_sod + _ambflag_head->intv*((*iter)->end_epo - 1), 0.0);
            //        if (time >= epoch2time((*iter)->beg_epo) && time <= epoch2time((*iter)->end_epo)) {
            //            pos = distance(_all_sats_ambflag[prn].begin(), iter);
            //            return true;
            //        }
            //    }
            //}
            //return false;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown mistake");
            return false;
        }
    }

    bool gnss_data_ambflag::is_carrier_range(const string &prn, const base_time &time)
    {
        if (_all_sats_ambflag.find(prn) == _all_sats_ambflag.end())
            return false;

        for (auto iter = _all_sats_ambflag[prn].begin(); iter != _all_sats_ambflag[prn].end(); ++iter)
        {
            if (time >= epoch2time((*iter)->beg_epo) && time <= epoch2time((*iter)->end_epo))
            {
                return (*iter)->identify == "IAM" ? true : false;
            }
        }
        return false;
    }

    bool gnss_data_ambflag::is_carrier_range(const string &prn, const base_time &time, double &c1, double &c2)
    {
        c1 = 0;
        c2 = 0;
        if (_all_sats_ambflag.find(prn) == _all_sats_ambflag.end())
            return false;

        for (auto iter = _all_sats_ambflag[prn].begin(); iter != _all_sats_ambflag[prn].end(); ++iter)
        {
            if (time >= epoch2time((*iter)->beg_epo) && time <= epoch2time((*iter)->end_epo))
            {
                if ((*iter)->identify != "IAM")
                {
                    return false;
                }
                else
                {
                    c1 = (*iter)->C1;
                    c2 = (*iter)->C2;
                    return true;
                }
            }
        }
        return false;
    }

    int gnss_data_ambflag::get_amb_pos(const string &prn, const base_time &time, bool apply_carrier_range)
    {
        if (_all_sats_ambflag.find(prn) == _all_sats_ambflag.end())
            return -1;

        int pos = -1;
        for (auto iter = _all_sats_ambflag[prn].begin(); iter != _all_sats_ambflag[prn].end(); ++iter)
        {
            if (time >= epoch2time((*iter)->beg_epo) && time <= epoch2time((*iter)->end_epo))
            {
                if ((*iter)->identify == "BAD" || (*iter)->identify == "DEL")
                {
                    return -1;
                }
                else if ((*iter)->identify == "IAM")
                {
                    if (!apply_carrier_range)
                    {
                        pos = distance(_all_sats_ambflag[prn].begin(), iter);
                        break;
                    }
                    else
                    {
                        return -1;
                    }
                }
                else
                {
                    pos = distance(_all_sats_ambflag[prn].begin(), iter);
                    break;
                }
            }
        }
        return pos;
    }

    base_time gnss_data_ambflag::epoch2time(const int &epo)
    {
        //base_time time;
        //time.from_mjd(_ambflag_head->beg_mjd, _ambflag_head->beg_sod + _ambflag_head->intv * (epo - 1), 0.0);
        base_time time(_ambflag_head->beg_mjd, _ambflag_head->beg_sod + _ambflag_head->intv * (epo - 1), 0.0);

        return time;
    }

    void gnss_data_ambflag::reset_iflag(const string &prn, const string &flag, const int &pos)
    {
        _all_sats_ambflag[prn][pos]->iflag = flag;
    }

    set<string> gnss_data_ambflag::getAllSatSet()
    {
        set<string> satlist;
        auto it_sat = _all_sats_ambflag.begin();
        for (; it_sat != _all_sats_ambflag.end(); it_sat++)
        {
            satlist.insert(it_sat->first);
        }
        return satlist;
    }

    base_time gnss_data_ambflag::getMaxEndEpoch()
    {
        int max_end_epo = 0;
        for (auto sat_record : _all_sats_ambflag)
        {
            for (auto record : sat_record.second)
            {
                if (record->end_epo > max_end_epo)
                {
                    max_end_epo = record->end_epo;
                }
            }
        }
        base_time beg_time(this->_ambflag_head->beg_mjd, int(this->_ambflag_head->beg_sod), this->_ambflag_head->beg_sod - int(this->_ambflag_head->beg_sod));
        base_time end_time = beg_time + (max_end_epo - 1) * this->_ambflag_head->intv;
        return end_time;
    }

    void gnss_data_ambflag::clearAllAmbFlagData()
    {
        _all_sats_ambflag.clear();
    }

} //namespace