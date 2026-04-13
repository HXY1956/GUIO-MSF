#include "hwa_gnss_data_poleut.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{

    /** @brief constructor. */
    gnss_data_poleut::gnss_data_poleut()
        : base_data()
    {
        id_type(base_data::ALLPOLEUT1);
    }
    gnss_data_poleut::gnss_data_poleut(base_log spdlog)
        : base_data(spdlog)
    {
        id_type(base_data::ALLPOLEUT1);
    }

    /**
    * @brief set begin and end time for poleut data.
    * @param[in]   beg        begin time of the poleut data
    * @param[in]   end        end time of the poleut data
    */
    void gnss_data_poleut::setBegEndTime(int beg, int end)
    {
        _beg_time = beg;
        _end_end = end;
    }

    /**
    * @brief add poleut data.
    * @details add one poleut data record to _poleutdata and set timetype and interval.
    *
    * @param[in]   mjdtime        (mjd)time
    * @param[in]   data            one poleut data record(xpole,ypole and so on)
    * @param[in]   mode            type of UT1(such as UT1R)
    * @param[in]   intv            interal(unit:day)
    */
    void gnss_data_poleut::setEopData(base_time mjdtime, map<string, double> data, string mode, double intv)
    {
        _poleut_data[mjdtime] = data;
        _UT1_mode = mode;
        _intv = intv;
    }

    /**
    * @brief get corresponding poleut data by time.
    * @param[in]   mjdtime        (mjd)time
    * @param[out]   data            one poleut data record(xpole,ypole and so on)
    */
    void gnss_data_poleut::getEopData(base_time mjdtime, map<string, double> &data)
    {
        if (_poleut_data.find(mjdtime) != _poleut_data.end())
        {
            data = _poleut_data[mjdtime];
        }
        // interpolation
        else
        {
            auto iter_right = _poleut_data.upper_bound(mjdtime);
            if (iter_right == _poleut_data.begin() || iter_right == _poleut_data.end())
            {
                std::cout << "interpolation poleut wrong!!" << endl;
                throw runtime_error("interpolation poleut wrong!!");
            }
            auto iter_left = iter_right--;

            double alpha = (mjdtime - iter_left->first) / (iter_right->first - iter_left->first);
            data.clear();
            for (auto iter = iter_left->second.begin(); iter != iter_left->second.end(); iter++)
            {
                double temp = iter_left->second[iter->first] + alpha * (iter_right->second[iter->first] - iter_left->second[iter->first]);
                data.insert(std::make_pair(iter->first, temp));
            }
        }
    }

    /** @brief whether the poleut data is empty.
    * @return  bool
    *    @retval   true   poleut data is empty
    *   @retval   false  poleut data is existent
    */
    bool gnss_data_poleut::isEmpty()
    {
        if (_poleut_data.size() == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief get data for given time by interpolating.
     * @param[in]    time   the time
     * @param[out]   data   store the record corrsponding to the mjdtime
     */
    void gnss_data_poleut::reset_eop_data(const base_time &time, map<string, double> &data)
    {
        for (const auto &iter : data)
        {
            if (double_eq(iter.second, 0.0))
            {
                return;
            }
        }
        for (const auto &iter : data)
        {
            _poleut_data[time][iter.first] = iter.second;
        }
    }

    /**
     * @brief get data for given time by interpolating.
     * @param[in]    tdt   the tdt time
     * @param[out]   time  the time for data
     * @param[out]   data  store the record corrsponding to the time
     */
    void gnss_data_poleut::get_pred_poleut_data(const base_time &tdt, base_time &time, map<string, double> &data)
    {
        map<base_time, map<string, double>>::iterator find_iter = _poleut_data.lower_bound(tdt);

        if (find_iter == _poleut_data.end() ||
            find_iter == _poleut_data.begin())
        {
            throw runtime_error(tdt.str_mjdsod("can not get poleut data"));
        }
        else
        {
            --find_iter;
            time = (*(find_iter)).first;
            data = (*(find_iter)).second;
        }
    }

    /**
     * @brief get data for given time by interpolating.
     * @param[in]    tdt   the tdt time
     * @param[out]   time  the time for data
     * @param[out]   data  store the record corrsponding to the time
     */
    void gnss_data_poleut::get_post_poleut_data(const base_time &tdt, base_time &time, map<string, double> &data)
    {
        map<base_time, map<string, double>>::iterator find_iter = _poleut_data.lower_bound(tdt);

        if (find_iter == _poleut_data.end() ||
            find_iter == _poleut_data.begin())
        {
            throw runtime_error(tdt.str_mjdsod("can not get poleut data"));
        }
        else
        {
            time = (*(find_iter)).first;
            data = (*(find_iter)).second;
        }
    }

    /**
     * @brief get data for given time by interpolating.
     * @param[in]    beg  the beg time
     * @param[in]    end  the end time
     * @param[out]   data  store the record corrsponding to the time
     */
    void gnss_data_poleut::get_list_poleut_data(const base_time &beg, const base_time &end, hwa_map_gnss_TIME_id_VALUE &data_list)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "get_list_poleut_data", "get_list_poleut_data");
        for (const auto &data : _poleut_data)
        {
            if (data.first >= beg && data.first <= end)
            {
                data_list.insert(std::make_pair(data.first, data.second));
            }
        }
    }

    /**
     * @brief get data for given time by interpolating.
     * @param[in]   beg       the beg time
     * @param[in]   end       the end time
     * @param[in]   beg_eop   eop data for beg time
     * @param[in]   end_eop   eop data for end time
     * @param[in]   data_list interpolating data
     * @param[out]  result    store the record corrsponding to the time
     */
    void gnss_data_poleut::get_inte_poleut_data(const base_time &beg,
                                           const base_time &end,
                                           const map<string, double> &beg_eop,
                                           const map<string, double> &end_eop,
                                           const hwa_map_gnss_TIME_id_VALUE &data_list,
                                           hwa_map_gnss_TIME_id_VALUE &result)
    {
        double intv = end.dmjd() - beg.dmjd();
        for (const auto &data : data_list)
        {
            double diff_beg = data.first.dmjd() - beg.dmjd();
            double diff_end = data.first.dmjd() - end.dmjd();

            double diff = 0.0;
            map<string, double> ref_dat;
            if (fabs(diff_beg) > fabs(diff_end))
            {
                diff = diff_end;
                ref_dat = end_eop;
            }
            else
            {
                diff = diff_beg;
                ref_dat = beg_eop;
            }

            map<string, double> inter_data;
            double xpole = ref_dat["XPOLE"];
            double ypole = ref_dat["YPOLE"];
            double dxpole = ref_dat["DXPOLE"];
            double dypole = ref_dat["DYPOLE"];
            double ut1 = ref_dat["UT1"];
            double dut1 = ref_dat["DUT1"];

            result[data.first]["XPOLE"] = xpole + diff / intv * dxpole;
            result[data.first]["YPOLE"] = ypole + diff / intv * dypole;
            result[data.first]["UT1"] = ut1 + diff / intv * dut1;
        }
    }
}