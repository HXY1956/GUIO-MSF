#include "hwa_gnss_data_poleutvlbi.h"
using namespace std;

namespace hwa_gnss
{
    /** @brief constructor. */
    gnss_data_poleut_vlbi::gnss_data_poleut_vlbi()
        : base_data()
    {
        id_type(base_data::ALLPOLEUT1_VLBI);
    }

    gnss_data_poleut_vlbi::gnss_data_poleut_vlbi(base_log spdlog)
        : base_data(spdlog)
    {
        id_type(base_data::ALLPOLEUT1_VLBI);
    }
    /**
    * @brief set begin and end time for poleut1 data.
    * @param[in]   beg        begin time of the poleut1 data
    * @param[in]   end        end time of the poleut1 data
    */
    void gnss_data_poleut_vlbi::setBegEndTime(int beg, int end)
    {
        _beg_time = beg;
        _end_end = end;
    }

    /**
    * @brief add poleut1 data.
    * @details add one poleut1 data record to _poleut1data and set timetype and interval.
    *
    * @param[in]   mjdtime        (mjd)time
    * @param[in]   data            one poleut1 data record(xpole,ypole and so on)
    * @param[in]   mode            type of UT1(such as UT1R)
    * @param[in]   intv            interal(unit:day)
    */
    void gnss_data_poleut_vlbi::setEopData(base_time mjdtime, map<string, double> data, string mode, double intv)
    {
        _poleut1_data[mjdtime] = data;
        _UT1_mode = mode;
        _intv = intv;
    }

    /**
    * @brief get corresponding poleut1 data by time.
    * @param[in]   mjdtime        (mjd)time
    * @param[out]   data            one poleut1 data record(xpole,ypole and so on)
    */
    void gnss_data_poleut_vlbi::getEopData(base_time mjdtime, map<string, double> &data)
    {
        if (_poleut1_data.find(mjdtime) != _poleut1_data.end())
        {
            data = _poleut1_data[mjdtime];
        }
        // interpolation
        else
        {
            auto iter_right = _poleut1_data.upper_bound(mjdtime);
            if (iter_right == _poleut1_data.begin() || iter_right == _poleut1_data.end())
            {
                std::cout << "interpolation poleut1 wrong!!" << endl;
                throw runtime_error("interpolation poleut1 wrong!!");
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

    /** @brief whether the poleut1 data is empty.
    * @return  bool
    *    @retval   true   poleut1 data is empty
    *   @retval   false  poleut1 data is existent
    */
    bool gnss_data_poleut_vlbi::isEmpty()
    {
        if (_poleut1_data.size() == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}