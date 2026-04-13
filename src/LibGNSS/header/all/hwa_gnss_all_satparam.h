/**
*
* @file            gallsatparam.h
* @brief        Storage the satellites parameters data
*
*/

#ifndef GALLSATPARAM_H
#define GALLSATPARAM_H

#include "hwa_base_data.h"
#include "hwa_gnss_data_satparam.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for all satellites parameters data storaging
    */
    class gnss_all_satinfo : public base_data
    {
    public:
        /** @brief default constructor. */
        explicit gnss_all_satinfo();
        explicit gnss_all_satinfo(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_satinfo();

        /**
        * @brief push one satellite parameters data into the data map.
        * @param[in]  prn        satellite name.
        * @param[in]  svn        satellite svn.
        * @param[in]  start        launched time of the satellite.
        * @param[in]  end        decommissioned time of the satellite.
        * @param[in]  cosparid    cospar_id of the satellite.
        * @param[in]  mass        mass of the satellite.
        * @param[in]  maxyaw    max_yaw of the satellite.
        * @param[in]  fid        frequence id of the satellite.
        * @param[in]  LRA[3]    LRA COM correction
        * @param[in]  lratype   LRA type
        * @param[in]  LPOWER    antenna transmit power
        * @param[in]  blocktype    BLOCK-TYPE of the satellite.
        * @return    whether exit normally
        */
        int addData(const std::string &sat_prn, const std::string &sat_svn,
                    const base_time &beg_time, const base_time &end_end,
                    const std::string &cosparid, const double &sat_mass, const double &sat_max_yaw,
                    const int &sat_fid, double lra[], const int &lratype, const double &lpower,
                    const std::string &block_type);

        /** @brief whether the satellite parameters data is empty.
        * @return  bool
        *    @retval   true    satellite parameters data is empty
        *   @retval   false   satellite parameters data is existent
        */
        bool isEmpty();

        /**
        * @brief get valid satellite parameters data.
        *
        * Get paramter data in corresponding time of one satellite from the data map
        *
        * @param[in]  sat            satellite name.
        * @param[in]  ics_start        begin time from ics data.
        * @param[in]  ics_end        end time from ics data.
        * @param[out] sat_param        corresponding satellite parameters data.
        * @return    if find the correct satellite parameters data.
        *     @retval true    get data successfully
        *     @retval false    get data failed
        */
        bool getSatinfoUsed(const std::string &sat, const base_time &beg_time, const base_time &end_time, gnss_data_satparam &satinfo);

        /**
        * @brief get valid glonass frequency id.
        * @param[in]  sat            satellite name.
        * @param[in]  epoch            time from sat.
        * @param[out] fid            frequency id.
        */
        int getGloFid(const std::string &sat, const base_time &epoch);

    protected:
        // fix the bug by yqyuan 2021.11.22
        //std::map<std::string, std::map<std::string, gnss_data_satparam>> _allsatinfo; ///< map for all satellites parameters data(prn,svn,record)
        std::map<std::string, std::map<std::string, std::vector<gnss_data_satparam>>> _allsatinfo; ///< map for all satellites parameters data(prn,svn,record)
    };

}
#endif // !GALLSATPARAM_H