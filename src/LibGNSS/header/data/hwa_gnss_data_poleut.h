/**
 * @file            gpoleut.h
 * @brief            The class for storaging poleut data.
 */

#ifndef hwa_gnss_data_poleut1_H
#define hwa_gnss_data_poleut1_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging poleut file data.
    */
    class gnss_data_poleut : public base_data
    {
    public:
        /// constructor.
        gnss_data_poleut();
        gnss_data_poleut(base_log spdlog);
        /// destructor.
        virtual ~gnss_data_poleut(){};

        /**
        *@brief      the map stored the record in the poleut file
        */
        typedef std::map<base_time, std::map<std::string, double>> hwa_map_gnss_TIME_id_VALUE;

        /**
         * @brief add begin and end time of file data.
         * @param[in]   beg     the begin time of data
         * @param[in]   end     the end time of data
         */
        void setBegEndTime(int beg, int end);

        /**
         * @brief add data for map of pole and ut1 data.
         * @param[in]   mjdtime   the time of data to store
         * @param[in]   data      the data record
         * @param[in]   mode      the time mode
         * @param[in]   intv      the interval of data
         */
        void setEopData(base_time mjdtime, std::map<std::string, double> data, std::string mode, double intv);

        /**
         * @brief get data for given time by interpolating.
         * @param[in]   mjdtime   the time to interpolate
         * @param[out]   data      store the record corrsponding to the mjdtime
         */
        void getEopData(base_time mjdtime, std::map<std::string, double> &data);

        /**
        * @brief whether the poleut data is empty
        * @return  
            @retval true the poleut data is empty
            @retval false the poleut data is existent
        */
        bool isEmpty();

        /**
        * @brief return the map of record.
        * @return  the map of record
        */
        hwa_map_gnss_TIME_id_VALUE *getPoleUt1DataMap() { return &_poleut_data; };

        /**
        * @brief return begin time of data in the poleut file.
        * @return  begin time
        */
        int getBegTime() { return _beg_time; };

        /**
        * @brief return end time of data in the poleut file.
        * @return  end time
        */
        int getEndTime() { return _end_end; };

        /**
        * @brief return mode of ut1.
        * @return  mode of ut1
        */
        std::string getUt1Mode() { return _UT1_mode; };

        /**
        * @brief return interval of poleut data
        * @return  interval
        */
        double getIntv() { return _intv; };

        /**
         * @brief get data for given time by interpolating.
         * @param[in]    time   the time
         * @param[out]   data   store the record corrsponding to the mjdtime
         */
        void reset_eop_data(const base_time &time, std::map<std::string, double> &data);

        /**
         * @brief get data for given time by interpolating.
         * @param[in]    tdt   the tdt time
         * @param[out]   time  the time for data
         * @param[out]   data  store the record corrsponding to the time
         */
        void get_pred_poleut_data(const base_time &tdt, base_time &time, std::map<std::string, double> &data);

        /**
         * @brief get data for given time by interpolating.
         * @param[in]    tdt   the tdt time
         * @param[out]   time  the time for data
         * @param[out]   data  store the record corrsponding to the time
         */
        void get_post_poleut_data(const base_time &tdt, base_time &time, std::map<std::string, double> &data);

        /**
         * @brief get data for given time by interpolating.
         * @param[in]    beg  the beg time
         * @param[in]    end  the end time
         * @param[out]   data  store the record corrsponding to the time
         */
        void get_list_poleut_data(const base_time &beg, const base_time &end, hwa_map_gnss_TIME_id_VALUE &data_list);

        /**
         * @brief get data for given time by interpolating.
         * @param[in]   beg       the beg time
         * @param[in]   end       the end time
         * @param[in]   beg_eop   eop data for beg time
         * @param[in]   end_eop   eop data for end time
         * @param[in]   data_list interpolating data
         * @param[out]  result    store the record corrsponding to the time
         */
        void get_inte_poleut_data(const base_time &beg,
                                   const base_time &end,
                                   const std::map<std::string, double> &beg_eop,
                                   const std::map<std::string, double> &end_eop,
                                   const hwa_map_gnss_TIME_id_VALUE &data_list,
                                   hwa_map_gnss_TIME_id_VALUE &result);

    protected:
        hwa_map_gnss_TIME_id_VALUE _poleut_data; ///< map of pole and ut1 data.
        std::string _UT1_mode;        ///< UT1 type.
        int _beg_time;           ///< begin time.
        int _end_end;            ///< end time.
        double _intv;            ///< interval.
    };
} //namespace

#endif