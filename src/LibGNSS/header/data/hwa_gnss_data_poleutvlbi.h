/**
 *
 * @file            gpoleut1.h
 * @brief            The class for storaging poleut1 data.
 */

#ifndef hwa_gnss_data_poleutVLBI_H
#define hwa_gnss_data_poleutVLBI_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_poleut.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging poleut1 file data.
    */
    class gnss_data_poleut_vlbi : public base_data
    {
    public:
        /// constructor.
        gnss_data_poleut_vlbi();
        gnss_data_poleut_vlbi(base_log spdlog);
        /// destructor.
        virtual ~gnss_data_poleut_vlbi(){};

        /**
        *@brief      the map stored the record in the poleut1 file
        */
        typedef std::map<base_time, std::map<std::string, double>> hwa_map_tiv;

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
        * @brief whether the poleut1 data is empty
        * @return
            @retval true the poleut1 data is empty
            @retval false the poleut1 data is existent
        */
        bool isEmpty();

        /**
        * @brief return the map of record.
        * @return  the map of record
        */
        hwa_map_tiv *getPoleUt1DataMap() { return &_poleut1_data; };

        /**
        * @brief return begin time of data in the poleut1 file.
        * @return  begin time
        */
        int getBegTime() { return _beg_time; };

        /**
        * @brief return end time of data in the poleut1 file.
        * @return  end time
        */
        int getEndTime() { return _end_end; };

        /**
        * @brief return mode of ut1.
        * @return  mode of ut1
        */
        std::string getUt1Mode() { return _UT1_mode; };

        /**
        * @brief return interval of poleut1 data
        * @return  interval
        */
        double getIntv() { return _intv; };

    protected:
        hwa_map_tiv _poleut1_data; ///< map of pole and ut1 data.
        std::string _UT1_mode;        ///< UT1 type.
        int _beg_time;           ///< begin time.
        int _end_end;            ///< end time.
        double _intv;            ///< interval.
    };
} //namespace

#endif