/**
*
* @file        gambflag.h
* @brief    Storage the XXXXddd0.yyo.log/log13 files' data(only one site)
*                XXXX  ---- SITE name
*                 ddd  ---- Doy of the file
*                  yy  ---- year
*/

#ifndef hwa_gnss_data_ambflag_H
#define hwa_gnss_data_ambflag_H

#include <stdio.h>
#include <string>
#include "hwa_gnss_data_obj.h"
#include "hwa_base_data.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /**
    *@brief       Class for one XXXXddd0.yyo.log/log13 file head data storaging
    */
    class gnss_data_ambflag_head
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambflag_head(){};

        /** @brief default destructor. */
        virtual ~gnss_data_ambflag_head(){};

        int beg_mjd = 0;      ///< start time mjd
        double beg_sod = 0.0; ///< start time sod
        int duration = 0;     ///< duration
        double intv = 30.0;   ///< sampling interval
        int max_amb_1epo = 0; ///< max ambc in one epoch
        int old_rm_obs = 0;   ///< old remvoed observations
        int new_rm_obs = 0;   ///< new removed observations
        int exist_amb = 0;    ///< existed ambiguities
        int new_amb = 0;      ///< new ambiguities
        int avaiable_obs = 0; ///< available observations

        std::string rec_name;               ///< recevier  name
        std::set<std::string> sys_name;          ///< system    name
        std::set<std::string> sat_name;          ///< satellite name
        std::map<std::string, std::string> freq_name; ///< get freq by band
        std::map<std::string, std::string> band_name; ///< get band by freq
    };

    /**
    * @brief       Class for one XXXXddd0.yyo.log/log13 file one satellite data storaging
    */
    class gnss_data_ambflag_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambflag_data(){};

        /** @brief default destructor. */
        virtual ~gnss_data_ambflag_data(){};

        std::string identify; ///< identification including AMB,DEL and BAD
        int beg_epo = 0; ///< begin epoch
        int end_epo = 0; ///< end epoch
        int nmb_arc = 0; ///< number of arc
        std::string iflag;    ///< flag for compute
        double C1 = 0.0; ///< coefficient C1
        double C2 = 0.0; ///< coefficient C2
        std::string reason;   ///< cause of ambiguity
        std::string log_type; ///< log type : L12 L13 L14 L15 or more
    };

    /** @brief vector container for storaging ambflag data ptr satellite/one station/all epoch-arc in a log file */
    typedef std::vector<std::shared_ptr<gnss_data_ambflag_data>> hwa_vector_ambflag;

    /** @brief map container using satellite name as a index for storaging hwa_vector_ambflag data of one station */
    typedef std::map<std::string, hwa_vector_ambflag> hwa_map_sat_ambflag;

    /**
    *@brief       Class for one XXXXddd0.yyo.log/log13 file data storaging
    */
    class gnss_data_ambflag : public base_data
    {

    public:
        /** @brief default constructor. */
        gnss_data_ambflag();

        gnss_data_ambflag(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_ambflag();

        /**
        * @brief set the file's belonging, it's station name.
        * @param[in] site site name
        */
        void setSite(const std::string &site) { _site = site; }

        /**
        * @brief add head data of ambflag file.
        * @param[in]  head        ambflag file head data.
        * @return      void
        */
        void setAmbFlagHead(const gnss_data_ambflag_head &head);

        /**
        * @brief reset the iflag of a data of one station/one satellite/one epoch-arc.
        * @param[in]  prn        satellite name.
        * @param[in]  flag        iflag in class gnss_data_ambflag_data.
        * @param[in]  pos        the serial number of epoch-arc where current epoch in.
        * @return      void
        */
        void reset_iflag(const std::string &prn, const std::string &flag, const int &pos);

        /**
        * @brief get ambflag head data of one file(one station datum's head).
        * @return ambflag head data
        */
        std::shared_ptr<gnss_data_ambflag_head> &getAmbFlagHead();

        /**
        * @brief get hwa_map_sat_ambflag data of one station/one satellite.
        * @return     hwa_map_sat_ambflag data of one station/one satellite
        */
        hwa_map_sat_ambflag &getAmbFlagData();

        /**
        * @brief get hwa_vector_ambflag data of one satellite.
        * @param[in]  sat        satellite name.
        * @return      hwa_vector_ambflag data of one satellite.
        */
        hwa_vector_ambflag &getSatAmbFlag(const std::string &sat);

        /**
        * @brief add ambflag data of one station/one satellite/one epoch-arc.
        * @param[in]  prn        satellite name.
        * @param[in]  data        ambflag data of one station/one satellite/one epoch-arc.
        * @return      void
        */
        void addAmbFlagData(const std::string &prn, const gnss_data_ambflag_data &data);

        /**
        * @brief judge if an epoch-arc data is usable.
        * @param[in]  prn        satellite name.
        * @param[in]  time        current epoch.
        * @param[in]  pos        the serial number of epoch-arc where current epoch in.
        * @param[out] pos
        * @return      true is available/ flase not available
        */
        bool isValid(const std::string &prn, const base_time &time, int &pos);

        /**
        * @brief judge if carrier range.
        * @param[in]  prn        satellite name.
        * @param[in]  time        current epoch.
        * @return      true is available/ flase not available
        */
        bool is_carrier_range(const std::string &prn, const base_time &time);

        /**
        * @brief judge if carrier range.
        * @param[in]   prn        satellite name.
        * @param[in]   time        current epoch.
        * @param[out]  c1        carrier_range.
        * @param[out]  c2        carrier_range.
        * @return      true is available/ flase not available
        */
        bool is_carrier_range(const std::string &prn, const base_time &time, double &c1, double &c2);

        /**
        * @brief get amb pos in log file
        * @param[in]   prn                        satellite name.
        * @param[in]   time                        current epoch.
        * @param[in]   apply_carrier_range        is apply carrier_range.
        * @return       amb pos
        */
        int get_amb_pos(const std::string &prn, const base_time &time, bool apply_carrier_range);

        /**
        * @brief translate amb epoch in log file to base_time
        * @param[in]   epoch                amb epoch in log file.
        * @return       base_time for epoch
        */
        base_time epoch2time(const int &epo);
        /**
         * @brief Get the All Sat Set
         * @return set<string> Satellite names
         */
        std::set<std::string> getAllSatSet();
        /**
         * @brief Get the Max End Epoch
         * @return base_time Max end of epoch
         */
        base_time getMaxEndEpoch();
        /**
         * @brief clear all ambflag data
         */
        void clearAllAmbFlagData();

    protected:
        std::string _site;                         ///< site name
        std::shared_ptr<gnss_data_ambflag_head> _ambflag_head; ///< ptr of ambflag head which storaging log file head of one station
        hwa_map_sat_ambflag _all_sats_ambflag;  ///< map container using satellite name as a index for storaging
                                              ///< hwa_vector_ambflag data of one station
    private:
    };

} // namespace

#endif