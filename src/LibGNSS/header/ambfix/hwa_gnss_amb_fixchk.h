#ifndef hwa_gnss_amb_CHK_H
#define hwa_gnss_amb_CHK_H

#include "hwa_set_amb.h"
#include "hwa_gnss_amb_fix.h"
#include "hwa_gnss_data_ambcon.h"
#include "hwa_gnss_all_recover.h"

namespace hwa_gnss
{
    class gnss_amb_fixchk
    {
    public:
        /**
         * @brief Construct a new t gambchk object
         */
        explicit gnss_amb_fixchk();
        /**
         * @brief Construct a new t gambchk object
         * @param[in]  spdlog       logbase control
         * @param[in]  set       setbase control
         */
        explicit gnss_amb_fixchk(base_log spdlog, set_base *set);
        /**
         * @brief Destroy the t gambchk object
         */
        virtual ~gnss_amb_fixchk();

        /**
         * @brief batch process for ambfix check
         */
        bool ProcessBatch();
        /**
         * @brief add data
         * @param[in]  res1      residuals1
         * @param[in]  res2      residuals2
         * @param[in]  ambcon    ambiguity
         */
        bool add_data(gnss_all_recover *res1, gnss_all_recover *res2, gnss_data_ambcon *ambcon);

    protected:
        /**
         * @brief Get the setting
         */
        bool get_setting();
        /**
         * @brief write ambiguity data
         */
        void write_ambcon();

    private:
        base_log _spdlog = nullptr;          ///< spdlog file for process
        std::string _class_id = "gnss_amb_fixchk";      ///< class name
        double _intv;                        ///< interval
        base_time _beg;                        ///< begin time
        base_time _end;                        ///< end time
        double _bias;                        ///< bias
        set_base *_gset = nullptr;         ///< setting from xml file
        gnss_all_recover *_grecover1 = nullptr; ///< residuals1
        gnss_all_recover *_grecover2 = nullptr; ///< residuals2
        /*hwa_map_time_equ& _res1;
        hwa_map_time_equ& _res2;*/
        std::map<std::string, std::map<std::string, std::map<int, int>>> _flag; ///< flag for ambiguity std::fixed
        gnss_data_ambcon *_gambcon = nullptr;                 ///< ambiguity data
        std::map<AMB_ID, std::map<GSYS, int>> _num_fixed;        ///< number of std::fixed ambiguity
        std::map<AMB_ID, std::map<GSYS, int>> _num_all;          ///< number of all ambiguity
        std::set<std::string> _sites;                            ///< station name
        std::set<std::string> _sats;                             ///< satellite name
    };
}

#endif