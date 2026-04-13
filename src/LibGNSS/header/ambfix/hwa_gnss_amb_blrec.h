/**
* @file        gambBlRec.h
* @brief    site baseline of SD and DD ambiguity.
*/

#ifndef hwa_gnss_amb_BLREC_H
#define hwa_gnss_amb_BLREC_H

#include "hwa_set_base.h"
#include "hwa_set_gtype.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_amb_BlSat.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_all_obs.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
     * @brief site baseline of SD and DD ambiguity.
     */
    class gnss_amb_baseline_rec : public gnss_amb_baseline_sat
    {
    public:
        /**
         * @brief Construct a new t gamb baseline recs object
         */
        explicit gnss_amb_baseline_rec();
        /**
         * @brief Construct a new t gamb baseline recs object
         * @param[in]  time       time
         * @param[in]  cut_length cut length
         */
        explicit gnss_amb_baseline_rec(const base_time &time, double cut_length);
        /**
         * @brief Destroy the t gamb baseline recs object
         */
        virtual ~gnss_amb_baseline_rec();
        /**
         * @brief add current 
         * @param[in]  time       current time
         * @param[in]  cut_length current time
         */
        void add_cut(const base_time &time, double cut_length);
        /**
         * @brief add object
         * @param[in]  obj       object
         */
        void add_gobj(gnss_all_obj *obj);
        /**
         * @brief add std::setbase control
         * @param[in]  std::set       std::setbase control
         */
        void add_gset(set_base *set);
        /**
         * @brief add observation
         * @param[in]  obs       observation
         */
        void add_gobs(gnss_all_obs *obs);
        /**
         * @brief add logbase control
         * @param[in]  spdlog       logbase control
         */
        void add_glog(base_log spdlog);
        /**
         * @brief add station and satellite
         * @param[in]  rec       station name 
         * @param[in]  sat       satellite name
         */
        void add_rec(const std::string &rec, const std::string &sat);
        /**
         * @brief add station, satallite and station coordinate
         * @param[in]  rec       station name
         * @param[in]  sat       satellite name
         * @param[in]  crd       station coordinate
         */
        void add_rec(const std::string &rec, const std::string &sat, Triple crd);
        /**
         * @brief get baselines
         * @return std::vector<std::pair<std::string, std::string>> baseline
         */
        std::vector<std::pair<std::string, std::string>> baselines();

    protected:
        set_base *_gset = nullptr; ///< std::setbase control
        gnss_all_obj *_gobj = nullptr;  ///< all object data
        base_log _spdlog;            ///< logbase control

        base_time _cut_time;         ///< current time
        base_time _beg_time;         ///< begin time
        base_time _end_time;         ///< end time
        double _cut_length = 3500; //km                ///< current length

        std::vector<std::string> _recs;          ///< station name list
        std::map<std::string, std::set<std::string>> _sys; ///< system std::map
        std::map<std::string, Triple> _xyz;   ///< station and coordinate std::map

    protected:
        /**
         * @brief get station coordinate
         */
        bool _get_rec_xyz();
        /**
         * @brief creat baseline
         */
        bool _create_baseline();
        /**
         * @brief check amb
         */
        bool _check_amb_depend();
    };
    /**
     * @brief compare two baselines
     * @param[in]  bl1       baselines_A
     * @param[in]  bl2       baselines_B
     * @return true 
     *         @retval true same
     *         @retval false different
     */
    bool compare(const gnss_amb_baseline &bl1, const gnss_amb_baseline &bl2);
}

#endif