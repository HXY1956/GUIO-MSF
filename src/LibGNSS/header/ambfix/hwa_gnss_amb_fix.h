#ifndef hwa_gnss_amb_fix_H
#define hwa_gnss_amb_fix_H

#include "hwa_set_base.h"
#include "hwa_set_amb.h"
#include "hwa_base_allpar.h"
#include "hwa_gnss_all_amb.h"
#include "hwa_gnss_amb_ow.h"
#include "hwa_gnss_amb_blrec.h"
#include "hwa_gnss_amb_BLSAT.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@ brief class for LAMADA search finding fixing ambiguity.
    */
    class gnss_amb_fix
    {

    public:
        /** @brief default constructor. */
        explicit gnss_amb_fix();
        /**
         * @brief Construct a new t gambfix object
         * @param[in]  spdlog       logbase control
         * @param[in]  set       setbase control
         */
        explicit gnss_amb_fix(base_log spdlog, set_base *set);
        /**
         * @brief add data
         * @param[in]  obs       all observation
         * @param[in]  bias      all bias data
         * @param[in]  pars      all parameters
         * @param[in]  objs      all object data
         */
        void add_data(gnss_all_obs *obs, gnss_all_bias *bias, base_allpar *pars, gnss_all_obj *objs);
        /**
         * @brief add data
         * @param[in]  obs       all observation
         * @param[in]  bias      all bias data
         * @param[in]  pars      all parameters
         * @param[in]  upd       all upd data
         * @param[in]  objs      all object data
         */
        void add_data(gnss_all_obs *obs, gnss_all_bias *bias, base_allpar *pars, gnss_data_upd *upd, gnss_all_obj *objs);
        /**
         * @brief add observation
         * @param[in]  obs       observation data
         */
        void add_data(gnss_all_obs *obs);
        /**
         * @brief add bias data
         * @param[in]  bias      bias data
         */
        void add_data(gnss_all_bias *bias);
        /**
         * @brief add parameter infomation
         * @param[in]  pars      parameter information
         */
        void add_data(base_allpar *pars);
        /**
         * @brief add object data
         * @param[in]  objs      dall object data
         */
        void add_data(gnss_all_obj *objs);

        /** @brief default destructor. */
        virtual ~gnss_amb_fix();
        /**
         * @brief batch process for ambfix
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        bool ProcessBatch(base_time &beg, base_time &end);

    protected:
        base_log _spdlog;             ///< logbase control
        set_base *_gset = nullptr;  ///< setbase control
        gnss_all_obs *_gobs = nullptr;   ///< observation data
        gnss_all_bias *_gbias = nullptr; ///< bias data
        base_allpar *_gpar = nullptr;   ///< all parameters
        gnss_all_obj *_gobj = nullptr;   ///< all object
        gnss_data_upd *_gupd = nullptr;      ///< upd data
        gnss_all_amb _gambs;            ///< all ambiguity

        std::set<std::string> _rec_list; ///< station list
        std::set<std::string> _sat_list; ///< satellite list

        base_time _beg; ///< begin epoch
        base_time _end; ///< end epoch
        base_time _crt; ///< current epoch
        double _intv; ///< sampling interval /s

        AMB_TYPE _mode = AMB_TYPE::DD;      ///< type for ambfix
        gnss_amb_bdeci *_evaluator_EWL = nullptr; ///< ectral wide lane
        gnss_amb_bdeci *_evaluator_WL = nullptr;  ///< wide lane
        gnss_amb_bdeci *_evaluator_NL = nullptr;  ///< narrow lane

        FIX_MODE _fix_mode;                    ///< set ambiguity fixing mode
        UPD_MODE _upd_mode;                    ///< set ambiguity fixing mode
        double _ratio;                         ///< threshold in LAMBDA method
        double _min_common_time;               ///< the Minimum common time of two observation arc
        double _wl_interval;                   ///< widelane ambiguity interval
        double _max_baseline_length;           ///< the limit of baseline length
        std::map<std::string, double> _map_EWL_decision; ///< deriation, _sigma in EWL-cycle
        std::map<std::string, double> _map_WL_decision;  ///< deriation, _sigma in WL-cycle
        std::map<std::string, double> _map_NL_decision;  ///< deriation, _sigma in NL-cycle
        /**
         * @brief Get the Wave Length
         * @param[in]  sat       satellite
         * @param[in]  type      type of observation
         * @return double wavelength
         */
        double getWaveLen(std::string sat, std::string type);
        //bool _gehwa_vector_amb_dd(Sd_amb);

        std::string _class_id = "gnss_amb_fix"; ///< class name

    private:
    };
}

#endif