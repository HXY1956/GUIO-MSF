#ifndef hwa_gnss_amb_fixdd_H
#define hwa_gnss_amb_BFIXDD_H

#include "hwa_gnss_amb_fix.h"
#include "hwa_set_amb.h"
#include "hwa_gnss_data_ambcon.h"

namespace hwa_gnss
{
    /**
    *@ brief class for LAMADA search finding fixing ambiguity.
    */
    class gnss_amb_fixdd : public gnss_amb_fix
    {
    public:
        /**
         * @brief Construct a new t gambfixDd object
         */
        explicit gnss_amb_fixdd();
        /**
         * @brief Construct a new t gambfixDd object
         * @param[in]  spdlog       logbase control
         * @param[in]  std::set       std::setbase control
         */
        explicit gnss_amb_fixdd(base_log spdlog, set_base *set);
        /**
         * @brief Destroy the t gambfixDd object
         */
        virtual ~gnss_amb_fixdd();
        /**
         * @brief batch process for ambiguity std::fixed
         */
        bool ProcessBatch();

    protected:
        gnss_amb_baseline_rec _gRecBl; ///< station baseline
        gnss_amb_baseline_sat _gSatBl; ///< satellite baseline

        //TODO COMMENT]
        /**
         * @brief select baseline
         */
        bool _select_baseline();
        /**
         * @brief write ambiguity data
         */
        void _write_ambcon();

    private:
        std::string _class_id = "gnss_amb_fixdd"; ///< class name
        base_log _spdlog = nullptr;
    };
}

#endif