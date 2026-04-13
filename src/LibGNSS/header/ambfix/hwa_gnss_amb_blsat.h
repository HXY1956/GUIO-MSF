#ifndef hwa_gnss_amb_BLSAT_H
#define hwa_gnss_amb_BLSAT_H

#include "hwa_set_gtype.h"
#include "hwa_gnss_amb_bl.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_amb_baseline_sat
    {
    public:
        /**
         * @brief Construct a new t gamb baseline sats object
         */
        explicit gnss_amb_baseline_sat();
        /**
         * @brief Construct a new t gamb baseline sats object
         * @param[in]  sats      satellite name list
         */
        explicit gnss_amb_baseline_sat(std::set<std::string> &sats);
        /**
         * @brief Destroy the t gamb baseline sats object
         */
        virtual ~gnss_amb_baseline_sat();
        /**
         * @brief add one satellte
         * @param[in]  sat       satellite name
         */
        void add_sat(const std::string &sat);
        /**
         * @brief get baseline number
         * @return int number of baseline
         */
        int baseline_num();
        /**
         * @brief get baseline name
         * @return std::vector<std::pair<std::string, std::string>>  station and satellite
         */
        std::vector<std::pair<std::string, std::string>> baselines();
        /**
         * @brief Get the depend double difference baseline
         * @param[in]  new_baseline new baselines
         * @return std::vector<std::tuple<std::string, std::string, std::string, std::string>> dependent baseline
         */
        std::vector<std::tuple<std::string, std::string, std::string, std::string>> get_depend_Dd_baseline(std::vector<std::tuple<std::string, std::string, std::string, std::string>> &new_baseline);

    protected:
        std::vector<gnss_amb_baseline> _baselines; ///< baselines
        std::vector<std::string> _sats;               ///< satellite name
    };
}

#endif