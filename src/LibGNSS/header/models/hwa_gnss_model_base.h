#ifndef hwa_gnss_model_baseMODEL_H
#define hwa_gnss_model_baseMODEL_H

#include "hwa_base_time.h"
#include "hwa_base_allpar.h"
#include "hwa_gnss_data_SATDATA.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_model_base_equation
    {
    public:
        /**
         * @brief Construct a new t gbaseEbase_quation object
         */
        gnss_model_base_equation();
        /**
         * @brief Destroy the t gbaseEbase_quation object
         */
        virtual ~gnss_model_base_equation();
        /**
         * @brief Define equation+
         * @param[in]  Other     another equation
         * @return gnss_model_base_equation& the equation + another equation
         */
        gnss_model_base_equation &operator+(gnss_model_base_equation &Other);

        std::vector<std::vector<std::pair<int, double>>> B; ///< coeff of equations
        std::vector<double> P;                    ///< weight of equations
        std::vector<double> l;                    ///< res of equations
    };

    // interface class for all mode
    class gnss_model_base
    {
    public:
        /**
         * @brief Construct a new t gbasemodel object
         */
        gnss_model_base();
        /**
         * @brief Destroy the t gbasemodel object
         */
        ~gnss_model_base();
        /**
         * @brief base madel calculate BPL
         * @param[in]  epoch     time
         * @param[in]  params    parameter
         * @param[in]  obsdata   observation data
         * @param[in]  result    result equation
         * @return
         *      @retval true can calculate equation
         *      @retval false can not calculate equation
         */
        virtual bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) = 0;
    };
}

#endif