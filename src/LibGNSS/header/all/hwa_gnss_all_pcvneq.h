/**
 * @file            gallpcvneq.h
 * @brief            The class for storaging PCV neq information for accumulating NEQ of all objects.
 */

#ifndef hwa_gnss_all_pcvneq_H
#define hwa_gnss_all_pcvneq_H

#include "hwa_gnss_data_pcvneq.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging neq matrix for pcv pars 
    */
    class gnss_all_pcvneq : public base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_all_pcvneq();

        explicit gnss_all_pcvneq(base_log spdlog);
        /** @brief default destructor. */
        ~gnss_all_pcvneq();

        /**
        * @brief      get the data storgging in pcvneq class
        * @return      the ptr for gnss_data_pcvneq class
        */
        std::map<std::string, std::shared_ptr<gnss_data_pcvneq>> allpcvneq();

        /**
        * @brief     add  the data to pcvneq class
        * @param[in]  obj        obj name.
        * @param[in]  pcvneq    pcvneq data.
        * @return      void
        */
        void addpcvneq(const std::string &obj, std::shared_ptr<gnss_data_pcvneq> pcvneq);

        /**
        * @brief    get one rec's pcvneq data
        * @param[in]  obj        obj name.
        * @return      pcvneq data
        */
        std::shared_ptr<gnss_data_pcvneq> pcvneq(const std::string &obj);

    protected:
        std::map<std::string, std::shared_ptr<gnss_data_pcvneq>> _allpcvneq; ///< pcvneq data(string is obj name)
    };

}

#endif