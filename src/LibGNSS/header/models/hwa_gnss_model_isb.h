/**
* @file        gisb.h
* @brief    isb model class
*/

#ifndef hwa_gnss_model_isb_H
#define hwa_gnss_model_isb_H

#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief model of ISB    */
    class gnss_model_isb
    {
    public:
        /** @brief constructor    */
        gnss_model_isb(){};

        /** @brief destructor.*/
        virtual ~gnss_model_isb(){};

        /** @brief get correction of ISB
        *
        *param[in] epoch            current epoch
        *param[in] sat                satlite name
        *param[in] rec                reciever name
        */
        double isbCorrection(const base_time &epoch, const std::string &rec, const std::string &sat);

    protected:
        std::string _crt_rec;  ///< current recievers
        std::string _crt_sat;  ///< current satlite
        base_time _crt_epo; ///< current epoch
    };
}

#endif