#ifndef hwa_gnss_model_ifb_H
#define hwa_gnss_model_ifb_H

#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief model of IFB    */
    class gnss_model_ifb
    {
    public:
        /** @brief constructor    */
        gnss_model_ifb(){};

        /** @brief destructor.*/
        virtual ~gnss_model_ifb(){};

        /** @brief get correction of IFB
        *
        *param[in] epoch            current epoch
        *param[in] sat                satlite name
        *param[in] rec                reciever name
        */
        double ifbCorrection(const base_time &epo, const std::string &rec, const std::string &sat);

    protected:
        std::string _crt_rec;  ///< current recievers
        std::string _crt_sat;  ///< current satlite
        base_time _crt_epo; ///< current epoch
    };
}

#endif