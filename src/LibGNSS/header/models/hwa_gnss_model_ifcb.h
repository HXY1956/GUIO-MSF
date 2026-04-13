#ifndef hwa_gnss_model_ifcb_H
#define hwa_gnss_model_ifcb_H

#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief model of IFCB    */
    class gnss_model_ifcb
    {
    public:
        /** @brief Constructor    */
        gnss_model_ifcb(){};

        /** @brief default destructor. */
        virtual ~gnss_model_ifcb(){};

        /** @brief get correction of IFCB
        *
        *param[in] epoch            current epoch
        *param[in] sat                satlite name
        *param[in] rec                reciever name
        */
        double ifcbCorrection(const base_time &epo, const std::string &rec, const std::string &sat);

    protected:
        std::string _crt_rec;  ///< current recievers
        std::string _crt_sat;  ///< current satlite
        base_time _crt_epo; ///< current epoch
    };
}

#endif