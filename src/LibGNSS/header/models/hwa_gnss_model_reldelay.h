/**
* @file        greldelay.h
* @brief    reldelay model class
*/

#ifndef hwa_gnss_model_reldelay_H
#define hwa_gnss_model_reldelay_H

#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_model_reldelay
    {
    public:
        /** @brief constructor. */
        gnss_model_reldelay(){};

        /** @brief default destructor. */
        virtual ~gnss_model_reldelay(){};

        /** @brief TODO
        *
        *param[in] crd_site                position of sites
        *param[in] vel_site                velocity of sites
        *param[in] crd_sat                position of satellites
        *param[in] vel_sat                velocity of satellites
        *return double                    TODO
        */
        double reldelay(Triple &crd_site, Triple &vel_site, Triple &crd_sat, Triple &vel_sat);

    protected:
    };
}

#endif