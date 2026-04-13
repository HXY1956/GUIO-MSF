/**
* @file              gcrs2acr.h
* @brief          CRS coordinate transform to ACR coordinate
*/

#ifndef hwa_gnss_CRS2ARC_H
#define hwa_GSS_CRS2ARC_H

#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for calculating CRS to ACR rotation matrix
    */
    class gnss_base_crs2acr
    {
    public:
        /** @brief default constructor. */
        gnss_base_crs2acr();

        /** @brief default destructor. */
        virtual ~gnss_base_crs2acr();

        /**
        * @brief calculate rotation matrix of CRS to ACR
        * @param[in]  xsat        position of satellite
        * @param[in]  vsat        velocity of satellite
        * @param[out] rotmat    rotation matrix of CRS to ACR
        */
        void calcRotMat(Vector &xsat, Vector &vsat, Matrix &rotmat);

    protected:
    };
}
#endif