#include "hwa_gnss_crs2acr.h"

namespace hwa_gnss
{
    /** @brief default constructor. */
    gnss_base_crs2acr::gnss_base_crs2acr()
    {
    }

    /** @brief default destructor. */
    gnss_base_crs2acr::~gnss_base_crs2acr()
    {
    }

    /**
    * @brief calculate rotation matrix of CRS to ACR
    * @param[in]  xsat        position of satellite
    * @param[in]  vsat        velocity of satellite
    * @param[out] rotmat    rotation matrix of CRS to ACR
    */
    void gnss_base_crs2acr::calcRotMat(Vector &xsat, Vector &vsat, Matrix &rotmat)
    {
        Triple radial;
        Triple along;
        Triple cross;
        radial.setZero();
        along.setZero();
        cross.setZero();

        radial = xsat / xsat.norm();
        along = vsat / vsat.norm();

        cross = radial.cross(along);
        cross = cross / cross.norm();

        along = radial.cross(cross);
        along = along / along.norm();

        for (int i = 0; i < 3; i++)
        {
            rotmat(0, i) = along(i);
            rotmat(1, i) = cross(i);
            rotmat(2, i) = radial(i);
        }
    }

}
