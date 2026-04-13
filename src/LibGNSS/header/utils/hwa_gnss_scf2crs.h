/**
* @file                gscf2crs.h
* @brief            GNSS satellite-std::fixed coordinate transform to CRS coordinate
*/

#ifndef hwa_gnss_ROT_SCF2CRS_H
#define hwa_gnss_ROT_SCF2CRS_H

#include "hwa_gnss_model_ppp.h"

namespace hwa_gnss
{
    /**
    *@brief       Class for computing rotation from GNSS satellite-std::fixed system to CRS
    */
    class gnss_base_scfcrs
    {
    public:
        /** @brief default constructor. */
        gnss_base_scfcrs();

        /** @brief default destructor. */
        virtual ~gnss_base_scfcrs();

        /**
        * @brief calculate rotation matrix of scf to crs
        * @param[in]  xsat        position of satellite
        * @param[in]  vsat        velocity of satellite
        * @param[in]  xsun        position of satellite
        * @param[out] rotmat    rotation matrix of scf to crs
        */
        void calcRotMat(Vector &xsat, Vector &vsat, Vector &xsun, Matrix &rotmat); // main funtion , calculate rotation matrix scf to crs
        // extended by yqyuan
        void calcRotMat(std::string blocktype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Matrix &rotmat); // main funtion , calculate rotation matrix scf to crs

    protected:
        /**
        * @brief get number of block type(not used for now)
        * @param[in]  type        block type
        * @return   number
        */
        int _blocktype_nam2num(std::string type);

        /**
        * @brief Remove std::string trailing spaces
        * @param[in]  str        std::string need to be removed trailing space
        * @return   std::string after removing trailing space
        */
        std::string trim(std::string str);
    };
}; // namespace

#endif