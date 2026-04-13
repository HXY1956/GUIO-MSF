#include "hwa_gnss_Scf2crs.h"
#include <math.h>

using namespace std;

namespace hwa_gnss
{
    /** @brief default constructor. */
    gnss_base_scfcrs::gnss_base_scfcrs()
    {
    }

    /** @brief default destructor. */
    gnss_base_scfcrs::~gnss_base_scfcrs()
    {
    }

    /**
    * @brief calculate rotation matrix of scf to crs
    * @param[in]  xsat        position of satellite
    * @param[in]  vsat        velocity of satellite
    * @param[in]  xsun        position of satellite
    * @param[out] rotmat    rotation matrix of scf to crs
    */
    void gnss_base_scfcrs::calcRotMat(Vector &xsat, Vector &vsat, Vector &xsun, Matrix &rotmat)
    {
        // double fx, fz;

        Triple scf_X;
        Triple scf_Y;
        Triple scf_Z;
        Triple unit_vsat;
        Triple unit_sc2sun;
        Triple unit_orbnorm;
        Triple unit_xsun;
        Triple unit_sunnorm;
        Triple unit_node;

        // sc-std::fixed z-axis, from sc to base_earth
        scf_Z = -xsat * 1e3;
        scf_Z = scf_Z / scf_Z.norm();

        // unint std::vector from sc to sun
        unit_sc2sun = xsun * 1e3 - xsat * 1e3;
        unit_sc2sun = unit_sc2sun / unit_sc2sun.norm();

        // unit std::vector of orbit plane
        unit_vsat = vsat / vsat.norm();

        // dx1
        unit_orbnorm = -scf_Z.cross(unit_vsat);
        unit_orbnorm = unit_orbnorm / unit_orbnorm.norm();

        // beta angel between orbit plane and sun (not used)
        unit_xsun = xsun / xsun.norm();

        // u angle from midnight
        unit_sunnorm = unit_xsun.cross(unit_orbnorm);
        unit_sunnorm = unit_sunnorm / unit_sunnorm.norm();
        unit_node = unit_sunnorm.cross(unit_orbnorm);
        unit_node = unit_node / unit_node.norm();
        // fx = dotproduct(unit_node, -scf_Z);
        // fz = dotproduct(cross(unit_node, -scf_Z), unit_orbnorm);

        // in shadow
        scf_Y = scf_Z.cross(unit_sc2sun);

        // for IGS antex-file not necessary. 20160607
        scf_Y = scf_Y / scf_Y.norm();

        // the sc-std::fixed z-axis
        scf_X = scf_Y.cross(scf_Z);

        //rotation-matrix
        for (int i = 0; i < 3; i++)
        {
            rotmat(i, 0) = scf_X(i);
            rotmat(i, 1) = scf_Y(i);
            rotmat(i, 2) = scf_Z(i);
        }

        /*if (sat[0] == 'G' || sat[0] == 'R')
        {
            
        }*/
        return;
    }

    void gnss_base_scfcrs::calcRotMat(std::string blocktype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Matrix &rotmat)
    {
        Vector scf_X(3);
        Vector scf_Y(3);
        Vector scf_Z(3);

        gnss_model_ppp *nav_attitude = new gnss_model_ppp();

        nav_attitude->attitude(blocktype, prn, xsat, vsat, xsun, scf_X, scf_Y, scf_Z);

        //rotation-matrix
        for (int i = 0; i < 3; i++)
        {
            rotmat(i, 0) = scf_X(i);
            rotmat(i, 1) = scf_Y(i);
            rotmat(i, 2) = scf_Z(i);
        }

        delete nav_attitude;
        nav_attitude = NULL;

        return;
    }

    /**
    * @brief get number of block type(not used for now)
    * @param[in]  type        block type
    * @return   number
    */
    int gnss_base_scfcrs::_blocktype_nam2num(std::string type)
    {
        std::string case_tmp = trim(type);
        int iblock;
        //GPS
        if (case_tmp == "BLOCK I")
        {
            iblock = 1;
        }
        else if (case_tmp == "BLOCK II")
        {
            iblock = 2;
        }
        else if (case_tmp == "BLOCK IIA")
        {
            iblock = 3;
        }
        else if (case_tmp == "BLOCK IIR")
        {
            iblock = 4;
        }
        else if (case_tmp == "BLOCK IIR-A")
        {
            iblock = 5;
        }
        else if (case_tmp == "BLOCK IIR-B")
        {
            iblock = 6;
        }
        else if (case_tmp == "BLOCK IIR-M")
        {
            iblock = 7;
        }
        else if (case_tmp == "BLOCK IIF")
        {
            iblock = 8;
        }

        //GLONSS
        else if (case_tmp == "GLONASS")
        {
            iblock = 101;
        }
        else if (case_tmp == "GLONASS-M")
        {
            iblock = 102;
        }
        else if (case_tmp == "GLONASS-K" || case_tmp == "GLONASS-K1")
        {
            iblock = 103;
        }

        //Galileo
        else if (case_tmp == "GALILEO GIOVEA" || case_tmp == "GALILEO-0A")
        {
            iblock = 201;
        }
        else if (case_tmp == "GALILEO GIOVEB" || case_tmp == "GALILEO-0B")
        {
            iblock = 201;
        }
        else if (case_tmp == "GALILEO IOV" || case_tmp == "GALILEO-1")
        {
            iblock = 202;
        }
        else if (case_tmp == "GALILEO FOC" || case_tmp == "GALILEO-2")
        {
            iblock = 203;
        }
        //BDS
        else if (case_tmp == "BEIDOU-2G")
        {
            iblock = 301;
        }
        else if (case_tmp == "BEIDOU-2I")
        {
            iblock = 302;
        }
        else if (case_tmp == "BEIDOU-2M")
        {
            iblock = 303;
        }
        //QZSS
        else if (case_tmp == "QZSS")
        {
            iblock = 401;
        }
        else
        {
            return -1;
        }
        return iblock;
    }

    // jdhuang : remove the warning, make some change
    /**
    * @brief Remove std::string trailing spaces
    * @param[in]  str        std::string need to be removed trailing space
    * @return   std::string after removing trailing space
    */
    std::string gnss_base_scfcrs::trim(std::string str)
    {
        int nlen = str.length();
        for (int i = nlen - 1; i >= 0; i--)
        {
            if (!isspace(str[i]))
            {
                return str.substr(0, i + 1);
            }
        }
        return str;
    }

} // namespace
