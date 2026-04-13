#include <iostream>
#include <cmath>
#include "hwa_base_par.h"

using namespace std;
using namespace hwa_gnss;

namespace hwa_base
{

    const std::string base_par::par_str_sep = "_";

    const std::vector<std::string> base_par::par_str =
    {
        "CRD_X",
        "CRD_Y",
        "CRD_Z", // coordinates
        "TRP",
        "GRD_N",
        "GRD_E",
        "SION",
        "VION", // atmospheric parameters
        "CLK",
        "CLK_SAT", // clocks
        "CLK13_G",
        "CLK13_E",
        "CLK13_C",
        "CLK13_J", // receiver clocks for different systems in IF_mode
        "CLK14_G",
        "CLK14_E",
        "CLK14_C",
        "CLK14_J", // receiver clocks for different systems in IF_mode
        "CLK15_G",
        "CLK15_E",
        "CLK15_C",
        "CLK15_J", // receiver clocks for different systems in IF_mode
        "CLK15_G_SAT",
        "E15CLK_SAT",
        "C15CLK_SAT",
        "J15CLK_SAT", // satellite clocks for 13 combination (add by xiongyun)
        "CLK_G",
        "CLK_E",
        "CLK_C",
        "CLK_R",
        "CLK_J", // receiver clocks for different systems (add by xiongyun)
        "CLK_VPW",
        "CLK_VR",
        "CLK_VQ",
        "IFB_C3",
        "IFB_C4",
        "IFB_C5",
        "IFCB_F3",
        "IFCB_F4",
        "IFCB_F5", // inter-freq. code biases for FREQ_3, FREQ_4, FREQ_5, inter-freq. clock bias for GPS FREQ_3
        "SAT_IFB_C3",
        "SAT_IFB_C4",
        "SAT_IFB_C5",
        "GPS_REC_IFB_C3",
        "GPS_REC_IFB_C4",
        "GPS_REC_IFB_C5",
        "GAL_REC_IFB_C3",
        "GAL_REC_IFB_C4",
        "GAL_REC_IFB_C5",
        "BDS_REC_IFB_C3",
        "BDS_REC_IFB_C4",
        "BDS_REC_IFB_C5",
        "QZS_REC_IFB_C3",
        "QZS_REC_IFB_C4",
        "QZS_REC_IFB_C5",
        "GPS_IFB_C3",
        "GPS_IFB_C4",
        "GPS_IFB_C5",
        "GAL_IFB_C3",
        "GAL_IFB_C4",
        "GAL_IFB_C5",
        "BDS_IFB_C3",
        "BDS_IFB_C4",
        "BDS_IFB_C5",
        "QZS_IFB_C3",
        "QZS_IFB_C4",
        "QZS_IFB_C5",
        "CLK_ICB",
        "CLUSTERB", // initial clock bias, cluster-dependent bias
        "AMB_IF",
        "AMB13_IF",
        "AMB14_IF",
        "AMB15_IF",
        "AMB_WL",
        "AMB_L1",
        "AMB_L2",
        "AMB_L3",
        "AMB_L4",
        "AMB_L5", // ambiguities for indiv. freq. (number indicates freq not band)
        "FCBS_IF",
        "FCBS_L1",
        "FCBS_L2",
        "FCBS_L3",
        "FCBS_L4",
        "FCBS_L5", // satellite base_type_conv::fractional cycle biases for indiv. freq.
        "FCBR_IF",
        "FCBR_L1",
        "FCBR_L2",
        "FCBR_L3",
        "FCBR_L4",
        "FCBR_L5", // receiver base_type_conv::fractional cycle biases for indiv. freq.
        "GLO_ISB",
        "GLO_ifcb",
        "GLO_IFPB",
        "GAL_ISB",
        "BDS_ISB",
        "QZS_ISB", // multi-GNSS
        "LEO_ISB",
        "IFB_GPS", "IFB_BDS", "IFB_QZS", "IFB_GAL", "IFB_GAL_2", "IFB_GAL_3",
        "IFB_BDS_2", "IFB_BDS_3",
        "ATT_X", "ATT_Y", "ATT_Z",
        "eb_X", "eb_Y", "eb_Z",
        "db_X", "db_Y", "db_Z",
        "gyro_scale_X", "gyro_scale_Y", "gyro_scale_Z",
        "acce_scale_X", "acce_scale_Y", "acce_scale_Z",
        "IMU_INST_ATT_X", "IMU_INST_ATT_Y", "IMU_INST_ATT_Z",
        "ODO_k",
        "CAM_ATT_X", "CAM_ATT_Y", "CAM_ATT_Z",
        "CAM_CRD_X", "CAM_CRD_Y", "CAM_CRD_Z",
        "EXTRINSIC_ATT_X", "EXTRINSIC_ATT_Y", "EXTRINSIC_ATT_Z",
        "EXTRINSIC_CRD_X", "EXTRINSIC_CRD_Y", "EXTRINSIC_CRD_Z",
        "GLO_IFB",                                                                        // inter frequency bias for glonass
        "VTEC", "DCB_REC", "DCB_SAT",                                                     // for DCB estimator
        "P1P2G_REC", "P1P2E_REC", "P1P2R_REC", "P1P2C_REC",                               // GNSS-specific receiver code DCB P1-P2
        "VEL_X", "VEL_Y", "VEL_Z",                                                        // velocity
        "ACC_X", "ACC_Y", "ACC_Z",                                                        // acceleration
        "CLK_RAT",                                                                        // satellite clock speed
        "SCALE", "GEOCX", "GEOCY", "GEOCZ",                                               // ERP parameter add by lijie
        "XPOLE", "YPOLE", "DXPOLE", "DYPOLE", "UT1", "DUT1", "UT1VLBI", "NUTDX", "NUTDY", // EOP parameter
        "PXSAT", "PYSAT", "PZSAT", "VXSAT", "VYSAT", "VZSAT",                             // satellite parameter
        "SR_scale", "DRAG_c",
        "EMP_Sc1", "EMP_Cc1", "EMP_Sa1", "EMP_Ca1", "EMP_Sr1", "EMP_Cr1",
        "EMP_Sc2", "EMP_Cc2", "EMP_Sa2", "EMP_Ca2", "EMP_Sr2", "EMP_Cr2",
        "EMP_Ac", "EMP_Bc", "EMP_Aa", "EMP_Ba", "EMP_Ar", "EMP_Br",
        "D0", "Dc", "Ds", "Y0", "Yc", "Ys", "B0", "Bc", "Bs",                // ECOM Force parameter
        "ECOMT_T30", "ECOMT_T3c2", "ECOMT_T3s2", "ECOMT_T3c4", "ECOMT_T3s4", // ECOM-T
        "ECOMT_T20", "ECOMT_T2c2", "ECOMT_T2s2", "ECOMT_T1s2",
        "ECOM2_D0", "ECOM2_D2c", "ECOM2_D2s", "ECOM2_D4c", "ECOM2_D4s",   // ECOM2
        "ECOM2_Y0", "ECOM2_X0", "ECOM2_X1c", "ECOM2_X1s",                 // ECOM2
        "ABW_SP", "ABW_SB", "ABW_Y0", "ABW_PXAD", "ABW_PZAD", "ABW_NZAD", // ABW
        "ABW_PXR", "ABW_PZR", "ABW_NZR"                                   // ABW
                              "IFBR_RAT1",
        "IFBR_RAT2",
        "RB", "RB_GLO", "RB_GAL", "RB_BDS", "RB_GPS", "RB_PSM4", "RB_PSM7", "RB_PSM9", "RB_all",
        "SION_3",
        "RA", "DE", "VRA", "VDE",
        "PCV_SAT", "PCO_SAT_X", "PCO_SAT_Y", "PCO_SAT_Z", // SAT PCV/PCO pars
        "PCV_REC", "PCO_REC_X", "PCO_REC_Y", "PCO_REC_Z"  // REC PCV/PCO pars
    };

    base_par_head::base_par_head(par_type type, std::string site, std::string sat) : type(type),
        site(site),
        sat(sat)
    {
    }
    base_par_head::base_par_head(const base_par_head& Other) : type(Other.type),
        sat(Other.sat),
        site(Other.site)
    {
    }
    base_par_head::~base_par_head()
    {
    }
    bool base_par_head::operator==(const base_par_head& Other) const
    {
        if (this->type == Other.type &&
            this->sat == Other.sat &&
            this->site == Other.site)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool base_par_head::operator<(const base_par_head& Other) const
    {
        if (
            this->type < Other.type ||
            (this->type == Other.type && this->site < Other.site) ||
            (this->type == Other.type && this->site == Other.site && this->sat < Other.sat)
            )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool base_par_head::operator<=(const base_par_head& Other) const
    {
        if (*this == Other || *this < Other)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool base_par_head::operator>(const base_par_head& Other) const
    {
        if (*this <= Other)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    bool base_par_head::operator>=(const base_par_head& Other) const
    {
        if (*this < Other)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
   
    base_time_arc::base_time_arc(const base_time& _beg, const base_time& _end) : begin(_beg), end(_end) {}
    base_time_arc::~base_time_arc() = default;
    bool base_time_arc::operator!=(const base_time_arc& Other) const
    {
        return !(*this == Other);
    }
    bool base_time_arc::operator==(const base_time_arc& Other) const
    {
        return (this->begin == Other.begin && this->end == Other.end);
    }
    bool base_time_arc::operator<(const base_time_arc& Other) const
    {
        return (this->begin < Other.begin || (this->begin == Other.begin && this->end < Other.end));
    }
    bool base_time_arc::operator<=(const base_time_arc& Other) const
    {
        return (*this == Other || *this < Other);
    }
    bool base_time_arc::operator>(const base_time_arc& Other) const
    {
        return !(*this <= Other);
    }
    bool base_time_arc::operator>=(const base_time_arc& Other) const
    {
        return !(*this < Other);
    }
    bool base_time_arc::inside(const base_time_arc& Other) const
    {
        return (this->begin >= Other.begin && this->end <= Other.end);
    }

    std::string gpar2str(const base_par& par)
    {
        std::string partype = par.str_type();
        partype = par.site + base_par::par_str_sep + partype;
        if (par.prn == "")
        {
            partype += base_par::par_str_sep;
        }
        return partype;
    }

    std::string ptype2str(const par_type& parType)
    {
        // get data type
        // ----------
        std::string type;
        switch (parType)
        {
        case par_type::CRD_X:
            type = "CRD_X";
            break;
        case par_type::CRD_Y:
            type = "CRD_Y";
            break;
        case par_type::CRD_Z:
            type = "CRD_Z";
            break;
        case par_type::TRP:
            type = "TRP";
            break;
        case par_type::SION:
            type = "SION_";
            break;
        case par_type::VION:
            type = "VION_";
            break;
        case par_type::CLK:
            type = "CLK";
            break;
        case par_type::CLK13_G:
            type = "CLK13_G";
            break;
        case par_type::CLK13_E:
            type = "CLK13_E";
            break;
        case par_type::CLK13_C:
            type = "CLK13_C";
            break;
        case par_type::CLK13_J:
            type = "CLK13_J";
            break;
        case par_type::CLK14_G:
            type = "CLK14_G";
            break;
        case par_type::CLK14_E:
            type = "CLK14_E";
            break;
        case par_type::CLK14_C:
            type = "CLK14_C";
            break;
        case par_type::CLK14_J:
            type = "CLK14_J";
            break;
        case par_type::CLK15_G:
            type = "CLK15_G";
            break;
        case par_type::CLK15_E:
            type = "CLK15_E";
            break;
        case par_type::CLK15_C:
            type = "CLK15_C";
            break;
        case par_type::CLK15_J:
            type = "CLK15_J";
            break;
        case par_type::CLK_G:
            type = "CLK_G";
            break;
        case par_type::CLK_E:
            type = "CLK_E";
            break;
        case par_type::CLK_C:
            type = "CLK_C";
            break;
        case par_type::CLK_R:
            type = "CLK_R";
            break;
        case par_type::CLK_J:
            type = "CLK_J";
            break;
        case par_type::CLK_VPW:
            type = "CLK_VPW";
            break;
        case par_type::CLK_VR:
            type = "CLK_PR";
            break;
        case par_type::CLK_VQ:
            type = "CLK_PQ";
            break;
        case par_type::IFCB_F3:
            type = "IFCB_F3";
            break;
        case par_type::IFCB_F4:
            type = "IFCB_F4";
            break;
        case par_type::IFCB_F5:
            type = "IFCB_F5";
            break;
        case par_type::IFB_C3:
            type = "IFB_F3";
            break;
        case par_type::IFB_C4:
            type = "IFB_F4";
            break;
        case par_type::IFB_C5:
            type = "IFB_F5";
            break;
        case par_type::SAT_IFB_C3:
            type = "IFB_SAT_F3";
            break;
        case par_type::SAT_IFB_C4:
            type = "IFB_SAT_F4";
            break;
        case par_type::SAT_IFB_C5:
            type = "IFB_SAT_F5";
            break;
        case par_type::GPS_REC_IFB_C3:
            type = "IFB_REC_F3";
            break;
        case par_type::GPS_REC_IFB_C4:
            type = "IFB_REC_F4";
            break;
        case par_type::GPS_REC_IFB_C5:
            type = "IFB_REC_F5";
            break;
        case par_type::GAL_REC_IFB_C3:
            type = "GAL_REC_IFB_C3";
            break;
        case par_type::GAL_REC_IFB_C4:
            type = "GAL_REC_IFB_C4";
            break;
        case par_type::GAL_REC_IFB_C5:
            type = "GAL_REC_IFB_C5";
            break;
        case par_type::BDS_REC_IFB_C3:
            type = "BDS_REC_IFB_C3";
            break;
        case par_type::BDS_REC_IFB_C4:
            type = "BDS_REC_IFB_C4";
            break;
        case par_type::BDS_REC_IFB_C5:
            type = "BDS_REC_IFB_C5";
            break;
        case par_type::QZS_REC_IFB_C3:
            type = "QZS_REC_IFB_C3";
            break;
        case par_type::QZS_REC_IFB_C4:
            type = "QZS_REC_IFB_C4";
            break;
        case par_type::QZS_REC_IFB_C5:
            type = "QZS_REC_IFB_C5";
            break;
        case par_type::CLK_SAT:
            type = "CLK_SAT";
            break;
        case par_type::CLK15_G_SAT:
            type = "CLK15_G_SAT";
            break; //add by xiongyun
        case par_type::E15CLK_SAT:
            type = "E15CLK_SAT_";
            break;
        case par_type::C15CLK_SAT:
            type = "C15CLK_SAT";
            break;
        case par_type::J15CLK_SAT:
            type = "J15CLK_SAT";
            break;
        case par_type::AMB_IF:
            type = "AMB_IF";
            break;
        case par_type::AMB13_IF:
            type = "AMB13_IF";
            break;
        case par_type::AMB14_IF:
            type = "AMB14_IF";
            break;
        case par_type::AMB15_IF:
            type = "AMB15_IF";
            break;
        case par_type::AMB_WL:
            type = "AMB_WL";
            break;
        case par_type::AMB_L1:
            type = "AMB_L1_";
            break;
        case par_type::AMB_L2:
            type = "AMB_L2_";
            break;
        case par_type::AMB_L3:
            type = "AMB_L3_";
            break;
        case par_type::AMB_L4:
            type = "AMB_L4_";
            break;
        case par_type::AMB_L5:
            type = "AMB_L5_";
            break;
        case par_type::FCBS_IF:
            type = "FCBS_IF";
            break;
        case par_type::FCBS_L1:
            type = "FCBS_L1";
            break;
        case par_type::FCBS_L2:
            type = "FCBS_L2";
            break;
        case par_type::FCBS_L3:
            type = "FCBS_L3";
            break;
        case par_type::FCBS_L4:
            type = "FCBS_L4";
            break;
        case par_type::FCBS_L5:
            type = "FCBS_L5";
            break;
        case par_type::FCBR_IF:
            type = "FCBR_IF";
            break;
        case par_type::FCBR_L1:
            type = "FCBR_L1";
            break;
        case par_type::FCBR_L2:
            type = "FCBR_L2";
            break;
        case par_type::FCBR_L3:
            type = "FCBR_L3";
            break;
        case par_type::FCBR_L4:
            type = "FCBR_L4";
            break;
        case par_type::FCBR_L5:
            type = "FCBR_L5";
            break;
        case par_type::CLK_ICB:
            type = "CLK_ICB";
            break;
        case par_type::CLUSTERB:
            type = "CLUSTERB";
            break;
        case par_type::GRD_N:
            type = "GRD_N";
            break;
        case par_type::GRD_E:
            type = "GRD_E";
            break;
        case par_type::GLO_ISB:
            type = "GLO_ISB";
            break;
        case par_type::GLO_ifcb:
            type = "GLO_ifcb";
            break;
        case par_type::GLO_IFPB:
            type = "GLO_IFPB";
            break;
        case par_type::GLO_IFB:
            type = "GLO_IFB";
            break;
        case par_type::GAL_ISB:
            type = "GAL_ISB";
            break;
        case par_type::BDS_ISB:
            type = "BDS_ISB";
            break;
        case par_type::QZS_ISB:
            type = "QZS_ISB";
            break;
        case par_type::IFB_GPS:
            type = "IFB_GPS";
            break;
        case par_type::IFB_BDS:
            type = "IFB_BDS";
            break;
        case par_type::IFB_GAL:
            type = "IFB_GAL";
            break;
        case par_type::IFB_GAL_2:
            type = "IFB_GAL_2";
            break;
        case par_type::IFB_GAL_3:
            type = "IFB_GAL_3";
            break;
        case par_type::IFB_BDS_2:
            type = "IFB_BDS_2";
            break;
        case par_type::IFB_BDS_3:
            type = "IFB_BDS_3";
            break;
        case par_type::IFB_QZS:
            type = "IFB_QZS";
            break;
        case par_type::P1P2G_REC:
            type = "P1P2G_REC";
            break;
        case par_type::P1P2E_REC:
            type = "P1P2E_REC";
            break;

        case par_type::SCALE:
            type = "SCALE";
            break;
        case par_type::GEOCX:
            type = "GEOCX";
            break;
        case par_type::GEOCY:
            type = "GEOCY";
            break;
        case par_type::GEOCZ:
            type = "GEOCZ";
            break;
        case par_type::XPOLE:
            type = "XPOLE";
            break;
        case par_type::YPOLE:
            type = "YPOLE";
            break;
        case par_type::DXPOLE:
            type = "DXPOLE";
            break;
        case par_type::DYPOLE:
            type = "DYPOLE";
            break;
        case par_type::UT1:
            type = "UT1";
            break;
        case par_type::DUT1:
            type = "DUT1";
            break;
        case par_type::UT1VLBI:
            type = "UT1VLBI";
            break;
        case par_type::NUTDX:
            type = "NUTDX";
            break;
        case par_type::NUTDY:
            type = "NUTDY";
            break;
        case par_type::PXSAT:
            type = "PXSAT";
            break;
        case par_type::PYSAT:
            type = "PYSAT";
            break;
        case par_type::PZSAT:
            type = "PZSAT";
            break;
        case par_type::VXSAT:
            type = "VXSAT";
            break;
        case par_type::VYSAT:
            type = "VYSAT";
            break;
        case par_type::VZSAT:
            type = "VZSAT";
            break;
        case par_type::D0:
            type = "D0";
            break;
        case par_type::Dc:
            type = "Dc";
            break;
        case par_type::Ds:
            type = "Ds";
            break;
        case par_type::Y0:
            type = "Y0";
            break;
        case par_type::Yc:
            type = "Yc";
            break;
        case par_type::Ys:
            type = "Ys";
            break;
        case par_type::B0:
            type = "X0";
            break;
        case par_type::Bc:
            type = "Xc";
            break;
        case par_type::Bs:
            type = "Xs";
            break;
        case par_type::SR_scale:
            type = "SR_scale";
            break;
        case par_type::DRAG_c:
            type = "DRAG_c";
            break;
        case par_type::EMP_Sc1:
            type = "EMP_Sc1";
            break;
        case par_type::EMP_Cc1:
            type = "EMP_Cc1";
            break;
        case par_type::EMP_Sa1:
            type = "EMP_Sa1";
            break;
        case par_type::EMP_Ca1:
            type = "EMP_Ca1";
            break;
        case par_type::EMP_Sr1:
            type = "EMP_Sr1";
            break;
        case par_type::EMP_Cr1:
            type = "EMP_Cr1";
            break;

        case par_type::EMP_Sc2:
            type = "EMP_Sc2";
            break;
        case par_type::EMP_Cc2:
            type = "EMP_Cc2";
            break;
        case par_type::EMP_Sa2:
            type = "EMP_Sa2";
            break;
        case par_type::EMP_Ca2:
            type = "EMP_Ca2";
            break;
        case par_type::EMP_Sr2:
            type = "EMP_Sr2";
            break;
        case par_type::EMP_Cr2:
            type = "EMP_Cr2";
            break;

        case par_type::EMP_Ac:
            type = "EMP_Ac";
            break;
        case par_type::EMP_Bc:
            type = "EMP_Bc";
            break;
        case par_type::EMP_Aa:
            type = "EMP_Aa";
            break;
        case par_type::EMP_Ba:
            type = "EMP_Ba";
            break;
        case par_type::EMP_Ar:
            type = "EMP_Ar";
            break;
        case par_type::EMP_Br:
            type = "EMP_Br";
            break;
        case par_type::ECOM2_D0:
            type = "ECOM2_D0";
            break;
        case par_type::ECOM2_D2c:
            type = "ECOM2_D2c";
            break;
        case par_type::ECOM2_D2s:
            type = "ECOM2_D2s";
            break;
        case par_type::ECOM2_D4c:
            type = "ECOM2_D4c";
            break;
        case par_type::ECOM2_D4s:
            type = "ECOM2_D4s";
            break;
        case par_type::ECOM2_Y0:
            type = "ECOM2_Y0";
            break;
        case par_type::ECOM2_X0:
            type = "ECOM2_X0";
            break;
        case par_type::ECOM2_X1c:
            type = "ECOM2_X1c";
            break;
        case par_type::ECOM2_X1s:
            type = "ECOM2_X1s";
            break;
        case par_type::ABW_SP:
            type = "ABW_SP";
            break;
        case par_type::ABW_SB:
            type = "ABW_SB";
            break;
        case par_type::ABW_Y0:
            type = "ABW_Y0";
            break;
        case par_type::ABW_PXAD:
            type = "ABW_PXAD";
            break;
        case par_type::ABW_PXR:
            type = "ABW_PXR";
            break;
        case par_type::ABW_PZAD:
            type = "ABW_PZAD";
            break;
        case par_type::ABW_PZR:
            type = "ABW_PZR";
            break;
        case par_type::ABW_NZAD:
            type = "ABW_NZAD";
            break;
        case par_type::ABW_NZR:
            type = "ABW_NZR";
            break;
        case par_type::RB:
            type = "RB";
            break;
        case par_type::RB_GPS:
            type = "RB_GPS";
            break;
        case par_type::RB_GLO:
            type = "RB_GLO";
            break;
        case par_type::RB_GAL:
            type = "RB_GAL";
            break;
        case par_type::RB_BDS:
            type = "RB_BDS";
            break;
        case par_type::RB_PSM4:
            type = "RB_PSM4";
            break;
        case par_type::RB_PSM7:
            type = "RB_PSM7";
            break;
        case par_type::RB_PSM9:
            type = "RB_PSM9";
            break;
        case par_type::RB_all:
            type = "RB_all";
            break;
        case par_type::C_kbr:
            type = "C_kbr";
            break;
        case par_type::C_lri:
            type = "C_lri";
            break;
        case par_type::RA:
            type = "RA";
            break;
        case par_type::DE:
            type = "DE";
            break;
        case par_type::VRA:
            type = "VRA";
            break;
        case par_type::VDE:
            type = "VDE";
            break;
        case par_type::PCV_SAT:
            type = "PCV_SAT";
            break;
        case par_type::PCO_SAT_X:
            type = "PCO_SAT_X";
            break;
        case par_type::PCO_SAT_Y:
            type = "PCO_SAT_Y";
            break;
        case par_type::PCO_SAT_Z:
            type = "PCO_SAT_Z";
            break;
        case par_type::PCV_REC:
            type = "PCV_REC";
            break;
        case par_type::PCO_REC_X:
            type = "PCO_REC_X";
            break;
        case par_type::PCO_REC_Y:
            type = "PCO_REC_Y";
            break;
        case par_type::PCO_REC_Z:
            type = "PCO_REC_Z";
            break;
        default:
            type = "UNDEF";
        }
        return type;
    }

    base_par str2gpar(const std::string& str_par)
    {
        int sep_first, sep_last = 0;
        std::string site, sat, str_type;
        sep_first = str_par.find(base_par::par_str_sep);
        site = str_par.substr(0, sep_first);

        sep_last = str_par.rfind(base_par::par_str_sep);
        sat = str_par.substr(sep_last + 1);

        str_type = str_par.substr(sep_first + 1, sep_last - sep_first - 1);

        par_type par_tp(par_type::NO_DEF);

        for (unsigned int i = 0; i < base_par::par_str.size(); i++)
        {
            if (base_par::par_str[i] == str_type)
            {
                par_tp = par_type(i);
                break;
            }
        }

        return base_par(site, par_tp, 0, sat);
    }

    // constructor
// --------------------------------------------------------
    base_par::base_par(const string& site, par_type t, unsigned i, const string& p, bool remove)
    {
        beg = FIRST_TIME;
        end = LAST_TIME;

        this->site = site;
        lremove = remove; //added for LEO
        parType = t;
        index = i;
        prn = p;
        lPWC = false;
        amb_ini = false;
        //indPWC = 0;
        nPWC = 1;
        value(0.0);
        apriori(0.0);
        idx_pcv = 0;
        zen1 = 0.0;
        zen2 = 0.0;
        azi1 = 0.0;
        azi2 = 0.0;
        dzen = 0.0;
        dazi = 0.0;
        nzen = 0;
        nazi = 0;
        sat_zero_con = false;
        rec_zero_con = false;
        nobs = 0;
    }

    base_par::base_par()
    {
        beg = FIRST_TIME;
        end = LAST_TIME;
    }

    // base_par destructor
    // ---------------------------------------------------
    base_par::~base_par()
    {
    }

    // Operators for base_par
    // -------------------------------------------------
    bool base_par::operator==(const base_par& par) const
    {
        if (parType == par.parType &&
            //  prn.compare(par.prn)   == 0 &&
            //  site.compare(par.site) == 0 &&
            beg == par.beg &&
            end == par.end)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // Doplnil gabo aby par mohla byt pouzita ako klic v mape
    bool base_par::operator<(const base_par& par) const
    {
        if ((parType < par.parType) ||
            (parType == par.parType && prn < par.prn) ||
            (parType == par.parType && prn == par.prn && site < par.site) ||
            (parType == par.parType && prn == par.prn && site == par.site && beg < par.beg) ||
            (parType == par.parType && prn == par.prn && site == par.site && beg == par.beg && end < par.end))
        {
            return true;
        }
        return false;
    }

    bool base_par::operator>(const base_par& par) const
    {
        if ((parType > par.parType) ||
            (parType == par.parType && prn < par.prn) ||
            (parType == par.parType && prn == par.prn && site < par.site) ||
            (parType == par.parType && prn == par.prn && site == par.site && beg < par.beg) ||
            (parType == par.parType && prn == par.prn && site == par.site && beg == par.beg && end < par.end))
        {
            return true;
        }
        return false;
    }

    base_par base_par::operator-(const base_par& p) const
    {
        base_par par = (*this);
        par.value(value() - p.value());
        return par;
    }

    base_par base_par::operator+(const base_par& p) const
    {
        base_par par = (*this);
        par.value(value() + p.value());
        return par;
    }

    // Setting begin and end time for validity object
    // -------------------------------------------------
    void base_par::setTime(const base_time& t1, const base_time& t2)
    {
        this->beg = t1;
        this->end = t2;
    }

    // get data type
    // ----------
    string base_par::str_type() const
    {
        // jdhuang : change to pretty recover file
        string type;
        switch (parType)
        {
        case par_type::CRD_X:
            type = "CRD_X";
            break;
        case par_type::CRD_Y:
            type = "CRD_Y";
            break;
        case par_type::CRD_Z:
            type = "CRD_Z";
            break;
        case par_type::TRP:
            type = "TRP";
            break;
        case par_type::VEL_X:
            type = "VEL_X";
            break;
        case par_type::VEL_Y:
            type = "VEL_Y";
            break;
        case par_type::VEL_Z:
            type = "VEL_Z";
            break;
        case par_type::CLK_RAT:
            type = "CLK_RAT";
            break;
        case par_type::SION:
            type = "SION_" + prn;
            break;
        case par_type::VION:
            type = "VION_" + prn;
            break;
        case par_type::CLK:
            type = "CLK";
            break;
        case par_type::CLK_G:
            type = "CLK_G";
            break;
        case par_type::CLK_C:
            type = "CLK_C";
            break;
        case par_type::CLK_E:
            type = "CLK_E";
            break;
        case par_type::CLK_R:
            type = "CLK_R";
            break;
        case par_type::CLK_J:
            type = "CLK_J";
            break;
        case par_type::CLK_VPW:
            type = "CLK_V";
            break;
        case par_type::CLK_VR:
            type = "CLK_VR";
            break;
        case par_type::CLK_VQ:
            type = "CLK_VQ";
            break;
        case par_type::CLK13_G:
            type = "CLK13_G";
            break;
        case par_type::CLK14_G:
            type = "CLK14_G";
            break;
        case par_type::CLK15_G:
            type = "CLK15_G";
            break;
        case par_type::CLK13_E:
            type = "CLK13_E";
            break;
        case par_type::CLK14_E:
            type = "CLK14_E";
            break;
        case par_type::CLK15_E:
            type = "CLK15_E";
            break;
        case par_type::CLK13_C:
            type = "CLK13_C";
            break;
        case par_type::CLK14_C:
            type = "CLK14_C";
            break;
        case par_type::CLK15_C:
            type = "CLK15_C";
            break;
        case par_type::CLK13_J:
            type = "CLK13_J";
            break;
        case par_type::CLK14_J:
            type = "CLK14_J";
            break;
        case par_type::CLK15_J:
            type = "CLK15_J";
            break;
        case par_type::IFCB_F3:
            type = "IFCB_F3_" + prn;
            break;
        case par_type::IFCB_F4:
            type = "IFCB_F4_" + prn;
            break;
        case par_type::IFCB_F5:
            type = "IFCB_F5_" + prn;
            break;
        case par_type::IFB_C3:
            type = "IFB_F3_" + prn;
            break;
        case par_type::IFB_C4:
            type = "IFB_F4_" + prn;
            break;
        case par_type::IFB_C5:
            type = "IFB_F5_" + prn;
            break;
        case par_type::GPS_REC_IFB_C3:
            type = "GPS_REC_IFB_C3";
            break;
        case par_type::GPS_REC_IFB_C4:
            type = "GPS_REC_IFB_C4";
            break;
        case par_type::GPS_REC_IFB_C5:
            type = "GPS_REC_IFB_C5";
            break;
        case par_type::GAL_REC_IFB_C3:
            type = "GAL_REC_IFB_C3";
            break;
        case par_type::GAL_REC_IFB_C4:
            type = "GAL_REC_IFB_C4";
            break;
        case par_type::GAL_REC_IFB_C5:
            type = "GAL_REC_IFB_C5";
            break;
        case par_type::BDS_REC_IFB_C3:
            type = "BDS_REC_IFB_C3";
            break;
        case par_type::BDS_REC_IFB_C4:
            type = "BDS_REC_IFB_C4";
            break;
        case par_type::BDS_REC_IFB_C5:
            type = "BDS_REC_IFB_C5";
            break;
        case par_type::QZS_REC_IFB_C3:
            type = "QZS_REC_IFB_C3";
            break;
        case par_type::QZS_REC_IFB_C4:
            type = "QZS_REC_IFB_C4";
            break;
        case par_type::QZS_REC_IFB_C5:
            type = "QZS_REC_IFB_C5";
            break;
        case par_type::SAT_IFB_C3:
            type = "GPS_SAT_IFB_C3" + prn;
            break;
        case par_type::SAT_IFB_C4:
            type = "GPS_SAT_IFB_C4" + prn;
            break;
        case par_type::SAT_IFB_C5:
            type = "GPS_SAT_IFB_C5" + prn;
            break;
        case par_type::CLK_SAT:
            type = "CLK_SAT_" + prn;
            break;
        case par_type::CLK15_G_SAT:
            type = "CLK15_G_SAT_" + prn;
            break;
        case par_type::E15CLK_SAT:
            type = "E15CLK_SAT_" + prn;
            break;
        case par_type::C15CLK_SAT:
            type = "C15CLK_SAT_" + prn;
            break;
        case par_type::J15CLK_SAT:
            type = "J15CLK_SAT_" + prn;
            break;
        case par_type::AMB_IF:
            type = "AMB_IF_" + prn;
            break;
        case par_type::AMB13_IF:
            type = "AMB13_IF_" + prn;
            break;
        case par_type::AMB14_IF:
            type = "AMB14_IF_" + prn;
            break;
        case par_type::AMB15_IF:
            type = "AMB15_IF_" + prn;
            break;
        case par_type::AMB_WL:
            type = "AMB_WL_" + prn;
            break;
        case par_type::AMB_L1:
            type = "AMB_L1_" + prn;
            break;
        case par_type::AMB_L2:
            type = "AMB_L2_" + prn;
            break;
        case par_type::AMB_L3:
            type = "AMB_L3_" + prn;
            break;
        case par_type::AMB_L4:
            type = "AMB_L4_" + prn;
            break;
        case par_type::AMB_L5:
            type = "AMB_L5_" + prn;
            break;
        case par_type::FCBS_IF:
            type = "FCBS_IF";
            break;
        case par_type::FCBS_L1:
            type = "FCBS_L1";
            break;
        case par_type::FCBS_L2:
            type = "FCBS_L2";
            break;
        case par_type::FCBS_L3:
            type = "FCBS_L3";
            break;
        case par_type::FCBS_L4:
            type = "FCBS_L4";
            break;
        case par_type::FCBS_L5:
            type = "FCBS_L5";
            break;
        case par_type::FCBR_IF:
            type = "FCBR_IF";
            break;
        case par_type::FCBR_L1:
            type = "FCBR_L1";
            break;
        case par_type::FCBR_L2:
            type = "FCBR_L2";
            break;
        case par_type::FCBR_L3:
            type = "FCBR_L3";
            break;
        case par_type::FCBR_L4:
            type = "FCBR_L4";
            break;
        case par_type::FCBR_L5:
            type = "FCBR_L5";
            break;
        case par_type::CLK_ICB:
            type = "CLK_ICB";
            break;
        case par_type::CLUSTERB:
            type = "CLUSTERB";
            break;
        case par_type::GRD_N:
            type = "GRD_N";
            break;
        case par_type::GRD_E:
            type = "GRD_E";
            break;
        case par_type::GLO_ISB:
            type = "GLO_ISB";
            break;
        case par_type::GLO_ifcb:
            type = "GLO_ifcb";
            break;
        case par_type::GLO_IFPB:
            type = "GLO_IFPB";
            break;
        case par_type::GLO_IFB:
            type = "GLO_IFB_" + prn;
            break;
        case par_type::GPS_IFB_C3:
            type = "GPS_IFB_C3";
            break;
        case par_type::GAL_IFB_C3:
            type = "GAL_IFB_C3";
            break;
        case par_type::BDS_IFB_C3:
            type = "BDS_IFB_C3";
            break;
        case par_type::QZS_IFB_C3:
            type = "QZS_IFB_C3";
            break;
        case par_type::GAL_ISB:
            type = "GAL_ISB";
            break;
        case par_type::BDS_ISB:
            type = "BDS_ISB";
            break;
        case par_type::QZS_ISB:
            type = "QZS_ISB";
            break;
        case par_type::IFB_GPS:
            type = "IFB_GPS";
            break;
        case par_type::IFB_BDS:
            type = "IFB_BDS";
            break;
        case par_type::IFB_GAL:
            type = "IFB_GAL";
            break;
        case par_type::IFB_QZS:
            type = "IFB_QZS";
            break;
        case par_type::IFB_GAL_2:
            type = "IFB_GAL_2";
            break;
        case par_type::IFB_GAL_3:
            type = "IFB_GAL_3";
            break;
        case par_type::IFB_BDS_2:
            type = "IFB_BDS_2";
            break;
        case par_type::IFB_BDS_3:
            type = "IFB_BDS_3";
            break;
        case par_type::P1P2G_REC:
            type = "P1P2G_REC";
            break;
        case par_type::P1P2E_REC:
            type = "P1P2E_REC";
            break;
        case  par_type::RCB_GPS_1:
            type = "RCB_GPS_1";
            break;
        case  par_type::RCB_GPS_2:
            type = "RCB_GPS_2";
            break;
        case  par_type::RCB_BDS_1:
            type = "RCB_BDS_1";
            break;
        case  par_type::RCB_BDS_2:
            type = "RCB_BDS_2";
            break;
        case  par_type::RCB_GAL_1:
            type = "RCB_GAL_1";
            break;
        case  par_type::RCB_GAL_2:
            type = "RCB_GAL_2";
            break;
        case  par_type::RCB_QZS_1:
            type = "RCB_QZS_1";
            break;
        case  par_type::RCB_QZS_2:
            type = "RCB_QZS_2";
            break;
        case par_type::ATT_X:
            type = "ATT_X";
            break;
        case par_type::ATT_Y:
            type = "ATT_Y";
            break;
        case par_type::ATT_Z:
            type = "ATT_Z";
            break;
        case par_type::eb_X:
            type = "eb_X";
            break;
        case par_type::eb_Y:
            type = "eb_Y";
            break;
        case par_type::eb_Z:
            type = "eb_Z";
            break;
        case par_type::db_X:
            type = "db_X";
            break;
        case par_type::db_Y:
            type = "db_Y";
            break;
        case par_type::db_Z:
            type = "db_Z";
            break;
        case par_type::gyro_scale_X:
            type = "gyro_scale_X";
            break;
        case par_type::gyro_scale_Y:
            type = "gyro_scale_Y";
            break;
        case par_type::gyro_scale_Z:
            type = "gyro_scale_Z";
            break;
        case par_type::acce_scale_X:
            type = "acce_scale_X";
            break;
        case par_type::acce_scale_Y:
            type = "acce_scale_Y";
            break;
        case par_type::acce_scale_Z:
            type = "acce_scale_Z";
            break;
        case par_type::IMU_INST_ATT_X:
            type = "IMU_INST_ATT_X";
            break;
        case par_type::IMU_INST_ATT_Y:
            type = "IMU_INST_ATT_Y";
            break;
        case par_type::IMU_INST_ATT_Z:
            type = "IMU_INST_ATT_Z";
            break;
        case par_type::ODO_k:
            type = "db_Z";
            break;
        case par_type::EXTRINSIC_ATT_X:
            type = "EXTRINSIC_ATT_X";
            break;
        case par_type::EXTRINSIC_ATT_Y:
            type = "EXTRINSIC_ATT_Y";
            break;
        case par_type::EXTRINSIC_ATT_Z:
            type = "EXTRINSIC_ATT_Z";
            break;
        case par_type::EXTRINSIC_CRD_X:
            type = "EXTRINSIC_CRD_X";
            break;
        case par_type::EXTRINSIC_CRD_Y:
            type = "EXTRINSIC_CRD_Y";
            break;
        case par_type::EXTRINSIC_CRD_Z:
            type = "EXTRINSIC_CRD_Z";
            break;
        case par_type::CAM_ATT_X:
            type = "CAM_ATT_X";
            break;
        case par_type::CAM_ATT_Y:
            type = "CAM_ATT_Y";
            break;
        case par_type::CAM_ATT_Z:
            type = "CAM_ATT_Z";
            break;
        case par_type::CAM_CRD_X:
            type = "CAM_CRD_X";
            break;
        case par_type::CAM_CRD_Y:
            type = "CAM_CRD_Y";
            break;
        case par_type::CAM_CRD_Z:
            type = "CAM_CRD_Z";
            break;

        case par_type::SCALE:
            type = "SCALE";
            break;
        case par_type::GEOCX:
            type = "GEOCX";
            break;
        case par_type::GEOCY:
            type = "GEOCY";
            break;
        case par_type::GEOCZ:
            type = "GEOCZ";
            break;
        case par_type::XPOLE:
            type = "XPOLE";
            break;
        case par_type::YPOLE:
            type = "YPOLE";
            break;
        case par_type::DXPOLE:
            type = "DXPOLE";
            break;
        case par_type::DYPOLE:
            type = "DYPOLE";
            break;
        case par_type::UT1:
            type = "UT1";
            break;
        case par_type::DUT1:
            type = "DUT1";
            break;
        case par_type::UT1VLBI:
            type = "UT1VLBI";
            break;
        case par_type::NUTDX:
            type = "NUTDX";
            break;
        case par_type::NUTDY:
            type = "NUTDY";
            break;
        case par_type::PXSAT:
            type = "PXSAT_" + prn;
            break;
        case par_type::PYSAT:
            type = "PYSAT_" + prn;
            break;
        case par_type::PZSAT:
            type = "PZSAT_" + prn;
            break;
        case par_type::VXSAT:
            type = "VXSAT_" + prn;
            break;
        case par_type::VYSAT:
            type = "VYSAT_" + prn;
            break;
        case par_type::VZSAT:
            type = "VZSAT_" + prn;
            break;
        case par_type::D0:
            type = "D0_" + prn;
            break;
        case par_type::Dc:
            type = "Dc_" + prn;
            break;
        case par_type::Ds:
            type = "Ds_" + prn;
            break;
        case par_type::Y0:
            type = "Y0_" + prn;
            break;
        case par_type::Yc:
            type = "Yc_" + prn;
            break;
        case par_type::Ys:
            type = "Ys_" + prn;
            break;
        case par_type::B0:
            type = "X0_" + prn;
            break;
        case par_type::Bc:
            type = "Xc_" + prn;
            break;
        case par_type::Bs:
            type = "Xs_" + prn;
            break;
        case par_type::SR_scale:
            type = "SR_scale_" + prn;
            break;
        case par_type::DRAG_c:
            type = "DRAG_c_" + prn;
            break;
        case par_type::EMP_Sc1:
            type = "EMP_Sc1_" + prn;
            break;
        case par_type::EMP_Cc1:
            type = "EMP_Cc1_" + prn;
            break;
        case par_type::EMP_Sa1:
            type = "EMP_Sa1_" + prn;
            break;
        case par_type::EMP_Ca1:
            type = "EMP_Ca1_" + prn;
            break;
        case par_type::EMP_Sr1:
            type = "EMP_Sr1_" + prn;
            break;
        case par_type::EMP_Cr1:
            type = "EMP_Cr1_" + prn;
            break;

        case par_type::EMP_Sc2:
            type = "EMP_Sc2_" + prn;
            break;
        case par_type::EMP_Cc2:
            type = "EMP_Cc2_" + prn;
            break;
        case par_type::EMP_Sa2:
            type = "EMP_Sa2_" + prn;
            break;
        case par_type::EMP_Ca2:
            type = "EMP_Ca2_" + prn;
            break;
        case par_type::EMP_Sr2:
            type = "EMP_Sr2_" + prn;
            break;
        case par_type::EMP_Cr2:
            type = "EMP_Cr2_" + prn;
            break;

        case par_type::EMP_Ac:
            type = "EMP_Ac_" + prn;
            break;
        case par_type::EMP_Bc:
            type = "EMP_Bc_" + prn;
            break;
        case par_type::EMP_Aa:
            type = "EMP_Aa_" + prn;
            break;
        case par_type::EMP_Ba:
            type = "EMP_Ba_" + prn;
            break;
        case par_type::EMP_Ar:
            type = "EMP_Ar_" + prn;
            break;
        case par_type::EMP_Br:
            type = "EMP_Br_" + prn;
            break;
        case par_type::ECOM2_D0:
            type = "ECOM2_D0_" + prn;
            break;
        case par_type::ECOM2_D2c:
            type = "ECOM2_D2c_" + prn;
            break;
        case par_type::ECOM2_D2s:
            type = "ECOM2_D2s_" + prn;
            break;
        case par_type::ECOM2_D4c:
            type = "ECOM2_D4c_" + prn;
            break;
        case par_type::ECOM2_D4s:
            type = "ECOM2_D4s_" + prn;
            break;
        case par_type::ECOM2_Y0:
            type = "ECOM2_Y0_" + prn;
            break;
        case par_type::ECOM2_X0:
            type = "ECOM2_X0_" + prn;
            break;
        case par_type::ECOM2_X1c:
            type = "ECOM2_X1c_" + prn;
            break;
        case par_type::ECOM2_X1s:
            type = "ECOM2_X1s_" + prn;
            break;
        case par_type::ECOMT_T30:
            type = "ECOMT_T30_" + prn;
            break;
        case par_type::ECOMT_T3c2:
            type = "ECOMT_T3c2_" + prn;
            break;
        case par_type::ECOMT_T3s2:
            type = "ECOMT_T3s2_" + prn;
            break;
        case par_type::ECOMT_T3c4:
            type = "ECOMT_T3c4_" + prn;
            break;
        case par_type::ECOMT_T3s4:
            type = "ECOMT_T3s4_" + prn;
            break;
        case par_type::ECOMT_T20:
            type = "ECOMT_T20_" + prn;
            break;
        case par_type::ECOMT_T2c2:
            type = "ECOMT_T2c2_" + prn;
            break;
        case par_type::ECOMT_T2s2:
            type = "ECOMT_T2s2_" + prn;
            break;
        case par_type::ECOMT_T1s2:
            type = "ECOMT_T1s2_" + prn;
            break;
        case par_type::ABW_SP:
            type = "ABW_SP_" + prn;
            break;
        case par_type::ABW_SB:
            type = "ABW_SB_" + prn;
            break;
        case par_type::ABW_PXAD:
            type = "ABW_PXAD_" + prn;
            break;
        case par_type::ABW_PXR:
            type = "ABW_PXR_" + prn;
            break;
        case par_type::ABW_PZAD:
            type = "ABW_PZAD_" + prn;
            break;
        case par_type::ABW_PZR:
            type = "ABW_PZR_" + prn;
            break;
        case par_type::ABW_NZAD:
            type = "ABW_NZAD_" + prn;
            break;
        case par_type::ABW_NZR:
            type = "ABW_NZR_" + prn;
            break;
        case par_type::ABW_Y0:
            type = "ABW_Y0_" + prn;
            break;
        case par_type::RB:
            type = "RB_" + prn;
            break;
        case par_type::RB_GPS:
            type = "RB_GPS";
            break;
        case par_type::RB_GLO:
            type = "RB_GLO";
            break;
        case par_type::RB_GAL:
            type = "RB_GAL";
            break;
        case par_type::RB_BDS:
            type = "RB_BDS";
            break;
        case par_type::RB_PSM4:
            type = "RB_PSM4";
            break;
        case par_type::RB_PSM7:
            type = "RB_PSM7";
            break;
        case par_type::RB_PSM9:
            type = "RB_PSM9";
            break;
        case par_type::RB_all:
            type = "RB_all";
            break;
        case par_type::C_kbr:
            type = "C_kbr";
            break;
        case par_type::C_lri:
            type = "C_lri";
            break;
        case par_type::RA:
            type = "RA";
            break;
        case par_type::DE:
            type = "DE";
            break;
        case par_type::VRA:
            type = "VRA";
            break;
        case par_type::VDE:
            type = "VDE";
            break;
        case par_type::PCV_SAT:
            type = "PCV_SAT";
            break;
        case par_type::PCO_SAT_X:
            type = "PCO_SAT_X";
            break;
        case par_type::PCO_SAT_Y:
            type = "PCO_SAT_Y";
            break;
        case par_type::PCO_SAT_Z:
            type = "PCO_SAT_Z";
            break;
        case par_type::PCV_REC:
            type = "PCV_REC";
            break;
        case par_type::PCO_REC_X:
            type = "PCO_REC_X";
            break;
        case par_type::PCO_REC_Y:
            type = "PCO_REC_Y";
            break;
        case par_type::PCO_REC_Z:
            type = "PCO_REC_Z";
            break;
        default:
            type = "UNDEF";
        }

        return type;
    }

    base_par_head base_par::get_head() const
    {
        return base_par_head(this->parType, this->site, this->prn);
    }

    base_time_arc base_par::get_timearc() const
    {
        return base_time_arc(this->beg, this->end);
    }

    // setting mapping function for ZTD
 // ---------------------------------------------------
    void base_par::setMF(ZTDMPFUNC MF)
    {
        _mf_ztd = MF;
    }

    // setting mapping function for GRD
    // ---------------------------------------------------
    void base_par::setMF(GRDMPFUNC MF)
    {
        _mf_grd = MF;
    }

    // Partial derivatives
    // ----------------------------------------------------
    double base_par::partial(gnss_data_sats& satData, base_time& epoch, Triple ground, gnss_data_obs& gobs)
    {
        double mfw, dmfw, mfh, dmfh;
        mfw = dmfw = mfh = dmfh = 0.0;
        // modified by glfeng : satData.rotmat() is the rotmat Matrix of TR_epoch
        // change to partial_panda because earth rotation matirx
        // Triple unit = satData.dloudx();
        //Vector partial_panda = -(satData.rotmat().transpose() * unit);
        switch (parType)
        {
        case par_type::CRD_X:
            if (satData.site() == this->site)
            {
                //cout << this->str_type() << " " << fixed << setprecision(10) << setw(25) << (value() - satData.satcrd()[0]) / satData.rho() << endl;
                return (value() - satData.satcrd()[0]) / satData.rho();
                // cout << this->str_type() << " " << fixed << setprecision(10) << setw(25) << partial_panda(1) << endl;
                //return partial_panda(1);
            }
            else
                return 0.0;
        case par_type::CRD_Y:
            if (satData.site() == this->site)
            {
                //cout << this->str_type() << " " << fixed << setprecision(10) << setw(25) << (value() - satData.satcrd()[1]) / satData.rho() << endl;
                return (value() - satData.satcrd()[1]) / satData.rho();
                /*  cout << this->str_type() << " " << fixed << setprecision(10) << setw(25) << partial_panda(2) << endl;
          return partial_panda(2);*/
            }
            else
                return 0.0;
        case par_type::CRD_Z:
            if (satData.site() == this->site)
            {
                //cout << this->str_type() << " " << fixed << setprecision(10) << setw(25) << (value() - satData.satcrd()[2]) / satData.rho() << endl;
                return (value() - satData.satcrd()[2]) / satData.rho();
                //cout << this->str_type() << " " << fixed << setprecision(10) << setw(25) << partial_panda(3) << endl;
                //return partial_panda(3);
            }
            else
                return 0.0;
        case par_type::CLK:
        {
            if (satData.site() == this->site)
                return 1.0;
            else
                return 0.0;
        }
        case par_type::CLK_SAT:
        {
            if (satData.sat() == this->prn)
                return -1.0;
            else
                return 0.0;
        }
        case par_type::IFCB_F3:
            if (gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_3 && prn == satData.sat())
                return -1.0;
            else
                return 0.0;
        case par_type::IFCB_F4:
            if (gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_4 && prn == satData.sat())
                return -1.0;
            else
                return 0.0;
        case par_type::IFCB_F5:
            if (gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_5 && prn == satData.sat())
                return -1.0;
            else
                return 0.0;
        case par_type::IFB_C3:
            if (gobs.is_code() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_3)
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_C4:
            if (gobs.is_code() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_4)
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_C5:
            if (gobs.is_code() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_5)
                return 1.0;
            else
                return 0.0;
        case par_type::TRP:
            if (satData.site() == this->site)
            {
                _getmf(satData, ground, epoch, mfw, mfh, dmfw, dmfh);
#ifdef DEBUG
                cout << this->str_type() << " " << scientific << setprecision(10) << setw(25) << mfw << endl;

#endif // DEBUG

                return mfw;
            }
            else
                return 0.0;
        case par_type::SION:
        {
            if (satData.site() == this->site)
            {
                // jdhuang : change from G01_F
                double f1 = satData.frequency(gnss_sys::band_priority(satData.gsys(), FREQ_1));
                double fk = satData.frequency(gobs.band());
                double alfa = 0.0;
                if (gobs.is_phase() && prn == satData.sat())
                {
                    alfa = -(f1 * f1) / (fk * fk);
                }
                if (gobs.is_code() && prn == satData.sat())
                {
                    alfa = (f1 * f1) / (fk * fk);
                }
                return alfa;
            }
            else
                return 0.0;
        }
        case par_type::VION:
        {
            if (satData.site() == this->site)
            {
                double f1 = satData.frequency(gnss_sys::band_priority(satData.gsys(), FREQ_1));
                double fk = satData.frequency(gobs.band());
                double mf = 1.0 / sqrt(1.0 - pow(R_SPHERE / (R_SPHERE + 450000.0) * sin(hwa_pi / 2.0 - satData.ele()), 2));
                double alfa = 0.0;
                if (gobs.is_phase() && prn == satData.sat())
                {
                    alfa = -(f1 * f1) / (fk * fk);
                }
                if (gobs.is_code() && prn == satData.sat())
                {
                    alfa = (f1 * f1) / (fk * fk);
                }
                //cout << "Partial " << satData.sat() << " " << prn << "  " << gobs2str(gobs.gobs()) << "  " << gobs.band() << "  " << alfa << "  " << f1 << "  " << fk << endl;
                return alfa * mf;
            }
            else
                return 0.0;
        }
        case par_type::P1P2G_REC:
        {
            if (satData.site() == this->site)
            {
                double f1 = G01_F;
                double fk = satData.frequency(gobs.band());
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (G02_F * G02_F) / (G01_F * G01_F - G02_F * G02_F);
                FREQ_SEQ freq = gnss_sys::band2freq(satData.gsys(), gobs.band());
                if (satData.gsys() == GPS && gobs.is_code() && (freq == FREQ_1 || freq == FREQ_2))
                {
                    return -alfa * beta;
                }
                else
                    return 0.0;
            }
            else
                return 0.0;
        }
        case par_type::P1P2E_REC:
        {
            if (satData.site() == this->site)
            {
                double f1 = E01_F;
                double fk = satData.frequency(gobs.band());
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (E05_F * E05_F) / (E01_F * E01_F - E05_F * E05_F);
                FREQ_SEQ freq = gnss_sys::band2freq(satData.gsys(), gobs.band());
                if (satData.gsys() == GAL && gobs.is_code() && (freq == FREQ_1 || freq == FREQ_2))
                {
                    return -alfa * beta;
                }
                else
                    return 0.0;
            }
            else
                return 0.0;
        }
        case par_type::GRD_N:
            if (satData.site() == this->site)
            {
                _getmf(satData, ground, epoch, mfw, mfh, dmfw, dmfh);
                if (_mf_grd == GRDMPFUNC::CHEN_HERRING)
                {
                    double sinel = sin(satData.ele());
                    double tanel = tan(satData.ele());
                    double cosaz = cos(satData.azi());
                    return (1.0 / (sinel * tanel + 0.0032)) * cosaz;
                }
                else if (_mf_grd == GRDMPFUNC::TILTING)
                {
                    double cosaz = cos(satData.azi());
                    return dmfw * cosaz;
                }
                else if (_mf_grd == GRDMPFUNC::BAR_SEVER)
                {
                    double tanel = tan(satData.ele());
                    double cosaz = cos(satData.azi());
                    return mfw * (1.0 / tanel) * cosaz;
                }
                else
                    std::cerr << "Grad N mapping function is not set up correctly!!!" << std::endl;
            }
            else
                return 0.0;
        case par_type::GRD_E:
            if (satData.site() == this->site)
            {
                _getmf(satData, ground, epoch, mfw, mfh, dmfw, dmfh);
                if (_mf_grd == GRDMPFUNC::CHEN_HERRING)
                {
                    double sinel = sin(satData.ele());
                    double tanel = tan(satData.ele());
                    double sinaz = sin(satData.azi());
                    return (1.0 / (sinel * tanel + 0.0032)) * sinaz;
                }
                else if (_mf_grd == GRDMPFUNC::TILTING)
                {
                    double sinaz = sin(satData.azi());
                    return dmfw * sinaz;
                }
                else if (_mf_grd == GRDMPFUNC::BAR_SEVER)
                {
                    double tanel = tan(satData.ele());
                    double sinaz = sin(satData.azi());
                    return mfw * (1 / tanel) * sinaz;
                }
                else
                    std::cerr << "Grad E mapping function is not set up correctly!!!" << std::endl;
            }
            else
                return 0.0;

        case par_type::AMB_IF:
            if (satData.site() == this->site && gobs.is_phase() && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::AMB_L1:
            if (satData.site() == this->site && gobs.is_phase() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_1 && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::AMB_L2:
            if (satData.site() == this->site && gobs.is_phase() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_2 && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::AMB_L3:
            if (satData.site() == this->site && gobs.is_phase() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_3 && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::AMB_L4:
            if (satData.site() == this->site && gobs.is_phase() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_4 && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::AMB_L5:
            if (satData.site() == this->site && gobs.is_phase() && gnss_sys::band2freq(satData.gsys(), gobs.band()) == FREQ_5 && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_ISB:
            if (satData.site() == this->site && satData.gsys() == GLO)
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_ifcb:
            if (satData.site() == this->site && !gobs.is_phase() && satData.gsys() == GLO && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_IFPB:
            if (satData.site() == this->site && gobs.is_phase() && satData.gsys() == GLO && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_IFB:
            if (satData.site() == this->site && satData.gsys() == GLO && prn == satData.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GAL_ISB:
            if (satData.site() == this->site && satData.gsys() == GAL)
                return 1.0;
            else
                return 0.0;
        case par_type::BDS_ISB:
            if (satData.site() == this->site && satData.gsys() == BDS)
                return 1.0;
            else
                return 0.0;
        case par_type::QZS_ISB:
            if (satData.site() == this->site && satData.gsys() == QZS)
                return 1.0;
            else
                return 0.0;
        default:
            return 0.0;
        }

        if (parType >= par_type::XPOLE && parType <= par_type::DUT1)
        {
            std::cout << this->str_type() << " " << std::scientific << std::setprecision(15) << std::setw(30) << partial_eop(satData) << std::endl;
            return partial_eop(satData);
        }
        else if (parType >= par_type::PXSAT && parType <= par_type::ECOM2_X1s)
        {
            if (satData.sat() == prn)
            {
                std::cout << this->str_type() << " " << std::scientific << std::setprecision(15) << std::setw(30) << partial_orb(satData) << std::endl;
                return partial_orb(satData);
            }
            else if (satData.site() == site)
            {
                //cout << this->str_type() << " " << scientific << setprecision(15) << setw(30) << partial_orb_leo(satData) << endl;
                return partial_orb_leo(satData);
            }
            else
                return 0.0;
        }

        return 0.0;
    }

    double base_par::partial_doppler(gnss_data_sats& satData, Triple& groundXYZ, Triple& groundVEL)
    {
        Triple satcrd = satData.satcrd();
        Triple satvel = satData.satvel();
        Triple cSat = satcrd;
        Triple vSat = satvel;
        Triple cRec = groundXYZ;
        Triple vRec = groundVEL;
        Triple conf_crd = (cSat - cRec).dot(vSat - vRec) * (cSat - cRec) / pow(satData.rho(), 3);
        conf_crd -= (vSat - vRec) / satData.rho();
        Triple e = (cSat - cRec) / satData.rho();
        satData._conf_crd = conf_crd;
        satData._e = e;
        switch (parType)
        {
        case par_type::CRD_X:
            return conf_crd(1);
        case par_type::CRD_Y:
            return conf_crd(2);
        case par_type::CRD_Z:
            return conf_crd(3);
        case par_type::VEL_X:
            return -e(1);
        case par_type::VEL_Y:
            return -e(2);
        case par_type::VEL_Z:
            return -e(3);
        case par_type::CLK_RAT:
            return 1.0;
        default:
            return 0.0;
        }
        return 0.0;
    }

    double base_par::partial_eop(gnss_data_sats& satdata)
    {

        // get REC crd in trs
        Triple xyz_rec_trs;
        xyz_rec_trs = satdata.reccrd();
        // get unit_vector in TRS
        Triple unit_rec2sat;
        // get
        unit_rec2sat = (satdata.satcrd() - satdata.reccrd());
        // change TRS to CRS
        unit_rec2sat = (satdata.rotmat()) * unit_rec2sat;
        unit_rec2sat = unit_rec2sat / unit_rec2sat.norm();

        // testZHJ
        //cout << setw(20) << setprecision(13) << fixed << xyz_rec_trs;

        // get drotdxpole
        Matrix drdxpole = satdata.drdxpole();
        //drdxpole = drdxpole * xyz_rec_trs;
        drdxpole = drdxpole / RAD2SEC;
        // get drotdypole
        Matrix drdypole = satdata.drdypole();
        //drdypole = drdypole * xyz_rec_trs;
        drdypole = drdypole / RAD2SEC;
        // get drotdut1
        Matrix drdut1 = satdata.drdut1();
        //drdut1 = drdut1 * xyz_rec_trs;
        drdut1 = drdut1 / RAD2TSEC;
        double diff = (satdata.recTime() - beg) / 86400.0;

        // debug ZHJ
        //cout << setw(20) << setprecision(13) << fixed << drdxpole;
        //cout << setw(20) << setprecision(13) << fixed << drdypole;
        //cout << setw(20) << setprecision(13) << fixed << drdut1;
        //cout << setw(20) << setprecision(13) << fixed << unit_rec2sat;

        // dxdxpole dxdypole dxdut1
        Matrix dxdpoleut1(3, 6);
        // dx/xpole
        dxdpoleut1.col(0) = drdxpole;          // dx/xpole
        dxdpoleut1.col(1) = drdxpole * diff;   // dx/dxpole
        dxdpoleut1.col(2) = drdypole;          // dx/ypole
        dxdpoleut1.col(3) = drdypole * diff;   // dx/dypole
        dxdpoleut1.col(4) = drdut1;            // dx/ut1
        dxdpoleut1.col(5) = drdut1 * diff;     // dx/dut1

        Matrix dr_derp(1, 6);
        dr_derp = unit_rec2sat.transpose() * dxdpoleut1;

        //bool debug = false;
        //if (debug) {
        //    for (int i = 0; i < 6; i++) {
        //        dr_derp(1,i + 1) = (*satdata.panda_info)[satdata.site()][satdata.sat()].epo_amat[i];
        //        if (i < 4) {
        //            dr_derp(1, i + 1) = -dr_derp(1, i + 1);
        //        }
        //    }
        //}

        switch (parType)
        {
        case par_type::XPOLE:
            return -dr_derp(0, 0);
            break;
        case par_type::YPOLE:
            return -dr_derp(0, 2);
            break;
        case par_type::DXPOLE:
            return -dr_derp(0, 1);
            break;
        case par_type::DYPOLE:
            return -dr_derp(0, 3);
            break;
        case par_type::UT1:
            return dr_derp(0, 4);
            break;
        case par_type::DUT1:
            //cout << setw(20) << setprecision(13) << fixed << satdata.drdut1();
            //cout << setw(20) << setprecision(13) << fixed << drdut1;
            //cout << setw(20) << setprecision(13) << fixed << dxdpoleut1;
            //cout << setw(20) << setprecision(13) << dr_derp << endl;
            return dr_derp(0, 5);
            break;
        default:
            return 0.0;
            break;
        }
    }

    double base_par::partial_orb(gnss_data_sats& satdata)
    {
        Matrix funct = satdata.orbfunct();

        //cout << setw(20) << setprecision(13) << funct << endl;

        int index_begin = satdata.satindex();

        // get unit_vector in TRS
        Triple unit_rec2sat;
        // get
        unit_rec2sat = (satdata.satcrd() - satdata.reccrd());
        // change TRS to CRS
        unit_rec2sat = (satdata.rotmat()) * unit_rec2sat;
        unit_rec2sat = unit_rec2sat / unit_rec2sat.norm();

        // order: P X0 Y0 Z0 VX0..VZ0 solarPar

        //cout << "Before " << endl << setw(20) << setprecision(13) << scientific << funct << endl;
        funct = funct * unit_rec2sat;
        funct = funct * 1E3;

        //cout << "After " << endl << setw(20) << setprecision(13)<< scientific << funct << endl;
        //cout << "After " << endl << setw(20) << setprecision(13) << scientific << unit_rec2sat << endl;

        //bool debug = false;
        //if (debug) {
        //    for (int i = 0; i < 11; i++) {
        //        funct(i + 1, 1) = (*satdata.panda_info)[satdata.site()][satdata.sat()].orb_amat[i];
        //    }
        //}

        return funct(index - index_begin, 0);
    }

    double base_par::partial_orb_leo(gnss_data_sats& satdata)
    {
        Matrix funct = satdata.orbfunct();
        int index_begin = satdata.satindex();

        // get unit_vector in crs
        Triple unit_rec2sat(3);
        // get
        unit_rec2sat = -satdata.sat2reccrs();

        unit_rec2sat = unit_rec2sat / unit_rec2sat.norm();

        /*cout << endl;
    cout << satdata.epoch().mjd()<<"  "<< satdata.epoch().sod() <<"   u2s  "<<setw(30)<<setprecision(15)<<scientific<< unit_rec2sat << endl;*/
    // order: P X0 Y0 Z0 VX0..VZ0 solarPar
        funct = funct * unit_rec2sat;
        funct = funct * 1E3;

        //cout << setw(20) << setprecision(13)<< scientific << funct << endl;
        return funct(index - index_begin, 0);
        /*switch (parType)
    {
    case par_type::PXSAT:case par_type::PYSAT:case par_type::PZSAT:
    case par_type::VXSAT:case par_type::VYSAT:case par_type::VZSAT:
        return funct(index - index_begin + 1, 1);
        break;
    case par_type::DRAG_c:
        return funct(7, 1);
        break;
    case par_type::EMP_Sc1:
        return funct(8, 1);
        break;
    case par_type::EMP_Cc1:
        return funct(9, 1);
        break;
    case par_type::EMP_Sa1:
        return funct(10, 1);
        break;
    case par_type::EMP_Ca1:
        return funct(11, 1);
        break;
    case par_type::EMP_Sr1:
        return funct(12, 1);
        break;
    case par_type::EMP_Cr1:
        return funct(13, 1);
        break;
    default:
        return 0;

    }*/
    }

    // get ZTD mf according to settings
    void base_par::_getmf(gnss_data_sats& satData,
        const Triple& crd,
        const base_time& epoch,
        double& mfw,
        double& mfh,
        double& dmfw,
        double& dmfh)
    {
        if (parType != par_type::TRP && parType != par_type::GRD_N && parType != par_type::GRD_E)
            return;

        double ele = satData.ele();

        if (_mf_ztd == ZTDMPFUNC::COSZ)
        {
            mfw = mfh = 1.0 / sin(ele);
        }
        else if (_mf_ztd == ZTDMPFUNC::GMF)
        {
            gnss_model_gmf mf;
            mf.gmf(epoch.mjd(), crd[0], crd[1], crd[2], hwa_pi / 2.0 - ele,
                mfh, mfw, dmfh, dmfw);
        }
        else
            std::cerr << "ZTD mapping function is not set up correctly!!!" << std::endl;
    }
} 
