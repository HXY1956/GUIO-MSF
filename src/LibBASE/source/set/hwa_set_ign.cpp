#include "hwa_set_ign.h"
#include "hwa_base_typeconv.h"
using namespace hwa_ins;
using namespace hwa_base;
using namespace std;

namespace hwa_ins {
    UNIT_TYPE str2Unit(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        UNIT_TYPE res = UNDF;
        if (tmp == "RAD")
            res = RAD;
        else if (tmp == "DEG")
            res = DEG;
        else if (tmp == "RPS")
            res = RPS;
        else if (tmp == "DPS")
            res = DPS;
        else if (tmp == "RPH")
            res = RPH;
        else if (tmp == "DPH")
            res = DPH;
        else if (tmp == "MPS")
            res = MPS;
        else if (tmp == "MPS2")
            res = MPS2;
        else
            res = UNDF;
        return res;
    }

    FLT_TYPE str2Flt(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "FORWARD")
            return FORWARD;
        if (tmp == "BACKWARD")
            return BACKWARD;
        if (tmp == "FBS")
            return FBS;
        if (tmp == "RTS")
            return RTS;
        return FORWARD;
    }

    ODR_TYPE str2odr(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "XYZ_XYZ_PRY_NW")
            return XYZ_XYZ_PRY_NW;
        if (tmp == "BLH_ENU_PRY_NW")
            return BLH_ENU_PRY_NW;
        if (tmp == "HOLO_ODO")
            return HOLO_ODO;
        return XYZ_XYZ_PRY_NW;
    }

    ALIGN_TYPE str2align(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "AUTO")
            return ALIGN_TYPE::AUTO;
        if (tmp == "MA")
            return ALIGN_TYPE::MA;
        if (tmp == "POS")
            return ALIGN_TYPE::POS_AGN;
        if (tmp == "VEL")
            return ALIGN_TYPE::VEL_AGN;
        if (tmp == "MULTI_ANT")
            return ALIGN_TYPE::MULTI_ANT;
        if (tmp == "VINS")
            return ALIGN_TYPE::VINS;
        if (tmp == "TRACK")
            return ALIGN_TYPE::TRACK;
        if (tmp == "STATIC")
            return ALIGN_TYPE::STC_AGN;
        if (tmp == "NONE")
            return ALIGN_TYPE::NONE;
        if (tmp == "OFF")
            return ALIGN_TYPE::NONE;
        return STC_AGN;
    }

    CPS_TYPE str2cps(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "POLY")
            return POLY;
        if (tmp == "CONE")
            return CONE;
        if (tmp == "ONE_PLUS_PRE")
            return ONE_PLUS_PRE;
        return POLY;
    }
}

namespace hwa_ins {
    START_ENV str2startenv(std::string str) {
        std::transform(str.begin(), str.end(), str.begin(),
            [](unsigned char c) { return std::toupper(c); });
        if (str == "OUTDOOR") return OUTDOOR;
        else return INDOOR;
    }

    Estimator str2estimator(std::string res) {
        std::transform(res.begin(), res.end(), res.begin(), ::toupper);
        if (res == "NORMAL") return NORMAL;
        if (res == "INEKF") return INEKF;
    }

    MEAS_TYPE ins2meas(const int& i)
    {
        return MEAS_TYPE();
    }
    string meas2str(MEAS_TYPE type)
    {
        string res;

        switch (type)
        {
        case GNSS_MEAS:
            res = "GNSS";
            break;
        case POS_MEAS:
            res = "GNSS_POS";
            break;
        case VEL_MEAS:
            res = "GNSS_VEL";
            break;
        case POS_VEL_MEAS:
            res = "GNSS";
            break;
        case DD_UC_OBS_MEAS:
            break;
        case DD_IF_OBS_MEAS:
            break;
        case FIXED_DD_UC_OBS_MEAS:
            break;
        case MOTION_MEAS:
            break;
        case ZUPT_MEAS:
            res = "ZUPT";
            break;
        case ZIHR_MEAS:
            res = "ZIHR";
            break;
        case NHC_MEAS:
            res = "NHC";
            break;
        case YAW_MEAS:
            res = "YAW";
            break;
        case ODO_MEAS:
            res = "ODO";
            break;
        case Hgt_MEAS:
            res = "HGT";
            break;
        case ATT_MEAS:
            res = "ATT";
            break;
        case ZUPT_POS_MEAS:
            res = "ZUPT";
            break;
        case NO_MEAS:
            break;
        case VIS_MEAS:
            res = "VIS";
            break;
        case DEFAULT_MEAS:
            break;
        case LIDAR_MEAS:
            res = "LIDAR";
            break;
        case UWB_MEAS:
            res = "UWB";
            break;
        case R_MIMU_MEAS:
            res = "MIMU ROT";
            break;
        case P_MIMU_MEAS:
            res = "MIMU TRANS";
            break;
        default:
            break;
        }
        return res;
    }
    LCI_TYPE str2lci(const string& s)
    {
        string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "SPP")
            return LCI_TYPE::SPP;
        if (tmp == "PPP")
            return LCI_TYPE::PPP;
        if (tmp == "RTK")
            return LCI_TYPE::RTK;
        if (tmp == "PPPRTK")
            return LCI_TYPE::PPPRTK;
        return LCI_TYPE::SPP;
    }
    Estimator str2fuse(const string& s) {
        string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "INEKF")
            return INEKF;
        if (tmp == "NORMAL")
            return NORMAL;
        return NORMAL;
    }
    string fuse2str(const Estimator s) {
        if (s == INEKF)
            return "InEKF";
        if (s == NORMAL)
            return "Normal";
    }
    IGN_TYPE str2ign(const string& s)
    {
        string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        if (tmp == "VIO_TCI")
            return VIO_TCI;
        if (tmp == "VIO_LCI")
            return VIO_LCI;
        if (tmp == "LIO_TCI")
            return LIO_TCI;
        if (tmp == "LIO_LCI")
            return LIO_LCI;
        if (tmp == "LVI_TCI")
            return LVI_TCI;
        if (tmp == "GI_LCI")
            return GI_LCI;
        if (tmp == "GI_TCI")
            return GI_TCI;
        if (tmp == "GI_STCI")
            return GI_STCI;
        if (tmp == "UI_LCI")
            return UI_LCI;
        if (tmp == "UI_TCI")
            return UI_TCI;
        if (tmp == "UVI_TCI")
            return UVI_TCI;
        if (tmp == "ULI_TCI")
            return ULI_TCI;
        if (tmp == "ULVI_TCI")
            return ULVI_TCI;
        if (tmp == "GUI_TCI")
            return GUI_TCI;
        if (tmp == "GVI_TCI")
            return GVI_TCI;
        if (tmp == "GUVI_TCI")
            return GUVI_TCI;
        if (tmp == "GLVI_TCI")
            return GLVI_TCI;
        if (tmp == "GULVI_TCI")
            return GULVI_TCI;

        return PURE_INS;
    }
}

namespace hwa_set{
    set_ign::set_ign() : set_base()
    {
        _set.insert(XMLKEY_IGN);

        _initial_misalignment_std = Triple(1, 1, 10);
        _initial_vel_std = 0.1 * Triple::Ones();
        _initial_pos_std = 3 * Triple::Ones();
        _initial_gyro_std = 10 * Triple::Ones();
        _initial_acce_std = 5 * Triple::Ones();
        _initial_gyro_scale_std = 0.01 * Triple::Ones();
        _initial_acce_scale_std = 0.01 * Triple::Ones();
        _initial_odoscale_std = 0.1;

        _min_misalignment_std = 0.1 * Triple::Ones();
        _min_vel_std = 0.005 * Triple::Ones();
        _min_pos_std = 0.01 * Triple::Ones();
        _min_gyro_std = 0.1 * Triple::Ones();
        _min_acce_std = 0.1 * Triple::Ones();
        _odo_std = 0.5;

        _misalignment_psd = 1 * Triple::Ones();
        _vel_psd = 0.1 * Triple::Ones();
        _pos_psd = 0 * Triple::Ones();
        _gyro_psd = 0 * Triple::Ones();
        _acce_psd = 0 * Triple::Ones();
        _gyro_scale_psd = 0 * Triple::Ones();
        _acce_scale_psd = 0 * Triple::Ones();
        _odo_psd = 0.1;

        _meas_vel_noise = Triple(0.5, 0.5, 0.5);
        _meas_pos_noise = Triple(0.5, 0.5, 0.5);

        _nq = 15;
        _nr = 6;
        _lever = _odo_lever = _uwb_lever = Triple::Zero();
        _max_pdop = 8.0;
        _min_sat = 5;
        _order = "XYZXYZ";
        _TS = 1.0;
        _delay_t = 0.0;
        _delay_odo = 0.001;
        _fltmode = FORWARD;
        _odo = "OFF";
        _NHC = _ZUPT = false;
        //_imu_type = IMU_TYPE::StarNeto;
    }

    set_ign::~set_ign()
    {
    }

    void set_ign::check()
    {
        xml_node node = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN);

        string str = to_string(_initial_misalignment_std(0)) + ' ' + to_string(_initial_misalignment_std(1)) + ' ' + to_string(_initial_misalignment_std(2));
        _default_attr(node, "initial_misalignment_std", str);

        str.clear();
        str = "";
        str = to_string(_initial_vel_std(0)) + ' ' + to_string(_initial_vel_std(1)) + ' ' + to_string(_initial_vel_std(2));
        _default_attr(node, "initial_vel_std", str);

        str.clear();
        str = "";
        str = to_string(_initial_pos_std(0)) + ' ' + to_string(_initial_pos_std(1)) + ' ' + to_string(_initial_pos_std(2));
        _default_attr(node, "initial_pos_std", str);

        _default_attr(node, "initial_gyro_std", _initial_gyro_std(0));
        _default_attr(node, "initial_acce_std", _initial_acce_std(0));
        _default_attr(node, "initial_gyro_scale_std", _initial_gyro_scale_std(0));
        _default_attr(node, "initial_acce_scale_std", _initial_acce_scale_std(0));
        _default_attr(node, "initial_odo_k_std", _initial_odoscale_std);
        _default_attr(node, "odo_std", _odo_std);
        _default_attr(node, "nhc_std", _odo_std);
        _default_attr(node, "zupt_std", _odo_std);
        _default_attr(node, "ZAUPT_std", _odo_std);

        _default_attr(node, "min_misalignment_std", _min_misalignment_std(0));
        _default_attr(node, "min_vel_std", _min_vel_std(0));
        _default_attr(node, "min_pos_std", _min_pos_std(0));
        _default_attr(node, "min_gyro_std", _min_gyro_std(0));
        _default_attr(node, "min_acce_std", _min_acce_std(0));

        _default_attr(node, "misalignment_psd", _misalignment_psd(0));
        _default_attr(node, "vel_psd", _vel_psd(0));
        _default_attr(node, "pos_psd", _pos_psd(0));
        _default_attr(node, "gyro_psd", _gyro_psd(0));
        _default_attr(node, "acce_psd", _acce_psd(0));
        _default_attr(node, "gyro_scale_psd", _gyro_scale_psd(0));
        _default_attr(node, "acce_scale_psd", _acce_scale_psd(0));
        _default_attr(node, "odo_psd", _odo_psd);

        _default_attr(node, "hor_vel_noise", _meas_vel_noise(0));
        _default_attr(node, "ver_vel_noise", _meas_vel_noise(2));
        _default_attr(node, "hor_pos_noise", _meas_pos_noise(0));
        _default_attr(node, "ver_pos_noise", _meas_pos_noise(2));

        str.clear();
        str = "";
        str = to_string(_lever(0)) + ' ' + to_string(_lever(1)) + ' ' + to_string(_lever(2));
        _default_attr(node, "lever", str);
        _default_attr(node, "odo_lever", str);
        _default_attr(node, "uwb_lever", str);

        _default_attr(node, "nq", _nq);
        _default_attr(node, "nr", _nr);
        _default_attr(node, "order", _order);
        _default_attr(node, "TS", _TS);
        _default_attr(node, "delay_t", _delay_t);
        _default_attr(node, "delay_odo", _delay_odo);
        _default_attr(node, "filtermode", _fltmode);
        _default_attr(node, "max_pdop", _max_pdop);
        _default_attr(node, "min_satnum", _min_sat);
        _default_attr(node, "odometry", _odo);
        _default_attr(node, "NHC", _NHC);
        _default_attr(node, "ZUPT", _ZUPT);
        _default_attr(node, "UWB", false);
        _default_attr(node, "Hgt", false);
    }

    void set_ign::help()
    {
        // no Implement
    }

    bool set_ign::align()
    {
        std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Alignment").attribute("Type").value();
        bool x = false;
        if (res != "OFF")x = true;
        return x;
    }

    ALIGN_TYPE set_ign::align_type()
    {
        std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Alignment").attribute("Type").value();
        return hwa_ins::str2align(res);
    }

    double set_ign::align_time()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Alignment").child("CoarseAlignTime").attribute("Value").as_double();
        return res;
    }

    std::string set_ign::start_env()
    {
        std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Alignment").child("StartEnv").attribute("Value").value();
        str_erase(res);
        std::transform(res.begin(), res.end(), res.begin(), ::toupper);
        if (res == "") res = "OUTDOOR";
        return res;
    }

    double set_ign::pos_dist()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Alignment").child("PositionVector").attribute("Value").as_double();
        return res;
    }

    double set_ign::vel_norm()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Alignment").child("VelocityVector").attribute("Value").as_double();
        return res;
    }

    Triple set_ign::initial_gyro_scale_std()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUScale").child("GyroScale").attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::initial_acce_scale_std()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUScale").child("AcceScale").attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::initial_imu_installation_att_std()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUInstallation").child("Rotation").attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    double set_ign::initial_odoscale_std()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        double res = ExtraStatesNode.child("OdometerScale").attribute("InitialSTD").as_double();
        return res;
    }

    Triple set_ign::min_misalignment_std()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_misalignment_std").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::min_vel_std()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_vel_std").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::min_pos_std()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_pos_std").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::min_gyro_std()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_gyro_std").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::min_acce_std()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_acce_std").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    double set_ign::min_odo_std()
    {
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_odo_k_std").as_double();
        return x;
    }

    Triple set_ign::gyro_scale_psd()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUScale").child("GyroScale").attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::acce_scale_psd()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUScale").child("AcceScale").attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::imu_inst_trans_psd()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUInstallation").child("Translation").attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    Triple set_ign::imu_inst_rot_psd()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUInstallation").child("Rotation").attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        return Triple(X, Y, Z);
    }

    double set_ign::odo_scale()
    {
        xml_node OdometerNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer");
        double x = OdometerNode.child("Scale").attribute("Value").as_double();
        return x;
    }

    double set_ign::odo_psd()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        double x = ExtraStatesNode.child("OdometerScale").attribute("ProcNoiseSD").as_double();
        return x;
    }

    double set_ign::odo_std()
    {
        xml_node OdometerNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer");
        double x = OdometerNode.child("NoiseSD").attribute("Value").as_double();
        return x;
    }

    Triple set_ign::nhc_std()
    {
        xml_node NHCNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("NHC");
        double x = NHCNode.child("NoiseSD").attribute("Value").as_double();
        return Triple(x, x, x);
    }

    Triple set_ign::zupt_std()
    {
        xml_node ZUPTNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ZUPT");
        double x = ZUPTNode.child("VelNoiseSD").attribute("Value").as_double();
        return Triple(x, x, x);
    }

    double set_ign::zupt_dstd()
    {
        xml_node ZUPTNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ZUPT");
        double x = ZUPTNode.child("VelNoiseSD").attribute("Value").as_double();
        return x;
    }

    double set_ign::zihr_std()
    {
        xml_node ZUPTNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ZUPT");
        double x = ZUPTNode.child("ZIHRNoiseSD").attribute("Value").as_double();
        return x * glv.deg;
    }

    double set_ign::Yaw_std()
    {
        xml_node ZUPTNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ZUPT");
        double x = ZUPTNode.child("YawNoiseSD").attribute("Value").as_double();
        return x * glv.deg;
    }

    Triple set_ign::att_std()
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Attitude");
        string x = AttNode.child("NoiseSD").attribute("Value").value();
        for (int i = 0; i < x.size(); i++)
        {
            if (x[i] == ',')x[i] = ' ';
        }
        stringstream ss(x);
        double p, r, y;
        ss >> p >> r >> y;
        return Triple(p, r, y) * glv.deg;
    }

    UNIT_TYPE set_ign::AttUnit()
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Attitude");
        string x = AttNode.child("Unit").attribute("Value").value();
        if (x == "deg")return UNIT_TYPE::DEG;
        if (x == "rad")return UNIT_TYPE::RAD;
        else
            return UNIT_TYPE::DEG;
    }

    Triple set_ign::meas_vel_noise()
    {
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("hor_vel_noise").as_double();
        double y = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("ver_vel_noise").as_double();
        return Triple(x, x, y);
    }

    Triple set_ign::meas_pos_noise()
    {
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("hor_pos_noise").as_double();
        double y = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("ver_pos_noise").as_double();
        return Triple(x, x, y);
    }

    string set_ign::order()
    {
        string x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("order").value();
        transform(x.begin(), x.end(), x.begin(), ::toupper);
        return x;
    }

    int set_ign::nq()
    {
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("nq").as_double();
        return x;
    }

    int set_ign::nr()
    {
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("nr").as_double();
        return x;
    }

    double set_ign::delay_odo()
    {
        //double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("delay_t").as_double();
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer").child("DelayTime").attribute("Value").as_double();
        return x;
    }

    Triple set_ign::odo_lever()
    {
        // IMU lever from imu center to wheel
        xml_node lever = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer").child("Installation").child("Lever");
        double lx = lever.attribute("X").as_double();
        double ly = lever.attribute("Y").as_double();
        double lz = lever.attribute("Z").as_double();
        return Triple(lx, ly, lz);
    }

    Triple set_ign::uwb_lever()
    {
        xml_node lever = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("UltraWideBand").child("AntennaLever");
        double lx = lever.attribute("X").as_double();
        double ly = lever.attribute("Y").as_double();
        double lz = lever.attribute("Z").as_double();
        return Triple(lx, ly, lz);
    }

    IGN_TYPE set_ign::_ign_type_()
    {
        string x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("Type").value();
        return str2ign(x);
    }

    Estimator set_ign::fuse_type()
    {
        string x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("FuseType").value();
        return str2fuse(x);
    }

    double set_ign::inflation()
    {
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("inflation").as_double();
        return x;
    }

    bool set_ign::static_flag()
    {
        bool x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("static").as_bool();
        return x;
    }

    map<double, int> set_ign::sim_gnss_outages()
    {
        map<double, int> outages;
        xml_node SimOutages = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child("SimOutages");
        string beg = SimOutages.attribute("beg").value();
        string len = SimOutages.attribute("length").value();
        stringstream begss(beg), lenss(len);
        double begt;
        int length;
        while (begss >> begt)
        {
            lenss >> length;
            outages.insert(make_pair(begt, length));
        }
        return outages;
    }

    Triple set_ign::gnss_lever()
    {
        //string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("lever").value();
        //for (int i = 0; i < str.size(); i++)
        //{
        //    if (str[i] == ',')
        //        str[i] = ' ';
        //}
        //stringstream ss(str);
        //double lx, ly, lz;
        //ss >> lx >> ly >> lz;
        xml_node LeverNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child("AntennaLever");
        string type = LeverNode.attribute("Type").value();
        double X = LeverNode.attribute("X").as_double();
        double Y = LeverNode.attribute("Y").as_double();
        double Z = LeverNode.attribute("Z").as_double();
        if (type == "RFU")
        {
            // X; Y; Z;
        }
        return Triple(X, Y, Z);
    }

    double set_ign::TS()
    {
        //double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("TS").as_double();
        double freq = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child("Frequency").attribute("Value").as_double();
        return 1.0 / freq;
    }

    double set_ign::TS_odo()
    {
        //double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("TS").as_double();
        double freq = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer").child("Frequency").attribute("Value").as_double();
        return 1.0 / freq;
    }

    double set_ign::delay_t()
    {
        //double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("delay_t").as_double();
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child("DelayTime").attribute("Value").as_double();
        return x;
    }

    double set_ign::max_pdop()
    {
        //double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("max_pdop").as_double();
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child("LCISetting").attribute("MaxPDOP").as_double();
        return x;
    }

    int set_ign::min_sat()
    {
        //int x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("min_satnum").as_int();
        double x = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child("LCISetting").attribute("MinSat").as_double();
        return x;
    }

    FLT_TYPE set_ign::fltmode()
    {
        xml_node EstimatorNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Estimator");
        string str = EstimatorNode.attribute("Type").value();
        return str2Flt(str);
    }

    bool set_ign::Odo()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    string set_ign::odo_inst()
    {
        xml_node OdometerNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer");
        string str = OdometerNode.child("Installation").attribute("Type").value();
        return str;
    }

    bool set_ign::NHC()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("NHC").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::ZUPT()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ZUPT").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::Attitude()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Attitude").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::Direct()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Attitude").child("mode").attribute("Type").value();
        bool direct = false;
        if (str == "Direct") direct = true;
        return direct;
    }

    bool set_ign::GNSS()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::VISION()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Vision").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::LIDAR()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Lidar").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::UWB()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("UltraWideBand").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::Hgt()
    {
        string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Hgt").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::imu_scale()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUScale").attribute("Type").value();
        bool b = false;
        if (str == "ON")b = true;
        return b;
    }

    bool set_ign::imu_inst_rot()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUInstallation").attribute("Type").value();
        bool b = false;
        if (str == "Rotation" || str == "TransRot")b = true;
        return b;
    }

    bool set_ign::imu_inst_trans()
    {
        xml_node ExtraStatesNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("ExtraStates");
        string str = ExtraStatesNode.child("IMUInstallation").attribute("Type").value();
        bool b = false;
        if (str == "Translation" || str == "TransRot")b = true;
        return b;
    }

    double set_ign::wheelraduis()
    {
        xml_node OdometerNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Odometer");
        double d = OdometerNode.child("WheelRadius").attribute("Value").as_double();
        return d;
    }

    double set_ign::UWB_start()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("UltraWideBand").child("Time").attribute("Start").as_double();
        return res;
    }

    double set_ign::UWB_end()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("UltraWideBand").child("Time").attribute("End").as_double();
        return res;
    }

    double set_ign::Hgt_start()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Hgt").child("Time").attribute("Start").as_double();
        return res;
    }

    double set_ign::Hgt_end()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Hgt").child("Time").attribute("End").as_double();
        return res;
    }

    double set_ign::Hgt_std()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Hgt").child("STD").attribute("Value").as_double();
        return res;
    }

    double set_ign::Hgt_info()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("Hgt").attribute("Value").as_double();
        return res;
    }

    double set_ign::att_precision_L()
    {
        string str = _doc.child(XMLKEY_ROOT).child("integration").child("Attitude").child("precision_L").attribute("Value").value();
        double precision_L = base_type_conv::str2dbl(str);
        return precision_L;
    }

    map<string, Triple> set_ign::MultiAntLever()
    {
        int i = 1;
        map<string, Triple> levers;
        set<string> sites = set_base::_setvals("gen", "rover");
        set<string> rover_sites = set_base::_setvals("gen", "rover");
        if (rover_sites.size() > 0)
        {
            sites = set<string>(rover_sites.begin(), rover_sites.end());
        }
        for (auto it : sites)
        {
            string Ant = "AntennaLever" + to_string(i);
            xml_node LeverNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).child("GNSS").child(Ant.c_str());
            string name = LeverNode.attribute("Name").value();
            if (name != it)
            {
                return map<string, Triple>();
            }
            string type = LeverNode.attribute("Type").value();
            if (type == "RFU")
            {
                // X; Y; Z;
            }
            double X = LeverNode.attribute("X").as_double();
            double Y = LeverNode.attribute("Y").as_double();
            double Z = LeverNode.attribute("Z").as_double();
            levers.insert(make_pair(name, Triple(X, Y, Z)));
            ++i;
        }
        return levers;
    }

    map<MEAS_TYPE, double> set_ign::max_norm()
    {
        map<MEAS_TYPE, double> map_norm;
        xml_node ign = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN);
        xml_node::iterator itnode = ign.begin();
        while (itnode != ign.end())
        {
            string name = itnode->name();
            MEAS_TYPE meas = xmlname2meas(name);
            if (meas != MEAS_TYPE::DEFAULT_MEAS)
            {
                double norm = 3.0;
                xml_node node = itnode->child("MaxNorm");
                if (!node.empty())
                    norm = node.attribute("Value").as_double();
                map_norm.insert(make_pair(meas, norm));
            }
            ++itnode;
        }
        return map_norm;
    }

    MEAS_TYPE set_ign::xmlname2meas(string str)
    {
        if (str == "GNSS")
            return MEAS_TYPE::GNSS_MEAS;
        else if (str == "Odometer")
            return MEAS_TYPE::ODO_MEAS;
        else if (str == "NHC")
            return MEAS_TYPE::NHC_MEAS;
        else if (str == "ZUPT")
            return MEAS_TYPE::ZUPT_MEAS;
        else if (str == "Attitude")
            return MEAS_TYPE::ATT_MEAS;
        else
            return MEAS_TYPE::DEFAULT_MEAS;
    }
}

