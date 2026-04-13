#include "hwa_set_ins.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_typeconv.h"
using namespace hwa_set;
using namespace hwa_ins;
double StartTime = 0;

namespace hwa_ins {
    IMU_FUSION_TYPE str2fusiontype(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "STACK")
            return IMU_FUSION_TYPE::STACK;
        if (tmp == "VIRTUAL")
            return IMU_FUSION_TYPE::VIRTUAL;
        return IMU_FUSION_TYPE::DEFAULT;
    }

    IMU_TYPE str2imu(const std::string& s)
    {
        std::string tmp = s;
        //transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "SPAN FSAS")
            return IMU_TYPE::NovAtel_SPAN_FSAS;
        if (tmp == "SPAN CPT")
            return IMU_TYPE::NovAtel_SPAN_CPT;
        if (tmp == "SPAN uIRS")
            return IMU_TYPE::NovAtel_SPAN_uIRS;
        if (tmp == "SPAN LCI100C")
            return IMU_TYPE::NovAtel_SPAN_LCI100C;
        if (tmp == "Navigation Grade")
            return IMU_TYPE::Navigation_Grade;
        if (tmp == "Tactical Grade")
            return IMU_TYPE::Tactical_Grade;
        if (tmp == "MEMS Grade")
            return IMU_TYPE::MEMS_Grade;
        if (tmp == "ADIS 16470")
            return IMU_TYPE::ADIS16470;
        if (tmp == "ADIS 16488")
            return IMU_TYPE::NovAtel_SPAN_ADIS16488;
        if (tmp == "StarNeto")
            return IMU_TYPE::StarNeto;
        if (tmp == "Customize")
            return IMU_TYPE::Customize;
        if (tmp == "MEMS Grade")
            return IMU_TYPE::MEMS_Grade;
        if (tmp == "MEMS Grade")
            return IMU_TYPE::MEMS_Grade;
        return IMU_TYPE::Customize;
    }

    std::map<IMU_TYPE, ERROR_MODEL> imuerror_models()
    {
        IMU_TYPE imu_type; std::map<IMU_TYPE, ERROR_MODEL> map_imu_error_models;

        imu_type = IMU_TYPE::NovAtel_SPAN_FSAS;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(0.324, 0.324, 0.324);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(2.000e-002, 2.000e-002, 5.000e-002) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(6.350, 6.350, 6.350) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(2.500e-007, 2.500e-007, 8.100e-007) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.000e-006, 1.000e-006, 1.000e-006) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(3.240e-012, 3.240e-012, 3.240e-012) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(9.000e-010, 9.000e-010, 9.000e-010) / glv.mgpsh;


        imu_type = IMU_TYPE::NovAtel_SPAN_CPT;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(1.99999800e+001, 1.99999800e+001, 1.99999800e+001);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(5.00000000e-002, 5.00000000e-002, 5.00000000e-002) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(9.80100000e+001, 9.80100000e+001, 9.80100000e+001) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(4.68722500e-008, 4.68722500e-008, 4.68722500e-008) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-008, 1.00000000e-008, 1.00000000e-008) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(7.71595062e-012, 7.71595062e-012, 7.71595062e-012) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(4.38906250e-009, 4.38906250e-009, 4.38906250e-009) / glv.mgpsh;


        imu_type = IMU_TYPE::NovAtel_SPAN_uIRS;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(3.60000000e-003, 3.60000000e-003, 3.60000000e-003);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(3.00000000e-004, 3.00000000e-004, 3.00000000e-004) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(1.00000080e-001, 1.00000080e-001, 1.00000080e-001) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(1.00000000e-006, 1.00000000e-006, 1.00000000e-006) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-006, 1.00000000e-006, 1.00000000e-006) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(9.99998950e-012, 9.99998950e-012, 9.99998950e-012) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(9.99799104e-014, 9.99799104e-014, 9.99799104e-014) / glv.mgpsh;


        imu_type = IMU_TYPE::NovAtel_SPAN_LCI100C;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(3.24000000e-001, 3.24000000e-001, 3.24000000e-001);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(2.00000000e-002, 2.00000000e-002, 2.00000000e-002) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(1.00000160e+000, 1.00000160e+000, 1.00000160e+000) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(1.00000000e-006, 1.00000000e-006, 1.00000000e-006) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-008, 1.00000000e-008, 1.00000000e-008) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(9.99998950e-016, 9.99998950e-016, 9.99998950e-016) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(2.50000000e-009, 2.50000000e-009, 2.50000000e-009) / glv.mgpsh;


        imu_type = IMU_TYPE::Navigation_Grade;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(0.01, 0.01, 0.01);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(0.0003, 0.0003, 0.0003) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(1.0, 1.0, 1.0) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(1.0e-6, 1.0e-6, 1.0e-6) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-006, 1.00000000e-006, 1.00000000e-006) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(1.0e-11, 1.0e-11, 1.0e-11) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(1.0e-11, 1.0e-11, 1.0e-11) / glv.mgpsh;


        imu_type = IMU_TYPE::Tactical_Grade;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(0.1, 0.1, 0.1);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(0.003, 0.003, 0.003) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(20.0, 20.0, 20.0) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(1.0e-6, 1.0e-6, 1.0e-6) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-006, 1.00000000e-006, 1.00000000e-006) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(1.0e-7, 1.0e-7, 1.0e-7) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(1.0e-7, 1.0e-7, 1.0e-7) / glv.mgpsh;


        imu_type = IMU_TYPE::MEMS_Grade;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(1, 1, 10);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(100, 100, 100);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(0.3, 0.3, 0.3) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(100000, 100000, 100000) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(1.0e-6, 1.0e-6, 1.0e-6) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-006, 1.00000000e-006, 1.00000000e-006) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(1.0e-4, 1.0e-4, 1.0e-4) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(1.0e-4, 1.0e-4, 1.0e-4) / glv.mgpsh;


        imu_type = IMU_TYPE::NovAtel_SPAN_ADIS16488;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(7.20000000e+002, 7.20000000e+002, 7.20000000e+002);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(5.00000000e-002, 5.00000000e-002, 5.00000000e-002) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(3.24000000e+002, 3.24000000e+002, 3.24000000e+002) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(4.68722500e-006, 4.68722500e-006, 4.68722500e-006) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-004, 1.00000000e-004, 1.00000000e-004) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(9.56014618e-003, 9.56014618e-003, 9.56014618e-003) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(3.16406250e-009, 3.16406250e-009, 3.16406250e-009) / glv.mgpsh;


        imu_type = IMU_TYPE::ADIS16470;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(1.20000000e+002, 1.20000000e+002, 1.20000000e+002);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(5.00000000e-002, 5.00000000e-002, 5.00000000e-002) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(1.74000000e+001, 1.74000000e+001, 1.74000000e+001) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(0.68722500e-003, 0.68722500e-003, 0.68722500e-003) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.00000000e-004, 1.00000000e-004, 1.00000000e-004) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(8.56014618e-002, 8.56014618e-002, 8.56014618e-002) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(3.16406250e-004, 3.16406250e-004, 3.16406250e-004) / glv.mgpsh;


        imu_type = IMU_TYPE::StarNeto;
        map_imu_error_models.insert(std::make_pair(imu_type, ERROR_MODEL()));
        map_imu_error_models[imu_type].AttInitialSTD = Triple(0.5, 0.5, 5);
        map_imu_error_models[imu_type].VelInitialSTD = Triple(0.5, 0.5, 0.5);
        map_imu_error_models[imu_type].PosInitialSTD = Triple(3, 3, 3);

        map_imu_error_models[imu_type].GyroBiasInitialSTD = Triple(1, 1, 1);
        map_imu_error_models[imu_type].AcceBiasInitialSTD = Triple(1.000e-002, 1.000e-002, 1.000e-002) / glv.mg;

        map_imu_error_models[imu_type].AttProcNoisePSD = Triple(16.350, 16.350, 16.350) * glv.mpsh;
        map_imu_error_models[imu_type].VelProcNoisePSD = Triple(1.600e-003, 1.600e-003, 1.600e-003) / glv.mgpsHz;
        map_imu_error_models[imu_type].PosProcNoisePSD = Triple(1.000e-006, 1.000e-006, 1.000e-006) / glv.mpsh;
        map_imu_error_models[imu_type].GyroBiasProcNoisePSD = Triple(3.240e-012, 3.240e-012, 3.240e-012) / glv.mpsh;
        map_imu_error_models[imu_type].AcceBiasProcNoisePSD = Triple(9.000e-010, 9.000e-010, 9.000e-010) / glv.mgpsh;

        return map_imu_error_models;
    }
}

namespace hwa_set {
    set_ins::set_ins() : set_base()
    {
        _set.insert(XMLKEY_INS);
        _freq = 200;
        _ts = 1.0 / _freq;
        _pos = _vel = _att = Triple::Zero();
        _align_time = 5 * 60;
        _GyroUnit = "DPS";
        _AcceUnit = "MPS2";
        _cps = "POLY";
        _subsample = 1;
        _order = "garfu";
        _out_order = "XYZ_XYZ_PRY_NW";
        _out_freq = 1;
        _int_sec = true;
        _align_type = "SA";
        _start = 0.0;
        _end = _start + 7 * 24 * 60 * 60;
        _acce_bias = Triple::Zero();
        _gyro_bias = Triple::Zero();
    }

    set_ins::~set_ins()
    {
    }

    void set_ins::check()
    {
        xml_node parent = _doc.child(XMLKEY_ROOT);
        xml_node node = _default_node(parent, XMLKEY_INS);

        _imu_type = hwa_ins::str2imu(_doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("ImuErrorModel").attribute("Type").value());
        _map_imu_error_models = imuerror_models();

        _default_attr(node, "compensation", _cps);
        _default_attr(node, "subsample", _subsample);
        _default_attr(node, "order", _order);
        _default_attr(node, "out_freq", _out_freq);
        _default_attr(node, "int_sec", _int_sec);
        _default_attr(node, "out_order", _out_order);
        _default_attr(node, "start", _start);
        _default_attr(node, "end", _end);
        _default_attr(node, "imu_scale", false);

        node = node.child(XMLKEY_INS_AUXILIARY);
        _default_attr(node, "num", 0);
        _default_attr(node, "estimate_imui_extrinsic", true);
        _default_attr(node, "estimate_imui_t", false);
        _default_attr(node, "initial_extrinsic_rotation_cov", "3.0462e-4,3.0462e-4,3.0462e-4");
        _default_attr(node, "initial_extrinsic_translation_cov", "1e-4,1e-4,1e-4");
        _default_attr(node, "initial_t_cov", "0.1");
    }

    void set_ins::help()
    {
        std::cerr << " <ins \n"
            << "   ts=\"" << _ts << "\" \n"
            << "   gyro_unit=\"" << _GyroUnit << "\" \n"
            << "   acce_unit=\"" << _AcceUnit << "\" \n"

            << "  />\n";
    }

    std::string set_ins::order()
    {
        std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("AxisOrder").attribute("Type").value();
        std::transform(res.begin(), res.end(), res.begin(), ::tolower);
        return res;
    }

    int set_ins::resampled_freq()
    {
        int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("Resample").attribute("Value").as_int();
        return res;
    }

    UNIT_TYPE set_ins::GyroUnit()
    {
        std::string gyrounit = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("GyroUnit").attribute("Type").value();
        return hwa_ins::str2Unit(gyrounit);
    }

    UNIT_TYPE set_ins::AcceUnit()
    {
        std::string acceunit = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("AcceUnit").attribute("Type").value();
        return hwa_ins::str2Unit(acceunit);
    }

    UNIT_TYPE set_ins::AttUnit()
    {
        std::string attunit = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("AttUnit").attribute("Type").value();
        return hwa_ins::str2Unit(attunit);
    }

    int set_ins::freq()
    {
        int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("Frequency").attribute("Value").as_int();
        return res;
    }

    double set_ins::ts()
    {
        _freq = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("DataFormat").child("Frequency").attribute("Value").as_int();
        double res = 1.0 / _freq;
        return res;
    }

    UNIT_TYPE set_ins::MagUnit()
    {
        std::string acceunit = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).attribute("mag_unit").value();
        return hwa_ins::str2Unit(acceunit);
    }

    ODR_TYPE set_ins::out_order()
    {
        std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).attribute("out_order").value();
        return hwa_ins::str2odr(res);
    }

    Triple set_ins::pos()
    {
        xml_node PosNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("InitialStates").child("Position");
        std::string type = PosNode.attribute("Type").value();
        double X = PosNode.attribute("X").as_double();
        double Y = PosNode.attribute("Y").as_double();
        double Z = PosNode.attribute("Z").as_double();
        if (type == "Cartesian")
        {
            // X; Y; Z;
        }
        else if (type == "Geodetic")
        {
            double ell[3] = { X, Y, Z }, XYZ[3];
            hwa_base::ell2xyz(ell, XYZ, true);
            X = XYZ[0]; Y = XYZ[1]; Z = XYZ[2];
        }
        else
        {
            X = Y = Z = 0.0;
        }
        return Triple(X, Y, Z);
    }

    Triple set_ins::vel()
    {
        xml_node VelNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("InitialStates").child("Velocity");
        std::string type = VelNode.attribute("Type").value();
        double X = VelNode.attribute("X").as_double();
        double Y = VelNode.attribute("Y").as_double();
        double Z = VelNode.attribute("Z").as_double();
        if (type == "Cartesian")
        {
            // X; Y; Z;
        }
        else
        {
            X = Y = Z = 0.0;
        }
        return Triple(X, Y, Z);
    }

    Triple set_ins::att()
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("InitialStates").child("Attitude");
        std::string type = AttNode.attribute("Type").value();
        double X = AttNode.attribute("Pitch").as_double();
        double Y = AttNode.attribute("Roll").as_double();
        double Z = AttNode.attribute("Yaw").as_double();
        if (type == "ON")
        {
            // X; Y; Z;
        }
        else
        {
            X = Y = Z = 0.0;
        }
        return Triple(X, Y, Z) * glv.deg;
    }

    Triple set_ins::gyro_bias()
    {
        xml_node GyroBiasNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("InitialStates").child("GyroBias");
        std::string type = GyroBiasNode.attribute("Type").value();
        double X = GyroBiasNode.attribute("X").as_double();
        double Y = GyroBiasNode.attribute("Y").as_double();
        double Z = GyroBiasNode.attribute("Z").as_double();
        if (type == "ON")
        {
            // X; Y; Z;
        }
        else if (type == "OFF")
        {
            X = Y = Z = 0.0;
        }
        else
        {
            //todo: MEAN
        }
        return Triple(X, Y, Z);
    }

    Triple set_ins::acce_bias()
    {
        xml_node AcceBiasNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("InitialStates").child("AcceBias");
        std::string type = AcceBiasNode.attribute("Type").value();
        double X = AcceBiasNode.attribute("X").as_double();
        double Y = AcceBiasNode.attribute("Y").as_double();
        double Z = AcceBiasNode.attribute("Z").as_double();
        if (type == "ON")
        {
            // X; Y; Z;
        }
        else if (type == "OFF")
        {
            X = Y = Z = 0.0;
        }
        else
        {
            //todo: MEAN
        }
        return Triple(X, Y, Z);
    }


    Triple set_ins::imu_installation_rotation()
    {
        xml_node InstallationNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Installation");
        std::string type = InstallationNode.attribute("Type").value();
        if (type == "OFF") { _gmutex.unlock(); return Triple::Zero(); }
        double X = InstallationNode.child("Rotation").attribute("Pitch").as_double();
        double Y = InstallationNode.child("Rotation").attribute("Roll").as_double();
        double Z = InstallationNode.child("Rotation").attribute("Yaw").as_double();
        return Triple(X, Y, Z) * glv.deg;
    }

    Triple set_ins::imu_installation_translation()
    {
        xml_node InstallationNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Installation");
        std::string type = InstallationNode.attribute("Type").value();
        if (type == "OFF") { _gmutex.unlock(); return Triple::Zero(); }
        double X = InstallationNode.child("Lever").attribute("X").as_double();
        double Y = InstallationNode.child("Lever").attribute("Y").as_double();
        double Z = InstallationNode.child("Lever").attribute("Z").as_double();
        return Triple(X, Y, Z);
    }

    IMU_FUSION_TYPE set_ins::imu_fusion_mode()
    {
        IMU_FUSION_TYPE type = hwa_ins::str2fusiontype(_doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).attribute("fusion_type").value());
        return type;
    }

    int set_ins::cps()
    {
        //std::string x = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).attribute("compensation").as_string();
        std::string x = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Compensation").attribute("Type").as_string();
        return hwa_ins::str2cps(x);
    }

    int set_ins::subsample()
    {
        //int x = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).attribute("subsample").as_int();
        int x = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Compensation").attribute("Subsample").as_int();
        return x;
    }

    double set_ins::start()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("ProcTime").attribute("Start").as_double();
        return res;
    }

    double set_ins::end()
    {
        double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("ProcTime").attribute("End").as_double();
        return res;
    }

    bool set_ins::out_intsec()
    {
        bool x = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).attribute("int_sec").as_bool();
        return x;
    }

    int set_ins::out_freq()
    {
        int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).attribute("out_freq").as_double();
        return res;
    }

    std::map<int, std::map<double, int>> set_ins::sim_imu_outages()
    {
        std::map<int, std::map<double, int>> all_outages;
        int num = this->num_of_ins_auxiliary();
        for (int i = 0; i < num + 1; i++)
        {
            std::string name = "IMU" + std::to_string(i);
            std::map<double, int> outages;
            if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_INS).find_child_by_attribute("SimOutages", "id", name.c_str()).empty())
            {
                xml_node SimOutages = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).find_child_by_attribute("SimOutages", "id", name.c_str());
                std::string beg = SimOutages.attribute("beg").value();
                std::string len = SimOutages.attribute("length").value();
                std::stringstream begss(beg), lenss(len);
                double begt;
                int length;
                while (begss >> begt)
                {
                    lenss >> length;
                    outages.insert(std::make_pair(begt, length));
                }
                all_outages.insert(std::make_pair(i, outages));
            }
        }
        return all_outages;
    }

    Triple set_ins::mimu_initial_misalignment_std(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Attitude");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("Attitude");
        }

        std::string str = AttNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AttInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_initial_vel_std(int id)
    {
        //string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_IGN).attribute("initial_misalignment_std").value();
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Velocity");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("Velocity");
        }

        std::string str = AttNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].VelInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_initial_pos_std(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Position");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("Position");
        }

        std::string str = AttNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].PosInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_initial_gyro_std(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("GyroBias");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("GyroBias");
        }

        std::string str = AttNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].GyroBiasInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_initial_acce_std(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("AcceBias");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("AcceBias");
        }

        std::string str = AttNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AcceBiasInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_misalignment_psd(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Attitude");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("Attitude");
        }

        std::string str = AttNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AttProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_vel_psd(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Velocity");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("Velocity");
        }

        std::string str = AttNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].VelProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_pos_psd(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Position");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("Position");
        }

        std::string str = AttNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].PosProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_gyro_psd(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("GyroBias");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("GyroBias");
        }

        std::string str = AttNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].GyroBiasProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::mimu_acce_psd(int id)
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("AcceBias");

        if (id != 0)
        {
            std::string dscrib = "Estimator" + base_type_conv::int2str(id);
            char_t s[13]; //max 999 imus
            strcpy(s, dscrib.c_str());
            AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(s).child("AcceBias");
        }

        std::string str = AttNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AcceBiasProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::initial_misalignment_std()
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Attitude");
        std::string str = AttNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AttInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::initial_vel_std()
    {
        xml_node VelNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Velocity");
        std::string str = VelNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].VelInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::initial_pos_std()
    {
        xml_node PosNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Position");
        std::string str = PosNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].PosInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::initial_gyro_std()
    {
        xml_node GyroNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("GyroBias");
        std::string str = GyroNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].GyroBiasInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::initial_acce_std()
    {
        xml_node AcceNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("AcceBias");
        std::string str = AcceNode.attribute("InitialSTD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AcceBiasInitialSTD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::misalignment_psd()
    {
        xml_node AttNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Attitude");
        std::string str = AttNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AttProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::vel_psd()
    {
        xml_node VelNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Velocity");
        std::string str = VelNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].VelProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::pos_psd()
    {
        xml_node PosNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("Position");
        std::string str = PosNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].PosProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::gyro_psd()
    {
        xml_node GyroNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("GyroBias");
        std::string str = GyroNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].GyroBiasProcNoisePSD;
        return Triple(X, Y, Z);
    }

    Triple set_ins::acce_psd()
    {
        xml_node AcceNode = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child("Estimator").child("AcceBias");
        std::string str = AcceNode.attribute("ProcNoiseSD").value();
        for (int i = 0; i < str.size(); i++)
        {
            if (str[i] == ',')
                str[i] = ' ';
        }
        std::stringstream ss(str);
        double X, Y, Z;
        ss >> X >> Y >> Z;
        if (_imu_type != IMU_TYPE::Customize)
            return _map_imu_error_models[_imu_type].AcceBiasProcNoisePSD;
        return Triple(X, Y, Z);
    }

    std::string set_ins::fuse_type()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("fuse_type");
        str_erase(tmp);
        return tmp;
    }

    bool set_ins::R2P()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("r2p");
        str_erase(tmp);
        bool tmp_bool = false; // default value
        if (tmp != "")
            tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
        return tmp_bool;
    }

    std::vector<std::string> set_ins::PortList()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("port_list");
        for (int i = 0; i < tmp.size(); i++)
        {
            if (tmp[i] == ',')
                tmp[i] = ' ';
        }
        std::stringstream ss(tmp);
        std::string node;
        std::vector<std::string> list;
        while (ss >> node)
            list.push_back(node);
        return list;
    }

    int set_ins::num_of_ins_auxiliary()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("num_of_ins_auxiliary");
        str_erase(tmp);
        int tmp_int = 0; // default value
        if (tmp != "")
            tmp_int = std::stoi(tmp);
        return tmp_int;
    }

    SO3 set_ins::R_imui_imu0(int id)
    {
        SO3 res;
        if (id != 0)
        {
            std::string dscrib = "R_imu" + hwa_base::base_type_conv::int2str(id) + "_imu0";
            char_t s[14]; //max 999 imus
            strcpy(s, dscrib.c_str());
            std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value(s);

            for (int i = 0; i < tmp.size(); i++)
            {
                if (tmp[i] == ',') tmp[i] = ' ';
            }
            std::stringstream ss(tmp);
            double R11 = 0.0, R12 = 0.0, R13 = 0.0, R21 = 0.0, R22 = 0.0, R23 = 0.0, R31 = 0.0, R32 = 0.0, R33 = 0.0;
            ss >> R11 >> R12 >> R13 >> R21 >> R22 >> R23 >> R31 >> R32 >> R33;
            res << R11, R12, R13, R21, R22, R23, R31, R32, R33;
        }
        else
        {
            res = SO3::Identity();
        }
        return res;
    }

    Triple set_ins::p_imui_imu0(int id)
    {
        Triple res;
        if (id != 0)
        {
            std::string dscrib = "p_imu" + hwa_base::base_type_conv::int2str(id) + "_imu0";
            char_t s[14]; //max 999 imus
            strcpy(s, dscrib.c_str());
            std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value(s);

            for (int i = 0; i < tmp.size(); i++)
            {
                if (tmp[i] == ',') tmp[i] = ' ';
            }
            std::stringstream ss(tmp);
            double X = 0.0, Y = 0.0, Z = 0.0;
            ss >> X >> Y >> Z;
            res = Triple(X, Y, Z);
        }
        else
        {
            res = Triple::Zero();
        }
        return res;
    }

    double set_ins::t_imui_imu0(int id)
    {
        double res = 0.0;
        if (id != 0)
        {
            std::string dscrib = "t_imu" + hwa_base::base_type_conv::int2str(id) + "_imu0";
            char_t s[14]; //max 999 imus
            strcpy(s, dscrib.c_str());

            std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value(s);

            if (tmp != "")
                res = std::stod(tmp);
        }
        return res;
    }

    bool set_ins::estimate_imui_extrinsic()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("estimate_imui_extrinsic");
        str_erase(tmp);
        bool tmp_bool = false; // default value
        if (tmp != "")
            tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
        return tmp_bool;
    }

    bool set_ins::estimate_imui_t()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("estimate_imui_t");
        str_erase(tmp);
        bool tmp_bool = false; // default value
        if (tmp != "")
            tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
        return tmp_bool;
    }

    Triple set_ins::initial_extrinsic_rotation_cov()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("initial_extrinsic_rotation_cov");

        if (tmp.empty()) return Triple(3.0462e-4, 3.0462e-4, 3.0462e-4);

        for (int i = 0; i < tmp.size(); i++)
        {
            if (tmp[i] == ',') tmp[i] = ' ';
        }
        std::stringstream ss(tmp);
        double s1 = 0.0, s2 = 0.0, s3 = 0.0;
        ss >> s1 >> s2 >> s3;
        Triple res;
        res << s1, s2, s3;
        return res;
    }

    Triple set_ins::initial_extrinsic_translation_cov()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("initial_extrinsic_translation_cov");

        if (tmp.empty()) return Triple(1e-4, 1e-4, 1e-4);

        for (int i = 0; i < tmp.size(); i++)
        {
            if (tmp[i] == ',') tmp[i] = ' ';
        }
        std::stringstream ss(tmp);
        double s1 = 0.0, s2 = 0.0, s3 = 0.0;
        ss >> s1 >> s2 >> s3;
        Triple res;
        res << s1, s2, s3;
        return res;
    }

    double set_ins::initial_t_cov()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("initial_t_cov");
        str_erase(tmp);
        double tmp_double = 0.0; // default value
        if (tmp != "")
            tmp_double = std::stod(tmp);
        return tmp_double;
    }

    Triple set_ins::enf_R_std()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("enf_R_std");

        if (tmp.empty()) return glv.deg * Triple(0.1, 0.1, 0.1);

        for (int i = 0; i < tmp.size(); i++)
        {
            if (tmp[i] == ',') tmp[i] = ' ';
        }
        std::stringstream ss(tmp);
        double s1 = 0.0, s2 = 0.0, s3 = 0.0;
        ss >> s1 >> s2 >> s3;
        Triple res;
        res << glv.deg * s1, glv.deg* s2, glv.deg* s3;
        return res;
    }

    Triple set_ins::enf_p_std()
    {
        std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_INS).child(XMLKEY_INS_AUXILIARY).child_value("enf_p_std");

        if (tmp.empty()) return Triple(0.05, 0.05, 0.05);

        for (int i = 0; i < tmp.size(); i++)
        {
            if (tmp[i] == ',') tmp[i] = ' ';
        }
        std::stringstream ss(tmp);
        double s1 = 0.0, s2 = 0.0, s3 = 0.0;
        ss >> s1 >> s2 >> s3;
        Triple res;
        res << s1, s2, s3;
        return res;
    }
}

