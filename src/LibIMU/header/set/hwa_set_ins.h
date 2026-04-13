#ifndef hwa_set_ins_h
#define hwa_set_ins_h
#define XMLKEY_INS "ins"
#define XMLKEY_INS_AUXILIARY "ins_auxiliary"
#include "hwa_set_base.h"
#include "hwa_set_ign.h"
#include "hwa_base_eigendef.h"

using namespace hwa_ins;

namespace hwa_ins {
    enum class IMU_TYPE
    {
        NovAtel_SPAN_FSAS,
        NovAtel_SPAN_CPT,
        NovAtel_SPAN_uIRS,
        NovAtel_SPAN_HG9900,
        NovAtel_SPAN_AG11or58,
        NovAtel_SPAN_CPTorKVH,
        NovAtel_SPAN_HG1900,
        NovAtel_SPAN_HG1930,
        NovAtel_SPAN_HG4930,
        NovAtel_SPAN_ADIS16488,
        NovAtel_SPAN_LCI100C,
        NovAtel_SPAN_STIM300,
        NovAtel_SPAN_KVH1750,
        NovAtel_SPAN_uIMU,

        Navigation_Grade,
        Tactical_Grade,
        MEMS_Grade,

        HG1900,
        HG1930,

        ADIS16470,
        StarNeto,
        Customize
    };

    enum class IMU_FUSION_TYPE
    {
        STACK,
        VIRTUAL,
        DEFAULT = 999 ///< default alignment
    };

    struct ERROR_MODEL
    {
        Triple AttInitialSTD;      // deg;
        Triple VelInitialSTD;      // m/s
        Triple PosInitialSTD;      // m
        Triple GyroBiasInitialSTD; // deg/h
        Triple AcceBiasInitialSTD; // mg

        Triple AttProcNoisePSD;      // dpsh
        Triple VelProcNoisePSD;      // mgpsHz
        Triple PosProcNoisePSD;      // mpsh
        Triple GyroBiasProcNoisePSD; // dphpsh
        Triple AcceBiasProcNoisePSD; // mgpsh
    };

    IMU_TYPE str2imu(const std::string& s);

    IMU_FUSION_TYPE str2fusiontype(const std::string& s);

    std::map<IMU_TYPE, ERROR_MODEL> imuerror_models();
}

namespace hwa_set
{
    class set_ins : public virtual set_base
    {
    public:
        set_ins();
        ~set_ins();

        void check();
        void help();
        double ts();
        int freq();
        int resampled_freq();
        UNIT_TYPE GyroUnit();
        UNIT_TYPE AcceUnit();
        UNIT_TYPE AttUnit();
        UNIT_TYPE MagUnit();
        std::string order();
        ODR_TYPE out_order();
        Triple pos();
        Triple vel();
        Triple att();
        bool out_intsec();
        int out_freq();
        int cps();
        int subsample();
        double start();
        double end();
        Triple acce_bias();
        Triple gyro_bias();
        Triple imu_installation_rotation();
        Triple imu_installation_translation();
        IMU_FUSION_TYPE imu_fusion_mode();

        std::string fuse_type();
        bool R2P();
        int num_of_ins_auxiliary();
        std::vector<std::string> PortList();
        SO3 R_imui_imu0(int id = 0);
        Triple p_imui_imu0(int id = 0);
        double t_imui_imu0(int id = 0);
        bool estimate_imui_extrinsic();
        bool estimate_imui_t();
        Triple initial_extrinsic_rotation_cov();
        Triple initial_extrinsic_translation_cov();
        double initial_t_cov();
        Triple enf_R_std();
        Triple enf_p_std();
        std::map<int, std::map<double, int>> sim_imu_outages();

        Triple misalignment_psd();
        Triple vel_psd();
        Triple pos_psd();
        Triple gyro_psd();
        Triple acce_psd();

        Triple initial_misalignment_std();
        Triple initial_vel_std();
        Triple initial_pos_std();
        Triple initial_gyro_std();
        Triple initial_acce_std();
        //multi-imu
        Triple mimu_initial_misalignment_std(int id = 0);
        Triple mimu_initial_vel_std(int id = 0);
        Triple mimu_initial_pos_std(int id = 0);
        Triple mimu_initial_gyro_std(int id = 0);
        Triple mimu_initial_acce_std(int id = 0);
        Triple mimu_misalignment_psd(int id = 0);
        Triple mimu_vel_psd(int id = 0);
        Triple mimu_pos_psd(int id = 0);
        Triple mimu_gyro_psd(int id = 0);
        Triple mimu_acce_psd(int id = 0);

    protected:
        IMU_TYPE _imu_type;
        double _ts;                       ///< sample interval.
        int _freq;                        ///< sample freq.
        std::string _cps;                      ///< compensation mode.
        int _subsample;                   ///< subsample number.
        std::string _order;                    ///< imu data order.
        std::string _out_order;                ///< output data format
        int _out_freq;                    ///< output data format
        bool _int_sec;                    ///< output data format
        std::string _GyroUnit;                 ///< Gyro data unit.
        std::string _AcceUnit;                 ///< Acce data unit.
        Triple _pos, _vel, _att; ///< pos is XYZ,vel is Vxyz,att is P,R,Y(deg).
        double _start, _end;              ///< start time and end time.
        double _align_time;               ///< coarse align time.
        std::string _align_type;               ///< coarse align type.
        Triple _acce_bias;       ///< constant accelerator bias. for simulation use.
        Triple _gyro_bias;       ///< constant gyroscope bias. for simulation use.
        std::map<IMU_TYPE, ERROR_MODEL> _map_imu_error_models;
    };

    extern double StartTime;
}
#endif