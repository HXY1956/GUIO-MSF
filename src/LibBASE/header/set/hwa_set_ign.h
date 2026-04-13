/**
*
* @verbatim
    The format of this block:
    <integration>
        <!--> vis setting node  <-->
        <IGN_TYPE> LIDAR_TCI </IGN_TYPE>                            <!-->  type of combination,
                                                                            options:LCI/TCI/STCI/VIO/VIS_LCI/VIS_TCI/VIS_STCI
                                                                            /LIO/LIDAR_LCI/LIDAR_TCI/LIDAR_STCI/VLO/MULTIGN_LCI/MULTIGN_TCI/MULTIGN_STCI    <-->
        <lever> -0.050,-0.650,0.150 </lever>                        <!-->  lever arm <-->
        <nq> 15 </nq>                                                <!-->  number of ins state vector <-->
        <nr> 6 </nr>                                                <!-->  ## <-->
        <min_sat> 3 </min_sat>                                        <!-->  minimum number of satellite <-->
        <max_pdop> 3 </max_pdop>                                    <!-->  maximum of PDOP <-->
        <delay_t> 0.005 </delay_t>                                    <!-->  maximum time delay <-->
        <initial_misalignment_std> 1,1,1 </initial_misalignment_std><!-->  initial standard convariance of misalignment <-->
        <initial_vel_std> 0.1,0.1,0.1 </initial_vel_std>            <!-->  initial standard convariance of velocity <-->
        <initial_pos_std> 1,1,1 </initial_pos_std>                    <!-->  initial standard convariance of position <-->
        <initial_gyro_std> 200 </initial_gyro_std>                    <!-->  initial standard convariance of gyro <-->
        <initial_acce_std> 10 </initial_acce_std>                    <!-->  initial standard convariance of accelerometer <-->

        <misalignment_psd> 1 </misalignment_psd>                    <!-->  Noise power spectral density of misalignment <-->
        <vel_psd> 1 </vel_psd>                                        <!-->  Noise power spectral density of velocity<-->
        <pos_psd> 0 </pos_psd>                                        <!-->  Noise power spectral density of position <-->
        <gyro_psd> 100 </gyro_psd>                                    <!-->  Noise power spectral density of gyro<-->
        <acce_psd> 0.5 </acce_psd>                                    <!-->  Noise power spectral density of accelerometer<-->
    <integration/>
    @endverbatim
*
*/

#ifndef hwa_set_ign_h
#define hwa_set_ign_h
#define XMLKEY_IGN "integration"
#include "hwa_set_base.h"
#include "hwa_base_time.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_posetrans.h"

namespace hwa_ins {
    enum CPS_TYPE
    {
        CONE,         ///< Cone mode
        ONE_PLUS_PRE, ///< One plus previous mdoe
        POLY          ///< Poly mode
    };
    enum MOTION_TYPE
    {
        m_keep,       ///< keep mode
        m_accelerate, ///< accelerate mode
        m_decelerate, ///< decelerate mode
        m_yawleft,    ///< yaw left mode
        m_yawright,   ///< yaw right mode
        m_pitchup,    ///< pitch uo mode
        m_pitchdown,  ///< pitch down mode
        m_rollleft,   ///< roll left mode
        m_rollright,  ///< roll right mode
        m_turnleft,   ///< turn left mode
        m_turnright,  ///< turn right mode
        m_climb,      ///< climb mode
        m_descent,    ///< descent mode
        m_s,          ///< S-shaped trajectory mode
        m_8,          ///< 8-shaped trajectory mode
        m_straight,   ///< straight mode
        m_static,     ///< static mode
        m_default     ///< default mode
    };

    enum UNIT_TYPE
    {
        RAD,  ///< rad
        DEG,  ///< deg
        RPS,  ///< TODO
        DPS,  ///< TODO
        RPH,  ///< TODO
        DPH,  ///< TODO
        MPS,  ///< TODO
        MPS2, ///< TODO
        UNDF  ///< undefined
    };

    enum ODR_TYPE
    {
        XYZ_XYZ_PRY_NW, ///< output format 1
        BLH_ENU_PRY_NW, ///< output format 2
        HOLO_ODO        ///< TODO
    };

    enum ALIGN_TYPE
    {
        AUTO,               ///< auto alignment
        STC_AGN,            ///< static base alignment
        MA,                 ///< moving base alignment
        POS_AGN,            ///< position std::vector alignment
        VEL_AGN,            ///< velocity std::vector alignment
        VINS,
        TRACK,
        MULTI_ANT,
        NONE,
        ALIGN_DEFAULT = 999 ///< default alignment
    };

    enum FLT_TYPE
    {
        FORWARD,  ///< forward mode
        BACKWARD, ///< backward mode
        FBS,      ///< forward and backward mode
        RTS       ///< RTS mode
    };

    enum MEAS_TYPE
    {
        GNSS_MEAS = 1,        ///< GNSS measurement
        POS_MEAS,             ///< position measurement
        VEL_MEAS,             ///< velocity measurement
        POS_VEL_MEAS,         ///< TODO
        DD_UC_OBS_MEAS,       ///< double-difference un-combination measurement
        DD_IF_OBS_MEAS,       ///< double-difference ionosphere-free combination measurement
        FIXED_DD_UC_OBS_MEAS, ///<fixed double-difference un-combination measurement

        MOTION_MEAS = 20, ///< motion measurement
        ZUPT_MEAS,        ///< zero velocity update measurement
        ZIHR_MEAS,        ///< zero integrated heading rate update measurement
        ODO_MEAS,         ///< odometer measurement
        NHC_MEAS,         ///< nonholonomic constraints measurement
        YAW_MEAS,         ///< yaw measurement
        Hgt_MEAS,         ///< height measurement
        ATT_MEAS,         ///< attitude measurement
        ZUPT_POS_MEAS,    ///< zero velocity update position measurement

        OTHER_MEAS = 34, ///< other measurement
        VIS_MEAS,        ///< vision measurement
        LIDAR_MEAS,      ///< LiDAR measurement

        INDOOR_MEAS = 50, ///< indoor measurement
        UWB_MEAS,         ///< UWB measurement
        WIFI_MEAS,        ///< WiFi measurement
        BLE_MEAS,         ///< BLE measurement
        IMGTAG_MEAS,      ///< image tag measurement

        R_MIMU_MEAS,      ///< multi-imu enforcing rotation constraint
        P_MIMU_MEAS,      ///< multi-imu enforcing translation constraint

        MIMU_MEAS,

        DEFAULT_MEAS = 100, ///< default measurement
        NO_MEAS             ///< none
    };

    /**
    * @enum LCI_TYPE
    * @brief GNSS position mode for loosely conpled integration mode
    */
    enum LCI_TYPE
    {
        SPP,   ///< SPP mode for loosely conpled integration mode
        PPP,   ///< PPP mode for loosely conpled integration mode
        RTK,   ///< RTK mode for loosely conpled integration mode
        PPPRTK ///< PPP-RRTK mode for loosely conpled integration mode
    };

    /**
    * @enum IGN_TYPE
    * @brief GNSS/INS integration option.
    * PURE_INS is ins process only;
    * LCI is loosely coupled integration;
    * TCI is tightly coupled intergration;
    * STCI is semi-tightly coupled integration.
    * MULTIGN_LC is GNSS/INS/VISION loosely coupled integrartion
    * MULTIGN_TC is GNSS/INS/VISION tightly coupled integrartion
    * MULTIGN_STC is GNSS/INS/VISION semi-tightly coupled integrartion
    * MULTIGN_LIDAR_LC is GNSS/INS/LIDAR loosely coupled integrartion
    * MULTIGN_LIDAR_TC is GNSS/INS/LIDAR tightly coupled integrartion
    * MULTIGN_LIDAR_STC is GNSS/INS/LIDAR semi-tightly coupled integrartion
    * LIDAR_INS is INS/LIDAR tightly coupled integration
    * VIO is INS/VISION tightly coupled integration
    */
    enum IGN_TYPE
    {
        PURE_INS,     ///< ins process only
        VIO_TCI,      ///< INS/VISION tightly coupled integration
        VIO_LCI,      ///< INS/VISION loosely coupled integration
        LIO_TCI,      ///< INS/LIDAR tighty coupled integration
        LIO_LCI,       ///< INS/LIDAR loosely coupled integration
        LVI_TCI,       ///< VISION/INS/LIDAR tighty coupled integration
        GI_LCI,         ///< GNSS/INS loosely coupled integrartion
        GI_TCI,       ///< GNSS/INS tightly coupled integrartion
        GI_STCI,      ///< GNSS/INS semi-tightly coupled integrartion
        UI_LCI,      ///< INS/UWB loosely coupled integration
        UI_TCI,      ///< INS/UWB tightly coupled integration
        UVI_TCI,       /// UWB/INS/VISION tightly coupled
        ULI_TCI,       /// UWB/INS/LIDAR tightly coupled
		ULVI_TCI,      /// UWB/LIDAR/INS/VISION tightly coupled
        GVI_TCI,      /// GNSS/INS/VISION tightly coupled
        GUI_TCI,      /// GNSS/INS/UWB tightly coupled
        GUVI_TCI,     /// GNSS/UWB/INS/VISION tightly coupled
		GLI_TCI,      /// GNSS/INS/LIDAR tightly coupled
		GLVI_TCI,     /// GNSS/INS/LIDAR/VISION tightly coupled
		GULVI_TCI,     /// GNSS/UWB/INS/LIDAR/VISION tightly coupled
        IGN_DEFAULT   ///< defalut coupled integration
    };

    enum Estimator {
        NORMAL,
        INEKF
    };

    enum START_ENV {
        OUTDOOR,
        INDOOR
    };

    START_ENV str2startenv(std::string s);

    Estimator str2estimator(std::string res);

    MEAS_TYPE ins2meas(const int& i);

    std::string meas2str(MEAS_TYPE type);

    LCI_TYPE str2lci(const std::string& s);

    IGN_TYPE str2ign(const std::string& s);

    Estimator str2fuse(const std::string& s);

    std::string fuse2str(const Estimator s);

    UNIT_TYPE str2Unit(const std::string& s);

    FLT_TYPE str2Flt(const std::string& s);

    ODR_TYPE str2odr(const std::string& s);

    ALIGN_TYPE str2align(const std::string& s);

    CPS_TYPE str2cps(const std::string& s);
}

using namespace hwa_ins;

namespace hwa_set{

    class set_ign : public virtual set_base
    {
    public:
        /** @brief default constructor. */
        set_ign();

        /** @brief default destructor. */
        ~set_ign();

        /**
        * @brief settings check.
        */
        void check();

        /**
        * @brief settings help.
        */
        void help();

        bool align();

        double align_time();

        std::string start_env();

        double pos_dist();

        double vel_norm();

        ALIGN_TYPE align_type();

        Triple initial_gyro_scale_std();

        Triple initial_acce_scale_std();
        /**
         * @brief
         *
         * @return Triple
         */
        Triple initial_imu_installation_att_std();

        /**
         * @brief
         *
         * @return double
         */
        double initial_odoscale_std(); ///< TODO

        /**
        * @brief    get minimum standard convariance of misalignment.
        * @return    Triple     minimum standard convariance of misalignment
        */
        Triple min_misalignment_std();

        /**
        * @brief    get minimum standard convariance of velocity.
        * @return    Triple     minimum standard convariance of velocity
        */
        Triple min_vel_std();

        /**
        * @brief    get minimum standard convariance of position.
        * @return    Triple     minimum standard convariance of position
        */
        Triple min_pos_std();

        /**
        * @brief    get minimum standard convariance of gyro.
        * @return    Triple     minimum standard convariance of gyro
        */
        Triple min_gyro_std();

        /**
        * @brief    get minimum standard convariance of accelerometer.
        * @return    Triple     minimum standard convariance of accelerometer
        */
        Triple min_acce_std();

        /**
        * @brief    get minimum standard convariance of odometer.
        * @return    double     minimum standard convariance of odometer
        */
        double min_odo_std();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple gyro_scale_psd();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple acce_scale_psd();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple imu_inst_rot_psd(); // imu installation attitude psd

        /**
         * @brief
         *
         * @return Triple
         */
        Triple imu_inst_trans_psd(); // imu translation attitude psd

        /**
        * @brief    get noise power spectral density of odometer.
        * @return    double     noise power spectral density of odometer
        */
        double odo_scale();

        /**
         * @brief
         *
         * @return double
         */
        double odo_psd();

        /**
         * @brief
         *
         * @return double
         */
        double odo_std();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple nhc_std();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple zupt_std();

        double zupt_dstd();

        /**
         * @brief
         *
         * @return double
         */
        double zihr_std();

        /**
         * @brief
         *
         * @return double
         */
        double Yaw_std();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple att_std();

        UNIT_TYPE AttUnit();
        /**
        * @brief    get measurement noise velocity.
        * @return    Triple     measurement noise velocity
        */
        Triple meas_vel_noise();

        /**
        * @brief    get measurement noise of position.
        * @return    Triple     measurement noise of position
        */
        Triple meas_pos_noise();

        /**
         * @brief
         *
         * @return string
         */
        std::string order(); ///< TODO

        /**
        * @brief    get number of ins state vector.
        * @return    int     number of ins state vector
        */
        int nq();

        /**
         * @brief
         *
         * @return int
         */
        int nr(); ///< TODO

        /**
         * @brief
         *
         * @return double
         */
        double TS(); ///< TODO for GNSS

        double TS_odo(); ///< TODO for ODO

        /**
        * @brief    get maximum time delay.
        * @return    double     maximum time delay
        */
        double delay_t();

        /**
         * @brief
         *
         * @return double
         */
        double delay_odo(); ///< TODO

        /**
        * @brief    get maximum of PDOP.
        * @return    double     maximum of PDOP
        */
        double max_pdop();

        /**
        * @brief    get minimum number of satellite.
        * @return    int     minimum number of satellite
        */
        int min_sat();

        /**
        * @brief    get lever arm.
        * @return    Triple     lever arm
        */
        Triple gnss_lever();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple odo_lever();

        /**
         * @brief
         *
         * @return Triple
         */
        Triple uwb_lever();

        /**
        * @brief    get integration type.
        * @return    IGN_TYPE     integration type
        */
        IGN_TYPE _ign_type_();

        Estimator fuse_type();

        double inflation();

        bool static_flag();

        std::map<double, int> sim_gnss_outages();

        /**
        * @brief    get filter mode.
        * @return    FLT_TYPE     filter mode
        */
        hwa_ins::FLT_TYPE fltmode();

        /**
         * @brief get the format of ODO file
         *
         * @return string
         * @return "OFF/Raw/Pulse/Velocity"
         */
        bool Odo(); ///< TODO

        /**
         * @brief get the installation of ODO
         *
         * @return string
         * @return "Left/Right"
         */
        std::string odo_inst(); ///< TODO

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool NHC();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool ZUPT();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool Attitude();

        bool Direct();
        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool UWB();
        bool GNSS();
        bool VISION();
        bool LIDAR();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool Hgt();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool imu_scale();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool imu_inst_rot();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool imu_inst_trans();

        /**
         * @brief
         *
         * @return double
         */
        double wheelraduis();

        double UWB_start();

        double UWB_end();

        double Hgt_start();

        double Hgt_end();

        double Hgt_std();

        double Hgt_info();

        double att_precision_L();

        std::map<std::string, Triple> MultiAntLever();

        std::map<MEAS_TYPE, double> max_norm();

        MEAS_TYPE xmlname2meas(std::string str);

    protected:
        Triple _initial_misalignment_std; ///< Standard deviation of initial misalignment angle [deg].
        Triple _initial_vel_std;          ///< Standard deviation of initial velocity [m/s].
        Triple _initial_pos_std;          ///< Standard deviation of initial position [m].
        Triple _initial_gyro_std;         ///< Standard deviation of initial gyro drift [deg/h].
        Triple _initial_acce_std;         ///< Standard deviation of initial accelerator bias [mg].
        Triple _initial_gyro_scale_std;   ///< Standard deviation of initial gyro scale [#].
        Triple _initial_acce_scale_std;   ///< Standard deviation of initial accelerator scale [#].
        double _initial_odoscale_std;              ///< TODO

        Triple _min_misalignment_std; ///< Standard deviation lower bound of misalignment angle [deg].
        Triple _min_vel_std;          ///< Standard deviation lower bound of velocity [m/s].
        Triple _min_pos_std;          ///< Standard deviation lower bound of position [m].
        Triple _min_gyro_std;         ///< Standard deviation lower bound of gyro drift [deg/h].
        Triple _min_acce_std;         ///< Standard deviation lower bound of accelerator bias [mg].
        double _odo_std;

        Triple _misalignment_psd; ///< Spectral density of misalignment angle [deg/sqrt(h)].
        Triple _vel_psd;          ///< Spectral density of velocity [mg/sqrt(Hz)].
        Triple _pos_psd;          ///< Spectral density of position
        Triple _gyro_psd;         ///< Spectral density of gyro bias
        Triple _acce_psd;         ///< Spectral density of accelerometer bias
        Triple _gyro_scale_psd;   ///< Spectral density of gyro scale
        Triple _acce_scale_psd;   ///< Spectral density of accelerometer scale
        double _odo_psd;

        Triple _meas_vel_noise; ///< Measurement noise of velocity [m/s]
        Triple _meas_pos_noise; ///< Measurement noise of position [m]

        Triple _lever, _odo_lever, _uwb_lever; ///< GNSS Receiver center's lever relative to imu center on b Coor
        int _nq, _nr;                                   ///< parameter dimension and measurement dimension
        hwa_ins::FLT_TYPE _fltmode;                       ///< filter mode (forward, forward and backward smooth, RTS)
        double _TS;                             ///< measurement data interval
        std::string _order;                                  ///< the order for pos file (size is 6)
        double _delay_t;                                ///< the GNSS misalignment time diff with imu
        double _delay_odo;                              ///< the ODO misalignment time diff with imu
        double _max_pdop;                               ///< the max PDOP threshold for LCI
        int _min_sat;                                   ///< the minimum sat num threshold.
        IGN_TYPE _ign_type;                             ///< the integrated mode
        std::string _odo;                                    ///< odometry
        bool _NHC, _ZUPT;                                ///< motion constraints
    };
}

#endif