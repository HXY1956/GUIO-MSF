#ifndef hwa_vis_proc_utility_h
#define hwa_vis_proc_utility_h
#include <string>
#include <memory>
#include "opencv2/opencv.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "hwa_set_vis.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis
{
    static Triple ACC_N, ACC_W, GYR_N, GYR_W;

    typedef long long int CamStateIDType;

    typedef long long int ImuStateIDType;
    typedef long long int FeatureIDType;

    struct IMG_PATH
    {
        double t;                     ///< time
        std::string img0_path;    ///< path of image0
        std::string img1_path;    ///< path of image1
    };

    struct ONE_FRAME
    {
        double t = 0;                            ///< time            
        std::shared_ptr<cv::Mat> img0 = nullptr;    ///< image 0
        std::shared_ptr<cv::Mat> img1 = nullptr;    ///< image 1
    };

    struct FeaturePoint
    {
        long long int id=0;            ///< feature id             
        cv::Point2f cam0_point;        ///< feature pixel coordinate in left image 
        cv::Point2f cam1_point;        ///< feature pixel coordinate in rgiht image
        double depth=0;             ///< feature initial depth in left image,only used in stereo_orb model
        float response=0;           ///< feature response
        int lifetime=0;             ///< feature lifetime
    };

    struct PointCloud
    {
        double time=0;                    ///< timestamp of current image
        std::vector<FeaturePoint> features;    ///< feature points std::set of current image
    };

    class IntegrationBase
    {
    public:
        double sum_dt = 0;
        Triple delta_p = Triple::Zero();
        Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
        Triple delta_v = Triple::Zero();

        double dt;
        Triple acc_0, gyr_0;
        Triple acc_1, gyr_1;

        Triple linearized_acc, linearized_gyr;
        Triple linearized_ba, linearized_bg;

        Eigen::Matrix<double, 15, 15> jacobian, covariance;
        Eigen::Matrix<double, 15, 15> step_jacobian;
        Eigen::Matrix<double, 15, 18> step_V;
        Eigen::Matrix<double, 18, 18> noise;
        int IMU_count = 0;

        std::vector<double> dt_buf;
        std::vector<Triple> acc_buf;
        std::vector<Triple> gyr_buf;

        IntegrationBase(const Triple& _acc_0, const Triple& _gyr_0, const int& IMU_count)
            : acc_0{ _acc_0 }, gyr_0{ _gyr_0 }, linearized_acc{ _acc_0 }, linearized_gyr{ _gyr_0 }, IMU_count{IMU_count},
            linearized_ba{ Triple::Zero()}, linearized_bg{Triple::Zero()},
            jacobian{ Eigen::Matrix<double, 15, 15>::Identity() }, covariance{ Eigen::Matrix<double, 15, 15>::Zero() },
            sum_dt{ 0.0 }, delta_p{ Triple::Zero() }, delta_q{ Eigen::Quaterniond::Identity() }, delta_v{ Triple::Zero() } {
            
            noise = Eigen::Matrix<double, 18, 18>::Zero();
            noise.block<3, 3>(0, 0) = ACC_N.array().abs2().matrix().asDiagonal();
            noise.block<3, 3>(3, 3) = GYR_N.array().abs2().matrix().asDiagonal();
            noise.block<3, 3>(6, 6) = ACC_N.array().abs2().matrix().asDiagonal();
            noise.block<3, 3>(9, 9) = GYR_N.array().abs2().matrix().asDiagonal();
            noise.block<3, 3>(12, 12) = ACC_W.array().abs2().matrix().asDiagonal();
            noise.block<3, 3>(15, 15) = GYR_W.array().abs2().matrix().asDiagonal();
            
        }

        void processIMU(double dt, const Triple& linear_acceleration, const Triple& angular_velocity, const Triple Bgs, const Triple Bas)
        {
            if (IMU_count == 0) {
                acc_0 = linear_acceleration;
                gyr_0 = angular_velocity;
                linearized_acc = acc_0;
                linearized_gyr = gyr_0;
            }
            dt_buf.push_back(dt);
            acc_buf.push_back(linear_acceleration);
            gyr_buf.push_back(angular_velocity);
            propagate(dt, linear_acceleration, angular_velocity);
            IMU_count++;
        }

        void midPointIntegration(double _dt,
            const Triple& _acc_0, const Triple& _gyr_0,
            const Triple& _acc_1, const Triple& _gyr_1,
            const Triple& delta_p, const Eigen::Quaterniond& delta_q, const Triple& delta_v,
            const Triple& linearized_ba, const Triple& linearized_bg,
            Triple& result_delta_p, Eigen::Quaterniond& result_delta_q, Triple& result_delta_v,
            Triple& result_linearized_ba, Triple& result_linearized_bg, bool update_jacobian)
        {
            Triple un_acc_0 = delta_q * (_acc_0 - linearized_ba);
            Triple un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
            Triple un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
            Triple un_acc = 0.5 * (un_acc_0 + un_acc_1);
            result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
            result_delta_v = delta_v + un_acc * _dt;
            result_linearized_ba = linearized_ba;
            result_linearized_bg = linearized_bg;

            if (update_jacobian)
            {
                Triple w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
                Triple a_0_x = _acc_0 - linearized_ba;
                Triple a_1_x = _acc_1 - linearized_ba;
                SO3 R_w_x, R_a_0_x, R_a_1_x;

                R_w_x << 0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
                R_a_0_x << 0, -a_0_x(2), a_0_x(1),
                    a_0_x(2), 0, -a_0_x(0),
                    -a_0_x(1), a_0_x(0), 0;
                R_a_1_x << 0, -a_1_x(2), a_1_x(1),
                    a_1_x(2), 0, -a_1_x(0),
                    -a_1_x(1), a_1_x(0), 0;

                Matrix F = Matrix::Zero(15, 15);
                F.block<3, 3>(0, 0) = SO3::Identity();
                F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                    -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (SO3::Identity() - R_w_x * _dt) * _dt * _dt;
                F.block<3, 3>(0, 6) = Matrix::Identity(3, 3) * _dt;
                F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
                F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
                F.block<3, 3>(3, 3) = SO3::Identity() - R_w_x * _dt;
                F.block<3, 3>(3, 12) = -1.0 * Matrix::Identity(3, 3) * _dt;
                F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                    -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (SO3::Identity() - R_w_x * _dt) * _dt;
                F.block<3, 3>(6, 6) = SO3::Identity();
                F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
                F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
                F.block<3, 3>(9, 9) = SO3::Identity();
                F.block<3, 3>(12, 12) = SO3::Identity();

                Matrix V = Matrix::Zero(15, 18);
                V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
                V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
                V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
                V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
                V.block<3, 3>(3, 3) = 0.5 * Matrix::Identity(3, 3) * _dt;
                V.block<3, 3>(3, 9) = 0.5 * Matrix::Identity(3, 3) * _dt;
                V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
                V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
                V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
                V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
                V.block<3, 3>(9, 12) = Matrix::Identity(3, 3) * _dt;
                V.block<3, 3>(12, 15) = Matrix::Identity(3, 3) * _dt;

                //step_jacobian = F;
                //step_V = V;
                jacobian = F * jacobian;
                covariance = F * covariance * F.transpose()+ V * noise * V.transpose();
            }

        }

        void propagate(double _dt, const Triple& _acc_1, const Triple& _gyr_1)
        {
            dt = _dt;
            acc_1 = _acc_1;
            gyr_1 = _gyr_1;
            Triple result_delta_p;
            Eigen::Quaterniond result_delta_q;
            Triple result_delta_v;
            Triple result_linearized_ba;
            Triple result_linearized_bg;

            midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                linearized_ba, linearized_bg,
                result_delta_p, result_delta_q, result_delta_v,
                result_linearized_ba, result_linearized_bg, 1);

            //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
            //                    linearized_ba, linearized_bg);
            delta_p = result_delta_p;
            delta_q = result_delta_q;
            delta_v = result_delta_v;
            linearized_ba = result_linearized_ba;
            linearized_bg = result_linearized_bg;
            delta_q.normalize();
            sum_dt += dt;
            acc_0 = acc_1;
            gyr_0 = gyr_1;

        }

        void repropagate(const Triple& _linearized_ba, const Triple& _linearized_bg)
        {
            sum_dt = 0.0;
            acc_0 = linearized_acc;
            gyr_0 = linearized_gyr;
            delta_p.setZero();
            delta_q.setIdentity();
            delta_v.setZero();
            linearized_ba = _linearized_ba;
            linearized_bg = _linearized_bg;
            jacobian.setIdentity();
            covariance.setZero();
            for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
                propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
        }

    };
    /**
    * @struct CamState
    * @brief store camera information used in back end optimization
    */
    struct CamState
    {
        CamStateIDType id;                    ///< id of camera state
        double time=0;                        ///< time when the image is recorded
        Eigen::Quaterniond orientation;        ///< attitude of camera state
        Triple position;            ///< position of camera state
        Eigen::Quaterniond orientation_b;
        Triple position_b;
        bool isKeyFrame;                    ///< identification of current image
        double mtracking_rate=0;            ///< feature tracking rate of current image
        std::vector<double> mvtracking_rate;        ///< feature tracking rate for multi-camera configuration    
        SO3 R_i_e;                ///< zzwu
        Triple wib,ve;
        Eigen::Quaterniond orientation_null;
        Triple position_null;
        Triple gravity;
        Eigen::Quaterniond qcb;    // Transform from IMU to Camera;
        Triple Tcb;       // Transform from IMU to Camera;
        Eigen::Quaterniond qbc;    // Transform from IMU to Camera;
        Triple Tbc;       // Transform from IMU to Camera;
        IntegrationBase *pre_integration;
        SO3 R_e_n;
        Eigen::Quaterniond qnc;    //Transform from c to n;
        Eigen::Quaterniond qnb;    //Transform from b to n;

        CamState() : id(0), time(0),
            orientation(Eigen::Quaterniond::Identity()),
            position(Triple::Zero()),
            orientation_b(Eigen::Quaterniond::Identity()),
            position_b(Triple::Zero()),
            ve(Triple::Zero()),
            orientation_null(Eigen::Quaterniond::Identity()),
            position_null(Triple::Zero()),
            gravity(Triple::Zero()),
            mtracking_rate(0.0),
            mvtracking_rate(std::vector<double>()),
            isKeyFrame(false) {
            pre_integration = new IntegrationBase{ Triple::Zero(), Triple::Zero(), 0 };
        }

        explicit CamState(const CamStateIDType& new_id) : id(new_id), time(0),
            orientation(Eigen::Quaterniond::Identity()),
            position(Triple::Zero()),
            orientation_b(Eigen::Quaterniond::Identity()),
            position_b(Triple::Zero()),
            orientation_null(Eigen::Quaterniond::Identity()),
            position_null(Triple::Zero()),
            gravity(Triple::Zero()),
            mtracking_rate(0.0),
            mvtracking_rate(std::vector<double>()),
            isKeyFrame(false) {
            pre_integration = new IntegrationBase{ Triple::Zero(), Triple::Zero(), 0 };
        }

        void TCI() {
            orientation_b = orientation.operator*(qcb);
            position_b = orientation.operator*(Tcb) + position;
        };
    };

    enum StateOrder
    {
        O_P = 0,
        O_R = 3,
        O_V = 6,
        O_BA = 9,
        O_BG = 12
    };

    enum NoiseOrder
    {
        O_AN = 0,
        O_GN = 3,
        O_AW = 6,
        O_GW = 9
    };
  
    typedef std::map<CamStateIDType, CamState, std::less<int>,
        Eigen::aligned_allocator<std::pair<const CamStateIDType, CamState>>> CamStateServer;

    struct Feature_param
    {
        double translation_threshold=0;            ///< transformation matrix threshold
        double huber_epsilon=0;                    ///< kernel function
        double estimation_precision=0;            ///< threshold of whether the update quantity is iterative
        double    initial_damping=0;                ///< initial lambda of Levenberg-Marquart
        int outler_loop_max_iteration=0;        ///< maximum iterations of outler loop
        int inner_loop_max_iteration=0;            ///< maximum iterations of inner loop
        SE3 T_cam0_cam1;            ///< transformation matrix between cam0 and cam1
        bool stereo;                            ///< stereo or mono

    };

     SO3 skew(const Triple& v);
     SO3 R_ENU_ECEF(const Triple &BLH);
     Triple XYZ2BLH(const Triple &X);

     template <typename Derived>
     static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived>& theta)
     {
         typedef typename Derived::Scalar Scalar_t;

         Eigen::Quaternion<Scalar_t> dq;
         Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
         half_theta /= static_cast<Scalar_t>(2.0);
         dq.w() = static_cast<Scalar_t>(1.0);
         dq.x() = half_theta.x();
         dq.y() = half_theta.y();
         dq.z() = half_theta.z();
         return dq;
     }
}

#endif
