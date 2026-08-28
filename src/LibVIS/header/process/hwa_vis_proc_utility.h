#ifndef hwa_vis_proc_utility_h
#define hwa_vis_proc_utility_h
#include <string>
#include <memory>
#include "opencv2/opencv.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "hwa_set_vis.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_preintegration.h"

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
        Eigen::Vector2d velocity;
        double cur_td;
    };

    struct PointCloud
    {
        double time=0;                    ///< timestamp of current image
        std::vector<FeaturePoint> features;    ///< feature points std::set of current image
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
        IntegrationBase *pre_integration = nullptr;
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
            //pre_integration = new IntegrationBase{ Triple::Zero(), Triple::Zero(), 0 };
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
            //pre_integration = new IntegrationBase{ Triple::Zero(), Triple::Zero(), 0 };
        }

        void TCI() {
            orientation_b = orientation.operator*(qcb);
            position_b = orientation.operator*(Tcb) + position;
        };
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
