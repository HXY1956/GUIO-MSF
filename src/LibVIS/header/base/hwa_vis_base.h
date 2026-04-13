#ifndef hwa_vis_base_h
#define hwa_vis_base_h
#include <sstream>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "hwa_set_vis.h"
#include "hwa_base_mutex.h"
#include "hwa_base_eigendef.h"
#include "hwa_vis_proc_utility.h"
#include "hwa_vis_proc_feature.h"
#include "hwa_vis_proc_imgproc.h"
#include "hwa_vis_proc_monolk.h"
#include "hwa_vis_proc_stereolk.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_vis
{
    typedef std::map<FeatureIDType, vis_feature> MapServer;
    typedef std::vector<std::pair<FeatureIDType, vis_feature>> EstimatedPoint;
    CamStateServer imuStateServer2CamStateServer(const CamStateServer& imu_states, const SO3 R_c_i, const Triple t_c_i);

    class vis_base
    {
    public:
        vis_base() {};
        vis_base(hwa_set::set_base* _set, int cam_group_id = 0);
        ~vis_base();
        void readChisquare_test();
        void PnP(Eigen::Quaterniond& q, Triple& t);
        bool keyframeCheck();
        bool checkStaticMotion();
        bool addFeatureObservations();
        void measurementJacobianEX(
            const CamStateIDType& cam_state_id,
            const FeatureIDType& feature_id,
            Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 6>& H_x_ex, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r);

        void measurementJacobian(
            const CamStateIDType& cam_state_id,
            const FeatureIDType& feature_id,
            Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r);

        void measurementJacobianEX(
            const CamStateIDType& cam_state_id,
            const FeatureIDType& feature_id,
            Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 6>& H_x_ex, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r);

        void measurementJacobian(
            const CamStateIDType& cam_state_id,
            const FeatureIDType& feature_id,
            Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r);

        bool featureJacobian(const FeatureIDType& feature_id,
            const std::vector<CamStateIDType>& cam_state_ids,
            Matrix& H_x, Vector& r);

        void findRedundantCamStates(std::vector<CamStateIDType>& rm_cam_state_ids);
        void findNonKeyCamStates(std::vector<CamStateIDType>& rm_cam_state_ids);
        bool solveRelativeRT(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& corres, SO3& R, Triple& T);
        bool relativePose(SO3& relative_R, Triple& relative_T, int& l);
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> getCorresponding(int frame_count_l, int frame_count_r);
        void UpdateVisualFeautures();

        bool initialStructure();
        bool visualInitialAlign();
        void RefineGravity(hwa_vis::CamStateServer& camstates, Triple& g, Vector& x);
        Matrix TangentBasis(Triple& g0);
        bool LinearAlignment(hwa_vis::CamStateServer& camstates, Triple& g, Vector& x);
        void solveGyroscopeBias(hwa_vis::CamStateServer& camstates);
        bool VisualIMUAlignment(hwa_vis::CamStateServer& camstates, Triple& g, Vector& x);
        IMG_PATH& get_curr_imgpath() { return imgproc->cur_img_path; }

        static SO3 g2R(const Triple& g)
        {
            SO3 R0;
            Triple ng1 = g.normalized();
            Triple ng2{ 0, 0, 1.0 };
            R0 = Eigen::Quaterniond::FromTwoVectors(ng1.matrix(), ng2.matrix()).toRotationMatrix();
            double yaw = R2ypr(R0).x();
            R0 = ypr2R(Triple{ -yaw, 0, 0 }) * R0;
            return R0;
        }

        static Triple R2ypr(const SO3& R)
        {
            Triple n = R.col(0);
            Triple o = R.col(1);
            Triple a = R.col(2);
            Triple ypr(3);
            double y = atan2(n(1), n(0));
            double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
            double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
            ypr(0) = y;
            ypr(1) = p;
            ypr(2) = r;
            return ypr / 3.1415926 * 180.0;
        }

        static SO3 ypr2R(const Triple& ypr)
        {
            double y = ypr(0) / 180.0 * 3.1415926;
            double p = ypr(1) / 180.0 * 3.1415926;
            double r = ypr(2) / 180.0 * 3.1415926;

            Eigen::Matrix<double, 3, 3> Rz;
            Rz << cos(y), -sin(y), 0,
                sin(y), cos(y), 0,
                0, 0, 1;

            Eigen::Matrix<double, 3, 3> Ry;
            Ry << cos(p), 0., sin(p),
                0., 1., 0.,
                -sin(p), 0., cos(p);

            Eigen::Matrix<double, 3, 3> Rx;
            Rx << 1., 0., 0.,
                0., cos(r), -sin(r),
                0., sin(r), cos(r);

            return Rz * Ry * Rx;
        }

    public:
        PROCESSER_TYPE processer;
        std::map<int, double> chi_squared_test_table;
        CamStateServer cam_states;
        CamStateServer imu_states;
        CamState cam_extrinsic;
        MapServer map_server;
        std::vector<Triple> visual_features;
        CamStateIDType cam_state_id;
        CamStateIDType pre_cam_state_id;
        CamStateIDType cam_next_id;
        bool isKeyFrame = false;
        int nonFrame_num = 0;
        std::vector<CamStateIDType> nonKeyFrameBuffer;
        std::vector<CamStateIDType> KeyFrameBuffer;
        double tracking_rate = 0;
        SO3 R_cam0_imu;
        Triple t_cam0_imu;
        SE3 T_cam0_imu;
        SO3 R_cam0_cam1;
        Triple t_cam0_cam1;
        SE3 T_cam0_cam1;
        double dt_cam0_imu = 0.0;
        Eigen::Quaterniond q_cam0_imu;
        int ex_param_num = 0;
        PointCloud _pointcloud;                                  ///< point cloud result,from font-end
        Eigen::Vector4d cam0_intrinsics;
        int max_camstate_size = 0;
        double _MinParallex;
        double feature_observation_noise = 0;
        int num_of_cam = 0;
        bool stereo;
        bool usingstereorecity;
        Triple Ipos;
        int frame_count = 0;
        int refer_id = 0;
        CLONE_TYPE clone;
        std::vector<Triple> Bgs;
        std::vector<Triple> Bas;
        double static_threshold;
        std::string vfusetype;
        std::unique_ptr<vis_imgproc_base> imgproc;
    };

    struct SFMFeature
    {
        bool state;
        FeatureIDType id;
        std::vector<std::pair<int, Eigen::Vector2d>> observation;
        double position[3];
        double depth;
    };

    struct ReprojectionError3D
    {
        ReprojectionError3D(double observed_u, double observed_v)
            :observed_u(observed_u), observed_v(observed_v) {}

        template <typename T>
        bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
        {
            T p[3];
            ceres::QuaternionRotatePoint(camera_R, point, p);
            p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
            T xp = p[0] / p[2];
            T yp = p[1] / p[2];
            residuals[0] = xp - T(observed_u);
            residuals[1] = yp - T(observed_v);
            return true;
        }

        static ceres::CostFunction* Create(const double observed_x, const double observed_y)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
                new ReprojectionError3D(observed_x, observed_y)));
        }

        double observed_u;
        double observed_v;
    };

    class GlobalSFM
    {
    public:
        GlobalSFM() { feature_num = 0; };
        bool construct(int frame_num, std::vector<Eigen::Quaterniond>& q, std::vector<Triple>& T, int& l,
            const SO3 relative_R, const Triple relative_T,
            std::vector<SFMFeature>& sfm_f, std::map<int, Triple>& sfm_tracked_points);

    private:
        bool solveFrameByPnP(SO3& R_initial, Triple& P_initial, int i, std::vector<SFMFeature>& sfm_f);
        void triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
            Eigen::Vector2d& point0, Eigen::Vector2d& point1, Triple& point_3d);
        void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4>& Pose0,
            int frame1, Eigen::Matrix<double, 3, 4>& Pose1,
            std::vector<SFMFeature>& sfm_f);
        int feature_num;
    };

}

#endif
