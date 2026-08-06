#ifndef hwa_vis_proc_imgproc_h
#define hwa_vis_proc_imgproc_h
#include "hwa_base_eigendef.h"
#include <opencv2/core/cuda.hpp>
#include <cuda_runtime.h>
#include "hwa_base_string.h"
#include "hwa_set_base.h"
#include "hwa_set_vis.h"
#include "hwa_vis_proc_utility.h"
#include "hwa_vis_coder.h"
#include "hwa_vis_yolo_base.h"
#include <unordered_set>

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_vis {
    class vis_yolo_v8ov;
}

namespace hwa_vis {
    struct IMU_MSG
    {
        double t = 0;                            ///< time                
        Triple linear_acceleration;    ///< linear acceleration
        Triple angular_velocity;        ///< angular velocity

        IMU_MSG(double timestamp,
            const Triple& accel,
            const Triple& gyro)
            : t(timestamp),
            linear_acceleration(accel),
            angular_velocity(gyro) {
        }
        IMU_MSG() {}
    };

    class vis_imgproc_base {
    public:
        vis_imgproc_base() {};
        vis_imgproc_base(set_base* _set, int cam_group_id);
        void load_imuobs(const double& t, const std::vector<Triple>& gyro, const std::vector<Triple>& acc, double imu_ts);
        void load_imgobs(const double& t, const IMG_PATH& img_path);
        void add_camera_pose(SO3 R, Triple t);
        virtual PointCloud ProcessBatch();
        virtual void ProcessBatchT(const double& t, PointCloud& pointcloud);
        virtual cv::Mat get_out_img() = 0;
        void removeDynamicPoints(
            const std::vector<cv::Point2f>& curr_pts,
            const std::vector<Detection>& dyna_box,
            std::vector<unsigned char>& inliers);
        void dyna_detect(const cv::Mat& img);

    public:
        std::unordered_set<int> dynamic_classes;
        vis_yolo_v8ov* _yolo;
		std::vector<Detection> dyna_box;            ///< std::vector store dynamic box

        IMG_PATH cur_img_path;            ///<current image path
        std::vector<IMU_MSG> _vecimu;            ///< std::vector store imu data between pre frame and cur frame
        double ts = 0;                        ///< camera sample interval. 
        int freq = 0;                        ///< camera sample freq. 
        bool TimeCostOut = false;
        /* Basic parameters */
        DISTORTION_TYPE cam0_distortion_model;        ///< distortion model of left camera
        DISTORTION_TYPE cam1_distortion_model;        ///< distortion model of right camera
        Eigen::Vector2d cam0_resolution;            ///< resolution of left camera
        Eigen::Vector2d cam1_resolution;            ///< resolution of right camera
        Eigen::Vector4d cam0_intrinsics;            ///< intrinsics of left camera
        Eigen::Vector4d cam1_intrinsics;            ///< intrinsics of right camera
        Eigen::Vector4d cam0_distortion_coeffs;        ///< distortion coeffs of left camera
        Eigen::Vector4d cam1_distortion_coeffs;        ///< distortion coeffs of right camera
        SO3 R_cam0_cam1;                ///< rotation from left to right camera
        Triple t_cam0_cam1;                ///< translation from left to right camera
        SE3 T_cam0_cam1;                ///< transformation from left to right camera
        SO3 R_cam0_imu;                    ///< rotation from left camera to IMU
        Triple t_cam0_imu;                    ///< translation from left camera to IMU
        SE3 T_cam0_imu;                ///< transformation from left camera to IMU
        SE3 T_cam1_imu;                ///< transformation from right camera to IMU
        Triple t_cam1_imu;                    ///< transformation from right camera to IMU
        SO3 R_cam1_imu;                    ///< transformation from right camera to IMU
        double dt_cam0_imu;
        PointCloud _pointCloud;            ///< std::vector store pointcloude

        /* Stereo Optical flow */
        int grid_row = 0;                ///< rows of grid
        int grid_col = 0;                ///< columns of grid
        int grid_min_feature_num = 0;    ///< min feature number in a grid
        int grid_max_feature_num = 0;    ///< max feature number in a gird
        int pyramid_levels = 0;            ///< levels of pyramid
        int patch_size = 0;                ///< optical flow tracking window size
        int fast_threshold = 0;            ///< feature detection threshold
        double ransac_threshold = 0;    ///< RANSAC threshold
        double stereo_threshold = 0;    ///< stereo match threshold
        int max_iteration = 0;            ///< max iterations
        double track_precision = 0;        ///< track precision
        /* Stereo Optical flow */

        /* Stereo Orb Optical Flow */
        int max_cnt = 0;                ///< use in st storb mono
        int equalize = 0;
        bool usingstereorecify = 0;        ///< TODO
        /* Stereo Orb Optical Flow */

        /* Estimator parameter */
        int max_cam_state_size = 0;                ///< maximum size of camera states
        double position_std_threshold = 0;        ///< threshold of position STD
        double rotation_threshold = 0;            ///< threshold of rotation
        double translation_threshold = 0;        ///< threshold of translation
        double feature_observation_noise = 0;    ///< feature observation noise
        /* Estimator parameter */

        /* Depth optimation parameter */
        double huber_epsilon = 0;                    ///< TODO
        double estimation_precision = 0;            ///< TODO
        double initial_damping = 0;                    ///< TODO
        int outler_loop_max_iteration = 0;            ///< TODO
        int inner_loop_max_iteration = 0;            ///< TODO
        /* Depth optimation parameter */

        /*Estimated Paramters*/
        bool estimate_extrinsic = 0;                ///< estimate extrinsic or not
        bool estimate_t = 0;                ///< estimate time extrinsic or not
        int ex_param_num = 0;

        Triple initial_cam_extrinsic_rotation_cov;        ///< initial convariance of rotation in camera's extrinsic
        Triple initial_cam_extrinsic_translation_cov;    ///< initial convariance of translation in camera's extrinsic
        double initial_cam_t_cov;    ///< initial convariance of translation in camera's extrinsic
        bool estimate_extrinsic_seperately = 0;        ///< estimate extrinsic seperately or not

        bool estimate_extrinsic_allcam = 0;        ///< estimate extrinsic seperately or not
        bool estimate_t_allcam = 0;        ///< estimate extrinsic seperately or not

        bool stereo;
        bool Status = false;
        double CameraDelay = -1000;
        cv::Vec3f mean_ang_vel;
        double ts_p_c = 0;
        int cam_update_skip = 0;                    ///< TODO
        int _cam_group_id = 0;                        ///< TODO
        std::deque<std::pair<Matrix, Triple>> _pose_history;    ///< TODO
        int _pose_history_max_size = 0;                                    ///< TODO
        std::vector<Triple> _stereo_pointcloud;                        ///< TODO
    };

    template <typename MatType>
    class vis_imgproc : public vis_imgproc_base
    {
    public:
        vis_imgproc() {};

        vis_imgproc(set_base* _set, int cam_group_id) : vis_imgproc_base(_set, cam_group_id){}

        ~vis_imgproc() {}

        cv::Mat get_out_img() override
        {
            cv::Mat frame;
            if constexpr (std::is_same_v<MatType, cv::Mat>)
            {
                frame = out_img->clone();
            }
            else if constexpr (std::is_same_v<MatType, cv::cuda::GpuMat>)
            {
                out_img->download(frame);
            }
            return frame;
        }

    public:
        std::pair<double, std::pair<MatType, MatType>> imageGroup;
        std::shared_ptr<MatType> out_img = nullptr;                ///< TODO
        std::shared_ptr<MatType> out_img_dense = nullptr;            ///< TODO
        std::shared_ptr<MatType> out_img_IPM = nullptr;            ///< TODO
    };
}

#endif
