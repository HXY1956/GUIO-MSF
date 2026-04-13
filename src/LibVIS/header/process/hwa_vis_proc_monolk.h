#ifndef hwa_vis_proc_monolk_h
#define hwa_vis_proc_monolk_h
#include "hwa_base_timecost.h"
#include "hwa_set_base.h"
#include "hwa_set_vis.h"
#include "hwa_vis_proc_utility.h"
#include "hwa_vis_proc_imgproc.h"
#include "hwa_vis_proc_cudafunction.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis {
    class vis_mono_lk_gpu : public vis_imgproc<cv::cuda::GpuMat>
    {
    using Points = std::vector<cv::Point2f>;
    public:
        vis_mono_lk_gpu(hwa_set::set_base* _set, int cam_group_id = 0);
        ~vis_mono_lk_gpu() {}
        bool Initialize();
        void SetImage(cv::Mat img) {
            _img0 = img;
        };
        virtual PointCloud ProcessBatch();
        bool initializeFirstFrame();
        bool monotrack();
        bool TrackPointsPyrLK_CUDA(
            const cv::cuda::GpuMat& prev_img,
            const cv::cuda::GpuMat& next_img,
            cv::cuda::GpuMat& cam0_points,
            cv::cuda::GpuMat& cam1_points,
            cv::cuda::GpuMat& lkstatus,
            int patch_size = 21,
            int pyramid_levels = 3,
            int max_iteration = 30,
            bool use_initial_flow = true,
            int device_id = 0
        );
        void drawFeaturesStereoGpu();
        void publish();
        void trackFeatures();
        void integrateImuData(cv::Matx33f& cam0_R_p_c, cv::Matx33f& cam1_R_p_c);
        void predictFeatureTracking(
            const std::vector<cv::Point2f>& input_pts,
            const cv::Matx33f& R_p_c,
            const cv::Vec4d& intrinsics,
            std::vector<cv::Point2f>& compensated_pts);
        void twoPointRansac(
            cv::cuda::GpuMat& pts1, cv::cuda::GpuMat& pts2,
            const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            const double& inlier_error,
            const double& success_probability
        );
        void addNewFeatures();
        cv::cuda::GpuMat distortPoints(
            const cv::cuda::GpuMat& pts_in,
            const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs);
        void undistortPoints(
            const cv::cuda::GpuMat& pts_in,
            const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            cv::cuda::GpuMat& pts_out,
            const cv::Matx33d& rectification_matrix = cv::Matx33d::eye(),
            const cv::Vec4d& new_intrinsics = cv::Vec4d(1, 1, 0, 0));
        void undistortPoints(
            const cv::cuda::GpuMat& pts_in,
            const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            Points& pts_out,
            const cv::Matx33d& rectification_matrix = cv::Matx33d::eye(),
            const cv::Vec4d& new_intrinsics = cv::Vec4d(1, 1, 0, 0));
        static bool featureCompareByResponse(
            const FeaturePoint& f1,
            const FeaturePoint& f2);
        template <typename Tem>
        void removeUnmarkedElements(
            const std::vector<Tem>& raw_vec,
            const std::vector<unsigned char>& markers,
            std::vector<Tem>& refined_vec);
        void rescalePoints(
            std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
            float& scaling_factor);
        static bool keyPointCompareByResponse(
            const cv::KeyPoint& pt1,
            const cv::KeyPoint& pt2);
        static bool featureCompareByLifetime(
            const FeaturePoint& f1,
            const FeaturePoint& f2);
        void resetIMU();
        void setTrackType(TRACK_TYPE _T) { Tp = _T; }
        void setFilterType(FILTER_TYPE _F) { F = _F; }
        void setEpipolar(bool _useEpipolar, cv::Matx33d _E = cv::Matx33d::eye(), double _threshold = 0) {
            useEpipolar = _useEpipolar;
            Epipolar = _E;
            threshold = _threshold;
        }

        void setparam(cv::cuda::GpuMat& _d_status, cv::cuda::GpuMat& _cam0);
        void setparam(cv::cuda::GpuMat& _d_status, cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1);
        void setparam_undistorted(cv::cuda::GpuMat& _cam0);
        void setparam_undistorted(cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1);
        void download(std::vector<cv::Point2f>& cam0_inlier);
        void download(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier);
        void upload(std::vector<cv::Point2f>& cam0_inlier);
        void upload(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier);

        void filterNewPointsGPU();
        void filterPointsGPU();
        void setflags();
        void setnewptsflags();
        void InitializeGrid();
        void InitializeNewPtsGrid();
        void AddNewPts();
        void Init();
        void InitNewPts();
        void ManageGrid();
        void PruneGrid();
        void DrawPts(std::shared_ptr<cv::cuda::GpuMat> _outimg);


    public:
        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> lk;
        cv::Ptr<cv::cuda::CornersDetector> gftt_fast;
        cv::Ptr<cv::cuda::CornersDetector> gftt_init;
        cv::cuda::GpuMat d_prev0, d_curr0;
        cv::cuda::Stream stream;
        cv::cuda::GpuMat d_prevPts0;
        cv::cuda::GpuMat d_currPts0;
        cv::cuda::GpuMat d_currPts0_new;
        cv::cuda::GpuMat d_prevPts0_undistorted;
        cv::cuda::GpuMat d_currPts0_undistorted;
        cv::cuda::GpuMat d_currPts0_new_undistorted;
        cv::cuda::GpuMat d_status;
        cv::cuda::GpuMat d_status_new;

        cv::Matx33d _R_cam0_imu;
        cv::Vec3d _t_cam0_imu;

        std::string _cam0_distortion_model;
        cv::Vec4d _cam0_intrinsics;
        cv::Vec4d _cam0_distortion_coeffs;

        cv::Mat _img0;
        std::ofstream TimeCostDebugOutFile;

    private:
        thrust::device_vector<int> lifetime;
        thrust::device_vector<int> area;
        thrust::device_vector<int> ids;
        thrust::device_vector<int> valid_flags;
        thrust::device_vector<int> lifetime_newpts;
        thrust::device_vector<int> area_newpts;
        thrust::device_vector<int> ids_newpts;
        thrust::device_vector<int> valid_flags_newpts;
        cv::Matx33d Epipolar;
        TRACK_TYPE Tp;
        FILTER_TYPE F;
        bool useEpipolar = false;
        double threshold = 0;
        int width;
        int height;
        int grid_width;
        int grid_height;
        double prev_img_t;
        double curr_img_t;
        long long int next_feature_id = 0;
        int before_tracking = 0;
        int after_tracking = 0;
        int after_matching = 0;
        int after_ransac0 = 0;
        int after_ransac1 = 0;
        std::map<long long int, int> track_cnt;
        bool isFirstImg;
        bool IsInitialized;
        std::map<int, int> Grid;
    };

    class vis_mono_lk_cpu : public vis_imgproc<cv::Mat>
    {
    public:

        vis_mono_lk_cpu(set_base* _set, int cam_group_id = 0);

        ~vis_mono_lk_cpu() {}

        virtual PointCloud ProcessBatch(const double& t);

        bool Initialize();

        bool inBorder(const cv::Point2f& pt);

        void _reduceVector(std::vector<cv::Point2f>& v, std::vector<uchar> status);

        void _reduceVector(std::vector<int>& v, std::vector<uchar> status);

        void setMask();

        void rejectWithF();

        double distance(cv::Point2f& pt1, cv::Point2f& pt2);

        void integrateImuData(cv::Matx33f& cam0_R_p_c);

        void predictFeatureTracking(
            const std::vector<cv::Point2f>& input_pts,
            const cv::Matx33f& R_p_c,
            const cv::Vec4d& intrinsics,
            std::vector<cv::Point2f>& compensated_pts);

        void trackFeatures();

        void publish();

        void addnewFeatures();

        void drawFeatures();

        void rescalePoints(
            std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
            float& scaling_factor);

        void twoPointRansac(
            const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
            const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            const double& inlier_error,
            const double& success_probability,
            std::vector<int>& inlier_markers);

        void undistortPoints(
            const std::vector<cv::Point2f>& pts_in,
            const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            std::vector<cv::Point2f>& pts_out,
            const cv::Matx33d& rectification_matrix = cv::Matx33d::eye(),
            const cv::Vec4d& new_intrinsics = cv::Vec4d(1, 1, 0, 0));

    public:
        int row = 0, col = 0;                        ///< rows and columns
        cv::Mat mask;                            ///< to remove feature points that stay close
        cv::Mat prev_img, cur_img;                ///< mat form used for KLT
        std::vector<cv::Point2f> n_pts;                ///< used in add new features
        std::vector<cv::Point2f> prev_pts, cur_pts;    ///< record pts in cur and prev feature point
        std::vector<int> prev_ids, cur_ids;            ///< record id in cur and prev img
        std::vector<int> track_cnt;                    ///< track number

        double cur_time = 0;                        ///< current time
        double prev_time = 0;                        ///< previous time
        int n_id = 0;                                ///< ID number

        bool IsInitialized = false;                ///<  used for class parameter initial
        bool flow_back = true;                    ///<  KLT back to check

        int before_tracking = 0;                    ///<  cur feature number before tracking
        int after_tracking = 0;                    ///<  cur feature number after tracking    
        int after_setmask = 0;                    ///<  cur feature number after set mask
        int after_ransac = 0;                        ///<  cur feature number after ransac
        int after_twopoint_ransac = 0;            ///<  cur feature number after two point ransac

        std::string _cam0_distortion_model;            ///<  distortion model of camera
        cv::Vec4d _cam0_intrinsics;                ///<  intrinsics of camera
        cv::Vec4d _cam0_distortion_coeffs;        ///<  distortion coeffs of camera

        cv::Matx33d _R_cam0_imu;                ///<  rotation from camera to IMU
        cv::Vec3d _t_cam0_imu;                    ///<  translation from camera to IMU
    };
}

#endif
