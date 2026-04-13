#ifndef hwa_vis_proc_stereolk_h
#define hwa_vis_proc_stereolk_h
#include "hwa_set_base.h"
#include "hwa_set_vis.h"
#include "hwa_vis_proc_utility.h"
#include "hwa_vis_proc_imgproc.h"
#include "hwa_base_TimeCost.h"
#include "hwa_vis_proc_CudaFunction.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis {
    class vis_stereo_lk_gpu : public vis_imgproc<cv::cuda::GpuMat>
    {
    using Points = std::vector<cv::Point2f>;
    public:
        vis_stereo_lk_gpu(hwa_set::set_base* _set, int cam_group_id = 0);
        ~vis_stereo_lk_gpu() {}

        bool Initialize();
        void SetImage(std::pair<cv::Mat, cv::Mat> img) {
            _img0 = img.first;
            _img1 = img.second;
        
        };
        virtual PointCloud ProcessBatch();
        bool initializeFirstFrame();
        bool stereoMatch(cv::cuda::GpuMat& pts0, cv::cuda::GpuMat& pts1, cv::cuda::GpuMat& status, TRACK_TYPE T);
        bool stereoMatch();
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
        void drawFeaturesStereoCpu();
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
        std::vector<Triple> stereoSemiDenseMatch(cv::Mat& disparity);
        void calculateIPM(cv::Mat& ipm);
        void resetIMU();
        void setTrackType(TRACK_TYPE _T) { Tp = _T;}
        void setFilterType(FILTER_TYPE _F) { F = _F; }
        void setEpipolar(bool _useEpipolar, cv::Matx33d _E = cv::Matx33d::eye(), double _threshold = 0) {
            useEpipolar = _useEpipolar;
            Epipolar = _E;
            threshold = _threshold;
        }
        void setparam(cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1, cv::cuda::GpuMat& _d_status);
        void setparam_undistorted(cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1);
        void download(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier);
        void upload(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier);
        void filterNewPointsGPU();
        void filterPointsGPU();
        void setnewptsflags();
        void setflags();
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
        cv::cuda::GpuMat d_prev0, d_prev1, d_curr0, d_curr1;
        cv::cuda::Stream stream;
        cv::cuda::GpuMat d_prevPts0;
        cv::cuda::GpuMat d_prevPts1;
        cv::cuda::GpuMat d_currPts0;
        cv::cuda::GpuMat d_currPts1;
        cv::cuda::GpuMat d_currPts0_new;
        cv::cuda::GpuMat d_currPts1_new;
        cv::cuda::GpuMat d_prevPts0_undistorted;
        cv::cuda::GpuMat d_prevPts1_undistorted;
        cv::cuda::GpuMat d_currPts0_undistorted;
        cv::cuda::GpuMat d_currPts1_undistorted;
        cv::cuda::GpuMat d_currPts0_new_undistorted;
        cv::cuda::GpuMat d_currPts1_new_undistorted;
        cv::cuda::GpuMat d_status;
        cv::cuda::GpuMat d_status_new;

        cv::Matx33d _R_cam0_cam1;
        cv::Vec3d _t_cam0_cam1;
        cv::Matx33d _R_cam0_imu;
        cv::Vec3d _t_cam0_imu;
        cv::Matx33d _R_cam1_imu;

        std::string _cam0_distortion_model;
        cv::Vec4d _cam0_intrinsics;
        cv::Vec4d _cam0_distortion_coeffs;

        std::string _cam1_distortion_model;
        cv::Vec4d _cam1_intrinsics;
        cv::Vec4d _cam1_distortion_coeffs;

        int _stereo_match_count = 0;
        int _init_IPM_mapping_function = 0;
        cv::Mat _IPM_mapx;
        cv::Mat _IPM_mapy;
        cv::Mat _IPM_M0_1;
        cv::Mat _IPM_M0_2;

        cv::Mat _img0;
        cv::Mat _img1;
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

    class vis_stereo_lk_cpu : public vis_imgproc<cv::Mat>
    {
    public:

        vis_stereo_lk_cpu(set_base* _set, int cam_group_id = 0);

        ~vis_stereo_lk_cpu() {}
        
        bool Initialize();

        virtual PointCloud ProcessBatch(const double& t);

        void createImagePyramids();

        void initializeFirstFrame();

        void stereoMatch(
            const std::vector<cv::Point2f>& cam0_points,
            std::vector<cv::Point2f>& cam1_points,
            std::vector<unsigned char>& inlier_markers);

        void drawFeaturesStereo();

        void publish();

        void trackFeatures();

        void integrateImuData(cv::Matx33f& cam0_R_p_c, cv::Matx33f& cam1_R_p_c);

        void predictFeatureTracking(
            const std::vector<cv::Point2f>& input_pts,
            const cv::Matx33f& R_p_c,
            const cv::Vec4d& intrinsics,
            std::vector<cv::Point2f>& compensated_pts);

        void twoPointRansac(
            const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
            const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            const double& inlier_error,
            const double& success_probability,
            std::vector<int>& inlier_markers);

        void addNewFeatures();

        void pruneGridFeatures();

        std::vector<cv::Point2f> distortPoints(
            const std::vector<cv::Point2f>& pts_in,
            const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs);

        void undistortPoints(
            const std::vector<cv::Point2f>& pts_in,
            const cv::Vec4d& intrinsics,
            const std::string& distortion_model,
            const cv::Vec4d& distortion_coeffs,
            std::vector<cv::Point2f>& pts_out,
            const cv::Matx33d& rectification_matrix = cv::Matx33d::eye(),
            const cv::Vec4d& new_intrinsics = cv::Vec4d(1, 1, 0, 0));

        static bool featureCompareByResponse(
            const FeaturePoint& f1,
            const FeaturePoint& f2);

        template <typename T>
        void removeUnmarkedElements(
            const std::vector<T>& raw_vec,
            const std::vector<unsigned char>& markers,
            std::vector<T>& refined_vec);

        void rescalePoints(
            std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
            float& scaling_factor);

        static bool keyPointCompareByResponse(
            const cv::KeyPoint& pt1,
            const cv::KeyPoint& pt2);

        static bool featureCompareByLifetime(
            const FeaturePoint& f1,
            const FeaturePoint& f2);

        std::vector<Eigen::Vector3d> stereoSemiDenseMatch(cv::Mat& disparity);
        void calculateIPM(cv::Mat& ipm);

    public:
        typedef std::map<int, std::vector<FeaturePoint> > GridFeatures;

        cv::Ptr<cv::Feature2D> detector_ptr;                ///< Fast feature point detector

        ONE_FRAME curr_img_msg;                                ///< store information about cur img
        ONE_FRAME prev_img_msg;                                ///< store information about pre img,used for tracking in next frame
        cv::Mat cam0_curr_img;                                ///< cur img0 clone
        cv::Mat cam1_curr_img;                                ///< cur img1 clone
        std::vector<cv::Mat> curr_cam0_pyramid;                ///< cur img0 pyramid
        std::vector<cv::Mat> curr_cam1_pyramid;                ///< cur img1 pyramid
        std::vector<cv::Mat> prev_cam0_pyramid;                ///< pre img0 pyramid used for optical flow
        long long int next_feature_id = 0;                    ///< ID for the next new feautre
        std::shared_ptr<GridFeatures> prev_features_ptr;    ///< store all feature observations in pre frame
        std::shared_ptr<GridFeatures> curr_features_ptr;    ///< store all feature observations in cur frame


        int before_tracking = 0;        ///< cur feature number before tracking
        int after_tracking = 0;        ///< cur feature number after tracking    
        int after_matching = 0;        ///< cur feature number after stereomatch
        int after_ransac = 0;            ///< cur feature number after two point ransac
        std::map<long long int, int> track_cnt;        ///< not used in stereo optical flow
        bool isFirstImg;            ///< A indicator to show if it is the first frame
        bool IsInitialized;            ///< A indicator to show if the system is initialized

        /* some parameter in parent class ,but form from eigen to cv */
        cv::Matx33d _R_cam0_cam1;
        cv::Vec3d _t_cam0_cam1;
        cv::Matx33d _R_cam0_imu;
        cv::Vec3d _t_cam0_imu;
        cv::Matx33d _R_cam1_imu;

        std::string _cam0_distortion_model;
        cv::Vec4d _cam0_intrinsics;
        cv::Vec4d _cam0_distortion_coeffs;

        std::string _cam1_distortion_model;
        cv::Vec4d _cam1_intrinsics;
        cv::Vec4d _cam1_distortion_coeffs;

        int _stereo_match_count = 0;
        int _init_IPM_mapping_function = 0;
        cv::Mat _IPM_mapx;
        cv::Mat _IPM_mapy;
        cv::Mat _IPM_M0_1;
        cv::Mat _IPM_M0_2;
        /* some parameter in parent class ,but form from eigen to cv */

    };

}

#endif
