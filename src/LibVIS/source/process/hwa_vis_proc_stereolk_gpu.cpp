#include "hwa_vis_proc_stereolk.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"

hwa_vis::vis_stereo_lk_gpu::vis_stereo_lk_gpu(hwa_set::set_base* _set, int cam_group_id) :vis_imgproc<cv::cuda::GpuMat>(_set, cam_group_id),
isFirstImg(true),
IsInitialized(false)
{
    next_feature_id = 0;
    out_img_dense = std::make_shared<cv::cuda::GpuMat>(100, 100, CV_8UC1);
    out_img_IPM = std::make_shared<cv::cuda::GpuMat>(100, 100, CV_8UC1);
    TimeCostDebugOutFile.open("TimeCostFeatureTrack.txt", std::ios::out | std::ios::trunc);
    lk = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(patch_size, patch_size),
        pyramid_levels,
        max_iteration,
        true);
    gftt_init = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, 1000, 0.01, 0, 2, true, 0.04);
    gftt_fast = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, 1000, 0.01, 0, 2, true, 0.04);
}

bool hwa_vis::vis_stereo_lk_gpu::Initialize()
{
    eigen2cv(R_cam0_cam1, _R_cam0_cam1);
    eigen2cv(t_cam0_cam1, _t_cam0_cam1);
    eigen2cv(R_cam0_imu, _R_cam0_imu);
    eigen2cv(t_cam0_imu, _t_cam0_imu);
    _R_cam1_imu = _R_cam0_imu * _R_cam0_cam1.t();
    _cam0_distortion_model = distortion2str(cam0_distortion_model);
    _cam0_intrinsics << cam0_intrinsics(0), cam0_intrinsics(1), cam0_intrinsics(2), cam0_intrinsics(3);
    _cam0_distortion_coeffs << cam0_distortion_coeffs(0), cam0_distortion_coeffs(1), cam0_distortion_coeffs(2), cam0_distortion_coeffs(3);

    _cam1_distortion_model = distortion2str(cam1_distortion_model);
    _cam1_intrinsics << cam1_intrinsics(0), cam1_intrinsics(1), cam1_intrinsics(2), cam1_intrinsics(3);
    _cam1_distortion_coeffs << cam1_distortion_coeffs(0), cam1_distortion_coeffs(1), cam1_distortion_coeffs(2), cam1_distortion_coeffs(3);
    return true;
}

bool hwa_vis::vis_stereo_lk_gpu::TrackPointsPyrLK_CUDA(
    const cv::cuda::GpuMat& prev_img,
    const cv::cuda::GpuMat& next_img,
    cv::cuda::GpuMat& d_prevPts,
    cv::cuda::GpuMat& d_currPts,
    cv::cuda::GpuMat& d_status,
    int patch_size,
    int pyramid_levels,
    int max_iteration,
    bool use_initial_flow,
    int device_id
) {
    int nGPU = cv::cuda::getCudaEnabledDeviceCount();
    if (nGPU <= 0) return false;
    cv::cuda::setDevice(device_id);
    if (prev_img.empty() || d_prevPts.empty())
        return false;
    auto toGrayU8 = [](const cv::cuda::GpuMat& src) -> cv::cuda::GpuMat {
        if (src.type() == CV_8UC1) return src;
        cv::cuda::GpuMat g;
        if (src.channels() == 3) cv::cvtColor(src, g, cv::COLOR_BGR2GRAY);
        else if (src.channels() == 4) cv::cvtColor(src, g, cv::COLOR_BGRA2GRAY);
        else throw std::runtime_error("Unsupported image type for CUDA LK");
        return g;
        };
    cv::cuda::GpuMat prev = toGrayU8(prev_img);
    cv::cuda::GpuMat next = toGrayU8(next_img);
    if (prev.size() != next.size()) return false;
    if (patch_size < 3 || (patch_size % 2) == 0) patch_size = std::max(3, patch_size | 1);
    if (prev.cols <= patch_size || prev.rows <= patch_size) return false;

    cv::cuda::GpuMat d_err;
    lk->calc(prev, next, d_prevPts, d_currPts, d_status, d_err);
    return true;
}

hwa_vis::PointCloud hwa_vis::vis_stereo_lk_gpu::ProcessBatch()
{
    base_scopedtimer timer("ProcessBatch()", TimeCostDebugOutFile, TimeCostOut);
    width = imageGroup.second.first.cols;
    height = imageGroup.second.first.rows;
    grid_height = height / grid_row;
    grid_width = width / grid_col;
    curr_img_t = imageGroup.first;
    d_curr0 = imageGroup.second.first;
    d_curr1 = imageGroup.second.second;
    out_img = std::make_shared<cv::cuda::GpuMat>(height, width * 2, CV_8UC3);
    cv::cuda::cvtColor(d_curr0, out_img->colRange(0, width), cv::COLOR_GRAY2BGR);
    cv::cuda::cvtColor(d_curr1, out_img->colRange(width, width * 2), cv::COLOR_GRAY2BGR);

    if (!IsInitialized)
    {
        Initialize();
        IsInitialized = true;
    }
    if (isFirstImg)
    {        
        isFirstImg = !initializeFirstFrame();  
    }
    else
    {
        trackFeatures();
        ManageGrid();
        addNewFeatures();
        PruneGrid();
        drawFeaturesStereoGpu();
    }
    publish();

    std::swap(d_prev0, d_curr0);
    std::swap(d_prev1, d_curr1);
    prev_img_t = curr_img_t;

    if (d_currPts0.empty()) {
        d_prevPts0.release();
        d_prevPts1.release();
        return _pointCloud;
    }

    d_prevPts0 = d_currPts0.clone();
    d_prevPts1 = d_currPts1.clone();
    return _pointCloud;
}

bool hwa_vis::vis_stereo_lk_gpu::initializeFirstFrame()
{
    gftt_init->detect(d_curr0, d_currPts0, cv::noArray(), stream);    
    stream.waitForCompletion();
    Init();
    if(!stereoMatch()) return false;
    setFilterType(MATCH_FILTER);
    setflags();
    filterPointsGPU();
    InitializeGrid();
    return true;
}

bool hwa_vis::vis_stereo_lk_gpu::stereoMatch(cv::cuda::GpuMat& pts0, cv::cuda::GpuMat& pts1, cv::cuda::GpuMat& status, TRACK_TYPE T)
{
    if (pts0.cols == 0)
        return false;
    cv::cuda::GpuMat cam0_points_undistorted;
    std::string _cam0_distortion_model = distortion2str(cam0_distortion_model);
    undistortPoints(pts0, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, cam0_points_undistorted,
        _R_cam0_cam1);
    pts1 = distortPoints(cam0_points_undistorted, _cam1_intrinsics,
        _cam1_distortion_model, _cam1_distortion_coeffs);

    {
        base_scopedtimer timer1("TrackPointsPyrLK_CUDA()", TimeCostDebugOutFile, TimeCostOut);
        TrackPointsPyrLK_CUDA(
            d_curr0, d_curr1,
            pts0, pts1, status,
            patch_size, pyramid_levels, max_iteration
        );
    }
    cv::Matx33d t_cam0_cam1_hat(
        0.0, -t_cam0_cam1[2], t_cam0_cam1[1],
        t_cam0_cam1[2], 0.0, -t_cam0_cam1[0],
        -t_cam0_cam1[1], t_cam0_cam1[0], 0.0);
    cv::Matx33d E = t_cam0_cam1_hat * _R_cam0_cam1;

    cv::cuda::GpuMat cam1_points_undistorted;
    undistortPoints(
        pts1, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, cam1_points_undistorted);
    double norm_pixel_unit = 4.0 / (
        _cam0_intrinsics[0] + _cam0_intrinsics[1] +
        _cam1_intrinsics[0] + _cam1_intrinsics[1]);

    setTrackType(T);
    setEpipolar(false, E, stereo_threshold * norm_pixel_unit);
    setparam_undistorted(cam0_points_undistorted, cam1_points_undistorted);
    return true;
}

bool hwa_vis::vis_stereo_lk_gpu::stereoMatch()
{
    base_scopedtimer timer1("stereoMatch()", TimeCostDebugOutFile, TimeCostOut);
    if (d_currPts0.cols == 0)
        return false;
    cv::cuda::GpuMat cam0_points_undistorted;
    std::string _cam0_distortion_model = distortion2str(cam0_distortion_model);
    undistortPoints(d_currPts0, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, cam0_points_undistorted,
        _R_cam0_cam1);
    d_currPts1 = distortPoints(cam0_points_undistorted, _cam1_intrinsics,
        _cam1_distortion_model, _cam1_distortion_coeffs);

    {
        base_scopedtimer timer1("StereoMatch_TrackPointsPyrLK_CUDA()", TimeCostDebugOutFile, TimeCostOut);  
        TrackPointsPyrLK_CUDA(
            d_curr0, d_curr1,
            d_currPts0, d_currPts1, d_status,
            patch_size, pyramid_levels, max_iteration
        );
    }

    cv::Matx33d t_cam0_cam1_hat(
        0.0, -t_cam0_cam1[2], t_cam0_cam1[1],
        t_cam0_cam1[2], 0.0, -t_cam0_cam1[0],
        -t_cam0_cam1[1], t_cam0_cam1[0], 0.0);
    cv::Matx33d E = t_cam0_cam1_hat * _R_cam0_cam1;

    cv::cuda::GpuMat cam1_points_undistorted;
    undistortPoints(
        d_currPts1, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, cam1_points_undistorted);  
    double norm_pixel_unit = 4.0 / (
        _cam0_intrinsics[0] + _cam0_intrinsics[1] +
        _cam1_intrinsics[0] + _cam1_intrinsics[1]);

    setTrackType(STEREO_MATCH);
    setEpipolar(false, E, stereo_threshold * norm_pixel_unit);
    setparam_undistorted(cam0_points_undistorted, cam1_points_undistorted);
    return true;
}

bool hwa_vis::vis_stereo_lk_gpu::monotrack() {
    if (d_prevPts0.cols == 0) return false;
    base_scopedtimer timer1("monotrack()", TimeCostDebugOutFile, TimeCostOut);
    cv::Matx33f cam0_R_p_c;
    cv::Matx33f cam1_R_p_c;
    integrateImuData(cam0_R_p_c, cam1_R_p_c);
    Points prev_cam0_points;
    Points curr_cam0_points;
    gcuda::_download(d_prevPts0, prev_cam0_points);
    predictFeatureTracking(prev_cam0_points, cam0_R_p_c, _cam0_intrinsics, curr_cam0_points);
    gcuda::_upload(d_currPts0, curr_cam0_points);
    {
        base_scopedtimer timer1("MonoTrack_TrackPointsPyrLK_CUDA_monotrack()", TimeCostDebugOutFile, TimeCostOut);
        TrackPointsPyrLK_CUDA(
            d_prev0, d_curr0,
            d_prevPts0, d_currPts0, d_status,
            patch_size, pyramid_levels, max_iteration
        );
    }
    setTrackType(MONO_TRACK);
    setEpipolar(false);
    return true;
}

void hwa_vis::vis_stereo_lk_gpu::trackFeatures()
{   
    base_scopedtimer timer("trackFeatures()", TimeCostDebugOutFile, TimeCostOut);
    before_tracking = d_prevPts0.cols;
    if (before_tracking == 0) return;

    setFilterType(TOTAL_FILTER);

    if(!monotrack()) return;
    setflags();
    filterPointsGPU();
    after_tracking = d_currPts0.cols;

    if(!stereoMatch()) return;
    setflags();
    filterPointsGPU();
    after_matching = d_currPts0.cols;

    cv::Matx33f cam0_R_p_c;
    cv::Matx33f cam1_R_p_c;
    integrateImuData(cam0_R_p_c, cam1_R_p_c);
    resetIMU();
    twoPointRansac(
        d_prevPts0, d_currPts0,
        cam0_R_p_c, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, ransac_threshold, 0.99
    );
    after_ransac0 = d_currPts0.cols;
    twoPointRansac(
        d_prevPts1, d_currPts1,
        cam1_R_p_c, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, ransac_threshold, 0.99
    );
    after_ransac1 = d_currPts0.cols;

    std::cout << "candidates: " << before_tracking
        << "; after_track: " << static_cast<double>(after_tracking) / before_tracking
        << "; after_match: " << static_cast<double>(after_matching) / before_tracking
        << "; after_ransac0: " << static_cast<double>(after_ransac0) / before_tracking
        << "; after_ransac1: " << static_cast<double>(after_ransac1) / before_tracking << std::endl;
    return;
}

void hwa_vis::vis_stereo_lk_gpu::drawFeaturesStereoGpu()
{
    int N = d_currPts0.cols;
    if (N == 0) return;
    base_scopedtimer timer("drawFeaturesStereoGpu()", TimeCostDebugOutFile, TimeCostOut);
    DrawPts(out_img);
    return;
}

void hwa_vis::vis_stereo_lk_gpu::drawFeaturesStereoCpu()
{
    cv::Scalar tracked(0, 255, 0);
    cv::Scalar new_feature(0, 255, 255);

    cv::Mat _outimg = cv::Mat(height, width * 2, CV_8UC3);
    cv::Mat curr0, curr1;
    d_curr0.download(curr0);
    d_curr1.download(curr1);
    cv::cvtColor(curr0, _outimg.colRange(0, width), cv::COLOR_GRAY2BGR);
    cv::cvtColor(curr1, _outimg.colRange(width, width * 2), cv::COLOR_GRAY2BGR);

    for (int i = 1; i < grid_row; ++i)
    {
        cv::Point pt1(0, i * grid_height);
        cv::Point pt2(width * 2, i * grid_height);
        cv::line(_outimg, pt1, pt2, cv::Scalar(255, 0, 0));
    }
    for (int i = 1; i < grid_col; ++i)
    {
        cv::Point pt1(i * grid_width, 0);
        cv::Point pt2(i * grid_width, height);
        cv:line(_outimg, pt1, pt2, cv::Scalar(255, 0, 0));
    }
    for (int i = 1; i < grid_col; ++i)
    {
        cv::Point pt1(i * grid_width + width, 0);
        cv::Point pt2(i * grid_width + width, height);
        cv::line(_outimg, pt1, pt2, cv::Scalar(255, 0, 0));
    }

    Points pts0, pts1;
    gcuda::_download(d_currPts0, pts0);
    gcuda::_download(d_currPts1, pts1);

    for (int i = 0; i<pts0.size();i++)
    {
        cv::Point2f curr_pt0 = pts0[i];
        cv::Point2f curr_pt1 = pts1[i] + cv::Point2f(width, 0.0);

        circle(_outimg, curr_pt0, 3, tracked, -1);
        circle(_outimg, curr_pt1, 3, tracked, -1);
    }

    out_img->upload(_outimg);

    return;
}


void hwa_vis::vis_stereo_lk_gpu::publish()
{
    if (d_currPts0.cols == 0) return;
    _pointCloud.features.clear();
    FeaturePoint feature_msg;
    _pointCloud.time = curr_img_t;

    Points curr_cam0_points_undistorted(0);
    Points curr_cam1_points_undistorted(0);
    undistortPoints(
        d_currPts0, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, curr_cam0_points_undistorted);
    undistortPoints(
        d_currPts1, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, curr_cam1_points_undistorted);
    thrust::host_vector<int> _ids(ids.begin(), ids.end());
    thrust::host_vector<int> _lifetime(lifetime.begin(), lifetime.end());

    for (int i = 0; i < d_currPts0.cols; ++i)
    {
        feature_msg.id = _ids[i];
        feature_msg.lifetime = _lifetime[i];
        feature_msg.cam0_point = curr_cam0_points_undistorted[i];
        feature_msg.cam1_point = curr_cam1_points_undistorted[i];
        _pointCloud.features.push_back(feature_msg);
    }
    return;
}

void hwa_vis::vis_stereo_lk_gpu::integrateImuData(
    cv::Matx33f& cam0_R_p_c, cv::Matx33f& cam1_R_p_c)
{ 
    //mean_ang_vel /= ts_p_c;
    cv::Vec3f cam0_mean_ang_vel = _R_cam0_imu.t() * mean_ang_vel;
    cv::Vec3f cam1_mean_ang_vel = _R_cam1_imu.t() * mean_ang_vel;

    double dtime = curr_img_t - prev_img_t;
    Rodrigues(cam0_mean_ang_vel*dtime, cam0_R_p_c);
    Rodrigues(cam1_mean_ang_vel*dtime, cam1_R_p_c);
    cam0_R_p_c = cam0_R_p_c.t();
    cam1_R_p_c = cam1_R_p_c.t();
    return;
}

void hwa_vis::vis_stereo_lk_gpu::predictFeatureTracking(
    const std::vector<cv::Point2f>& input_pts, 
    const cv::Matx33f& R_p_c,             
    const cv::Vec4d& intrinsics,          
    std::vector<cv::Point2f>& compensated_pts) 
{
    if (input_pts.size() == 0)
    {
        compensated_pts.clear();
        return;
    }
    compensated_pts.resize(input_pts.size());
    cv::Matx33f K(
        intrinsics[0], 0.0, intrinsics[2], 
        0.0, intrinsics[1], intrinsics[3], 
        0.0, 0.0, 1.0);                 

    //单应性变换
    cv::Matx33f H = K * R_p_c * K.inv();
    for (int i = 0; i < input_pts.size(); ++i)
    {
        cv::Vec3f p1(input_pts[i].x, input_pts[i].y, 1.0f);
        cv::Vec3f p2 = H * p1;
        compensated_pts[i].x = p2[0] / p2[2];
        compensated_pts[i].y = p2[1] / p2[2];
    }
    return;
}

void hwa_vis::vis_stereo_lk_gpu::twoPointRansac(
    cv::cuda::GpuMat& pts1, cv::cuda::GpuMat& pts2,
    const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
    const std::string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability
 )
{
    std::vector<int> inlier_markers;    
    if (pts1.cols != pts2.cols)
        printf("Sets of different size (%d and %d) are used...\n",
            pts1.cols, pts2.cols);

    double norm_pixel_unit = 2.0 / (intrinsics[0] + intrinsics[1]);
    int iter_num = static_cast<int>(
        ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));
    inlier_markers.clear();
    inlier_markers.resize(pts1.cols, 1);

    Points pts1_undistorted(pts1.cols);
    Points pts2_undistorted(pts2.cols);
    undistortPoints(
        pts1, intrinsics, distortion_model,
        distortion_coeffs, pts1_undistorted);
    undistortPoints(
        pts2, intrinsics, distortion_model,
        distortion_coeffs, pts2_undistorted);
    
    for (auto& pt : pts1_undistorted)
    {
        cv::Vec3f pt_h(pt.x, pt.y, 1.0f);
        cv::Vec3f pt_hc = R_p_c * pt_h;
        pt.x = pt_hc[0];
        pt.y = pt_hc[1];
    }

    float scaling_factor = 0.0f;
    rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
    norm_pixel_unit *= scaling_factor;
    std::vector<cv::Point2d> pts_diff(pts1_undistorted.size());
    for (int i = 0; i < pts1_undistorted.size(); ++i)
        pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

    double mean_pt_distance = 0.0;
    int raw_inlier_cntr = 0;
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
        if (distance > 50.0 * norm_pixel_unit)
        {
            inlier_markers[i] = 0;
        }
        else
        {
            mean_pt_distance += distance;
            ++raw_inlier_cntr;
        }
    }
    mean_pt_distance /= raw_inlier_cntr;
    if (raw_inlier_cntr < 3)
    {
        for (auto& marker : inlier_markers)
            marker = 0;
        return;
    }

    if (mean_pt_distance < norm_pixel_unit)
    {
        for (int i = 0; i < pts_diff.size(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;
            if (sqrt(pts_diff[i].dot(pts_diff[i])) > inlier_error * norm_pixel_unit)
                inlier_markers[i] = 0;
        }
        return;
    }

    Matrix coeff_t(pts_diff.size(), 3);
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        coeff_t(i, 0) = pts_diff[i].y;
        coeff_t(i, 1) = -pts_diff[i].x;
        coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y - pts1_undistorted[i].y * pts2_undistorted[i].x;
    }

    std::vector<int> raw_inlier_idx;
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] != 0)
            raw_inlier_idx.push_back(i);
    }

    std::vector<int> best_inlier_set;
    double best_error = 1e10;
    cv::RNG randomNumber;

    for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx)
    {
        int pair_idx1_temp = randomNumber.uniform(0, raw_inlier_idx.size());
        int pair_idx1 = raw_inlier_idx[pair_idx1_temp];
        int idx_diff = randomNumber.uniform(1, raw_inlier_idx.size());
        int pair_idx2_temp = pair_idx1_temp + idx_diff < raw_inlier_idx.size() ? pair_idx1_temp + idx_diff : pair_idx1_temp + idx_diff - raw_inlier_idx.size();
        int pair_idx2 = raw_inlier_idx[pair_idx2_temp];
        Eigen::Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
        Eigen::Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
        Eigen::Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
        std::vector<double> coeff_l1_norm(3);
        coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
        coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
        coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
        int base_indicator = min_element(coeff_l1_norm.begin(), coeff_l1_norm.end()) - coeff_l1_norm.begin();

        Triple model(0.0, 0.0, 0.0);
        if (base_indicator == 0)
        {
            Eigen::Matrix2d A;
            A << coeff_ty, coeff_tz;
            Eigen::Vector2d solution = A.inverse() * (-coeff_tx);
            model(0) = 1.0;
            model(1) = solution(0);
            model(2) = solution(1);
        }
        else if (base_indicator == 1)
        {
            Eigen::Matrix2d A;
            A << coeff_tx, coeff_tz;
            Eigen::Vector2d solution = A.inverse() * (-coeff_ty);
            model(0) = solution(0);
            model(1) = 1.0;
            model(2) = solution(1);
        }
        else {
            Eigen::Matrix2d A;
            A << coeff_tx, coeff_ty;
            Eigen::Vector2d solution = A.inverse() * (-coeff_tz);
            model(0) = solution(0);
            model(1) = solution(1);
            model(2) = 1.0;
        }

        Vector error = coeff_t * model;
        std::vector<int> inlier_set;
        for (int i = 0; i < error.rows(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;
            if (std::abs(error(i)) < inlier_error * norm_pixel_unit)
                inlier_set.push_back(i);
        }

        if (inlier_set.size() < 0.2 * pts1_undistorted.size())
            continue;

        Vector coeff_tx_better(inlier_set.size());
        Vector coeff_ty_better(inlier_set.size());
        Vector coeff_tz_better(inlier_set.size());
        for (int i = 0; i < inlier_set.size(); ++i) {
            coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
            coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
            coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
        }

        Triple model_better(0.0, 0.0, 0.0);
        if (base_indicator == 0) {
            Matrix A(inlier_set.size(), 2);
            A << coeff_ty_better, coeff_tz_better;
            Eigen::Vector2d solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
            model_better(0) = 1.0;
            model_better(1) = solution(0);
            model_better(2) = solution(1);
        }
        else if (base_indicator == 1) {
            Matrix A(inlier_set.size(), 2);
            A << coeff_tx_better, coeff_tz_better;
            Eigen::Vector2d solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
            model_better(0) = solution(0);
            model_better(1) = 1.0;
            model_better(2) = solution(1);
        }
        else {
            Matrix A(inlier_set.size(), 2);
            A << coeff_tx_better, coeff_ty_better;
            Eigen::Vector2d solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
            model_better(0) = solution(0);
            model_better(1) = solution(1);
            model_better(2) = 1.0;
        }
        Vector new_error = coeff_t * model_better;

        double this_error = 0.0;
        for (const auto& inlier_idx : inlier_set)
            this_error += std::abs(new_error(inlier_idx));
        this_error /= inlier_set.size();
        if (inlier_set.size() > best_inlier_set.size())
        {
            best_error = this_error;
            best_inlier_set = inlier_set;
        }
    }
    inlier_markers.clear();
    inlier_markers.resize(pts1.cols, 0);
    for (const auto& inlier_idx : best_inlier_set)
        inlier_markers[inlier_idx] = 1;

    gcuda::_upload(d_status, inlier_markers);
    setEpipolar(false);
    setflags();
    filterPointsGPU();
    return;
}

void hwa_vis::vis_stereo_lk_gpu::addNewFeatures()
{
    base_scopedtimer timer("addNewFeatures()", TimeCostDebugOutFile, TimeCostOut);
    Points curr_pts0;
    cv::cuda::GpuMat d_mask(height, width, CV_8UC1);
    d_mask.setTo(cv::Scalar::all(255), stream);
    int N = d_currPts0.cols;
    if (N != 0) {
        gcuda::_download(d_currPts0, curr_pts0);
        for (const auto& feature : curr_pts0) {
            int y = static_cast<int>(feature.y);
            int x = static_cast<int>(feature.x);
            int up = std::max(y - 2, 0);
            int down = std::min(y + 3, height);
            int left = std::max(x - 2, 0);
            int right = std::min(x + 3, width);
            d_mask(cv::Range(up, down), cv::Range(left, right)).setTo(cv::Scalar::all(0), stream);
        }
    }
    {
        base_scopedtimer timer("gftt->detect", TimeCostDebugOutFile, TimeCostOut);
        gftt_fast->detect(d_curr0, d_currPts0_new, d_mask, stream);
        stream.waitForCompletion();
    }
    if(!stereoMatch(d_currPts0_new, d_currPts1_new, d_status_new, STEREO_DETECT)) return;
    InitNewPts();
    setnewptsflags();
    filterNewPointsGPU();
    InitializeNewPtsGrid();
    AddNewPts();
    return;
}

cv::cuda::GpuMat hwa_vis::vis_stereo_lk_gpu::distortPoints(
    const cv::cuda::GpuMat& pts_in,             
    const cv::Vec4d& intrinsics,                   
    const std::string& distortion_model,              
    const cv::Vec4d& distortion_coeffs) 
{
    const cv::Matx33d K(intrinsics[0], 0.0, intrinsics[2],
        0.0, intrinsics[1], intrinsics[3],
        0.0, 0.0, 1.0);

    cv::cuda::GpuMat pts_out;
    Points _pts_in, _pts_out;
    pts_in.download(_pts_in);
    if (distortion_model == "radtan") 
    {
        std::vector<cv::Point3f> homogenous_pts;
        cv::convertPointsToHomogeneous(_pts_in, homogenous_pts);
        cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
            distortion_coeffs, _pts_out);
    }
    else if (distortion_model == "equidistant")
    {
        cv::fisheye::distortPoints(_pts_in, _pts_out, K, distortion_coeffs);
    }
    else
    {
        std::vector<cv::Point3f> homogenous_pts;
        cv::convertPointsToHomogeneous(_pts_in, homogenous_pts);
        cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
            distortion_coeffs, _pts_out);
    }
    pts_out.upload(_pts_out);
    return pts_out;
}

void hwa_vis::vis_stereo_lk_gpu::undistortPoints(
    const cv::cuda::GpuMat& pts_in,
    const cv::Vec4d& intrinsics,
    const std::string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    Points& pts_out,
    const cv::Matx33d& rectification_matrix,
    const cv::Vec4d& new_intrinsics)
{
    if (pts_in.cols == 0)
        return;

    const cv::Matx33d K(
        intrinsics[0], 0.0, intrinsics[2],
        0.0, intrinsics[1], intrinsics[3],
        0.0, 0.0, 1.0);
    const cv::Matx33d K_new(
        new_intrinsics[0], 0.0, new_intrinsics[2],
        0.0, new_intrinsics[1], new_intrinsics[3],
        0.0, 0.0, 1.0);

    Points _pts_in;
    gcuda::_download(pts_in, _pts_in);

    if (distortion_model == "radtan")
    {
        cv::undistortPoints(_pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else if (distortion_model == "equidistant")
    {
        cv::fisheye::undistortPoints(_pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else
    {
        cv::undistortPoints(_pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    return;
}


void hwa_vis::vis_stereo_lk_gpu::undistortPoints(
    const cv::cuda::GpuMat& pts_in,             
    const cv::Vec4d& intrinsics,                
    const std::string& distortion_model,             
    const cv::Vec4d& distortion_coeffs,         
    cv::cuda::GpuMat& pts_out,               
    const cv::Matx33d& rectification_matrix,      
    const cv::Vec4d& new_intrinsics)
{
    if (pts_in.cols == 0)
        return;

    const cv::Matx33d K(
        intrinsics[0], 0.0, intrinsics[2],
        0.0, intrinsics[1], intrinsics[3],
        0.0, 0.0, 1.0);
    const cv::Matx33d K_new(
        new_intrinsics[0], 0.0, new_intrinsics[2],
        0.0, new_intrinsics[1], new_intrinsics[3],
        0.0, 0.0, 1.0);

    Points _pts_in, _pts_out;
    gcuda::_download(pts_in, _pts_in);

    if (distortion_model == "radtan")  
    {
        cv::undistortPoints(_pts_in, _pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else if (distortion_model == "equidistant") 
    {
        cv::fisheye::undistortPoints(_pts_in, _pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else
    {
        cv::undistortPoints(_pts_in, _pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    gcuda::_upload(pts_out, _pts_out);
    return;
}


bool hwa_vis::vis_stereo_lk_gpu::featureCompareByResponse(
    const FeaturePoint& f1,
    const FeaturePoint& f2)
{

    return f1.response > f2.response;
}

template <typename Tem>
void hwa_vis::vis_stereo_lk_gpu::removeUnmarkedElements(
    const std::vector<Tem>& raw_vec,
    const std::vector<unsigned char>& markers,
    std::vector<Tem>& refined_vec)
{
    if (raw_vec.size() != markers.size())
    {
        printf("The input size of raw_vec(%d) and markers(%d) does not match...\n",
            raw_vec.size(), markers.size());
    }
    for (int i = 0; i < markers.size(); ++i)
    {
        if (markers[i] == 0) continue;
        refined_vec.push_back(raw_vec[i]);
    }
    return;
}

void hwa_vis::vis_stereo_lk_gpu::rescalePoints(
    std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
    float& scaling_factor)
{
    scaling_factor = 0.0f;


    for (int i = 0; i < pts1.size(); ++i)
    {
        scaling_factor += sqrt(pts1[i].dot(pts1[i]));
        scaling_factor += sqrt(pts2[i].dot(pts2[i]));
    }


    scaling_factor = (pts1.size() + pts2.size()) /
        scaling_factor * sqrt(2.0f);


    for (int i = 0; i < pts1.size(); ++i)
    {
        pts1[i] *= scaling_factor;
        pts2[i] *= scaling_factor;
    }

    return;
}

bool hwa_vis::vis_stereo_lk_gpu::keyPointCompareByResponse(
    const cv::KeyPoint& pt1,
    const cv::KeyPoint& pt2)
{
    return pt1.response > pt2.response;
}

bool hwa_vis::vis_stereo_lk_gpu::featureCompareByLifetime(
    const FeaturePoint& f1,
    const FeaturePoint& f2)
{
    return f1.lifetime > f2.lifetime;
}

std::vector<Triple> hwa_vis::vis_stereo_lk_gpu::stereoSemiDenseMatch(cv::Mat& disparity)
{
    std::vector<Triple> pl;
    cv::Matx33d K0(_cam0_intrinsics(0), 0, _cam0_intrinsics(2),
        0, _cam0_intrinsics(1), _cam0_intrinsics(3),
        0, 0, 1);
    cv::Matx33d K1(_cam1_intrinsics(0), 0, _cam1_intrinsics(2),
        0, _cam1_intrinsics(1), _cam1_intrinsics(3),
        0, 0, 1);
    cv::Mat R0, P0, R1, P1, Q;
    stereoRectify(K0, _cam0_distortion_coeffs, K1, _cam1_distortion_coeffs, d_curr0.size(),
        _R_cam0_cam1, _t_cam0_cam1, R0, R1, P0, P1, Q);

    cv::Mat M0_1, M0_2, M1_1, M1_2;
    initUndistortRectifyMap(K0, _cam0_distortion_coeffs, R0, P0, d_curr0.size(), CV_32FC1, M0_1, M0_2);
    initUndistortRectifyMap(K1, _cam1_distortion_coeffs, R1, P1, d_curr1.size(), CV_32FC1, M1_1, M1_2);
    cv::Mat img0_rectify, img1_rectify, img0_sobel;
    cv::Mat img0_rectify_new;
    remap(d_curr0, img0_rectify, M0_1, M0_2, cv::INTER_LINEAR);
    remap(d_curr1, img1_rectify, M1_1, M1_2, cv::INTER_LINEAR);

    Matrix P1_e;
    cv2eigen(P1, P1_e);
    double f = P1_e(0, 0);
    double bl = -P1_e(0, 3);
    double cx = P1_e(0, 2);
    double cy = P1_e(1, 2);

    int img_height = img0_rectify.rows;
    int img_width = img0_rectify.cols;
    int resize_factor = 2;
    if (resize_factor != 1)
    {
        cv::resize(img0_rectify, img0_rectify, cv::Size(img_width / resize_factor, img_height / resize_factor));
        cv::resize(img1_rectify, img1_rectify, cv::Size(img_width / resize_factor, img_height / resize_factor));
    }


    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 64, 7, 0, 0, 0, 0, 30, 50, 1);
    sgbm->compute(img0_rectify, img1_rectify, disparity);

    bilateralFilter(img0_rectify, img0_rectify_new, 5, 10, 15);
    Canny(img0_rectify_new, img0_sobel, 100, 150);
    //fld->detect(img0_rectify, lines_fld);


    double x, y, z;
    for (int i = 0; i < disparity.rows; i++)
        for (int j = 0; j < disparity.cols; j++)
        {
            double depth = bl / (disparity.at<uint16_t>(i, j) / 16.0 * resize_factor);
            if (depth <= 0) continue;
            x = (j * resize_factor - cx) / f * depth;
            y = (i * resize_factor - cy) / f * depth;
            if (y > 1.5 && depth < 10.0 && img0_sobel.at<uchar>(i, j)>30)
            //if (img0_sobel.at<uchar>(i, j)>30)
                pl.push_back(Triple(x, y, depth));
        }
    disparity.convertTo(disparity, CV_8UC1, 0.3);
    disparity = img0_sobel + img0_rectify_new;


    return pl;
}

void hwa_vis::vis_stereo_lk_gpu::resetIMU() {
    mean_ang_vel = cv::Vec3f(0, 0, 0);
    ts_p_c = 0;
}

void hwa_vis::vis_stereo_lk_gpu::calculateIPM(cv::Mat & ipm)
{
    Triple att = Triple(91.5 / 180 * 3.1415926535, 0, 0); // between the reference IPM frame and the "average" camera frame
    double d = 1.8;
    int IPM_height = 1000;
    int IPM_width = 500;

    SO3 R_c1c0;
    double sp = sin(att(0)), cp = cos(att(0));
    double sr = sin(att(1)), cr = cos(att(1));
    double sy = sin(att(2)), cy = cos(att(2));
    R_c1c0 << cy * cr - sy * sp*sr, -sy * cp, cy*sr + sy * sp*cr,
        sy*cr + cy * sp*sr, cy*cp, sy*sr - cy * sp*cr,
        -cp * sr, sp, cp*cr;

    if (_pose_history.size() == _pose_history_max_size)
    {
        Triple att_ave(0, 0, 0);
        Triple att_diff;
        SO3 R_b_n;
        Triple att_this;
        for (int i = 0; i < _pose_history.size(); i++)
        {
            R_b_n = _pose_history[i].first * R_cam0_imu.transpose();
            att_this(0) = asin(R_b_n(2, 1));
            att_this(1) = atan2(-R_b_n(2, 0), R_b_n(2, 2));
            att_this(2) = atan2(-R_b_n(0, 1), R_b_n(1, 1));
            att_ave += att_this;
            if (i == _pose_history.size() - 1)
            {
                att_ave /= _pose_history.size();
                att_diff = att_this - att_ave;
            }
        }
        SO3 R_ilatest_iave;
        double sp = sin(att_diff(0)), cp = cos(att_diff(0));
        double sr = sin(att_diff(1)), cr = cos(att_diff(1));
        double sy = sin(0), cy = cos(0);
        R_ilatest_iave << cy * cr - sy * sp*sr, -sy * cp, cy*sr + sy * sp*cr,
            sy*cr + cy * sp*sr, cy*cp, sy*sr - cy * sp*cr,
            -cp * sr, sp, cp*cr;
        Matrix R_clatest_cave = R_cam0_imu.transpose()*R_ilatest_iave*R_cam0_imu;
        R_c1c0 = R_c1c0 * R_clatest_cave;
    }

    cv::Matx33d K0(_cam0_intrinsics(0), 0, _cam0_intrinsics(2),
        0, _cam0_intrinsics(1), _cam0_intrinsics(3),
        0, 0, 1);
    initUndistortRectifyMap(K0, _cam0_distortion_coeffs, cv::Matx33d::eye(), K0, d_curr0.size(), CV_32FC1, _IPM_M0_1, _IPM_M0_2);
    SO3 K0_e;
    cv2eigen(K0, K0_e);

    _IPM_mapx = cv::Mat(IPM_height, IPM_width, CV_32FC1);
    _IPM_mapy = cv::Mat(IPM_height, IPM_width, CV_32FC1);

    Triple c1_p_c1f;
    Triple xy1_y;
    Triple xy1;
    Triple uv1;
    for (int i = 0; i < IPM_height; i++)
    {
        for (int j = 0; j < IPM_width; j++)
        {
            c1_p_c1f(0) = (j - IPM_width / 2)*0.02;
            c1_p_c1f(1) = -5 - i * 0.02;
            c1_p_c1f(2) = d;
            xy1_y = R_c1c0.transpose()*c1_p_c1f / d;
            xy1 = xy1_y / xy1_y(2);
            uv1 = K0_e * xy1;
            _IPM_mapx.at<float>(IPM_height - i - 1, j) = uv1(0);
            _IPM_mapy.at<float>(IPM_height - i - 1, j) = uv1(1);
        }
    }

    ipm = cv::Mat(IPM_height, IPM_width, CV_8UC1);
    cv::Mat ipm_temp;
    cv::remap(d_curr0, ipm_temp, _IPM_M0_1, _IPM_M0_2, cv::INTER_LINEAR);
    cv::remap(ipm_temp, ipm, _IPM_mapx, _IPM_mapy, cv::INTER_LINEAR);
}