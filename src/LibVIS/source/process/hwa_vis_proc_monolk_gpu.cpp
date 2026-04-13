#include "hwa_vis_proc_monolk.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"

hwa_vis::vis_mono_lk_gpu::vis_mono_lk_gpu(hwa_set::set_base* _set, int cam_group_id) :vis_imgproc<cv::cuda::GpuMat>(_set, cam_group_id),
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
    gftt_fast = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, 200, 0.01, 0, 2, true, 0.04);
}

bool hwa_vis::vis_mono_lk_gpu::Initialize()
{
    eigen2cv(R_cam0_imu, _R_cam0_imu);
    eigen2cv(t_cam0_imu, _t_cam0_imu);
    _cam0_distortion_model = distortion2str(cam0_distortion_model);
    _cam0_intrinsics << cam0_intrinsics(0), cam0_intrinsics(1), cam0_intrinsics(2), cam0_intrinsics(3);
    _cam0_distortion_coeffs << cam0_distortion_coeffs(0), cam0_distortion_coeffs(1), cam0_distortion_coeffs(2), cam0_distortion_coeffs(3);
    return true;
}

bool hwa_vis::vis_mono_lk_gpu::TrackPointsPyrLK_CUDA(
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

hwa_vis::PointCloud hwa_vis::vis_mono_lk_gpu::ProcessBatch()
{
    base_scopedtimer timer("ProcessBatch()", TimeCostDebugOutFile, TimeCostOut);
    width = imageSingle.second.cols;
    height = imageSingle.second.rows;
    grid_height = height / grid_row;
    grid_width = width / grid_col;
    curr_img_t = imageSingle.first;
    d_curr0 = imageSingle.second;
    out_img = std::make_shared<cv::cuda::GpuMat>(height, width, CV_8UC3);
    cv::cuda::cvtColor(d_curr0, out_img->colRange(0, width), cv::COLOR_GRAY2BGR);

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
    prev_img_t = curr_img_t;
    if (d_currPts0.empty()) {
        d_prevPts0.release();
        return _pointCloud;
    }
    d_prevPts0 = d_currPts0.clone();
    return _pointCloud;
}

bool hwa_vis::vis_mono_lk_gpu::initializeFirstFrame()
{
    gftt_init->detect(d_curr0, d_currPts0, cv::noArray(), stream);
    stream.waitForCompletion();
    Init();
    setEpipolar(false);
    setflags();
    filterPointsGPU();
    InitializeGrid();
    return true;
}

bool hwa_vis::vis_mono_lk_gpu::monotrack() {
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

void hwa_vis::vis_mono_lk_gpu::trackFeatures()
{
    base_scopedtimer timer("trackFeatures()", TimeCostDebugOutFile, TimeCostOut);
    before_tracking = d_prevPts0.cols;
    if (before_tracking == 0) return;

    if (!monotrack()) return;
    setflags();
    filterPointsGPU();
    after_tracking = d_currPts0.cols;

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

    std::cout << "candidates: " << before_tracking
        << "; after_track: " << static_cast<double>(after_tracking) / before_tracking
        << "; after_ransac0: " << static_cast<double>(after_ransac0) / before_tracking << std::endl;
    return;
}

void hwa_vis::vis_mono_lk_gpu::drawFeaturesStereoGpu()
{
    int N = d_currPts0.cols;
    if (N == 0) return;
    base_scopedtimer timer("drawFeaturesStereoGpu()", TimeCostDebugOutFile, TimeCostOut);
    DrawPts(out_img);
    return;
}

void hwa_vis::vis_mono_lk_gpu::publish()
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
    thrust::host_vector<int> _ids(ids.begin(), ids.end());
    thrust::host_vector<int> _lifetime(lifetime.begin(), lifetime.end());

    for (int i = 0; i < d_currPts0.cols; ++i)
    {
        feature_msg.id = _ids[i];
        feature_msg.lifetime = _lifetime[i];
        feature_msg.cam0_point = curr_cam0_points_undistorted[i];
        _pointCloud.features.push_back(feature_msg);
    }
    return;
}

void hwa_vis::vis_mono_lk_gpu::integrateImuData(
    cv::Matx33f& cam0_R_p_c, cv::Matx33f& cam1_R_p_c)
{
    cv::Vec3f cam0_mean_ang_vel = _R_cam0_imu.t() * mean_ang_vel;
    double dtime = curr_img_t - prev_img_t;
    Rodrigues(cam0_mean_ang_vel * dtime, cam0_R_p_c);
    cam0_R_p_c = cam0_R_p_c.t();
    return;
}

void hwa_vis::vis_mono_lk_gpu::predictFeatureTracking(
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

void hwa_vis::vis_mono_lk_gpu::twoPointRansac(
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
            Vector solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
            model_better(0) = 1.0;
            model_better(1) = solution(0);
            model_better(2) = solution(1);
        }
        else if (base_indicator == 1) {
            Matrix A(inlier_set.size(), 2);
            A << coeff_tx_better, coeff_tz_better;
            Vector solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
            model_better(0) = solution(0);
            model_better(1) = 1.0;
            model_better(2) = solution(1);
        }
        else {
            Matrix A(inlier_set.size(), 2);
            A << coeff_tx_better, coeff_ty_better;
            Vector solution =
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

void hwa_vis::vis_mono_lk_gpu::addNewFeatures()
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
    InitNewPts();
    setnewptsflags();
    filterNewPointsGPU();
    InitializeNewPtsGrid();
    AddNewPts();
    return;
}

cv::cuda::GpuMat hwa_vis::vis_mono_lk_gpu::distortPoints(
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

void hwa_vis::vis_mono_lk_gpu::undistortPoints(
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


void hwa_vis::vis_mono_lk_gpu::undistortPoints(
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


bool hwa_vis::vis_mono_lk_gpu::featureCompareByResponse(
    const FeaturePoint& f1,
    const FeaturePoint& f2)
{

    return f1.response > f2.response;
}

template <typename Tem>
void hwa_vis::vis_mono_lk_gpu::removeUnmarkedElements(
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

void hwa_vis::vis_mono_lk_gpu::rescalePoints(
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

bool hwa_vis::vis_mono_lk_gpu::keyPointCompareByResponse(
    const cv::KeyPoint& pt1,
    const cv::KeyPoint& pt2)
{
    return pt1.response > pt2.response;
}

bool hwa_vis::vis_mono_lk_gpu::featureCompareByLifetime(
    const FeaturePoint& f1,
    const FeaturePoint& f2)
{
    return f1.lifetime > f2.lifetime;
}

void hwa_vis::vis_mono_lk_gpu::resetIMU() {
    mean_ang_vel = cv::Vec3f(0, 0, 0);
    ts_p_c = 0;
}