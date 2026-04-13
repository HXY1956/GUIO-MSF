#include "hwa_vis_proc_stereolk.h"
#include "hwa_set_vis.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"

using namespace hwa_base;
using namespace hwa_vis;
using namespace hwa_set;
using namespace std;
using namespace cv;

vis_stereo_lk_cpu::vis_stereo_lk_cpu(set_base* _set, int cam_group_id) :vis_imgproc<cv::Mat>(_set, cam_group_id),
prev_features_ptr(new GridFeatures()),
curr_features_ptr(new GridFeatures()),
isFirstImg(true),
IsInitialized(false)
{
    next_feature_id = 0;
    out_img_dense = make_shared<cv::Mat>(cv::Mat(100, 100, CV_8UC1));
    out_img_IPM = make_shared<cv::Mat>(cv::Mat(100, 100, CV_8UC1));
}

PointCloud vis_stereo_lk_cpu::ProcessBatch(const double& t)
{
    cv::Mat _img0 = cv::imread(cur_img_path.img0_path, 0);
    cv::Mat _img1 = cv::imread(cur_img_path.img1_path, 0);
    ONE_FRAME _img_msg;
    _img_msg.t = cur_img_path.t;
    _img_msg.img0 = make_shared<cv::Mat>(_img0);

    _img_msg.img1 = make_shared<cv::Mat>(_img1);
    cam0_curr_img = *(_img_msg.img0);
    cam1_curr_img = *(_img_msg.img1);

    curr_img_msg = _img_msg;

    if (!IsInitialized)
    {
        Initialize();
        IsInitialized = true;
    }

    eigen2cv(R_cam0_cam1, _R_cam0_cam1);
    eigen2cv(t_cam0_cam1, _t_cam0_cam1);
    eigen2cv(R_cam0_imu, _R_cam0_imu);
    eigen2cv(t_cam0_imu, _t_cam0_imu);
    _R_cam1_imu = _R_cam0_imu * _R_cam0_cam1.t();

    createImagePyramids();

    if (isFirstImg)
    {
        if (_vecimu.size() != 0) {
            initializeFirstFrame();
            std::cout << "detect first image" << std::endl;
            isFirstImg = false;

            IMU_MSG last_imu = _vecimu.at(_vecimu.size() - 1);
            _vecimu.clear();
            _vecimu.push_back(last_imu);
        }
    }
    else
    {
        trackFeatures();
        addNewFeatures();
        pruneGridFeatures();
    }

    drawFeaturesStereo();
    publish();
    prev_img_msg = curr_img_msg;
    prev_features_ptr = curr_features_ptr;
    std::swap(prev_cam0_pyramid, curr_cam0_pyramid);

    curr_features_ptr.reset(new GridFeatures());
    for (int code = 0; code < grid_row * grid_col; ++code)
    {
        (*curr_features_ptr)[code] = vector<FeaturePoint>(0);
    }

    return _pointCloud;
}

bool vis_stereo_lk_cpu::Initialize()
{
    // Create feature detector（fast）
    detector_ptr = cv::FastFeatureDetector::create(fast_threshold);

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

void vis_stereo_lk_cpu::createImagePyramids()
{
    // 为摄像头 0 图像创建光流金字塔，使用指定的补丁大小、金字塔层级和边界选项。
    // 这有助于在多尺度下进行光流计算。
    buildOpticalFlowPyramid(
        cam0_curr_img,               // 当前摄像头 0 的图像
        curr_cam0_pyramid,           // 输出的摄像头 0 图像金字塔
        cv::Size(patch_size, patch_size),  // 金字塔中每个补丁的大小
        pyramid_levels,              // 金字塔的层级数
        true,                        // 是否使用高斯金字塔
        cv::BORDER_REFLECT_101,      // 图像的边界处理方式（reflect_101 通常用于边界反射）
        cv::BORDER_CONSTANT,         // 图像边界外部的常量值
        false                        // 是否使用初始化标志
    );

    if (cam1_curr_img.empty()) {
        std::cerr << "Error: cam1 image is empty!" << std::endl;
        return;
    }

    // 为摄像头 1 图像创建光流金字塔，设置与摄像头 0 相同
    buildOpticalFlowPyramid(
        cam1_curr_img,               // 当前摄像头 1 的图像
        curr_cam1_pyramid,           // 输出的摄像头 1 图像金字塔
        cv::Size(patch_size, patch_size),  // 金字塔中每个补丁的大小
        pyramid_levels,              // 金字塔的层级数
        true,                        // 是否使用高斯金字塔
        cv::BORDER_REFLECT_101,      // 图像的边界处理方式
        cv::BORDER_CONSTANT,         // 图像边界外部的常量值
        false                        // 是否使用初始化标志
    );
}


void vis_stereo_lk_cpu::initializeFirstFrame()
{
    // 获取当前摄像头 0 的图像
    const Mat& img = cam0_curr_img;

    // 计算每个网格的高度和宽度
    static int grid_height = img.rows / grid_row;
    static int grid_width = img.cols / grid_col;

    // 用于存储新特征点
    vector<KeyPoint> new_features(0);
    // 使用检测器提取当前图像中的特征点
    detector_ptr->detect(img, new_features);

    // 将检测到的特征点的坐标转换为 Point2f 格式
    vector<cv::Point2f> cam0_points(new_features.size());
    for (int i = 0; i < new_features.size(); ++i)
        cam0_points[i] = new_features[i].pt;

    // 用于存储摄像头 1 上的特征点
    vector<cv::Point2f> cam1_points(0);
    // 用于标记哪些特征点是内点
    vector<unsigned char> inlier_markers(0);
    // 进行立体匹配，得到摄像头 1 上的对应点和内点标记
    stereoMatch(cam0_points, cam1_points, inlier_markers);

    // 存储内点特征点
    vector<cv::Point2f> cam0_inliers(0);
    vector<cv::Point2f> cam1_inliers(0);
    vector<float> response_inliers(0);

    // 遍历内点标记，筛选出有效的特征点
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] == 0)
            continue;
        cam0_inliers.push_back(cam0_points[i]);
        cam1_inliers.push_back(cam1_points[i]);
        response_inliers.push_back(new_features[i].response);
    }

    // 创建网格特征字典，用于存储每个网格的特征点
    GridFeatures grid_new_features;
    for (int code = 0; code < grid_row * grid_col; ++code)
        grid_new_features[code] = vector<FeaturePoint>(0);

    // 将每个特征点根据所在网格位置分配到对应网格
    for (int i = 0; i < cam0_inliers.size(); ++i)
    {
        const cv::Point2f& cam0_point = cam0_inliers[i];
        const cv::Point2f& cam1_point = cam1_inliers[i];
        const float& response = response_inliers[i];

        // 根据特征点的位置计算它所在的网格编号
        int row = static_cast<int>(cam0_point.y / grid_height);
        int col = static_cast<int>(cam0_point.x / grid_width);
        int code = row * grid_col + col;

        // 创建新的特征点并将其加入到对应的网格
        FeaturePoint new_feature;
        new_feature.response = response;
        new_feature.cam0_point = cam0_point;
        new_feature.cam1_point = cam1_point;
        grid_new_features[code].push_back(new_feature);
    }

    // 对每个网格内的特征点按响应值进行排序
    for (auto& item : grid_new_features)
        std::sort(item.second.begin(), item.second.end(),
            &vis_stereo_lk_cpu::featureCompareByResponse);

    // 将排序后的特征点添加到当前特征点集合中
    for (int code = 0; code < grid_row * grid_col; ++code)
    {
        // 获取当前网格的特征点
        vector<FeaturePoint>& features_this_grid = (*curr_features_ptr)[code];
        // 获取新检测到的特征点
        vector<FeaturePoint>& new_features_this_grid = grid_new_features[code];

        // 将新特征点添加到当前网格特征点列表中，最多添加 grid_min_feature_num 个特征点
        for (int k = 0; k < grid_min_feature_num && k < new_features_this_grid.size(); ++k)
        {
            features_this_grid.push_back(new_features_this_grid[k]);
            // 分配特征点 ID 和生命周期
            features_this_grid.back().id = next_feature_id++;
            features_this_grid.back().lifetime = 1;
        }
    }

    return;
}


void vis_stereo_lk_cpu::stereoMatch(const vector<cv::Point2f>& cam0_points,
    vector<cv::Point2f>& cam1_points,
    vector<unsigned char>& inlier_markers)
{
    // 如果摄像头 0 的特征点为空，直接返回
    if (cam0_points.size() == 0)
        return;

    // 如果摄像头 1 的特征点为空，则需要根据摄像头 0 的特征点来计算摄像头 1 的特征点
    if (cam1_points.size() == 0)
    {
        // 存储去畸变后的摄像头 0 特征点
        vector<cv::Point2f> cam0_points_undistorted;
        // 获取摄像头 0 畸变模型字符串
        string _cam0_distortion_model = distortion2str(cam0_distortion_model);

        // 去畸变处理：将摄像头 0 的特征点去畸变，得到去畸变后的特征点
        undistortPoints(cam0_points, _cam0_intrinsics, _cam0_distortion_model,
            _cam0_distortion_coeffs, cam0_points_undistorted,
            _R_cam0_cam1);

        // 对去畸变后的特征点进行畸变处理，得到摄像头 1 的对应点
        cam1_points = distortPoints(cam0_points_undistorted, _cam1_intrinsics,
            _cam1_distortion_model, _cam1_distortion_coeffs);
    }

    // 使用金字塔光流算法计算光流，得到摄像头 1 上的特征点位置
    calcOpticalFlowPyrLK(curr_cam0_pyramid, curr_cam1_pyramid,
        cam0_points, cam1_points,
        inlier_markers, noArray(),
        Size(patch_size, patch_size),
        pyramid_levels,
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
            max_iteration, track_precision),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    // 对光流计算出的摄像头 1 特征点位置进行边界检查，如果超出图像边界，标记为内点
    for (int i = 0; i < cam1_points.size(); ++i)
    {
        if (inlier_markers[i] == 0)
            continue;
        if (cam1_points[i].y < 0 ||
            cam1_points[i].y > cam1_curr_img.rows - 1 ||
            cam1_points[i].x < 0 ||
            cam1_points[i].x > cam1_curr_img.cols - 1)
        {
            inlier_markers[i] = 0;
        }
    }

    // 构造摄像头 0 到摄像头 1 的基础矩阵 E，用于之后的极线约束
    const cv::Matx33d t_cam0_cam1_hat(
        0.0, -t_cam0_cam1[2], t_cam0_cam1[1],
        t_cam0_cam1[2], 0.0, -t_cam0_cam1[0],
        -t_cam0_cam1[1], t_cam0_cam1[0], 0.0);
    const cv::Matx33d E = t_cam0_cam1_hat * _R_cam0_cam1;

    // 存储去畸变后的摄像头 0 和摄像头 1 特征点
    vector<cv::Point2f> cam0_points_undistorted(0);
    vector<cv::Point2f> cam1_points_undistorted(0);

    // 对摄像头 0 和摄像头 1 的特征点进行去畸变处理
    undistortPoints(
        cam0_points, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, cam0_points_undistorted);
    undistortPoints(
        cam1_points, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, cam1_points_undistorted);    // 根据 msckf_vio 调整

    // 计算像素单位的标准化因子，用于后续的极线误差阈值
    double norm_pixel_unit = 4.0 / (
        _cam0_intrinsics[0] + _cam0_intrinsics[1] +
        _cam1_intrinsics[0] + _cam1_intrinsics[1]);

    // 对每个特征点，计算其极线误差并检查是否满足阈值
    for (int i = 0; i < cam0_points_undistorted.size(); ++i)
    {
        if (inlier_markers[i] == 0)
            continue;

        // 将去畸变后的特征点转换为齐次坐标
        cv::Vec3d pt0(cam0_points_undistorted[i].x,
            cam0_points_undistorted[i].y, 1.0);
        cv::Vec3d pt1(cam1_points_undistorted[i].x,
            cam1_points_undistorted[i].y, 1.0);

        // 计算极线：E * pt0
        cv::Vec3d epipolar_line = E * pt0;

        // 计算极线误差
        double error = fabs((pt1.t() * epipolar_line)[0]) / sqrt(
            epipolar_line[0] * epipolar_line[0] +
            epipolar_line[1] * epipolar_line[1]);

        // 如果误差超过阈值，标记为非内点
        if (error > stereo_threshold * norm_pixel_unit)
            inlier_markers[i] = 0;
    }

    return;
}


void vis_stereo_lk_cpu::drawFeaturesStereo()
{

    Scalar tracked(0, 255, 0);
    Scalar new_feature(0, 255, 255);


    static int grid_height =
        cam0_curr_img.rows / grid_row;
    static int grid_width =
        cam0_curr_img.cols / grid_col;


    int img_height = cam0_curr_img.rows;
    int img_width = cam0_curr_img.cols;
    out_img = make_shared<cv::Mat>(img_height, img_width * 2, CV_8UC3);

    cv::cvtColor(cam0_curr_img, out_img->colRange(0, img_width), cv::COLOR_GRAY2BGR);
    cv::cvtColor(cam1_curr_img, out_img->colRange(img_width, img_width * 2), cv::COLOR_GRAY2BGR);



    for (int i = 1; i < grid_row; ++i)
    {
        Point pt1(0, i * grid_height);
        Point pt2(img_width * 2, i * grid_height);
        line(*out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < grid_col; ++i)
    {
        Point pt1(i * grid_width, 0);
        Point pt2(i * grid_width, img_height);
        line(*out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < grid_col; ++i)
    {
        Point pt1(i * grid_width + img_width, 0);
        Point pt2(i * grid_width + img_width, img_height);
        line(*out_img, pt1, pt2, Scalar(255, 0, 0));
    }





    vector<long long int> prev_ids(0);
    for (const auto& grid_features : *prev_features_ptr)
        for (const auto& feature : grid_features.second)
            prev_ids.push_back(feature.id);



    map<long long int, Point2f> prev_cam0_points;
    map<long long int, Point2f> prev_cam1_points;
    for (const auto& grid_features : *prev_features_ptr)
        for (const auto& feature : grid_features.second)
        {
            prev_cam0_points[feature.id] = feature.cam0_point;
            prev_cam1_points[feature.id] = feature.cam1_point;
        }


    map<long long int, Point2f> curr_cam0_points;
    map<long long int, Point2f> curr_cam1_points;
    for (const auto& grid_features : *curr_features_ptr)
        for (const auto& feature : grid_features.second)
        {
            curr_cam0_points[feature.id] = feature.cam0_point;
            curr_cam1_points[feature.id] = feature.cam1_point;
        }



    for (const auto& id : prev_ids)
    {

        if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
            curr_cam0_points.find(id) != curr_cam0_points.end())
        {
            cv::Point2f prev_pt0 = prev_cam0_points[id];
            cv::Point2f prev_pt1 = prev_cam1_points[id] + Point2f(img_width, 0.0);
            cv::Point2f curr_pt0 = curr_cam0_points[id];
            cv::Point2f curr_pt1 = curr_cam1_points[id] + Point2f(img_width, 0.0);

            circle(*out_img, curr_pt0, 3, tracked, -1);
            circle(*out_img, curr_pt1, 3, tracked, -1);
            line(*out_img, prev_pt0, curr_pt0, tracked, 1);
            line(*out_img, prev_pt1, curr_pt1, tracked, 1);

            prev_cam0_points.erase(id);
            prev_cam1_points.erase(id);
            curr_cam0_points.erase(id);
            curr_cam1_points.erase(id);
        }
    }


    for (const auto& new_cam0_point : curr_cam0_points)
    {
        cv::Point2f pt0 = new_cam0_point.second;
        cv::Point2f pt1 = curr_cam1_points[new_cam0_point.first] +
            Point2f(img_width, 0.0);

        circle(*out_img, pt0, 3, new_feature, -1);
        circle(*out_img, pt1, 3, new_feature, -1);
    }

    cv::resize(*out_img, *out_img, out_img->size());
    //imwrite("stereo.jpg", out_img);
    cv::resize(*out_img, *out_img, cv::Size(out_img->cols / 2.0, out_img->rows / 2.0));
    //imshow(string("Feature: cam_group_")+to_string(_cam_group_id), *out_img);
    //waitKey(1);
    return;
}

void vis_stereo_lk_cpu::publish()
{
    //sensor_msgs::FeaturePoint feature_msg;
    _pointCloud.features.clear();
    FeaturePoint feature_msg;
    //pointCloud = sensor_msgs::PointCloud();
    _pointCloud.time = curr_img_msg.t;

    vector<long long int> curr_ids(0);
    vector<Point2f> curr_cam0_points(0);
    vector<Point2f> curr_cam1_points(0);


    for (const auto& grid_features : (*curr_features_ptr))
    {
        for (const auto& feature : grid_features.second)
        {
            curr_ids.push_back(feature.id);
            curr_cam0_points.push_back(feature.cam0_point);
            curr_cam1_points.push_back(feature.cam1_point);
        }
    }
    vector<Point2f> curr_cam0_points_undistorted(0);
    vector<Point2f> curr_cam1_points_undistorted(0);

    undistortPoints(
        curr_cam0_points, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, curr_cam0_points_undistorted);
    undistortPoints(
        curr_cam1_points, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, curr_cam1_points_undistorted);


    for (int i = 0; i < curr_ids.size(); ++i)
    {
        //cout << "imgproc_id" << feature_msg.id << endl;
        feature_msg.id = curr_ids[i];
        feature_msg.cam0_point = curr_cam0_points_undistorted[i];

        feature_msg.cam1_point = curr_cam1_points_undistorted[i];

        _pointCloud.features.push_back(feature_msg);
    }
    return;
}

void vis_stereo_lk_cpu::trackFeatures()
{

    // 计算每个网格单元的高度和宽度
    static int grid_height = cam0_curr_img.rows / grid_row;
    static int grid_width = cam0_curr_img.cols / grid_col;

    // 用于存储计算的相机旋转矩阵
    Matx33f cam0_R_p_c;
    Matx33f cam1_R_p_c;

    // 通过集成IMU数据计算当前相机的旋转矩阵
    integrateImuData(cam0_R_p_c, cam1_R_p_c);

    // 存储前一帧的特征点数据
    vector<long long int> prev_ids;               // 特征点ID
    vector<int> prev_lifetime;                    // 特征点生命周期
    vector<Point2f> prev_cam0_points;             // 前一帧在相机0中的特征点
    vector<Point2f> prev_cam1_points;             // 前一帧在相机1中的特征点

    // 从 `prev_features_ptr` 中提取前一帧的特征点信息
    for (const auto& item : *prev_features_ptr)
    {
        for (const auto& prev_feature : item.second)
        {
            prev_ids.push_back(prev_feature.id);
            prev_lifetime.push_back(prev_feature.lifetime);
            prev_cam0_points.push_back(prev_feature.cam0_point);
            prev_cam1_points.push_back(prev_feature.cam1_point);
        }
    }

    // 记录前一帧特征点的数量
    before_tracking = prev_cam0_points.size();

    // 如果没有特征点，则直接返回
    if (prev_ids.size() == 0)
        return;

    // 用于存储当前帧的特征点
    vector<Point2f> curr_cam0_points;
    vector<unsigned char> track_inliers;  // 用于记录光流追踪的内点

    // 预测当前帧相机0的特征点位置
    predictFeatureTracking(prev_cam0_points, cam0_R_p_c, _cam0_intrinsics, curr_cam0_points);

    // 使用金字塔光流法计算当前帧相机0的特征点
    calcOpticalFlowPyrLK(
        prev_cam0_pyramid, curr_cam0_pyramid,  // 当前和前一帧的图像金字塔
        prev_cam0_points, curr_cam0_points,    // 前一帧和当前帧的特征点
        track_inliers, noArray(),              // 内点标记
        Size(patch_size, patch_size),          // 金字塔块的大小
        pyramid_levels,                       // 金字塔层数
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, max_iteration, track_precision), // 终止条件
        cv::OPTFLOW_USE_INITIAL_FLOW         // 使用初始估计来提高追踪精度
    );

    // 排除图像外的特征点
    for (int i = 0; i < curr_cam0_points.size(); ++i) {
        if (track_inliers[i] == 0) continue;
        if (curr_cam0_points[i].y < 0 || curr_cam0_points[i].y > cam0_curr_img.rows - 1 ||
            curr_cam0_points[i].x < 0 || curr_cam0_points[i].x > cam0_curr_img.cols - 1)
            track_inliers[i] = 0;
    }

    // 存储当前帧的追踪特征点
    vector<long long int> prev_tracked_ids;
    vector<int> prev_tracked_lifetime;
    vector<Point2f> prev_tracked_cam0_points;
    vector<Point2f> prev_tracked_cam1_points;
    vector<Point2f> curr_tracked_cam0_points;

    // 通过内点标记从之前的特征点中移除未追踪到的特征点
    removeUnmarkedElements(prev_ids, track_inliers, prev_tracked_ids);
    removeUnmarkedElements(prev_lifetime, track_inliers, prev_tracked_lifetime);
    removeUnmarkedElements(prev_cam0_points, track_inliers, prev_tracked_cam0_points);
    removeUnmarkedElements(prev_cam1_points, track_inliers, prev_tracked_cam1_points);
    removeUnmarkedElements(curr_cam0_points, track_inliers, curr_tracked_cam0_points);

    // 记录当前帧的追踪特征点数量
    after_tracking = curr_tracked_cam0_points.size();



    // Outlier removal involves three steps, which forms a close
    // loop between the previous and current frames of cam0 (left)
    // and cam1 (right). Assuming the stereo matching between the
    // previous cam0 and cam1 images are correct, the three steps are:
    //
    // prev frames cam0 ----------> cam1
    //              |                |
    //              |ransac          |ransac
    //              |   stereo match |
    // curr frames cam0 ----------> cam1
    //
    // 1) Stereo matching between current images of cam0 and cam1.
    // 2) RANSAC between previous and current images of cam0.
    // 3) RANSAC between previous and current images of cam1.
    //
    // For Step 3, tracking between the images is no longer needed.
    // The stereo matching results are directly used in the RANSAC.


    // 立体匹配：将当前帧相机0的特征点与相机1进行匹配
    vector<Point2f> curr_cam1_points;
    vector<unsigned char> match_inliers;
    stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);

    // 存储匹配后的特征点数据
    vector<long long int> prev_matched_ids;
    vector<int> prev_matched_lifetime;
    vector<Point2f> prev_matched_cam0_points;
    vector<Point2f> prev_matched_cam1_points;
    vector<Point2f> curr_matched_cam0_points;
    vector<Point2f> curr_matched_cam1_points;

    // 通过内点标记移除未匹配的特征点
    removeUnmarkedElements(prev_tracked_ids, match_inliers, prev_matched_ids);
    removeUnmarkedElements(prev_tracked_lifetime, match_inliers, prev_matched_lifetime);
    removeUnmarkedElements(prev_tracked_cam0_points, match_inliers, prev_matched_cam0_points);
    removeUnmarkedElements(prev_tracked_cam1_points, match_inliers, prev_matched_cam1_points);
    removeUnmarkedElements(curr_tracked_cam0_points, match_inliers, curr_matched_cam0_points);
    removeUnmarkedElements(curr_cam1_points, match_inliers, curr_matched_cam1_points);

    // 记录匹配后的特征点数量
    after_matching = curr_matched_cam0_points.size();

    // 对匹配的特征点进行 RANSAC 去除外点
    vector<int> cam0_ransac_inliers(0);
    twoPointRansac(
        prev_matched_cam0_points, curr_matched_cam0_points,
        cam0_R_p_c, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, ransac_threshold,
        0.99, cam0_ransac_inliers
    );

    vector<int> cam1_ransac_inliers(0);
    twoPointRansac(
        prev_matched_cam1_points, curr_matched_cam1_points,
        cam1_R_p_c, _cam1_intrinsics, _cam1_distortion_model,
        _cam1_distortion_coeffs, ransac_threshold,
        0.99, cam1_ransac_inliers
    );

    // 记录经过 RANSAC 筛选后的特征点数量
    after_ransac = 0;
    for (int i = 0; i < cam0_ransac_inliers.size(); ++i)
    {
        if (cam0_ransac_inliers[i] == 0 || cam1_ransac_inliers[i] == 0)
            continue;

        // 计算当前特征点所属的网格单元
        int row = static_cast<int>(curr_matched_cam0_points[i].y / grid_height);
        int col = static_cast<int>(curr_matched_cam0_points[i].x / grid_width);

        // 计算网格的编码
        int code = row * grid_col + col;

        // 将特征点加入到网格中
        (*curr_features_ptr)[code].push_back(FeaturePoint());
        FeaturePoint& grid_new_feature = (*curr_features_ptr)[code].back();

        // 更新新特征点的属性
        grid_new_feature.id = prev_matched_ids[i];
        grid_new_feature.lifetime = ++prev_matched_lifetime[i];
        grid_new_feature.cam0_point = curr_matched_cam0_points[i];
        grid_new_feature.cam1_point = curr_matched_cam1_points[i];

        ++after_ransac;
    }

    // 计算前后两帧特征点的数量
    int prev_feature_num = 0;
    for (const auto& item : *prev_features_ptr)
        prev_feature_num += item.second.size();

    int curr_feature_num = 0;
    for (const auto& item : *curr_features_ptr)
        curr_feature_num += item.second.size();

    // 输出相关统计信息（可以选择性输出）
    /*printf("candidates: %d; track: %d; match: %d; ransac: %d/%d=%f; after_ransac：%d \n",
        before_tracking, after_tracking, after_matching,
        curr_feature_num, prev_feature_num,
        static_cast<double>(curr_feature_num) /
        (static_cast<double>(prev_feature_num) + 1e-5), after_ransac);*/

    return;
}



void vis_stereo_lk_cpu::integrateImuData(
    cv::Matx33f& cam0_R_p_c, cv::Matx33f& cam1_R_p_c)
{

    auto begin_iter = _vecimu.begin();
    while (begin_iter != _vecimu.end())
    {
        if (begin_iter->t -
            prev_img_msg.t < -0.01)
            ++begin_iter;
        else
            break;
    }

    auto end_iter = begin_iter;
    while (end_iter != _vecimu.end())
    {
        /*if (end_iter->t -
            curr_img_msg.t < 0.005)*/
            //修改
        if (end_iter->t -
            curr_img_msg.t < 0.002)
            ++end_iter;
        else
            break;
    }
    //cout << "imu_size:" << _vecimu.size() << endl;
    //cout << "prev_img:" <<setprecision(10)<< prev_img_msg.t << endl;
    //cout << "cur_img:" <<setprecision(10)<< curr_img_msg.t << endl;
    //for (auto imu : _vecimu)
    //{
    ////    cout << "imu_t:" << imu.t << endl;
    //}
    //cout << "use_imu_size:" << end_iter - begin_iter << endl;

    Vec3f mean_ang_vel(0.0, 0.0, 0.0);
    for (auto iter = begin_iter; iter < end_iter; ++iter)

    {
        mean_ang_vel += Vec3f(iter->angular_velocity.x(),
            iter->angular_velocity.y(), iter->angular_velocity.z());

    }

    if (end_iter - begin_iter > 0)
        mean_ang_vel *= 1.0f / (end_iter - begin_iter);


    Vec3f cam0_mean_ang_vel = _R_cam0_imu.t() * mean_ang_vel;
    Vec3f cam1_mean_ang_vel = _R_cam1_imu.t() * mean_ang_vel;


    double dtime = (curr_img_msg.t -
        prev_img_msg.t);
    Rodrigues(cam0_mean_ang_vel * dtime, cam0_R_p_c);
    Rodrigues(cam1_mean_ang_vel * dtime, cam1_R_p_c);
    cam0_R_p_c = cam0_R_p_c.t();
    cam1_R_p_c = cam1_R_p_c.t();


    _vecimu.erase(_vecimu.begin(), end_iter);
    return;

}

void vis_stereo_lk_cpu::predictFeatureTracking(
    const vector<cv::Point2f>& input_pts, // 输入特征点
    const cv::Matx33f& R_p_c,             // 旋转矩阵，从上一帧(p)到当前帧(c)
    const cv::Vec4d& intrinsics,          // 相机内参 [fx, fy, cx, cy]
    vector<cv::Point2f>& compensated_pts) // 预测补偿后的特征点
{
    // 如果输入点为空，则清空输出并返回
    if (input_pts.size() == 0)
    {
        compensated_pts.clear();
        return;
    }

    // 预分配输出点的空间，与输入点数量一致
    compensated_pts.resize(input_pts.size());

    // 构造相机内参矩阵 K
    cv::Matx33f K(
        intrinsics[0], 0.0, intrinsics[2], // fx,  0, cx
        0.0, intrinsics[1], intrinsics[3], //  0, fy, cy
        0.0, 0.0, 1.0);                    //  0,  0,  1

    // 计算单应性变换矩阵 H = K * R_p_c * K^-1
    cv::Matx33f H = K * R_p_c * K.inv();

    // 遍历所有输入点，应用单应性变换
    for (int i = 0; i < input_pts.size(); ++i)
    {
        // 将 2D 点扩展为齐次坐标 (x, y, 1)
        cv::Vec3f p1(input_pts[i].x, input_pts[i].y, 1.0f);

        // 应用单应性变换 H
        cv::Vec3f p2 = H * p1;

        // 归一化，使其回到 2D 形式 (x/z, y/z)
        compensated_pts[i].x = p2[0] / p2[2];
        compensated_pts[i].y = p2[1] / p2[2];
    }

    return;
}

void vis_stereo_lk_cpu::twoPointRansac(
    const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2,
    const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
    const std::string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    vector<int>& inlier_markers)
{
    // 如果两个点集的大小不一致，打印警告信息
    if (pts1.size() != pts2.size())
        printf("Sets of different size (%d and %d) are used...\n",
            pts1.size(), pts2.size());

    // 计算像素单位的归一化系数
    double norm_pixel_unit = 2.0 / (intrinsics[0] + intrinsics[1]);

    // 根据成功概率计算 RANSAC 迭代次数
    int iter_num = static_cast<int>(
        ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));

    // 清空内点标记并初始化为全1（表示所有点都是内点）
    inlier_markers.clear();
    inlier_markers.resize(pts1.size(), 1);

    // 去畸变处理
    vector<Point2f> pts1_undistorted(pts1.size());
    vector<Point2f> pts2_undistorted(pts2.size());
    undistortPoints(
        pts1, intrinsics, distortion_model,
        distortion_coeffs, pts1_undistorted);
    undistortPoints(
        pts2, intrinsics, distortion_model,
        distortion_coeffs, pts2_undistorted);

    // 使用相机旋转矩阵将 pts1 转换到参考坐标系
    for (auto& pt : pts1_undistorted)
    {
        Vec3f pt_h(pt.x, pt.y, 1.0f);
        Vec3f pt_hc = R_p_c * pt_h;
        pt.x = pt_hc[0];
        pt.y = pt_hc[1];
    }

    // 计算归一化比例因子
    float scaling_factor = 0.0f;
    rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
    norm_pixel_unit *= scaling_factor;

    // 计算两组点之间的差异
    vector<Point2d> pts_diff(pts1_undistorted.size());
    for (int i = 0; i < pts1_undistorted.size(); ++i)
        pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

    // 计算点的均值距离，并确定初始的内点
    double mean_pt_distance = 0.0;
    int raw_inlier_cntr = 0;
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        double distance = sqrt(pts_diff[i].dot(pts_diff[i]));

        // 如果点之间的距离太大，则标记为外点
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

    // 如果内点数量少于 3，直接返回
    if (raw_inlier_cntr < 3)
    {
        for (auto& marker : inlier_markers)
            marker = 0;
        return;
    }

    // 如果均值距离小于归一化像素单位，进行进一步的外点去除
    if (mean_pt_distance < norm_pixel_unit)
    {
        for (int i = 0; i < pts_diff.size(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;

            // 如果点间距离大于容忍的误差，则标记为外点
            if (sqrt(pts_diff[i].dot(pts_diff[i])) > inlier_error * norm_pixel_unit)
                inlier_markers[i] = 0;
        }
        return;
    }

    // 构建系数矩阵，准备进行 RANSAC
    Matrix coeff_t(pts_diff.size(), 3);
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        coeff_t(i, 0) = pts_diff[i].y;
        coeff_t(i, 1) = -pts_diff[i].x;
        coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y - pts1_undistorted[i].y * pts2_undistorted[i].x;
    }

    // 提取内点的索引
    vector<int> raw_inlier_idx;
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] != 0)
            raw_inlier_idx.push_back(i);
    }

    // RANSAC 主循环
    vector<int> best_inlier_set;
    double best_error = 1e10;
    cv::RNG randomNumber;

    for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx)
    {
        // 随机选择两个内点对
        int pair_idx1 = raw_inlier_idx[randomNumber.uniform(0, raw_inlier_idx.size())];
        int idx_diff = randomNumber.uniform(1, raw_inlier_idx.size());
        int pair_idx2 = pair_idx1 + idx_diff < raw_inlier_idx.size() ? pair_idx1 + idx_diff : pair_idx1 + idx_diff - raw_inlier_idx.size();

        // 构建模型
        Eigen::Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
        Eigen::Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
        Eigen::Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
        vector<double> coeff_l1_norm(3);
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

        // 计算误差并找出符合模型的内点
        Vector error = coeff_t * model;
        vector<int> inlier_set;
        for (int i = 0; i < error.rows(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;
            if (std::abs(error(i)) < inlier_error * norm_pixel_unit)
                inlier_set.push_back(i);
        }

        // 如果内点数量太少，跳过当前模型
        if (inlier_set.size() < 0.2 * pts1_undistorted.size())
            continue;

        // 使用所有内点重新拟合模型
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

        // 计算新的误差并更新最佳模型
        Vector new_error = coeff_t * model_better;

        double this_error = 0.0;
        for (const auto& inlier_idx : inlier_set)
            this_error += std::abs(new_error(inlier_idx));
        this_error /= inlier_set.size();

        // 如果当前模型比之前的好，更新最佳模型
        if (inlier_set.size() > best_inlier_set.size())
        {
            best_error = this_error;
            best_inlier_set = inlier_set;
        }
    }

    // 更新最终的内点标记
    inlier_markers.clear();
    inlier_markers.resize(pts1.size(), 0);
    for (const auto& inlier_idx : best_inlier_set)
        inlier_markers[inlier_idx] = 1;

    return;
}


void vis_stereo_lk_cpu::addNewFeatures()
{
    // 获取当前相机图像
    const Mat& curr_img = cam0_curr_img;

    // 计算每个网格的高度和宽度（静态变量，初始化一次）
    static int grid_height = cam0_curr_img.rows / grid_row;
    static int grid_width = cam0_curr_img.cols / grid_col;

    // 创建掩码图像（用于特征点检测时屏蔽已有特征点区域）
    Mat mask(curr_img.rows, curr_img.cols, CV_8U, Scalar(1));

    // 遍历当前帧已有特征点，在 mask 上标记特征点附近区域为 0，避免重复检测
    for (const auto& features : *curr_features_ptr)
    {
        for (const auto& feature : features.second)
        {
            const int y = static_cast<int>(feature.cam0_point.y);
            const int x = static_cast<int>(feature.cam0_point.x);

            int up_lim = max(y - 2, 0);
            int bottom_lim = min(y + 3, curr_img.rows);
            int left_lim = max(x - 2, 0);
            int right_lim = min(x + 3, curr_img.cols);

            Range row_range(up_lim, bottom_lim);
            Range col_range(left_lim, right_lim);
            mask(row_range, col_range) = 0;
        }
    }

    // 通过特征点检测器（如 FAST, ORB）检测新特征点
    vector<KeyPoint> new_features;
    detector_ptr->detect(curr_img, new_features, mask);

    // 按照网格划分新检测的特征点，存入 new_feature_sieve
    vector<vector<KeyPoint>> new_feature_sieve(grid_row * grid_col);
    for (const auto& feature : new_features)
    {
        int row = min(static_cast<int>(feature.pt.y / grid_height), grid_row - 1);
        int col = min(static_cast<int>(feature.pt.x / grid_width), grid_col - 1);
        new_feature_sieve[row * grid_col + col].push_back(feature);
    }
    new_features.clear();

    // 对每个网格的特征点按响应值排序，并限制特征点数目
    for (auto& item : new_feature_sieve)
    {
        if (item.size() > grid_max_feature_num)
        {
            std::sort(item.begin(), item.end(), &vis_stereo_lk_cpu::keyPointCompareByResponse);
            item.erase(item.begin() + grid_max_feature_num, item.end());
        }
        new_features.insert(new_features.end(), item.begin(), item.end());
    }
    int detected_new_features = new_features.size();

    // 提取特征点坐标
    vector<cv::Point2f> cam0_points(new_features.size());
    for (int i = 0; i < new_features.size(); ++i)
        cam0_points[i] = new_features[i].pt;

    // 进行双目匹配，获取右目相应的特征点
    vector<cv::Point2f> cam1_points;
    vector<unsigned char> inlier_markers;
    stereoMatch(cam0_points, cam1_points, inlier_markers);

    // 仅保留双目匹配成功的特征点
    vector<cv::Point2f> cam0_inliers, cam1_inliers;
    vector<float> response_inliers;
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] == 0) continue;
        cam0_inliers.push_back(cam0_points[i]);
        cam1_inliers.push_back(cam1_points[i]);
        response_inliers.push_back(new_features[i].response);
    }

    // 统计成功匹配的特征点数目，并检测是否图像时间不同步
    int matched_new_features = cam0_inliers.size();
    if (matched_new_features < 5 &&
        static_cast<double>(matched_new_features) / static_cast<double>(detected_new_features) < 0.1)
    {
        printf("Images at [%f] seems unsynced...\n", curr_img_msg.t);
    }

    // 将新特征点存入网格结构
    GridFeatures grid_new_features;
    for (int code = 0; code < grid_row * grid_col; ++code)
        grid_new_features[code] = vector<FeaturePoint>();

    for (int i = 0; i < cam0_inliers.size(); ++i)
    {
        int row = static_cast<int>(cam0_inliers[i].y / grid_height);
        int col = static_cast<int>(cam0_inliers[i].x / grid_width);
        int code = row * grid_col + col;

        FeaturePoint new_feature;
        new_feature.response = response_inliers[i];
        new_feature.cam0_point = cam0_inliers[i];
        new_feature.cam1_point = cam1_inliers[i];
        grid_new_features[code].push_back(new_feature);
    }

    // 对新特征点按响应值排序
    for (auto& item : grid_new_features)
        std::sort(item.second.begin(), item.second.end(), &vis_stereo_lk_cpu::featureCompareByResponse);

    int new_added_feature_num = 0;

    // 向当前特征点列表中添加新特征点，保证每个网格至少有 grid_min_feature_num 个特征点
    for (int code = 0; code < grid_row * grid_col; ++code)
    {
        vector<FeaturePoint>& features_this_grid = (*curr_features_ptr)[code];
        vector<FeaturePoint>& new_features_this_grid = grid_new_features[code];

        if (features_this_grid.size() >= grid_min_feature_num)
            continue;

        int vacancy_num = grid_min_feature_num - features_this_grid.size();
        for (int k = 0; k < vacancy_num && k < new_features_this_grid.size(); ++k)
        {
            features_this_grid.push_back(new_features_this_grid[k]);
            features_this_grid.back().id = next_feature_id++;
            features_this_grid.back().lifetime = 1;
            ++new_added_feature_num;
        }
    }
    return;
}

void vis_stereo_lk_cpu::pruneGridFeatures()
{
    // 遍历当前存储的特征点网格
    for (auto& item : *curr_features_ptr)
    {
        auto& grid_features = item.second;

        // 如果当前网格中的特征点数小于等于最大允许数量，则无需裁剪
        if (grid_features.size() <= grid_max_feature_num)
            continue;

        // 按特征点的生命周期（lifetime）进行排序，生命周期长的排在前面
        std::sort(grid_features.begin(), grid_features.end(),
            &vis_stereo_lk_cpu::featureCompareByLifetime);

        // 仅保留最长生命周期的 grid_max_feature_num 个特征点
        grid_features.erase(grid_features.begin() + grid_max_feature_num, grid_features.end());
    }
    return;
}

vector<cv::Point2f> vis_stereo_lk_cpu::distortPoints(
    const vector<cv::Point2f>& pts_in,             // 输入的特征点（去畸变后的点）
    const cv::Vec4d& intrinsics,                   // 相机内参
    const string& distortion_model,                // 畸变模型类型（如 radtan 或 equidistant）
    const cv::Vec4d& distortion_coeffs)            // 畸变系数
{
    // 构建相机内参矩阵 K（相机内参）
    const cv::Matx33d K(intrinsics[0], 0.0, intrinsics[2],
        0.0, intrinsics[1], intrinsics[3],
        0.0, 0.0, 1.0);

    vector<cv::Point2f> pts_out;  // 输出的畸变后的特征点

    // 根据畸变模型进行畸变处理
    if (distortion_model == "radtan")  // 如果是针孔相机的径向畸变模型
    {
        // 将输入的特征点转换为齐次坐标
        vector<cv::Point3f> homogenous_pts;
        cv::convertPointsToHomogeneous(pts_in, homogenous_pts);

        // 使用 projectPoints 将齐次坐标转换为畸变后的点
        cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
            distortion_coeffs, pts_out);
    }
    else if (distortion_model == "equidistant")  // 如果是等距畸变模型（如鱼眼镜头）
    {
        // 使用 cv::fisheye::distortPoints 进行畸变处理
        cv::fisheye::distortPoints(pts_in, pts_out, K, distortion_coeffs);
    }
    else
    {
        // 如果模型不认识，则打印警告并使用默认的径向畸变模型进行畸变处理
        printf("The model %s is unrecognized, using radtan instead...",
            distortion_model.c_str());

        // 将输入的特征点转换为齐次坐标
        vector<cv::Point3f> homogenous_pts;
        cv::convertPointsToHomogeneous(pts_in, homogenous_pts);

        // 使用 projectPoints 将齐次坐标转换为畸变后的点
        cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
            distortion_coeffs, pts_out);
    }

    // 返回畸变后的特征点
    return pts_out;
}


void vis_stereo_lk_cpu::undistortPoints(
    const vector<cv::Point2f>& pts_in,             // 输入的特征点（畸变后的点）
    const cv::Vec4d& intrinsics,                   // 相机内参
    const string& distortion_model,                // 畸变模型类型（如 radtan 或 equidistant）
    const cv::Vec4d& distortion_coeffs,            // 畸变系数
    vector<cv::Point2f>& pts_out,                  // 输出的去畸变后的特征点
    const cv::Matx33d& rectification_matrix,       // 视差矩阵（用于图像校正）
    const cv::Vec4d& new_intrinsics)               // 新的相机内参（去畸变后的内参）
{
    // 如果输入的特征点为空，则直接返回
    if (pts_in.size() == 0)
        return;

    // 构建相机内参矩阵 K（原始相机内参）
    const cv::Matx33d K(
        intrinsics[0], 0.0, intrinsics[2],
        0.0, intrinsics[1], intrinsics[3],
        0.0, 0.0, 1.0);

    // 构建新的相机内参矩阵 K_new（去畸变后的内参）
    const cv::Matx33d K_new(
        new_intrinsics[0], 0.0, new_intrinsics[2],
        0.0, new_intrinsics[1], new_intrinsics[3],
        0.0, 0.0, 1.0);

    // 根据畸变模型进行去畸变处理
    if (distortion_model == "radtan")  // 如果是针孔相机的径向畸变模型
    {
        // 使用 cv::undistortPoints 进行去畸变处理
        cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else if (distortion_model == "equidistant")  // 如果是等距畸变模型（如鱼眼镜头）
    {
        // 使用 cv::fisheye::undistortPoints 进行去畸变处理
        cv::fisheye::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else
    {
        // 如果模型不认识，则打印警告并使用默认的径向畸变模型进行处理
        printf("The model %s is unrecognized, use radtan instead...",
            distortion_model.c_str());
        cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    return;
}


bool vis_stereo_lk_cpu::featureCompareByResponse(
    const FeaturePoint& f1,
    const FeaturePoint& f2)
{

    return f1.response > f2.response;
}

template <typename T>
void vis_stereo_lk_cpu::removeUnmarkedElements(
    const std::vector<T>& raw_vec,
    const std::vector<unsigned char>& markers,
    std::vector<T>& refined_vec)
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

void vis_stereo_lk_cpu::rescalePoints(
    vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2,
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

bool vis_stereo_lk_cpu::keyPointCompareByResponse(
    const cv::KeyPoint& pt1,
    const cv::KeyPoint& pt2)
{
    return pt1.response > pt2.response;
}





bool vis_stereo_lk_cpu::featureCompareByLifetime(
    const FeaturePoint& f1,
    const FeaturePoint& f2)
{
    return f1.lifetime > f2.lifetime;
}

vector<Triple> vis_stereo_lk_cpu::stereoSemiDenseMatch(cv::Mat& disparity)
{
    vector<Triple> pl;
    cv::Matx33d K0(_cam0_intrinsics(0), 0, _cam0_intrinsics(2),
        0, _cam0_intrinsics(1), _cam0_intrinsics(3),
        0, 0, 1);
    cv::Matx33d K1(_cam1_intrinsics(0), 0, _cam1_intrinsics(2),
        0, _cam1_intrinsics(1), _cam1_intrinsics(3),
        0, 0, 1);
    cv::Mat R0, P0, R1, P1, Q;
    stereoRectify(K0, _cam0_distortion_coeffs, K1, _cam1_distortion_coeffs, cam0_curr_img.size(),
        _R_cam0_cam1, _t_cam0_cam1, R0, R1, P0, P1, Q);
    cv::Mat M0_1, M0_2, M1_1, M1_2;
    initUndistortRectifyMap(K0, _cam0_distortion_coeffs, R0, P0, cam0_curr_img.size(), CV_32FC1, M0_1, M0_2);
    initUndistortRectifyMap(K1, _cam1_distortion_coeffs, R1, P1, cam1_curr_img.size(), CV_32FC1, M1_1, M1_2);
    cv::Mat img0_rectify, img1_rectify, img0_sobel;
    cv::Mat img0_rectify_new;
    remap(cam0_curr_img, img0_rectify, M0_1, M0_2, cv::INTER_LINEAR);
    remap(cam1_curr_img, img1_rectify, M1_1, M1_2, cv::INTER_LINEAR);

    Eigen::MatrixXd P1_e;
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

    Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 64, 7, 0, 0, 0, 0, 30, 50, 1);
    sgbm->compute(img0_rectify, img1_rectify, disparity);
    bilateralFilter(img0_rectify, img0_rectify_new, 5, 10, 15);
    Canny(img0_rectify_new, img0_sobel, 100, 150);

    double x, y, z;
    for (int i = 0; i < disparity.rows; i++)
        for (int j = 0; j < disparity.cols; j++)
        {
            double depth = bl / (disparity.at<uint16_t>(i, j) / 16.0 * resize_factor);
            if (depth <= 0) continue;
            x = (j * resize_factor - cx) / f * depth;
            y = (i * resize_factor - cy) / f * depth;
            if (y > 1.5 && depth < 10.0 && img0_sobel.at<uchar>(i, j)>30)
                pl.push_back(Triple(x, y, depth));
        }
    disparity.convertTo(disparity, CV_8UC1, 0.3);
    disparity = img0_sobel + img0_rectify_new;

    return pl;
}

void vis_stereo_lk_cpu::calculateIPM(cv::Mat& ipm)
{
    Triple att = Triple(91.5 / 180 * 3.1415926535, 0, 0); // between the reference IPM frame and the "average" camera frame
    double d = 1.8;
    int IPM_height = 1000;
    int IPM_width = 500;

    SO3 R_c1c0;
    double sp = sin(att(0)), cp = cos(att(0));
    double sr = sin(att(1)), cr = cos(att(1));
    double sy = sin(att(2)), cy = cos(att(2));
    R_c1c0 << cy * cr - sy * sp * sr, -sy * cp, cy* sr + sy * sp * cr,
        sy* cr + cy * sp * sr, cy* cp, sy* sr - cy * sp * cr,
        -cp * sr, sp, cp* cr;

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
        R_ilatest_iave << cy * cr - sy * sp * sr, -sy * cp, cy* sr + sy * sp * cr,
            sy* cr + cy * sp * sr, cy* cp, sy* sr - cy * sp * cr,
            -cp * sr, sp, cp* cr;
        Matrix R_clatest_cave = R_cam0_imu.transpose() * R_ilatest_iave * R_cam0_imu;
        R_c1c0 = R_c1c0 * R_clatest_cave;
    }

    cv::Matx33d K0(_cam0_intrinsics(0), 0, _cam0_intrinsics(2),
        0, _cam0_intrinsics(1), _cam0_intrinsics(3),
        0, 0, 1);
    initUndistortRectifyMap(K0, _cam0_distortion_coeffs, cv::Matx33d::eye(), K0, cam0_curr_img.size(), CV_32FC1, _IPM_M0_1, _IPM_M0_2);

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
            c1_p_c1f(0) = (j - IPM_width / 2) * 0.02;
            c1_p_c1f(1) = -5 - i * 0.02;
            c1_p_c1f(2) = d;
            xy1_y = R_c1c0.transpose() * c1_p_c1f / d;
            xy1 = xy1_y / xy1_y(2);
            uv1 = K0_e * xy1;
            _IPM_mapx.at<float>(IPM_height - i - 1, j) = uv1(0);
            _IPM_mapy.at<float>(IPM_height - i - 1, j) = uv1(1);
        }
    }

    ipm = cv::Mat(IPM_height, IPM_width, CV_8UC1);
    cv::Mat ipm_temp;
    cv::remap(cam0_curr_img, ipm_temp, _IPM_M0_1, _IPM_M0_2, cv::INTER_LINEAR);
    cv::remap(ipm_temp, ipm, _IPM_mapx, _IPM_mapy, cv::INTER_LINEAR);
}