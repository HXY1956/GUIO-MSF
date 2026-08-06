#include "hwa_vis_proc_imgproc.h"
#include "hwa_vis_yolo_v8model.h"

hwa_vis::vis_imgproc_base::vis_imgproc_base(set_base* _set, int cam_group_id) {

    auto set = dynamic_cast<set_vis*>(_set);
    /*  ***Basic Parameters*** */
    _yolo = new vis_yolo_v8ov(_set, XMLKEY_YOLO_DYNA);
    dynamic_classes = { 0,1,2,3,5,7 };
    ts = set->ts(cam_group_id);
    freq = set->freq(cam_group_id);
    cam0_resolution = set->cam0_resolution(cam_group_id);
    cam1_resolution = set->cam1_resolution(cam_group_id);
    cam0_distortion_model = set->cam0_distortion_model(cam_group_id);
    cam1_distortion_model = set->cam1_distortion_model(cam_group_id);
    cam0_intrinsics = set->cam0_intrinsics(cam_group_id);
    cam1_intrinsics = set->cam1_intrinsics(cam_group_id);
    cam0_distortion_coeffs = set->cam0_distortion_coeffs(cam_group_id);
    cam1_distortion_coeffs = set->cam1_distortion_coeffs(cam_group_id);

    stereo = set->stereo(cam_group_id);
    R_cam0_cam1 = set->R_cam0_cam1(cam_group_id);
    t_cam0_cam1 = set->t_cam0_cam1(cam_group_id);
    T_cam0_cam1 = set->T_cam0_cam1(cam_group_id);

    R_cam0_imu = set->R_cam0_imu(cam_group_id);
    t_cam0_imu = set->t_cam0_imu(cam_group_id);
    T_cam0_imu = set->T_cam0_imu(cam_group_id);

    T_cam1_imu = T_cam0_imu * T_cam0_cam1.inverse();
    R_cam1_imu = T_cam1_imu.linear();
    t_cam1_imu = T_cam1_imu.translation();
    dt_cam0_imu = set->dt_cam0_imu(cam_group_id);

    grid_row = set->grid_row(cam_group_id);
    grid_col = set->grid_col(cam_group_id);
    grid_min_feature_num = set->grid_min_feature_num(cam_group_id);
    grid_max_feature_num = set->grid_max_feature_num(cam_group_id);
    pyramid_levels = set->pyramid_levels(cam_group_id);
    patch_size = set->patch_size(cam_group_id);
    fast_threshold = set->fast_threshold(cam_group_id);
    ransac_threshold = set->ransac_threshold(cam_group_id);
    stereo_threshold = set->stereo_threshold(cam_group_id);
    max_iteration = set->max_iteration(cam_group_id);
    track_precision = set->track_precision(cam_group_id);
    equalize = set->_equalize(cam_group_id);
    max_cam_state_size = set->max_cam_state_size(cam_group_id);
    position_std_threshold = set->position_std_threshold(cam_group_id);
    rotation_threshold = set->rotation_threshold(cam_group_id);

    feature_observation_noise = set->feature_observation_noise(cam_group_id);
    estimate_extrinsic = set->estimate_extrinsic(cam_group_id);
    estimate_t = set->estimate_t(cam_group_id);

    if (estimate_extrinsic)
        ex_param_num += 6;
    if (estimate_t)
        ex_param_num += 1;

    _cam_group_id = cam_group_id;
    if (estimate_t)
    {
        initial_cam_t_cov = set->initial_t_cov(cam_group_id);
        estimate_t_allcam = set->estimate_t_allcam(cam_group_id);
    }
    if (estimate_extrinsic)
    {
        initial_cam_extrinsic_rotation_cov = set->initial_extrinsic_rotation_cov(cam_group_id);
        initial_cam_extrinsic_translation_cov = set->initial_extrinsic_translation_cov(cam_group_id);
        estimate_extrinsic_seperately = set->estimate_extrinsic_seperately(cam_group_id);
        estimate_extrinsic_allcam = set->estimate_extrinsic_allcam(cam_group_id);
    }
    cam_update_skip = set->cam_update_skip(cam_group_id);
    _pose_history_max_size = 40;
    max_cnt = set->max_cnt(cam_group_id);
    usingstereorecify = set->usingstereorecify();
}

void hwa_vis::vis_imgproc_base::load_imuobs(const double &t,const std::vector<Triple> & wm,const std::vector<Triple> & vm, double imu_ts)
{
    assert(wm.size() == vm.size());
    /* only one sample can be processed successfully */
    assert(wm.size() == 1);
    for (int i = 0; i < wm.size(); i++)
    {
        IMU_MSG tmp_imu;
        tmp_imu.t = t;
        tmp_imu.angular_velocity = wm[i]/imu_ts;
        tmp_imu.linear_acceleration = vm[i]/imu_ts;
        _vecimu.push_back(tmp_imu);
    }
}

void hwa_vis::vis_imgproc_base::dyna_detect(const cv::Mat& img) {

    dyna_box.clear();

    std::vector<hwa_vis::Detection> detections = _yolo->detect(img);

    for (const auto& det : detections)
    {
        if (dynamic_classes.count(det.class_id))
            dyna_box.emplace_back(det);
    }
}

void hwa_vis::vis_imgproc_base::load_imgobs(const double &t, const IMG_PATH &img_path)
{
    cur_img_path = img_path;
}

hwa_vis::PointCloud hwa_vis::vis_imgproc_base::ProcessBatch()
{
    PointCloud res;
    return res;
}

void hwa_vis::vis_imgproc_base::ProcessBatchT(const double& t, PointCloud & pointcloud)
{
    pointcloud = ProcessBatch();
}

void hwa_vis::vis_imgproc_base::add_camera_pose(SO3 R, Triple t)
{
    _pose_history.push_back(std::make_pair(R, t));
    while (_pose_history.size() > _pose_history_max_size)
        _pose_history.pop_front();
}


void hwa_vis::vis_imgproc_base::removeDynamicPoints(
    const std::vector<cv::Point2f>& curr_pts,
    const std::vector<Detection>& dyna_box,
    std::vector<unsigned char>& inliers)
{
    inliers.clear();

    for (size_t i = 0; i < curr_pts.size(); ++i)
    {
        bool is_dynamic = false;

        for (const auto& det : dyna_box)
        {
            if (det.box.contains(curr_pts[i]))
            {
                is_dynamic = true;
				//std::cout << "find dynamic point at: " << curr_pts[i] << std::endl;
                break;
            }
        }

        if (!is_dynamic)
            inliers.push_back(1);
        else
            inliers.push_back(0);
    }
}
