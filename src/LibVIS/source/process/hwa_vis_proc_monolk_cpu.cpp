#include "hwa_vis_proc_monolk.h"
#include "hwa_set_vis.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"

using namespace hwa_base;
using namespace hwa_vis;
using namespace hwa_set;
using namespace std;
using namespace cv;

vis_mono_lk_cpu::vis_mono_lk_cpu(set_base* _set, int cam_group_id) :vis_imgproc<cv::Mat>(_set, cam_group_id),
n_id(0)
{

}

PointCloud vis_mono_lk_cpu::ProcessBatch(const double& t)
{
    cv::Mat _img0 = cv::imread(cur_img_path.img0_path, 0);
    ONE_FRAME _img_msg;
    _img_msg.t = cur_img_path.t;
    _img_msg.img0 = make_shared<cv::Mat>(_img0);
    cur_img = *(_img_msg.img0);

    if (!IsInitialized)
    {
        Initialize();
        IsInitialized = true;
    }

    row = cur_img.rows;
    col = cur_img.cols;

    if (0)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
    }

    cur_pts.clear();

    if (prev_pts.size() > 0)//no_fisrt_img:
    {
        trackFeatures();
        rejectWithF();
    }
    else //first_img
    {
        if (_vecimu.size() > 0)
        {
            //只保留最后一个imu
            IMU_MSG last_imu = _vecimu.at(_vecimu.size() - 1);

            _vecimu.clear();
            _vecimu.push_back(last_imu);
        }

    }

    for (auto& n : track_cnt)
        n++;


    addnewFeatures();

    drawFeatures();

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_time = cur_time;
    prev_ids = cur_ids;

    publish();

    return _pointCloud;
}

bool vis_mono_lk_cpu::Initialize()
{
    eigen2cv(R_cam0_imu, _R_cam0_imu);
    eigen2cv(t_cam0_imu, _t_cam0_imu);
    _cam0_distortion_model = distortion2str(cam0_distortion_model);
    _cam0_intrinsics << cam0_intrinsics(0), cam0_intrinsics(1), cam0_intrinsics(2), cam0_intrinsics(3);
    _cam0_distortion_coeffs << cam0_distortion_coeffs(0), cam0_distortion_coeffs(1), cam0_distortion_coeffs(2), cam0_distortion_coeffs(3);
    return true;
}

bool vis_mono_lk_cpu::inBorder(const cv::Point2f& pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

void vis_mono_lk_cpu::_reduceVector(vector<cv::Point2f>& v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void vis_mono_lk_cpu::_reduceVector(vector<int>& v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void vis_mono_lk_cpu::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], cur_ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>>& a, const pair<int, pair<cv::Point2f, int>>& b)
        {
            return a.first > b.first;
        });

    cur_pts.clear();
    cur_ids.clear();
    track_cnt.clear();

    for (auto& it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            cur_ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            //！！！ vins 设置为30 ！！！
            cv::circle(mask, it.second.first, 30, 0, -1);
        }
    }
    after_setmask = cur_pts.size();
    //cout << "after_setmask:" << after_setmask << endl;
}

void vis_mono_lk_cpu::addnewFeatures()
{
    setMask();
    printf("candidates: %d; track: %d; two_ransac: %d; cv_ransac: %d; ransac: %d/%d=%f; after_ransac：%d; mask: %d\n",
        before_tracking, after_tracking, after_twopoint_ransac,
        after_ransac, after_ransac, after_tracking,
        static_cast<double>(after_ransac) /
        (static_cast<double>(before_tracking) + 1e-5), after_ransac, after_setmask);
    cout << "==================================" << endl;

    int n_max_cnt = max_cnt - static_cast<int>(cur_pts.size());
    if (n_max_cnt > 0)
    {
        if (mask.empty())
            cout << "mask is empty " << endl;
        if (mask.type() != CV_8UC1)
            cout << "mask type wrong " << endl;
        //MIN_DIST=30
        cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.001, 30, mask);
    }
    else
        n_pts.clear();


    for (auto& p : n_pts)
    {
        cur_pts.push_back(p);
        cur_ids.push_back(n_id++);
        track_cnt.push_back(1);
    }
    //printf("feature cnt after add %d\n", (int)cur_ids.size());
}
void vis_mono_lk_cpu::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        //vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        //undistortPoints(
        //    prev_pts, _cam0_intrinsics, _cam0_distortion_model,
        //    _cam0_distortion_coeffs, un_prev_pts);
        //undistortPoints(
        //    cur_pts, _cam0_intrinsics, _cam0_distortion_model,
        //    _cam0_distortion_coeffs, un_cur_pts);

        //vector<uchar> status;
        //cv::findFundamentalMat(prev_pts, cur_pts, cv::FM_RANSAC, 1.0, 0.99, status);
        //_reduceVector(prev_pts, status);
        //_reduceVector(cur_pts, status);

        //_reduceVector(prev_ids, status);
        //_reduceVector(cur_ids, status);
        //_reduceVector(track_cnt, status);

        after_ransac = cur_pts.size();


    }
}

double vis_mono_lk_cpu::distance(cv::Point2f& pt1, cv::Point2f& pt2)
{
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void vis_mono_lk_cpu::undistortPoints(
    const vector<cv::Point2f>& pts_in,
    const cv::Vec4d& intrinsics,
    const string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    vector<cv::Point2f>& pts_out,
    const cv::Matx33d& rectification_matrix,
    const cv::Vec4d& new_intrinsics)
{

    if (pts_in.size() == 0)
        return;

    const cv::Matx33d K(
        intrinsics[0], 0.0, intrinsics[2],
        0.0, intrinsics[1], intrinsics[3],
        0.0, 0.0, 1.0);

    const cv::Matx33d K_new(
        new_intrinsics[0], 0.0, new_intrinsics[2],
        0.0, new_intrinsics[1], new_intrinsics[3],
        0.0, 0.0, 1.0);


    if (distortion_model == "radtan")
    {
        cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else if (distortion_model == "equidistant")
    {
        cv::fisheye::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else
    {
        printf("The model %s is unrecognized, use radtan instead...",
            distortion_model.c_str());
        cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    return;
}

void vis_mono_lk_cpu::integrateImuData(cv::Matx33f& cam0_R_p_c)
{
    auto begin_iter = _vecimu.begin();
    while (begin_iter != _vecimu.end())
    {
        if (begin_iter->t -
            prev_time < -0.01)
            ++begin_iter;
        else
            break;
    }

    auto end_iter = begin_iter;
    while (end_iter != _vecimu.end())
    {
        if (end_iter->t -
            cur_time < 0.002)
            ++end_iter;
        else
            break;
    }

    Vec3f mean_ang_vel(0.0, 0.0, 0.0);
    for (auto iter = begin_iter; iter < end_iter; ++iter)

    {
        mean_ang_vel += Vec3f(iter->angular_velocity.x(),
            iter->angular_velocity.y(), iter->angular_velocity.z());

    }

    if (end_iter - begin_iter > 0)
        mean_ang_vel *= 1.0f / (end_iter - begin_iter);


    Vec3f cam0_mean_ang_vel = _R_cam0_imu.t() * mean_ang_vel;



    double dtime = (cur_time - prev_time);
    Rodrigues(cam0_mean_ang_vel * dtime, cam0_R_p_c);

    cam0_R_p_c = cam0_R_p_c.t();


    _vecimu.erase(_vecimu.begin(), end_iter);
    return;
}

void vis_mono_lk_cpu::predictFeatureTracking(
    const vector<cv::Point2f>& input_pts,
    const cv::Matx33f& R_p_c,
    const cv::Vec4d& intrinsics,
    vector<cv::Point2f>& compensated_pts)
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

void vis_mono_lk_cpu::trackFeatures()
{
    Matx33f cam0_R_p_c;

    integrateImuData(cam0_R_p_c);

    predictFeatureTracking(prev_pts,
        cam0_R_p_c, _cam0_intrinsics, cur_pts);

    before_tracking = prev_pts.size();
    //这个之后放在track里面,并且加入imu预测部分，不过可以放在后续
    vector<uchar> status;
    vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
    //  cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
    //    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
    // reverse check
    if (flow_back)
    {
        vector<uchar> reverse_status;
        vector<cv::Point2f> reverse_pts = prev_pts;
        cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
            {
                status[i] = 1;
            }
            else
                status[i] = 0;
        }
    }

    for (int i = 0; i < int(cur_pts.size()); i++)
        if (status[i] && !inBorder(cur_pts[i]))
            status[i] = 0;
    _reduceVector(prev_pts, status);
    _reduceVector(cur_pts, status);
    _reduceVector(prev_ids, status);
    _reduceVector(cur_ids, status);
    _reduceVector(track_cnt, status);
    after_tracking = cur_pts.size();


    vector<int> cam0_ransac_inliers(0);
    twoPointRansac(prev_pts, cur_pts,
        cam0_R_p_c, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, ransac_threshold,
        0.99, cam0_ransac_inliers);

    _reduceVector(prev_pts, status);
    _reduceVector(cur_pts, status);
    _reduceVector(prev_ids, status);
    _reduceVector(cur_ids, status);
    _reduceVector(track_cnt, status);
    after_twopoint_ransac = cur_pts.size();
    //cout << "after_twopoint_tracking:" << after_twopoint_ransac << endl;

}

void vis_mono_lk_cpu::rescalePoints(
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

void vis_mono_lk_cpu::twoPointRansac(
    const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2,
    const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
    const std::string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    vector<int>& inlier_markers)
{


    if (pts1.size() != pts2.size())
        printf("Sets of different size (%d and %d) are used...\n",
            pts1.size(), pts2.size());


    double norm_pixel_unit = 2.0 / (intrinsics[0] + intrinsics[1]);


    int iter_num = static_cast<int>(
        ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));


    inlier_markers.clear();
    inlier_markers.resize(pts1.size(), 1);


    vector<Point2f> pts1_undistorted(pts1.size());
    vector<Point2f> pts2_undistorted(pts2.size());
    undistortPoints(
        pts1, intrinsics, distortion_model,
        distortion_coeffs, pts1_undistorted);
    undistortPoints(
        pts2, intrinsics, distortion_model,
        distortion_coeffs, pts2_undistorted);



    for (auto& pt : pts1_undistorted)
    {
        Vec3f pt_h(pt.x, pt.y, 1.0f);
        Vec3f pt_hc = R_p_c * pt_h;
        pt.x = pt_hc[0];
        pt.y = pt_hc[1];
    }


    float scaling_factor = 0.0f;
    rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);


    norm_pixel_unit *= scaling_factor;




    vector<Point2d> pts_diff(pts1_undistorted.size());
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
        printf("Degenerated motion...\n");
        for (int i = 0; i < pts_diff.size(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;


            if (sqrt(pts_diff[i].dot(pts_diff[i])) >
                inlier_error * norm_pixel_unit)
                inlier_markers[i] = 0;
        }
        return;
    }


    Matrix coeff_t(pts_diff.size(), 3);
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        coeff_t(i, 0) = pts_diff[i].y;
        coeff_t(i, 1) = -pts_diff[i].x;
        coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y -
            pts1_undistorted[i].y * pts2_undistorted[i].x;
    }

    vector<int> raw_inlier_idx;
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] != 0)
            raw_inlier_idx.push_back(i);
    }


    vector<int> best_inlier_set;
    double best_error = 1e10;
    cv::RNG randomNumber;


    for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx)
    {
        int pair_idx1 = raw_inlier_idx[randomNumber.uniform(
            0, raw_inlier_idx.size())];

        int idx_diff = randomNumber.uniform(
            1, raw_inlier_idx.size());


        int pair_idx2 = pair_idx1 + idx_diff < raw_inlier_idx.size() ?
            pair_idx1 + idx_diff : pair_idx1 + idx_diff - raw_inlier_idx.size();

        // Construct the model;
        Eigen::Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
        Eigen::Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
        Eigen::Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
        vector<double> coeff_l1_norm(3);
        coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
        coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
        coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
        int base_indicator = min_element(coeff_l1_norm.begin(),
            coeff_l1_norm.end()) - coeff_l1_norm.begin();

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

        // Find all the inliers among point pairs.
        Vector error = coeff_t * model;

        std::vector<int> inlier_set;
        for (int i = 0; i < error.rows(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;
            if (std::abs(error(i)) < inlier_error * norm_pixel_unit)
                inlier_set.push_back(i);
        }

        // If the number of inliers is small, the current
        // model is probably wrong.
        if (inlier_set.size() < 0.2 * pts1_undistorted.size())
            continue;

        // Refit the model using all of the possible inliers.
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

        // Compute the error and upate the best model if possible.
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

    // Fill in the markers.
    inlier_markers.clear();
    inlier_markers.resize(pts1.size(), 0);
    for (const auto& inlier_idx : best_inlier_set)
        inlier_markers[inlier_idx] = 1;

    return;
}

void vis_mono_lk_cpu::drawFeatures()
{
    Scalar tracked(0, 255, 0);
    Scalar new_feature(0, 255, 255);


    int img_height = cur_img.rows;
    int img_width = cur_img.cols;
    out_img = std::make_shared<cv::Mat>(img_height, img_width, CV_8UC3);

    cv::cvtColor(cur_img, *out_img, cv::COLOR_GRAY2BGR);

    map<long long int, Point2f> prev_cam0_points;
    assert(prev_ids.size() == prev_pts.size());
    for (int i = 0; i < prev_ids.size(); i++)
    {
        auto& id = prev_ids.at(i);
        auto& pts = prev_pts.at(i);
        prev_cam0_points[id] = pts;
    }

    map<long long int, Point2f> cur_cam0_points;
    assert(cur_ids.size() == cur_pts.size());
    for (int i = 0; i < cur_ids.size(); i++)
    {
        auto& id = cur_ids.at(i);
        auto& pts = cur_pts.at(i);
        cur_cam0_points[id] = pts;
    }

    for (const auto& id : prev_ids)
    {

        if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
            cur_cam0_points.find(id) != cur_cam0_points.end())
        {
            cv::Point2f prev_pt0 = prev_cam0_points[id];
            cv::Point2f cur_pt0 = cur_cam0_points[id];


            circle(*out_img, cur_pt0, 3, tracked, -1);
            line(*out_img, prev_pt0, cur_pt0, tracked, 1);

            prev_cam0_points.erase(id);
            cur_cam0_points.erase(id);

        }
    }


    for (const auto& new_cam0_point : cur_cam0_points)
    {
        cv::Point2f pt0 = new_cam0_point.second;
        circle(*out_img, pt0, 3, new_feature, -1);
    }

    cv::resize(*out_img, *out_img, out_img->size());
    cv::resize(*out_img, *out_img, cv::Size(out_img->cols / 2.0, out_img->rows / 2.0));
    //imshow(string("Feature: cam_group_") + to_string(_cam_group_id), *out_img);
    //waitKey(1);
    return;
}

void vis_mono_lk_cpu::publish()
{
    vector<cv::Point2f> cur_un_pts;
    undistortPoints(
        cur_pts, _cam0_intrinsics, _cam0_distortion_model,
        _cam0_distortion_coeffs, cur_un_pts);

    _pointCloud.features.clear();
    FeaturePoint feature_msg;

    _pointCloud.time = cur_time;

    assert(cur_ids.size() == cur_pts.size());
    assert(cur_ids.size() == cur_un_pts.size());

    for (int i = 0; i < cur_ids.size(); ++i)
    {
        feature_msg.id = cur_ids[i];
        feature_msg.cam0_point = cur_un_pts[i];
        _pointCloud.features.push_back(feature_msg);
    }
    return;
}
