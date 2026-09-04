#include "hwa_pg_posegraph.h"
#include "hwa_pg_internal.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/ceres.h>
#include <cmath>
#include <iostream>
#include <chrono>

namespace hwa_pg {
namespace {

// VINS-Mono 4DoF pose-graph helpers (ported from pose_graph.h/.cpp): each
// keyframe is parameterized by yaw (degrees) + translation only; pitch and
// roll are locked to the raw VIO attitude.
const double kPi4DoF = 3.14159265358979323846;

double normalizeAngleDeg(double a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

template <typename T>
T NormalizeAngleDeg(const T& angle_degrees) {
    if (angle_degrees > T(180.0))
        return angle_degrees - T(360.0);
    else if (angle_degrees < T(-180.0))
        return angle_degrees + T(360.0);
    else
        return angle_degrees;
}

// 1-DOF yaw angle parameterization (degrees), like VINS AngleLocalParameterization.
class AngleLocalParameterization : public ceres::LocalParameterization {
public:
    virtual ~AngleLocalParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
        *x_plus_delta = normalizeAngleDeg(*x + *delta);
        return true;
    }
    virtual bool ComputeJacobian(const double*, double* jacobian) const {
        jacobian[0] = 1.0;
        return true;
    }
    virtual int GlobalSize() const { return 1; }
    virtual int LocalSize() const { return 1; }
};

// euler angles in degrees -> R = Rz(yaw) * Ry(pitch) * Rx(roll)
template <typename T>
void yprToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9]) {
    const T y = yaw / T(180.0) * T(kPi4DoF);
    const T p = pitch / T(180.0) * T(kPi4DoF);
    const T r = roll / T(180.0) * T(kPi4DoF);

    R[0] = cos(y) * cos(p);
    R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
    R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
    R[3] = sin(y) * cos(p);
    R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
    R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
    R[6] = -sin(p);
    R[7] = cos(p) * sin(r);
    R[8] = cos(p) * cos(r);
}

template <typename T>
void rotationMatrixTranspose(const T R[9], T inv_R[9]) {
    inv_R[0] = R[0]; inv_R[1] = R[3]; inv_R[2] = R[6];
    inv_R[3] = R[1]; inv_R[4] = R[4]; inv_R[5] = R[7];
    inv_R[6] = R[2]; inv_R[7] = R[5]; inv_R[8] = R[8];
}

template <typename T>
void rotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3]) {
    r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
    r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
    r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
}

// VINS FourDOFError: odometry edges (relative_t expressed in frame i and
// relative_yaw in degrees; pitch/roll of frame i are fixed VIO values).
struct FourDOFError {
    FourDOFError(double t_x, double t_y, double t_z, double relative_yaw,
                 double pitch_i, double roll_i)
        : t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw),
          pitch_i(pitch_i), roll_i(roll_i) {}

    template <typename T>
    bool operator()(const T* const yaw_i, const T* ti,
                    const T* yaw_j, const T* tj, T* residuals) const {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];

        // euler to rotation
        T w_R_i[9];
        yprToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        rotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        rotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

        residuals[0] = t_i_ij[0] - T(t_x);
        residuals[1] = t_i_ij[1] - T(t_y);
        residuals[2] = t_i_ij[2] - T(t_z);
        residuals[3] = NormalizeAngleDeg(yaw_j[0] - yaw_i[0] - T(relative_yaw));

        return true;
    }

    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw,
                                       const double pitch_i, const double roll_i) {
        return new ceres::AutoDiffCostFunction<FourDOFError, 4, 1, 3, 1, 3>(
            new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i));
    }

    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
};

// VINS FourDOFWeightError: loop edges (used with Huber loss; yaw residual /10).
struct FourDOFWeightError {
    FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw,
                       double pitch_i, double roll_i)
        : t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw),
          pitch_i(pitch_i), roll_i(roll_i) {
        weight = 1;
    }

    template <typename T>
    bool operator()(const T* const yaw_i, const T* ti,
                    const T* yaw_j, const T* tj, T* residuals) const {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];

        // euler to rotation
        T w_R_i[9];
        yprToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        rotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        rotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngleDeg(yaw_j[0] - yaw_i[0] - T(relative_yaw)) * T(weight) / T(10.0);

        return true;
    }

    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw,
                                       const double pitch_i, const double roll_i) {
        return new ceres::AutoDiffCostFunction<FourDOFWeightError, 4, 1, 3, 1, 3>(
            new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i));
    }

    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
};

// rotation -> yaw/pitch/roll in degrees (VINS Utility::R2ypr)
Eigen::Vector3d r2yprDeg(const Eigen::Matrix3d& R) {
    Eigen::Vector3d n = R.col(0), o = R.col(1), a = R.col(2);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    return Eigen::Vector3d(y, p, r) * (180.0 / kPi4DoF);
}

// yaw/pitch/roll in degrees -> R = Rz(yaw) * Ry(pitch) * Rx(roll)
Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr) {
    const double y = ypr(0) / 180.0 * kPi4DoF;
    const double p = ypr(1) / 180.0 * kPi4DoF;
    const double r = ypr(2) / 180.0 * kPi4DoF;

    Eigen::Matrix3d Rz;
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;
    Eigen::Matrix3d Ry;
    Ry << cos(p), 0, sin(p),
          0, 1, 0,
          -sin(p), 0, cos(p);
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, cos(r), -sin(r),
          0, sin(r), cos(r);
    return Rz * Ry * Rx;
}

double yawOf(const Eigen::Matrix3d& R) {
    return std::atan2(R(1, 0), R(0, 0));
}

double normalizeAngle(double a) {
    while (a > 3.141592653589793) a -= 2.0 * 3.141592653589793;
    while (a < -3.141592653589793) a += 2.0 * 3.141592653589793;
    return a;
}


// Local ENU-like frame (x=east, y=north, z=up) for an ECEF position. z is the
// geocentric radial direction, which is close enough to the geodetic normal to
// turn the 4DoF "yaw" into a rotation about LOCAL up instead of the ECEF
// z-axis (the VINS assumption that is invalid in raw ECEF coordinates).
Eigen::Matrix3d ecefLocalUpFrame(const Eigen::Vector3d& pos_ecef) {
    const Eigen::Vector3d up = pos_ecef.normalized();
    Eigen::Vector3d east = Eigen::Vector3d::UnitZ().cross(up);
    if (east.norm() < 1e-9)
        east = Eigen::Vector3d::UnitY();
    east.normalize();
    const Eigen::Vector3d north = up.cross(east);
    Eigen::Matrix3d C;
    C.row(0) = east.transpose();
    C.row(1) = north.transpose();
    C.row(2) = up.transpose();
    return C;
}

template <typename T>
void reduceVector(std::vector<T>& v, const std::vector<unsigned char>& status) {
    int j = 0;
    for (int i = 0; i < (int)v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

} // namespace

PGPoseGraph::PGPoseGraph(const PGConfig& cfg) : cfg_(cfg) {
    worker_ = std::thread(&PGPoseGraph::workerLoop, this);
}

PGPoseGraph::~PGPoseGraph() {
    stopWorker();
}


void PGPoseGraph::setCamera(const Eigen::Matrix3d& R_cb, const Eigen::Vector3d& t_cb,
                            Eigen::Vector4d cam_instrinsics, Eigen::Vector4d cam_distortion_coeffs, std::string cam_distortion_model) {
    cfg_.R_cb = R_cb;
    cfg_.t_cb = t_cb;
    cfg_.fx = cam_instrinsics[0];
    cfg_.fy = cam_instrinsics[1];
    cfg_.cx = cam_instrinsics[2];
    cfg_.cy = cam_instrinsics[3];

    cfg_._cam0_distortion_model = cam_distortion_model;
    cfg_._cam0_intrinsics << cam_instrinsics(0), cam_instrinsics(1), cam_instrinsics(2), cam_instrinsics(3);
    cfg_._cam0_distortion_coeffs << cam_distortion_coeffs(0), cam_distortion_coeffs(1), cam_distortion_coeffs(2), cam_distortion_coeffs (3);
}

void PGPoseGraph::addKeyFrame(const RawKeyFrame& raw) {
    std::lock_guard<std::mutex> lk(m_queue_);
    // Bound the backlog: if the worker cannot keep up, drop the oldest raw
    // keyframe and keep the newest. Otherwise the queue grows without limit and
    // the pose-graph thread stays permanently behind (looks stuck in query).
    while (raw_queue_.size() >= cfg_.max_raw_queue)
        raw_queue_.pop();
    raw_queue_.push(raw);
    cv_.notify_all();
}

void PGPoseGraph::setKeyframeThinning(int skip_first, double min_disp, double min_yaw_deg) {
    cfg_.skip_first_cnt = skip_first;
    cfg_.min_keyframe_disp = min_disp;
    cfg_.min_keyframe_yaw_deg = min_yaw_deg;
}

bool PGPoseGraph::getReloResult(
    double& time, Eigen::Vector3d& old_pos, Eigen::Matrix3d& old_rot,
    std::vector<std::pair<double, Eigen::Vector2d>>& obs) const {
    std::lock_guard<std::mutex> lk(m_relo_);
    if (!relo_found_) return false;
    time = relo_time_;
    old_pos = relo_old_pos_;
    old_rot = relo_old_rot_;
    obs = relo_obs_;
    return !obs.empty();
}

size_t PGPoseGraph::size() const {
    std::lock_guard<std::mutex> lk(m_queue_);
    return kfs_.size();
}

bool PGPoseGraph::getPoseGraphSnapshot(
    std::vector<Eigen::Vector3d>& positions,
    std::vector<std::pair<int, int>>& loop_pairs) const {
    std::lock_guard<std::mutex> lk(m_queue_);
    positions.clear();
    loop_pairs.clear();
    if (kfs_.empty()) return false;
    positions.reserve(kfs_.size());
    for (size_t i = 0; i < kfs_.size(); ++i) {
        const auto& kf = kfs_[i];
        // Current OPTIMIZED pose (rot/pos are rewritten by optimizeGraph;
        // vio_rot/vio_pos keep the raw VIO snapshot).
        positions.push_back(kf->pos);
        if (kf->has_loop && kf->loop_index >= 0 &&
            kf->loop_index < (int)kfs_.size() && kf->loop_index != (int)i)
            loop_pairs.emplace_back((int)kf->id, kf->loop_index);
    }
    return true;
}

void PGPoseGraph::publishReloResult(const std::shared_ptr<PGKeyFrame>& old_kf) {
    std::lock_guard<std::mutex> lk(m_relo_);
    if (old_kf) {
        // old-frame pose AFTER the 4DoF optimization (corrected anchor)
        relo_old_pos_ = old_kf->pos;
        relo_old_rot_ = old_kf->rot;
    }
    relo_found_ = true;
    relo_pending_ = false;
    relo_cv_.notify_all();
}

bool PGPoseGraph::isReloPending() const {
    std::lock_guard<std::mutex> lk(m_relo_);
    return relo_pending_;
}

bool PGPoseGraph::waitForReloReady(int timeout_ms) const {
    std::unique_lock<std::mutex> lk(m_relo_);
    if (timeout_ms <= 0) {
        relo_cv_.wait(lk, [this] { return !relo_pending_; });
    } else {
        relo_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                          [this] { return !relo_pending_; });
    }
    return !relo_pending_ && relo_found_;
}

void PGPoseGraph::applyPoseDrift(Eigen::Matrix3d& R, Eigen::Vector3d& p) const {
    std::lock_guard<std::mutex> lk(m_queue_);
    if (!drift_valid_ || !local_frame_init_) return;
    const Eigen::Matrix3d C = local_C_e_;
    const Eigen::Vector3d p0 = local_p0_ecef_;
    const Eigen::Matrix3d Rz = ypr2R(Eigen::Vector3d(drift_yaw_deg_, 0, 0));
    Eigen::Vector3d pl = C * (p - p0);
    pl = Rz * pl + drift_t_;
    p = C.transpose() * pl + p0;
    R = C.transpose() * (Rz * (C * R));
}

bool PGPoseGraph::getDrift(double& yaw_deg, Eigen::Vector3d& t) const {
    std::lock_guard<std::mutex> lk(m_queue_);
    if (!drift_valid_) return false;
    yaw_deg = drift_yaw_deg_;
    t = drift_t_;
    return true;
}

void PGPoseGraph::stopWorker() {
    {
        std::lock_guard<std::mutex> lk(m_queue_);
        if (worker_stopped_) return;
        stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable())
        worker_.join();
    worker_stopped_ = true;
}

bool PGPoseGraph::getPoseGraphFinalTrajectory(
    std::vector<std::pair<double, Eigen::Vector3d>>& out) {
    // Drain the worker backlog first so the result contains every keyframe
    // that was queued before the program ended.
    stopWorker();
    std::lock_guard<std::mutex> lk(m_queue_);
    out.clear();
    out.reserve(kfs_.size());
    for (const auto& kf : kfs_)
        out.emplace_back(kf->time, kf->pos);   // optimized pose (ECEF)
    return !out.empty();
}



void PGPoseGraph::undistortPoints(
    const vector<cv::KeyPoint>& pts_in,             // 输入的特征点（畸变后的点）
    const cv::Vec4d& intrinsics,                   // 相机内参
    const string& distortion_model,                // 畸变模型类型（如 radtan 或 equidistant）
    const cv::Vec4d& distortion_coeffs,            // 畸变系数
    vector<cv::KeyPoint>& pts_out,                  // 输出的去畸变后的特征点
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

    // KeyPoint -> Point2f
    std::vector<cv::Point2f> points_in;
    points_in.reserve(pts_in.size());

    for (const auto& kp : pts_in)
        points_in.push_back(kp.pt);

    // 去畸变后的 Point2f
    std::vector<cv::Point2f> points_out;

    // 根据畸变模型进行去畸变处理
    if (distortion_model == "radtan")  // 如果是针孔相机的径向畸变模型
    {
        // 使用 cv::undistortPoints 进行去畸变处理
        cv::undistortPoints(points_in, points_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else if (distortion_model == "equidistant")  // 如果是等距畸变模型（如鱼眼镜头）
    {
        // 使用 cv::fisheye::undistortPoints 进行去畸变处理
        cv::fisheye::undistortPoints(points_in, points_out, K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    else
    {
        // 如果模型不认识，则打印警告并使用默认的径向畸变模型进行处理
        printf("The model %s is unrecognized, use radtan instead...",
            distortion_model.c_str());
        cv::undistortPoints(points_in, points_out , K, distortion_coeffs,
            rectification_matrix, K_new);
    }
    // KeyPoint -> Point2f
    pts_out.clear();
    pts_out.reserve(points_out.size());
    for (const auto& pt : points_out)
    {
        cv::KeyPoint kp;
        kp.pt = pt;
        pts_out.push_back(kp);
    }

    return;
}

// VINS KeyFrame::computeBRIEFPoint + computeWindowBRIEFPoint
std::shared_ptr<PGKeyFrame> PGPoseGraph::buildKeyFrame(const RawKeyFrame& raw) {
    cv::Mat image = cv::imread(raw.image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty())
        return nullptr;

    PG_BriefExtractor extractor(cfg_.brief_pattern);
    auto kf = std::make_shared<PGKeyFrame>();
    kf->time = raw.time;
    kf->rot = raw.rot;
    kf->pos = raw.pos;
    kf->vio_rot = raw.rot;   // raw VIO snapshot for 4DoF re-optimization
    kf->vio_pos = raw.pos;
    // Initialize the PG pose in the corrected map frame: raw VIO pose
    // transformed by the latest raw->corrected drift (identity before the
    // first loop). vio_* keeps the raw snapshot for measurements.
    applyPoseDrift(kf->rot, kf->pos);
    kf->point_3d = raw.point_3d;
    kf->point_2d_uv = raw.point_2d_uv;
    kf->point_2d_norm = raw.point_2d_norm;
    kf->point_id = raw.point_id;
    kf->image = image;
    kf->dbow = std::make_shared<PGKeyFrameDBoW>();

    // computeBRIEFPoint: FAST corners + BRIEF + normalized coordinates
    cv::FAST(image, kf->keypoints, cfg_.fast_threshold, true);
    extractor(image, kf->keypoints, kf->dbow->brief_descriptors);
    kf->keypoints_norm.clear();

    undistortPoints(
        kf->keypoints, cfg_._cam0_intrinsics, cfg_._cam0_distortion_model,
        cfg_._cam0_distortion_coeffs, kf->keypoints_norm);

    // computeWindowBRIEFPoint: BRIEF at tracked feature locations
    kf->window_keypoints.clear();
    for (const auto& pt : kf->point_2d_uv) {
        cv::KeyPoint key;
        key.pt = pt;
        kf->window_keypoints.push_back(key);
    }
    extractor(image, kf->window_keypoints, kf->dbow->window_brief_descriptors);
    kf->image.release();
    return kf;
}

// VINS PoseGraph::detectLoop: query first, then add, then return the earliest
// qualifying candidate (score > MIN_LOOP_SCORE) when the best neighbour score
// is high enough. Geometric verification (findConnection) is done once by the
// caller on that single candidate, exactly like VINS-Mono.
int PGPoseGraph::detectLoop(const std::shared_ptr<PGKeyFrame>& kf) {
    std::vector<std::pair<int, double>> cands = db_.queryCandidates(
        kf, cfg_.dbow_top, (int)db_.nextEntryId() - cfg_.dbow_min_id_gap);
    db_.addKeyFrame(kf);

    // ret[0] is the nearest neighbour; VINS requires its score > 0.05
    if (cands.size() < 1 || cands[0].second <= 0.05)
        return -1;

    // candidates are considered from index 1; return the earliest (min id)
    int min_index = -1;
    for (size_t i = 1; i < cands.size(); i++) {
        if (cands[i].second > cfg_.min_loop_score) {
            if (min_index == -1 || cands[i].first < min_index)
                min_index = cands[i].first;
        }
    }
    return (min_index != -1 && kf->id > cfg_.dbow_min_id_gap) ? min_index : -1;
}

// VINS KeyFrame::findConnection: window-BRIEF matching + PnP RANSAC.
bool PGPoseGraph::findConnection(const std::shared_ptr<PGKeyFrame>& cur,
                                 const std::shared_ptr<PGKeyFrame>& old_kf) {
    if (!cur->dbow || !old_kf->dbow) return false;
    const std::vector<DBoW2::FBrief::TDescriptor>& cur_win = cur->dbow->window_brief_descriptors;
    const std::vector<DBoW2::FBrief::TDescriptor>& old_desc = old_kf->dbow->brief_descriptors;
    if (cur_win.empty() || old_desc.empty()) return false;

    std::vector<cv::Point2f> matched_2d_cur = cur->point_2d_uv;
    std::vector<cv::Point2f> matched_2d_cur_norm = cur->point_2d_norm;
    std::vector<cv::Point3f> matched_3d = cur->point_3d;
    std::vector<double> matched_id = cur->point_id;
    std::vector<cv::Point2f> matched_2d_old, matched_2d_old_norm;
    std::vector<unsigned char> status;

    // searchByBRIEFDes: best Hamming match of each current window descriptor
    // against the old keyframe's BRIEF descriptors (VINS: distance < 80).
    for (size_t i = 0; i < cur_win.size(); i++) {
        cv::Point2f best_pt(0.f, 0.f), best_norm(0.f, 0.f);
        int best_dist = 128, best_idx = -1;
        for (int j = 0; j < (int)old_desc.size(); j++) {
            int dis = (int)((cur_win[i] ^ old_desc[j]).count());
            if (dis < best_dist) { best_dist = dis; best_idx = j; }
        }
        if (best_idx != -1 && best_dist < 80) {
            status.push_back(1);
            best_pt = old_kf->keypoints[best_idx].pt;
            best_norm = old_kf->keypoints_norm[best_idx].pt;
        } else {
            status.push_back(0);
        }
        matched_2d_old.push_back(best_pt);
        matched_2d_old_norm.push_back(best_norm);
    }
    reduceVector(matched_2d_cur, status);
    reduceVector(matched_2d_cur_norm, status);
    reduceVector(matched_3d, status);
    reduceVector(matched_id, status);
    reduceVector(matched_2d_old, status);
    reduceVector(matched_2d_old_norm, status);

    if ((int)matched_2d_cur.size() <= cfg_.min_loop_num)
        return false;

    // PnPRANSAC (VINS): 3D points of the current keyframe + 2D observations of
    // the old keyframe; initial guess from the current keyframe pose.
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Eigen::Matrix3d R_w_c = cur->rot * cfg_.R_cb;              // camera pose in ECEF
    Eigen::Vector3d T_w_c = cur->pos + cur->rot * cfg_.t_cb;
    Eigen::Matrix3d R_initial = R_w_c.inverse();
    Eigen::Vector3d P_initial = -(R_initial * T_w_c);
    cv::Mat tmp_r, rvec, t;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);

    cv::Mat inliers;
    cv::solvePnPRansac(matched_3d, matched_2d_old_norm, K, cv::Mat(),
                       rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    std::vector<unsigned char> pnp_status(matched_2d_old_norm.size(), 0);
    for (int i = 0; i < inliers.rows; i++)
        pnp_status[inliers.at<int>(i)] = 1;
    reduceVector(matched_2d_cur, pnp_status);
    reduceVector(matched_2d_cur_norm, pnp_status);
    reduceVector(matched_3d, pnp_status);
    reduceVector(matched_id, pnp_status);
    reduceVector(matched_2d_old, pnp_status);
    reduceVector(matched_2d_old_norm, pnp_status);

    if ((int)matched_2d_cur.size() <= cfg_.min_loop_num)
        return false;

    cv::Mat r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d R_pnp, R_w_c_old;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R_pnp(i, j) = r.at<double>(i, j);
    R_w_c_old = R_pnp.transpose();
    Eigen::Vector3d T_pnp(t.at<double>(0), t.at<double>(1), t.at<double>(2));
    Eigen::Vector3d T_w_c_old = R_w_c_old * (-T_pnp);

    Eigen::Matrix3d PnP_R_old = R_w_c_old * cfg_.R_cb.transpose();  // old body pose in ECEF
    Eigen::Vector3d PnP_T_old = T_w_c_old - PnP_R_old * cfg_.t_cb;

    // Measurements are taken in the RAW VIO frame: the PnP object points are
    // raw ECEF (buildRawKeyFrame), while cur->pos may already carry a PG drift
    // correction, so we must use cur->vio_* here to keep one frame. Yaw is
    // extracted about the session-local up axis, not the ECEF z-axis.
    const Eigen::Matrix3d C = local_frame_init_ ? local_C_e_ : Eigen::Matrix3d::Identity();
    Eigen::Vector3d relative_t = PnP_R_old.transpose() * (cur->vio_pos - PnP_T_old);
    Eigen::Quaterniond relative_q(PnP_R_old.transpose() * cur->vio_rot);
    double relative_yaw = normalizeAngle(yawOf(C * cur->vio_rot) - yawOf(C * PnP_R_old));

    if (std::fabs(relative_yaw) < cfg_.max_relative_yaw_deg * 3.141592653589793 / 180.0 &&
        relative_t.norm() < cfg_.max_relative_t) {
        cur->has_loop = true;
        cur->loop_index = old_kf->id;
        cur->loop_inliers = (int)matched_2d_cur.size();
        cur->loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
            relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
            relative_yaw;

        // edge (cur -> old) with T_cur_old (maps cur body frame -> old body frame)
        Eigen::Matrix4d T_cur_old = Eigen::Matrix4d::Identity();
        T_cur_old.block<3, 3>(0, 0) = relative_q.toRotationMatrix();
        T_cur_old.block<3, 1>(0, 3) = relative_t;
        loop_edges_.emplace_back(cur->id, old_kf->id, T_cur_old);
        printf("[PG] loop found cur=%d old=%d rel_t=(%.3f %.3f %.3f) rel_yaw=%.3f deg gap_vio=%.3f\\n",
               cur->id, old_kf->id, relative_t.x(), relative_t.y(), relative_t.z(),
               relative_yaw * 180.0 / 3.141592653589793,
               (cur->vio_pos - old_kf->vio_pos).norm());

        std::cout << "[PG] loop closed: kf " << cur->id << " <-> " << old_kf->id
                  << " score " << cur->loop_score
                  << " inliers " << cur->loop_inliers << std::endl;

        // Two-phase relocalization handshake (see header): announce the loop
        // NOW, before the slow 4DoF optimizeGraph, and publish the corrected
        // old-frame anchor afterwards in publishReloResult(). This lets the
        // FGO main thread wait so the loop keyframe cannot slide out of its
        // sliding window while the graph is being optimized.
        {
            std::lock_guard<std::mutex> lk(m_relo_);
            relo_time_ = cur->time;
            relo_obs_.clear();
            for (size_t i = 0; i < matched_2d_old_norm.size(); i++)
                relo_obs_.emplace_back(matched_id[i],
                                       Eigen::Vector2d(matched_2d_old_norm[i].x,
                                                       matched_2d_old_norm[i].y));
            relo_found_ = false;
            relo_pending_ = true;
            relo_cv_.notify_all();
        }
        return true;
    }
    return false;
}

void PGPoseGraph::workerLoop() {
    while (true) {
        RawKeyFrame raw;
        {
            std::unique_lock<std::mutex> lk(m_queue_);
            cv_.wait(lk, [this] { return stop_ || !raw_queue_.empty(); });
            if (stop_ && raw_queue_.empty())
                break;
            raw = raw_queue_.front();
            raw_queue_.pop();
        }

        // ensure vocabulary (VINS loads a pre-trained .bin; runtime training is
        // only a fallback when no vocabulary path is configured)
        if (!db_.hasVocabulary() && !vocab_attempted_) {
            vocab_attempted_ = true;
            bool loaded = false;
            if (!cfg_.vocabulary_path.empty())
                loaded = db_.loadVocabulary(cfg_.vocabulary_path);
            if (!loaded && cfg_.vocabulary_path.empty() &&
                (int)kfs_.size() >= cfg_.vocab_train_frames)
                db_.trainVocabulary(kfs_, cfg_.vocab_k, cfg_.vocab_L);
        }

        // VINS pose_graph_node keyframe thinning (SKIP_FIRST_CNT + SKIP_DIS):
        // drop the first few keyframes, then only accept frames with enough
        // motion from the last accepted pose-graph keyframe. This keeps the
        // DBoW2 database sparse so query stays fast.
        if (skip_first_done_ < cfg_.skip_first_cnt) {
            skip_first_done_++;
            continue;
        }
        if (!last_pg_pose_valid_) {
            last_pg_pose_valid_ = true;
            last_pg_pos_ = raw.pos;
            last_pg_rot_ = raw.rot;
        } else {
            const double yaw_deg =
                std::fabs(normalizeAngle(yawOf(raw.rot) - yawOf(last_pg_rot_))) *
                180.0 / 3.141592653589793;
            const double disp = (last_pg_rot_.transpose() * (raw.pos - last_pg_pos_)).norm();
            if (disp < cfg_.min_keyframe_disp &&
                yaw_deg < cfg_.min_keyframe_yaw_deg) {
                continue;   // not a keyframe: skip
            }
            last_pg_pos_ = raw.pos;
            last_pg_rot_ = raw.rot;
        }

        // Fix the session-local frame once, from the first accepted keyframe.
        if (!local_frame_init_) {
            local_p0_ecef_ = raw.pos;
            local_C_e_ = ecefLocalUpFrame(raw.pos);
            local_frame_init_ = true;
        }
        std::shared_ptr<PGKeyFrame> kf = buildKeyFrame(raw);
        if (!kf)
            continue;

        int loop_id = -1;
        {
            std::lock_guard<std::mutex> lk(m_queue_);
            kf->id = (int)kfs_.size();
            // odometry edge from the previous keyframe (if any)
            if (!kfs_.empty()) {
                const auto& prev = kfs_.back();
                Eigen::Matrix4d T_rel = kf->poseSE3() * prev->poseSE3().inverse();
                loop_edges_.emplace_back(kfs_.size() - 1, kfs_.size(), T_rel);
            }
            kfs_.push_back(kf);
        }

        if (db_.hasVocabulary())
            loop_id = detectLoop(kf);

        if (loop_id != -1) {
            std::shared_ptr<PGKeyFrame> old_kf =
                (loop_id >= 0 && loop_id < (int)kfs_.size()) ? kfs_[loop_id] : nullptr;
            if (old_kf && old_kf->id != kf->id && findConnection(kf, old_kf)) {
                if (cfg_.enable_posegraph_opt)
                    optimizeGraph();      // correct the historical keyframes
                publishReloResult(old_kf); // publish the CORRECTED anchor
            }
        }

        // Bound the DBoW database / pose-graph window: prune the oldest
        // keyframes so db.query() cost stays bounded (see pruneKeyframes).
        pruneKeyframes();
    }
}

void PGPoseGraph::optimizeGraph() {
    std::lock_guard<std::mutex> lk(m_queue_);
    const int n = (int)kfs_.size();
    if (n < 2) return;

    // Session-local ENU frame: makes the 4DoF yaw a rotation about LOCAL up
    // instead of the ECEF z-axis (the VINS assumption). Fixed once at the first
    // accepted keyframe so it stays stable across pruning.
    Eigen::Matrix3d C;
    Eigen::Vector3d p0;
    if (local_frame_init_) { C = local_C_e_; p0 = local_p0_ecef_; }
    else { p0 = kfs_[0]->vio_pos; C = ecefLocalUpFrame(p0); }

    std::vector<double> yaw(n), pitch(n), roll(n);
    std::vector<Eigen::Vector3d> t(n);            // translation in local frame
    std::vector<Eigen::Vector3d> vio_t_e(n);      // raw ECEF translation
    std::vector<Eigen::Matrix3d> vio_R_e(n);      // raw ECEF rotation

    ceres::Problem problem;
    ceres::LocalParameterization* angle_param = new AngleLocalParameterization();

    for (int i = 0; i < n; ++i) {
        vio_R_e[i] = kfs_[i]->vio_rot;
        vio_t_e[i] = kfs_[i]->vio_pos;
        const Eigen::Matrix3d Rl = C * vio_R_e[i];
        const Eigen::Vector3d ypr = r2yprDeg(Rl);
        yaw[i] = ypr(0);
        pitch[i] = ypr(1);
        roll[i] = ypr(2);
        t[i] = C * (vio_t_e[i] - p0);

        problem.AddParameterBlock(&yaw[i], 1, angle_param);
        problem.AddParameterBlock(t[i].data(), 3);
    }
    // Anchor = oldest retained keyframe, fixed at its RAW local pose (same
    // domain as the measurements, so raw/corrected poses are never mixed).
    problem.SetParameterBlockConstant(&yaw[0]);
    problem.SetParameterBlockConstant(t[0].data());

    // Odometry edges (VINS j = 1..4). The relative translation R_old^T*dP is
    // body-frame invariant; the yaw difference is now a local-up difference.
    for (int i = 1; i < n; ++i) {
        for (int j = 1; j <= 4 && i - j >= 0; ++j) {
            const int old = i - j;
            const Eigen::Vector3d rel_t =
                vio_R_e[old].transpose() * (vio_t_e[i] - vio_t_e[old]);
            const double rel_yaw = yaw[i] - yaw[old];   // both local VIO yaw (deg)
            ceres::CostFunction* cf = FourDOFError::Create(
                rel_t.x(), rel_t.y(), rel_t.z(), rel_yaw, pitch[old], roll[old]);
            problem.AddResidualBlock(cf, nullptr,
                                     &yaw[old], t[old].data(),
                                     &yaw[i], t[i].data());
        }
    }

    // Loop edges: relative_t (in the old body frame) is frame-invariant;
    // relative_yaw was stored by findConnection as a local-up yaw diff (rad).
    ceres::LossFunction* loop_loss = new ceres::HuberLoss(0.1);
    for (int i = 1; i < n; ++i) {
        const auto& kf = kfs_[i];
        if (!kf->has_loop) continue;
        const int old = kf->loop_index;
        if (old < 0 || old >= n || old == i) continue;
        const Eigen::Vector3d rel_t(kf->loop_info(0), kf->loop_info(1),
                                    kf->loop_info(2));
        const double rel_yaw = kf->loop_info(7) * (180.0 / kPi4DoF);
        ceres::CostFunction* cf = FourDOFWeightError::Create(
            rel_t.x(), rel_t.y(), rel_t.z(), rel_yaw, pitch[old], roll[old]);
        problem.AddResidualBlock(cf, loop_loss,
                                 &yaw[old], t[old].data(),
                                 &yaw[i], t[i].data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 5;
    options.function_tolerance = 1e-8;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Write back to ECEF: rot = C^T * Rz(yaw)Ry(pitch)Rx(roll), pos = C^T*t+p0.
    for (int i = 0; i < n; ++i) {
        kfs_[i]->pos = C.transpose() * t[i] + p0;
        kfs_[i]->rot = C.transpose() *
                       ypr2R(Eigen::Vector3d(yaw[i], pitch[i], roll[i]));
    }

    // --- diagnostic: how the graph moved each loop/chain (no behavior change) ---
    {
        double max_move = 0.0;
        int max_id = -1;
        for (int i = 0; i < n; ++i) {
            const Eigen::Vector3d dm = C * (kfs_[i]->pos - kfs_[i]->vio_pos);
            const double m = dm.head<2>().norm();
            if (m > max_move) { max_move = m; max_id = i; }
        }
        printf("[PG] opt done n=%d max_move=%.3f@%d\\n", n, max_move, max_id);
        for (int i = 1; i < n; ++i) {
            const auto& kf = kfs_[i];
            if (!kf->has_loop) continue;
            const int old = kf->loop_index;
            if (old < 0 || old >= n || old == i) continue;
            const Eigen::Vector3d vg = C * (kf->vio_pos - kfs_[old]->vio_pos);
            const Eigen::Vector3d pg = C * (kf->pos - kfs_[old]->pos);
            printf("[PG] loop i=%d old=%d gap_vio=%.3f gap_pg=%.3f\\n",
                   i, old, vg.head<2>().norm(), pg.head<2>().norm());
        }
    }

    // Recompute the raw->corrected drift in the SAME local (z-up) frame so new
    // keyframes and live outputs join the corrected map with a true heading
    // correction.
    if (!kfs_.empty()) {
        const auto& nkf = kfs_.back();
        const Eigen::Vector3d corr_l = C * (nkf->pos - p0);
        const Eigen::Vector3d raw_l = C * (nkf->vio_pos - p0);
        const double yaw_corr = r2yprDeg(C * nkf->rot)(0);
        const double yaw_raw = r2yprDeg(C * nkf->vio_rot)(0);
        drift_yaw_deg_ = yaw_corr - yaw_raw;
        drift_t_ = corr_l - ypr2R(Eigen::Vector3d(drift_yaw_deg_, 0, 0)) * raw_l;
        drift_valid_ = true;
    }
}


void PGPoseGraph::pruneKeyframes() {
    // Only prune once the vocabulary is loaded: before that the keyframes are
    // buffered inside the database as "pending" and ids are not mapped to DBoW
    // entries yet, so pruning would desynchronize the two.
    if (!db_.hasVocabulary())
        return;

    std::lock_guard<std::mutex> lk(m_queue_);
    while (kfs_.size() > cfg_.max_pg_keyframes) {
        // Drop the oldest keyframe from the DBoW database.
        const auto& old = kfs_.front();
        if (old->dbow && old->dbow->entry >= 0)
            db_.deleteEntry(old->dbow->entry);
        kfs_.erase(kfs_.begin());

        // Keyframe ids are dense (id == index into kfs_); rebase the remaining
        // keyframes and loop edges after removing the first one.
        for (auto& k : kfs_) {
            k->id -= 1;
            // loop_index is also an id into kfs_: keep it aligned after the
            // front keyframe is pruned; a loop to the pruned keyframe (id 0)
            // is no longer usable.
            if (k->loop_index == 0) {
                k->has_loop = false;
                k->loop_index = -1;
            } else if (k->loop_index > 0) {
                k->loop_index -= 1;
            }
        }
        db_.rebaseKfIds(1);

        std::vector<std::tuple<int, int, Eigen::Matrix4d>> kept;
        kept.reserve(loop_edges_.size());
        for (const auto& e : loop_edges_) {
            int i = std::get<0>(e) - 1;
            int j = std::get<1>(e) - 1;
            if (i < 0 || j < 0)
                continue;   // edge references the pruned keyframe
            kept.emplace_back(i, j, std::get<2>(e));
        }
        loop_edges_.swap(kept);
    }
}

} // namespace hwa_pg
