#ifndef HWA_PG_TYPES_H
#define HWA_PG_TYPES_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>

namespace hwa_pg {

struct PGKeyFrameDBoW;   // opaque DBoW2/BRIEF payload (defined in hwa_pg_database.cpp)

// Keyframe used by the pose graph / loop closure module.
// Follows the VINS-Mono KeyFrame layout:
//   - FAST corners + BRIEF descriptors (computed inside the pose-graph thread),
//   - tracked feature points with 3D landmarks (ECEF) for PnP verification,
//   - window BRIEF descriptors at tracked feature locations for matching.
struct PGKeyFrame {
    int id = -1;                       // sequential keyframe id
    double time = 0.0;                 // image time (seconds)

    // VIO pose (same convention as the FGO sliding window):
    //   rot : body -> ECEF (C_b^e),  pos : body origin in ECEF
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    // Raw VIO pose snapshot at keyframe birth; never overwritten by
    // optimizeGraph. VINS keeps vio_R_w_i/vio_T_w_i separately so the 4DoF
    // pose-graph optimization can always be re-run from the original VIO
    // measurements (rot/pos hold the optimized "global" pose).
    Eigen::Matrix3d vio_rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d vio_pos = Eigen::Vector3d::Zero();

    cv::Mat image;                     // gray image (released after BRIEF extraction)

    // FAST corners + normalized coordinates (VINS computeBRIEFPoint)
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::KeyPoint> keypoints_norm;

    // tracked feature locations used for window BRIEF matching
    std::vector<cv::KeyPoint> window_keypoints;

    // 3D landmarks (ECEF) + 2D observations of the tracked features
    std::vector<cv::Point3f> point_3d;
    std::vector<cv::Point2f> point_2d_uv;
    std::vector<cv::Point2f> point_2d_norm;
    std::vector<double> point_id;

    int loop_index = -1;               // matched loop keyframe id
    bool has_loop = false;
    int loop_inliers = 0;
    double loop_score = 0.0;
    Eigen::Matrix<double, 8, 1> loop_info = Eigen::Matrix<double, 8, 1>::Zero();

    // BRIEF descriptors / BoW (opaque, keeps boost out of the FGO TUs)
    std::shared_ptr<PGKeyFrameDBoW> dbow;

    Eigen::Matrix4d poseSE3() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rot;
        T.block<3, 1>(0, 3) = pos;
        return T;
    }
};

// Raw data handed from the FGO to the pose-graph worker thread.
// The worker performs FAST + BRIEF extraction and loop detection asynchronously.
struct RawKeyFrame {
    double time = 0.0;
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    std::string image_path;            // gray image path (read inside the worker)

    std::vector<cv::Point3f> point_3d;      // ECEF
    std::vector<cv::Point2f> point_2d_uv;   // pixel
    std::vector<cv::Point2f> point_2d_norm; // normalized
    std::vector<double> point_id;
};

struct PGLoopResult {
    bool found = false;
    int cur_kf_id = -1;
    int loop_kf_id = -1;
    int inliers = 0;
    // relative body transform: maps loop body frame -> current body frame
    Eigen::Matrix4d T_cur_loop = Eigen::Matrix4d::Identity();
    // 3D-2D matches for relocalization: p_world(ECEF) -> normalized obs in current keyframe
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>> relo_3d2d;
};

struct PGConfig {
    // BRIEF / FAST (VINS-Mono defaults)
    std::string brief_pattern;         // yml pattern file (compatible with the vocabulary)
    int fast_threshold = 20;

    // DBoW2 settings (mirrors VINS-Mono pose graph defaults)
    std::string vocabulary_path;       // text ORBvoc.txt or binary .bin; empty -> train at runtime
    int vocab_k = 10;
    int vocab_L = 6;
    int vocab_train_frames = 50;
    int dbow_top = 4;
    int dbow_min_id_gap = 50;
    int min_loop_num = 25;             // VINS MIN_LOOP_NUM (matches + PnP inliers)

    // loop acceptance / sanity checks (VINS)
    double min_loop_score = 0.015;
    double max_relative_yaw_deg = 30.0;
    double max_relative_t = 20.0;
    double min_loop_time_gap = 15.0;
    int max_loop_distance = 30;

    // keyframe insertion thinning (mirrors VINS pose_graph_node:
    // SKIP_FIRST_CNT + SKIP_DIS). Without it the pose graph database grows at
    // the FGO marginalization rate and DBoW2's inverted-file query becomes
    // quadratic, which makes the worker appear stuck in db.query().
    int skip_first_cnt = 10;              // ignore the first few keyframes
    double min_keyframe_disp = 0.15;      // min translation (m) from last PG keyframe
    double min_keyframe_yaw_deg = 5.0;    // min yaw change (deg) from last PG keyframe
    size_t max_raw_queue = 32;            // bound the worker backlog (drop oldest)
    // Bound the retained pose-graph keyframe window (and the DBoW database).
    // Once full, the oldest keyframes are pruned so DBoW2's full inverted-file
    // query and the pose-graph optimization stay bounded in cost (otherwise the
    // worker thread falls permanently behind and appears stuck in db.query()).
    size_t max_pg_keyframes = 5000;       // sliding window: drop oldest beyond this

    bool enable_posegraph_opt = true;


    std::string _cam0_distortion_model;            ///<  distortion model of camera
    cv::Vec4d _cam0_intrinsics;                ///<  intrinsics of camera
    cv::Vec4d _cam0_distortion_coeffs;        ///<  distortion coeffs of camera
    // camera model (pinhole): normalized coords = (pt - c)/f
    double fx = 1.0, fy = 1.0, cx = 0.0, cy = 0.0;
    // camera -> body extrinsic (same as R_cam0_imu / t_cam0_imu)
    Eigen::Matrix3d R_cb = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cb = Eigen::Vector3d::Zero();
};

} // namespace hwa_pg

#endif
