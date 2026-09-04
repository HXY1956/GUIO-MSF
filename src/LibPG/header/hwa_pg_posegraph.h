#ifndef HWA_PG_POSEGRAPH_H
#define HWA_PG_POSEGRAPH_H

#include "hwa_pg_types.h"
#include "hwa_pg_database.h"
#include <Eigen/Dense>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <tuple>
#include <vector>

namespace hwa_pg {

// Pose graph with loop closure detection and fast relocalization support.
//
// Follows the VINS-Mono architecture:
//   - BRIEF descriptors (VINS BriefExtractor: FAST + DVision::BRIEF),
//   - DBoW2 query-then-add loop detection,
//   - descriptor matching + PnP RANSAC verification (VINS KeyFrame::findConnection),
//   - pose graph optimization,
// all executed in a dedicated worker thread so the FGO main thread is never
// blocked. The relocalization result is published under a mutex and can be
// polled by the sliding window.
class PGPoseGraph {
public:
    explicit PGPoseGraph(const PGConfig& cfg = PGConfig());
    ~PGPoseGraph();
    PGPoseGraph(const PGPoseGraph&) = delete;
    PGPoseGraph& operator=(const PGPoseGraph&) = delete;

    void setCamera(const Eigen::Matrix3d& R_cb, const Eigen::Vector3d& t_cb,
        Eigen::Vector4d cam_instrinsics, Eigen::Vector4d cam_distortion_coeffs, std::string cam_distortion_model);
    void setVocabularyPath(const std::string& p) { cfg_.vocabulary_path = p; }
    void setBriefPattern(const std::string& p) { cfg_.brief_pattern = p; }
    // VINS-style keyframe thinning: skip the first `skip_first` keyframes, then
    // only accept a keyframe when translation >= min_disp (m) or yaw change
    // >= min_yaw_deg (deg) from the last accepted pose-graph keyframe.
    void setKeyframeThinning(int skip_first, double min_disp, double min_yaw_deg);
    // Cap the pending raw-keyframe queue (drop oldest when full).
    void setMaxRawQueue(size_t n) { cfg_.max_raw_queue = n; }
    // Bound the retained pose-graph keyframes (and DBoW entries). Oldest
    // keyframes are pruned once the window is full so db.query() and
    // optimizeGraph() stay bounded in cost.
    void setMaxPoseGraphSize(size_t n) { cfg_.max_pg_keyframes = n; }

    void undistortPoints(
        const std::vector<cv::KeyPoint>& pts_in,
        const cv::Vec4d& intrinsics,
        const std::string& distortion_model,
        const cv::Vec4d& distortion_coeffs,
        std::vector<cv::KeyPoint>& pts_out,
        const cv::Matx33d& rectification_matrix = cv::Matx33d::eye(),
        const cv::Vec4d& new_intrinsics = cv::Vec4d(1, 1, 0, 0));
    // Push a raw keyframe; loop detection / BRIEF / optimization run in the
    // background worker thread. Non-blocking.
    void addKeyFrame(const RawKeyFrame& raw);

    // Latest relocalization result (if a loop was found recently).
    // VINS-Mono FAST_RELOCALIZATION semantics: the old (loop) keyframe's
    // normalized observations + feature ids, plus the old keyframe pose in the
    // world frame (ECEF here). The FGO side anchors the sliding-window feature
    // structure to these observations via a reprojection factor.
    bool getReloResult(double& time, Eigen::Vector3d& old_pos, Eigen::Matrix3d& old_rot,
                       std::vector<std::pair<double, Eigen::Vector2d>>& obs) const;

    // Two-phase relocalization handshake: the worker announces a detected loop
    // BEFORE running the (slow) 4DoF pose-graph optimization and publishes the
    // corrected old-frame anchor only afterwards. The FGO main thread calls
    // isReloPending()/waitForReloReady() so the loop keyframe cannot slide out
    // of the sliding window while the graph is being optimized.
    bool isReloPending() const;
    // Wait up to timeout_ms (<=0: forever) until the pending anchor is
    // published. Returns true when a relocalization result is available.
    bool waitForReloReady(int timeout_ms) const;

    // VIO(raw) -> PG(corrected) 4DoF drift (world-z yaw rotation + translation),
    // recomputed after every 4DoF optimizeGraph from the newest keyframe
    // (corrected pos/rot vs its raw vio pos/rot). Newly inserted keyframes are
    // initialized in the corrected map frame with this drift. Returns false
    // (identity) before the first loop optimization.
    bool getDrift(double& yaw_deg, Eigen::Vector3d& t) const;

    size_t size() const;

    // Thread-safe snapshot of the current pose-graph state for visualization:
    //   positions  - current optimized keyframe positions (kf->pos, order ==
    //                kfs_ order; these are updated by optimizeGraph),
    //   loop_pairs - loop-closure pairs as keyframe ids (indices into the
    //                returned position list).
    // The GLFW renderer redraws the whole pose-graph path and all loop lines
    // from this snapshot every frame, so poses corrected by the 4DoF
    // optimization automatically move the already-drawn trajectory.
    bool getPoseGraphSnapshot(std::vector<Eigen::Vector3d>& positions,
                              std::vector<std::pair<int, int>>& loop_pairs) const;

    // Program-end final dump: stops/drains the background worker (idempotent)
    // and returns the FINAL optimized pose-graph trajectory as (keyframe time,
    // optimized ECEF position) pairs, in keyframe order.
    bool getPoseGraphFinalTrajectory(std::vector<std::pair<double, Eigen::Vector3d>>& out);

private:
    PGConfig cfg_;
    PGDatabase db_;
    std::vector<std::shared_ptr<PGKeyFrame>> kfs_;
    std::vector<std::tuple<int, int, Eigen::Matrix4d>> loop_edges_;   // (i, j, T_i_j)

    // worker thread
    std::thread worker_;
    mutable std::mutex m_queue_;
    std::condition_variable cv_;
    std::queue<RawKeyFrame> raw_queue_;
    bool stop_ = false;
    bool worker_stopped_ = false;
    bool vocab_attempted_ = false;
    int skip_first_done_ = 0;
    bool last_pg_pose_valid_ = false;
    Eigen::Vector3d last_pg_pos_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d last_pg_rot_ = Eigen::Matrix3d::Identity();

    // relocalization result (published by the worker)
    mutable std::mutex m_relo_;
    mutable std::condition_variable relo_cv_;
    bool relo_pending_ = false;         // loop found, anchor not published yet
    double relo_time_ = 0.0;
    Eigen::Vector3d relo_old_pos_ = Eigen::Vector3d::Zero();   // old kf pose (ECEF)
    Eigen::Matrix3d relo_old_rot_ = Eigen::Matrix3d::Identity();
    std::vector<std::pair<double, Eigen::Vector2d>> relo_obs_; // (feature id, old kf normalized obs)
    bool relo_found_ = false;

    // raw -> corrected drift (worker-maintained under m_queue_)
    bool drift_valid_ = false;
    double drift_yaw_deg_ = 0.0;
    Eigen::Vector3d drift_t_ = Eigen::Vector3d::Zero();

    // Fixed session-local ENU frame (z = local up at the first accepted
    // keyframe). The 4DoF yaw and the raw->corrected drift are expressed in
    // this frame so "yaw" is a rotation about local up, not about the ECEF
    // z-axis (which is what VINS-Mono assumes and what makes 4DoF valid).
    bool local_frame_init_ = false;
    Eigen::Vector3d local_p0_ecef_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d local_C_e_ = Eigen::Matrix3d::Identity();   // ECEF -> local

    void workerLoop();
    // Stop the worker thread (drains the pending queue). Idempotent.
    void stopWorker();
    // Apply the current raw->corrected drift to a raw VIO pose (worker only).
    void applyPoseDrift(Eigen::Matrix3d& R, Eigen::Vector3d& p) const;

    // VINS KeyFrame construction: FAST corners + BRIEF, window BRIEF at tracked
    // features, normalized coordinates.
    std::shared_ptr<PGKeyFrame> buildKeyFrame(const RawKeyFrame& raw);

    // VINS PoseGraph::detectLoop: query then add, filter candidates.
    int detectLoop(const std::shared_ptr<PGKeyFrame>& kf);

    // VINS KeyFrame::findConnection: window-BRIEF matching + PnP RANSAC.
    bool findConnection(const std::shared_ptr<PGKeyFrame>& cur,
                        const std::shared_ptr<PGKeyFrame>& old_kf);

    // Publish the final relocalization result with the old-frame pose that was
    // just corrected by optimizeGraph (worker thread only).
    void publishReloResult(const std::shared_ptr<PGKeyFrame>& old_kf);

    // 6DOF SE3 pose graph optimization (first node fixed).
    void optimizeGraph();

    // Drop the oldest keyframes (and their DBoW entries) once the pose-graph
    // window exceeds max_pg_keyframes. Keeps keyframe ids dense and aligned
    // with the kfs_ vector. Worker thread only.
    void pruneKeyframes();
};

} // namespace hwa_pg

#endif
