#ifndef hwa_fgo_vis_processer_h
#define hwa_fgo_vis_processer_h

// DBoW2 / boost headers must be processed before windows.h (hwa_set_base.h)
#include "hwa_pg_posegraph.h"
#include "hwa_pg_relo_factor.h"

#include "hwa_set_base.h"
#include "hwa_set_proc.h"
#include "hwa_base_Time.h"
#include "hwa_base_TimeCost.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_filter.h"
#include "hwa_base_allpar.h"
#include "hwa_base_iof.h"
#include "hwa_base_eigendef.h"
#include "hwa_vis_coder_stereousb.h"
#include "hwa_vis_base.h"
#include "hwa_vis_data.h"
#include "hwa_vis_coder.h"
#include "hwa_fgo_baseprocesser.h"
#include "hwa_fgo_factor_feature_projection.h"
#include "hwa_fgo_factor_feature_projection_td.h"

using namespace hwa_vis;
#define NUM_OF_CAM 2

namespace hwa_pg {
    struct PGKeyFrame;
    struct RawKeyFrame;
}

namespace hwa_fgo {
    class visprocesser : public baseprocesser, public vis_base {
    public:
        explicit visprocesser(const baseprocesser& B, int ID, base_data* data = nullptr);
        explicit visprocesser(std::shared_ptr<set_base> gset, std::string site, int ID, base_log spdlog = nullptr, base_data* data = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
        ~visprocesser();

        void _write_calib();
        bool align_vins();
        void align_feedback();
        void load_imuobs();
        void initializeDepth();
        void resetDepth();
        // Build raw keyframe data (image path + tracked feature 2D/3D) from the
        // latest optimized window state. BRIEF extraction runs in the worker.
        hwa_pg::RawKeyFrame buildRawKeyFrame();
        // Current FGO node index of the latest keyframe (-1 if none).
        int currentPGNodeIndex() const;
        // Insert the latest optimized keyframe into the pose graph; on loop
        // detection this arms the relocalization factors for the next solve.
        void updatePoseGraph();
        // Thread-safe snapshot of the pose graph for visualization: current
        // optimized keyframe positions (ECEF) + loop pairs (keyframe ids).
        // Reads directly from the pose-graph worker's state under its mutex.
        bool getPoseGraphPath(std::vector<Eigen::Vector3d>& positions,
                              std::vector<std::pair<int, int>>& loop_pairs) const
        {
            if (!posegraph_) return false;
            return posegraph_->getPoseGraphSnapshot(positions, loop_pairs);
        }
        // Program-end dump: drain the pose-graph worker and return the final
        // optimized keyframe trajectory as (time, ECEF pos) pairs.
        bool getPoseGraphFinalTrajectory(std::vector<std::pair<double, Eigen::Vector3d>>& out)
        {
            if (!posegraph_) return false;
            return posegraph_->getPoseGraphFinalTrajectory(out);
        }
        // Latest VIO(raw) -> PG(corrected) 4DoF drift for propagating the pose
        // graph correction to live outputs. False (identity) before the first
        // loop optimization.
        bool getPoseGraphDrift(double& yaw_deg, Eigen::Vector3d& t) const
        {
            if (!posegraph_) return false;
            return posegraph_->getDrift(yaw_deg, t);
        }
        bool addFeatureObservations() override;
        int ProcessOneEpoch() override;
        void AddData(base_data* data) override { imgdata = dynamic_cast<vis_data*>(data); };
        bool _init() override { return false; };
		void _addResidualBlocks(ceres::Problem& problem) override;
        void _addRelocalizationFactors(ceres::Problem& problem);
        void _addMarginInfo() override;
        void slide_window() override;
        bool _time_valid(base_time inst) override;
        bool load_data() override;
        void _fgo_double_to_vector() override {
            int feature_index = -1;
            for (auto& it_per_id : map_server)
            {
                auto& feature = it_per_id.second;
                if (feature.inv_depth < 0) continue;

                int used_num = feature.observations.size();
                int start_frame = feature.start_frame;
                if (!(used_num >= 2 && start_frame < _fgo_info->_window_size - 3))
                    continue;

                feature_index++;
                if (feature_index >= 1000) break;
                feature.inv_depth = _fgo_info->_para_feature[feature_index][0];
            }
            tic[0].x() = _fgo_info->_para_ex_pose[0][0];
            tic[0].y() = _fgo_info->_para_ex_pose[0][1];
            tic[0].z() = _fgo_info->_para_ex_pose[0][2];
            Eigen::Quaterniond q;
            q.x() = _fgo_info->_para_ex_pose[0][3];
            q.y() = _fgo_info->_para_ex_pose[0][4];
            q.z() = _fgo_info->_para_ex_pose[0][5];
            q.w() = _fgo_info->_para_ex_pose[0][6];
            ric[0] = SO3(q);
        };
        void _fgo_vector_to_double() override {
            int feature_index = -1;
            for (auto& it_per_id : map_server)
            {
                auto& feature = it_per_id.second;
                if (feature.inv_depth < 0) continue;

                int used_num = feature.observations.size();
                int start_frame = feature.start_frame;
                if (!(used_num >= 2 && start_frame < _fgo_info->_window_size - 3))
                    continue;

                feature_index++;
                if (feature_index >= 1000) break;
				_fgo_info->_para_feature[feature_index][0] = feature.inv_depth;
                //std::cout << std::setprecision(15) << "feature inv_depth [" << feature_index << "]:" << _fgo_info->_para_feature[feature_index][0] << "\n";
            }
            _fgo_info->_para_ex_pose[0][0] = tic[0].x();
            _fgo_info->_para_ex_pose[0][1] = tic[0].y();
            _fgo_info->_para_ex_pose[0][2] = tic[0].z();
            Eigen::Quaterniond q(ric[0].matrix());
            _fgo_info->_para_ex_pose[0][3] = q.x();
            _fgo_info->_para_ex_pose[0][4] = q.y();
            _fgo_info->_para_ex_pose[0][5] = q.z();
            _fgo_info->_para_ex_pose[0][6] = q.w();
        };

    private:
        void _init_posegraph();
        SO3 ric[NUM_OF_CAM];
        Triple tic[NUM_OF_CAM];   // transform form camera to imu

        vis_data* imgdata = nullptr;
        Triple initCamPos;
        std::string _site;
        std::unique_ptr<hwa_pg::PGPoseGraph> posegraph_;
        bool relo_active_ = false;
        double relo_time_ = 0.0;
        Eigen::Vector3d relo_old_pos_ = Eigen::Vector3d::Zero();    // old (loop) kf pose in ECEF
        Eigen::Matrix3d relo_old_rot_ = Eigen::Matrix3d::Identity();
        std::vector<std::pair<double, Eigen::Vector2d>> relo_obs_;  // (feature id, old kf normalized obs)
        double relo_old_pose_[7] = {0, 0, 0, 0, 0, 0, 1};           // constant Ceres block for old kf pose
        base_iof* _fcalib = nullptr;
        int cam_group_id;
        bool mIsFirstImg;
        int imu_frequency;
        int _imgproc_count = 0;
        std::ofstream TimeCostDebugOutFile;
        bool TimeCostDebugStatus = false;
		bool time_lock = false;
        double _initial_last_time = 0;
    };
}
#endif
