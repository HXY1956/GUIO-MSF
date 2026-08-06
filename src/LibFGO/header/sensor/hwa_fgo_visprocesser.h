#ifndef hwa_fgo_vis_processer_h
#define hwa_fgo_vis_processer_h
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
        bool addFeatureObservations() override;
        int ProcessOneEpoch() override;
        void AddData(base_data* data) override { imgdata = dynamic_cast<vis_data*>(data); };
        bool _init() override { return false; };
		void _addResidualBlocks(ceres::Problem& problem) override;
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
                feature_index++;
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
                feature_index++;
				_fgo_info->_para_feature[feature_index][0] = feature.inv_depth;
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
        SO3 ric[NUM_OF_CAM];
        Triple tic[NUM_OF_CAM];   // transform form camera to imu

        vis_data* imgdata = nullptr;
        Triple initCamPos;
        std::string _site;
        base_iof* _fcalib = nullptr;
        int cam_group_id;
        bool mIsFirstImg;
        int imu_frequency;
        int _imgproc_count = 0;
        std::ofstream TimeCostDebugOutFile;
        bool TimeCostDebugStatus = false;
		bool time_lock = false;
    };
}
#endif
