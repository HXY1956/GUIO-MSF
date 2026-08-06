#include "hwa_fgo_zuptprocesser.h"

using namespace std;

namespace hwa_fgo {

    void zuptprocesser::insert(base_time time) {
        Eigen::Quaterniond e_q = Eigen::Quaterniond(_sins->qeb.q0, _sins->qeb.q1, _sins->qeb.q2, _sins->qeb.q3);
        _fgo_info->_map_motion.insert(make_pair(time, MOTION_TYPE::m_static));
        _fgo_info->_map_pose.insert(make_pair(time, make_pair(_sins->pos_ecef, e_q)));
    }

    void zuptprocesser::_addResidualBlocks(ceres::Problem& problem) {
    }

    void zuptprocesser::_addMarginInfo() {

        if (!_fgo_info->time_to_margin()) return;

        base_time crt = base_time(TimeStamp.gwk(), _fgo_info->_Time[0]);
        std::map<base_time, MOTION_TYPE>::const_iterator it = _fgo_info->_map_motion.find(crt);
        if (it->second == MOTION_TYPE::m_static)
        {
            for (it; it != _fgo_info->_map_motion.begin(); --it)
            {
                if (it->second != MOTION_TYPE::m_static)
                {
                    ++it;
                    break;
                }
            }
            std::map<base_time, pair<Eigen::Vector3d, Eigen::Quaterniond>>::const_iterator it_pose = _fgo_info->_map_pose.find(it->first);
            Eigen::Vector3d pos = it_pose->second.first;
            Eigen::Quaterniond quat = it_pose->second.second;
            InitialPoseFactor* pose_factor = new InitialPoseFactor(pos, quat);
            pose_factor->sqrt_info = 1e4 * Eigen::Matrix<double, 6, 6>::Identity();
            ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(pose_factor, NULL,
                vector<double*>{_fgo_info->_para_pose[0]}, vector<int>{0});
            _fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
        }
    }

    int zuptprocesser::ProcessOneEpoch() {
        return 1;
    }
}

