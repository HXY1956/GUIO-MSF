#include "hwa_fgo_nhcprocesser.h"

using namespace std;

namespace hwa_fgo {

    void nhcprocesser::insert(base_time time) {
        _fgo_info->_map_motion.insert(make_pair(time, MOTION_TYPE::m_straight));
    }

    void nhcprocesser::_addResidualBlocks(ceres::Problem& problem) {
    }

    void nhcprocesser::_addMarginInfo() {

    }

    int nhcprocesser::ProcessOneEpoch() {
        return 1;
    }
}

