#include "hwa_fgo_margprocesser.h"

using namespace std;

namespace hwa_fgo {

    void margprocesser::reset() {
        _fgo_info->marginalization_info = new MarginalizationInfo();
        _fgo_info->addr_shift.clear();
    }

    void margprocesser::_addResidualBlocks(ceres::Problem& problem) {
        _fgo_info->cost = 0;
        std::vector<double> residuals;
        if (_fgo_info->_last_marginalization_info && _fgo_info->_last_marginalization_info->valid)
        {
            MarginalizationFactor* marginalization_factor = new MarginalizationFactor(_fgo_info->_last_marginalization_info);
            problem.AddResidualBlock(marginalization_factor, NULL, _fgo_info->_last_marginalization_parameter_blocks);

            problem.Evaluate(
                ceres::Problem::EvaluateOptions(),
                &_fgo_info->cost,
                &residuals,
                nullptr,
                nullptr);

            std::cout
                << std::fixed 
                << std::setprecision(6)
                << "marginalization cost: "
                << _fgo_info->cost
                << std::endl;
        }
    }

    void margprocesser::_addMarginInfo() {

        if (!_fgo_info->time_to_margin()) return;

        if (_fgo_info->_last_marginalization_info && _fgo_info->_last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(_fgo_info->_last_marginalization_parameter_blocks.size()); i++)
            {
                if (_fgo_info->_last_marginalization_parameter_blocks[i] == _fgo_info->_para_pose[0] ||
                    _fgo_info->_last_marginalization_parameter_blocks[i] == _fgo_info->_para_speed_bias[0])
                    drop_set.push_back(i);
            }
            MarginalizationFactor* marginalization_factor = new MarginalizationFactor(_fgo_info->_last_marginalization_info);
            ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                _fgo_info->_last_marginalization_parameter_blocks,
                drop_set);
            _fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
        }
    }

    int margprocesser::ProcessOneEpoch() {

        _fgo_info->marginalization_info->preMarginalize();

        _fgo_info->marginalization_info->marginalize();

        vector<double*> parameter_blocks = _fgo_info->marginalization_info->getParameterBlocks(_fgo_info->addr_shift); /// reserved parameters
        _fgo_info->_last_marginalization_parameter_blocks = parameter_blocks;

        if (_fgo_info->_last_marginalization_info) {
            delete _fgo_info->_last_marginalization_info;
			_fgo_info->_last_marginalization_info = nullptr;
        }
            
        _fgo_info->_last_marginalization_info = _fgo_info->marginalization_info;

        if (_fgo_info->_last_marginalization_info->factors.size() == 0)
            _fgo_info->_last_marginalization_info->valid = false;

        return 1;
    }
}

