#ifndef hwa_fgo_margprocesser_h
#define hwa_fgo_margprocesser_h

#include "hwa_set_base.h"
#include "hwa_base_eigendef.h"
#include "hwa_fgo_baseprocesser.h"

using namespace hwa_base;

namespace hwa_fgo {
	class margprocesser : public baseprocesser {
	public:
		explicit margprocesser(const baseprocesser& B) : baseprocesser(B) {};
		void reset();
		void _addResidualBlocks(ceres::Problem& problem) override;
		void _addMarginInfo() override;
		void _fgo_vector_to_double() override {};
		void _fgo_double_to_vector() override {};
		void slide_window() override {}
		int ProcessOneEpoch() override;
	};
}



#endif