#ifndef hwa_fgo_zuptprocesser_h
#define hwa_fgo_zuptprocesser_h

#include "hwa_set_base.h"
#include "hwa_base_filter.h"
#include "hwa_base_eigendef.h"
#include "hwa_fgo_baseprocesser.h"

using namespace hwa_base;

namespace hwa_fgo {
	class zuptprocesser : public baseprocesser {
	public:
		explicit zuptprocesser(const baseprocesser& B) : baseprocesser(B) {};
		void insert(base_time time);
		bool align_track() { return true; };
		void _addResidualBlocks(ceres::Problem& problem) override;
		void _addMarginInfo() override;
		void slide_window() override {}
		void _fgo_vector_to_double() override {};
		void _fgo_double_to_vector() override {};
		int ProcessOneEpoch() override;

	private:
	};
}



#endif