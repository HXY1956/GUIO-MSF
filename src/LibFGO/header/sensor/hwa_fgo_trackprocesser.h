#ifndef hwa_fgo_trackprocesser_h
#define hwa_fgo_trackprocesser_h

#include "hwa_set_base.h"
#include "hwa_base_filter.h"
#include "hwa_base_eigendef.h"
#include "hwa_vis_Tracker_Circle.h"
#include "hwa_fgo_baseprocesser.h"

using namespace hwa_vis;
using namespace hwa_set;

namespace hwa_fgo {
	class trackprocesser : public baseprocesser {
	public:
		explicit trackprocesser(const baseprocesser& B) : baseprocesser(B) {};
		explicit trackprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME) : baseprocesser(gset, spdlog, site, _beg, _end) {};
		bool align_track() { return true; };
		void _addResidualBlocks(ceres::Problem& problem) override {};
		void _addMarginInfo() override {};
		void slide_window() override {}
		void _fgo_vector_to_double() override {};
		void _fgo_double_to_vector() override {};
		int ProcessOneEpoch() override { return 0; };

	private:
		vis_circle_tracker Tracker;
		Matrix R_t_i;
		Vector t_t_i;
		hwa_base::base_updater updater;
		double Tracker_Rot_noise;
		double Tracker_Trans_noise;
	};
}



#endif