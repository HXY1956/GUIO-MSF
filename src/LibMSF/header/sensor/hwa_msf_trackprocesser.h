#ifndef hwa_msf_trackprocesser_h
#define hwa_msf_trackprocesser_h

#include "hwa_set_base.h"
#include "hwa_base_filter.h"
#include "hwa_base_eigendef.h"
#include "hwa_vis_Tracker_Circle.h"
#include "hwa_msf_baseprocesser.h"

using namespace hwa_vis;
using namespace hwa_set;

namespace hwa_msf {
	class trackprocesser : public baseprocesser {
	public:
		explicit trackprocesser(const baseprocesser& B) : baseprocesser(B) {};
		explicit trackprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME) : baseprocesser(gset, spdlog, site, _beg, _end) {};
		bool align_track() { return true; };
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