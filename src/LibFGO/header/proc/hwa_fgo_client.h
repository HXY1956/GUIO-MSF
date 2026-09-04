#ifndef hwa_fgo_client_h
#define hwa_fgo_client_h

#include "hwa_set_all.h"
#include "hwa_set_proc.h"
#include "hwa_base_allproc.h"
#include "hwa_fgo_baseprocesser.h"
#include "hwa_fgo_insprocesser.h"
#include "hwa_fgo_visprocesser.h"
#include "hwa_fgo_uwbprocesser.h"
#include "hwa_fgo_trackprocesser.h"
#include "hwa_fgo_gnssprocesser.h"
#include "hwa_fgo_lidarprocesser.h"
#include "hwa_fgo_margprocesser.h"
#include "hwa_fgo_zuptprocesser.h"
#include "hwa_fgo_nhcprocesser.h"

namespace hwa_fgo
{
    class fgo_client{
    public:
        explicit fgo_client(std::string site, std::string site_base, base_time beg, base_time end, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* data);
        ~fgo_client() {};

        fgo_client(const fgo_client&) = delete;
        fgo_client& operator=(const fgo_client&) = delete;
        fgo_client(fgo_client&&) = default;
        fgo_client& operator=(fgo_client&&) = default;

        int ProcessBatchFB();
        bool cascaded_align(Triple pos, Triple vel, MEAS_TYPE _Flag);
        int _init();
        void PreTimeSynchronization();
        bool align_process();
        bool _getMeas();
        void merge_init();
        void optimization();
        void optimization_with_poterior();
		void _vector_to_double();
        void _double_to_vector();
        void marginalizaiton();
        void slide_window();
        void write2file();
        void writePoseGraphFinalFile();
        void prtState() const {
            baseworker.printSlidingWindowStates();
        }
        void feed_back() {
            base_posdata::data_pos _pos;
			_pos.pos = Triple::Zero();
            bool robustflag = false;

            if (UseGnss && isGNSSUpdate) {
                robustflag = gnssworker->_getRobustFixedPosition();
                _pos = gnssworker->get_posdata();
            }

            insworker->_feed_back(_pos, robustflag);
        }
        bool new_node() {
            return baseworker.new_node_inserted();
        }
        void reset() {
            baseworker.reset_status();
        }
        bool _time_to_margin() {
            return baseworker._time_to_margin();
        }

    protected:
        baseprocesser baseworker;
        std::unique_ptr<gnssprocesser> gnssworker = nullptr;
        std::unique_ptr<insprocesser> insworker = nullptr;
        std::unique_ptr<uwbprocesser> uwbworker = nullptr;
        std::unique_ptr<trackprocesser> trackworker = nullptr;
        std::unique_ptr<lidarprocesser> lidarworker = nullptr;
        std::map<int, std::unique_ptr<visprocesser>> visworker;
		std::unique_ptr<margprocesser> margworker = nullptr;
        std::unique_ptr<zuptprocesser> zuptworker = nullptr;
        std::unique_ptr<nhcprocesser> nhcworker = nullptr;
        std::vector<baseprocesser*> all_workers;

    private:
        base_log _spdlog;
        IGN_TYPE _ign_type;
        MEAS_TYPE Flag;
        bool UseLidar;
        bool UseGnss;
        bool UseVis;
        bool UseUwb;
        bool UseIns = true;
        bool UseNhc;
        bool UseOdo;
        bool UseZupt;
        bool UseHgt;
        bool _aligned = false;
        bool initial_merge = true;
        bool isGNSSUpdate = false;
        int irc;
        START_ENV startenv;
        MEAS_INFO measinfo;
        ALIGN_TYPE align_type;
        base_posdata::data_pos posdata;
        std::set<MEAS_TYPE> _Meas_Type;
    };
}

#endif
