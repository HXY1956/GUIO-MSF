#ifndef hwa_msf_client_h
#define hwa_msf_client_h

#include "hwa_set_all.h"
#include "hwa_set_proc.h"
#include "hwa_base_allproc.h"
#include "hwa_msf_baseprocesser.h"
#include "hwa_msf_insprocesser.h"
#include "hwa_msf_visprocesser.h"
#include "hwa_msf_uwbprocesser.h"
#include "hwa_msf_trackprocesser.h"
#include "hwa_msf_gnssprocesser.h"
#include "hwa_msf_lidarprocesser.h"

namespace hwa_msf
{
    class msf_client{
    public:
        explicit msf_client(std::string site, std::string site_base, base_time beg, base_time end, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* data);
        ~msf_client() {};

        msf_client(const msf_client&) = delete;
        msf_client& operator=(const msf_client&) = delete;
        msf_client(msf_client&&) = default;
        msf_client& operator=(msf_client&&) = default;

        int ProcessBatchFB();
        bool cascaded_align(Triple pos, Triple vel, MEAS_TYPE _Flag);
        int _init();
        void PreTimeSynchronization();
        bool align_process();
        bool _getMeas();
        void merge_init();
        void crt_feed_back();
        void write2file();

    protected:
        baseprocesser baseworker;
        std::unique_ptr<gnssprocesser> gnssworker;
        std::unique_ptr<insprocesser> insworker;
        std::unique_ptr<uwbprocesser> uwbworker;
        std::unique_ptr<trackprocesser> trackworker;
        std::unique_ptr<lidarprocesser> lidarworker;
        std::map<int, std::unique_ptr<visprocesser>> visworker;

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
        int irc;
        START_ENV startenv;
        MEAS_INFO measinfo;
        ALIGN_TYPE align_type;
        base_posdata::data_pos posdata;
        std::set<MEAS_TYPE> _Meas_Type;
    };
}

#endif