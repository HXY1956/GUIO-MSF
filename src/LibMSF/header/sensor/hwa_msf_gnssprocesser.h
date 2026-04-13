#ifndef hwa_msf_gnss_processer_h
#define hwa_msf_gnss_processer_h

#include "hwa_set_ign.h"
#include "hwa_base_allproc.h"
#include "hwa_msf_baseprocesser.h"
#include "hwa_gnss_proc_lsq.h"
#include "hwa_gnss_proc_pppflt.h"
#include "hwa_gnss_proc_pvtflt.h"

using namespace hwa_gnss;

namespace hwa_msf {
    class gnssprocesser : public baseprocesser, public gnss_proc_pvtflt {
    public:
        explicit gnssprocesser(std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
        explicit gnssprocesser(const baseprocesser& B, std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc);
        bool _init() override;
        void timesynchronization(base_time t) override;
        int ProcessOneEpoch() override;
        void _feed_back() override;
        bool _time_valid(base_time instime) override;
        MEAS_TYPE _getPOS(base_time inst, base_posdata::data_pos& pos, MEAS_INFO& m) override;
        bool load_data() override;
        const Triple get_lever() { return lever; }
        void _prt_port(base_time instime);
        int _prt_ins_kml(base_time instime);
        void valid_ins_constraint();
        int _merge(Matrix& A);
        double get_pdop() { return _dop.pdop(); }
        Triple get_site_pos() {
            Triple gnss_pos;
            _param.getCrdParam(_site, gnss_pos);
            return gnss_pos; 
        }
        std::set<std::string> ambs_name(){ return _param.amb_prns(); }

    private:
        Triple lever;
    };
}

#endif
