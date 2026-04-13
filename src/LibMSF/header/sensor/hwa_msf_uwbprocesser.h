#ifndef hwa_msf_uwb_processer_h
#define hwa_msf_uwb_processer_h

#include "hwa_set_proc.h"
#include "hwa_set_uwb.h"
#include "hwa_base_Time.h"
#include "hwa_base_filter.h"
#include "hwa_base_TimeCost.h"
#include "hwa_base_posetrans.h"
#include "hwa_uwb_proc.h"
#include "hwa_uwb_data.h"
#include "hwa_uwb_coder.h"
#include "hwa_msf_baseprocesser.h"

using namespace hwa_uwb;

namespace hwa_msf{
    class uwbprocesser : public baseprocesser, public uwb_proc{
    public:
        explicit uwbprocesser(const baseprocesser& B, base_data* data = nullptr);
        explicit uwbprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog = nullptr, base_data* data = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
        ~uwbprocesser();
        int _setMeas();
        MEAS_TYPE _getPOS(base_time uwbt, base_posdata::data_pos& pos, MEAS_INFO& m) override;
        int ProcessOneEpoch() override;
        bool _init() override;
        bool _time_valid(base_time instime) override;
        void timesynchronization(base_time t) override;
        bool load_data() override;
        void _feed_back() override;
        const Triple get_lever() { return lever; }
        int get_anchor_number(){ return valid_node.size(); }
        Triple get_site_pos() { return _pos; }
        double get_pdop() { return uwb_pdop; }

    private:
        std::vector<std::string> _nlos;
        Triple lever;
        double uwb_pdop;
        std::map<std::string, std::ofstream> _outfile;
    };
}

#endif // DEBUG

