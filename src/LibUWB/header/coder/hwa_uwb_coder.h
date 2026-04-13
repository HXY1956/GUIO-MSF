#ifndef hwa_uwb_coder_h
#define hwa_uwb_coder_h
#include "hwa_set_uwb.h"
#include "hwa_base_coder.h"
#include "hwa_base_time.h"
#include "hwa_base_roughcoder.h"
#include "hwa_base_eigendef.h"
#include "hwa_uwb_data.h"

using namespace hwa_base;

namespace hwa_uwb {
    class uwb_coder : virtual public hwa_base::base_coder
    {
    public:
        uwb_coder(hwa_set::set_base* s, int sz);
        ~uwb_coder() {}
        int decode_head(char* buff, int sz, std::vector<std::string>& errmsg) override;
        int decode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg) override;
        int decode_rangenet(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);
        int decode_tdis(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);
        int decode_nooploop(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);
        int decode_viral(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);
        int decode_simulation(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);

    private:
        double _tt;
        //for decode_data in different times, Zongzhou Wu
        int _year, _mon, _day, _hour, _mins;
        double _secs;
        base_time _t;
        std::string _order;
        std::string _node_crt;
    };
}

#endif // !hwa_uwb_coder_H