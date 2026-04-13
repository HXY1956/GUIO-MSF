#ifndef hwa_vis_coder_h
#define hwa_vis_coder_h

#include "hwa_set_vis.h"
#include "hwa_base_coder.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis
{
    class vis_coder : virtual public hwa_base::base_coder
    {
    public:
        vis_coder(hwa_set::set_base* _gset, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);
        ~vis_coder() {}
        int decode_head(char* buff, int sz, std::vector<std::string>& errmsg) override;
        int decode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg) override;
    private:
        double _ts = 0;                     ///< data interval
    };
}
#endif