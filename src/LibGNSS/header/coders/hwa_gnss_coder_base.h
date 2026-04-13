#ifndef hwa_gnss_coder_BASE_H
#define hwa_gnss_coder_BASE_H

#define BUFFER_INCREASE_FAC 1.5
#define DEFAULT_BUFFER_SIZE 4096
#define MAXIMUM_BUFFER_SIZE 240000 // 9000000

#include "hwa_base_log.h"
#include "hwa_base_coder.h"
#include "hwa_set_gtype.h"
#include "hwa_set_gen.h"
#include "hwa_set_gbase.h"

using namespace hwa_set;
using namespace hwa_base;
using namespace spdlog;

namespace hwa_gnss
{
    /**
    *@brief       Class for decoding
    */
    class gnss_base_coder : virtual public base_coder
    {
    public:

        gnss_base_coder();
        explicit gnss_base_coder(hwa_set::set_base* s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE, std::string id = "gcoder");
        gnss_base_coder(base_time beg, base_time end, hwa_set::set_base* s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE, std::string id = "gcoder");
        explicit gnss_base_coder(hwa_set::set_base* s, int sz = DEFAULT_BUFFER_SIZE);
        virtual ~gnss_base_coder();

        int _gset(set_base* s) override;
        typedef std::map<int, std::pair<GOBS, int>> hwa_vector_obs; ///<        std::map

    protected:
        bool _filter_gnss(const std::string& prn);
        bool _filter_gnss_addleo(const std::string& prn); //add leo

        std::map<GSYS, std::set<std::string>> _sat; ///< default satellites
        std::map<GSYS, std::set<std::string>> _obs; ///< default observations (all mixed for single GNSS !!)
        std::map<GSYS, std::set<std::string>> _nav; ///< default navigation messages
        set_base* _set;             ///< set pointer

    private:
    };

} // namespace

#endif
