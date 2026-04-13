#include "hwa_gnss_all_cycleslip.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_all_cycleslip::gnss_all_cycleslip()
    {
    }

    hwa_gnss::gnss_all_cycleslip::gnss_all_cycleslip(set_base *set, base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
        if (nullptr == set)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _set = set;
        }
    }

    hwa_gnss::gnss_all_cycleslip::~gnss_all_cycleslip()
    {
    }

    void hwa_gnss::gnss_all_cycleslip::read_amb_info_files(const FREQ_SEQ &freq_1, const FREQ_SEQ &freq_2, const std::vector<std::string> &_amb_info_files, gnss_all_ambflag *_all_ambflag)
    {
    }
}
