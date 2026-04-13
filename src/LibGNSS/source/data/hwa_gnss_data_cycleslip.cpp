#include "hwa_gnss_data_cycleslip.h"

hwa_gnss::gnss_data_cycleslip::gnss_data_cycleslip()
{
}

hwa_gnss::gnss_data_cycleslip::gnss_data_cycleslip(set_base *set, base_log spdlog)
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

    _slip_model = dynamic_cast<set_gproc *>(set)->slip_model();
}

hwa_gnss::gnss_data_cycleslip::~gnss_data_cycleslip()
{
}
