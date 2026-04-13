#include "hwa_gnss_model.h"

namespace hwa_gnss
{
    gnss_model::gnss_model()
    {
    }

    gnss_model::gnss_model(base_log spdlog)
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
    }
    // Destructor
    // -----------------
    gnss_model::~gnss_model()
    {
    }

    void gnss_model::spdlog(base_log spdlog)
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
    }
    // Set site name
    // -----------
    void gnss_model::setSite(const std::string &site)
    {
        this->_site = site;
    }

} // namespace
