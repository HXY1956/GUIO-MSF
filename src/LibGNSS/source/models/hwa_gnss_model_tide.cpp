#include <stdlib.h>
#include <iostream>
#include <iomanip>

#include "hwa_base_globaltrans.h"
#include "hwa_gnss_model_tide.h"

namespace hwa_gnss
{
    gnss_model_tide::gnss_model_tide() : _gotl(nullptr)
    {
    }

    gnss_model_tide::gnss_model_tide(base_log spdlog) : _gotl(nullptr)
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

    // destructor
    // ----------
    gnss_model_tide::~gnss_model_tide() {}

    // solid base_earth tides
    // ----------
    Triple gnss_model_tide::tide_earth(const base_time &epoch, Triple &crd)
    {
        _mutex.lock();
        Triple dxyz(0.0, 0.0, 0.0);
        _mutex.unlock();
        return dxyz;
    }

    // pole tides
    // ----------
    Triple gnss_model_tide::tide_pole()
    {
        _mutex.lock();
        Triple dxyz(0.0, 0.0, 0.0);
        _mutex.unlock();
        return dxyz;
    }

    // ocean tide loading
    // ----------
    Triple gnss_model_tide::load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec)
    {
        _mutex.lock();
        Triple dxyz(0.0, 0.0, 0.0);
        _mutex.unlock();
        return dxyz;
    }

    // atmospheric tide loading
    // ----------
    Triple gnss_model_tide::load_atmosph()
    {
        _mutex.lock();
        Triple dxyz(0.0, 0.0, 0.0);
        _mutex.unlock();
        return dxyz;
    }

    void gnss_model_tide::spdlog(base_log spdlog)
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
    // set OTL pointer
    // -----------------
    void gnss_model_tide::setOTL(gnss_all_otl *gallotl)
    {
        _mutex.lock();
        _gotl = gallotl;
        _mutex.unlock();
        return;
    }

    //// set ATL pointer
    //// -----------------
    //void gnss_model_tide::setATL(t_gallatl* gallatl)
    //{
    //    _mutex.lock();
    //    _gatl = gallatl;
    //    _mutex.unlock();
    //    return;
    //
    //}
    //
    //// set ANTL pointer
    //// -----------------
    //void gnss_model_tide::setANTL(t_gallantl* gallantl)
    //{
    //    _mutex.lock();
    //    _gantl = gallantl;
    //    _mutex.unlock();
    //    return;
    //
    //}
    //
    //void gnss_model_tide::setOPL(gnss_data_opl* gallopl)
    //{
    //    _mutex.lock();
    //    _gopl = gallopl;
    //    _mutex.unlock();
    //    return;
    //
    //}

} // namespace
