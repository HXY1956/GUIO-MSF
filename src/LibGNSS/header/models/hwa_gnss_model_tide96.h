
/**
* @file        gtide96.h
* @brief    Purpose: implements tides
*/

#ifndef hwa_gnss_model_tide96_H
#define hwa_gnss_model_tide96_H

#include <vector>

#ifdef BMUTEX
#include <boost/thread/mutex.hpp>
#endif

#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_gnss_model_tide.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for gnss_model_tide96 derive from gnss_model_tide. */
    class gnss_model_tide96 : public gnss_model_tide
    {

    public:
        /** @brief constructor 1. */
        gnss_model_tide96(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_model_tide96();

        /** @brief solid base_earth tides. */
        virtual Triple tide_earth(const base_time &epo, Triple &xyz);

        /** @brief pole tides. */
        virtual Triple tide_pole();

        /** @brief ocean tide loading. */
        virtual Triple load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec);

        /** @brief atmospheric tide loading. */
        virtual Triple load_atmosph();

    protected:
    };

} // namespace

#endif
