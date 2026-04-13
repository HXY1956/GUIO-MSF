
/**
* @file        gtide.h
* @brief    Purpose: implements tides
*/

#ifndef hwa_gnss_model_tide_H
#define hwa_gnss_model_tide_H

#include <vector>

#ifdef BMUTEX
#include <boost/thread/mutex.hpp>
#endif

#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_base_mutex.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_gnss_all_otl.h"
#include "hwa_gnss_all_opl.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model_tide. */
    class gnss_model_tide
    {

    public:
        /** @brief constructor 1. */
        gnss_model_tide(); ///< spdlog pointer)

        gnss_model_tide(base_log spdlog); ///< spdlog pointer)
        /** @brief default destructor. */
        virtual ~gnss_model_tide();

        /** @brief set the log. */
        void spdlog(base_log spdlog);

        /** @brief set OTL. */
        virtual void setOTL(gnss_all_otl *gallotl);
        //virtual void setATL(t_gallatl* gallatl);
        //virtual void setANTL(t_gallantl* gallantl);
        //virtual void setOPL(gnss_data_opl* gallopl);

        /** @brief solid base_earth tides. */
        virtual Triple tide_earth(const base_time &epo, Triple &xyz);

        /** @brief pole tides. */
        virtual Triple tide_pole();

        /** @brief ocean tide loading. */
        virtual Triple load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec);

        /** @brief atmospheric tide loading. */
        virtual Triple load_atmosph();

    protected:
        gnss_model_ephplan _planEph; ///< plan eph
        gnss_all_otl *_gotl;    ///< all otl
        //t_gallatl*   _gatl;
        //t_gallantl*  _gantl;
        //gnss_data_opl* _gopl;
        base_mutex _mutex; ///< mutex

        base_log _spdlog; ///< spdlog pointer

#ifdef BMUTEX
        boost::mutex _mutex;
#endif
    };

} // namespace

#endif
