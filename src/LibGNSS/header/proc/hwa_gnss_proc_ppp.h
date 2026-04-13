/**
* @file        gppp.h
* @brief    Purpose: implements PPP client abstract class
*/

#ifndef hwa_gnss_proc_ppp_H
#define hwa_gnss_proc_ppp_H

#include "hwa_base_par.h"
#include "hwa_gnss_proc_spp.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_all_otl.h"
#include "hwa_gnss_all_rslt.h"
#include "hwa_gnss_model_tropo.h"
#include "hwa_gnss_model_tropo.h"
#include "hwa_gnss_model_tide.h"
#include "hwa_gnss_model_tide96.h"
#include "hwa_gnss_model_tide2010.h"
#include "hwa_gnss_model_ephplan.h"

#ifdef _WIN32
#pragma warning(disable : 4250)
#endif

namespace hwa_gnss
{
    /** @brief class for gnss_proc_ppp derive from gnss_proc_spp. */
    class gnss_proc_ppp : public virtual gnss_proc_spp
    {
    public:
        /** @brief constructor 1. */
        gnss_proc_ppp(std::string mark, set_base *set);

        gnss_proc_ppp(std::string mark, set_base *set, base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_proc_ppp();

        /** @brief std::set OTL. 
        * @param[in] gotl ocean tide data
        */
        virtual void setOTL(gnss_all_otl *gotl);

        /** @brief std::set OBJ. 
        * @param[in] gobj object data
        */
        virtual void setOBJ(gnss_all_obj *gobj);

        /** @brief get isRunning. */
        virtual bool isRunning();

    protected:
        /** @brief Get settings from XML file and std::set local variables. */
        virtual int _get_settings();

        /** @brief Print results. */
        virtual void _prt_results(base_iof *giof, const gnss_all_rslt &rslt);

        gnss_all_otl *_gotl; ///< Ocean loading date.
        gnss_model_tide *_tides;  ///< Tides.

        bool _running;    ///< Running status.
        double _sigAmbig; ///< Ambiguity sigma (constraints).

        base_mutex _gmutex; ///< gmutex

    private:
    };

} // namespace

#endif //GPPP_H
