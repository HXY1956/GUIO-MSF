#ifndef hwa_gnss_model_precisebias_GPP_H
#define hwa_gnss_model_precisebias_GPP_H

#include "hwa_set_base.h"
#include "hwa_base_allpar.h"
#include "hwa_set_gen.h"
#include "hwa_gnss_model_bias.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_gnss_model_attitude.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_gnss_TRS2CRS.h"

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for gnss_model_precise_biasgpp, derive from gnss_model_precise_bias
    */
    class gnss_model_precise_biasgpp : public gnss_model_precise_bias
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  data              the data pointer
        * @param[in]  log              the log pointer
        * @param[in]  setting          the set pointer
        */
        explicit gnss_model_precise_biasgpp( base_all_proc *data, hwa_set::set_base *setting);

        gnss_model_precise_biasgpp( base_all_proc *data, base_log spdlog, hwa_set::set_base *setting);
        /** @brief default destructor. */
        ~gnss_model_precise_biasgpp();

        /**
        * @brief combine EQU.
        *
        * @param[in]  epoch              the current time
        * @param[in]  params          the parameters
        * @param[in]  obsdata          the observation data
        * @param[in]  gobs              the observation object
        * @param[in]  result          the EQU result
        * @return      bool              combine EQU mode
        */
        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result) override;

        /**
        * @brief prepare observation for GPP.
        *
        * @param[in]  epoch              the current time
        * @param[in]  nav             the navigation data
        * @param[in]  gallobj          the all object
        * @param[in]  pars              the pars
        * @return      bool              prepare mode
        */
        bool _prepare_obs_GPP(const base_time &epoch, gnss_all_nav *nav, gnss_all_obj *gallobj, base_allpar &pars);

        /**
        * @brief update observation information for GPP.
        *
        * @param[in]  epoch              the current time
        * @param[in]  nav             the navigation data
        * @param[in]  gallobj          the all object
        * @param[in]  obsdata          the observation data
        * @param[in]  pars              the pars
        * @return      bool              update mode
        */
        bool _update_obs_info_GPP(const base_time &epoch, gnss_all_nav *nav, gnss_all_obj *gallobj, gnss_data_sats &obsdata, base_allpar &pars);

        /**
        * @brief update clock obiect for GPP.
        *
        * @param[in]  obj              the obj
        * @param[in]  epoch              the current time
        * @param[in]  nav             the navigation data
        * @param[in]  pars              the pars
        * @param[in]  clk                the clock value
        * @param[in]  obj_clk         clock obj
        * @return      bool              update mode
        */
        bool _update_obj_clk_GPP(const std::string &obj, const base_time &epo, gnss_all_nav *nav, base_allpar &par, double &clk, std::map<std::string, std::pair<base_time, double>> &obj_clk);
        //virtual void reset_SatPCO(bool cal = true) { _isCalSatPCO = cal; };         //zzwu added according to lvhb
    protected:

        /**
        * @brief compute obs.
        *
        * @param[in]  epoch              the current time
        * @param[in]  sat              the satellite name
        * @param[in]  rec             the receiver data
        * @param[in]  param              the paramter
        * @param[in]  gsatdata        the satellite data
        * @param[in]  gobs            the obs
        * @return      double          the compute value
        */
        double cmpObs(base_time &epoch, std::string &sat, std::string &rec, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs) override;

    private:

        /**
        * @brief prt all observation.
        *
        * @param[in]  crt_epo          the current time
        * @param[in]  obsdata          the satellite data
        * @param[in]  pars             the paramter
        * @param[in]  gobs            the obs
        * @param[in]  coeff           the coeff
        * @return      bool             the prt mode
        */
        bool _prt_obs_all(const base_time &crt_epo, gnss_data_sats &obsdata, base_allpar &pars, gnss_data_obs &gobs, std::vector<std::pair<int, double>> &coeff);

        /**
        * @brief get satellite clock correction.
        *
        * @param[in]  sat_epoch          the current time
        * @param[in]  sat             the satellite name
        * @param[in]  clk             the clock value
        * @param[in]  dclk            the dclock value
        * @return      bool             get mode
        */
        bool _get_sat_clk_corr(const base_time &sat_epoch, const std::string &sat, double &clk, double &dclk); //add by zhshen

        NPP_MODEL nppmodel;                 ///< the npp model
        bool _ddmode;                       ///< dd mode, lvhb added in 20210313
    };
}

#endif