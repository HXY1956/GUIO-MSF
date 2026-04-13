/**
* @file            gnss_model_biasemodel.h
* @brief        mainly about how to cacultae B P l single
*/

#ifndef hwa_gnss_model_biasEMODEL_H
#define hwa_gnss_model_biasEMODEL_H
#include "hwa_base_time.h"
#include "hwa_base_allpar.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_gnss_data_obsmanager.h"
#include "hwa_gnss_model_base.h"
#include "hwa_gnss_model_iono.h"
#include "hwa_gnss_model_tropo.h"
#include "hwa_gnss_model_isb.h"
#include "hwa_gnss_model_ifcb.h"
#include "hwa_gnss_model_ifb.h"
#include "hwa_gnss_model_reldelay.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief gnss_model_bias Class for bias model
    */
    class gnss_model_bias
    {
    public:
        /** @brief default constructor.
        *
        *param[in]    spdlog            data spdlog
        *param[in]    settings        std::setbase control
        */
        gnss_model_bias(set_base *settings);

        /** @brief default constructor. 
        *
        *param[in]    spdlog            data spdlog    
        *param[in]    settings        std::setbase control
        */
        gnss_model_bias(base_log spdlog, set_base *settings);

        /** @brief default constructor.*/
        gnss_model_bias();

        /** @brief default destructor. */
        virtual ~gnss_model_bias();

        /** @brief return false */
        virtual bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result) = 0;

        /** @brief empty function */
        virtual void update_obj_clk(const std::string &obj, const base_time &epo, double clk){};

        /** @brief get clk of reciever */
        virtual double get_rec_clk(const std::string &obj) { return 0.0; }; //add by xiongyun

		virtual bool get_omc_obs_all(const base_time& crt_epo, gnss_data_sats& obsdata, base_allpar& pars, gnss_data_obs& gobs, double& omc);

        Triple _trs_rec_crd; ///< coordinates of reciever in terrestrial reference system
        Triple _crs_rec_crd; ///< coordinates of reciever in coordinate reference system
        Triple _trs_sat_crd; ///< coordinates of satellite in terrestrial reference system
        Triple _crs_sat_crd; ///< coordinates of satellite in coordinate reference system
        Triple _trs_rec_pco; ///< PCO of reciever in terrestrial reference system
        Triple _crs_rec_pco; ///< PCO of reciever in coordinate reference system
        Triple _trs_sat_pco; ///< PCO of satellite in terrestrial reference system
        Triple _crs_sat_pco; ///< PCO of satellite in coordinate reference system
        Triple _trs_rec_vel; ///< velocity of reciever in terrestrial reference system
        Triple _crs_rec_vel; ///< velocity of reciever in coordinate reference system
        Triple _trs_sat_vel; ///< velocity of satellite in terrestrial reference system
        Triple _crs_sat_vel; ///< velocity of satellite in coordinate reference system

        std::string _crt_sat; ///< current satellite
        std::string _crt_rec; ///< current reciever
        GSYS _crt_sys;   ///< current system

    protected:
        /** @brief calculate distance of models
        *
        *param[in] epoch            current epoch
        *param[in] sat                satlite name
        *param[in] rec                reciever name
        *param[in] param            reciever name
        *param[in] gsatdata            satlite data
        *param[in] gobs                observation data
        */
        virtual double cmpObs(base_time &epoch, std::string &sat, std::string &rec, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs) { return 0.0; };

        // wegight obs
        /** @brief weight RAW_ALL observation
        *
        *param[in] type                type of observation
        *param[in] gobs1            observations
        *param[in] obsdata            observation data
        *param[in] factorP            factor P
        *param[in] wgt                weight
        */
        bool _wgt_obs_all(const base_data::ID_TYPE &type, gnss_data_obs &gobs1, gnss_data_sats &obsdata, const double &factorP, double &wgt);

        /** @brief omc RAW_ALL observation
        *
        *param[in] crt_epo            current epoch
        *param[in] obsdata            observation data
        *param[in] pars                parameter
        *param[in] omc                TODO
        */
        bool _omc_obs_all(const base_time &crt_epo, gnss_data_sats &obsdata, base_allpar &pars, gnss_data_obs &gobs, double &omc);

        /** @brief calculate basic partial
        *
        *param[in] epoch            current epoch
        *param[in] obsdata            observation data
        *param[in] gobs                observations
        *param[in] par                parameter
        *param[in] partial            partial
        */
        bool _Partial_basic(const base_time &epoch, gnss_data_sats &obsdata, const gnss_data_obs &gobs, const base_par &par, double &partial);

        /** @brief get mapPing function
        *
        *param[in] par                parameter
        *param[in] satData            data of satellite
        *param[in] crd                coordinate
        *param[in] epoch            epoch
        *param[in] mfw                TODO
        *param[in] mfh                TODO
        *param[in] dmfw                TODO
        *param[in] dmfh                TODO
        */
        void _getmf(const base_par &par, gnss_data_sats &satData, const Triple &crd, const base_time &epoch, double &mfw, double &mfh, double &dmfw, double &dmfh);

        base_log _spdlog;  ///< spdlog data
        set_base *_gset; ///< std::set base control

        //======================================================================================================\
        // xml swtting
        // trop
        int _frequency;                      ///< _frequency
        CONSTRPAR _crd_est = CONSTRPAR::FIX; ///< TODO
        std::shared_ptr<gnss_model_tropo> _tropoModel;    ///< model of tropo
        TROPMODEL _trpModStr;                ///< TODO
        ZTDMPFUNC _mf_ztd;                   ///< mapPing function for ZTD
        GRDMPFUNC _mf_grd;                   ///< mapPing function for GRD
        IONMODEL _ion_model;                 ///< model of IONO

        OBSWEIGHT _weight; ///< weight calculation method

        double _sigCodeGPS;    ///< code bias of GPS
        double _sigCodeGLO;    ///< code bias of GLO
        double _sigCodeGAL;    ///< code bias of GAL
        double _sigCodeBDS;    ///< code bias of BDS
        double _sigCodeQZS;    ///< code bias of QZS
        double _sigCodeGPSLEO; ///< code bias of GPS for LEO by zhangwei
        double _sigCodeGLOLEO; ///< code bias of GLO for LEO
        double _sigCodeGALLEO; ///< code bias of GAL for LEO
        double _sigCodeBDSLEO; ///< code bias of BDS for LEO
        double _sigCodeQZSLEO; ///< code bias of QZS for LEO

        double _sigPhaseGPS;    ///< phase bias of GPS
        double _sigPhaseGLO;    ///< phase bias of GLO
        double _sigPhaseGAL;    ///< phase bias of GAL
        double _sigPhaseBDS;    ///< phase bias of BDS
        double _sigPhaseQZS;    ///< phase bias of QZS
        double _sigPhaseGPSLEO; ///< phase bias of GPS for LEO
        double _sigPhaseGLOLEO; ///< phase bias of GLO for LEO
        double _sigPhaseGALLEO; ///< phase bias of GAL for LEO
        double _sigPhaseBDSLEO; ///< phase bias of BDS for LEO
        double _sigPhaseQZSLEO; ///< phase bias of QZS for LEO

        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< index of band
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; ///< index of frequency
        OBSCOMBIN _observ;                              ///< type of observations
        modeofmeanpole _meanpole_model;
        //======================================================================================================\

    };

} // namespace hwa_gnss

#endif