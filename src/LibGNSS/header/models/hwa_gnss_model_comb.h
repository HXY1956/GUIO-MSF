/**
* @file            gnss_model_comb.h
* @brief        base combine biase model
*/

#ifndef hwa_gnss_model_combMODEL_H
#define hwa_gnss_model_combMODEL_H

#include "hwa_gnss_model_base.h"
#include "hwa_gnss_model_Bias.h"
#include "hwa_gnss_model_iono.h"
#include "hwa_gnss_all_Bias.h"
#include "hwa_base_allproc.h"
#include "hwa_base_common.h"
#include <map>
#include <memory>

namespace hwa_gnss
{
    /** @brief Class for combine biase model */
    class gnss_model_comb : virtual public gnss_model_base
    {
    public:
        /** @brief default constructor.
        *
        *param[in]    spdlog                spdlog control
        *param[in]    settings        std::setbase control
        *param[in]    bias_model        model of bias
        *param[in]    data            all data
        */
        gnss_model_comb(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);

        /** @brief default constructor.
        *
        *param[in]    spdlog                spdlog control
        *param[in]    settings        std::setbase control
        *param[in]    bias_model        model of bias
        *param[in]    data            all data
        */
        gnss_model_comb(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);

        /** @brief default destructor. */
        virtual ~gnss_model_comb();

        /** @brief get index of band */
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> get_band_index();

        /** @brief get index of frequency */
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> get_freq_index();

    protected:
        /** @brief apply DCB in observation data */
        bool _apply_dcb_in_obsdata(gnss_data_sats &obsdata);

        // ====================================================================================================================
        /** @brief  combine equ for raw all
        *
        *param[in] gobs                observation data
        *param[in] satdata            satellite data
        *param[in] factorP            TODO
        *param[in] wgt_type            weight type
        *param[in] wgt                weight
        */
        bool _wgt_raw_obs(const gnss_data_obs &gobs, const gnss_data_sats &satdata, const double &factorP, const OBSWEIGHT &wgt_type, double &wgt);

        // ====================================================================================================================
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< index of band

        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; ///< index of frequency

        std::shared_ptr<gnss_model_bias> _bias_model; ///< baise model

        gnss_all_bias *_gallbias = nullptr; ///< bias setting derive
        base_log _spdlog = nullptr;      ///< implements spdlog class derived

        int _frequency;         ///< frequency
        IFB_model _ifb_model;   ///< model of IFB
        IFCB_MODEL _ifcb_model; ///< model of IFCB
        IONMODEL _ion_model;    ///< model of ION
        OBSCOMBIN _observ;      ///< observation
        bool _update_amb_lite;  ///< TODO

    protected:
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
    };
    class gnss_model_comb_if : virtual public gnss_model_comb
    {
    public:
        gnss_model_comb_if(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);
        gnss_model_comb_if(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);
        ~gnss_model_comb_if();

        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) override;
        bool cmb_equ_IF(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, GOBSBAND b1, GOBSBAND b2, gnss_model_base_equation &result);

    private:
        bool _add_IF_multi_rec_clk(const FREQ_SEQ &freq, gnss_data_sats &obsdata, base_allpar &params, std::vector<std::pair<int, double>> &coef_IF);

        std::map<std::pair<FREQ_SEQ, GSYS>, par_type> _clk_type_index;
    };

    class gnss_model_comb_if1x : virtual public gnss_model_comb
    {
    public:
        gnss_model_comb_if1x(set_base *setting, base_log spdlog, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data);
        ~gnss_model_comb_if1x();

        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) override;

    protected:
        gnss_model_comb_if _cmb_IF;
    };

    class gnss_model_comb_all : virtual public gnss_model_comb
    {
    public:
        gnss_model_comb_all(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);
        gnss_model_comb_all(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);
        ~gnss_model_comb_all();

        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) override;

    private:
        std::map<FREQ_SEQ, par_type> ambtype_list;
    };

    class gnss_model_comb_wl : public gnss_model_comb
    {
    public:
        gnss_model_comb_wl(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);
        gnss_model_comb_wl(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data);
        ~gnss_model_comb_wl();

        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) override;
        bool cmb_equ_WL(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, GOBSBAND b1, GOBSBAND b2, gnss_model_base_equation &result);
    };

    class gnss_model_comb_dd : virtual public gnss_model_comb_all, virtual public gnss_model_comb_if
    {
    public:
        gnss_model_comb_dd(set_base *setting, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data);
        gnss_model_comb_dd(set_base *setting, base_log spdlog, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data);
        ~gnss_model_comb_dd();

        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) override;

        void set_observ(OBSCOMBIN observ);
        void set_base_data(std::vector<gnss_data_sats> *data_base);
        void set_site(const std::string &site, const std::string &site_base);
        void set_rec_info(const Triple &xyz_base, double clk_rover, double clk_base);

    protected:
        bool _temp_params(base_allpar &params, base_allpar &params_temp);
        OBSCOMBIN _observ;
        std::vector<gnss_data_sats> *_data_base;
        std::string _site, _site_base;
        double _clk_rover, _clk_base;
        Triple _crd_base;
    };

    class gnss_model_comb_mix : virtual public gnss_model_comb_all, virtual public gnss_model_comb_if
    {
    public:
        gnss_model_comb_mix(set_base *setting, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data);
        gnss_model_comb_mix(set_base *setting, base_log spdlog, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data);
        ~gnss_model_comb_mix();
        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result) override;

    protected:
        gnss_data_ionex *_gdata_ionex = nullptr;
        bool _cmb_equ_IF_P1(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result);
    };

}
#endif