#ifndef hwa_gnss_model_precisebias_H
#define hwa_gnss_model_precisebias_H

#include "hwa_set_base.h"
#include "hwa_set_gen.h"
#include "hwa_gnss_model_bias.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_TRS2CRS.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_all_opl.h"
#include "hwa_gnss_all_atmloading.h"
#include "hwa_gnss_model_tide.h"
#include "hwa_gnss_model_attitude.h"
#include "hwa_gnss_data_navde.h"
#include "hwa_gnss_data_ifcb.h"
#include "hwa_gnss_data_poleut.h"

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for precise bias    */
    class gnss_model_precise_bias : public gnss_model_bias
    {
    public:
        gnss_model_precise_bias( base_all_proc *data, set_base *setting);
        gnss_model_precise_bias( base_all_proc *data, base_log spdlog, set_base *setting);
        ~gnss_model_precise_bias();

        void set_multi_thread(const std::set<std::string> &sites);

        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result) override;
        void update_obj_clk(const std::string &obj, const base_time &epo, double clk) override;
        double get_rec_clk(const std::string &obj) override;

        double tropoDelay(base_time &epoch, std::string &rec, base_allpar &param, Triple site_ell, gnss_data_sats &satdata);
        double ionoDelay(base_time &epoch, base_allpar &param, gnss_data_sats &satdata, IONMODEL &ion_model, GOBSBAND &band_1, gnss_data_obs &gobs);
        double isbDelay(base_allpar &param, gnss_data_sats &satdata, std::string &sat, std::string &rec, gnss_data_obs &gobs);
        double ifbDelay(base_allpar &param, gnss_data_sats &satdata, std::string &sat, std::string &rec, gnss_data_obs &gobs);
        double ifcbDelay(gnss_data_sats &satdata, gnss_data_ifcb *ifcb, OBSCOMBIN obscombin);
        double relDelay(Triple &crd_site, Triple &vel_site, Triple &crd_sat, Triple &vel_sat);
        bool _update_obs_info(gnss_data_sats &obsdata);
        virtual void reset_SatPCO(bool cal = true) { _isCalSatPCO = cal; }; //zzwu added according to lvhb
		virtual gnss_data_sats get_crt_obs() { return _crt_obs; }

    protected:
        bool _apply_rec_RTK(const base_time &crt_epo, const base_time &rec_epo, base_allpar &pars, bool calculate_tides);// add by hyChang, _apply_rec_tides can be removed in RTK to achieve faster computing speed
        bool _apply_rec(const base_time &crt_epo, const base_time &rec_epo, base_allpar &pars);
        bool _apply_sat(const base_time &rec_epo, base_time &sat_epo, gnss_all_nav *nav);
        bool _apply_rec_tides(const base_time &epoch, Triple &rec);
        void _update_rot_matrix(const base_time &epoch);

        int _correction_orb(double *xyz, double *vxyz, double *xyz_corr, double *vxyz_corr); // add by xiongyun
        int _correction_clk(double &clk, double &dclk, double &clk_corr, double &dclk_corr); // add by zhshen

        bool _get_crs_sat_crd(const base_time &sat_epoch, const std::string &sat, gnss_all_nav *nav, Triple &crs_sat_crd);
        bool _get_crs_sat_vel(const base_time &sat_epoch, const std::string &sat, gnss_all_nav *nav, Triple &crs_sat_vel);
        bool _get_crs_sat_crdvel(const base_time& sat_epoch, const std::string& sat, const int& iod, Triple& crs_sat_crd, Triple& crs_sat_vel);//add by tyx

    protected:
        /**
        * @brief compute windup correciton
        * @note override for std::fixed bugs in computing satellite attitude
        * @param[in] satdata observ data info
        * @param[in] rRec coord of receiver in TRS
        * @return correction of windup
        */
        double windUp(const GOBSBAND &freq_2, gnss_data_sats &satdata, const Triple &rRec);
        double PCV(bool corrt_sat, bool corrt_rec, bool realtime, base_time &epoch, base_time &crt_sat_epo, Triple &trs_rec_crd, gnss_data_sats &satdata, gnss_data_obs &gobs);

        /**
        * @brief get rotmatrix from ant to TRS or CRS
        * @param[in] obsdata observ data info
        * @param[in] epoch specified epoch
        * @param[in] obj pcv info for site
        * @param[in] isCRS ture to CRS false to TRS
        * @return Rotmatrix
        */
        Matrix _RotMatrix_Ant(gnss_data_sats &obsdata, const base_time &receive_epoch, const base_time &transmit_epoch, std::shared_ptr<gnss_data_obj> obj, bool isCRS);
        bool _prt_obs_all(const base_time &epoch, gnss_data_sats &obsdata, base_allpar &pars, gnss_data_obs &gobs, std::vector<std::pair<int, double>> &coeff);
        double _Partial(const base_time &epoch, gnss_data_sats &obsdata, const gnss_data_obs &gobs, base_par &par);

    protected:
        base_time _crt_epo; ///< receive epoch
        gnss_data_sats _crt_obs;
        std::shared_ptr<gnss_data_obj> _crt_obj;
        /*base_time             _crt_eop;     ///<receive epoch*/
        base_time _crt_rec_epo;
        base_time _crt_sat_epo; ///< transmit epoch
        double _crt_rec_clk;
        double _crt_sat_clk;

        CONSTRPAR _crd_est = CONSTRPAR::FIX;

        ATTITUDES _attitudes;
        gnss_model_attitude _gattitude_model;
        bool _realtime = false;     ///< for realtime mode
        bool  _ultrasp3 = false;//added by xiongyun(whether use sp3 file in realtime)
        bool _trop_est = true;      ///< estimate tropo or not
        bool _is_flt = false;       ///< flt or lsq
        bool _corrt_sat_pcv = true; ///< correct sat pcv or not
        bool _corrt_rec_pcv = true; ///< correct rec pcv or not
        GRDMPFUNC _grad_mf;
        std::shared_ptr<gnss_model_tide> _tide; ///< tide correction model

        gnss_data_poleut *_gdata_erp = nullptr; ///< all poleut data
        gnss_data_navde *_gdata_navde = nullptr; ///< all panetnav info
        gnss_all_nav *_gall_nav = nullptr;   ///< all nav data include rinexn,sp3,clk
        gnss_all_obj *_gallobj = nullptr;

        gnss_all_atmloading *_atm_loading = nullptr;
        gnss_all_opl *_opl = nullptr;
        modeofmeanpole _mean_pole_model = modeofmeanpole::cubic;

        std::map<base_time, std::shared_ptr<gnss_base_trs2crs>> _trs2crs_list;
        std::shared_ptr<gnss_base_trs2crs> _trs2crs_2000; ///< trs2crs matrix
        double _minElev;                      ///< min ele for prepare

        //Matrix _rot_ant2crs;            ///< from ground site antenna reference to crs
        Matrix _rot_scf2crs; ///< record scf2crs matrix
        Matrix _rot_scf2trs; ///< record scf2trs matrix

        std::map<std::string, std::map<std::string, std::map<base_time, double>>> _phase_windup; ///< recording for calculating windup

        gnss_data_ifcb *_gifcb = nullptr;
        std::map<std::string, std::pair<base_time, double>> _obj_clk;
        std::map<std::string, double> _rec_clk; //add by xiongyun

        bool _isCalSatPCO = true; //zzwu added according to lvhb
        std::tuple<std::string, std::string, base_time> _rec_sat_before;
    };
}

#endif