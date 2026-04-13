
/**
*
* @file        gsppmodel.h
* @brief    Purpose: various SPP models
*/

#ifndef hwa_gnss_model_spp_H
#define hwa_gnss_model_spp_H

#include "hwa_base_eigendef.h"
#include "hwa_base_log.h"
#include "hwa_base_allpar.h"
#include "hwa_base_globaltrans.h"
#include "hwa_set_proc.h"
#include "hwa_set_gtype.h"
#include "hwa_set_gbase.h"
#include "hwa_set_rec.h"
#include "hwa_gnss_model.h"
#include "hwa_gnss_model_gmf.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_data_Obj.h"
#include "hwa_gnss_data_obs.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /** @brief class for gnss_model_spp derive from t_godel. */
    class gnss_model_spp : public gnss_model
    {
    public:
        /** @brief constructor 1. */
        gnss_model_spp(std::string site, set_base *settings);

        gnss_model_spp(base_log spdlog, std::string site, set_base *settings);
        /** @brief default constructor. */
        gnss_model_spp();

        /** @brief default destructor. */
        virtual ~gnss_model_spp();

        /** @brief Outliers detection. */
        virtual int outlierDetect(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &, const Vector &); // OLD
        virtual int outlierDetect(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &, std::vector<gnss_data_sats>::iterator &);
        virtual int outlierDetect(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &);
        virtual int outlierDetect_chi(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &, const Vector &); // OLD

        /** @brief model for computed range value. */
        virtual double cmpObs(base_time &epoch, base_allpar &param, gnss_data_sats &, gnss_data_obs &gobs, bool com = false);

        /** @brief model for IRC computed range value. */
        virtual double cmpObsIRC(base_time &epoch, base_allpar &param, gnss_data_sats &, gnss_data_obs &gobs, bool com = false); //add by xiongyun

        /** @brief model computed D range value (phase/code). */
        virtual double cmpObsD(base_time &epoch, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs);

        /** @brief get trop Delay. */
        double tropoDelay(base_time &epoch, base_allpar &param, Triple site_ell, gnss_data_sats &satdata);

        /** @brief get iono Delay. */
        virtual double ionoDelay(base_time &epoch, base_allpar &param, Triple site_ell, gnss_data_sats &satdata, gnss_data_obs &gobs);

        // jdhuang add
        /** @brief is Correction. */
        double isbCorrection(base_allpar &param, std::string &sat, std::string &rec, gnss_data_obs &gobs);

        /** @brief is reset observation */
        virtual void reset_observ(OBSCOMBIN observ) override;

        /** @brief is reset tropmf */
        virtual void reset_tropmf(ZTDMPFUNC mf) { _tropo_mf = mf; } //LvHB added in 201908

        /** @brief is set rec */
        virtual void setrec(std::shared_ptr<gnss_data_obj> rec);

        /** @brief get outlier satellite */
        std::vector<std::string> get_outlier_sat() { return _outlier_sat; } //added by xiongyun

    protected:
        /** @brief Find maximal residual */
        double _maxres(const Vector &v, GSYS gs, bool phase, std::vector<gnss_data_sats> &data, std::vector<gnss_data_sats>::iterator &itDATA); // OLD
        double _maxres(bool phase, std::vector<gnss_data_sats> &data, std::vector<gnss_data_sats>::iterator &itDATA, RESIDTYPE res_type, GSYS gs = GNS);

        /** @brief check maximal residual */
        bool _check_outl(bool phase, double &maxres, std::vector<gnss_data_sats>::iterator &itData, std::vector<gnss_data_sats> &data); // OLD
        bool _check_outl(bool phase, double &maxresNORM, std::vector<gnss_data_sats>::iterator &itDataNORM,
                         double &maxresORIG, std::vector<gnss_data_sats>::iterator &itDataORIG,
                         std::vector<gnss_data_sats>::iterator &itDataErase, std::vector<gnss_data_sats> &data);

        /** @brief logging outlier */
        void _logOutl(bool phase, std::string prn, int data_size, double maxres, double ele, base_time epo, RESIDTYPE resid_type);

        /** @brief devide multi-freq residuals std::vector into single-freq */
        std::vector<Vector> _devide_res(const Vector &v_orig);

        std::shared_ptr<std::fstream> debug_output;               ///< debug output
        std::map<GSYS, double> _maxres_C;                    ///< code maximal residual
        std::map<GSYS, double> _maxres_L;                    ///< phase maximal residual
        double _maxres_norm;                            ///< normal maximal residual
        std::shared_ptr<gnss_data_obj> _grec;                       ///< grec
        Triple _antennal_height;                     ///< antennal height
        TROPMODEL _trpModStr;                           ///< trop mod(str)
        ZTDMPFUNC _tropo_mf;                            ///< trop mf
        RESIDTYPE _resid_type;                          ///< residual type
        OBSCOMBIN _observ;                              ///< observation
        CBIASCHAR _cbiaschar;                           ///< bias(char)
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< band
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; ///< freq
        std::vector<std::string> _outlier_sat;                    //added by xiongyun

        std::string strEst;          ///< str est
        bool emptyBase = false; ///< empty Base
    };

} // namespace

#endif //  GSPPMODEL_H
