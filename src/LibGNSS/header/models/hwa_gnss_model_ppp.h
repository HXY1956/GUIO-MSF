
/**
*
*
* @file        gpppmodel.h
* @brief    Purpose: various PPP models
*.
*/

#ifndef hwa_gnss_model_ppp_H
#define hwa_gnss_model_ppp_H

#include <string>
#include <map>
#include <cmath>

#include "hwa_gnss_model_spp.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_gnss_all_Obj.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for gnss_model_ppp derive from gnss_model_spp. */
    class gnss_model_ppp : public gnss_model_spp
    {
    public:
        /** @brief constructor 1. */
        gnss_model_ppp(std::string site, base_log spdlog, set_base *settings);

        /** @brief default constructor. */
        gnss_model_ppp();

        /** @brief default destructor. */
        virtual ~gnss_model_ppp();

        /** @brief get WindUp. */
        virtual double windUp(gnss_data_sats &satdata, const Vector &);

        /** @brief model computed range value (phase/code). */
        virtual double cmpObs(base_time &epoch, base_allpar &param, gnss_data_sats &, gnss_data_obs &gobs, bool com);

        /** @brief get trop Delay. */
        virtual double tropoDelay(base_time &epoch, base_allpar &param, Triple ell, gnss_data_sats &satdata);

        /** @brief set object. */
        virtual void setOBJ(gnss_all_obj *obj);

        // attitude modeling - public interface
        int attitude_old(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k); // from RTKlib (to remove)
        int attitude(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        int attitude(gnss_data_sats &satdata, double yaw, Vector &i, Vector &j, Vector &k);
        int attitude(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

    protected:
        // From RTKlib - needs to be removed
        int _yaw(gnss_data_sats &satdata, std::string antype, Vector &xs, Vector &ys, Vector &zs);

        // attitude niminal modeling
        void _ysm(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _ysm(std::string prn, double bata, double mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _onm(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _onm(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _noon_turn(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k);
        void _midnight_turn(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k);
        void _noon_turn(std::string _prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);
        void _midnight_turn(Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIA
        void _attitude_GPSIIA(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GPSIIA(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _midnight_turn_GPSIIA(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k);
        void _midnight_turn_GPSIIA(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIR
        void _attitude_GPSIIR(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GPSIIR(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIR-M
        void _attitude_GPSIIRM(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIF
        void _attitude_GPSIIF(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GPSIIF(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _midnight_turn_GPSIIF(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k);
        void _midnight_turn_GPSIIF(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        //atttude for GPS Block III
        void _attitude_GPSIII(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GPSIII(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for Galileo IOV
        void _attitude_GAL1(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GAL1(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _noon_turn_GAL1(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _noon_turn_GAL1(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for Galileo FOC
        void _attitude_GAL2(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GAL2(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _noon_turn_GAL2(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _noon_turn_GAL2(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // Continuous yaw steering attitude modes of BDS satellites
        void _cys_cast(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _cys_cast(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);
        void _cys_secm(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _cys_secm(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);
        // attitude for BeiDou
        void _attitude_BDS(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_BDS(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // continuous yaw steering attitude modes of QZS-2 IGSO satellites, added by yqyuan
        void _cys_qzs(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _cys_qzs(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);
        void _switch_qzs1(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k);
        void _switch_qzs1(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for QZSS
        void _attitude_QZS(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_QZS(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for GLO
        void _attitude_GLO(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k);
        void _attitude_GLO(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        void _midnight_turn_GLOM(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k);
        void _noon_turn_GLOM(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k);
        void _midnight_turn_GLOM(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);
        void _noon_turn_GLOM(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        /** @brief Calculate satellite-std::fixed std::vectors from yaw angle. */
        void _yaw2ijk(gnss_data_sats &satdata, double &yaw, Vector &i, Vector &j, Vector &k);
        void _yaw2ijk(Vector &xsat, Vector &vsat, Vector &xsun, double &yaw, Vector &i, Vector &j, Vector &k);

        double _orb_angle(Vector &xsat, Vector &vsat, Vector &xsun);
        double _beta(Vector &xsat, Vector &vsat, Vector &xsun);

        double sign(double a, double b);

        void _set_beta0(double beta);
        double _get_beta0();

        std::map<std::string, double> _windUpTime; ///< windup Time
        std::map<std::string, double> _windUpSum;  ///< windup sum
        gnss_model_ephplan _ephplan;             ///< ephplan
        gnss_all_obj *_gallobj;             ///< all obj
        GRDMPFUNC _grad_mf;              ///< grad mf
        std::string _trop_corr;               ///< trop corr
        ATTITUDES _attitudes;            ///< attitudes

        std::map<std::string, double> _last_beta; ///< last_beta
        std::map<std::string, double> _last_yaw;  ///< last_yaw
        std::map<std::string, base_time> _last_epo; ///< last_epo

        double _beta0 = 0; ///< beta_angle at the fisrt epoch; for QZS-1 attitude; added by yqyuan
    };

} // namespace

#endif //  GPPPMODEL_H
