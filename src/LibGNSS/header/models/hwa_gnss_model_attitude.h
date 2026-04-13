/**
* @file        gattitudemodel.h
* @brief    attitude model class
*/

#ifndef hwa_gnss_model_attitude_H
#define hwa_gnss_model_attitude_H

#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_model_ephplan.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
     *@brief Class for satellite attitude model
     */
    class gnss_model_attitude
    {
    public:
        /** @brief Constructor    */
        gnss_model_attitude(){};

        /** @brief default destructor. */
        virtual ~gnss_model_attitude(){};

        // attitude modeling - public interface
        /** @brief get satellite attitude.
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        *return 
            @retval =0                success
            @retval =1                fail
        */
        int attitude_old(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k); // from RTKlib (to remove)

        /** @brief get satellite attitude.
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        *return
            @retval =0                success
            @retval =1                fail
        */
        int attitude(gnss_data_sats &satdata, double yaw, Triple &i, Triple &j, Triple &k);

        /** @brief satellite attitude model ( with input Xsat, Vsat, Xsun) used in OI
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                TODO
        *param[in] vsat                TODO
        *param[in] xsun                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        *return
            @retval =0                success
            @retval =1                fail
        */
        int attitude(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief get satellite attitude.
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        *return
            @retval =-1                fail
            @retval =0                fail
            @retval =1                success
        */
        int attitude(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

    protected:
        // From RTKlib - needs to be removed
        /** @brief yaw model of different satellite
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] xs                satellite position X
        *param[in] ys                satellite position Y
        *param[in] zs                satellite position Z
        *return
            @retval =0                fail
            @retval =1                success
        */
        int _yaw(gnss_data_sats &satdata, std::string antype, Triple &xs, Triple &ys, Triple &zs);

        // attitude niminal modeling
        /** @brief Yaw-steering mode attitude model
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _ysm(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief Yaw-steering mode attitude model
        *
        *param[in] prn                satellite prn
        *param[in] bata                TODO
        *param[in] mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _ysm(std::string prn, double bata, double mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief orbit normal mode (i toward the velocity)
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _onm(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief orbit normal mode (i toward the velocity)
        *
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _onm(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief noon maneuver
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn(gnss_data_sats &satdata, double R, Triple &i, Triple &j, Triple &k);

        /** @brief midnight maneuver
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn(gnss_data_sats &satdata, double R, Triple &i, Triple &j, Triple &k);

        /** @brief noon maneuver
        *
        *param[in] _prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] R                columnVector
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn(std::string _prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        /** @brief midnight maneuver
        *
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn(Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIA
        /** @brief Attitude modelling for GPS Block IIA
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIA(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief Attitude modelling for GPS Block IIA
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIA(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief     midnight maneuver for GPS Block IIA
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn_GPSIIA(gnss_data_sats &satdata, double R, Triple &i, Triple &j, Triple &k);

        /** @brief     midnight maneuver for GPS Block IIA
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn_GPSIIA(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIR
        /** @brief attitude for GPS Block IIR
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIR(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for GPS Block IIR
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIR(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for GPS Block IIR-M
        /** @brief attitude for GPS Block IIR-M
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIRM(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        // attitude for GPS Block IIF
        /** @brief attitude for GPS Block IIF
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIF(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for GPS Block IIF
        *
        *param[in] antype            antenna type
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIIF(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief midnight maneuver for GPS Block IIF
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn_GPSIIF(gnss_data_sats &satdata, double R, Triple &i, Triple &j, Triple &k);

        /** @brief midnight maneuver for GPS Block IIF
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn_GPSIIF(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        //atttude for GPS Block III
        /** @brief midnight maneuver for GPS Block III
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIII(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for GPS Block III
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GPSIII(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for Galileo IOV
        /** @brief attitude for Galileo IOV
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GAL1(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for Galileo IOV
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GAL1(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief noon maneuver for Galileo IOV
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn_GAL1(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief noon maneuver for Galileo IOV
        *
        *param[in] antype            antenna type
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn_GAL1(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for Galileo FOC
        /** @brief attitude for Galileo FOC
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GAL2(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for Galileo FOC
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GAL2(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief noon maneuver for Galileo FOC
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn_GAL2(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief noon maneuver for Galileo FOC
        *
        *param[in] antype            antenna type
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn_GAL2(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // Continuous yaw steering attitude modes of BDS satellites
        /** @brief Continuous yaw steering attitude modes of BDS satellites in CAST
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _cys_cast(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief Continuous yaw steering attitude modes of BDS satellites in CAST
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _cys_cast(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief Continuous yaw steering attitude modes of BDS satellites in SECM
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _cys_secm(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief Continuous yaw steering attitude modes of BDS satellites in SECM
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _cys_secm(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for BeiDou
        /** @brief attitude for BeiDou
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_BDS(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for BeiDou
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_BDS(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // continuous yaw steering attitude modes of QZS-2 IGSO satellites, added by yqyuan
        /** @brief Continuous yaw steering attitude modes of of QZS-2 IGSO satellites
        *
        *param[in] satdata            satellite data
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _cys_qzs(gnss_data_sats &satdata, Triple &i, Triple &j, Triple &k);

        /** @brief Continuous yaw steering attitude modes of of QZS-2 IGSO satellites
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _cys_qzs(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for QZSS
        /** @brief attitude for QZSS
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_QZS(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for QZSS
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_QZS(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        // attitude for GLO
        /** @brief attitude for GLO
        *
        *param[in] satdata            satellite data
        *param[in] antype            antenna type
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GLO(gnss_data_sats &satdata, std::string antype, Triple &i, Triple &j, Triple &k);

        /** @brief attitude for QZSS
        *
        *param[in] antype            antenna type
        *param[in] prn                satellite prn
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _attitude_GLO(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k);

        /** @brief midnight maneuver for GLONASS-M
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn_GLOM(gnss_data_sats &satdata, double R, Triple &i, Triple &j, Triple &k);

        /** @brief noon maneuver for GLONASS-M
        *
        *param[in] satdata            satellite data
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn_GLOM(gnss_data_sats &satdata, double R, Triple &i, Triple &j, Triple &k);

        /** @brief midnight maneuver for GLONASS-M
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _midnight_turn_GLOM(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        /** @brief midnight maneuver for GLONASS-M
        *
        *param[in] prn                satellite prn
        *param[in] _beta            TODO
        *param[in] _mi                TODO
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] R                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _noon_turn_GLOM(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k);

        /** @brief Calculate satellite-std::fixed std::vectors from yaw angle
        *
        *param[in] satdata            satellite data
        *param[in] yaw                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _yaw2ijk(gnss_data_sats &satdata, double &yaw, Triple &i, Triple &j, Triple &k);

        /** @brief Calculate satellite-std::fixed std::vectors from yaw angle
        *
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        *param[in] yaw                TODO
        *param[in] i                columnVector1
        *param[in] j                columnVector2
        *param[in] k                columnVector3
        */
        void _yaw2ijk(Vector &xsat, Vector &vsat, Vector &xsun, double &yaw, Vector &i, Vector &j, Vector &k);

        /** @brief Calculate orbit angle
        *
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        */
        double _orb_angle(Vector &xsat, Vector &vsat, Vector &xsun);

        /** @brief Calculate beta
        *
        *param[in] xsat                satellite position
        *param[in] vsat                satellite velocity
        *param[in] xsun                sun position
        */
        double _beta(Vector &xsat, Vector &vsat, Vector &xsun);

        /** @brief Calculate the value of a with the sign of b
        *
        *param[in] a                parameter 1 
        *param[in] b                parameter 2
        */
        double sign(double a, double b);

        ATTITUDES _attitudes;           ///< satellite attitudes
        gnss_model_ephplan _ephplan;            ///< planetary ephemerises
        std::map<std::string, double> _last_beta; ///< last beta
        std::map<std::string, double> _last_yaw;  ///< last yaw
        std::map<std::string, base_time> _last_epo; ///< last epoch
    };
}

#endif