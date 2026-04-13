
#ifndef hwa_gnss_data_navirn_H
#define hwa_gnss_data_navirn_H

#include "hwa_gnss_data_Nav.h"
#include "hwa_base_time.h"
#include "hwa_base_const.h"
#include "hwa_base_common.h"
#include "hwa_base_eigendef.h"

#define MAX_rinexn_REC_IRN 28
#define SYS_IRN 'I'

namespace hwa_gnss
{
#define GM_WGS84 3.986005e14     ///< WGS 84 value of base_earth's graviational constant for GPS user [m^3/s^2]
#define OMGE_DOT 7.2921151467e-5 ///< WGS 84 value of the base_earth's rotation rate [rad/sec]
#define SC2R 3.1415926535898     ///< semi-circle to radian

    /** @brief class for gnss_data_navirn. */
    class gnss_data_navirn : public gnss_data_nav
    {

    public:
        /** @brief default constructor. */
        gnss_data_navirn();

        /**
         * @brief Construct a new t gnavirn object
         * 
         * @param spdlog 
         */
        gnss_data_navirn(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_navirn();

        // pointers to support NULL if not requested!
        /**
         * @brief 
         * 
         * @param t 
         * @param xyz 
         * @param var 
         * @param vel 
         * @param chk_health 
         * @return int 
         */
        int pos(const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true) override; //[m]

        /**
         * @brief 
         * 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @param chk_health 
         * @return int 
         */
        int clk(const base_time &t, double *clk, double *var = NULL, double *dclk = NULL, bool chk_health = true) override; //[s]

        /**
         * @brief 
         * 
         * @param msg 
         * @return int 
         */
        int chk(std::set<std::string> &msg) override;

        /** @brief convert data to nav. */
        int data2nav(std::string, const base_time &ep, const gnss_data_navDATA &data) override;

        /** @brief convert nav to data. */
        int nav2data(gnss_data_navDATA &data) override;

        /** @brief get iod. */
        int iod() const override { return _iode; }

        /** @brief get rec. */
        int rec() const override { return MAX_rinexn_REC_IRN; }

        /** @brief check tot. */
        virtual bool chktot(const base_time &t) override;

        /** @brief get/std::set parameter. */
        hwa_map_time_value param(const NAVDATA &n) override;

        /**
         * @brief 
         * 
         * @param n 
         * @param val 
         * @return int 
         */
        int param(const NAVDATA &n, double val) override;

        /** @brief get line. */
        std::string line() const override;

        /** @brief get line format. */
        std::string linefmt() const override;

    private:
        /** @brief get health state. */
        bool _healthy() const override;

        /**
         * @brief 
         * 
         * @param dt 
         * @param Ek 
         * @param dEk 
         */
        void _ecc_anomaly(double dt, double &Ek, double &dEk);

        /**
         * @brief 
         * 
         * @return double 
         */
        double _mean_motion();

    private:
        int _iode;      ///< issue of ephemeris
        int _iodc;      ///< issue of clocks
        int _health;    ///< sv health (0:ok)
        base_time _toe;   ///< time of ephemerides
        base_time _toc;   ///< time of clocks
        base_time _tot;   ///< time of transmission
        double _a;      ///< major semiaxis [m]
        double _e;      ///< eccentricity
        double _m;      ///< mean anomaly at t0 [rad]
        double _i;      ///< inclination [rad]
        double _idot;   ///< inclination change [rad/sec]
        double _omega;  ///< argument of perigee [rad]
        double _OMG;    ///< longit. of ascend. node at weekly epoch [rad]
        double _OMGDOT; ///< longit. of ascend. node at weekly epoch's change [rad/sec]
        double _dn;     ///< mean motion difference, delta n [rad/sec]
        double _crc;    ///< amplit. of cos harm. terms - radius [m]
        double _cic;    ///< amplit. of cos harm. terms - inclination [rad]
        double _cuc;    ///< amplit. of cos harm. terms - arg-of-latitude [rad]
        double _crs;    ///< amplit. of sin harm. terms - radius [m]
        double _cis;    ///< amplit. of sin harm. terms - inclination [rad]
        double _cus;    ///< amplit. of sin harm. terms - arg-of-latitude [rad]
        double _f0;     ///< sv clock parameters [sec]
        double _f1;     ///< sv clock parameters [sec/sec]
        double _f2;     ///< sv clock parameters [sec/sec^2]
        double _acc;    ///< sv accuracy [m]
        double _tgd[4]; ///< group delay parameters [sec]
    };

} // namespace

#endif
