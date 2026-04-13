#ifndef hwa_gnss_data_navbds_H
#define hwa_gnss_data_navbds_H

#include "hwa_gnss_data_Nav.h"
#include "hwa_base_time.h"
#include "hwa_base_const.h"
#include "hwa_base_common.h"
#include "hwa_base_eigendef.h"

#define MAX_rinexn_REC_BDS 29

namespace hwa_gnss
{
///< ICD BeiDou v2
#define SC2R 3.1415926535898 ///< semi-circle to radian

///< RTCM3 BDS EPH decoding consistency with BNC
//////////////////////////////////////////////////////////
#define BDSTOINT(type, value) static_cast<type>(round(value))
#define BDSADDBITS(a, b)                                                               \
    {                                                                                  \
        bitbuffer = (bitbuffer << (a)) | (BDSTOINT(long long, b) & ((1ULL << a) - 1)); \
        numbits += (a);                                                                \
        while (numbits >= 8)                                                           \
        {                                                                              \
            buffer[size++] = bitbuffer >> (numbits - 8);                               \
            numbits -= 8;                                                              \
        }                                                                              \
    }
#define BDSADDBITSFLOAT(a, b, c)                      \
    {                                                 \
        long long i = BDSTOINT(long long, (b) / (c)); \
        BDSADDBITS(a, i)                              \
    };

    /** @brief Class for bds navigation data storing. */
    class gnss_data_navbds : public gnss_data_nav
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_navbds();

        explicit gnss_data_navbds(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_navbds();

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

        /**
         * @brief 
         * 
         * @param ep 
         * @param data 
         * @return int 
         */
        int data2nav(std::string, const base_time &ep, const gnss_data_navDATA &data) override;

        /**
         * @brief 
         * 
         * @param data 
         * @return int 
         */
        int nav2data(gnss_data_navDATA &data) override;

        /**
         * @brief 
         * 
         * @return int 
         */
        int iod() const override;

        /**
         * @brief 
         * 
         * @return int 
         */
        int rec() const override { return MAX_rinexn_REC_BDS; }

        /**
         * @brief 
         * 
         * @return double 
         */
        double tgd() { return (C02_F * C02_F) / (C02_F * C02_F - C07_F * C07_F) * _tgd[0] - (C07_F * C07_F) / (C02_F * C02_F - C07_F * C07_F) * _tgd[1]; }

        /**
         * @brief 
         * 
         * @return double 
         */
        double rel() { return _rel; }

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        virtual bool chktot(const base_time &t) override;

        /**
         * @brief 
         * 
         * @param n 
         * @return hwa_map_time_value 
         */
        hwa_map_time_value param(const NAVDATA &n) override;

        /**
         * @brief 
         * 
         * @param n 
         * @param val 
         * @return int 
         */
        int param(const NAVDATA &n, double val) override;

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string line() const override;

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string linefmt() const override;

    private:
        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
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

        /**
         * @brief 
         * 
         * @return const int 
         */
        const int _getIODC() const;

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
        double _acc;    ///< sv accuracy index (URA)
        double _tgd[2]; ///< group delay parameters [sec]  0: B1/B3,  1: B2/B3
        double _rel;    ///< relativity correction calculated with ICD
    };

}
#endif
