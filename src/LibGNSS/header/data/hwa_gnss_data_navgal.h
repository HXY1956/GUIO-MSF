#ifndef hwa_gnss_data_navgal_H
#define hwa_gnss_data_navgal_H

#include "hwa_gnss_data_Nav.h"
#include "hwa_base_time.h"
#include "hwa_base_const.h"
#include "hwa_base_common.h"
#include "hwa_base_eigendef.h"

#define MAX_rinexn_REC_GAL 28

namespace hwa_gnss
{
#define SC2R 3.1415926535898 ///< Ratio of a circle's circumference to its diameter

    // ura values (ref [3] 20.3.3.3.1.3) [meters]
    //static const double ura_eph[]=
    //{2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,3072.0,6144.0,0.0};

    class gnss_data_navgal : public gnss_data_nav
    {
    public:
        /** @brief default constructor. */
        gnss_data_navgal();

        /**
         * @brief Construct a new t gnavgal object
         * 
         * @param spdlog 
         */
        gnss_data_navgal(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_navgal();

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
         * @param full 
         * @return GNAVTYPE 
         */
        GNAVTYPE gnavtype(bool full = true) const override; ///< distinguish INAV/FNAV or in full details

        /**
         * @brief 
         * 
         * @param full 
         * @return int 
         */
        int src(bool full = true) const override; ///< distinguish INAV/FNAV or in full details

        /** @brief get iod value. */
        int iod() const override { return _iode; }

        /** @brief get rec value. */
        int rec() const override { return MAX_rinexn_REC_GAL; }

        /** @brief get _rec value. */
        double rel() { return _rel; }

        /** @brief check tot. */
        virtual bool chktot(const base_time &t) override;

        /** @brief get parameter. */
        hwa_map_time_value param(const NAVDATA &n) override;

        /** @brief get parameter. */
        int param(const NAVDATA &n, double val) override;

        /** @brief get line. */
        std::string line() const override;

        /** @brief get line format. */
        std::string linefmt() const override;

        /** @brief convert health to std::string. */
        std::string health_str() const override { return _health_str(); }

    protected:
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
         * @return std::string 
         */
        std::string _health_str() const override;

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string _source_str() const;

        /**
         * @brief 
         * 
         * @param full 
         * @return GNAVTYPE 
         */
        GNAVTYPE _gnavtype(bool full = true) const;

    private:
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

        int _iode;      ///< issue of ephemeris
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
        double _tgd[2]; ///< [seconds]  0:E5a/E1   1:E5b/E1
        int _source;    ///< data source
        double _sisa;   ///< SISA Signal in space accuracy [meters]
        double _rel;    ///<relativity correction calculated with ICD
    };

} // namespace

#endif
