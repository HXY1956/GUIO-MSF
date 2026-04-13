#ifndef hwa_gnss_data_navglo_H
#define hwa_gnss_data_navglo_H

#include "hwa_gnss_data_Nav.h"
#include "hwa_base_time.h"
#include "hwa_base_const.h"
#include "hwa_base_common.h"
#include "hwa_base_eigendef.h"

#define MAX_rinexn_REC_GLO 15
#define MIN_GLO_RADIUS 25300      ///< km
#define MAX_GLO_RADIUS 25700      ///< km
#define MAX_GLO_RADDIF 50         ///< km
#define MAX_GLO_CLKDIF 50         ///< ns
#define RAD_GLO_FACTOR 0.001      ///< m->km
#define CLK_GLO_FACTOR 1000000000 ///< sec->ns

namespace hwa_gnss
{
    /* ura values (ref [3] 20.3.3.3.1.3) [meters] */
    //static const double ura_eph[]=
    //  {2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,3072.0,6144.0,0.0};

    /** @brief class for gnss_data_navglo. */
    class gnss_data_navglo : public gnss_data_nav
    {
\

    public:
        /** @brief default constructor. */
        gnss_data_navglo();

        /**
         * @brief Construct a new t gnavglo object
         * 
         * @param spdlog 
         */
        gnss_data_navglo(hwa_base::base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_navglo();

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
        virtual int nav(const hwa_base::base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true) override;

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
        virtual int pos(const hwa_base::base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true) override;

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
        virtual int clk(const hwa_base::base_time &t, double *clk, double *var = NULL, double *dclk = NULL, bool chk_health = true) override;

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int channel() const;

        /**
         * @brief 
         * 
         * @param msg 
         * @return int 
         */
        virtual int chk(std::set<std::string> &msg) override;

        /** @brief convert data to nav. */
        virtual int data2nav(std::string sat, const hwa_base::base_time &ep, const gnss_data_navDATA &data) override;

        /** @brief convert nav to data. */
        virtual int nav2data(gnss_data_navDATA &data) override;

        /** @brief get iod. */
        virtual int iod() const override { return _iodc; }

        /** @brief get rec. */
        int rec() const override { return MAX_rinexn_REC_GLO; }

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

        /** @brief get freq_num. */
        int freq_num() const override { return _freq_num; }

    protected:
        /**
         * @brief 
         * 
         * @return int 
         */
        int _iod() const;

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool _healthy() const override;

    private:
        /**
         * @brief 
         * 
         * @param xx 
         * @param acc 
         * @return Vector 
         */
        Vector _deriv(const Vector &xx, const Triple &acc);

        /**
         * @brief 
         * 
         * @param step 
         * @param nsteps 
         * @param yy 
         * @param acc 
         * @return Vector 
         */
        Vector _RungeKutta(double step, int nsteps, const Vector &yy, const Triple &acc);

        double _maxEphAge; ///< max age of ephemerises [s]

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
        int _pos(const hwa_base::base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true);

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
        int _clk(const hwa_base::base_time &t, double *clk, double *var = NULL, double *dclk = NULL, bool chk_health = true);

    private:
        int _iodc; ///< issue of clocks

        double _x;      ///< position X [km]
        double _x_d;    ///< velocity X [km/s]
        double _x_dd;   ///< acceleration X [km/s^2]
        double _y;      ///< position Y [km]
        double _y_d;    ///< velocity Y [km/s]
        double _y_dd;   ///< acceleration Y [km/s^2]
        double _z;      ///< position Z [km]
        double _z_d;    ///< velocity Z [km/s]
        double _z_dd;   ///< acceleration Z [km/s^2]
        double _E;      ///< Age of oper. information [days]
        int _freq_num;  ///< frequency number (-7 ... 13)
        double _health; ///< health 0 = OK
        hwa_base::base_time _toc;   ///< Epoch of clocks [s]
        double _gamma;  ///< SV relative frequency bias []
        double _tau;    ///< SV clock bias [s]
        double _tki;    ///< message frame time [0 ... 86400 s]
        int _min_step;  ///< mininal step length for Runge Kutta
    };

} // namespace

#endif
