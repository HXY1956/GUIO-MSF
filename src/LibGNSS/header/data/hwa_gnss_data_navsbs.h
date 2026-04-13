
#ifndef hwa_gnss_data_navsbs_H
#define hwa_gnss_data_navsbs_H

#include "hwa_gnss_data_Nav.h"
#include "hwa_base_time.h"
#include "hwa_base_const.h"
#include "hwa_base_common.h"
#include "hwa_base_eigendef.h"

#define MAX_rinexn_REC_SBS 15
#define MIN_SBS_RADIUS 25400       // km
#define MAX_SBS_RADIUS 25600       // km
#define MAX_SBS_RADDIF 25          // km
#define MAX_SBS_CLKDIF 15          // ns
#define RAD_SBS_FACTOR 0.001       // m->km
#define CLK_SBS_FACTOR 1000000000  // sec->ns

namespace hwa_gnss
{
    /* ura values (ref [3] 20.3.3.3.1.3) [meters] */
    //static const double ura_eph[]=
    //  {2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,3072.0,6144.0,0.0};

    /** @brief class for gnss_data_navsbs. */
    class gnss_data_navsbs : public gnss_data_nav
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_navsbs();

        /**
         * @brief Construct a new t gnavsbs object
         * 
         * @param spdlog 
         */
        explicit gnss_data_navsbs(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_navsbs();

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
        virtual int pos(const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true) override;

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
        virtual int clk(const base_time &t, double *clk, double *var = NULL, double *dclk = NULL, bool chk_health = true) override;

        /**
         * @brief 
         * 
         * @param msg 
         * @return int 
         */
        int chk(std::set<std::string> &msg) override;

        /** @brief convert data to nav. */
        int data2nav(std::string sat, const base_time &ep, const gnss_data_navDATA &data) override;

        /** @brief convert nav to data. */
        int nav2data(gnss_data_navDATA &data) override;

        /** @brief get iod. */
        int iod() const override { return _iod; }

        /** @brief get rec. */
        int rec() const override { return MAX_rinexn_REC_SBS; }

        /** @brief get/std::set parameter. */
        virtual hwa_map_time_value param(const NAVDATA &n) override;

        /**
         * @brief 
         * 
         * @param n 
         * @param val 
         * @return int 
         */
        virtual int param(const NAVDATA &n, double val) override;

        /** @brief get line. */
        std::string line() const override;

        /** @brief get line format. */
        std::string linefmt() const override;

    protected:
        /** @brief get health. */
        bool _healthy() const override;

    private:
        double _x;      ///< position X [km]
        double _x_d;    ///< velocity X [km/s]
        double _x_dd;   ///< acceleration X [km/s^2]
        double _y;      ///< position Y [km]
        double _y_d;    ///< velocity Y [km/s]
        double _y_dd;   ///< acceleration Y [km/s^2]
        double _z;      ///< position Z [km]
        double _z_d;    ///< velocity Z [km/s]
        double _z_dd;   ///< acceleration Z [km/s^2]
        double _f0;     ///< SV clock bias [s]
        double _tki;    ///< Transmission time of message in GPS seconds of weak
        double _health; ///< health 0 = OK
        double _C_rms;  ///< Accuracy code [m]
        int _iod;       ///< Issue of Data Navigation
        double _f1;     ///< SV relative frequency bias []
        base_time _toc;   ///< Epoch of ephemerides
    };

} // namespace

#endif
