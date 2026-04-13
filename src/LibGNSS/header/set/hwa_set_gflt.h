#ifndef hwa_set_flt_h
#define hwa_set_flt_h
#include <string>
#include <iostream>
#include "hwa_base_typeconv.h"
#include "hwa_set_base.h"

#define XMLKEY_FLT "filter" ///< The defination of filter node

namespace hwa_set
{
    class set_flt : public virtual set_base
    {
    public:
        /// constructor
        set_flt();

        /// destructor
        ~set_flt();

        /// settings check
        void check();

        /// settings help
        void help();

        /**
         * @brief get the flt method
         * @return std::string : the method of flt
         */
        std::string method_flt();

        /**
         * @brief get the smt method
         * @return std::string : the method of smt
         */
        std::string method_smt();

        /**
         * @brief get the noise of clk in flt
         * @return double : the noise of clk in flt
         */
        double noise_clk();

        /**
         * @brief get the noise of crd in flt
         * @return double : the noise of crd in flt
         */
        double noise_crd();

        /**
         * @brief get the noise of dclk in flt
         * @return double : the noise of dclk in flt
         */
        double noise_dclk();

        /**
         * @brief get the noise of vel in flt
         * @return double : the noise of vel in flt
         */
        double noise_vel();

        /**
        * @brief get the random walk of glonass in flt
        * @return double : the random walk of gps in flt
        */
        double rndwk_gps();

        /**
         * @brief get the random walk of glonass in flt
         * @return double : the random walk of glonass in flt
         */
        double rndwk_glo();

        /**
         * @brief get the random walk of galileo in flt
         * @return double : the random walk of galileo in flt
         */
        double rndwk_gal();

        /**
         * @brief get the random walk of Beidou in flt
         * @return double : the random walk of Beidou in flt
         */
        double rndwk_bds();

        double rndwk_amb(); // lhb added 201908

        /**
         * @brief get the random walk of QZSS in flt
         * @return double : the random walk of QZSS in flt
         */
        double rndwk_qzs();

        /**
         * @brief get the random walk of ztd in flt
         * @return double : the random walk of ztd in flt
         */
        double rndwk_ztd();

        /**
         * @brief get the noise of vion in flt
         * @return double : the noise of vion in flt
         */
        double noise_vion();

        /**
        * @brief get the random walk of ion in flt
        * @return double : the random walk of ion in flt
        */
        double rndwk_vion();

        /**
         * @brief get the random walk of grd in flt
         * @return double : the random walk of grd in flt
         */
        double rndwk_grd();

        /**
         * @brief get the reset value of amb in flt
         * @return int : the reset value of amb in flt
         */
        int reset_amb();

        /**
         * @brief get the reset value of par in flt
         * @return int : the reset value of par in flt
         */
        int reset_par();
        /**
         * @brief get the reset value of par in flt
         * @return int : the reset value of par in flt
         */
        int reset_par(double d);

        /**
         * @brief get the delay value of smt in flt
         * @return int : the delay value of smt in flt
         */
        int smt_delay();

        /**
         * @brief get the std::set of smoothing in flt or not
         * @return bool : smooth in flt or not
         */
        bool smooth();

        double barrior();

        double kappa_sig();

        double alpha_sig();

        double E0();

        double G0();

        int dof1();

        int dof2();

        int max_iter();

        int num_particles();

        double Tau();

        double proc_noise();

        double max_res_norm();

        std::string filter();

    protected:
        std::string _method_flt; ///< type of filtering method (kalman, SRCF)
        std::string _method_smt; ///< type of filtering method (kalman, SRCF)
        double _noise_clk;  ///< white noise for receiver clock [m]
        double _noise_crd;  ///< white noise for coordinates [m]
        double _noise_dclk; ///< white noise for receiver clock speed [m/s]
        double _noise_vel;  ///< white noise for velocities [m/s]
        double _rndwk_gps;
        double _rndwk_glo;  ///< random walk process for GLONASS system time offset
        double _rndwk_gal;  ///< random walk process for Galileo system time offset
        double _rndwk_bds;  ///< random walk process for BeiDou system time offset
        double _rndwk_qzs;  ///< random walk process for ZQSS system time offset
        double _rndwk_ztd;  ///< random walk process for ZTD [mm/sqrt(hour)]
        double _rndwk_vion; ///< random walk process for VION [mm/sqrt(hour)]
        double _noise_vion; ///< white noise process for VION [m]
        double _rndwk_grd;  ///< random walk process for tropo grad [mm/sqrt(hour)]
        double _rndwk_amb;  ///< random walk process for ambiguity  [mm/sqrt(hour)]
        int _reset_amb;     ///< interval for reseting ambiguity [s]
        int _reset_par;     ///< interval for reseting CRD, ZTD, AMB [s]
        bool _smooth;       ///< backward smoothing
        int _smt_delay;     ///< delay for backwart smoothing [s]
        double _e0;
        double _g0;
        double _max_res_norm;
        double _barrior;
        double _kappa_sig;
        double _alpha_sig;
        int _dof1;
        int _dof2;
        double _tau;
        double _proc_noise;
        int _max_iter;
        int _num_particles;

    private:
    };

} // namespace

#endif
