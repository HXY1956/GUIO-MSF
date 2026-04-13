#ifndef hwa_gnss_all_prec_H
#define hwa_gnss_all_prec_H

#include "hwa_gnss_all_nav.h"
#include "hwa_gnss_data_ephprec.h"
#include "hwa_gnss_model_poly.h"

namespace hwa_gnss
{
    typedef std::map<std::string, double> hwa_map_iv;     ///< single data for a single satellite
    typedef std::map<base_time, hwa_map_iv> hwa_map_tiv; ///< all data for a single satellite
    typedef std::map<std::string, hwa_map_tiv> hwa_map_ITIV;  ///< all data for all satellites

    /**
    *@brief Class for gnss_all_prec derive from gnss_all_nav
    */
    class gnss_all_prec : public gnss_all_nav
    {
    public:
        /** @brief clk type. */
        enum clk_type
        {
            AS,   ///< sitellite
            AR,   ///< receiver
            UNDEF ///< undefined
        };

    public:
        /** @brief default constructor. */
        gnss_all_prec();

        /**
         * @brief Construct a new t gallprec object
         * 
         * @param spdlog 
         */
        gnss_all_prec(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_prec();

        typedef std::map<std::string, std::shared_ptr<gnss_data_ephprec>> hwa_map_EPHPREC; ///< std::map sp3

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @return true 
         * @return false 
         */
        bool health(const std::string &sat, const base_time &t) override; ///< inherited from gallnav to fix healthy

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param xyz 
         * @param var 
         * @param vel 
         * @param chk_mask 
         * @return int 
         */
        int pos(const std::string &sat, const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, const bool &chk_mask = true); // [m]

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param xyz 
         * @param var 
         * @param vel 
         * @param chk_mask 
         * @return int 
         */
        int nav(const std::string &sat, const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, const bool &chk_mask = true) override; // [m] GNAV quality

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @param chk_mask 
         * @return int 
         */
        int clk(const std::string &sat, const base_time &t, double *clk, double *var = NULL, double *dclk = NULL, const bool &chk_mask = true) override; // [s]

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @param chk_mask 
         * @return int 
         */
        int clk_ifcb(const std::string &sat, const base_time &t, double *clk, double *var = NULL, double *dclk = NULL, const bool &chk_mask = true) override; // [s] add by xiongyun

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @param ifcb 
         * @param chk_mask 
         * @return int 
         */
        int clk_cdr(const std::string &sat, const base_time &t, double *clk, double *var = NULL, double *dclk = NULL, double *ifcb = NULL, const bool &chk_mask = true); //add ifcb by xiongyun

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @return int 
         */
        int clk_sp3(const std::string &sat, const base_time &t, double *clk, double *var = NULL, double *dclk = NULL); // [s]

        // ALTERNATIVES for direct interpolations - using gephprec!
        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param xyz 
         * @param var 
         * @param vel 
         * @return int 
         */
        int pos_int(const std::string &sat, const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL); // [m]

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @return int 
         */
        int clk_int(const std::string &sat, const base_time &t, double *clk, double *var = NULL, double *dclk = NULL); // [s]

        // ALTERNATIVES for direct interpolations !
        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param xyz 
         * @param var 
         * @param vel 
         * @return int 
         */
        int pos_alt(const std::string &sat, const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL); // [m]

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param clk 
         * @param var 
         * @param dclk 
         * @return int 
         */
        int clk_alt(const std::string &sat, const base_time &t, double *clk, double *var = NULL, double *dclk = NULL); // [s]

        /**
         * @brief 
         * 
         * @param sat 
         * @return int 
         */
        int intv(const std::string &sat);

        /**
         * @brief 
         * 
         * @return int 
         */
        int intv();

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param xyz 
         * @return true 
         * @return false 
         */
        bool posnew(const std::string &sat, const base_time &t, double xyz[3]);

        /**
         * @brief 
         * 
         * @param sat 
         * @param intv 
         */
        void add_interval(const std::string &sat, int intv);

        /**
         * @brief 
         * 
         * @param intv 
         */
        void add_interval(const int &intv);

        /**
         * @brief 
         * 
         * @param producer 
         */
        void add_agency(const std::string &producer);

        /**
         * @brief 
         * 
         * @param intv 
         */
        void add_clk_interval(const double &intv);

        /**
         * @brief 
         * 
         * @param intv 
         */
        void add_orb_interval(const double &intv);

        /**
         * @brief 
         * 
         * @param sat 
         * @param ep 
         * @param xyzt 
         * @param var 
         * @return int 
         */
        int addvel(const std::string &sat, const base_time &ep, double xyzt[4], double var[4]);

        /**
         * @brief 
         * 
         * @param sat 
         * @param ep 
         * @param clk 
         * @param var 
         * @return * int 
         */
        int addclk(const std::string &sat, const base_time &ep, double clk[3], double var[3]);

        /**
         * @brief 
         * 
         * @param sat 
         * @param ep 
         * @param clk 
         * @param var 
         * @return * int 
         */
        int addclk_tri(const std::string &sat, const base_time &ep, double clk[3], double var[3]);

        /**
         * @brief 
         * 
         * @param sat 
         * @param ep 
         * @param xyz 
         * @param t 
         * @param dxyz 
         * @param dt 
         * @param xyzt 
         * @param var 
         * @param obs_num 
         * @return * int 
         */
        int add_pos_vel(const std::string &sat, const base_time &ep, const Triple &xyz, const double &t, const Triple &dxyz, const double &dt, double xyzt[4], double var[4], const int &obs_num = 0); // [s]

        /**
         * @brief 
         * 
         * @param sat 
         * @param ep 
         * @param xyz 
         * @param t 
         * @param dxyz 
         * @param dt 
         * @return int 
         */
        int addpos(const std::string &sat, const base_time &ep, const Triple &xyz, const double &t, const Triple &dxyz, const double &dt); // [m], [usec]                                                                                                                                                        /** @brief add the ssr position and velocity correction. added by zhShen */

        /**
         * @brief 
         * 
         * @param sat 
         * @param ep 
         * @param iod 
         * @param dxyz 
         * @param dvxyz 
         * @return int 
         */
        int add_delta_pos_vel(const std::string &sat, const base_time &ep, const int &iod, const Triple &dxyz, const Triple &dvxyz);

        /** @brief add the ssr clk correction. added by zhShen */
        int add_delta_clk(const std::string &sat, const base_time &ep, const int &iod, const double &dt, const double &dot_dt, const double &dot_dot_dt);

        /**
         * @brief Get the pos clk correction object
         * 
         * @param sat 
         * @param t 
         * @param xyz 
         * @param vxyz 
         * @param clk 
         * @param dclk 
         * @return int 
         */
        virtual int get_pos_clk_correction(const std::string &sat, const base_time &t, double *xyz, double *vxyz, double &clk, double &dclk);

        /**
         * @brief Get the pos clk correction object
         * 
         * @param sat 
         * @param t 
         * @param iod 
         * @param xyz 
         * @param vxyz 
         * @param clk 
         * @param dclk 
         * @return int 
         */
        virtual int get_pos_clk_correction(const std::string &sat, const base_time &t, const int& iod, double *xyz, double *vxyz, double &clk, double &dclk);

        /*added by xiongyun, get ssr clk correction*/
        int get_clk_correction(const std::string &sat, const base_time &t, const int &iod, double &clk, double &dclk);

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool corr_avali();

        /**
         * @brief 
         * 
         * @param now 
         * @return true 
         * @return false 
         */
        bool corr_avali(const base_time &now);

        /**
         * @brief 
         * 
         * @param b 
         */
        void use_clkrnx(const bool &b) { _clkrnx = b; }

        /**
         * @brief 
         * 
         * @param b 
         */
        void use_clksp3(const bool &b) { _clksp3 = b; }

        /**
         * @brief 
         * 
         * @param b 
         */
        void use_clknav(const bool &b) { _clknav = b; }

        /**
         * @brief 
         * 
         * @param b 
         */
        void use_posnav(const bool &b) { _posnav = b; }

        /**
         * @brief 
         * 
         * @param b 
         */
        void use_ultrasp3(const bool &b) { _ultrasp3 = b; }

        /**
         * @brief 
         * 
         */
        void clean_all()
        {
            _mapsp3.clear();
            _mapprec.clear();
        }

        /**
         * @brief 
         * 
         * @param beg 
         * @param end 
         */
        void clean_outer(const base_time &beg = FIRST_TIME, const base_time &end = LAST_TIME) override;

        /**
         * @brief Get the beg object
         * 
         * @return base_time 
         */
        base_time get_beg();

        /**
         * @brief Get the end object
         * 
         * @return base_time 
         */
        base_time get_end();

        /**
         * @brief Get the agency object
         * 
         * @return std::string 
         */
        std::string get_agency();

        /**
         * @brief Get the sats object
         * 
         * @return std::vector<std::string> 
         */
        std::vector<std::string> get_sats();

        /**
         * @brief Get the sat3 object
         * 
         * @return std::vector<std::string> 
         */
        std::vector<std::string> get_sat3();

        /**
         * @brief Get the data type object
         * 
         * @return std::string 
         */
        std::string get_data_type();

        /**
         * @brief Get the sat type object
         * 
         * @return std::string 
         */
        std::string get_sat_type();

        /**
         * @brief Get the sp3 size object
         * 
         * @return int 
         */
        int gegnss_coder_sp3_size();

        /**
         * @brief Get the clksp3 object
         * 
         * @return true 
         * @return false 
         */
        bool get_clksp3() { return _clksp3; } //lvhb added for brdc in 20201218

        /**
         * @brief Set the beg object
         * 
         * @param beg 
         */
        void set_beg(const base_time &beg);

        /**
         * @brief Set the end object
         * 
         * @param end 
         * @param mode 
         */
        void set_end(const base_time &end, const std::string &mode = "");

        /**
         * @brief Set the sat object
         * 
         * @param sat 
         */
        void set_sat(const std::vector<std::string> &sat);

        /**
         * @brief Set the sat3 object
         * 
         * @param sat 
         */
        void set_sat3(const std::vector<std::string> &sat);

        /**
         * @brief Set the data type object
         * 
         * @param type 
         */
        void set_data_type(const std::string &type);

        /**
         * @brief Set the agency object
         * 
         * @param agency 
         */
        void set_agency(const std::string &agency);

        /**
         * @brief Set the sat type object
         * 
         * @param type 
         */
        void set_sat_type(const std::string &type);

        /**
         * @brief Get the pos vel object
         * 
         * @param sat 
         * @param epoch 
         * @param pos 
         * @param vel 
         * @param obsnum 
         * @return true 
         * @return false 
         */
        bool get_pos_vel(const std::string &sat, const base_time &epoch, double pos[3], double vel[3], int &obsnum);

        virtual int get_ssr_iod(const std::string& sat, const base_time& t, int& pv_iod, int& clk_iod);  //from tyx

        /**
         * @brief 
         * 
         * @return std::set<std::string> 
         */
        virtual std::set<std::string> satellites() const override; // get all satellites

        /**
         * @brief 
         * 
         * @param prn 
         * @return unsigned int 
         */
        virtual unsigned int nepochs(const std::string &prn) override; // get number of epochs

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time beg_data(const std::string &prn = ""); // get first gnss_data_ephprec epoch

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time end_data(const std::string &prn = ""); // get  last gnss_data_ephprec epoch

        // IMPROVE beg_time/end_time to distinguish GALLNAV/GALLPREC - t_satview !
        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time beg_time(const std::string &prn = "") const override { return beg_gnav(prn); } // get first gnss_data_ephprec epoch

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time end_time(const std::string &prn = "") const override { return end_gnav(prn); } // get  last gnss_data_ephprec epoch
        // =========================================================

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time beg_clk(const std::string &prn = ""); // get first precise clocks epoch

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time end_clk(const std::string &prn = ""); // get last precise clocks epoch

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time beg_prec(const std::string &prn = ""); // get first precise polynomials epoch

        /**
         * @brief 
         * 
         * @param prn 
         * @return base_time 
         */
        virtual base_time end_prec(const std::string &prn = ""); // get last precise polynomials epoch

        /**
         * @brief 
         * 
         * @return std::set<std::string> 
         */
        virtual std::set<std::string> clk_objs(); // get all clk satellites and receivers

        /**
         * @brief 
         * 
         * @return std::set<base_time> 
         */
        virtual std::set<base_time> clk_epochs();

        /**
         * @brief Get the clk type object
         * 
         * @return std::set<clk_type> 
         */
        std::set<clk_type> get_clk_type() const;

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param crd 
         * @return int 
         */
        virtual int lagrange_pos(const std::string &sat, const base_time &t, Triple &crd) override; // add by glfeng to lagrange_interpolate

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param crd 
         * @return int 
         */
        virtual int lagrange_vel(const std::string &sat, const base_time &t, Triple &crd) override; // add by yjqin to lagrange_interpolate

    protected:
        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @return std::shared_ptr<gnss_data_eph> 
         */
        virtual std::shared_ptr<gnss_data_eph> _find(const std::string &sat, const base_time &t); // find appropriate gnss_data_eph element

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @return int 
         */
        virtual int _get_crddata(const std::string &sat, const base_time &t); // fill PT,X,Y,Z std::vectors

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @return int 
         */
        virtual int _get_clkdata(const std::string &sat, const base_time &t); // fill CT,C std::vectors

        /** @brief return the ssr position and velocity correction. added by zhShen */
        virtual int _get_delta_pos_vel(const std::string &sat, const base_time &t);

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param iod 
         * @param tRef 
         * @param orbcorr 
         * @return int 
         */
        virtual int _get_delta_pos_vel(const std::string &sat, const base_time &t, int iod, base_time &tRef, hwa_map_iv &orbcorr);

        /** @brief return the ssr position and velocity correction. added by zhShen */
        virtual int _get_delta_clk(const std::string &sat, const base_time &t);

        /**
         * @brief 
         * 
         * @param sat 
         * @param t 
         * @param iod 
         * @param tRef 
         * @param clkcorr 
         * @return int 
         */
        virtual int _get_delta_clk(const std::string &sat, const base_time &t, int iod, base_time &tRef, hwa_map_iv &clkcorr);

        // also gallnav member can be used here !!!
        hwa_map_sat _mapprec; // std::map of sp3 polynomials
        hwa_map_ITIV _mapsp3;  // precise orbits&clocks (SP3) - full discrete data setS
        hwa_map_ITIV _mapclk;  // precise clocks (CLOCK-RINEX) - full discrete data setS

    private:
        hwa_map_EPHPREC _prec;          ///< CACHE: single SP3 precise ephemeris for all satellites
        unsigned int _degree_sp3; ///< polynom degree for satellite sp3 position and clocks
        double _sec;              ///< default polynomial units
        base_time _ref;             ///< selected reference epoch for crd data/polynomials
        base_time _clkref;          ///< selected reference epoch for clk data/polynomials
        bool _clkrnx;             ///< true: use               clk from Rinex Clocks
        bool _clksp3;             ///< true: use alternatively clk from sp3 (~15min!)
        bool _clknav;             ///< true: use alternatively nav (low-precise clocks)
        bool _posnav;             ///< true: use alternatively nav (low-precise orbits)
        bool _ultrasp3;           ///< whether use sp3 file in realtime (added by glfeng)
        bool _realtime;           ///< real time process(added by zhShen)
        unsigned int _udclkInt;   ///< SSR clk interval (added by zhShen)
        unsigned int _udorbInt;   ///< SSR orb interval (added by zhShen)
        int _intv;
        std::map<std::string, int> _intvm;
        std::string _agency;
        base_time _tbeg;
        base_time _tend, _tend_clk, _tend_sp3;
        std::vector<std::string> _sats;
        std::vector<std::string> _sat3;
        std::string _datatype;
        std::string _sattype;
        std::string _frame;
        // CACHE for approximative solutions
        std::map<std::string, base_time> _poly_beg;
        std::map<std::string, base_time> _poly_end;
        std::map<std::string, gnss_model_poly> _poly_x;
        std::map<std::string, gnss_model_poly> _poly_y;
        std::map<std::string, gnss_model_poly> _poly_z;

        // BEGIN OF TEMPORARY (ALTERNATIVE for direct interpolation)
        std::vector<double> _PT; ///< std::vector of time-difference (X for polynomials)
        std::vector<base_time> _T; ///< std::vector of full time       (X for polynomials)
        std::vector<double> _X;  ///< std::vector of x-coordinate    (Y for polynomials)
        std::vector<double> _Y;  ///< std::vector of y-coordinate    (Y for polynomials)
        std::vector<double> _Z;  ///< std::vector of z-coordinate    (Y for polynomials)

        std::vector<double> _CT;      ///< std::vector of time-difference (X for polynomials)
        std::vector<double> _C;       ///< std::vector of clk correction  (Y for polynomials)
        std::vector<double> _ifcb_F3; ///< std::vector of ifcb_f3 (add by xiongyun)

        std::vector<double> _PTCorr; ///< std::vector of time-difference for SSR Correction(X for polynomials)
        std::vector<base_time> _TCorr; ///< std::vector of full time for SSR Correction      (X for polynomials)
        std::vector<double> _XCorr;  ///< std::vector of x-coordinate for SSR Correction   (Y for polynomials)
        std::vector<double> _YCorr;  ///< std::vector of y-coordinate for SSR Correction   (Y for polynomials)
        std::vector<double> _ZCorr;  ///< std::vector of z-coordinate for SSR Correction   (Y for polynomials)

        std::vector<double> _CTCorr;       ///< std::vector of time-difference for SSR Correction(X for polynomials)
        std::vector<double> _CCorr;        ///< std::vector of clk correction for SSR Correction (Y for polynomials)
                                      ///< END OF TEMPORARY (ALTERNATIVE)
        std::set<clk_type> _clk_type_list; ///< CLK TYPE
    };

} // namespace

#endif
