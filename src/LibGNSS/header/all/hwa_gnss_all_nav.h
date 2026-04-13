#ifndef hwa_gnss_all_nav_H
#define hwa_gnss_all_nav_H

#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_gnss_tetrad.h"
#include "hwa_gnss_data_Eph.h"
#include "hwa_gnss_data_Nav.h"
#include "hwa_gnss_data_rxnhdr.h"
#include "hwa_gnss_data_navglo.h" 
#include "hwa_gnss_data_navgps.h" 
#include "hwa_gnss_data_NavBDS.h"  
#include "hwa_gnss_data_navgal.h"  
#include "hwa_gnss_data_navqzs.h"   
#include "hwa_gnss_data_navsbs.h"   

#define MAX_GPS_PRN 32
#define MAX_GLO_PRN 24
#define MAX_GAL_PRN 30
#define NAV_BUF 1024

namespace hwa_gnss
{
    /**
     *@brief Class for navigation system setting derive from base_data
     */
    class gnss_all_nav : public hwa_base::base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_all_nav();

        explicit gnss_all_nav(hwa_base::base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_all_nav();

        /// @relates gnss_all_nav
        ///< first : time, second : shared ptr
        typedef std::multimap<hwa_base::base_time, std::shared_ptr<gnss_data_eph>> hwa_map_ref; ///<  all data for a single satellite

        /// @relates gnss_all_nav
        ///< first : name, second : ref
        typedef std::map<std::string, hwa_map_ref> hwa_map_sat; ///<  all data for all satellites

        /**
        * @brief std::set nav healthy status control.
        *
        * @param[in]  b    checking satellite health
        * @return    void
        */
        void chk_health(const bool &b) { _chkHealth = b; }

        /**
        * @brief std::set nav internal quality control.
        *
        * @param[in]  b    nav internal quality control
        * @return    void
        */
        void chk_navig(const bool &b) { _chkNavig = b; }

        /**
        * @brief test if tot < t.
        *
        * @param[in]  b    if tot < t
        * @return    void
        */
        void chk_tot(const bool &b) { _chkTot = b; }

        /**
        * @brief get satellite health.
        *
        * @param[in]  sat    satellite
        * @param[in]  t        time
        * @return    status
        */
        virtual bool health(const std::string &sat, const hwa_base::base_time &t);

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
        virtual int nav(const std::string &sat,
                        const hwa_base::base_time &t,
                        double xyz[3] = NULL,
                        double var[3] = NULL,
                        double vel[3] = NULL,
                        const bool &chk_mask = true); ///< [m]

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
        virtual int pos(const std::string &sat,
                        const hwa_base::base_time &t,
                        double xyz[3] = NULL,
                        double var[3] = NULL,
                        double vel[3] = NULL,
                        const bool &chk_mask = true); ///< [m]

        /**
         * @brief 
         * 
         * @param sat 
         * @param iod 
         * @param t 
         * @param xyz 
         * @param var 
         * @param vel 
         * @param chk_mask 
         * @return int 
         */
        virtual int pos(const std::string &sat,
                        const int &iod,
                        const hwa_base::base_time &t,
                        double xyz[3] = NULL,
                        double var[3] = NULL,
                        double vel[3] = NULL,
                        const bool &chk_mask = true); ///< [m] add by glfeng

        /**
        * @brief return clock corrections.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  clk        clock offset
        * @param[in]  var        
        * @param[in]  dclk        difference of clock offset
        * @param[in]  chk_mask    
        * @return    irc
        */
        virtual int clk(const std::string &sat,
                        const hwa_base::base_time &t,
                        double *clk = NULL,
                        double *var = NULL,
                        double *dclk = NULL,
                        const bool &chk_mask = true);

        /**
        * @brief return clock corrections.
        *
        * @param[in]  sat        satellite
        * @param[in]  iod        issue of data 
        * @param[in]  t            time
        * @param[in]  clk        clock offset
        * @param[in]  var
        * @param[in]  dclk        difference of clock offset
        * @param[in]  chk_mask
        * @return    irc
        */
        virtual int clk(const std::string &sat,
                        const int &iod,
                        const hwa_base::base_time &t,
                        double *clk = NULL,
                        double *var = NULL,
                        double *dclk = NULL,
                        const bool &chk_mask = true);

        /**
        * @brief Clock offset-Inter-Frequency Clock Bias.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  clk        clock offset
        * @param[in]  var
        * @param[in]  dclk        difference of clock offset
        * @param[in]  chk_mask
        * @return    1
        */
        virtual int clk_ifcb(const std::string &sat,
                             const hwa_base::base_time &t,
                             double *clk = NULL,
                             double *var = NULL,
                             double *dclk = NULL,
                             const bool &chk_mask = true);

        /**
        * @brief for lagranging interpolate.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  crd        coordinate
        * @return    0
        */
        virtual int lagrange_pos(const std::string &sat, const hwa_base::base_time &t, Triple &crd) { return 0; };

        /**
        * @brief for lagranging interpolate.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  crd        coordinate
        * @return    0
        */
        virtual int lagrange_vel(const std::string &sat, const hwa_base::base_time &t, Triple &crd) { return 0; };

        /**
        * @brief get pos clock offset correction.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  iod        issue of data
        * @param[in]  xyz        coordinate
        * @param[in]  vxyz        speed
        * @param[in]  clk        clock offset
        * @param[in]  dclk        difference of clock offset
        * @return    -1
        */
        virtual int get_pos_clk_correction(const std::string &sat,
                                           const hwa_base::base_time &t,
                                           const int &iod,
                                           double *xyz,
                                           double *vxyz,
                                           double &clk,
                                           double &dclk);

        /**
        * @brief get iod.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @return    iod
        */
        virtual std::pair<int, int> get_iod(const std::string &sat, const hwa_base::base_time &t);

        /**
        * @brief print sats in time t.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @return    void
        */
        virtual void print(const std::string &sat, const hwa_base::base_time &t);

        /**
        * @brief clean invalid messages.
        *
        * @return    void
        */
        virtual void clean_invalid();

        /**
        * @brief clean duplicit messages.
        *
        * @return    void
        */
        virtual void clean_duplicit();

        /**
        * @brief clean function.
        *
        * @param[in]  beg        begin time
        * @param[in]  end        end time
        * @return    void
        */
        virtual void clean_outer(const hwa_base::base_time &beg = hwa_base::FIRST_TIME,
                                 const hwa_base::base_time &end = hwa_base::LAST_TIME);

        /**
        * @brief get first gnss_data_nav epoch.
        *
        * @param[in]  prn    satellite prn
        * @return    tmp
        */
        virtual hwa_base::base_time beg_gnav(const std::string &prn = "") const;

        /**
        * @brief get last gnss_data_nav epoch.
        *
        * @param[in]  prn    satellite prn
        * @return    tmp
        */
        virtual hwa_base::base_time end_gnav(const std::string &prn = "") const;

        // IMPROVE beg_time/end_time to distinguish GALLNAV/GALLPREC - t_satview !
        /**
        * @brief get first gnss_data_nav epoch.
        *
        * @param[in]  prn    satellite prn
        * @return    beg_gnav epoch
        */
        virtual hwa_base::base_time beg_time(const std::string &prn = "") const { return beg_gnav(prn); }

        /**
        * @brief get last gnss_data_nav epoch.
        *
        * @param[in]  prn    satellite prn
        * @return    end_gnav epoch
        */
        virtual hwa_base::base_time end_time(const std::string &prn = "") const { return end_gnav(prn); }
        // =========================================================

        /**
        * @brief get all satellites.
        *
        * @return    all satellites
        */
        virtual std::set<std::string> satellites() const;

        /**
        * @brief get all systems.
        *
        * @return    all systems
        */
        virtual std::set<GSYS> systems() const;

        /**
        * @brief add single navigation message.
        *
        * @param[in]  nav    navigation system
        * @return    0
        */
        virtual int add(std::shared_ptr<gnss_data_nav> nav);

        /**
        * @brief get number of epochs.
        *
        * @param[in]  prn    satellite prn
        * @return    number of epochs
        */
        virtual unsigned int nepochs(const std::string &prn);

        /**
        * @brief get list of nav epochs (all=default, G=system, G01=prn).
        *
        * @param[in]  prn    satellite prn
        * @return    list of epochs
        */
        virtual std::set<hwa_base::base_time> epochs(const std::string &prn = "");

        /**
        * @brief get list of nav messages.
        *
        * @param[in]  prn    satellite prn
        * @return    navigation messages
        */
        virtual std::vector<std::shared_ptr<gnss_data_eph>> vec_nav(const std::string &prn = "");

        /**
        * @brief get list of calculated crd.
        *
        * @param[in]  prn    satellite prn
        * @param[in]  beg    begin time
        * @return    list of calculated crd
        */
        virtual std::map<std::string, Triple> map_xyz(const std::set<std::string> &prns, const hwa_base::base_time &beg);

        /**
        * @brief get list of calculated clk.
        *
        * @param[in]  prn    satellite prn
        * @param[in]  beg    begin time
        * @return    list of calculated clk
        */
        virtual std::map<std::string, double> map_clk(const std::set<std::string> &prns, const hwa_base::base_time &beg);

        /**
         * @brief 
         * 
         * @param prns 
         * @param epo 
         * @return std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, Triple>> 
         */
        std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, Triple>> multi_xyz(const std::set<std::string> &prns, const hwa_base::base_time &epo); ///< list of calculated pos from all redundant navig. messages

        /**
         * @brief 
         * 
         * @param prns 
         * @param epo 
         * @return std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, double>> 
         */
        std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, double>> multi_clk(const std::set<std::string> &prns, const hwa_base::base_time &epo); ///< list of calculated clk from all redundant navig. messages

        /**
        * @brief std::set/get std::multimap mode.
        *
        * @param[in]  b    std::multimap mode
        * @return    void
        */
        void multi(const bool &b) { _multimap = b; }

        /**
        * @brief return flag of std::multimap.
        *
        * @return    std::multimap
        */
        const bool &multi() const { return _multimap; }

        /**
        * @brief std::set/get overwrite mode.
        *
        * @param[in]  b    overwrite mode
        * @return    void
        */
        void overwrite(const bool &b) { _overwrite = b; }

        /**
        * @brief .. not for std::multimap, but derived classe.
        *
        * @return    overwrite
        */
        const bool &overwrite() const { return _overwrite; }

        /**
        * @brief position/clock reference point.
        *
        * @param[in]  b    
        * @return    void
        */
        void com(const bool &b) { _com = b; }

        /**
        * @brief position/clock reference point.
        *
        * @return    com
        */
        const bool &com() const { return _com; }

        /**
        * @brief std::set/get offset for satdata.
        *
        * @param[in]  i    offset
        * @return    void
        */
        void offset(const int &i) { _offset = i; }

        /**
        * @brief return offset.
        *
        * @return    offset
        */
        virtual const int &offset() const { return _offset; }

        /**
        * @brief get number of satellites.
        *
        * @param[in]  gs        navigation system
        * @return    number of satellites
        */
        virtual int nsat(const GSYS &gs) const;

        /**
        * @brief get interval between messages.
        *
        * @param[in]  gs        navigation system
        * @return    interval
        */
        virtual int intv(const GSYS &gs) const;

        /**
        * @brief get existing number of messages.
        *
        * @param[in]  gs        navigation system
        * @param[in]  beg    begin time
        * @param[in]  end    end time
        * @return    existing
        */
        virtual int have(const GSYS &gs, const hwa_base::base_time &beg, const hwa_base::base_time &end) const;

        /**
        * @brief get expected number of messages.
        *
        * @param[in]  gs        navigation system
        * @param[in]  beg    begin time
        * @param[in]  end    end time
        * @return    expected number of messages
        */
        int expt(const GSYS &gs, const hwa_base::base_time &beg, const hwa_base::base_time &end) const;

        /**
        * @brief get excluded number of messages.
        *
        * @param[in]  gs        navigation system
        * @param[in]  beg    begin time
        * @param[in]  end    end time
        * @return    excluded number of messages
        */
        int excl(const GSYS &gs, const hwa_base::base_time &beg, const hwa_base::base_time &end) const;

        /**
        * @brief consolidate (confident interval, 0 = auto).
        *
        * @param[in]  cfdi    
        * @return    0
        */
        int consolidate(const double &cfdi = 0.0);

        /**
        * @brief consolidate healthy status & biases.
        *
        * @return    0
        */
        int consolidate_others() const;

        /**
        * @brief find appropriate gnss_data_eph element (interface only).
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  chk_mask    
        * @return    tmp
        */
        std::shared_ptr<gnss_data_eph> find(const std::string &sat, const hwa_base::base_time &t, const bool &chk_mask = true);

        /**
        * @brief find appropriate gnss_data_eph elements (interface only).
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @return    vec
        */
        std::vector<std::shared_ptr<gnss_data_eph>> find_mult(const std::string &sat, const hwa_base::base_time &t) const;

        /**
        * @brief find appropriate gnss_data_eph element (iod only), add by glfeng for realtime.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  chk_mask
        * @return    tmp
        */
        std::shared_ptr<gnss_data_eph> find(const std::string &sat, const int &iod, const hwa_base::base_time &t, const bool &chk_mask = true);

        /**
        * @brief return frequency number of GLONASS.
        *
        * @return    _glo_freq_num
        */
        const std::map<std::string, int> &glo_freq_num() const { return _gloFreqNum; }

        /**
        * @brief get ionosphere correction.
        *
        * @param[in]  c        ionosphere correction
        * @return    io
        */
        gnss_data_iono_corr gegnss_data_iono_corr(const IONO_CORR &c) const;

        /**
        * @brief add ionosphere correction.
        *
        * @param[in]  c        ionosphere correction
        * @param[in]  io        ionosphere correction
        * @return    void
        */
        void add_iono_corr(const IONO_CORR &c, const gnss_data_iono_corr &io);

        /**
        * @brief add check health value.
        *
        * @param[in]  chk_health        bool, check health or not
        * @return    void
        */
        void set_chk_health(const bool &chk_health) { _chkHealth = chk_health; }

    protected:
        /**
        * @brief find appropriate gnss_data_eph element.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @param[in]  chk_mask
        * @return    null
        */
        virtual std::shared_ptr<gnss_data_eph> _find(const std::string &sat, const hwa_base::base_time &t, const bool &chk_mask = true);

        /**
        * @brief find std::vector of appropriate gnss_data_eph elements.
        *
        * @param[in]  sat        satellite
        * @param[in]  t            time
        * @return    vec_geph
        */
        std::vector<std::shared_ptr<gnss_data_eph>> _find_mult(const std::string &sat, const hwa_base::base_time &t) const;

        /**
        * @brief find appropriate gnss_data_eph element.
        *
        * @param[in]  sat        satellite
        * @param[in]  iod        issue of data 
        * @param[in]  t            time
        * @param[in]  chk_mask
        * @return    
        */
        virtual std::shared_ptr<gnss_data_eph> _find(const std::string &sat, const int &iod, const hwa_base::base_time &t, const bool &chk_mask = true);

    protected:
        bool _com = false;       ///< position/clock reference point (com = true; apc = false);
        bool _multimap = false;  ///< use std::multimap for redundant records
        bool _overwrite = false; ///< overwrite mode (for derived classes with MAP)
        bool _chkHealth = false; ///< check satellite health (navigation)
        bool _chkNavig = false;  ///< check navigation messages (internal)
        bool _chkTot = false;
        int _offset = 0;                          ///< offset for RTCM corrections
        int _nepoch = 0;                          ///< maximum number of epochs (0 = keep all)
        hwa_map_sat _mapsat;                        ///< std::map over all satellites (positions,..?)
        std::shared_ptr<gnss_data_eph> _null;                 ///< null pointer
        hwa_map_iono_CORR _brdc_iono_cor;                ///< ionosphere correction in BRDC
        std::map<std::string, int> _gloFreqNum;             ///< frequency number of GLONASS
        std::map<std::string, std::shared_ptr<gnss_data_eph>> _ephPrev; ///< save for realtime
        std::map<std::string, std::shared_ptr<gnss_data_eph>> _ephLast; ///< save for realtime
    };

} // namespace

#endif
