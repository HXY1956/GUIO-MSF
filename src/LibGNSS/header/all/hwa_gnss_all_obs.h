#ifndef hwa_gnss_all_obs_H
#define hwa_gnss_all_obs_H

#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"
#include "hwa_set_gen.h"
#include "hwa_set_gtype.h"
#include "hwa_gnss_data_obsmanager.h"
#include "hwa_gnss_data_satellite.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_gnss_Satview.h"
#include "hwa_gnss_sys.h"

#define DIFF_SEC_NOMINAL 0.905 ///< [sec] returns observations within +- DIFF_SEC for 1Hz

// normalize for 1Hz !
#define DIFF_SEC(a) (((a) > (0.0) && (a) < (1.0)) ? (DIFF_SEC_NOMINAL * a) : (DIFF_SEC_NOMINAL))

namespace hwa_gnss
{
    typedef std::shared_ptr<gnss_data_obs_manager> hwa_spt_obsmanager;

    /**
    *@brief Class for t_allobs derive from base_data
    */
    class gnss_all_obs : public base_data
    {

    public:
        /** @brief default constructor. */
        gnss_all_obs();

        /** @brief default constructor. */
        gnss_all_obs(base_log spdlog);
        /** @brief default constructor. */
        gnss_all_obs(base_log spdlog, set_base *set);
        /** @brief default destructor. */
        virtual ~gnss_all_obs();

        /** @brief XDATA type. */
        enum XDATA
        {
            XDATA_BEG,
            XDATA_END,
            XDATA_SMP,
            XDATA_SYS
        }; ///< QC

        /// @relates gnss_all_obs
        ///< first : XDATA, second :
        typedef std::map<XDATA, int> hwa_map_xdata; ///< std::map of site filtered data (QC)

        struct hwa_stc_xfilter
        {
            hwa_map_xdata xdat;
            base_time beg, end;
        }; ///< filtered data (QC)

        /// @relates gnss_all_obs
        ///< first : , second : file filtered data
        typedef std::map<std::string, hwa_stc_xfilter> hwa_map_id_xfilter; ///< std::map of file filtered data (QC)

        /// @relates gnss_all_obs
        ///< first : , second : all filtered data
        typedef std::map<std::string, hwa_map_id_xfilter> hwa_map_ii_xfilter; ///< std::map of  all filtered data (QC)

        /// @relates gnss_all_obs
        ///< first : name, second : all data-types/single epoch
        typedef std::map<std::string, hwa_spt_obsmanager> hwa_map_id_obsspt;

        /// @relates gnss_all_obs
        ///< first : time, second : all data-types/all epochs/single object
        typedef std::map<base_time, hwa_map_id_obsspt> hwa_map_ti_obsspt;

        /// @relates gnss_all_obs
        ///< first : time, second : all data-types/all epochs/all objects
        typedef std::map<std::string, hwa_map_ti_obsspt> hwa_map_iti_obsspt;

        /// @relates gnss_all_obs
        ///< first : , second : , third : , fourth:
        typedef std::map<std::string, std::map<GOBSBAND, std::map<GOBS, int>>> hwa_map_freq; ///< signals occurance

        /**
        * @brief settings of system&sampling&scalefc.
        *
        * @param[in]  *set        settings
        * @return void
        */
        virtual void gset(set_base *);

        /**
        * @brief get all stations.
        *
        * @return all stations
        */
        virtual std::set<std::string> stations();

        /**
        * @brief return std::set of available GNSS systems.
        *
        * @param[in]  site        sites
        * @return available systems
        */
        virtual std::set<GSYS> sys(const std::string &site);

        /**
        * @brief get all systems for epoch t.
        *
        * @param[in]  site    sites
        * @param[in]  t        epoch
        * @return all systems for epoch
        */
        virtual std::set<GSYS> sys(const std::string &site, const base_time &t);

        /**
        * @brief if std::map contains the site.
        *
        * @param[in]  site    sites
        * @return
            if std::map contains the site        true
            else                            false
        */
        virtual bool isSite(const std::string &site); ///<

        /**
        * @brief get all satellites for epoch t and system.
        *
        * @param[in]  site        sites
        * @param[in]  t            time
        * @param[in]  gnss        system type
        * @return
            the list of available satellites.
        */
        virtual std::set<std::string> sats(const std::string &site, const base_time &t, GSYS gnss);

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         * @return std::vector<gnss_data_sats> 
         */
        virtual std::vector<gnss_data_sats> obs(const std::string &site, const base_time &t); ///< get all gnss_data_sats for epoch t

        /**
         * @brief 
         * 
         * @param isPtr 
         * @param site 
         * @param t 
         * @return std::vector<gnss_data_obs_manager *> 
         */
        virtual std::vector<gnss_data_obs_manager *> obs(bool isPtr, const std::string &site, const base_time &t); ///<

        /**
         * @brief 
         * 
         * @param sites 
         * @param t 
         * @return std::vector<gnss_data_sats> 
         */
        virtual std::vector<gnss_data_sats> obs(const std::set<std::string> &sites, const base_time &t); ///< get all gnss_data_sats for epoch t for all sites

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         * @return std::vector<hwa_spt_obsmanager> 
         */
        virtual std::vector<hwa_spt_obsmanager> obs_pt(const std::string &site, const base_time &t); ///< get all gnss_data_obs_manager pointers for epoch t

        /**
         * @brief 
         * 
         * @param site 
         * @param prn 
         * @param beg 
         * @param end 
         * @return std::vector<hwa_spt_obsmanager> 
         */
        virtual std::vector<hwa_spt_obsmanager> obs_prn_pt(const std::string &site, const std::string &prn,   /// <
                                              const base_time &beg, const base_time &end); ///< get all gnss_data_obs_manager pointers for prn in interval

        /**
         * @brief 
         * 
         * @param site 
         * @return std::vector<base_time> 
         */
        virtual std::vector<base_time> epochs(const std::string &site); ///< get all t_gepochs for site

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         * @return base_time 
         */
        virtual base_time load(const std::string &site, const double &t); ///< added by zhShen

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         */
        virtual void erase(const std::string &site, const base_time &t); ///< added by zhShen

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         * @param sat 
         */
        virtual void erase(const std::string &site, const base_time &t, const std::string &sat); ///< added by jqwu

        /**
         * @brief 
         * 
         */
        virtual void clear_obj();

        /**
         * @brief 
         * 
         * @param site 
         * @param beg 
         * @param end 
         */
        virtual void clean_outer(const std::string &site = "",
                                 const base_time &beg = FIRST_TIME,
                                 const base_time &end = LAST_TIME);

        /**
         * @brief 
         * 
         * @return base_time 
         */
        virtual base_time begT(); // get first gnss_data_obs epoch in all sites;zhshen

        /**
         * @brief 
         * 
         * @param site 
         * @param smpl 
         * @return base_time 
         */
        virtual base_time beg_obs(const std::string &site, double smpl = 0.0); // get first gnss_data_obs epoch for site

        /**
         * @brief 
         * 
         * @param site 
         * @return base_time 
         */
        virtual base_time end_obs(const std::string &site); // get last  gnss_data_obs epoch for site

        /**
         * @brief 
         * 
         * @param site 
         * @param file 
         * @param xflt 
         */
        void xdata(const std::string &site, const std::string &file, const hwa_stc_xfilter &xflt); // add site-specific filtered data/epochs

        /**
         * @brief 
         * 
         * @param site 
         * @param file 
         * @return hwa_stc_xfilter 
         */
        hwa_stc_xfilter xdata(const std::string &site, const std::string &file); // get site-specific filtered data/epochs

        /**
         * @brief 
         * 
         * @param obs 
         * @return int 
         */
        int addobs(hwa_spt_obsmanager obs); // add single station observation (P and L in meters !)

        /**
         * @brief 
         * 
         * @param b 
         */
        void overwrite(const bool &b) { _overwrite = b; } // std::set/get overwrite mode

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &overwrite() const { return _overwrite; }

        /**
         * @brief 
         * 
         * @param site 
         */
        void setepoches(const std::string &site); // acquire all epoches

        /**
         * @brief 
         * 
         * @param i 
         */
        void maxepoch(const unsigned int &i) { _nepoch = i; } // std::set/get maximum number of epochs stored

        /**
         * @brief 
         * 
         * @return unsigned int 
         */
        const unsigned int &maxepoch() const { return _nepoch; }

        /**
         * @brief 
         * 
         * @param site 
         * @return unsigned int 
         */
        unsigned int nepochs(const std::string &site);

        /**
         * @brief 
         * 
         * @param site 
         * @param beg 
         * @param end 
         * @param sampl 
         * @param n 
         * @return * unsigned int 
         */
        unsigned int nepochs(const std::string &site, const base_time &beg, const base_time &end, double sampl, std::map<GSYS, std::pair<int, int>> &n); // get number of epochs for station expect/have according to sampl

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         * @return hwa_map_id_obsspt 
         */
        virtual hwa_map_id_obsspt find(const std::string &site, const base_time &t); // find appropriate gnss_data_obs_manager element for site/epoch

        /**
         * @brief 
         * 
         * @param site 
         * @param t 
         * @param prn 
         * @param gobs 
         * @return double 
         */
        virtual double find(const std::string &site, const base_time &t, const std::string &prn, const GOBS &gobs);

        /**
         * @brief 
         * 
         * @param site 
         * @return hwa_map_freq 
         */
        hwa_map_freq frqobs(const std::string &site); // get number of occurance of individual signals
                                              // add by glfeng

        /**
         * @brief 
         * 
         * @param site 
         * @param crd 
         */
        void addsitecrd(const std::string &site, const Triple &crd) { _mapcrds[site] = crd; }

        /**
         * @brief 
         * 
         * @param site 
         * @return Triple 
         */
        const Triple &getsitecrd(const std::string &site) const { return _mapcrds.at(site); }

        /**
         * @brief 
         * 
         * @return std::map<std::string, int> 
         */
        const std::map<std::string, int> &glo_freq_num() const { return _glofrq; }

        /**
         * @brief 
         * 
         * @param sat 
         * @param freqNum 
         */
        void add_glo_freq(const std::string &sat, int freqNum)
        {
            if (_glofrq.find(sat) == _glofrq.end())
                _glofrq[sat] = freqNum;
        }

        /**
         * @brief Get the Site List object
         * 
         * @return const std::set<std::string>& 
         */
        const std::set<std::string> &getSiteList() { return _map_sites; }

        /**
         * @brief 
         * 
         * @param site 
         * @param sats 
         * @param bands 
         */
        void exclude_sat_band(const std::string &site, const std::map<GSYS, std::set<std::string>> &sats, const std::map<GSYS, GOBSBAND> &bands);

    protected:
        virtual std::set<std::string> _sats(const std::string &site, const base_time &t, GSYS gnss);
        virtual std::set<GSYS> _gsys(const std::string &site, const base_time &t);
        virtual std::vector<gnss_data_sats> _gobs(const std::string &site, const base_time &t);
        virtual std::vector<gnss_data_obs_manager *> _gobs(bool isPtr, const std::string &site, const base_time &t);

        int _find_epo(const std::string &site, const base_time &epo, base_time &tt); // find epoch from the std::map w.r.t. DIFF_SEC

    protected:
        set_base *_set = nullptr;
        unsigned int _nepoch;            ///< maximum number of epochs (0 = keep all)
        hwa_map_iti_obsspt _mapobj;              ///< std::map over all objects (receivers)
        hwa_map_ii_xfilter _filter;              ///< structure of stations/files filtered data (QC)
        bool _overwrite;                 ///< rewrite/add only mode
        std::set<std::string> _sys;                ///< systems settings
        double _smp;                     ///< sampling settings
        double _scl;                     ///< sampling scale-factor
        std::vector<base_time> _allepoches;     ///< all epoches (added by zhShen)
        std::map<std::string, Triple> _mapcrds; ///< all sites apr coordinates (add by glfeng)
        std::map<std::string, int> _glofrq;        ///< std::map of GLONASS slot/frequency (add by glfeng)
        std::set<std::string> _map_sites;          ///< std::map of sites (glfeng)
    private:
    };

} // namespace

#endif
