#ifndef hwa_gnss_proc_preprocESSING_H
#define hwa_gnss_proc_preprocESSING_H

#include <sstream>

#include "hwa_base_eigendef.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_set_gen.h"
#include "hwa_set_gbase.h"
#include "hwa_set_gproc.h"

using namespace hwa_base;

namespace hwa_gnss
{

    class gnss_data_obs_pair;

    /** @brief class for gnss_proc_preproc derive from gnss_data_monit. */
    class gnss_proc_preproc
    {
    public:
        /** @brief constructor 1. */
        gnss_proc_preproc(gnss_all_obs *obs, set_base *settings);

        /** @brief default destructor. */
        ~gnss_proc_preproc();

        /** @brief set/get glog pointer. */
        void spdlog(base_log spdlog);
        base_log spdlog() const { return _spdlog; }

        /** @brief for Cycle slip detection. */
        //rename by zzwu
        int ProcessBatch(std::string site, const base_time &beg, const base_time &end, double sampl, bool sync, bool save = false);

        /** @brief for set the priviate values. */
        void setNav(gnss_all_nav *nav);

        /** @brief for set site. */
        void setSite(std::string site);

        /** @brief for Cycle slip. */
        std::map<base_time, std::map<std::string, std::map<GOBS, double>>> getSlips();
        std::map<base_time, std::map<std::string, std::map<GOBS, int>>> getSlipsGap();
        std::map<base_time, int> getClkjump();

        typedef std::map<int, std::map<gnss_data_obs_pair, double>> hwa_map_slp;
        typedef std::map<int, std::vector<gnss_data_obs_pair>> t_vec_slp;

        void setSingleEpoData(std::vector<gnss_data_sats> *input_data);

    protected:
        gnss_all_obs *_obs;      ///< observation
        gnss_all_nav *_nav;      ///< navigation
        std::string _site;         ///< site
        base_log _spdlog;     ///< spdlog
        set_base *_set;     ///< setting
        double _sigCode;      ///< sigma of code observation
        double _sigPhase;     ///< sigma of phase observation
        double _sigCode_GLO;  ///< sigma of GLONASS code observation
        double _sigPhase_GLO; ///< sigma of GLONASS phase observstion
        double _sigCode_GAL;  ///< sigma of GAL code observation
        double _sigPhase_GAL; ///< sigma of GAL phase observation
        double _sigCode_BDS;  ///< sigma of BDS code observation
        double _sigPhase_BDS; ///< sigma of BDS phase observation
        double _sigCode_QZS;  ///< sigma of QZS code observation
        double _sigPhase_QZS; ///< sigma of QZS phase observation
        double _sigCode_IRN;  ///< sigma of IRN code observation
        double _sigPhase_IRN; ///< sigma of IRN phase observation
        double _sumS;         ///< sumS
        double _scl;          ///< scl
        Triple _pos;       ///< position
        std::set<std::string> _sys;     ///< system
        std::set<std::string> _sat;     ///< satellite

        bool _beg_end;

        std::map<std::string, std::map<std::string, double>> _dI; ///< ionospheric delay (site sys value)
        //std::map<std::string, int>     _msoffset;
        std::map<std::string, std::map<base_time, int>> _mbreaks; ///< for logging (site time CJ)

        // --- global settings
        std::map<std::string, bool> _firstEpo;                 ///< fisrt epoch
        std::map<std::string, std::vector<gnss_data_obs_manager>> _epoDataPre; ///< epoch data prepare

        hwa_map_slp _m_lcslp; ///< m_lcslp
        t_vec_slp _v_lcslp; ///< v_lcslp

        std::map<std::string, std::map<base_time, std::map<std::string, std::map<GOBS, double>>>> _mslips; ///< for logging - slip due to true CS  (site ....)
        std::map<std::string, std::map<base_time, std::map<std::string, std::map<GOBS, int>>>> _mslipsGap; ///  for logging - slip due to data gap => 1 (epoch gap), 2(sat gap), 3(GOBS gap) (site ...)

        // for npp
        std::vector<std::shared_ptr<gnss_data_obs_manager>> _inputEpoData;   ///< input epoch data
        std::vector<gnss_data_sats> *_satdata_npp = nullptr;     ///< satellite data npp
        std::map<std::string, std::map<std::string, int>> _map_nppdata_idx; ///< site sat idx

        /** @brief check phase cycl slips. */
        int _slip(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2);

        /** @brief Compute threshold for cycle slip. */
        double _slipThreshold(std::string LC, hwa_spt_obsmanager obs, gnss_data_band &band1, gnss_data_band &band2, double sampl = 30.0);

        /** @brief check coherency between range and phase caused by clock jump. */
        int _jumps(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2);

        /** @brief Repair phase due to clk jump. */
        void _repair(std::vector<hwa_spt_obsmanager> epoData, double dL);

        /** @brief Transform dN to slips on individual band. */
        int _transform(hwa_spt_obsmanager gobs, bool save);

        /** @brief Save estimated cycle slips. */
        void _save(hwa_spt_obsmanager gobs, const std::map<GOBS, double> &slips);

        /** @brief remove slips from gobsgnss* and _mslips. */
        void _remove_slip(std::vector<hwa_spt_obsmanager> gobs);

        /** @brief only common items remain. */
        void _common(std::set<GOBSBAND> &set1, std::set<GOBSBAND> &set2);
        void _common(std::set<GOBS> &set1, std::set<GOBS> &set2);

        /** @brief ionosphere scale factor: wl - nl. */
        double _disf(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2, gnss_data_obs &s1, gnss_data_obs &s2);

        /** @brief ionosphere. */
        void _iono(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2, gnss_data_obs &s1, gnss_data_obs &s2);

        /** @brief find wl slip. */
        double _findSlp(int &i, gnss_data_obs_pair &gpair);

        /** @brief report epo data as slips due to epoch data gap. */
        void _gapReport(std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData);
        void _gapReport(std::shared_ptr<gnss_data_obs_manager> data);

        /** @brief compare two epoch data. */
        void _compare(std::vector<gnss_data_obs_manager> data1, std::vector<std::shared_ptr<gnss_data_obs_manager>> data2);

        /** @brief update Npp data. */
        void _updateNppData(std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData, bool clock_jump);
    };

    /** @brief class for gnss_data_obs_pair. */
    class gnss_data_obs_pair
    {
    public:
        /** @brief obs std::pair. */
        gnss_data_obs_pair(gnss_data_obs &gobs1, gnss_data_obs &gobs2);
        gnss_data_obs obs1; ///< obs1
        gnss_data_obs obs2; ///< obs2
        double val;  ///< val

        /** @brief override operator <. */
        bool operator<(const gnss_data_obs_pair &t) const;
    };

} // namespace

#endif
