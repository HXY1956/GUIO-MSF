#ifndef hwa_gnss_all_amb_h
#define hwa_gnss_all_amb_h
#include "hwa_base_allpar.h"
#include "hwa_base_mutex.h"
#include "hwa_base_par.h"
#include "hwa_base_log.h"
#include "hwa_gnss_amb_ow.h"
#include "hwa_gnss_amb_bdeci.h"
#include "hwa_gnss_all_Obs.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_all_Bias.h"
#include "hwa_gnss_data_Upd.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /**
     * @brief Class for all amb
     */
    class gnss_all_amb
    {

    public:
        /// @relates gnss_all_amb
        ///< std::string : sat name
        typedef std::map<std::string, std::vector<std::shared_ptr<gnss_amb_ow>>> t_ow_obj;
        /// @relates gnss_all_amb
        ///< std::string : sat name
        typedef std::map<std::string, std::vector<std::shared_ptr<gnss_amb_SD>>> t_sd_obj;
        using t_amb_sum = std::map<AMB_ID, std::map<GSYS, int>>;
        ///< amd combine option
        // not used now, jqwu 2021
        enum class AMB_comb_OPTION
        {
            CMB_REC_SD, ///< Station single difference
            CMB_SAT_SD, ///< Satellite Single Difference
            CMB_REC_DD, ///< Double difference of stations on the basis of satellite single difference
            CMB_SAT_DD, ///< Satellite double difference on the basis of single difference of the measurement station
            UNDEF
        };
        /**
         * @brief Construct a new t gallambs object
         */
        explicit gnss_all_amb();
        /**
         * @brief Construct a new t gallambs object
         * @param[in]  gset      std::setbase control
         * @param[in]  spdlog       logbase control
         */
        explicit gnss_all_amb(set_base *gset, base_log spdlog);
        /**
         * @brief Destroy the t gallambs object
         */
        virtual ~gnss_all_amb();
        /**
         * @brief std::set std::setbase control
         * @param[in]  gset      std::setbase control
         */
        void add_set(set_base *gset);
        /**
         * @brief std::set logbase control
         * @param[in]  spdlog       logbase control
         */
        void add_log(base_log spdlog);
        /**
         * @brief add data
         * @param[in]  pars      parameters
         * @param[in]  obs       observation
         * @param[in]  allbias   all bias data
         * @param[in]  obj       all object data
         * @param[in]  upd       upd data
         */
        void add_data(base_allpar *pars, gnss_all_obs *obs, gnss_all_bias *allbias, gnss_all_obj *obj = nullptr, gnss_data_upd *upd = nullptr);
        /**
         * @brief add, Extral Wide Lane, Wide Lane, Narrow Lane
         * @param[in]  EWL       Extral Wide Lane
         * @param[in]  WL        Wide Lane
         * @param[in]  NL        Narrow Lane
         */
        void add_bdeci(gnss_amb_bdeci *EWL, gnss_amb_bdeci *WL, gnss_amb_bdeci *NL);
        /**
         * @brief get station name list
         * @return std::set<std::string> station name lists
         */
        const std::set<std::string> &rec_list() const { return _rec_list; }
        /**
         * @brief get satellite name list
         * @return std::set<std::string> satellite name lists
         */
        const std::set<std::string> &sat_list() const { return _sat_list; }
        /**
         * @brief get system list
         * @return std::set<std::string> system lists
         */
        const std::set<std::string> &gsys() const { return _gsys; }
        /**
         * @brief get station coordinate
         * @return std::map<std::string, Triple> station name & coordinate
         */
        const std::map<std::string, Triple> &rec_crd() const { return _rec_crd; }
        /**
         * @brief Get the certain station coordinate
         * @param[in]  rec       station name
         * @return Triple coordinate
         */
        Triple get_crd(const std::string &rec) const { return (_rec_crd.find(rec) != _rec_crd.end() ? _rec_crd.at(rec) : Triple()); }
        /**
         * @brief get certain station satellite list
         * @param[in]  rec       station name
         * @return std::set<std::string> satellite name list
         */
        const std::set<std::string> &sats_of_rec(const std::string &rec) const { return _sats_of_rec.at(rec); }
        /**
         * @brief get all ow ambs
         * @return std::map<std::string, t_ow_obj> all ow ambs
         */
        const std::map<std::string, t_ow_obj> &ow_ambs() const { return _all_ow_ambs; }
        /**
         * @brief calculate station single difference
         */
        bool cmb_sd_by_sat();
        /**
         * @brief calculate double difference of one satellite and one station
         * @param[in]  bl        baseline
         */
        bool cmb_one_dd(const std::pair<std::string, std::string> &bl);
        /**
         * @brief calculate double difference of one satellite and one station
         * @param[in]  bl        baseline
         * @param[in]  vdd       double difference data
         */
        void _cmb_one_dd(const std::pair<std::string, std::string> &bl, std::vector<std::shared_ptr<gnss_amb_DD>> &vdd) const;
        /**
         * @brief calculate double difference
         * @param[in]  baselines baselines
         */
        void process_dd_ambs(const std::vector<std::pair<std::string, std::string>> &baselines);
        /**
         * @brief calculate single difference
         */
        void process_sd_ambs();
        /**
         * @brief fix single difference amb
         */
        void fix_ow_by_sd();
        /**
         * @brief get the double difference ambs
         * @return std::vector<std::shared_ptr<gnss_amb_DD>> double difference ambs
         */
        const std::vector<std::shared_ptr<gnss_amb_DD>> &get_amb_dds() const { return _vdd; }
        /**
         * @brief get the double single difference ambs
         * @return std::vector<std::shared_ptr<gnss_amb_SD>> single difference ambs
         */
        const std::vector<std::shared_ptr<gnss_amb_SD>> &get_sd_ambs() const { return _vsd; }
        /**
         * @brief clear certain type
         * @param[in]  tp        AMB type
         */
        void clear(AMB_TYPE tp)
        {
            if (tp == AMB_TYPE::UD)
                _all_ow_ambs.clear();
            else if (tp == AMB_TYPE::SD)
                _all_sd_ambs.clear();
            else if (tp == AMB_TYPE::DD)
                _all_dd_ambs.clear();
        }

        std::map<AMB_ID, std::map<GSYS, int>> num_fixed; ///< system & number of std::fixed
        std::map<AMB_ID, std::map<GSYS, int>> num_all;   ///< system & number of all

    protected:
        /**
         * @brief get ow amb from parameter
         */
        void _get_amb_oneway_from_pars();
        /**
         * @brief get wide line from observation
         */
        void _get_wl_from_gobs();
        /**
         * @brief get wide line from parameter
         */
        void _get_wl_from_pars();
        /**
         * @brief get value of amb cycle
         * @param[in]  sat       satellite name
         * @param[in]  amb_id    amb id
         * @param[in]  val       value of cycle(m)
         * @return double value of cycle
         */
        double _amb_in_cycle(const std::string &sat, const AMB_ID &amb_id, const double &val);
        /**
         * @brief ambiguity std::fixed
         * @param[in]  amb       amb data
         */
        void _fix_ambiguity(const std::shared_ptr<gnss_amb> &amb);
        /**
         * @brief single difference std::fixed
         * @param[in]  sd        single difference
         */
        void _fix_one_ow(const std::shared_ptr<gnss_amb_SD> &sd);
        /**
         * @brief check amb depend
         * @param[in]  isFirst   wheather first
         * @param[in]  iNamb     index of amb
         * @param[in]  iNdef     TODO
         * @param[in]  iN_oneway TODO
         * @param[in]  arriIpt2ow TODO
         * @param[in]  iMaxamb_ow TODO
         * @param[in]  iMaxamb_for_check TODO
         * @return true
         * @return false
         */
        bool _check_amb_depend(bool isFirst, int iNamb, int *iNdef, int iN_oneway, std::vector<int> arriIpt2ow, int iMaxamb_ow, int iMaxamb_for_check);
        /**
         * @brief check amb depend
         */
        bool _check_amb_depend(double pdE[], double pdC[], int iNamb, int &iNdef, int iN_oneway, std::vector<int> arriIpt2ow, int iNdim_ow, int iNdim_for_check) const;
        /**
         * @brief check amb depend SVD
         */
        bool _check_amb_depend_SVD(std::vector<int> idx, int num_ow, Matrix &vector_space);
        /**
         * @brief get single difference upd
         */
        double _get_sd_upd(const UPDTYPE &tp, const std::string &sat1, const std::string &sat2, const base_time &t);
        /**
         * @brief get ow amb position
         */
        int _get_oneway_pos(const std::string &site, const std::string &sat, const base_time &beg, const base_time &end);
        /**
         * @brief get one station observation
         */
        void _get_obs_onesite(const std::string &site);
        /**
         * @brief get wide line from observation
         */
        std::pair<double, double> _get_ow_wl(const std::vector<hwa_spt_obsmanager> &vobs);
        /**
         * @brief get extral wide line from observation
         */
        std::pair<double, double> _get_ow_ewl(const std::vector<hwa_spt_obsmanager> &vobs, const AMB_ID &mode = AMB_ID::EWL);
        /**
         * @brief calcuate summary of amb
         */
        void _summary_amb(const gnss_amb &amb);
        /**
         * @brief calcuate summary of amb
         */
        void _summary_amb(const gnss_amb &amb, t_amb_sum &nfixed, t_amb_sum &nall) const;
        /**
         * @brief update summary of amb
         */
        void _update_amb_sum(const t_amb_sum &nfixed, const t_amb_sum &nall);

    private:
        base_log _spdlog;             ///< logbase control
        set_base *_gset = nullptr;  ///< std::setbase control
        gnss_all_obs *_gobs = nullptr;   ///< all observation data
        gnss_all_bias *_gbias = nullptr; ///< all bias data
        base_allpar *_gpars = nullptr;  ///< all parameters
        gnss_all_obj *_gobj = nullptr;   ///< all object
        gnss_data_upd *_gupd = nullptr;      ///< upd data
        double *_pdE = nullptr;       ///< TODO
        double *_pdC = nullptr;       ///< TODO

        gnss_amb_bdeci *_evaluator_EWL = nullptr; ///< extral wide line
        gnss_amb_bdeci *_evaluator_WL = nullptr;  ///< wide line
        gnss_amb_bdeci *_evaluator_NL = nullptr;  ///< narrow line

        DD_MODEL _dd_mode;                              ///< double difference mode
        UPD_MODE _upd_mode;                             ///< upd mode
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< band index
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; ///< frequency index
        int _frequency = 2;                             ///< number of frequency
        OBSCOMBIN _obs_comb = OBSCOMBIN::DEF_OBSCOMBIN; ///< TODO
        std::set<AMB_ID> _required_ids;                      ///< TODO

        std::set<std::string> _gsys;               ///< system
        std::set<std::string> _rec_list;           ///< station name list
        std::set<std::string> _sat_list;           ///< satellite name list
        std::map<std::string, Triple> _rec_crd; ///< station coordinate
        std::string _class_id = "gnss_all_amb"; ///< name of class
        base_mutex _gmutex;                ///< TODO

        double _min_common_time;                                               ///< minumum of common time
        int _num_ows = 0;                                                      ///< number of ow
        std::unordered_map<std::string, std::set<std::string>> _sats_of_rec;                       ///< satellite name & station name
        std::map<std::string, t_ow_obj> _all_ow_ambs;                                    ///< _all_ow_ambs[site][sat][0,1,2...]
        t_sd_obj _all_sd_ambs;                                                 ///< _all_sd_ambs[site][0,1,2...]
        std::map<std::pair<std::string, std::string>, std::vector<std::shared_ptr<gnss_amb_DD>>> _all_dd_ambs; ///< _all_dd_ambs[{site, site}][0,1,2...]
        std::vector<std::shared_ptr<gnss_amb_DD>> _vdd;                                    ///< double difference
        std::vector<std::shared_ptr<gnss_amb_SD>> _vsd;                                    ///< single difference
        std::vector<std::shared_ptr<gnss_amb_ow>> _vow;                                    ///< TODO
    };
}

#endif