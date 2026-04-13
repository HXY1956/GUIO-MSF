#ifndef hwa_gnss_amb_common_H
#define hwa_gnss_amb_common_H

#include <string>
#include <map>
#include <memory>
#include <tuple>
#include <list>
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_amb_oneway
    {
    public:
        /** @brief default constructor. */
        explicit gnss_amb_oneway();

        /** @brief default destructor. */
        virtual ~gnss_amb_oneway(){};

        std::string ambtype;     ///< C/1/2/W
        std::string sat;         ///< satellite name
        int ipt = 0;        ///< index of par
        int beg_epo = 0;    ///< start time
        int end_epo = 0;    ///< t_end time
        double rwl = 0.0;   ///< real value widelane(cyc) jdhuang : remove warnning
        double srwl = 0.0;  ///< its sigma
        double rewl = 0.0;  ///< real value extrawidelane(cyc)
        double srewl = 0.0; ///< its sigma
        double rlc = 0.0;   ///< real value lc from slution(m)
        double srlc = 0.0;  ///< its sigma
        double ele = 0.0;   ///< elevation
        double r = 0.0;     ///< ambiguity in UCUD mode
        double sr = 0.0;    ///< sigma of ambiguity
    };

    /**
    * @brief  single-differece ambiguity  for arc.
    */
    class gnss_amb_dd_base
    {
    public:
        /** @brief default constructor. */
        gnss_amb_dd_base(){};

        /** @brief default destructor. */
        virtual ~gnss_amb_dd_base(){};

        std::string ambtype; ///< C/1/2/W

        bool isEwlFixed = false;   ///< extral widelane std::fixed
        bool isEwl24Fixed = false; ///< extral frequency24 std::fixed
        bool isEwl25Fixed = false; ///< extral frequency25 std::fixed
        bool isWlFixed = false;    ///< widelane std::fixed
        bool isNlFixed = false;    ///< narrowlane std::fixed
        std::string site = "default";
        //std::map<std::string, int> ipt2ow;
        std::vector<std::tuple<std::string, int, int>> ddSats; ///< sat_name, index in all_pars, index in amb_pars
        hwa_base::base_time beg_epo;                        ///< begin time
        hwa_base::base_time end_epo;                        ///< end time
        hwa_base::base_time end_epo_save;                   ///< save end epoch time
        double rwl_R1 = 0.0;                    ///< real widelane for glonass
        double rwl_R2 = 0.0;                    ///< real widelane for glonass
        double srwl_R1 = 0.0;                   ///< real widelane sigma for glonass
        double srwl_R2 = 0.0;                   ///< real widelane sigma for glonass
        double rewl = 0.0;                      ///< real Extra widelane and its sigma
        double srewl = 0.0;                     ///< real Extra widelane and its sigma
        double rewl24 = 0.0;                    ///< real Extra widelane and its sigma
        double srewl24 = 0.0;                   ///< real Extra widelane and its sigma
        double rewl25 = 0.0;                    ///< real Extra widelane and its sigma
        double srewl25 = 0.0;                   ///< real Extra widelane and its sigma
        double rwl = 0.0;                       ///< real widelane and its sigma
        double srwl = 0.0;                      ///< real widelane and its sigma
        double rwl_q1 = 0.0;                    ///< real widelane and its q1
        double rwl_q2 = 0.0;                    ///< real widelane and its q2
        double rnl = 0.0;                       ///< real narrowlane and its sigma
        double srnl = 0.0;                      ///< real narrowlane and its sigma
        double rlc = 0.0;                       ///< lc ambiguity and its sigma
        double srlc = 0.0;                      ///< lc ambiguity and its sigma
        int iwl = 0;                            ///< integer wide - and wide lane
        int inl = 0;                            ///< integer wide - and narrow lane
        int iewl = 0;                           ///< integer wide - and extral wide lane
        int iewl24 = 0;                         ///< integer wide - and extral wide lane24
        int iewl25 = 0;                         ///< integer wide - and extral wide lane25
        double factor = 0.0;                    ///< TODO
        double sd_rnl_cor = 0.0;                ///< correction of SD, narrowlane
        double sd_rwl_cor = 0.0;                ///< correction of SD, widelane
        double sd_rewl_cor = 0.0;               ///< correction of SD, extra-widelane
        double sd_rewl24_cor = 0.0;             ///< correction of SD, extra-widelane
        double sd_rewl25_cor = 0.0;             ///< correction of SD, extra-widelane
        double sd_r1_cor = 0.0;                 ///< correction of N1
        double sd_r2_cor = 0.0;                 ///< correction of N2
        double sd_r3_cor = 0.0;                 ///< correction of N3
        double sd_r4_cor = 0.0;                 ///< correction of N4
        double sd_r5_cor = 0.0;                 ///< correction of N5
        double sigcor = 0.0;                    ///<sigma
        int fix_epoch = 1;                      /// std::fixed epochs of dd ambiguity
        bool isSngleFreq = false;               /// exist single frequency satellites or not
    };

    /**
    * @brief ambiguity info for one site one satellite one epoch  used in nl upd.
    */
    class gnss_amb_epoch
    {
    public:
        /** @brief default constructor. */
        gnss_amb_epoch();

        /** @brief default destructor. */
        virtual ~gnss_amb_epoch();

        base_time epo;           ///< current epoch
        bool isnewarc = false; ///< new arc or not
        int nepo = 0;          ///< number of epoch
        double bc = 0.0;       ///< ambiguity value
        double sbc = 0.0;      ///< IF ambiguity sigma
        double bw = 0.0;       ///< Widelane ambiguitiy value
        double sbw = 0.0;      ///< Widelane ambiguitiy sigma value
        double bw0 = 0.0;      ///< Widelane ambiguitiy init value
        double bn = 0.0;       ///< narrowlane ambiguity
        double elev = 0.0;     ///< elevation
        double bwi = 0.0;      ///< TODO
        double bewi = 0.0;     ///< TODO
    };

    /** @brief ambiguity residual. */
    class gnss_amb_epoch_res
    {
    public:
        /** @brief default constructor. */
        gnss_amb_epoch_res();

        /** @brief default destructor. */
        virtual ~gnss_amb_epoch_res();
        bool wl_fixed = false;  ///< wide lane std::fixed
        bool ewl_fixed = false; ///< extral wide lane std::fixed
        bool nl_fixed = false;  ///< narrow lane std::fixed
        double nl_res = 0.0;    ///< narrow lane residuals
        double wl_res = 0.0;    ///< wide lane reiduals
    };

    // UPD
    /** @brief std::map for storaging ambiguity info ,hwa_map_amb_epoch[site][res] = gnss_amb_epoch. */
    typedef std::map<std::string, std::map<std::string, gnss_amb_epoch>> hwa_map_amb_epoch;

    /** @brief std::map for storaging ambiguity residual ,hwa_map_amb_epoch_res[site][res] = gnss_amb_epoch_res. */
    typedef std::map<std::string, std::map<std::string, gnss_amb_epoch_res>> hwa_map_amb_epoch_res;

    // PPP AR/ WL+EWL UPD
    /** @brief std::map for storaging oneway_ambiguity  */
    typedef std::vector<std::shared_ptr<gnss_amb_oneway>> hwa_vector_amb_oneway; //satellites OW;

    /** @brief std::map for storaging oneway_ambiguity , one station/all satellite/all epoch. */
    typedef std::map<std::string, hwa_vector_amb_oneway> hwa_map_amb_oneway; //satellites OW;

    //** @brief std::vector for storaging double-differece ambiguity. */
    typedef std::vector<gnss_amb_dd_base> hwa_vector_amb_dd;

    // WL/EWL UPD
    //** @brief std::map for storaging ambiguity value, one station/all epo/all satellite. */
    typedef std::map<int, std::map<std::string, double>> hwa_map_amb_value;

    // MW in PPP
    //** @brief std::map for storaging ambiguity valid time. */
    typedef std::map<std::string, std::vector<std::pair<base_time, base_time>>> hwa_map_amb_time;

    //** @brief std::map for storaging MW obs value, one station/all epo/all satellite/idx=(1,2,3,4,5). */
    typedef std::map<base_time, std::map<std::string, std::map<int, double>>> hwa_map_mw;

    /**
    * @brief Compute mean of the base_type_conv::fractional ambiguities.
    * @param[in]  values    list std::pairs contain data and it's weight.
    * @param[in]  mean        data mean.
    * @param[in]  sigma        data sigma.
    * @param[in]  sigx        sigma's sigma.
    * @param[out] mean
    * @param[out] sigma
    * @param[out] sigx
    * @return      void
    */
    void getMeanFract(std::list<std::pair<double, double>>& values, double& mean, double& sigma, double& sigx);

    /**
    * @brief Get mean, sigma and sigma of the mean of a std::set sampled data.
    * @param[in]  is_edit   whether to eliminate errors.
    * @param[in]  wx         list std::pairs contain data and it's weight.
    * @param[in]  mean        data mean.
    * @param[in]  sigma        data sigma.
    * @param[in]  sigx        sigma's sigma.
    * @param[out] mean
    * @param[out] sigma
    * @param[out] sigx
    * @return      void
    */
    void getMeanWgt(bool is_edit, std::list<std::pair<double, double>> &wx, double &mean, double &sigma, double &mean_sig);

    /**
    * @brief Adjust ambiguities between -0.5~0.5 cycle.
    * @param[in]  x     real value
    * @param[in]  min   [0,1), [-0,5,0.5), [-1.0,1.0)
    * @return      base_type_conv::fraction of ambiguities
    */
    double getFraction(double x, double min);

}

#endif
