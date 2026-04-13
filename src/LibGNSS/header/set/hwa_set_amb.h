/**
*
* @verbatim
    The format of this block:
    <!--> ambiguity fix setting node <-->
    <ambiguity>
        <is_ppprtk>NO</is_ppprtk>                    <!--> PPP-RTK or not, default: NO, option: YES/NO <!-->
        <fix_mode>SEARCH</fix_mode>                    <!--> ambiguity fixing mode, option: ROUND ��round to integer, NO ��not fix, SEARCH ��lambda search <!-->
        <part_fix>YES</part_fix>                    <!--> part ambiguity fix option: YES/NO <!-->
        <ratio>2.0</ratio>                            <!--> lambda search ratio <!-->
        <add_leo>NO</add_leo>                        <!--> add LEO data or not, default: NO, option: YES/NO <!-->
        <all_baselines>NO</all_baselines>            <!--> use all baselines or not, default: NO, option: YES/NO <-->
        <min_common_time>900</min_common_time>        <!--> minimum common time, unit: second <!-->
        <baseline_length_limit>3500</baseline_length_limit> <!--> maximum baseline length, unit: km <!-->
        <widelane_interval>30</widelane_interval>    <!--> widelane interval, unit: second <!-->
        <dd_mode>IF_CB_WN</dd_mode>                    <!--> double difference mode <!-->
        <widelane_decision alpha="1000" maxdev="0.15" maxsig="0.10" />      <!--> widelane setting, option: alpha&maxdev(Confidence interval parameter), maxsig(maximum sigma)<!-->
        <narrowlane_decision alpha="1000" maxdev="0.15" maxsig="0.10" /> <!--> narrowlane setting, option: alpha&maxdev(Confidence interval parameter), maxsig(maximum sigma) <!-->
    </ambiguity>
  @endverbatim

* @verbatim
    History
        -1.0 glFeng   2019-02-27 creat the file.
        -1.1 BoWong   2019-04-10 Adding Doxygen Style Code Remarks
        -1.2 glFeng   2019-04-11 Add function part_ambfix
  @endverbatim
* Copyright (c) 2018, Wuhan University. All rights reserved.
*
* @file           gsetamb.h
* @brief       control std::set from XML
*
* @author      glFeng, Wuhan University
* @version       1.2.0
* @date           2019-04-10
*
*/

#ifndef hwa_set_ambiguity_h
#define hwa_set_ambiguity_h
#define XMLKEY_AMBIGUITY "ambiguity"

#include "hwa_set_base.h"

namespace hwa_gnss {
    /** @brief class enum of UPD type. */
    enum class UPDTYPE
    {
        WL,         ///< wide lane.
        EWL,        ///< extra wide lane.
        EWL24,      ///< extra wide lane.(2/4 frequency)
        EWL25,      ///< extra wide lane.(2/5 frequency)
        EWL_EPOCH,  ///< TODO
        NL,         ///< narrow lane.
        IFCB,       ///< inter frequency clock bias.
        RTPPP_SAVE, ///< TODO
        NONE        ///< none.
    };

    /** @brief class enum of ambiguity type. */
    enum class AMB_TYPE
    {
        UD,   ///< undifferenced.
        SD,   ///< Single-difference, usually between satellites.
        DD,   ///< double-difference.
        UNDEF ///< undefined.
    };

    /** @brief class enum of double-difference ambiguity type. */
    enum class DD_MODEL
    {
        IF_CB_WN = 0,  ///< TODO
        RAW_CB_WN = 1, ///< TODO
        RAW_CB = 2,    ///< TODO
        RAW_CB_2 = 3,  ///< TODO
        NONE = 4       ///< none.
    };

    /** @brief enum of FIX mode. */
    enum class FIX_MODE
    {
        NO,     ///< float.
        ROUND,  ///< round.
        SEARCH, ///< ambiguity search.
        HOLD    ///< TODO
    };

    /** @brief enum of UPD mode. */
    enum class UPD_MODE
    {
        UPD, ///< wl upd + nl upd.
        IRC, ///< wl upd.
        OSB  ///< no upd.
    };
};

using namespace hwa_gnss;

namespace hwa_set
{
    std::string updmode2str(UPDTYPE mode);
    UPDTYPE str2updmode(std::string str);
    /**
    * @brief        class for std::set ambiguity std::fixed xml
    */
    class set_amb : public virtual set_base
    {
    public:
        /** @brief default constructor. */
        set_amb();

        /** @brief default destructor. */
        virtual ~set_amb();

        /**
        * @brief settings check.
        */
        void check();

        /**
        * @brief settings help.
        */
        void help();

        /**
        * @brief  get upd type.
        * @return    UPDTYPE    type of UPD
        */
        UPDTYPE get_updtype();

        /**
        * @brief  get ambiguity type.
        * @return    AMB_TYPE    type of ambiguity
        */
        AMB_TYPE amb_type();

        /**
        * @brief  get ambiguity fixing mode .
        * @return    FIX_MODE    ambiguity fixing mode
        */
        FIX_MODE fix_mode();
        void fix_mode(FIX_MODE mode);

        /**
        * @brief  get upd mode .
        * @return    UPD_MODE    upd mode
        */
        UPD_MODE upd_mode();

        /**
        * @brief  get double-difference ambiguity mode .
        * @return    DD_MODEL    double-difference ambiguity mode
        */
        DD_MODEL dd_mode();

        /**
        * @brief  get lambda ratio .
        * @return    double    lambda ratio
        */
        double lambda_ratio();

        /**
        * @brief  get bootstrapping rate.
        * @return    double    bootstrapping rate
        */
        double bootstrapping();

        /**
        * @brief  check whether add leo.
        * @return
            @retval true    add leo
            @retval false    do not add leo
        */
        bool addleo();

        /**
        * @brief  check whether the baseline is independ.
        * @return
            @retval true    independ baseline
            @retval false    relevant baseline
        */
        bool independ_baseline();

        /**
        * @brief  get minimum common time of two observation arc.
        * @return    double    minimum common time of two observation arc
        */
        double min_common_time();

        /**
        * @brief  get the limit of baseline length.
        * @return    double    the limit of baseline length
        */
        double max_baseline_length();

        /**
        * @brief   widelane ambiguity interval.
        * @return    double    widelane ambiguity interval
        */
        double wl_interval();

        /**
        * @brief   get ambiguity decision.
        * @return    std::map<std::string, double>    ambiguity decision
        */
        std::map<std::string, double> get_amb_decision(std::string str);

        /**
        * @brief   whether take partial ambiguity std::fixed mode.
        * @return
            @retval true    take partial ambiguity std::fixed mode
            @retval false    do not take partial ambiguity std::fixed mode
        */
        bool part_ambfix();

        /**
        * @brief   value's size which take partial ambiguity std::fixed mode.
        * @return    int    value's size which take partial ambiguity std::fixed mode
        */
        int part_ambfix_num();

        /**
        * @brief   whether write carrier range
        * @return
            @retval true    write carrier range
            @retval false    do not write carrier range
        */
        bool carrier_range_out();

        /**
        * @brief   whether use carrier range
        * @return
            @retval true    use carrier range
            @retval false    do not use carrier range
        */
        bool apply_carrier_range();

        /**
        * @brief   whether std::set reference satellite.
        * @return
            @retval true    std::set reference satellite
            @retval false    do not std::set reference satellite
        */
        bool isSetRefSat();

        double FixFixSep(); ///< TODO

        double FloatFixSep(); ///< TODO

        /**
        * @brief   whether clear flag.
        * @return
            @retval true    clear flag
            @retval false    do not clear flag
        */
        bool clear_flag();

        /**
        * @brief get re-fix ambiguity settings.
        * @param[out]  last_fixepo_gap    last std::fixed epoch gap
        * @param[out]  min_fixed_num    minimum std::fixed num
        */
        void refixsettings(int &last_fixepo_gap, int &min_fixed_num);

        /**
        * @brief get full fix num
        * @return    int full fix num
        */
        int full_fix_num();

        std::map<std::string, std::string> ambiguity_engaged();  ///< zhshen for engaging ambiguity resolution to estimator

        FIX_MODE str2fixmode(std::string str);
        std::string fixmode2str(FIX_MODE mode);
        UPD_MODE str2upd_mode(std::string str);
        AMB_TYPE str2ambtype(std::string str);
        UPDTYPE str2updmode(std::string str);

    protected:
        std::map<std::string, std::map<std::string, double>> _default_decision = {
            {"EWL", {{"maxdev", 0.07}, {"maxsig", 0.10}, {"alpha", 1000}}},
            {"WL", {{"maxdev", 0.25}, {"maxsig", 0.10}, {"alpha", 1000}}},
            {"NL", {{"maxdev", 0.25}, {"maxsig", 0.10}, {"alpha", 1000}}}}; ///< default ambiguity decision

    private:
    };
}
#endif
