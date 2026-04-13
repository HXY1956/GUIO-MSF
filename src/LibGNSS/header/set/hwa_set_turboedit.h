/**
*
* The format :
* @verbatim
  post-processing mode:
  <turboedit lite_mode="false" >
    <amb_output  valid="true"  />     
    <ephemeris   valid="true"  />    
    <check_pc   pc_limit="250" valid="true"  />
    <check_mw   mw_limit="4"   valid="true"  />
    <check_gf   gf_limit="1"   gf_rms_limit="2"   valid="true" />
    <check_sf   sf_limit="1"   valid="false" />
    <check_gap    gap_limit="20"    valid="true" />                                             
    <check_short  short_limit="10"  valid="true" />                                            
    <check_statistics  min_percent="60"  min_mean_nprn="4"  max_mean_namb="3"  valid="true" /> 
  </turboedit>

  real-time mode or lite-mode:
  <turboedit lite_mode="true" >
    <check_mw   valid="true"  />
    <check_gf   valid="true"  />
    <smooth_win  value="25"   />  smooth windows length
    <check_gap  gap_limit="1"  valid="true" />           
  </turboedit>

* @file            gsetturboedit.h
* @brief        turboedit set 
*/

#ifndef hwa_set_turboedit_h
#define hwa_set_turboedit_h

#define XMLKEY_TURBOEDIT "turboedit"

#include <map>
#include <string>
#include <iostream>
#include "hwa_set_base.h"
#include "hwa_base_fileconv.h"

using namespace hwa_base;

namespace hwa_set
{
    /**
    * @brief    class for set turboedit xml
    */
    class set_turboedit : public virtual set_base
    {
    public:
        /** @brief default constructor. */
        set_turboedit();

        /** @brief default destructor. */
        virtual ~set_turboedit(){};

        /**
        * @brief  check whether use lite mode.
        * @return
            @retval true    use lite mode
            @retval false    do not use lite mode
        */
        bool liteMode();

        /**
        * @brief  check whether output ambiguity.
        * @return
            @retval true    output ambiguity
            @retval false    do not output ambiguity
        */
        bool isAmbOutput();

        /**
        * @brief  check whether use broadcast emphemeris.
        * @return
            @retval true    use broadcast emphemeris
            @retval false    do not use broadcast emphemeris
        */
        bool isEphemeris();

        /**
        * @brief  check code observation.
        * @param[out]    pc_limit    pc threshold 
        * @return
            @retval true    use code observation
            @retval false    do not use code observation
        */
        bool checkPC(double &pc_limit);

        /**
        * @brief  check MW combination.
        * @param[out]    mw_limit    MW threshold
        * @return
            @retval true    use MW combination
            @retval false    do not use MW combination
        */
        bool checkMW(double &mw_limit);

        /**
        * @brief  check GF combination.
        * @param[out]    gf_limit        GF threshold
        * @param[out]    gf_rms_limit    GF RMS threshold
        * @return
            @retval true    use GF combination
            @retval false    do not use GF combination
        */
        bool checkGF(double &gf_limit, double &gf_rms_limit);

        /**
        * @brief  check single frequency.
        * @param[out]    sf_limit        single frequency threshold
        * @return
            @retval true    use single frequency
            @retval false    do not use single frequency
        */
        bool checkSingleFreq(double &sf_limit);

        /**
        * @brief  check data gap.
        * @param[out]    gap_limit        data gap threshold
        * @return
            @retval true    use data gap
            @retval false    do not use data gap
        */
        bool checkGap(int &gap_limit);

        /**
        * @brief  get smooth windows.
        * @return    int    smooth windows
        */
        int smoothWindows();

        /**
        * @brief  check short arc.
        * @param[out]    short_limit        short arc threshold
        * @return
            @retval true    use short arc
            @retval false    do not use short arc
        */
        bool checkShort(int &short_limit);

        /**
        * @brief  check statistics, for post network processing.
        * @param[out]    minPercent        TODO
        * @param[out]    minMeanNprn        TODO
        * @param[out]    maxMeanNamb        TODO
        * @return
            @retval true    use statistics
            @retval false    do not use statistics
        */
        bool checkStatistics(double &minPercent, int &minMeanNprn, int &maxMeanNamb);

        /**
         * @brief  get string outputs
         * @param[in] fmt file format
         * @return string : string outputs
         */
        std::string outputs(const std::string& fmt);

        /** @brief settings help. */
        void help();

        void check() {};

    protected:
        /**
         * @brief  get string output file
         * @param[in] fmt file format
         * @return string : string output file
         */
        std::string _outputs(const std::string& fmt);

        double _defaultPCLimit;         ///< default pc threshold
        double _defaultMWLimit;         ///< default MW threshold
        double _defaultGFLimit;         ///< default GF threshold
        double _defaultGFRmsLimit;      ///< default GR RMS threshold
        double _defaultSingleFreqLimit; ///< default single frequency threshold
        int _defaultGapArcLimit;        ///< default data gap threshold
        int _defaultShortArcLimit;      ///< default short arc threshold
        double _defaultMinPercent;      ///< TODO
        int _defaultMinMeanNprn;        ///< TODO
        int _defaultMaxMeanNamb;        ///< TODO
    };
}
#endif // !SETTURBOEDIT_H
