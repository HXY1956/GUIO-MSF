/**
*
* @verbatim
    The format of this block:
    <!--> NPP setting node <-->
    <npp>
        <client> YES </client>                               <!-- whether is client -->
        <comp_aug> YES </comp_aug>                           <!-- whether comput aug -->
        <npp_model> PPP_RTK </npp_model>                     <!-- npp model: NONE,VRS,URTK,PPP_RTK -->
        <npp_delaunay> NO </npp_delaunay>                    <!-- Whether to use Tironi triangle for correction interpolation -->
        <aug_limit  comp="0.015"  ion="0.0"  trop="0.0" />   <!-- Correction difference threshold between epochs: Comprehensive,iono,trop -->
        <reset_amb_ppprtk> YES </reset_amb_ppprtk>           <!-- Whether to reset the ambiguity if the ambiguity fix fails -->
        <self_cor> NO </self_cor>                            <!-- Whether to change this site by this site -->
        <obs_level> 0 </obs_level>                           <!-- Selected observation information��0-Original,1-Corrected by the number of corrections interpolated by 1 reference station...,n-Corrected by the number of corrections interpolated by n reference stations -->
    </npp>
* @file        gsetnpp.h
* @brief    xml setting for npp
*/

#ifndef hwa_set_npp_h
#define hwa_set_npp_h
#define XMLKEY_NPP "npp" ///< The defination of npp node in XML
#include <string>
#include <iostream>
#include "hwa_set_base.h"
#include "hwa_set_gproc.h"

using namespace hwa_base;
using namespace pugi;

namespace hwa_set
{
    /**
    *@brief    class for set npp xml
    */
    class set_npp : public virtual set_base
    {
    public:
        /** @brief default constructor. */
        set_npp();

        /** @brief default destructor. */
        ~set_npp();

        /**
        * @brief settings check.
        */
        void check();

        /**
        * @brief settings help.
        */
        void help();

        /**
        * @brief    check whether to use Tironi triangle for correction interpolation.
        * @return
            @retval true    use Tironi triangle for correction interpolation
            @retval false    do not use Tironi triangle for correction interpolation
        */
        bool npp_delaunay();

        /**
        * @brief    check whether compute aug.
        * @return
            @retval true    compute aug
            @retval false    do not compute aug
        */
        bool comp_aug(); // check if comprehensive_aug or not

        /**
        * @brief    check whether compute grid aug.
        * @return
            @retval true    compute grid aug
            @retval false    do not compute grid aug
        */
        bool grid_aug(); // check if comprehensive_aug or not

        /**
        * @brief    get correction difference threshold between epochs: comprehensive,iono,trop.
        * @param[out]    comp_aug    comprehensive correction difference threshold between epochs
        * @param[out]    ion_aug        iono correction difference threshold between epochs
        * @param[out]    trop_aug    trop correction difference threshold between epochs
        */
        void aug_limit(double &comp_aug, double &ion_aug, double &trop_aug); // Aug difference threshold between epochs

        /**
        * @brief    check whether to reset the ambiguity if the ambiguity fix fails.
        * @return
            @retval true    reset the ambiguity if the ambiguity fix fails
            @retval false    do not reset the ambiguity if the ambiguity fix fails
        */
        bool reset_amb_ppprtk();

        /**
        * @brief    check whether to change this site by this site.
        * @return
            @retval true    change this site by this site
            @retval false    do not change this site by this site
        */
        bool self_cor();

        /**
        * @brief    get    selected observation information.
        * @return    int    selected observation information
        */
        int obs_level();

        /**
        * @brief    check whether is client.
        * @return
            @retval true    is client
            @retval false    is not client
        */
        bool isClient(); // PPP-RTK identify Server or Client

         /**
        * @brief    check whether correct observation
        * @return
            @retval true    compute aug
            @retval false    do not compute aug
        */
        bool cor_obs(); // check whether correct observation

        NPP_MODEL npp_model();

    protected:
        //NPP_MODEL _nppmodel;
    private:
    };

} // namespace
#endif
