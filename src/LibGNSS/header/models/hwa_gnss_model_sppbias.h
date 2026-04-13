#ifndef hwa_gnss_model_sppBIAS_H
#define hwa_gnss_model_sppBIAS_H

#include "hwa_gnss_model_bias.h"

namespace hwa_gnss
{
    /** @brief model of bias.*/
    class gnss_proc_sppbias : public gnss_model_bias
    {
    public:
        /** @brief constructor. 
        *
        *param[in] spdlog                std::set spdlog control
        *param[in] settings            std::set base control
        */
        gnss_proc_sppbias(base_log spdlog, set_base *settings);

        /** @brief default destructor. */
        virtual ~gnss_proc_sppbias();

        /**
        * @brief combine equation
        * @param [in]  epoch       epoch
        * @param [in]  params      parameters
        * @param [in]  obs           observation data
        * @param [out] result      B P L matrix
        * @return
               *    @retval   false     combine equation unsuccessfully
               *    @retval   true      combine equation successfully
        */
        bool cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result) override;

        /**
         * @brief calculate rec/sat clk for epoch
         * @param [in]  obj         object
         * @param [in]  epo         epoch
         * @param [out] obj_clk     clk of object
         * @return
                *    @retval   false     no rec or sat clk
                *    @retval   true
         */
        void update_obj_clk(const std::string &obj, const base_time &epo, double clk) override;

        /**
         * @brief get satellite clk correction
         * @param [in]  sat_epoch   satellite epoch
         * @param [in]  sat         satellite
         * @param [in]  clk         satellite clk
         * @param [out]  dclk       satellite clk diff
         * @return
                *    @retval   true
         */
        double get_rec_clk(const std::string &obj) override; //add by xiongyun

    protected:
        /**
         * @brief calculate delay of tropo
         * @param [in]  epoch        current epoch
         * @param [in]  rec         satellite
         * @param [in]  param       parameter
         * @param [in]  site_ell    position of satellite
         * @param [in]  satdata     satellite data
         * @return double            delay of tropo
         */
        double tropoDelay(base_time &epoch, std::string &rec, base_allpar &param, Triple site_ell, gnss_data_sats &satdata);

        /**
         * @brief calculate delay of iono
         * @param [in]  epoch        current epoch
         * @param [in]  param       parameter
         * @param [in]  site_ell    position of satellite
         * @param [in]  satdata     data of satellite
         * @param [in]  gobs        observation data
         * @return double            delay of iono
         */
        double ionoDelay(base_time &epoch, base_allpar &param, Triple site_ell, gnss_data_sats &satdata, gnss_data_obs &gobs);

        /**
         * @brief calculate delay of isb
         * @param [in]  rec         reciever
         * @param [in]  param       parameter
         * @param [in]  sat            satellite
         * @param [in]  gobs        observation data
         * @return double            delay of isb
         */
        double isbDelay(base_allpar &param, std::string &sat, std::string &rec, gnss_data_obs &gobs);

        /**
         * @brief calculate delay of ifb
         * @param [in]  rec         reciever
         * @param [in]  param       parameter
         * @param [in]  sat            satellite
         * @param [in]  gobs        observation data
         * @return double            delay of ifb
         */
        double ifbDelay(base_allpar &param, std::string &sat, std::string &rec, gnss_data_obs &gobs);

        /**
         * @brief calculate rec/sat clk for epoch
         * @param [in]  epoch       epoch
         * @param [in]  sat         satellite
         * @param [in]  rec         receiver
         * @param [in]  param       all parameters
         * @param [out] gsatdata    data of satellite
         * @param [out] gobs        observation
         * @return      calculate theoretical value
         */
        double cmpObs(base_time &epoch, std::string &sat, std::string &rec, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs) override;

        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< index of band
        std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; ///< index of frequency

        std::string _crt_rec; ///< current recievers
        std::string _crt_sat; ///< current satellite
        GSYS _crt_sys;   ///< current system
    };
}

#endif