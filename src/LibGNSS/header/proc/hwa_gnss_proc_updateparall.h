/**
*
* @file            gupdatepar.h
* @brief        define update parameter IF model
*/

#ifndef hwa_gnss_proc_updateparall_H
#define hwa_gnss_proc_updateparall_H

#include <map>
#include "hwa_gnss_proc_UPDATEPAR.h"

namespace hwa_gnss
{

    /**
     * @brief class for parameter update
     */
    class gnss_proc_update_par_all : public gnss_proc_update_par
    {
    public:
        /**
         * @brief Construct a new t gupdateparALL object
         * @param[in]  cycleslip cycle slip
         * @param[in]  band_list band list
         */
        gnss_proc_update_par_all(std::shared_ptr<gnss_data_cycleslip> cycleslip, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list);
        /**
         * @brief Construct a new t gupdateparALL object
         * @param[in]  cycleslip_12 cycle slip12
         * @param[in]  cycleslip_13 cycle slip13
         * @param[in]  band_list band list
         */
        gnss_proc_update_par_all(std::shared_ptr<gnss_data_cycleslip> cycleslip_12, std::shared_ptr<gnss_data_cycleslip> cycleslip_13, std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_list);
        /**
         * @brief Construct a new t gupdateparALL object
         * @param[in]  cycleslip_12 cycle slip12
         * @param[in]  cycleslip_13 cycle slip13
         * @param[in]  cycleslip_14 cycle slip14
         * @param[in]  cycleslip_15 cycle slip15
         * @param[in]  band_list band list
         */
        gnss_proc_update_par_all(std::shared_ptr<gnss_data_cycleslip> cycleslip_12, std::shared_ptr<gnss_data_cycleslip> cycleslip_13, std::shared_ptr<gnss_data_cycleslip> cycleslip_14, std::shared_ptr<gnss_data_cycleslip> cycleslip_15, std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_list);
        /**
         * @brief Destroy the t gupdateparALL object
         */
        virtual ~gnss_proc_update_par_all();
        /**
         * @brief Get the all update parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameter
         * @param[in]  obsdata   observation
         */
        gnss_proc_update_par_info get_all_update_parameters(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata) override;

    protected:
        /**
         * @brief update amb parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  update_info update infomation
         */
        void _update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info) override;
        /**
         * @brief update amb parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  update_info update infomation
         */
        void _lite_update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info);
        /**
         * @brief update iono parameter
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  update_info update infomation
         */
        void _udpate_ion_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info);
        /**
         * @brief check log
         * @param[in]  cycleslip_12 cycle slip12
         * @param[in]  cycleslip_13 cycle slip13
         */
        void _check_log(std::shared_ptr<gnss_data_cycleslip> cycleslip_12, std::shared_ptr<gnss_data_cycleslip> cycleslip_13);

        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_list;    ///< band list
        std::shared_ptr<gnss_data_cycleslip> _cycleslip_13 = nullptr; ///< cycle slip13
        std::shared_ptr<gnss_data_cycleslip> _cycleslip_14 = nullptr; ///< cycle slip14
        std::shared_ptr<gnss_data_cycleslip> _cycleslip_15 = nullptr; ///< cycle slip15

        int _freq; ///< frequency
    };

}
#endif // !GUPDATEPARALL_H
