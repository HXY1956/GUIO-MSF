/**
* @file            gupdatepar.h
* @brief        define update parameter IF model
*/

#ifndef hwa_gnss_proc_updateparif_H
#define hwa_gnss_proc_updateparif_H

#include <map>
#include "hwa_gnss_proc_Updatepar.h"

namespace hwa_gnss
{
    /**
     * @brief class for IONO free parameter update
     */
    class gnss_proc_update_par_if : public gnss_proc_update_par
    {
    public:
        /**
         * @brief Construct a new t gupdateparIF object
         * @param[in]  cycleslip cycle slip
         * @param[in]  band_list band list
         */
        gnss_proc_update_par_if(std::shared_ptr<gnss_data_cycleslip> cycleslip, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list);
        //gnss_proc_update_par_if(std::shared_ptr<gnss_data_cycleslip> cycleslip, set_base* set);//add by xiongyun
        /**
         * @brief Destroy the t gupdateparIF object
         */
        virtual ~gnss_proc_update_par_if();
        /**
         * @brief Set the break ambiguity by observation
         * @param[in]  isSet     whether std::set
         */
        void set_break_amb_by_obs(bool isSet) { isBreakObs = isSet; };

    protected:
        /**
         * @brief update amb parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  update_info update infomation
         */
        virtual void _update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info) override;
        /**
         * @brief update one type amb parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  amb_type  type of amb
         * @param[in]  _cycleslip cycle slip
         * @param[in]  update_info update infomation
         */
        void _update_one_type_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, par_type amb_type, gnss_data_cycleslip *_cycleslip, gnss_proc_update_par_info &update_info);
        // add by xiongyun for realtime
        /**
         * @brief update one type amb parameters for realtime
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  amb_type  type of amb
         * @param[in]  update_info update infomation
         */
        void _update_one_type_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, par_type amb_type, gnss_proc_update_par_info &update_info);

    protected:
        std::map<std::pair<std::string, std::string>, int> _amb_id;        ///< id of amb
        bool isBreakObs = true;                        ///< whether break observation
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_list; ///< band list
    };
    /**
     * @brief Class for IONO free 1X parameter update
     */
    class gnss_proc_update_par_if_1X : public gnss_proc_update_par_if
    {
    public:
        /**
         * @brief Construct a new t gupdateparIF 1X object
         * @param[in]  cycleslip12 cycle slip12
         * @param[in]  band_list band list
         */
        gnss_proc_update_par_if_1X(std::shared_ptr<gnss_data_cycleslip> cycleslip12, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list);
        /**
         * @brief Construct a new t gupdateparIF 1X object
         * @param[in]  cycleslip12 cycle slip12
         * @param[in]  cycleslip13 cycle slip13
         * @param[in]  band_list band list
         */
        gnss_proc_update_par_if_1X(std::shared_ptr<gnss_data_cycleslip> cycleslip12, std::shared_ptr<gnss_data_cycleslip> cycleslip13, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list);
        /**
         * @brief Destroy the t gupdateparIF 1X object
         */
        virtual ~gnss_proc_update_par_if_1X();
        /**
         * @brief Set the cycleslip13 object
         * @param[in]  cycleslip13 cycle slip13
         */
        void set_cycleslip13(std::shared_ptr<gnss_data_cycleslip> cycleslip13);
        /**
         * @brief Set the cycleslip14 object
         * @param[in]  cycleslip14 cycle slip14
         */
        void set_cycleslip14(std::shared_ptr<gnss_data_cycleslip> cycleslip14);
        /**
         * @brief Set the cycleslip15 object
         * @param[in]  cycleslip15 cycle slip15
         */
        void set_cycleslip15(std::shared_ptr<gnss_data_cycleslip> cycleslip15);
        /**
         * @brief Get the all update parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   all observation
         * @return gnss_proc_update_par_info update infomation
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

        std::shared_ptr<gnss_data_cycleslip> _cycleslip13 = nullptr; ///< cycle spli13
        std::shared_ptr<gnss_data_cycleslip> _cycleslip14 = nullptr; ///< cycle spli14
        std::shared_ptr<gnss_data_cycleslip> _cycleslip15 = nullptr; ///< cycle spli15
    };
    /**
     * @brief Class for wide lane parameter update
     */
    class t_updateparWL : public gnss_proc_update_par_if
    {
    public:
        /**
         * @brief Construct a new t updateparWL object
         * @param[in]  cycleslip cycle slip
         * @param[in]  band_list band list
         */
        t_updateparWL(std::shared_ptr<gnss_data_cycleslip> cycleslip, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list);
        /**
         * @brief Destroy the t updateparWL object
         */
        virtual ~t_updateparWL();

    protected:
        /**
         * @brief update amb parameters
         * @param[in]  epoch     time
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation
         * @param[in]  update_info update infomation
         */
        virtual void _update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info) override;
    };

}
#endif // !GUPDATEPARIF_H
