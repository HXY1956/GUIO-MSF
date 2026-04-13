#ifndef hwa_gnss_proc_UPDATEPAR_H
#define hwa_gnss_proc_UPDATEPAR_H

#include "hwa_base_allpar.h"
#include "hwa_base_time.h"
#include "hwa_set_base.h"
#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_proc_lsqmatrix.h"
#include "hwa_gnss_model_stochastic.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_proc_update_par_info
    {
    public:
        /**
         * @brief Construct a new t gupdateparinfo object
         */
        gnss_proc_update_par_info();
        /**
         * @brief Destroy the t gupdateparinfo object
         */
        ~gnss_proc_update_par_info();
        /**
         * @brief wheather exist certain index
         * @param[in]  id        id
         * @return true exist
         * @return false not exist
         */
        bool exist(int id);

        /**
         * @brief add remove info
         * @param[in]  id        index for base_allpar
         */
        void add(int id);
        /**
         * @brief add remove info
         * @param[in]  newpar    new parameter
         */
        void add(base_par newpar);

        /**
         * @brief add update info
         * @param[in]  id        index
         * @param[in]  newpar    new parameter
         */
        void add(int id, base_par newpar);

        /**
         * @brief add update info with state equ
         * @param[in]  update_par update parameter
         * @param[in]  state_equ  equ
         */
        void add(std::vector<std::pair<int, base_par>> update_par, gnss_proc_lsq_equationmatrix state_equ);

        /**
         * @brief get remove info
         * @param[in]  remove_id remove index
         */
        void get(std::vector<int> &remove_id);

        /**
         * @brief get new parlist info
         * @param[in]  newparlist new parameters list
         */
        void get(std::vector<base_par> &newparlist);

        /**
         * @brief get state equ info
         * @param[in]  update_newpar new update parameters list
         * @param[in]  equ       equ
         */
        void get(std::vector<base_par> &update_newpar, gnss_proc_lsq_equationmatrix &equ); //change by xiongyun

        /**
         * @brief remove all info
         * @param[in]  remove_id index of remode
         * @param[in]  newparlist  new parameters list
         * @param[in]  equ_parlist equ parameters lisr
         * @param[in]  equ       equ
         */
        void get(std::vector<int> &remove_id, std::vector<base_par> &newparlist, std::vector<base_par> &equ_parlist, gnss_proc_lsq_equationmatrix &equ);

    private:
        std::set<int> _remove_id;             ///< TODO COMMENT
        std::vector<base_par> _new_parlist;     ///< TODO COMMENT
        std::vector<base_par> _equ_parlist;     ///< TODO COMMENT
        gnss_proc_lsq_equationmatrix _state_equ; ///< TODO COMMENT
    };

    class gnss_proc_update_par
    {
    public:
        /**
         * @brief Construct a new t gupdatepar object
         */
        gnss_proc_update_par();
        /**
         * @brief Destroy the t gupdatepar object
         */
        virtual ~gnss_proc_update_par();
        /**
         * @brief Construct a new t gupdatepar object
         * @param[in]  Other     update parameters
         */
        gnss_proc_update_par(const gnss_proc_update_par &Other);

        /**
         * @brief Set the interval
         * @param[in]  interval  interval
         */
        void set_interval(double interval);

        /**
         * @brief Set the sig ion
         * @param[in]  sigion    sig of iono
         */
        void set_sig_ion(double sigion);

        /**
         * @brief Set the cycle slip
         * @param[in]  cycleslip cycle slip
         */
        void set_cycleslip(std::shared_ptr<gnss_data_cycleslip> cycleslip);

        /**
         * @brief Set the amb update way
         * @param[in]  way       way of update amb
         */
        void set_amb_update_way(bool way);

        /**
         * @brief Set the par state mode
         * @param[in]  type      parameter type
         * @param[in]  order     order
         * @param[in]  dt        delta value
         * @param[in]  noise     noise
         */
        void set_par_state_mode(par_type type, int order, double dt, double noise);

        /**
         * @brief update amb parameter
         * @param[in]  epoch     epoch
         * @param[in]  allpars   all parameters
         * @param[in]  obsdata   observation data
         * @param[in]  update_info update infomation
         */
        void update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info);

        /**
        * @brief  all lsq par with now obs data
        * @note according to obsdata update amb par , and time update outsate par
        * @param[in] update epoch
        * @param[in] all lsq updateparameters
        * @param[in] obsdata observ info data
        * @return all update idx and newpar( if newpar is none represent only remove but not add) 
        */
        virtual gnss_proc_update_par_info get_all_update_parameters(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata);

    protected:
        /**
         * @brief update amb parameter
         */
        virtual void _update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info) = 0;

        /**
         * @brief update process parameter
         * @param[in]  epoch     epoch
         * @param[in]  allpars   all parameters
         * @param[in]  update_info update infomation
         */
        void _update_process_pars(const base_time &epoch, base_allpar &allpars, gnss_proc_update_par_info &update_info);

        /**
         * @brief update one process par
         * @param[in]  epoch     epoch
         * @param[in]  update_id index of update
         * @param[in]  allpars   all parameters
         * @param[in]  update_info update infomation
         */
        void _update_state_par(const base_time &epoch, int update_id, base_allpar &allpars, gnss_proc_update_par_info &update_info);

        //TODO COMMENT
        /**
         * @brief update process parameter
         * @param[in]  epoch     epoch
         * @param[in]  update_id index of update
         * @param[in]  allpars   all parameters
         * @param[in]  update_info update infomation
         */
        void _update_process_par(const base_time &epoch, int update_id, base_allpar &allpars, gnss_proc_update_par_info &update_info);

        //TODO COMMENT
        /**
         * @brief update gps station ifb parameter
         * @param[in]  epoch     epoch
         * @param[in]  update_id index of update
         * @param[in]  allpars   all parameters
         * @param[in]  update_info update infomation
         */
        void _udpate_gps_rec_ifb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info);

    protected:
        double _intv = 0.0;     ///< interval
        double _sig_ion = 9000; ///< TODO COMMENT

        std::shared_ptr<gnss_data_cycleslip> _cycleslip;    ///< cycle slip
        std::map<par_type, gnss_model_state_mode> _state_mode; ///< mode of state
        bool _lite_update_amb;                  ///< amb parameter update list
    };

}

#endif /* GUPDATEPAR_H */