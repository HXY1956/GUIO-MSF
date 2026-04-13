#ifndef hwa_gnss_proc_lsqbase_H
#define hwa_gnss_proc_lsqbase_H

#include "hwa_base_eigendef.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_allpar.h"
#include "hwa_base_ioBigf.h"
#include "hwa_gnss_all_pcvneq.h"
#include "hwa_gnss_all_recover.h"
#include "hwa_gnss_model_stochastic.h"
#include "hwa_gnss_model.h"
#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_proc_lsqmatrix.h"
#include "hwa_gnss_proc_Updatepar.h"
#include <thread>

using namespace hwa_base;

namespace hwa_gnss
{

    /**
    *@brief       Class for lsq estimator include upadte,slove,recover
    */
    class gnss_proc_lsqbase
    {
    public:
        explicit gnss_proc_lsqbase();

        /** @brief    constructor */
        explicit gnss_proc_lsqbase(set_base *set);

        explicit gnss_proc_lsqbase(base_log spdlog, set_base *set);
        /** @brief  copy constructor */
        explicit gnss_proc_lsqbase(const gnss_proc_lsqbase &Other);

        /** @brief  default destructor */
        virtual ~gnss_proc_lsqbase();

        /** @brief  set log file */
        base_log spdlog() { return _spdlog; }
        void spdlog(base_log spdlog);

        /**
         * @brief cleart tempfile
         */
        void clear_tempfile();
        /**
        * @brief  add now parameter
        * @param[in] par new par preapred to add
        * return -1 for fail 1 for success
        */
        virtual int add_parameter(const base_par &par);

        /**
        * @brief  add the partype and the state mode
        * @param[in] par_type type of parameter
        * @param[in] order 0:white noise 1:random walk
        * @param[in] dt [unit:hour]
        * @param[in] noise process noise
        */
        void add_par_state_equ(par_type par_type, int order, double dt, double noise);
        /**
         * @brief Set the update parameters
         * @param[in]  updatepar update parameter
         */
        void set_update_par(std::shared_ptr<gnss_proc_update_par> updatepar);

        /**
        * @brief update all lsq par with now obs data
        * @note according to obsdata update amb par , and time update outsate par
        * @        use matrix remove way may have little loss of accruacy
        * @param[in] epoch
        * @param[in] obsdata observ info data
        * @param[in] if use matrix remove way
        * @return -1 for fail 1 for success
        */
        virtual bool update_parameter(const base_time &epoch, std::vector<gnss_data_sats> &obsdata, bool matrix_remove = true, bool write_temp = true); // add write_temp by glfeng

        /**
        * @brief update all lsq par with now obs data
        * @note according to obsdata update amb par , and time update outsate par
        * @        use matrix remove way may have little loss of accruacy
        * @param[in] phase phase only or both
        * @param[in] epoch
        * @param[in] obsdata observ info data
        * @param[in] if use matrix remove way
        * @return -1 for fail 1 for success
        */
        virtual bool update_parameter(bool phase, const base_time &epoch, std::vector<gnss_data_sats> &obsdata, bool matrix_remove = true, bool write_temp = true); //add by xiongyun

        /**
        * @brief set lsq par
        * @note this set will clean the par and other info before
        * @param[in] parameters all parmaters preapred to add
        */
        virtual int set_parameter(const base_allpar &parameters);

        /**
        * @brief add new observ equations[newmat matrix format]
        * @param[in] B coeff of equations
        * @param[in] P weight of equations
        * @param[in] l res of equations
        * @param[in] epoch time of equations
        */
        int add_equation(const Matrix &B, const Diag &P, const Vector &l, const base_time &epoch = base_time());

        /**
        * @brief add new observ equations[lsqmatrix format]
        * @param[in] equ equations info
        * @param[in] epoch time of equations
        */
        int add_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch = base_time(), bool write_temp = true);

        /**
        * @brief add new observ equations[lsqmatrix format]
        * @param[in] phase phase only or both
        * @param[in] equ equations info
        * @param[in] epoch time of equations
        * @param[in] if write equation
        */
        int add_equation(bool phase, const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch = base_time(), bool write_temp = true); //add by xiongyun
        /**
         * @brief Get the equ observation total num
         * @return int number of observation
         */
        int get_equ_obs_total_num();

        /** @brief del old observ equations[lsqmatrix format]
        * @param[in] equ equations info
        * @param[in] epoch time of equations
        */
        int del_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch = base_time());

        /**
        * @brief write the equtions information to tempfile for recovering later[lsqmatrix format
        * @param[in] equ equations of lsqmatrix format
        * @param[in] epoch time of equations
        * @param[in] info information of equations
        * @return true for success false for fail
        */
        bool write_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch);

        /**
        * @brief write the equtions information to tempfile for recovering later[lsqmatrix format
        * @param[in] equ equations of lsqmatrix format
        * @param[in] epoch time of equations
        * @param[in] phase phase only or both
        * @return true for success false for fail
        */
        bool write_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch, bool phase); //add by xiongyun

        /**
        * @brief remove the specified par in the lsq estimator
        * @note idx begin from 1
        */
        virtual int remove_parameter(const int &idx, bool write_temp = true);   // add write_temp by glfeng
        virtual int remove_parameter(std::vector<int> &idx, bool write_temp = true); // add write_temp by glfeng

        // sovle the least squares
        /**
        * @brief solve the equtions by least squares
        * @note result save into the dx
        */
        virtual int solve_NEQ();

        /**
        * @brief solve the equtions by least squares without inverse the matrix,generally use for lsq_epo
        * @note result save into the dx
        */
        virtual void solve_x();

        /**
        * @brief recover the removed par by information in tempfile 
        * @note recover should after solving equations and have temp file
        *        recover result will save into the gnss_all_recover data struct
        */
        virtual bool recover_parameters(gnss_all_recover &allrecover);

        /** @brief get the final solution of parameters;save result to gnss_all_recover */
        void get_result_parameter(gnss_all_recover &allrecover);

        /** @brief add apriori for all parameter */
        void add_apriori_weight();

        /**
        * @brief add apriori for specified parameter
        * @note  idx begin from 1
        */
        void add_apriori_weight(const int &idx);

        /**
        * @brief add specified apriori for specified parameter
        * @note  idx begin from 1
        * @param[in] idx specified location
        * @param[in] value given intial value
        * @param[in] weight given apriori weight
        */
        void add_apriori_weight(const int &idx, const double &value, const double &weight);

        /** @brief reset filename of tempfile */
        void reset_tempfile(std::string filename);

        /** @brief add ISB/IFB constraint */
        int lsq_sysbias_constraint();

        /** @brief add CLK zero-mean constraint */
        int lsq_clk_constraint();

        /** @brief add minimum constraint */
        bool lsq_minimum_constraint(std::string minimum_constraint, std::set<std::string> core_station, double sig);

        /** @brief add helmert translation constraint */
        bool lsq_helmert_constraint(HMTCONSTR hmtcst);

        int lsq_antpcv_constraint();
        int lsq_antpcv_addneq(gnss_all_pcvneq *pcvneq);

        /** @brief print matrix of NEQ(BTPB) and W(BTPL) */
        void print_matrx();

        /** @brief Get NEQ Matrix*/
        Symmetric NEQ() const;
        /**
         * @brief Get W Matrix
         */
        Vector W();
        /**
         * @brief Get NEQ
         */
        V_Symmetric v_NEQ() const;
        /**
         * @brief Get W
         */
        V_Vector v_W() const;

        /** @brief get covariance matrix */
        const Symmetric &Qx() const;

        /** @brief get coerrection of parametes */
        Vector dx() const;

        /** @brief get covariance of coerrection of parametes */
        Vector stdx() const;
        /**
         * @brief get Qx value
         * @param[in]  col       column
         * @param[in]  row       row
         * @return double value of certain Qx
         */
        double Qx(const int &col, const int &row) const;

        /** @brief get coerrection of specified parametes */
        double dx(int idx) const;

        /** @brief get the covariance of specified parameter */
        double stdx(int idx) const;

        /** @brief get sgima0 */
        double sigma0() const;

        /** @brief get sum of res squares */
        double vtpv() const;

        /** @brief get total observation numbers*/
        int nobs_total() const;

        /** @brief get nubmer of parameter*/
        int npar_number() const;

        /** @brief get current time  */
        base_time epo() const;

        /** @brief set current time */
        void epo(const base_time &t);

        /** @brief get lsq mode */
        LSQ_MODE mode();

        /** @brief set log file */
        void setlog(base_log spdlog);

        // change HXJ
        /** @brief set NEQ Matrix specified location */
        void change_NEQ(int row, int col, double xx);

        /** @brief set Qx Matrix specified location */
        void change_Qx(int row, int col, double xx);
        void change_Qx(Symmetric Qx);

        /** @brief set dx  specified location */
        void change_dx(int n, double xx);
        void change_dx(Vector dx);

        /** @brief set stdx specified location */
        void change_stdx(int n, double xx);
        void change_stdx(Vector stdx);

        /** @brief set sigma0 by  specified sigma */
        void change_sigma0(double sigma);

        /** @brief set vtpv by  specified value */
        void change_vtpv(double xx);
        /** @brief change NEW and W */
        void set_new_NEQ(const V_Vector &W, const V_Symmetric &NEQ);

        //// Change glfeng
        ///** @brief get one site max amb in one epoch */
        int reset_npar_total(const base_par &par);

        base_allpar _x_solve;         ///< all parameter in lsq estimator
        int _obs_total_num_epo = 0; ///< totoal observation number of current epoch (added by xiongyun)

    protected:
        /**
        * @brief  write the removed NEQ W to tempfile
        * @note idx from1
        */
        int _write_coefficient(int idx);
        // remove_id from 1;
        int _write_parchage(const std::vector<int> &remove_id);

        /** @brief recover parameter */
        void _recover_par(base_io_READTEMP &tempfile_in, gnss_all_recover &_allrecover);
        void _recover_par_part(base_io_READTEMP &tempfile_in, gnss_all_recover &_allrecover);
        void _recover_par_swap(base_io_READTEMP &tempfile_in);

        /** @brief recover obs */
        void _recover_obs(base_io_READTEMP &tempfile_in, gnss_all_recover &_allrecover);

        /** @brief remove zero element in NEQ matrix */
        void _remove_zero_element(Symmetric &B, Vector &l, int idx);

        /** @brief solve equation*/
        void _solve_equation(const Symmetric &NEQ, const Vector &W, Vector &ans, Vector &Q);
        void _solve_x(const Symmetric &NEQ, const Vector &W, Vector &ans);

        base_log _spdlog = nullptr; ///< spdlog file
        set_base *_gset = nullptr;

        V_Vector _W;      ///< BTPL Matrix
        V_Symmetric _NEQ; ///< BTPB Matrix

        Symmetric _Qx;    ///< storage Qx after solve
        Vector _dx;       ///< correction of all parameter
        Vector _dx_final; ///< last correction of all parameter
        Vector _stdx;     ///< covarience of correction of all parameter

        double _res_obs;    ///< sum of residuals of currecnt epoch
        double _vtpv;       ///< sum of residuals of all epoch
        double _sigma0;     ///< error in unit weight
        int _obs_total_num; ///< totoal observation number
        int _npar_tot_num;  ///< total par number include ll eliminated parameters

        base_time _epo;                        ///< current time
        base_time _beg;                        ///< begin time
        base_time _end;                        ///< end time
        double _interval;                    ///< intv glfeng 2019.5.1
        base_io_bigf *_tempfile = nullptr;      ///< temp file
        LSQ_MODE _mode;                       ///< lsq model
        int _buffer_size = 1024 * 1000 * 10; ///< tempfile buffer size

        std::shared_ptr<gnss_proc_update_par> _update_lsqpar;

        base_mutex _lsq_mtx;

        int size_int = sizeof(int);
        int size_dbl = sizeof(double);
    };

    // mainly for recover or exctract par
    base_time str2gtime(std::string str_time);

} // namespace

#endif //Glsq_H
