#ifndef hwa_uwb_proc_h
#define hwa_uwb_proc_h
#include <random>
#include <string>
#include <vector>
#include <map>
#include "hwa_set_uwb.h"
#include "hwa_base_mutex.h"
#include "hwa_base_iof.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_xml.h"
#include "hwa_base_eigendef.h"
#include "hwa_uwb_data.h"
#include "hwa_uwb_coder.h"
#include <boost/math/special_functions/digamma.hpp>
#include <ceres/ceres.h>
#include <ceres/problem.h>

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_uwb {
    struct CostFunctor {
        CostFunctor(double r_i, const Triple& x_u_i, double gamma)
            : r_i(r_i), x_u_i(x_u_i), gamma(gamma) {}
        template <typename T>
        bool operator()(const T* const x_b, const T* const mu_i, T* residual) const {
            T distance = sqrt((x_b[0] - T(x_u_i[0])) * (x_b[0] - T(x_u_i[0])) +
                (x_b[1] - T(x_u_i[1])) * (x_b[1] - T(x_u_i[1])) +
                (x_b[2] - T(x_u_i[2])) * (x_b[2] - T(x_u_i[2])));
            residual[0] = (T(r_i) - distance - mu_i[0]) * (T(r_i) - distance - mu_i[0]);

            residual[0] += gamma * ceres::abs(mu_i[0]);
            return true;
        }
        double r_i;
        Triple x_u_i;
        double gamma;
    };

    struct CostFunctor_xb {
        CostFunctor_xb(const Triple& x_b_predict, double alpha)
            : x_b_predict(x_b_predict), alpha(alpha) {}
        template <typename T>
        bool operator()(const T* const x_b, T* residual) const {
            T distance = ceres::abs(x_b[0] - T(x_b_predict[0])) +
                ceres::abs(x_b[1] - T(x_b_predict[1])) +
                ceres::abs(x_b[2] - T(x_b_predict[2]));
            residual[0] = alpha * distance;
            return true;
        }
        Triple x_b_predict;
        double alpha;
    };

    struct CostFunctor_R {
        CostFunctor_R(double r_i, const Triple& x_u_i, double gamma)
            : r_i(r_i), x_u_i(x_u_i), gamma(gamma) {}

        template <typename T>
        bool operator()(const T* const x_b, const T* const mu_i, T* residual) const {
            T distance = sqrt((x_b[0] - T(x_u_i[0])) * (x_b[0] - T(x_u_i[0])) +
                (x_b[1] - T(x_u_i[1])) * (x_b[1] - T(x_u_i[1])) +
                (x_b[2] - T(x_u_i[2])) * (x_b[2] - T(x_u_i[2])));
            residual[0] = (T(r_i) - distance - mu_i[0]) * (T(r_i) - distance - mu_i[0]);
            residual[0] += gamma / ceres::abs(T(r_i) - distance - mu_i[0]) * ceres::abs(mu_i[0]);
            return true;
        }
        double r_i;
        Triple x_u_i;
        double gamma;
    };

    struct CostFunctor_R1 {
        CostFunctor_R1(double r_i, const Triple& x_u_i, double gamma)
            : r_i(r_i), x_u_i(x_u_i), gamma(gamma) {}

        template <typename T>
        bool operator()(const T* const x_b, const T* const mu_i, T* residual) const {
            T distance = sqrt((x_b[0] - T(x_u_i[0])) * (x_b[0] - T(x_u_i[0])) +
                (x_b[1] - T(x_u_i[1])) * (x_b[1] - T(x_u_i[1])) +
                (x_b[2] - T(x_u_i[2])) * (x_b[2] - T(x_u_i[2])));
            residual[0] = (T(r_i) - distance - mu_i[0]) * (T(r_i) - distance - mu_i[0]);
            residual[0] += gamma / ceres::abs(T(r_i) - distance) * ceres::abs(mu_i[0]);
            return true;
        }

        double r_i;
        Triple x_u_i;
        double gamma;
    };

    struct TotalResidual {
        TotalResidual(const std::vector<Triple>& x_u, const std::vector<double>& R, double gamma)
            : x_u(x_u), R(R), gamma(gamma) {}

        template <typename T>
        bool operator()(const T* const x_b, const T* const mu, T* residual) const {
            T sum_of_squares = T(0);
            for (size_t i = 0; i < x_u.size(); ++i) {
                T distance = sqrt((x_b[0] - T(x_u[i][0])) * (x_b[0] - T(x_u[i][0])) +
                    (x_b[1] - T(x_u[i][1])) * (x_b[1] - T(x_u[i][1])) +
                    (x_b[2] - T(x_u[i][2])) * (x_b[2] - T(x_u[i][2])));

                T range_error = T(R[i]) - distance - mu[i];
                sum_of_squares += range_error * range_error;
            }
            residual[0] = sqrt(sum_of_squares);
            for (size_t i = 0; i < x_u.size(); ++i) {
                residual[0] += gamma * ceres::abs(mu[i]);
            }
            return true;
        }

        const std::vector<Triple>& x_u;
        const std::vector<double>& R;
        double gamma;
    };
}

namespace hwa_uwb
{
    enum PenalType{
        SIGMOID,
        EXP,
        LOG,
        LINEAR,
    };

    PenalType str2Penal(std::string s);

    class uwb_proc
    {
    public:
        uwb_proc() {};
        explicit uwb_proc(hwa_set::set_base* gset, std::string mark, base_data* data);
        virtual ~uwb_proc();

        virtual int ProcessOneEpoch(const base_time& now);
        virtual int ProcessBatch(const base_time& beg, const base_time& end);
        Triple _get_pos();
        Triple _get_pos_std();
        virtual bool _prepareData(const base_time& now);
        void _preprocess();
        void _nlos_detect();
        void _smooth();
        std::map<std::string, Triple> _get_anchor_info();
        UWB_WEIGHT _get_wgt_type();
        hwa_map_id_uwbnode _get_uwb_epo();
        int _get_iter();
        void _remove_anchor(const std::string& name);
        void _remove_anchor(const bool& indoor = false, const bool& = false);
        bool _particles_init = false;
        void set_particles();
        void residual_resample(const Vector& weights, Vector& indexes);
        int _get_nr() { return uwb_epo.size(); };
        int _outlierDetect(const Vector& v, int& outlier, double& lambda);
        int _outlierDetect(const Vector& v, int& outlier);
        void _posterioriTest(const Matrix& A, const Matrix& P, const Vector& l,
            const Vector& dx, const Matrix& Q, Vector& v_norm, double& vtpv);
        void _set_pos(const Triple& pos);
        Triple _get_pos_psd();
        SO3 _get_pos_var();
        Matrix& _get_Pk();
        double _get_ts();
        base_time _get_uwb_time();

        std::vector<std::string> valid_node;
        Vector IniRange;
        Vector CrtRange;
        std::vector<Triple> AnchorPos;
        std::vector<std::string> _anchor_list;

    protected:
        virtual void _resetMatrix();
        virtual int _setMeas(const bool& isfirst = true);
        virtual void time_update(double kfts);
        virtual int obs_update(const base_time& now);
        virtual int _uwb_init();
        virtual int _avaliable(const base_time& crt);

        void _print_head();
        void _write();
        uwb_data* uwbdata;
        void _recoverPk();
        void _restorePk();
        void _reTime(const base_time& now);
        void _res_output(const Vector& v);
        void _error_compensation(Vector v);
        void _extract_value();
        int nlos_est(std::vector<double>& mu, Triple pos_uwb_ecef);
        std::vector<double> generateArithmeticSequence(double r, double Z);
        void generateAllCombinations(
            const std::vector<std::vector<double>>& NLOS,
            std::vector<double>& current_combination,
            int index,
            std::vector<std::vector<double>>& all_combinations);
        void Optimization(double& Cost, Triple& x_b, const std::vector<Triple> x_u, std::vector<double>& mu, const Vector Residuals_Original, Vector r);
        void _nlos_detectbypos(Triple pos_uwb);

        std::vector<std::string> _nlos;

        bool _ut(Triple& param, Vector& Xk, Matrix& Pk);
        bool _meas_updata_ukf(Triple& param, Vector& Xk, Matrix& Pk, Vector& Zk, Matrix& Rk);
        bool _meas_updata_ckf(Triple& param, Vector& Xk, Matrix& Pk, Vector& Zk, Matrix& Rk);
        bool _meas_updata_pf(Triple& param, Vector& Xk, Matrix& Pk, Vector& Zk, Matrix& Rk);

        double _sample;
        int _num_particles = 0;
        std::default_random_engine _particles_engine;
        std::normal_distribution<double> _particles_distribution;
        Matrix _particles;
        Vector weights_nonnormalized, weights_normalized;

        PenalType penal;
        Matrix Sk, xSig, zSig, cSig, kSig;
        Vector zSigBar;
        std::string filter;
        std::string filter_gnss_copy;
        double proc_noise_gnss_copy = 0;
        double gama;
        double alpha;
        bool pred_constraint;
        double barrior;
        double kappa_sig;
        double alpha_sig;
        double sigmoid_parameter;
        double sigmoid_threshold;
        double tolerance;
        double log_parameter;
        double exp_parameter;
        std::vector<std::string> anchorname;
        bool SDP;
        bool interpolation = false;
        double interpolation_noise = 0.05;
        bool posterior;
        bool smooth;
        bool addnoise;
        int smooth_point;
        double meas_range_std;
        double best_range_std;
        bool output_res;
        double range_lim, snr_lim;
        double hdop, vdop, pdop;
        double max_res_norm;
        double e0;
        double g0;
        int dof1;
        int dof2;
        int max_iter;
        double tau;
        double proc_noise;
        int nq;
        int nr;
        int _iter;
        std::vector<std::string> _indoor_anchor_list, _outdoor_anchor_list;
        Triple _indoor_mean_crd, _outdoor_mean_crd;
        std::map<std::string, Triple> _anchor_info;
        hwa_map_id_uwbnode uwb_epo;
        int _valid_node_num;
        double _ts;
        std::string _site;
        Triple _pos, _initial_pos_std, _pos_psd;

    private:
        bool _is_first;
        double _intv;
        base_time _now_uwb;

        Matrix _init_Pk;
        UWB_WEIGHT _wgt_type;

        int _dim;

        Matrix Hk, Rk;
        Vector Zk;

        Matrix Ft_uwb, Pk_uwb, Hk_uwb, Rk_uwb, Pk_Sav_uwb, Phik_uwb;                /// Kalman Matrix
        Vector Xk_uwb, Zk_uwb, Qt_uwb, Rt_uwb, rts_uwb, Pmax_uwb, Pmin_uwb,
            Rmax_uwb, Rmin_uwb, Rb_uwb, Rbeta_uwb,
            FBTau_uwb, FBMax_uwb, FBXk_uwb, FBTotal_uwb;                    /// Kalman Vector

        hwa_base::base_iof* _fuwb = nullptr;
        hwa_base::base_iof* _fuwbres = nullptr;
        std::map<std::string, std::ofstream> _outfile;

    };
}

#endif
