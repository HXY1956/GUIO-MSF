#ifndef hwa_base_filter_h
#define hwa_base_filter_h
#include "hwa_base_eigendef.h"
#include <boost/math/special_functions/digamma.hpp>
#include <ceres/ceres.h>
#include <random>   
#include <ceres/problem.h>
#include <iomanip>

namespace hwa_set {
    class set_base;
}

namespace hwa_base {
    double VecNorm(Vector x);

    enum Updater {
        EKF,
        UKF,
        CKF,
        PF,
        VBAKF,
        GSTM,
        GSTM_1,
        GSTM_2
    };

    Updater str2updater(std::string str);

    class base_updater {
    public:
        base_updater() {
            kappa_sig = 0;
            alpha_sig = 0.001;
            tau = 1000.0;
            proc_noise = 0.05;
            g0 = 0.85;
            e0 = 0.85;
            max_iter = 20;
            _num_particles = 500;
        }
        base_updater(hwa_set::set_base* _gset, std::string sensor);
        void set_particles(Matrix& Pk);
        void residual_resample(const Vector& weights, Vector_T<int>& indexes);
        int _meas_update_ekf(const Matrix& Hk, const Vector& Zk, const Matrix& Rk, Vector& Xk, Matrix& Pk);
        int _meas_update_vbakf(const Matrix& Hk, const Vector& Zk, const Matrix& Rk, Vector& Xk, Matrix& Pk);
        int _meas_update_gstm(Matrix& Hk, Vector& Zk, Matrix& Rk, Vector& Xk, Matrix& Pk);
        int _meas_update_ukf(Vector& Xk, Matrix& Pk, Matrix& Rk);
        int _meas_update_pf(Vector& Xk, Matrix& Pk, Matrix& Rk);
        int _meas_update(Matrix& Hk, Vector& Zk, Matrix& Rk, Vector& Xk, Matrix& Pk);

        void setRecpos(const Triple& c) { CarPos = c; };
        void setRange(const Vector& r) { IniRange = r;  };
        void setAncpos(const std::vector<Triple>& a) { AnchorPos = a; }
        void reset() {
            CarPos.resize(0, 0);
            IniRange.resize(0, 0);
            AnchorPos.clear();
            curr_iter = 0;
        }
        Updater filter;

    private:
        bool _particles_init = false;
        std::default_random_engine _particles_engine;
        std::normal_distribution<double> _particles_distribution;
        double kappa_sig;
        double alpha_sig;
        double tau;
        double proc_noise;
        double g0;
        double e0;
        int max_iter;
        int _num_particles;
        double barrier;
        int dof1;
        int dof2;
        double max_res_norm;
        Matrix _particles;
        Vector weights_nonnormalized, weights_normalized;
        Vector IniRange;
        Triple CarPos;
        std::vector<Triple> AnchorPos;
        int curr_iter = 0;
    };
};
   


#endif