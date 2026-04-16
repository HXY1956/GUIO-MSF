#ifndef hwa_ins_proc_h
#define hwa_ins_proc_h

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "windows.h"
#include "hwa_set_ins.h"
#include "hwa_set_out.h"
#include "hwa_base_earth.h"
#include "hwa_base_quaternion.h"
#include "hwa_base_mutex.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_xml.h"
#include "hwa_base_posdata.h"
#include "hwa_base_iof.h"
#include "hwa_base_globaltrans.h"
#include "hwa_ins_data.h"
#include "hwa_ins_base_utility.h"
#include "hwa_ins_base_avar.h"

namespace hwa_ins {

    enum ins_fuse_mode {
        VIRTUAL,
        STACK
    };

    ins_fuse_mode str2ins_fuse_mode(std::string s);

    struct mimu {
        int _num_of_imu_axiliary = 0;
        std::map<int, SO3> _R_imui_imu0;
        std::map<int, Triple> _p_imui_imu0;
    };

    struct sins2
    {
        base_quat qeb;
        SO3  Cne;
        Triple  ve, pos_ecef, eb, db;
    };

    struct IMU_MSG
    {
        double t;
        Triple wm, vm;
    };

    class imu
    {
    public:
        explicit imu();
        ~imu() {};
        Triple phi_cone_crt(const std::vector<Triple>& wm);
        Triple phi_poly_crt(const std::vector<Triple>& wm);
        Triple vm_cone_crt(const std::vector<Triple>& wm, const std::vector<Triple>& vm);
        Triple vm_poly_crt(const std::vector<Triple>& wm, const std::vector<Triple>& vm);
        void Update(const std::vector<Triple>& wm, const std::vector<Triple>& vm, const ins_scheme& scm);
        Triple phim, dvbm;
    private:
        Triple wm_1, vm_1;        /// last increment.
        double pf1, pf2;                /// flag of first time.
    };

	class ins_obj {
    public:
        explicit ins_obj(const base_quat& qnb0 = qI, const Triple& vn0 = Triple::Zero(), const Triple& pos0 = Triple::Zero(), double t0 = 0.0);
        explicit ins_obj(hwa_set::set_base* _set);
        ~ins_obj() {};
        void set_posvel(const Triple& pos0, const Triple& vn0 = Triple::Zero());
        void set_posvel(const Triple& pos0, const Triple& vn0, const base_quat q);
        void set_tau(const Triple& tauG, const Triple& tauA);
        Triple align_coarse(const Triple& wmm, const Triple& vmm);
        Triple acc_align(const Triple& vmm);
        void Update(const std::map<int, std::vector<Triple>>& wm, const std::map<int, std::vector<Triple>>& vm, const std::map<int, ins_scheme>& scm);
        void Update(const std::vector<Triple>& wm, const std::vector<Triple>& vm, const ins_scheme& scm);
        void preintergration(std::vector<IMU_MSG> imu_msg, Triple& pos, Triple& vel, base_quat& q);
        void Update_ipn(const std::vector<Triple>& wm, const std::vector<Triple>& vm, const ins_scheme& scm, const Triple& gravity);
        void get_gcc(Triple gcc);
        void err_mat();
        void prt_header(std::ostringstream& os);
        void prt_sins(std::ostringstream& os);
        void prt_sins_ipn(std::ostringstream& os);
        void int_sec(bool _beg_end = true);
        void _debug_ins_info();

	public:
        mimu _mimu;
        SO3 jacobian_v_bg;
        SO3 jacobian_v_etag;
        int _num_of_imu_axiliary;
        double nts, t;                                /// ultimate interval,now
        imu _imu;                                    /// imu data
        base_earth eth;                                /// base_earth info
        Triple att, vn, pos, an;            /// attitude,velocity,position,accelarate
        Triple pos_store;
        Triple fb, fn, wib, web, wnb, vb;    /// special force,angular
        Triple eb, db, Kg, Ka;                /// gyro and acce bias and scale factors
        Triple _tauG, _tauA,
            _tauGScale, _tauAScale;                    /// Markov time
        SO3 Cnb, Cbn;                    /// DCM,Scale
        base_quat qnb, qnb_store;                                /// base_quat
        SO3 Faa, Fav, Fap, Fva, Fvv, Fvp, Fpv, Fpp;   /// Error Matrix
        SO3 Fng;                        /// transfer Matrix from d_enu to d_blh

        std::vector<Triple> _wm;
        std::vector<Triple> _vm;

        Eigen::Quaterniond orientation, orientation_store;
        Triple position;
        Triple velocity, velocity_store;
        Triple obs_wib, obs_fb;             // initial observation
        Triple gcc_b;
        Triple ve, ae, fe, pos_ecef, initial_pos;
        SO3 Ceb, Cbe;                    /// DCM
        base_quat qeb;
        base_quat qbe;
        SO3 Cvb, Cbv;                    ///< IMU installation Rotation 
        base_quat qvb;
        Matrix Xf = Matrix::Zero(5, 5);

        double output_freq;                            /// imu output frepency. wh
        bool output_intsec;                            /// imu output in integer second. wh
        std::map<double, base_quat> vec_qnb;               /// std::map qnb for interpolation

        double _tauOdo;
        double pure_ins_time = 0;

        Matrix _global_variance, _gv_sav;
        std::vector<IMU_MSG> imu_msg;
        Triple gyo_noise, acc_noise, ba_noise, bg_noise;
        Triple Gravity{ 0.0, 0.0, 9.8 };

        Matrix Ft, Pk, Hk, Rk, Qm, Pk_Sav, Phik, G;
        Vector Xk, Zk, Qt, Q, Rt, rts, Pmax, Pmin,
            Rmax, Rmin, Rb, Rbeta,
            FBTau, FBMax, FBXk, FBTotal;
        double hdop, vdop, pdop;
        int nq = 15, nr = 3; 

    protected:
        ODR_TYPE _order;
	};
}

#endif