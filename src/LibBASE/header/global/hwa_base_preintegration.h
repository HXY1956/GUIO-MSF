#ifndef hwa_base_integration_h
#define hwa_base_integration_h

#include "hwa_base_eigendef.h"
#include "hwa_base_posetrans.h"

namespace hwa_base
{
    enum StateOrder
    {
        O_P = 0,
        O_R = 3,
        O_V = 6,
        O_BA = 9,
        O_BG = 12
    };

    enum NoiseOrder
    {
        O_AN = 0,
        O_GN = 3,
        O_AW = 6,
        O_GW = 9
    };

    class IntegrationBase
    {
    public:
        double dt = 0;
        Eigen::Vector3d acc_0, gyr_0;
        Eigen::Vector3d acc_1, gyr_1;

        Eigen::Vector3d linearized_acc, linearized_gyr;
        Eigen::Vector3d linearized_ba, linearized_bg;

        Eigen::Matrix<double, 15, 15> jacobian, covariance;
        Eigen::Matrix<double, 15, 15> step_jacobian;
        Eigen::Matrix<double, 15, 18> step_V;
        Eigen::Matrix<double, 18, 18> noise;

        double sum_dt = 0;
        Eigen::Vector3d delta_p;
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_v;

        std::vector<double> dt_buf;
        std::vector<Eigen::Vector3d> acc_buf;
        std::vector<Eigen::Vector3d> gyr_buf;

        double ACC_N = 0, ACC_W = 0;
        double GYR_N = 0, GYR_W = 0;
        Eigen::Vector3d gravity;

		int IMU_count = 0;

        IntegrationBase(const Triple& _acc_0, const Triple& _gyr_0, const int& IMU_count)
            : acc_0{ _acc_0 }, gyr_0{ _gyr_0 }, linearized_acc{ _acc_0 }, linearized_gyr{ _gyr_0 }, IMU_count{ IMU_count },
            linearized_ba{ Triple::Zero() }, linearized_bg{ Triple::Zero() },
            jacobian{ Eigen::Matrix<double, 15, 15>::Identity() }, covariance{ Eigen::Matrix<double, 15, 15>::Zero() },
            sum_dt{ 0.0 }, delta_p{ Triple::Zero() }, delta_q{ Eigen::Quaterniond::Identity() }, delta_v{ Triple::Zero() } {


        }

        IntegrationBase(const Triple& _acc_0, const Triple& _gyr_0,
            const Triple& _linearized_ba, const Triple& _linearized_bg)
            : acc_0{ _acc_0 }, gyr_0{ _gyr_0 }, linearized_acc{ _acc_0 }, linearized_gyr{ _gyr_0 },
            linearized_ba{ _linearized_ba }, linearized_bg{ _linearized_bg },
            jacobian{ Eigen::Matrix<double, 15, 15>::Identity() }, covariance{ Eigen::Matrix<double, 15, 15>::Zero() },
            sum_dt{ 0.0 }, delta_p{ Triple::Zero() }, delta_q{ Eigen::Quaterniond::Identity() }, delta_v{ Triple::Zero() }

        {

        }

        Triple get_gravity()
        {
            return gravity;
		}

        void init_ins(double _ACC_N, double _ACC_W, double _GYR_N, double _GYR_W, Eigen::Vector3d _gravity);

        void reset_integration_base();

        void processIMU(double dt, const Triple& linear_acceleration, const Triple& angular_velocity, const Triple Bgs, const Triple Bas);

        void push_back(
            double dt,
            const Eigen::Vector3d& acc,
            const Eigen::Vector3d& gyr);


        void repropagate(const Triple& _linearized_ba, const Triple& _linearized_bg);


        Eigen::Matrix<double, 15, 1> evaluate(
            const Eigen::Vector3d& Pi,
            const Eigen::Quaterniond& Qi,
            const Eigen::Vector3d& Vi,
            const Eigen::Vector3d& Bai,
            const Eigen::Vector3d& Bgi,
            const Eigen::Vector3d& Pj,
            const Eigen::Quaterniond& Qj,
            const Eigen::Vector3d& Vj,
            const Eigen::Vector3d& Baj,
            const Eigen::Vector3d& Bgj);


        void midPointIntegration(double _dt,
            const Triple& _acc_0, const Triple& _gyr_0,
            const Triple& _acc_1, const Triple& _gyr_1,
            const Triple& delta_p, const Eigen::Quaterniond& delta_q, const Triple& delta_v,
            const Triple& linearized_ba, const Triple& linearized_bg,
            Triple& result_delta_p, Eigen::Quaterniond& result_delta_q, Triple& result_delta_v,
            Triple& result_linearized_ba, Triple& result_linearized_bg, bool update_jacobian);


        void propagate(double _dt, const Triple& _acc_1, const Triple& _gyr_1);

        void print();

    };
}

# endif
