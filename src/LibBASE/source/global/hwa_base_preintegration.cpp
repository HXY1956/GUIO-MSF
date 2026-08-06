#include "hwa_base_preintegration.h"
#include <iostream>
#include <iomanip>

using namespace hwa_base;

void IntegrationBase::init_ins(double _ACC_N, double _ACC_W, double _GYR_N, double _GYR_W, Eigen::Vector3d _gravity)
{
    gravity.setZero();
    ACC_N = _ACC_N;
    ACC_W = _ACC_W;
    GYR_N = _GYR_N;
    GYR_W = _GYR_W;
    gravity = _gravity;

    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
}

void IntegrationBase::reset_integration_base()
{
    acc_0 = Eigen::Vector3d::Identity();
    gyr_0 = Eigen::Vector3d::Identity();
    acc_1 = Eigen::Vector3d::Identity();
    gyr_1 = Eigen::Vector3d::Identity();
    linearized_acc = Eigen::Vector3d::Identity();
    linearized_gyr = Eigen::Vector3d::Identity();
    linearized_ba = Eigen::Vector3d::Identity();
    linearized_bg = Eigen::Vector3d::Identity();
    step_jacobian = Eigen::Matrix<double, 15, 15>::Identity();
    step_V = Eigen::Matrix<double, 15, 18>::Identity();
    jacobian = Eigen::Matrix<double, 15, 15>::Identity();
    covariance = Eigen::Matrix<double, 15, 15>::Zero();
    sum_dt = 0.0;
    delta_p = Eigen::Vector3d::Zero();
    delta_q = Eigen::Quaterniond::Identity();
    delta_v = Eigen::Vector3d::Zero();
    dt_buf.clear();
    acc_buf.clear();
    gyr_buf.clear();
}

void IntegrationBase::push_back(double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr)
{
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void IntegrationBase::processIMU(double dt, const Triple& linear_acceleration, const Triple& angular_velocity, const Triple Bgs, const Triple Bas)
{
    if (IMU_count == 0) {
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        linearized_acc = acc_0;
        linearized_gyr = gyr_0;
    }
    dt_buf.push_back(dt);
    acc_buf.push_back(linear_acceleration);
    gyr_buf.push_back(angular_velocity);
    propagate(dt, linear_acceleration, angular_velocity);
    IMU_count++;
}

void IntegrationBase::midPointIntegration(double _dt,
    const Triple& _acc_0, const Triple& _gyr_0,
    const Triple& _acc_1, const Triple& _gyr_1,
    const Triple& delta_p, const Eigen::Quaterniond& delta_q, const Triple& delta_v,
    const Triple& linearized_ba, const Triple& linearized_bg,
    Triple& result_delta_p, Eigen::Quaterniond& result_delta_q, Triple& result_delta_v,
    Triple& result_linearized_ba, Triple& result_linearized_bg, bool update_jacobian)
{
    Triple un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Triple un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Triple un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Triple un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian)
    {
        Triple w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Triple a_0_x = _acc_0 - linearized_ba;
        Triple a_1_x = _acc_1 - linearized_ba;
        SO3 R_w_x, R_a_0_x, R_a_1_x;

        R_w_x << 0, -w_x(2), w_x(1),
            w_x(2), 0, -w_x(0),
            -w_x(1), w_x(0), 0;
        R_a_0_x << 0, -a_0_x(2), a_0_x(1),
            a_0_x(2), 0, -a_0_x(0),
            -a_0_x(1), a_0_x(0), 0;
        R_a_1_x << 0, -a_1_x(2), a_1_x(1),
            a_1_x(2), 0, -a_1_x(0),
            -a_1_x(1), a_1_x(0), 0;

        Matrix F = Matrix::Zero(15, 15);
        F.block<3, 3>(0, 0) = SO3::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
            -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (SO3::Identity() - R_w_x * _dt) * _dt * _dt;
        F.block<3, 3>(0, 6) = Matrix::Identity(3, 3) * _dt;
        F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
        F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
        F.block<3, 3>(3, 3) = SO3::Identity() - R_w_x * _dt;
        F.block<3, 3>(3, 12) = -1.0 * Matrix::Identity(3, 3) * _dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
            -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (SO3::Identity() - R_w_x * _dt) * _dt;
        F.block<3, 3>(6, 6) = SO3::Identity();
        F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
        F.block<3, 3>(9, 9) = SO3::Identity();
        F.block<3, 3>(12, 12) = SO3::Identity();

        Matrix V = Matrix::Zero(15, 18);
        V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
        V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) = 0.5 * Matrix::Identity(3, 3) * _dt;
        V.block<3, 3>(3, 9) = 0.5 * Matrix::Identity(3, 3) * _dt;
        V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
        V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = Matrix::Identity(3, 3) * _dt;
        V.block<3, 3>(12, 15) = Matrix::Identity(3, 3) * _dt;

        //step_jacobian = F;
        //step_V = V;
        jacobian = F * jacobian;
        covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }

}

void IntegrationBase::propagate(double _dt, const Triple& _acc_1, const Triple& _gyr_1)
{
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Triple result_delta_p;
    Eigen::Quaterniond result_delta_q;
    Triple result_delta_v;
    Triple result_linearized_ba;
    Triple result_linearized_bg;

    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
        linearized_ba, linearized_bg,
        result_delta_p, result_delta_q, result_delta_v,
        result_linearized_ba, result_linearized_bg, 1);

    //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
    //                    linearized_ba, linearized_bg);
    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;

}

void IntegrationBase::repropagate(const Triple& _linearized_ba, const Triple& _linearized_bg)
{
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
}

/**
 * @brief Evaluate IMU pre-integration residuals between two states
 *
 * Computes residuals for position, velocity, attitude and bias states using pre-integrated measurements with bias correction.
 * Handles gravity compensation and quaternion error formulation for factor graph.
 */
Eigen::Matrix<double, 15, 1> IntegrationBase::evaluate(const Eigen::Vector3d& Pi, const Eigen::Quaterniond& Qi, const Eigen::Vector3d& Vi, const Eigen::Vector3d& Bai, const Eigen::Vector3d& Bgi,
    const Eigen::Vector3d& Pj, const Eigen::Quaterniond& Qj, const Eigen::Vector3d& Vj, const Eigen::Vector3d& Baj, const Eigen::Vector3d& Bgj)
{
    Eigen::Matrix<double, 15, 1> residuals;
    residuals.setZero();

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;


    Eigen::Quaterniond corrected_delta_q = delta_q * base_att_trans::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    //gravity.setZero();
    residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * gravity * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (gravity * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
}