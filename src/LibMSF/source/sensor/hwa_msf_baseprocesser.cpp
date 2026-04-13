#include "hwa_msf_baseprocesser.h"
#include "hwa_base_filter.h"
#include "hwa_set_all.h"
using namespace hwa_msf;
using namespace hwa_set;
using namespace hwa_base;

SENSOR_TYPE str2st(std::string s) {
    if (s == "UWB") return UWB;
    else if (s == "GNSS") return GNSS;
    else if (s == "VISION") return VISION;
    else if (s == "LIDAR") return LIDAR;
}

base_updater::base_updater(hwa_set::set_base* _gset, std::string s) {
    SENSOR_TYPE sensor = str2st(s);
    switch (sensor) {
    case UWB:
        filter = str2updater(dynamic_cast<set_uwb*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_uwb*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_uwb*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_uwb*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_uwb*>(_gset)->proc_noise();
        g0 = dynamic_cast<set_uwb*>(_gset)->G0();
        e0 = dynamic_cast<set_uwb*>(_gset)->E0();
        max_iter = dynamic_cast<set_uwb*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_uwb*>(_gset)->num_particles();
        barrier = dynamic_cast<set_uwb*>(_gset)->barrior();
        dof1 = dynamic_cast<set_uwb*>(_gset)->dof1();
        dof2 = dynamic_cast<set_uwb*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_uwb*>(_gset)->max_res_norm();
        break;
    case GNSS:
        filter = str2updater(dynamic_cast<set_flt*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_flt*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_flt*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_flt*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_flt*>(_gset)->proc_noise();
        g0 = dynamic_cast<set_flt*>(_gset)->G0();
        e0 = dynamic_cast<set_flt*>(_gset)->E0();
        max_iter = dynamic_cast<set_flt*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_flt*>(_gset)->num_particles();
        barrier = dynamic_cast<set_flt*>(_gset)->barrior();
        dof1 = dynamic_cast<set_flt*>(_gset)->dof1();
        dof2 = dynamic_cast<set_flt*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_flt*>(_gset)->max_res_norm();
        break;
    case VISION:
        filter = str2updater(dynamic_cast<set_vis*>(_gset)->filter());
        kappa_sig = dynamic_cast<set_vis*>(_gset)->kappa_sig();
        alpha_sig = dynamic_cast<set_vis*>(_gset)->alpha_sig();
        tau = dynamic_cast<set_vis*>(_gset)->Tau();
        proc_noise = dynamic_cast<set_vis*>(_gset)->proc_noise();
        g0 = dynamic_cast<set_vis*>(_gset)->G0();
        e0 = dynamic_cast<set_vis*>(_gset)->E0();
        max_iter = dynamic_cast<set_vis*>(_gset)->max_iter();
        _num_particles = dynamic_cast<set_vis*>(_gset)->num_particles();
        barrier = dynamic_cast<set_vis*>(_gset)->barrior();
        dof1 = dynamic_cast<set_vis*>(_gset)->dof1();
        dof2 = dynamic_cast<set_vis*>(_gset)->dof2();
        max_res_norm = dynamic_cast<set_vis*>(_gset)->max_res_norm();
        break;
    }
}

void baseprocesser::_feed_back() {
    int iatt = param_of_sins->getParam(_name, par_type::ATT_X, "");

    if (_Estimator == INEKF) {
        int K = 2;
        Matrix X = Matrix::Identity(3 + K, 3 + K);
        Eigen::Matrix3d R;
        Eigen::Matrix3d Jl;
        Eigen::Vector3d w = _sins->Xk.head(3);
        double theta = w.norm();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        if (theta < 1e-10) {
            R = I;
            Jl = I;
        }
        else {
            Eigen::Matrix3d A = askew(w);
            double theta2 = theta * theta;
            double stheta = sin(theta);
            double ctheta = cos(theta);
            double oneMinusCosTheta2 = (1 - ctheta) / (theta2);
            Eigen::Matrix3d A2 = A * A;
            R = I + (stheta / theta) * A + oneMinusCosTheta2 * A2;
            Jl = I + oneMinusCosTheta2 * A + ((theta - stheta) / (theta2 * theta)) * A2;
        }
        X.block<3, 3>(0, 0) = R;
        for (int i = 0; i < K; ++i) {
            X.block<3, 1>(0, 3 + i) = _sins->Xk.segment<3>(3 + 3 * i);
        }
        _sins->Xf = X.colPivHouseholderQr().solve(Matrix::Identity(5, 5)) * _sins->Xf;
        _sins->Ceb = _sins->Xf.block(0, 0, 3, 3);
        _sins->ve = _sins->Xf.block(0, 3, 3, 1);
        _sins->pos_ecef = _sins->Xf.block(0, 4, 3, 1) + _sins->initial_pos;
        _sins->eb = _sins->eb - _sins->Xk.block(9, 0, 3, 1);
        _sins->db = _sins->db - _sins->Xk.block(12, 0, 3, 1);
        _sins->qeb = base_att_trans::m2qua(_sins->Ceb);
    }

    if (_Estimator == NORMAL) {
        _sins->qeb = _sins->qeb - _sins->Xk.block(0, 0, 3, 1);
        _sins->ve = _sins->ve - _sins->Xk.block(3, 0, 3, 1);
        _sins->pos_ecef = _sins->pos_ecef - _sins->Xk.block(6, 0, 3, 1);
        _sins->eb = _sins->eb + _sins->Xk.block(9, 0, 3, 1);
        _sins->db = _sins->db + _sins->Xk.block(12, 0, 3, 1);
        _sins->Ceb = base_att_trans::q2mat(_sins->qeb);
    }

    int ilever = param_of_sins->getParam(_name, par_type::EXTRINSIC_CRD_X, "gnss");
    int iscale = param_of_sins->getParam(_name, par_type::gyro_scale_X, "");
    int iodo = param_of_sins->getParam(_name, par_type::ODO_k, "");
    int irot = param_of_sins->getParam(_name, par_type::IMU_INST_ATT_X, "");

    if (_shm->_imu_scale)
    {
        _sins->Kg = _sins->Kg - _sins->Xk.block(iscale, 0, 3, 1);
        _sins->Ka = _sins->Ka - _sins->Xk.block(iscale + 3, 0, 3, 1);
    }
    if (_shm->_imu_inst_rot)
    {
        _sins->qvb = _sins->qvb - _sins->Xk.block(irot, 0, 3, 1);
        _sins->Cvb = base_att_trans::q2mat(_sins->qvb);
        _sins->Cbv = _sins->Cvb.transpose();
    }
    _sins->pos = Cart2Geod(_sins->pos_ecef, false);
    _sins->eth.Update(_sins->pos, _sins->vn);
    _sins->vn = _sins->eth.Cne * _sins->ve;
    _sins->qnb = base_att_trans::m2qua(_sins->eth.Cne) * _sins->qeb;
    _sins->att = base_att_trans::q2att(_sins->qnb);
    _sins->Cnb = base_att_trans::q2mat(_sins->qnb);
    _sins->orientation = _sins->Ceb.transpose();
    _sins->velocity = _sins->ve;
    _sins->position = Geod2Cart(_sins->pos, false);
}