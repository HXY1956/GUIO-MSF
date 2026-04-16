#include "hwa_msf_insprocesser.h"

namespace hwa_msf {
    insprocesser::insprocesser(const baseprocesser& B, base_data* data): baseprocesser(B),
        insdata(dynamic_cast<ins_data*>(data))
    {
        nq = 15;
        FuseType = hwa_ins::str2ins_fuse_mode(dynamic_cast<set_ins*>(_gset.get())->fuse_type());
        R2P = dynamic_cast<set_ins*>(_gset.get())->R2P();
        _num_of_imu_axiliary = dynamic_cast<set_ins*>(_gset.get())->num_of_ins_auxiliary();
        inflation = dynamic_cast<set_ign*>(_gset.get())->inflation();
        initial_pos = dynamic_cast<set_ins*>(_gset.get())->pos();
        initial_vel = dynamic_cast<set_ins*>(_gset.get())->vel();
        initial_att = dynamic_cast<set_ins*>(_gset.get())->att();
        _estimate_imui_extrinsic = dynamic_cast<set_ins*>(_gset.get())->estimate_imui_extrinsic();
        _estimate_imui_t = dynamic_cast<set_ins*>(_gset.get())->estimate_imui_t();
        _initial_extrinsic_rotation_cov = dynamic_cast<set_ins*>(_gset.get())->initial_extrinsic_rotation_cov();
        _initial_extrinsic_translation_cov = dynamic_cast<set_ins*>(_gset.get())->initial_extrinsic_translation_cov();
        _initial_t_cov = dynamic_cast<set_ins*>(_gset.get())->initial_t_cov();
        _enf_R_std = dynamic_cast<set_ins*>(_gset.get())->enf_R_std();
        _enf_p_std = dynamic_cast<set_ins*>(_gset.get())->enf_p_std();
        int add_dim = 15;
        if (_estimate_imui_extrinsic)
            add_dim = add_dim + 6;
        if (_estimate_imui_t)
            add_dim = add_dim + 1;
        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
            nq += add_dim * _num_of_imu_axiliary;

        for (int id = 0; id < _num_of_imu_axiliary + 1; id++)
        {
            _R_imui_imu0[id] = dynamic_cast<set_ins*>(_gset.get())->R_imui_imu0(id);
            _p_imui_imu0[id] = dynamic_cast<set_ins*>(_gset.get())->p_imui_imu0(id);
            _t_imui_imu0[id] = dynamic_cast<set_ins*>(_gset.get())->t_imui_imu0(id);
            sins_mimu[id] = std::make_unique<hwa_ins::ins_obj>(_gset.get());
            _shm_mimu[id] = ins_scheme(_gset.get());
        }
        _sins->_mimu._num_of_imu_axiliary = _num_of_imu_axiliary;
        _sins->_mimu._p_imui_imu0 = _p_imui_imu0;
        _sins->_mimu._R_imui_imu0 = _R_imui_imu0;
        _sins->Pk.resize(nq, nq);
        _sins->Xk.resize(nq);

        Ft = Matrix::Zero(nq, nq);
        G = Matrix::Zero(15, 12);
        Vector r = Vector::Zero(5);
        r << 0.1, 0.1, 1 * glv.deg, 0.1, 0.1 * glv.dps;
        _avar.init(5, dynamic_cast<set_ins*>(_gset.get())->ts(), r, Vector::Ones(5));

        set_ins_out();
        _publisher.Initialize();

        beg.from_secs(dynamic_cast<set_ins*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_ins*>(_gset.get())->end());
        TimeStamp = beg;
    };

    insprocesser::insprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog, base_data* data, base_time _beg, base_time _end) : baseprocesser(gset, spdlog, site, _beg, _end),
        insdata(dynamic_cast<ins_data*>(data))
    {
        nq = 15;
        FuseType = hwa_ins::str2ins_fuse_mode(dynamic_cast<set_ins*>(_gset.get())->fuse_type());
        R2P = dynamic_cast<set_ins*>(_gset.get())->R2P();
        _num_of_imu_axiliary = dynamic_cast<set_ins*>(_gset.get())->num_of_ins_auxiliary();
        inflation = dynamic_cast<set_ign*>(_gset.get())->inflation();
        initial_pos = dynamic_cast<set_ins*>(_gset.get())->pos();
        initial_vel = dynamic_cast<set_ins*>(_gset.get())->vel();
        initial_att = dynamic_cast<set_ins*>(_gset.get())->att();
        _estimate_imui_extrinsic = dynamic_cast<set_ins*>(_gset.get())->estimate_imui_extrinsic();
        _estimate_imui_t = dynamic_cast<set_ins*>(_gset.get())->estimate_imui_t();
        _initial_extrinsic_rotation_cov = dynamic_cast<set_ins*>(_gset.get())->initial_extrinsic_rotation_cov();
        _initial_extrinsic_translation_cov = dynamic_cast<set_ins*>(_gset.get())->initial_extrinsic_translation_cov();
        _initial_t_cov = dynamic_cast<set_ins*>(_gset.get())->initial_t_cov();
        _enf_R_std = dynamic_cast<set_ins*>(_gset.get())->enf_R_std();
        _enf_p_std = dynamic_cast<set_ins*>(_gset.get())->enf_p_std();
        int add_dim = 15;
        if (_estimate_imui_extrinsic)
            add_dim = add_dim + 6;
        if (_estimate_imui_t)
            add_dim = add_dim + 1;
        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
            nq += add_dim * _num_of_imu_axiliary;

        for (int id = 0; id < _num_of_imu_axiliary + 1; id++)
        {
            _R_imui_imu0[id] = dynamic_cast<set_ins*>(_gset.get())->R_imui_imu0(id);
            _p_imui_imu0[id] = dynamic_cast<set_ins*>(_gset.get())->p_imui_imu0(id);
            _t_imui_imu0[id] = dynamic_cast<set_ins*>(_gset.get())->t_imui_imu0(id);
            sins_mimu[id] = std::make_unique<hwa_ins::ins_obj>(_gset.get());
        }
        _sins->_mimu._num_of_imu_axiliary = _num_of_imu_axiliary;
        _sins->_mimu._p_imui_imu0 = _p_imui_imu0;
        _sins->_mimu._R_imui_imu0 = _R_imui_imu0;
        _sins->Pk.resize(nq, nq);
        _sins->Xk.resize(nq);

        Ft = Matrix::Zero(nq, nq);
        G = Matrix::Zero(15, 12);
        Vector r = Vector::Zero(5);
        r << 0.1, 0.1, 1 * glv.deg, 0.1, 0.1 * glv.dps;
        _avar.init(5, dynamic_cast<set_ins*>(_gset.get())->ts(), r, Vector::Ones(5));

        set_ins_out();
        _publisher.Initialize();

        beg.from_secs(dynamic_cast<set_ins*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_ins*>(_gset.get())->end());
        TimeStamp = beg;
    }

    bool insprocesser::_init()
    {
        init_par();
        Vector v(nq);
        int d = 0, itg = 15;
        Triple angle_tmp, v_tmp, pos_tmp, eb_tmp, db_tmp, gscale_tmp, ascale_tmp, inst_att_tmp, lever_gnss_tmp;
        double dk_tmp;
        angle_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_misalignment_std();
        v_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_vel_std();
        pos_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_pos_std();
        eb_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_gyro_std();
        db_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_acce_std();
        v << angle_tmp * hwa_base::glv.deg, v_tmp, pos_tmp, eb_tmp* hwa_base::glv.dph, db_tmp* hwa_base::glv.mg;
        v = v.array().abs2();
        if (_num_of_imu_axiliary > 0 && FuseType == STACK)
        {
            int imui = param_of_sins->getParam(_name, hwa_base::par_type::ATT_X, "imu1");
            if (_estimate_imui_t)
            {
                d = 1;
                v(imui - d) = sqrt(_initial_t_cov);
                itg = itg + 1;
            }
            if (_estimate_imui_extrinsic)
            {
                v.block(imui - 6 - d, 0, 3, 1) = _initial_extrinsic_rotation_cov * hwa_base::glv.deg * hwa_base::glv.deg;
                v.block(imui - 3 - d, 0, 3, 1) = _initial_extrinsic_translation_cov;
                itg = itg + 6;
                for (int it = 0; it < 6; it++)
                    v(imui - 6 - d + it) = sqrt(v(imui - 6 - d + it));
            }
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                angle_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_misalignment_std();
                v.block(imui, 0, 3, 1) = angle_tmp * hwa_base::glv.deg;
                v_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_vel_std();
                v.block(imui + 3, 0, 3, 1) = v_tmp;
                pos_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_pos_std();
                v.block(imui + 6, 0, 3, 1) = pos_tmp;
                eb_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_gyro_std();
                v.block(imui + 9, 0, 3, 1) = eb_tmp * hwa_base::glv.dph;
                db_tmp = dynamic_cast<set_ins*>(_gset.get())->initial_acce_std();
                v.block(imui + 12, 0, 3, 1) = db_tmp * hwa_base::glv.mg;
                d = 0;
                if (_estimate_imui_extrinsic)
                {
                    v.block(imui + 15, 0, 3, 1) = _initial_extrinsic_rotation_cov * hwa_base::glv.deg * hwa_base::glv.deg;
                    v.block(imui + 15 + 3, 0, 3, 1) = _initial_extrinsic_translation_cov;
                    d += 6;
                    for (int it = 0; it < 6; it++)
                        v(imui + 15 + it) = sqrt(v(imui + 15 + it));
                }
                if (_estimate_imui_t)
                {
                    v(imui + 15 + d) = sqrt(_initial_t_cov);
                    d += 1;
                }
                imui = imui + itg;
            }
        }
        _sins->Pk = v.array().matrix().asDiagonal();
        _sins->Pk.block<3, 3>(0, 0) = _sins->eth.Cen * _sins->Pk.block<3, 3>(0, 0) * _sins->eth.Cen.transpose();

        // Read Proc Noise from XML.
        angle_tmp = dynamic_cast<set_ins*>(_gset.get())->misalignment_psd();
        v_tmp = dynamic_cast<set_ins*>(_gset.get())->vel_psd();
        pos_tmp = dynamic_cast<set_ins*>(_gset.get())->pos_psd();
        eb_tmp = dynamic_cast<set_ins*>(_gset.get())->gyro_psd();
        db_tmp = dynamic_cast<set_ins*>(_gset.get())->acce_psd();

        _sins->gyo_noise = angle_tmp * hwa_base::glv.dpsh;
        _sins->acc_noise = v_tmp * hwa_base::glv.mgpsHz;
        _sins->ba_noise = db_tmp * hwa_base::glv.mgpsh;
        _sins->bg_noise = eb_tmp * hwa_base::glv.dphpsh;

        if (_Estimator == NORMAL) {
            _sins->Qt << angle_tmp * hwa_base::glv.dpsh, v_tmp* hwa_base::glv.mgpsHz, pos_tmp* hwa_base::glv.mpsh, eb_tmp* hwa_base::glv.dphpsh, db_tmp* hwa_base::glv.mgpsh;
        }
        else if (_Estimator == INEKF) {
            _sins->Qt = Vector(nq - 3);
            _sins->Qt << angle_tmp * hwa_base::glv.dpsh, v_tmp* hwa_base::glv.mgpsHz, eb_tmp* hwa_base::glv.dphpsh, db_tmp* hwa_base::glv.mgpsh;
        }
        if (_num_of_imu_axiliary > 0 && FuseType == STACK)
        {
            int imui = param_of_sins->getParam(_name, hwa_base::par_type::ATT_X, "imu1");
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                angle_tmp = dynamic_cast<set_ins*>(_gset.get())->misalignment_psd();
                _sins->Qt.block(imui, 0, 3, 1) = angle_tmp * hwa_base::glv.dpsh;
                v_tmp = dynamic_cast<set_ins*>(_gset.get())->vel_psd();
                _sins->Qt.block(imui + 3, 0, 3, 1) = v_tmp * hwa_base::glv.mgpsHz;
                pos_tmp = dynamic_cast<set_ins*>(_gset.get())->pos_psd();
                _sins->Qt.block(imui + 6, 0, 3, 1) = pos_tmp * hwa_base::glv.mpsh;
                eb_tmp = dynamic_cast<set_ins*>(_gset.get())->gyro_psd();
                _sins->Qt.block(imui + 9, 0, 3, 1) = eb_tmp * hwa_base::glv.dphpsh;
                db_tmp = dynamic_cast<set_ins*>(_gset.get())->acce_psd();
                _sins->Qt.block(imui + 12, 0, 3, 1) = db_tmp * hwa_base::glv.mgpsh;
                imui = imui + itg;
            }
        }
        _sins->Qt = _sins->Qt.array().abs2();
        _sins->Qm = _sins->Qt.array().matrix().asDiagonal();

        _sins->FBTau << 1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
        _sins->FBMax << hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF,
            10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph, 500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg;

        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
        {
            int imui = param_of_sins->getParam(_name, hwa_base::par_type::ATT_X, "imu1");
            _sins->FBTau.block(imui - d, 0, d, 1) = 1.0 * Vector::Ones(d);
            _sins->FBMax.block(imui - d, 0, d, 1) = hwa_base::glv.INF * Vector::Ones(d);
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                _sins->FBTau.block(imui, 0, 3, 1) = Triple(1.0, 1.0, 10.0);
                _sins->FBTau.block(imui + 3, 0, 3, 1) = Triple(1.0, 1.0, 1.0);
                _sins->FBTau.block(imui + 6, 0, 3, 1) = Triple(1.0, 1.0, 1.0);
                _sins->FBTau.block(imui + 9, 0, 3, 1) = Triple(10.0, 10.0, 10.0);
                _sins->FBTau.block(imui + 12, 0, 3, 1) = Triple(10.0, 10.0, 10.0);
                _sins->FBTau.block(imui + 15, 0, d, 1) = 1.0 * Vector::Ones(d);
                _sins->FBMax.block(imui, 0, 3, 1) = Triple(hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF);
                _sins->FBMax.block(imui + 3, 0, 3, 1) = Triple(hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF);
                _sins->FBMax.block(imui + 6, 0, 3, 1) = Triple(hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF);
                _sins->FBMax.block(imui + 9, 0, 3, 1) = Triple(3000.0 * hwa_base::glv.dph, 3000.0 * hwa_base::glv.dph, 3000.0 * hwa_base::glv.dph);
                _sins->FBMax.block(imui + 9, 0, 3, 1) = Triple(10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph);
                _sins->FBMax.block(imui + 12, 0, 3, 1) = Triple(500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg);
                _sins->FBMax.block(imui + 15, 0, d, 1) = hwa_base::glv.INF * Vector::Ones(d);
                imui = imui + itg;
            }
        }

        // Multi-Imu States Initialization
        _sins->Xf.block(0, 0, 3, 3) = _sins->Ceb;
        _sins->Xf.block(0, 3, 3, 1) = _sins->ve;
        _sins->Xf(3, 3) = 1;
        _sins->Xf(4, 4) = 1;
        int icrdx = param_of_sins->operator[](param_of_sins->getParam(_name, hwa_base::par_type::CRD_X, "")).index;
        for (int imui = 0; imui <= _num_of_imu_axiliary; imui++)
        {
            sins_mimu[imui]->qnb = _sins->qnb * hwa_base::base_att_trans::m2qua(_R_imui_imu0[imui]);
            sins_mimu[imui]->Cnb = hwa_base::base_att_trans::q2mat(sins_mimu[imui]->qnb);
            sins_mimu[imui]->pos_ecef = _sins->pos_ecef - sins_mimu[imui]->eth.Cen * sins_mimu[imui]->Cnb * _p_imui_imu0[imui];
            sins_mimu[imui]->pos = hwa_base::Cart2Geod(sins_mimu[imui]->pos_ecef, false);
            sins_mimu[imui]->eth.Update(sins_mimu[imui]->pos, Triple::Zero());
            sins_mimu[imui]->Xf.block(0, 0, 3, 3) = sins_mimu[imui]->Ceb;
            sins_mimu[imui]->Xf.block(0, 3, 3, 1) = sins_mimu[imui]->ve;
            sins_mimu[imui]->Xf(3, 3) = 1;
            sins_mimu[imui]->Xf(4, 4) = 1;

            if (imui > 0 && FuseType == hwa_ins::STACK)
            {
                int icrdx_i = param_of_sins->operator[](param_of_sins->getParam(_name, hwa_base::par_type::CRD_X, "imu" + hwa_base::base_type_conv::int2str(imui))).index;
                SO3 Ceb_i = sins_mimu[imui]->eth.Cen * sins_mimu[imui]->Cnb;
                SO3 p_extr_cov = SO3::Zero();
                SO3 R_extr_cov = SO3::Zero();
                if (_estimate_imui_extrinsic)
                {
                    p_extr_cov(0, 0) = _initial_extrinsic_translation_cov(0);
                    p_extr_cov(1, 1) = _initial_extrinsic_translation_cov(1);
                    p_extr_cov(2, 2) = _initial_extrinsic_translation_cov(2);
                    R_extr_cov(0, 0) = _initial_extrinsic_rotation_cov(0) * hwa_base::glv.deg * hwa_base::glv.deg;
                    R_extr_cov(1, 1) = _initial_extrinsic_rotation_cov(1) * hwa_base::glv.deg * hwa_base::glv.deg;
                    R_extr_cov(2, 2) = _initial_extrinsic_rotation_cov(2) * hwa_base::glv.deg * hwa_base::glv.deg;
                }
                _sins->Pk.block(icrdx_i - 1, icrdx_i - 1, 3, 3) = _sins->Pk.block(icrdx - 1, icrdx - 1, 3, 3) + Ceb_i * p_extr_cov * Ceb_i.transpose();
                _sins->Pk.block(icrdx_i - 7, icrdx_i - 7, 3, 3) = _sins->Pk.block(icrdx - 7, icrdx - 7, 3, 3) + Ceb_i * R_extr_cov * Ceb_i.transpose();
            }
        }
        _sins->Pk_Sav = _sins->Pk;

        if (insdata)
        {
            double start = insdata->beg_obs();
            double _end = insdata->end_obs();
            double start_set = dynamic_cast<set_ins*>(_gset.get())->start();
            double end_set = dynamic_cast<set_ins*>(_gset.get())->end();
            start = (start < start_set) ? start_set : start;
            _end = (_end < end_set) ? _end : end_set;

            beg = base_time(beg.gwk(), start);
            end = base_time(beg.gwk(), _end);
            TimeStamp = beg;
        }
        else
        {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, std::string("t_gipn_client:  ") + "IMU observation is not existing!!! ");
            return -1;
        }

        return true;
    }

    bool insprocesser::align_coarse()
    {
        _sins->t = _shm->t;

        for (int j = 0; j < _shm->nSamples; j++) {
            wmm = wmm + _sins->_wm[j] - _sins->eb * _shm->ts;
            vmm = vmm + _sins->_vm[j] - _sins->db * _shm->ts;
            _align_count++;
        }
        if (abs(_align_count - _shm->align_time * _shm->freq) < 1e-5)
        {
            std::cerr << "static alignment" << std::endl;
            wmm = wmm / _align_count; vmm = vmm / _align_count;
            _sins->qnb = base_att_trans::a2qua(_sins->align_coarse(wmm, vmm));
            return true;
        }
        return false;
    }

    bool insprocesser::align_pva(const Triple& pos)
    {
        if (_first_align)
        {
            _first_pos = _pre_pos = pos;
            _first_align = false;
            return false;
        }
        Triple endpos = pos;
        Triple baseline = XYZ2ENU(endpos, _first_pos);
        double dist = SQRT(SQR(baseline(0)) + SQR(baseline(1)));

        // double yaw;
        double dyaw = 10 * glv.deg;
        double pos_dist = dynamic_cast<set_ign*>(_gset.get())->pos_dist();
        if (dist > pos_dist)
        {
            double yaw = acos(fabs(baseline(1)) / dist);
            if (baseline(1) > 0 && baseline(0) > 0)yaw = -yaw;
            if (baseline(1) > 0 && baseline(0) < 0)yaw = yaw;
            if (baseline(1) < 0 && baseline(0) > 0)yaw = -(glv.PI - yaw);
            if (baseline(1) < 0 && baseline(0) < 0)yaw = glv.PI - yaw;
            _sins->qnb = base_att_trans::a2qua(Triple(0, 0, yaw));
            if (_yaw0 == 0.0)
            {
                _yaw0 = yaw;
                _first_pos = endpos;
                return false;
            }
            if (fabs(yaw - _yaw0) < dyaw) return true;
            _yaw0 = 0.0;
            _first_align = true;
        }
        return false;
    }

    bool insprocesser::align_vva(const Triple& vel)
    {
        double vel_norm = dynamic_cast<set_ign*>(_gset.get())->vel_norm();
        if (vel.norm() > vel_norm)
        {
            double yaw = atan2(fabs(vel(0)), fabs(vel(1)));
            if (vel(1) > 0 && vel(0) > 0)yaw = -yaw;
            if (vel(1) > 0 && vel(0) < 0)yaw = yaw;
            if (vel(1) < 0 && vel(0) > 0)yaw = -(glv.PI - yaw);
            if (vel(1) < 0 && vel(0) < 0)yaw = glv.PI - yaw;
            _sins->qnb = base_att_trans::a2qua(Triple(0, 0, yaw));
            return true;
        }
        std::cerr << "Velocity vector alignment failed. Please speed up! " << std::endl;
        return false;
    }

    bool insprocesser::align_static() {
        int n = vm_align.size();
        double ts = 1.0 / imu_frequency;
        if (n < 500) return false;
        Triple vm = Triple::Zero();
        Triple wm = Triple::Zero();
        for (int i = 0; i < n; i++) {
            vm += vm_align[i];
            wm += wm_align[i];
        }
        vm /= (n * ts); wm /= (n * ts);

        double gravity_norm = vm.norm();
        Triple gravity = Triple(0.0, 0.0, gravity_norm);

        Eigen::Quaterniond q_i_n = Eigen::Quaterniond::FromTwoVectors(vm, gravity); //vm -> gravity
        _sins->qnb = hwa_base::Qbase2eigen(q_i_n);
        _sins->Cnb = hwa_base::base_att_trans::q2mat(_sins->qnb);
        _sins->Cbn = _sins->Cnb.transpose();
        _sins->qeb = hwa_base::base_att_trans::m2qua(_sins->eth.Cen) * _sins->qnb;
        _sins->eb = wm;
        _sins->db = vm + _sins->Cbn * _sins->eth.gcc;

        std::cerr << "Static Align Successfully!" << std::endl;
        std::cerr << "Initial Cnb: " << std::endl << std::setiosflags(std::ios::fixed) << std::setprecision(3) << _sins->Cnb << std::endl;

        for (int imui = 0; imui <= _num_of_imu_axiliary; imui++)
        {
            sins_mimu[imui]->qnb = _sins->qnb * hwa_base::base_att_trans::m2qua(_R_imui_imu0[imui]);
            sins_mimu[imui]->Cnb = hwa_base::base_att_trans::q2mat(sins_mimu[imui]->qnb);
            sins_mimu[imui]->pos_ecef = _sins->pos_ecef - sins_mimu[imui]->eth.Cen * sins_mimu[imui]->Cnb * _p_imui_imu0[imui];
            sins_mimu[imui]->pos = hwa_base::Cart2Geod(sins_mimu[imui]->pos_ecef, false);
            sins_mimu[imui]->Xf.block(0, 0, 3, 3) = sins_mimu[imui]->Ceb;
            sins_mimu[imui]->Xf.block(0, 3, 3, 1) = sins_mimu[imui]->ve;
            sins_mimu[imui]->Xf(3, 3) = 1;
            sins_mimu[imui]->Xf(4, 4) = 1;
        }
        return true;
    }

    void insprocesser::merge_init(const Triple& pos, const Triple& lever, const Matrix& var, SENSOR_TYPE sensor)
    {
        _sins->eth.Update(Cart2Geod(Eigen::Vector3d(pos[0], pos[1], pos[2]), false), Eigen::Vector3d::Zero());
        _sins->Cnb = base_att_trans::q2mat(_sins->qnb);
        _sins->Ceb = _sins->eth.Cen * _sins->Cnb;
        _sins->pos_ecef = Eigen::Vector3d(pos[0], pos[1],pos[2]) - _sins->eth.Cen * _sins->Cnb * lever;
        _sins->pos = Cart2Geod(_sins->pos_ecef, false);
        _sins->initial_pos = _sins->pos_ecef;
        _sins->Xf.block(0, 0, 3, 3) = _sins->Ceb;
        _sins->Xf.block(0, 3, 3, 1) = _sins->ve;
        _sins->Xf(3, 3) = 1;
        _sins->Xf(4, 4) = 1;
        int icrdx = param_of_sins->operator[](param_of_sins->getParam(_name, par_type::CRD_X, "")).index;
        //if (sensor == GNSS) _sins->Pk = var.eval();
        if (sensor == GNSS) {
           SO3 tmp = var.block(icrdx, icrdx, 3, 3);
           Matrix tmp_pk = _sins->Pk;
           tmp_pk.block(icrdx, icrdx, 3, 3) = tmp;
           _sins->Pk = var.eval();
           _sins->Pk.block(0, 0, nq, nq) = tmp_pk;
        }
        else if (sensor == UWB) _sins->Pk.block(icrdx, icrdx, 3, 3) = var.block(0, 0, 3, 3);
        _sins->Xk.resize(_sins->Pk.rows());
        _sins->Xk.setZero();
        if (_num_of_imu_axiliary > 0 && FuseType == STACK)
        {
            for (int imui = 0; imui < 1 + _num_of_imu_axiliary; imui++)
            {
                sins_mimu[imui]->qnb = _sins->qnb * base_att_trans::m2qua(_R_imui_imu0[imui]);
                sins_mimu[imui]->eth.Update(Cart2Geod(Eigen::Vector3d(pos[0], pos[1], pos[2]), false), Eigen::Vector3d::Zero());
                sins_mimu[imui]->Cnb = base_att_trans::q2mat(sins_mimu[imui]->qnb);
                sins_mimu[imui]->pos_ecef = Eigen::Vector3d(pos[0], pos[1], pos[2]) - sins_mimu[imui]->eth.Cen * sins_mimu[imui]->Cnb * (lever - _p_imui_imu0[imui]);
                sins_mimu[imui]->pos = Cart2Geod(sins_mimu[imui]->pos_ecef, false);
                //imu0 has been considered
                if (imui > 0)
                {
                    int icrdx_i = param_of_sins->operator[](param_of_sins->getParam(_name, par_type::CRD_X, "imu" + base_type_conv::int2str(imui))).index;
                    Eigen::Matrix3d Ceb_i = sins_mimu[imui]->eth.Cen * sins_mimu[imui]->Cnb;
                    Eigen::Matrix3d p_extr_cov = Eigen::Matrix3d::Zero();
                    Eigen::Matrix3d R_extr_cov = Eigen::Matrix3d::Zero();
                    if (_estimate_imui_extrinsic)
                    {
                        p_extr_cov(0, 0) = _initial_extrinsic_translation_cov(0);
                        p_extr_cov(1, 1) = _initial_extrinsic_translation_cov(1);
                        p_extr_cov(2, 2) = _initial_extrinsic_translation_cov(2);
                        R_extr_cov(0, 0) = _initial_extrinsic_rotation_cov(0) * glv.deg * glv.deg;
                        R_extr_cov(1, 1) = _initial_extrinsic_rotation_cov(1) * glv.deg * glv.deg;
                        R_extr_cov(2, 2) = _initial_extrinsic_rotation_cov(2) * glv.deg * glv.deg;
                    }
                    _sins->Pk.block(icrdx_i, icrdx_i, 3, 3) = _sins->Pk.block(icrdx, icrdx, 3, 3) + Ceb_i * p_extr_cov * Ceb_i.transpose();
                    _sins->Pk.block(icrdx_i - 6, icrdx_i - 6, 3, 3) = _sins->Pk.block(icrdx - 6, icrdx - 6, 3, 3) + Ceb_i * R_extr_cov * Ceb_i.transpose();
                }
            }
        }
    }

    void insprocesser::erase_bef(base_time t) {
        TimeStamp = insdata->erase_bef(t);
        for (int imui = 0; imui < _num_of_imu_axiliary + 1; imui++)
        {
            insdata->erase_bef(imui, t);
        }
    }

    MOTION_TYPE insprocesser::motion_state()
    {
        Eigen::Vector3d wmm = Eigen::Vector3d::Zero(3), vmm = Eigen::Vector3d::Zero(3);
        double nts = _shm->nSamples * _shm->ts;
        for (auto wm : _sins->_wm) wmm = wmm + wm;
        for (auto vm : _sins->_vm) vmm = vmm + vm;
        Eigen::Vector3d wbib = wmm / nts;
        Eigen::Vector3d fbib = vmm / nts;
        Vector r(5);
        r << _sins->an.norm(), _sins->vn.norm(), _sins->wnb.norm(), fbib.norm(), wbib.norm();
        _avar.update(r);
        _map_yaw.insert(std::make_pair(_sins->t, _sins->att(2)));
        if (!_aligned)
        {
            if (wbib.norm() < 0.05 * glv.dps && _avar(3) < 0.5 && _avar(4) < 0.5 * glv.dps)
                return m_static;
            else if (wbib.norm() < 0.1 * glv.dps && _avar(4) < 0.5 * glv.dps)
                return m_straight;
            else
                return m_default;
        }
        else
        {
            double vf = -glv.INF;
            if (_sins->wnb.norm() < 1.5 * glv.dps && abs(_sins->vb(1)) > 3)
                return m_straight;
            else if (_sins->an.norm() < 1 && _sins->wnb.norm() < 1 * glv.dps && _sins->vn.norm() < 0.02)
                return m_static;
            else
                return m_default;
        }
        return m_default;
    }

    MEAS_TYPE insprocesser::meas_state()
    {
        MOTION_TYPE motion = motion_state();
        switch (motion)
        {
        case m_static:
            return ZUPT_MEAS; break;
        case m_straight:
            return NHC_MEAS; break;
        default:
            return NO_MEAS; break;
        }
    }

    void insprocesser::set_posvel(Triple blh, Triple vn) {
        _sins->set_posvel(blh, vn);
    }

    void insprocesser::set_ins_out()
    {
        std::string tmp;
        tmp = dynamic_cast<set_out*>(_gset.get())->outputs("ins");
        if (tmp.empty())
        {
            tmp = _name + "result.ins";
        }
        _fins = new hwa_base::base_iof;
        if (_name != "") {
            hwa_base::base_type_conv::substitute(tmp, "$(rec)", _name, false);
        }
        _fins->mask(tmp);
        _fins->append(dynamic_cast<set_out*>(_gset.get())->append());

        _fcalib = new hwa_base::base_iof;
        _fcalib->mask("calibration_result.txt");
        _fcalib->append(dynamic_cast<set_out*>(_gset.get())->append());

        _fcalibstd = new hwa_base::base_iof;
        _fcalibstd->mask("calibration_result_std.txt");
        _fcalibstd->append(dynamic_cast<set_out*>(_gset.get())->append());

        tmp = dynamic_cast<set_out*>(_gset.get())->outputs("inskf");
        if (!tmp.empty())
        {
            _fkf = new hwa_base::base_iof;
            if (_name != "") {
                hwa_base::base_type_conv::substitute(tmp, "$(rec)", _name, false);
            }
            _fkf->mask(tmp);
            _fkf->append(dynamic_cast<set_out*>(_gset.get())->append());
        }
        else
        {
            _fkf = nullptr;
        }

        tmp = dynamic_cast<set_out*>(_gset.get())->outputs("inskfpk");
        if (!tmp.empty())
        {
            _fpk = new hwa_base::base_iof;
            if (_name != "") {
                hwa_base::base_type_conv::substitute(tmp, "$(rec)", _name, false);
            }
            _fpk->mask(tmp);
            _fpk->append(dynamic_cast<set_out*>(_gset.get())->append());
        }
        else
        {
            _fpk = nullptr;
        }

        std::ostringstream os;
        _sins->prt_header(os);
        _fins->write(os.str().c_str(), os.str().size());
        os.clear();
        os << "# Time(s)" << "\t" << "GNSS Lever-X(m)" << "\t" << "GNSS Lever-Y(m)" << "\t" << "GNSS Lever-Z(m)"
            << "\t" << "IMU-i Rotation-X(deg)" << "\t" << "IMU-i Rotation-Y(deg)" << "\t" << "IMU-i Rotation-Z(deg)"
            << "\t" << "IMU-i Tranlation-X(m)" << "\t" << "IMU-i Tranlation-Y(m)" << "\t" << "IMU-i Tranlation-Z(m)" << "\t" << "IMU-i t(s)" << std::endl;
        if (_fcalib)_fcalib->write(os.str().c_str(), os.str().size());
        if (_fcalibstd)_fcalibstd->write(os.str().c_str(), os.str().size());
    }

    void insprocesser::init_par()
    {
        // attitude
        param_of_sins->delAllParam();
        for (int ipar = (int)hwa_base::par_type::ATT_X; ipar <= (int)hwa_base::par_type::ATT_Z; ipar++)
            param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));
        // velocity
        for (int ipar = (int)hwa_base::par_type::VEL_X; ipar <= (int)hwa_base::par_type::VEL_Z; ipar++)
            param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));
        // position 
        for (int ipar = (int)hwa_base::par_type::CRD_X; ipar <= (int)hwa_base::par_type::CRD_Z; ipar++)
            param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));
        // bias
        for (int ipar = (int)hwa_base::par_type::eb_X; ipar <= (int)hwa_base::par_type::db_Z; ipar++)
            param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));

        //multi-imu
        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
        {
            //imui--imu0, i=1,2,...,_num_of_imu_axiliary
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                // attitude
                for (int ipar = (int)hwa_base::par_type::ATT_X; ipar <= (int)hwa_base::par_type::ATT_Z; ipar++)
                    param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                // velocity
                for (int ipar = (int)hwa_base::par_type::VEL_X; ipar <= (int)hwa_base::par_type::VEL_Z; ipar++)
                    param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                // position 
                for (int ipar = (int)hwa_base::par_type::CRD_X; ipar <= (int)hwa_base::par_type::CRD_Z; ipar++)
                    param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                // bias
                for (int ipar = (int)hwa_base::par_type::eb_X; ipar <= (int)hwa_base::par_type::db_Z; ipar++)
                    param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));

                if (_estimate_imui_extrinsic)
                {
                    //extrinsic orientation
                    for (int ipar = (int)hwa_base::par_type::EXTRINSIC_ATT_X; ipar <= (int)hwa_base::par_type::EXTRINSIC_ATT_Z; ipar++)
                        param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                    //extrinsic position 
                    for (int ipar = (int)hwa_base::par_type::EXTRINSIC_CRD_X; ipar <= (int)hwa_base::par_type::EXTRINSIC_CRD_Z; ipar++)
                        param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                }
                //extrinsic time
                if (_estimate_imui_t)
                {
                    param_of_sins->addParam(hwa_base::base_par(_name, hwa_base::par_type((int)hwa_base::par_type::EXTRINSIC_T), (int)hwa_base::par_type::EXTRINSIC_T, "imu" + hwa_base::base_type_conv::int2str(i)));
                }
            }
        }
        param_of_sins->reIndex();
    }

    void insprocesser::time_update_flex(double kfts, int fback, double inflation)
    {
        int n = _sins->Pk.rows();
        set_Ft();
        Matrix Fk = Matrix::Zero(nq, nq);
        Fk.block(0, 0, 15, 15) = Ft.block(0, 0, 15, 15) * kfts;
        if (FuseType == hwa_ins::STACK)
        {
            for (int imui = 1; imui < 1 + _num_of_imu_axiliary; imui++)
            {
                int iattx_i = param_of_sins->getParam(_name, hwa_base::par_type::ATT_X, "imu" + hwa_base::base_type_conv::int2str(imui));
                Fk.block(iattx_i, iattx_i, 15, 15) = Ft.block(iattx_i, iattx_i, 15, 15) * (sins_mimu[imui]->nts);
            }
        }
        Fk = Fk + Matrix::Identity(nq, nq);
        _sins->Xk.block(0, 0, nq, 1) = Fk * _sins->Xk.block(0, 0, nq, 1).eval();
        Matrix Qk = (_sins->Qt * kfts * inflation).array().matrix().asDiagonal();
        _sins->Pk.block(0, 0, nq, nq) = Fk * _sins->Pk.block(0, 0, nq, nq).eval() * (Fk.transpose());
        _sins->Pk.block(0, 0, _sins->Qt.size(), _sins->Qt.size()) += Qk;
        if (n > nq) {
            _sins->Pk.block(nq, 0, n - nq, nq) = _sins->Pk.block(nq, 0, n - nq, nq).eval() * Fk.transpose();
            _sins->Pk.block(0, nq, nq, n - nq) = Fk * _sins->Pk.block(0, nq, nq, n - nq).eval();
        }
        _sins->Pk_Sav = _sins->Pk;
    }

    void insprocesser::set_Ft()
    {     
        _sins->err_mat();

        //InEKF
        if (_Estimator == INEKF) {
            Ft.block(0, 0, 3, 3) = -hwa_base::askew(_sins->eth.weie);  // att-att
            Ft.block(3, 0, 3, 3) = hwa_base::askew(_sins->eth.Cen * _sins->eth.gcc); //vel-att
            Ft.block(0, 9, 3, 3) = -_sins->Ceb; //att-eb
            Ft.block(3, 3, 3, 3) = -2 * hwa_base::askew(_sins->eth.weie);      // vel-vel
            Ft.block(3, 9, 3, 3) = -hwa_base::askew(_sins->ve) * _sins->Ceb; //vel-eb
            Ft.block(3, 12, 3, 3) = -_sins->Ceb; //vel-db
            Ft.block(6, 9, 3, 3) = -hwa_base::askew(_sins->pos_ecef - _sins->initial_pos) * _sins->Ceb;//pos-eb
            Ft.block(6, 3, 3, 3) = SO3::Identity(); //pos-vel
            Ft.block(6, 0, 3, 3) = hwa_base::askew(_sins->eth.Cen * _sins->eth.gcc) * _sins->nts; //pos-att
            Ft.block(6, 12, 3, 3) = -_sins->Ceb * _sins->nts; // pos-db

            G.block(0, 0, 3, 3) = _sins->Ceb;
            G.block(3, 0, 3, 3) = hwa_base::askew(_sins->ve) * _sins->Ceb;
            G.block(6, 0, 3, 3) = hwa_base::askew(_sins->pos_ecef - _sins->initial_pos) * _sins->Ceb;
            G.block(6, 3, 3, 3) = _sins->Ceb * _sins->nts;
            G.block(3, 3, 3, 3) = _sins->Ceb;
            G.block(9, 6, 3, 3) = -SO3::Identity();
            G.block(12, 9, 3, 3) = -SO3::Identity();
        }

        if (_Estimator == NORMAL) {
            Ft.block(0, 0, 3, 3) = -hwa_base::askew(_sins->eth.weie);  // att-att
            Ft.block(0, 9, 3, 3) = -_sins->Ceb;                        // att-eb
            Ft.block(3, 0, 3, 3) = hwa_base::askew(_sins->eth.Cen * _sins->fn);  // vel-att
            Ft.block(3, 3, 3, 3) = -2 * hwa_base::askew(_sins->eth.weie);      // vel-vel
            Ft.block(3, 12, 3, 3) = _sins->Ceb;                              // vel-db
            if (FuseType == hwa_ins::VIRTUAL && _num_of_imu_axiliary > 0) {
                Ft.block(3, 9, 3, 3) = _sins->jacobian_v_bg;
            }
            Ft.block(6, 3, 3, 3) = SO3::Identity();              // pos-vel
            Ft.block(9, 9, 3, 3) = _sins->_tauG.array().matrix().asDiagonal();    // eb-eb
            Ft.block(12, 12, 3, 3) = _sins->_tauA.array().matrix().asDiagonal();    // db-db
        }

        if (_Estimator == NORMAL && FuseType == STACK)
        {
            for (int imui = 1; imui <= _num_of_imu_axiliary; imui++)
            {
                int iattx_i = param_of_sins->getParam(_name, hwa_base::par_type::ATT_X, "imu" + hwa_base::base_type_conv::int2str(imui));
                Ft.block(iattx_i, iattx_i, 3, 3) = -hwa_base::askew(sins_mimu[imui]->eth.weie);  // att-att
                Ft.block(iattx_i, iattx_i + 6, 3, 3) = SO3::Zero();            // att-pos
                Ft.block(iattx_i, iattx_i + 9, 3, 3) = -sins_mimu[imui]->Ceb;                        // att-eb
                Ft.block(iattx_i + 3, iattx_i, 3, 3) = hwa_base::askew(sins_mimu[imui]->eth.Cen * sins_mimu[imui]->fn);  // vel-att
                Ft.block(iattx_i + 3, iattx_i + 3, 3, 3) = -2 * hwa_base::askew(sins_mimu[imui]->eth.weie);      // vel-vel
                Ft.block(iattx_i + 3, iattx_i + 12, 3, 3) = sins_mimu[imui]->Ceb;                              // vel-db
                Ft.block(iattx_i + 6, iattx_i + 3, 3, 3) = SO3::Identity();              // pos-vel
                Ft.block(iattx_i + 9, iattx_i + 9, 3, 3) = sins_mimu[imui]->_tauG.array().matrix().asDiagonal();    // eb-eb
                Ft.block(iattx_i + 12, iattx_i + 12, 3, 3) = sins_mimu[imui]->_tauA.array().matrix().asDiagonal();    // db-db
            }
        }
        return;
    }

    bool insprocesser::load_data() {
        bool ok = insdata->load(_sins->_wm, _sins->_vm, _shm->t, _shm->ts, _shm->nSamples, _shm->Status);

        //TimeStamp.add_dsec(_shm->ts);
        TimeStamp = base_time(TimeStamp.gwk(), _shm->t);
        _sins->t = TimeStamp.sow() + TimeStamp.dsec();

        for (int imui = 0; imui < 1 + _num_of_imu_axiliary; imui++)
        {
            insdata->load(imui, _wm_mimu[imui], _vm_mimu[imui], _shm_mimu[imui].t, _shm_mimu[imui].ts, _shm_mimu[imui].nSamples, _shm_mimu[imui].Status);
        }
        return ok;
    }

    int insprocesser::ProcessOneEpoch() {
        _sins->Update(_wm_mimu, _vm_mimu, _shm_mimu);

        time_update_flex(_sins->nts, 0, inflation);

        if (FuseType == hwa_ins::STACK) {
            for (int imui = 0; imui <= _num_of_imu_axiliary; imui++) sins_mimu[imui]->Update(_wm_mimu[imui], _vm_mimu[imui], _shm_mimu[imui]);
        }
        return 1;
    }

    int insprocesser::ProcessWithoutCovUpdate() {
        _sins->Update(_wm_mimu, _vm_mimu, _shm_mimu);

        if (FuseType == hwa_ins::STACK) {
            for (int imui = 0; imui <= _num_of_imu_axiliary; imui++) sins_mimu[imui]->Update(_wm_mimu[imui], _vm_mimu[imui], _shm_mimu[imui]);
        }
        return 1;
    }

    void insprocesser::MeasCrt() {
        _sins->_imu.Update(_sins->_wm, _sins->_vm, *_shm);
        for (int imui = 0; imui <= _num_of_imu_axiliary; imui++)
        {
            sins_mimu[imui]->_imu.Update(_wm_mimu[imui], _vm_mimu[imui], _shm_mimu[imui]);
        }
    }

    void insprocesser::UpdateViewer() {
        _imu_state.orientation = Eigen::Quaterniond(base_att_trans::q2mat(_sins->qnb));
        _imu_state.position = Geod2Cart(_sins->pos, false);
        _publisher.UpdateNewState(_imu_state);
    }

    void insprocesser::prt_sins(std::ostringstream& os) {
        _sins->prt_sins(os);
        if (_shm->_imu_scale)
            os << std::fixed << std::setprecision(4) <<
            std::setw(15) << _sins->Kg(0) <<
            std::setw(15) << _sins->Kg(1) <<
            std::setw(15) << _sins->Kg(2) <<
            std::setw(15) << _sins->Ka(0) <<
            std::setw(15) << _sins->Ka(1) <<
            std::setw(15) << _sins->Ka(2);
        if (_shm->_imu_inst_rot)
        {
            Eigen::Vector3d inst_att = base_att_trans::q2att(_sins->qvb) / glv.deg;
            Eigen::Vector3d ins_vv = _sins->Cvb * _sins->vb;
            os << std::fixed << std::setprecision(4) <<
                std::setw(15) << inst_att(0) <<
                std::setw(15) << inst_att(1) <<
                std::setw(15) << inst_att(2) <<
                std::setw(15) << ins_vv(0) <<
                std::setw(15) << ins_vv(1) <<
                std::setw(15) << ins_vv(2);
        }
    }

    void insprocesser::_feed_back() {
        baseprocesser::_feed_back();
        if (_num_of_imu_axiliary > 0 && FuseType == STACK)
        {
            for (int imui = 0; imui < 1 + _num_of_imu_axiliary; imui++)
            {
                std::string name = "";
                if (imui > 0)
                    name = "imu" + base_type_conv::int2str(imui);
                int i = 0;

                i = param_of_sins->getParam(_name, par_type::ATT_X, name);
                sins_mimu[imui]->qeb = sins_mimu[imui]->qeb - _sins->Xk.block(i, 0, 3, 1);
                sins_mimu[imui]->ve = sins_mimu[imui]->ve - _sins->Xk.block(i + 3, 0, 3, 1);
                sins_mimu[imui]->pos_ecef = sins_mimu[imui]->pos_ecef - _sins->Xk.block(i + 6, 0, 3, 1);
                sins_mimu[imui]->eb = sins_mimu[imui]->eb + _sins->Xk.block(i + 9, 0, 3, 1);
                sins_mimu[imui]->db = sins_mimu[imui]->db + _sins->Xk.block(i + 12, 0, 3, 1);

                int iscale = param_of_sins->getParam(_name, par_type::gyro_scale_X, "");
                int iodo = param_of_sins->getParam(_name, par_type::ODO_k, "");
                int irot = param_of_sins->getParam(_name, par_type::IMU_INST_ATT_X, "");
                if (_shm->_imu_scale)
                {
                    sins_mimu[0]->Kg = sins_mimu[0]->Kg - _sins->Xk.block(iscale, 0, 3, 1);
                    sins_mimu[0]->Ka = sins_mimu[0]->Ka - _sins->Xk.block(iscale + 3, 0, 3, 1);
                }
                if (_shm->_imu_inst_rot)
                {
                    std::cout << "imu_inst_att: " << std::fixed << std::setprecision(10) << std::setw(15) << base_att_trans::q2att(sins_mimu[0]->qvb).transpose() / glv.deg << std::endl;
                    sins_mimu[0]->qvb = sins_mimu[0]->qvb - _sins->Xk.block(irot, 0, 3, 1);
                    sins_mimu[0]->Cvb = base_att_trans::q2mat(sins_mimu[0]->qvb);
                    sins_mimu[0]->Cbv = sins_mimu[0]->Cvb.transpose();
                }

                sins_mimu[imui]->pos = Cart2Geod(sins_mimu[imui]->pos_ecef, false);
                sins_mimu[imui]->eth.Update(sins_mimu[imui]->pos, sins_mimu[imui]->vn);
                sins_mimu[imui]->vn = sins_mimu[imui]->eth.Cne * sins_mimu[imui]->ve;
                sins_mimu[imui]->qnb = base_att_trans::m2qua(sins_mimu[imui]->eth.Cne) * sins_mimu[imui]->qeb;
                sins_mimu[imui]->att = base_att_trans::q2att(sins_mimu[imui]->qnb);
                sins_mimu[imui]->Ceb = base_att_trans::q2mat(sins_mimu[imui]->qeb);
                sins_mimu[imui]->Cnb = base_att_trans::q2mat(sins_mimu[imui]->qnb);

                if (_estimate_imui_extrinsic && imui > 0)
                {
                    i = param_of_sins->getParam(_name, par_type::EXTRINSIC_ATT_X, name);

                    base_quat dq_cam = base_att_trans::rv2q(-_sins->Xk.block(i, 0, 3, 1));
                    Eigen::Quaterniond _dq_ext;
                    _dq_ext.w() = dq_cam.q0; _dq_ext.x() = dq_cam.q1; _dq_ext.y() = dq_cam.q2; _dq_ext.z() = dq_cam.q3;
                    _dq_ext.normalize();
                    //_R_imui_imu0[imui] = _R_imui_imu0[imui] * (Eigen::Matrix3d::Identity() - t_gbase::askew(_sins->Xk.block(i, 0, 3, 1)));
                    _R_imui_imu0[imui] = _R_imui_imu0[imui] * _dq_ext;
                    _p_imui_imu0[imui] = _p_imui_imu0[imui] - _sins->Xk.block(i + 3, 0, 3, 1);
                }
                if (_estimate_imui_t && imui > 0)
                {
                    i = param_of_sins->getParam(_name, par_type::EXTRINSIC_T, name);
                    _t_imui_imu0[imui] = _t_imui_imu0[imui] - _sins->Xk(i);
                }
            }
        }
        _sins->Xk = Vector::Zero(_sins->Pk.rows());

        //std::cout << TimeStamp.sow() + TimeStamp.dsec() << " Cnb: " << std::fixed << std::setprecision(6) << _sins->Cnb << "\n";
        //std::cout << TimeStamp.sow() + TimeStamp.dsec() << " eb: " << std::fixed << std::setprecision(6) << _sins->eb.transpose() << "\n";
        //std::cout << TimeStamp.sow() + TimeStamp.dsec() << " db: " << std::fixed << std::setprecision(6) << _sins->db.transpose() << "\n";
        //std::cout << TimeStamp.sow() + TimeStamp.dsec() << " vn: " << std::fixed << std::setprecision(6) << _sins->vn.transpose() << "\n";
        //std::cout << TimeStamp.sow() + TimeStamp.dsec() << " Pos: " << std::fixed << std::setprecision(6) << _sins->pos_ecef.transpose() << "\n";

    }
}
