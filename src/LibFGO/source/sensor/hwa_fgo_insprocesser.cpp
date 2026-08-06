#include "hwa_fgo_insprocesser.h"

namespace hwa_fgo {
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

    void insprocesser::merge_init(const Triple& pos, const Triple& lever)
    {
        if (pos.norm() == 0) return;
        _sins->eth.Update(Cart2Geod(Eigen::Vector3d(pos[0], pos[1], pos[2]), false), Eigen::Vector3d::Zero());
        _sins->Cnb = base_att_trans::q2mat(_sins->qnb);
        _sins->Ceb = _sins->eth.Cen * _sins->Cnb;
        _sins->pos_ecef = Eigen::Vector3d(pos[0], pos[1],pos[2]) - _sins->eth.Cen * _sins->Cnb * lever;
        _sins->pos = Cart2Geod(_sins->pos_ecef, false);
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

    void insprocesser::_input_imu_data(const double& t, const std::vector<Eigen::Vector3d>& wm, const std::vector<Eigen::Vector3d>& vm)
    {
        assert(wm.size() == vm.size());
        assert(wm.size() == 1);
        for (int i = 0; i < wm.size(); i++)
        {
            _fgo_info->_accBuf.push(std::make_pair(t, vm[i] / _shm->ts));
            _fgo_info->_gyrBuf.push(std::make_pair(t, wm[i] / _shm->ts));
        }
    }

    int insprocesser::ProcessOneEpoch() {
        _sins->Update(_wm_mimu, _vm_mimu, _shm_mimu);
        _input_imu_data(dTime(), _wm_mimu[0], _vm_mimu[0]);
        _fgo_info->_gravity = _sins->eth.Cen  * -_sins->eth.gcc;  // NAV_REFERENCE_FRAME::E_F

        if (FuseType == hwa_ins::STACK) {
            for (int imui = 0; imui <= _num_of_imu_axiliary; imui++) sins_mimu[imui]->Update(_wm_mimu[imui], _vm_mimu[imui], _shm_mimu[imui]);
        }
        return 1;
    }

    void insprocesser::_addResidualBlocks(ceres::Problem& problem) {

        double cost_save = _fgo_info->cost;
        std::vector<double> residuals;
        
        for (int i = 0; i < _fgo_info->rover_count; i++)
        {
            ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(_fgo_info->_para_pose[i], SIZE_POSE, local_parameterization);
            problem.AddParameterBlock(_fgo_info->_para_speed_bias[i], SIZE_SPEEDBIAS);

            Eigen::Vector3d pos(_fgo_info->_para_pose[i][0], _fgo_info->_para_pose[i][1], _fgo_info->_para_pose[i][2]);
            Eigen::Quaterniond quat(_fgo_info->_para_pose[i][6], _fgo_info->_para_pose[i][3], _fgo_info->_para_pose[i][4], _fgo_info->_para_pose[i][5]);
            InitialPoseFactor* initial_pose = new InitialPoseFactor(pos, quat);
            initial_pose->sqrt_info = 1e-7 * Eigen::Matrix<double, 6, 6>::Identity();
            problem.AddResidualBlock(initial_pose, NULL, _fgo_info->_para_pose[i]);

            InitialVelBiasFactor* initial_bias = new InitialVelBiasFactor(
                Eigen::Vector3d(_fgo_info->_para_speed_bias[i][0], _fgo_info->_para_speed_bias[i][1], _fgo_info->_para_speed_bias[i][2]),
                Eigen::Vector3d(_fgo_info->_para_speed_bias[i][3], _fgo_info->_para_speed_bias[i][4], _fgo_info->_para_speed_bias[i][5]),
                Eigen::Vector3d(_fgo_info->_para_speed_bias[i][6], _fgo_info->_para_speed_bias[i][7], _fgo_info->_para_speed_bias[i][8]));
            initial_bias->sqrt_info = 1e-7 * Eigen::Matrix<double, 9, 9>::Identity();
            problem.AddResidualBlock(initial_bias, NULL, _fgo_info->_para_speed_bias[i]);
        }

        problem.Evaluate(
            ceres::Problem::EvaluateOptions(),
            &_fgo_info->cost,
            &residuals,
            nullptr,
            nullptr);

        std::cout
            << std::fixed
            << std::setprecision(10)
            << "initial bias cost = "
            << _fgo_info->cost - cost_save
            << std::endl;

        cost_save = _fgo_info->cost;

        for (int i = 0; i < _fgo_info->rover_count - 1; i++)
        {
            int j = i + 1;
            if (_fgo_info->_pre_integrations[j]->sum_dt > 5)
                continue;
            IMUFactor* imu_factor = new IMUFactor(_fgo_info->_pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, _fgo_info->_para_pose[i], _fgo_info->_para_speed_bias[i], _fgo_info->_para_pose[j], _fgo_info->_para_speed_bias[j]);
        
            cost_save = _fgo_info->cost;
            problem.Evaluate(
                ceres::Problem::EvaluateOptions(),
                &_fgo_info->cost,
                &residuals,
                nullptr,
                nullptr);

            std::cout
                << std::fixed
                << std::setprecision(10)
                << "imu preintegration cost [" << i << "] = "
                << _fgo_info->cost - cost_save
                << std::endl;      
        }
    }

    void insprocesser::_addMarginInfo() {

        if (!_fgo_info->time_to_margin()) return;

        if (_fgo_info->_pre_integrations[1]->sum_dt <= 5)
        {
            IMUFactor* imu_factor = new IMUFactor(_fgo_info->_pre_integrations[1]);
            ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                std::vector<double*>{_fgo_info->_para_pose[0], _fgo_info->_para_speed_bias[0], _fgo_info->_para_pose[1], _fgo_info->_para_speed_bias[1]},
                std::vector<int>{0, 1});
            _fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        for (int i = 1; i <= _fgo_info->rover_count - 1; i++)
        {
            _fgo_info->addr_shift[reinterpret_cast<long>(_fgo_info->_para_pose[i])] = _fgo_info->_para_pose[i - 1];
            _fgo_info->addr_shift[reinterpret_cast<long>(_fgo_info->_para_speed_bias[i])] = _fgo_info->_para_speed_bias[i - 1];
        }
    }
}

namespace hwa_fgo {
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
            //_sins->qnb = base_att_trans::a2qua(_sins->align_coarse(wmm, vmm));
            _sins->eb = wmm / _shm->ts;
            _sins->db = vmm / _shm->ts + _sins->eth.gcc;
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
        Triple vel = endpos - _pre_pos; _pre_pos = endpos;
        Triple blh = Cart2Geod(pos, false);
        Triple vn = Cen(blh).transpose() * vel;
        _sins->set_posvel(blh, vn);

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
        std::cerr << "Velocity std::vector alignment failed. Please speed up! " << std::endl;
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
}

namespace hwa_fgo {

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

    void insprocesser::_feed_back(const base_posdata::data_pos& _pos, const Triple RobustFixedPos) {

//#ifdef DEBUG_NEW
//        std::cout << std::fixed << std::setprecision(6);
//
//        std::cout << "================ INS Begin ================" << std::endl;
//
//        std::cout << "Position (ECEF) [m] : "
//            << _sins->pos_ecef(0) << "  "
//            << _sins->pos_ecef(1) << "  "
//            << _sins->pos_ecef(2) << std::endl;
//
//        std::cout << "Velocity (ECEF) [m/s] : "
//            << _sins->ve(0) << "  "
//            << _sins->ve(1) << "  "
//            << _sins->ve(2) << std::endl;
//
//        std::cout << "Attitude (Roll Pitch Yaw) [deg] : "
//            << _sins->att(0) << "  "
//            << _sins->att(1) << "  "
//            << _sins->att(2) << std::endl;
//
//        std::cout << "==============================================" << std::endl;
//#endif

        Eigen::Vector3d mean_ba = _fgo_info->_Bas[_fgo_info->rover_count - 1];
        Eigen::Vector3d mean_bg = _fgo_info->_Bgs[_fgo_info->rover_count - 1];
        Eigen::Quaterniond e_q = Eigen::Quaterniond(_fgo_info->_Rs[_fgo_info->rover_count - 1]);
        e_q.normalized();
        _sins->qeb = base_quat(e_q.w(), e_q.x(), e_q.y(), e_q.z());
        _sins->Ceb = base_att_trans::q2mat(_sins->qeb);
        _sins->ve = _fgo_info->_Vs[_fgo_info->rover_count - 1];
        _sins->pos_ecef = _fgo_info->_Ps[_fgo_info->rover_count - 1];
        if (RobustFixedPos[0] != 0)
            _sins->pos_ecef = RobustFixedPos;
        Eigen::Vector3d robustpos;
        _sins->eb = mean_bg;
        _sins->db = mean_ba;
        _sins->qnb = base_att_trans::m2qua(_sins->eth.Cne) * _sins->qeb;
        _sins->Cnb = base_att_trans::q2mat(_sins->qnb);
        _sins->vn = _sins->eth.Cne * _sins->ve;
        _sins->pos = Cart2Geod(_sins->pos_ecef, false);
        _sins->att = base_att_trans::q2att(_sins->qnb);
        if(_pos.pos[0] != 0)
            _sins->xyz_out = _pos.pos;
        else
			_sins->xyz_out = _sins->pos_ecef;

//#ifdef DEBUG_NEW
//        std::cout << std::fixed << std::setprecision(6);
//
//        std::cout << "================ INS Feedback ================" << std::endl;
//
//        std::cout << "Position (ECEF) [m] : "
//            << _sins->pos_ecef(0) << "  "
//            << _sins->pos_ecef(1) << "  "
//            << _sins->pos_ecef(2) << std::endl;
//
//        std::cout << "Velocity (ECEF) [m/s] : "
//            << _sins->ve(0) << "  "
//            << _sins->ve(1) << "  "
//            << _sins->ve(2) << std::endl;
//
//        std::cout << "Attitude (Roll Pitch Yaw) [deg] : "
//            << _sins->att(0) << "  "
//            << _sins->att(1) << "  "
//            << _sins->att(2) << std::endl;
//
//        std::cout << "==============================================" << std::endl;
//#endif
    }
}
