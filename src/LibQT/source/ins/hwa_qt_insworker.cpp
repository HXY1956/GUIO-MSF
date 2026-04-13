#include "hwa_qt_insworker.h"
#include "hwa_set_ign.h"

namespace hwa_qt {
    qt_ins_worker::qt_ins_worker(std::shared_ptr<hwa_set::set_base> _gset, std::string _dir, QObject* parent) : QObject(parent), gset(_gset), dir(_dir) {
        nq = 15;
        _name = "HXY";
        sins = std::make_shared<hwa_ins::ins_obj>(gset.get());

        inflation = dynamic_cast<hwa_set::set_ign*>(gset.get())->inflation();
        EstimatorType = dynamic_cast<hwa_set::set_ign*>(gset.get())->fuse_type();
        ZUPT_Noise = dynamic_cast<hwa_set::set_ign*>(gset.get())->zupt_dstd(); ZUPT_Noise *= ZUPT_Noise;

        ts = 1.0 / dynamic_cast<hwa_set::set_ins*>(gset.get())->freq();
        FuseType = hwa_ins::str2ins_fuse_mode(dynamic_cast<hwa_set::set_ins*>(gset.get())->fuse_type());
        R2P = dynamic_cast<hwa_set::set_ins*>(gset.get())->R2P();
        _num_of_imu_axiliary = dynamic_cast<hwa_set::set_ins*>(gset.get())->num_of_ins_auxiliary();
        _initial_extrinsic_rotation_cov = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_extrinsic_rotation_cov();
        _initial_extrinsic_translation_cov = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_extrinsic_translation_cov();
        _initial_t_cov = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_t_cov();
        _enf_R_std = dynamic_cast<hwa_set::set_ins*>(gset.get())->enf_R_std();
        _enf_p_std = dynamic_cast<hwa_set::set_ins*>(gset.get())->enf_p_std();
        initial_pos_ecef = dynamic_cast<hwa_set::set_ins*>(gset.get())->pos();
        inipos = hwa_base::Cart2Geod(initial_pos_ecef, false);

        R_t_i = dynamic_cast<hwa_set::set_tracker*>(gset.get())->R_tracker_imu();
        t_t_i = dynamic_cast<hwa_set::set_tracker*>(gset.get())->T_tracker_imu();
        Tracker_Trans_noise = dynamic_cast<hwa_set::set_tracker*>(gset.get())->trans_noise();
        Tracker_Rot_noise = dynamic_cast<hwa_set::set_tracker*>(gset.get())->rot_noise();

        interval = dynamic_cast<hwa_set::set_proc*>(gset.get())->insinterval();WindowSize = int(5.0 / interval);
        vis_interval = dynamic_cast<hwa_set::set_proc*>(gset.get())->visinterval();

        VIS_Noise = dynamic_cast<hwa_set::set_vis*>(gset.get())->feature_observation_noise();VIS_Noise *= VIS_Noise;
        cam_list = dynamic_cast<hwa_set::set_vis*>(gset.get())->camera_list();
        vis_size = dynamic_cast<hwa_set::set_vis*>(gset.get())->max_cam_state_size();

        for (int id = 0; id < _num_of_imu_axiliary + 1; id++)
        {
            _R_imui_imu0[id] = dynamic_cast<hwa_set::set_ins*>(gset.get())->R_imui_imu0(id);
            _p_imui_imu0[id] = dynamic_cast<hwa_set::set_ins*>(gset.get())->p_imui_imu0(id);
            _t_imui_imu0[id] = dynamic_cast<hwa_set::set_ins*>(gset.get())->t_imui_imu0(id);
            sins_mimu[id] = std::make_unique<hwa_ins::ins_obj>(gset.get());
        }
        sins->_mimu._num_of_imu_axiliary = _num_of_imu_axiliary;
        sins->_mimu._p_imui_imu0 = _p_imui_imu0;
        sins->_mimu._R_imui_imu0 = _R_imui_imu0;
        int add_dim = 15;
        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
            nq += add_dim * _num_of_imu_axiliary;
        set_out();

        Ft = Matrix::Zero(nq, nq);
        G = Matrix::Zero(nq, nq - 3);
        ins_udp_client udp("127.0.0.1", 8888);
        std::vector<std::string> portlist = dynamic_cast<hwa_set::set_ins*>(gset.get())->PortList();
        std::string Imusavedir = dir + "\\ImuData";
        std::filesystem::create_directories(Imusavedir.c_str());
        num_of_ins = portlist.size();
        for (int i = 0; i < num_of_ins; ++i) {
            readers[i] = std::make_unique<ins_reader>(i, portlist[i], 921600, ts, &udp, Imusavedir, false);
        }
    }

    qt_ins_worker::~qt_ins_worker() {
        for (int i = 0; i < num_of_ins; ++i) {
            readers[i]->stop();
        }
        for (auto& t : threads) if (t.joinable()) t.join();
        cycleTimer->stop();
    }

    double qt_ins_worker::floor1w(double t) {
        return floor(t * 10) / 10.0;
    }

    double qt_ins_worker::floor2w(double t) {
        return floor(t * 100) / 100.0;
    }

    Eigen::Quaterniond qt_ins_worker::I2E(hwa_base::base_quat q) {
        return Eigen::Quaterniond(q.q0, q.q1, q.q2, q.q3);
    }

    hwa_base::base_quat qt_ins_worker::E2I(Eigen::Quaterniond q) {
        return hwa_base::base_quat(q.w(), q.x(), q.y(), q.z());
    }

    SO3 qt_ins_worker::askew(const Triple& v)
    {
        SO3 vnx;
        vnx << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return vnx;
    }

    bool qt_ins_worker::Iequal(double t1, double t2) {
        return abs(t1 - t2) < ts / 2;
    }

    Eigen::Quaterniond qt_ins_worker::M2Q(SO3 R) {
        Eigen::JacobiSVD<SO3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        SO3 U = svd.matrixU();
        SO3 V = svd.matrixV();
        SO3 R_so3 = U * V.transpose();
        if (R_so3.determinant() < 0)
            R_so3 = U * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * V.transpose();
        Eigen::Quaterniond q(R_so3);
        return q;
    }

    void qt_ins_worker::set_out()
    {
        std::string tmp;
        tmp = dynamic_cast<hwa_set::set_out*>(gset.get())->outputs("ins");
        if (tmp.empty())
        {
            tmp = _name + "result.ins";
        }
        _fins = new hwa_base::base_iof;
        if (_name != "") {
            hwa_base::base_type_conv::substitute(tmp, "$(rec)", _name, false);
        }
        _fins->mask(tmp);
        _fins->append(dynamic_cast<hwa_set::set_out*>(gset.get())->append());

        _fcalib = nullptr;
        _fcalib = new hwa_base::base_iof;
        _fcalib->mask("calibration_result.txt");
        _fcalib->append(dynamic_cast<hwa_set::set_out*>(gset.get())->append());

        _fcalibstd = nullptr;
        _fcalibstd = new hwa_base::base_iof;
        _fcalibstd->mask("calibration_result_std.txt");
        _fcalibstd->append(dynamic_cast<hwa_set::set_out*>(gset.get())->append());

        tmp = dynamic_cast<hwa_set::set_out*>(gset.get())->outputs("inskf");
        if (!tmp.empty())
        {
            _fkf = new hwa_base::base_iof;
            if (_name != "") {
                hwa_base::base_type_conv::substitute(tmp, "$(rec)", _name, false);
            }
            _fkf->mask(tmp);
            _fkf->append(dynamic_cast<hwa_set::set_out*>(gset.get())->append());
        }
        else
        {
            _fkf = nullptr;
        }

        tmp = dynamic_cast<hwa_set::set_out*>(gset.get())->outputs("inskfpk");
        if (!tmp.empty())
        {
            _fpk = new hwa_base::base_iof;
            if (_name != "") {
                hwa_base::base_type_conv::substitute(tmp, "$(rec)", _name, false);
            }
            _fpk->mask(tmp);
            _fpk->append(dynamic_cast<hwa_set::set_out*>(gset.get())->append());
        }
        else
        {
            _fpk = nullptr;
        }


        std::ostringstream os;
        sins->prt_header(os);
        _fins->write(os.str().c_str(), os.str().size());
        os.clear();
        os << "# Time(s)" << "\t" << "GNSS Lever-X(m)" << "\t" << "GNSS Lever-Y(m)" << "\t" << "GNSS Lever-Z(m)"
            << "\t" << "IMU-i Rotation-X(deg)" << "\t" << "IMU-i Rotation-Y(deg)" << "\t" << "IMU-i Rotation-Z(deg)"
            << "\t" << "IMU-i Tranlation-X(m)" << "\t" << "IMU-i Tranlation-Y(m)" << "\t" << "IMU-i Tranlation-Z(m)" << "\t" << "IMU-i t(s)" << std::endl;
        if (_fcalib)_fcalib->write(os.str().c_str(), os.str().size());
        if (_fcalibstd)_fcalibstd->write(os.str().c_str(), os.str().size());

    }

    void qt_ins_worker::init_par()
    {
        // attitude
        param_of_sins.delAllParam();
        for (int ipar = (int)hwa_base::par_type::ATT_X; ipar <= (int)hwa_base::par_type::ATT_Z; ipar++)
            param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));
        // velocity
        for (int ipar = (int)hwa_base::par_type::VEL_X; ipar <= (int)hwa_base::par_type::VEL_Z; ipar++)
            param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));
        // position 
        for (int ipar = (int)hwa_base::par_type::CRD_X; ipar <= (int)hwa_base::par_type::CRD_Z; ipar++)
            param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));
        // bias
        for (int ipar = (int)hwa_base::par_type::eb_X; ipar <= (int)hwa_base::par_type::db_Z; ipar++)
            param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, ""));

        //multi-imu
        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
        {
            //imui--imu0, i=1,2,...,_num_of_imu_axiliary
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                // attitude
                for (int ipar = (int)hwa_base::par_type::ATT_X; ipar <= (int)hwa_base::par_type::ATT_Z; ipar++)
                    param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                // velocity
                for (int ipar = (int)hwa_base::par_type::VEL_X; ipar <= (int)hwa_base::par_type::VEL_Z; ipar++)
                    param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                // position 
                for (int ipar = (int)hwa_base::par_type::CRD_X; ipar <= (int)hwa_base::par_type::CRD_Z; ipar++)
                    param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                // bias
                for (int ipar = (int)hwa_base::par_type::eb_X; ipar <= (int)hwa_base::par_type::db_Z; ipar++)
                    param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));

                if (_estimate_imui_extrinsic)
                {
                    //extrinsic orientation
                    for (int ipar = (int)hwa_base::par_type::EXTRINSIC_ATT_X; ipar <= (int)hwa_base::par_type::EXTRINSIC_ATT_Z; ipar++)
                        param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                    //extrinsic position 
                    for (int ipar = (int)hwa_base::par_type::EXTRINSIC_CRD_X; ipar <= (int)hwa_base::par_type::EXTRINSIC_CRD_Z; ipar++)
                        param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type(ipar), ipar, "imu" + hwa_base::base_type_conv::int2str(i)));
                }
                //extrinsic time
                if (_estimate_imui_t)
                {
                    param_of_sins.addParam(hwa_base::base_par(_name, hwa_base::par_type((int)hwa_base::par_type::EXTRINSIC_T), (int)hwa_base::par_type::EXTRINSIC_T, "imu" + hwa_base::base_type_conv::int2str(i)));
                }
            }
        }
        param_of_sins.reIndex();
        _param = param_of_sins;
    }

    bool qt_ins_worker::align_tracker() {
        _OneBatch Res = EmitQueue.front();
        SO3 R_t_r(Res.Q);

        std::lock_guard<std::mutex> lock(ins_mutex);
        R_r_e = sins->Ceb * R_t_i * R_t_r.transpose();
        q_r_e = M2Q(R_r_e);
        Triple t_t_r = Res.T;
        Triple t_t_e = sins->pos_ecef + sins->qeb * t_t_i;
        t_r_e = -R_r_e * t_t_r + t_t_e;

        std::cerr << "Tracker Aligned With ECEF Succesfully" << std::endl;
        return true;
    }

    bool qt_ins_worker::align_static() {
        int n = vm_align.size();
        if (n < 200) return false;
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
        sins->qnb = hwa_base::Qbase2eigen(q_i_n);
        sins->Cnb = hwa_base::base_att_trans::q2mat(sins->qnb);
        sins->Cbn = sins->Cnb.transpose();
        sins->set_posvel(inipos, Triple::Zero(), sins->qnb);
        sins->Ceb = sins->eth.Cen * sins->Cnb;
        sins->Cbe = sins->Ceb.transpose();
        sins->qeb = hwa_base::base_att_trans::m2qua(sins->Ceb);
        sins->pos_ecef = Geod2Cart(sins->pos, false);
        sins->qeb = hwa_base::base_att_trans::m2qua(sins->eth.Cen) * sins->qnb;
        sins->eb = wm;
        sins->db = vm + sins->Cbn * sins->eth.gcc;

        std::cerr << "Static Align Successfully!" << std::endl;
        std::cerr << "Initial Cnb: " << std::endl << std::setiosflags(std::ios::fixed) << std::setprecision(3) << sins->Cnb << std::endl;

        if (_num_of_imu_axiliary > 0)
        {
            for (int imui = 0; imui < 1 + _num_of_imu_axiliary; imui++)
            {
                if (imui == 0)
                {
                    sins_mimu[imui]->set_posvel(inipos, Triple::Zero(), sins->qnb);
                }
                else
                {
                    sins_mimu[imui]->set_posvel(inipos + sins->qeb * _p_imui_imu0[imui], Triple::Zero(), sins->qnb * hwa_base::base_att_trans::m2qua(_R_imui_imu0[imui]));
                }
                sins_mimu[imui]->qeb = hwa_base::base_att_trans::m2qua(sins_mimu[imui]->eth.Cen) * sins_mimu[imui]->qnb;
            }
        }
        return true;
    }

    int qt_ins_worker::_merge_init()
    {
        sins->pos = inipos;
        sins->pos_ecef = Geod2Cart(inipos, false);
        sins->eth.Update(inipos, Triple::Zero());
        sins->Xf.block(0, 0, 3, 3) = sins->Ceb;
        sins->Xf.block(0, 3, 3, 1) = sins->ve;
        sins->Xf(3, 3) = 1;
        sins->Xf(4, 4) = 1;

        int icrdx = _param[_param.getParam(_name, hwa_base::par_type::CRD_X, "")].index;
        for (int imui = 0; imui <= _num_of_imu_axiliary; imui++)
        {
            sins_mimu[imui]->qnb = sins->qnb * hwa_base::base_att_trans::m2qua(_R_imui_imu0[imui]);
            sins_mimu[imui]->eth.Update(inipos, Triple::Zero());
            sins_mimu[imui]->Cnb = hwa_base::base_att_trans::q2mat(sins_mimu[imui]->qnb);
            sins_mimu[imui]->pos_ecef = sins->pos_ecef - sins_mimu[imui]->eth.Cen * sins_mimu[imui]->Cnb * _p_imui_imu0[imui];
            sins_mimu[imui]->pos = Cart2Geod(sins_mimu[imui]->pos_ecef, false);
            sins_mimu[imui]->Xf.block(0, 0, 3, 3) = sins_mimu[imui]->Ceb;
            sins_mimu[imui]->Xf.block(0, 3, 3, 1) = sins_mimu[imui]->ve;
            sins_mimu[imui]->Xf(3, 3) = 1;
            sins_mimu[imui]->Xf(4, 4) = 1;

            if (imui > 0 && FuseType == hwa_ins::STACK)
            {
                int icrdx_i = _param[_param.getParam(_name, hwa_base::par_type::CRD_X, "imu" + hwa_base::base_type_conv::int2str(imui))].index;
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
                sins->_global_variance.block(icrdx_i - 1, icrdx_i - 1, 3, 3) = sins->_global_variance.block(icrdx - 1, icrdx - 1, 3, 3) + Ceb_i * p_extr_cov * Ceb_i.transpose();
                sins->_global_variance.block(icrdx_i - 7, icrdx_i - 7, 3, 3) = sins->_global_variance.block(icrdx - 7, icrdx - 7, 3, 3) + Ceb_i * R_extr_cov * Ceb_i.transpose();
            }
        }

        sins->Pk_Sav = sins->_global_variance;
        return 1;
    }

    bool qt_ins_worker::_ins_init()
    {
        Vector v(15);
        int d = 0, itg = 15;
        Triple angle_tmp, v_tmp, pos_tmp, eb_tmp, db_tmp, gscale_tmp, ascale_tmp, inst_att_tmp, lever_gnss_tmp;
        double dk_tmp;
        angle_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_misalignment_std();
        v_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_vel_std();
        pos_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_pos_std();
        eb_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_gyro_std();
        db_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_acce_std();
        v << angle_tmp * hwa_base::glv.deg, v_tmp, pos_tmp, eb_tmp* hwa_base::glv.dph, db_tmp* hwa_base::glv.mg;
        v = v.array().abs2();

        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
        {
            int imui = param_of_sins.getParam(_name, hwa_base::par_type::ATT_X, "imu1");
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
                angle_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_misalignment_std();
                v.block(imui, 0, 3, 1) = angle_tmp * hwa_base::glv.deg;
                v_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_vel_std();
                v.block(imui + 3, 0, 3, 1) = v_tmp;
                pos_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_pos_std();
                v.block(imui + 6, 0, 3, 1) = pos_tmp;
                eb_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_gyro_std();
                v.block(imui + 9, 0, 3, 1) = eb_tmp * hwa_base::glv.dph;
                db_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->initial_acce_std();
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

        sins->Pk = v.array().matrix().asDiagonal();
        SO3 Patt = sins->Pk.block<3, 3>(0, 0);
        sins->Pk.block<3, 3>(0, 0) = sins->eth.Cen * Patt * sins->eth.Cen.transpose();
        sins->_global_variance = sins->Pk;

        angle_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->misalignment_psd();
        v_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->vel_psd();
        pos_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->pos_psd();
        eb_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->gyro_psd();
        db_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->acce_psd();

        sins->gyo_noise = angle_tmp * hwa_base::glv.dpsh;
        sins->acc_noise = v_tmp * hwa_base::glv.mgpsHz;
        sins->ba_noise = db_tmp * hwa_base::glv.mgpsh;
        sins->bg_noise = eb_tmp * hwa_base::glv.dphpsh;

        if (EstimatorType == NORMAL) {
            sins->Qt << angle_tmp * hwa_base::glv.dpsh, v_tmp* hwa_base::glv.mgpsHz, pos_tmp* hwa_base::glv.mpsh, eb_tmp* hwa_base::glv.dphpsh, db_tmp* hwa_base::glv.mgpsh;
        }
        else if (EstimatorType == INEKF) {
            sins->Qt = Vector(nq - 3);
            sins->Qt << angle_tmp * hwa_base::glv.dpsh, v_tmp* hwa_base::glv.mgpsHz, eb_tmp* hwa_base::glv.dphpsh, db_tmp* hwa_base::glv.mgpsh;
        }

        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
        {
            int imui = param_of_sins.getParam(_name, hwa_base::par_type::ATT_X, "imu1");
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                angle_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->misalignment_psd();
                sins->Qt.block(imui, 0, 3, 1) = angle_tmp * hwa_base::glv.dpsh;
                v_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->vel_psd();
                sins->Qt.block(imui + 3, 0, 3, 1) = v_tmp * hwa_base::glv.mgpsHz;
                pos_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->pos_psd();
                sins->Qt.block(imui + 6, 0, 3, 1) = pos_tmp * hwa_base::glv.mpsh;
                eb_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->gyro_psd();
                sins->Qt.block(imui + 9, 0, 3, 1) = eb_tmp * hwa_base::glv.dphpsh;
                db_tmp = dynamic_cast<hwa_set::set_ins*>(gset.get())->acce_psd();
                sins->Qt.block(imui + 12, 0, 3, 1) = db_tmp * hwa_base::glv.mgpsh;
                imui = imui + itg;
            }
        }

        sins->Qt = sins->Qt.array().abs2();
        sins->Qm = sins->Qt.array().matrix().asDiagonal();

        sins->FBTau << 1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
        sins->FBMax << hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF,
            10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph, 500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg;

        //multi-imu
        if (_num_of_imu_axiliary > 0 && FuseType == hwa_ins::STACK)
        {
            int imui = param_of_sins.getParam(_name, hwa_base::par_type::ATT_X, "imu1");
            sins->FBTau.block(imui - d, 0, d, 1) = 1.0 * Vector::Ones(d);
            sins->FBMax.block(imui - d, 0, d, 1) = hwa_base::glv.INF * Vector::Ones(d);
            for (int i = 1; i <= _num_of_imu_axiliary; i++)
            {
                sins->FBTau.block(imui, 0, 3, 1) = Triple(1.0, 1.0, 10.0);
                sins->FBTau.block(imui + 3, 0, 3, 1) = Triple(1.0, 1.0, 1.0);
                sins->FBTau.block(imui + 6, 0, 3, 1) = Triple(1.0, 1.0, 1.0);
                sins->FBTau.block(imui + 9, 0, 3, 1) = Triple(10.0, 10.0, 10.0);
                sins->FBTau.block(imui + 12, 0, 3, 1) = Triple(10.0, 10.0, 10.0);
                sins->FBTau.block(imui + 15, 0, d, 1) = 1.0 * Vector::Ones(d);
                sins->FBMax.block(imui, 0, 3, 1) = Triple(hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF);
                sins->FBMax.block(imui + 3, 0, 3, 1) = Triple(hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF);
                sins->FBMax.block(imui + 6, 0, 3, 1) = Triple(hwa_base::glv.INF, hwa_base::glv.INF, hwa_base::glv.INF);
                sins->FBMax.block(imui + 9, 0, 3, 1) = Triple(3000.0 * hwa_base::glv.dph, 3000.0 * hwa_base::glv.dph, 3000.0 * hwa_base::glv.dph);
                sins->FBMax.block(imui + 9, 0, 3, 1) = Triple(10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph, 10000.0 * hwa_base::glv.dph);
                sins->FBMax.block(imui + 12, 0, 3, 1) = Triple(500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg, 500.0 * hwa_base::glv.mg);
                sins->FBMax.block(imui + 15, 0, d, 1) = hwa_base::glv.INF * Vector::Ones(d);
                imui = imui + itg;
            }
        }
        return true;
    }

    void qt_ins_worker::set_Ft()
    {
        //InEKF
        if (EstimatorType == INEKF) {
            Ft.block(0, 0, 3, 3) = -hwa_base::askew(sins->eth.weie);  // att-att
            Ft.block(3, 0, 3, 3) = hwa_base::askew(sins->eth.Cen * sins->eth.gcc); //vel-att
            Ft.block(0, 9, 3, 3) = -sins->Ceb; //att-eb
            Ft.block(3, 3, 3, 3) = -2 * hwa_base::askew(sins->eth.weie);      // vel-vel
            Ft.block(3, 9, 3, 3) = -hwa_base::askew(sins->ve) * sins->Ceb; //vel-eb
            Ft.block(3, 12, 3, 3) = -sins->Ceb; //vel-db
            Ft.block(6, 9, 3, 3) = -hwa_base::askew(sins->pos_ecef - sins->initial_pos) * sins->Ceb;//pos-eb
            Ft.block(6, 3, 3, 3) = SO3::Identity(); //pos-vel
            Ft.block(6, 0, 3, 3) = hwa_base::askew(sins->eth.Cen * sins->eth.gcc) * sins->nts; //pos-att
            Ft.block(6, 12, 3, 3) = -sins->Ceb * sins->nts; // pos-db

            G.block(0, 0, 3, 3) = sins->Ceb;
            G.block(3, 0, 3, 3) = hwa_base::askew(sins->ve) * sins->Ceb;
            G.block(6, 0, 3, 3) = hwa_base::askew(sins->pos_ecef - sins->initial_pos) * sins->Ceb;
            G.block(6, 3, 3, 3) = sins->Ceb * sins->nts;
            G.block(3, 3, 3, 3) = sins->Ceb;
            G.block(9, 6, 3, 3) = -SO3::Identity();
            G.block(12, 9, 3, 3) = -SO3::Identity();
        }

        if (EstimatorType == NORMAL) {
            Ft.block(0, 0, 3, 3) = -hwa_base::askew(sins->eth.weie);  // att-att
            Ft.block(0, 9, 3, 3) = -sins->Ceb;                        // att-eb
            Ft.block(3, 0, 3, 3) = hwa_base::askew(sins->eth.Cen * sins->fn);  // vel-att
            Ft.block(3, 3, 3, 3) = -2 * hwa_base::askew(sins->eth.weie);      // vel-vel
            Ft.block(3, 12, 3, 3) = sins->Ceb;                              // vel-db
            if (FuseType == hwa_ins::VIRTUAL) {
                Ft.block(3, 9, 3, 3) = sins->jacobian_v_bg;
            }
            Ft.block(6, 3, 3, 3) = SO3::Identity();              // pos-vel
            Ft.block(9, 9, 3, 3) = sins->_tauG.array().matrix().asDiagonal();    // eb-eb
            Ft.block(12, 12, 3, 3) = sins->_tauA.array().matrix().asDiagonal();    // db-db
        }

        if (EstimatorType == NORMAL && FuseType == hwa_ins::STACK)
        {
            for (int imui = 1; imui <= _num_of_imu_axiliary; imui++)
            {
                int iattx_i = param_of_sins.getParam(_name, hwa_base::par_type::ATT_X, "imu" + hwa_base::base_type_conv::int2str(imui));
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

    void qt_ins_worker::time_update_flex(double kfts, int fback, double inflation)
    {
        set_Ft();
        Matrix Fk = Matrix::Zero(sins->_global_variance.rows(), sins->_global_variance.cols());
        Fk.block(0, 0, 15, 15) = Ft.block(0, 0, 15, 15) * kfts;
        if (FuseType == hwa_ins::STACK)
        {
            for (int imui = 1; imui < 1 + _num_of_imu_axiliary; imui++)
            {
                int iattx_i = param_of_sins.getParam(_name, hwa_base::par_type::ATT_X, "imu" + hwa_base::base_type_conv::int2str(imui));
                Fk.block(iattx_i, iattx_i, 15, 15) = Ft.block(iattx_i, iattx_i, 15, 15) * (sins_mimu[imui]->nts);
            }
        }
        Fk = Fk + Matrix::Identity(sins->_global_variance.rows(), sins->_global_variance.cols());

        Matrix Qk = Matrix::Zero(sins->_global_variance.rows(), sins->_global_variance.cols());
        Qk.block(0, 0, sins->Qt.size(), sins->Qt.size()) = (sins->Qt * kfts * inflation).array().matrix().asDiagonal();
        sins->_global_variance = Fk * sins->_global_variance * (Fk.transpose()) + Qk;
        sins->Pk_Sav = sins->_global_variance;

        if (!inswindow.empty()) {
            auto iter = inswindow.rbegin();
            auto& attribute = iter->second.Attribute;
            attribute.Ft = Fk * attribute.Ft;
            attribute.Qt = attribute.Ft * attribute.Qt * attribute.Ft.transpose() + Qk;
        }
    }

    Eigen::Quaterniond qt_ins_worker::V2Q(const Triple& rv)
    {
        constexpr int F1 = 2 * 1;
        constexpr int F2 = F1 * 2 * 2;
        constexpr int F3 = F2 * 2 * 3;
        constexpr int F4 = F3 * 2 * 4;
        constexpr int F5 = F4 * 2 * 5;
        Eigen::Quaterniond qtmp = Eigen::Quaterniond::Identity();
        double c, f, n2 = rv.norm() * rv.norm();
        if (n2 < (hwa_base::glv.PI / 180.0 * hwa_base::glv.PI / 180.0))
        {
            double n4 = n2 * n2;
            c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
            f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
        }
        else
        {
            double n_2 = sqrt(n2) / 2.0;
            c = cos(n_2);
            f = sin(n_2) / n_2 * 0.5;
        }
        return Eigen::Quaterniond(c, f * rv(0), f * rv(1), f * rv(2));
    }

    void qt_ins_worker::Update(const std::map<int, std::vector<Triple>>& _wm_mimu, const std::map<int, std::vector<Triple>>& _vm_mimu, const std::map<int, hwa_ins::ins_scheme>& _scm_mimu)
    {
        if (_wm_mimu.size() == 0) {
            std::cerr << " Error Occur Because of No IMU Data" << std::endl;
            return;
        }
        double nts = _scm_mimu.at(0).ts;
        double nts_2 = nts / 2.0;
        int imu_number = _num_of_imu_axiliary + 1;
        Matrix R = Matrix::Zero(imu_number * 3, 3);
        Vector L_g = Vector::Zero(imu_number * 3);
        std::map<int, hwa_ins::imu> mimu_data;

        //Project the auxiliary IMU on the virtual IMU
        for (int i = 0; i < imu_number; i++) {
            if (!_scm_mimu.at(i).Status) continue;
            mimu_data[i].Update(_wm_mimu.at(i), _vm_mimu.at(i), _scm_mimu.at(i));
            R.block(i * 3, 0, 3, 3) = _R_imui_imu0[i].transpose();
            L_g.block(i * 3, 0, 3, 1) = mimu_data[i].phim;
        }
        Triple virtual_wm = (R.transpose() * R).inverse() * R.transpose() * L_g / nts;

        Vector Temp_L_a = Vector::Zero(imu_number * 3);
        for (int i = 0; i < imu_number; i++) {
            if (!_scm_mimu.at(i).Status) continue;
            Temp_L_a.block(i * 3, 0, 3, 1) = mimu_data[i].dvbm / nts - _R_imui_imu0[i].transpose() * hwa_base::askew(virtual_wm) * hwa_base::askew(virtual_wm) * _p_imui_imu0[i];
        }

        //Null Space Projection
        Matrix temp_matrix = Matrix::Zero(imu_number * 3, 3);
        for (int i = 0; i < imu_number; i++) {
            if (!_scm_mimu.at(i).Status) continue;
            temp_matrix.block(i * 3, 0, 3, 3) = _R_imui_imu0[i].transpose() * hwa_base::askew(_p_imui_imu0[i]);
        }

        Eigen::JacobiSVD<Matrix> svd_helper(temp_matrix, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Matrix A = svd_helper.matrixU().rightCols(
            imu_number * 3 - 3);

        if (temp_matrix == Matrix::Zero(imu_number * 3, 3)) {
            A = Matrix::Identity(imu_number * 3, imu_number * 3);
        }
        Vector L_a = A.transpose() * Temp_L_a;
        Matrix R_a = A.transpose() * R;
        Matrix design_matrix = (R_a.transpose() * R_a).inverse() * R_a.transpose();
        Triple virtual_vm = design_matrix * L_a;

        sins->_imu.phim = virtual_wm * nts;
        sins->_imu.dvbm = virtual_vm * nts;
        sins->obs_wib = sins->_imu.phim / nts;
        sins->obs_fb = sins->_imu.dvbm / nts;
        sins->_imu.phim = sins->Kg.asDiagonal() * sins->_imu.phim - sins->eb * nts;
        sins->_imu.dvbm = sins->Ka.asDiagonal() * sins->_imu.dvbm - sins->db * nts;
        Triple vn1_2 = sins->vn + sins->an * nts_2, pos1_2 = sins->pos + sins->eth.v2dp(vn1_2, nts_2);

        sins->eth.Update(pos1_2, vn1_2);
        sins->wib = sins->_imu.phim / nts;
        sins->fb = sins->_imu.dvbm / nts;
        sins->web = sins->wib - sins->Cbn * sins->eth.wnie;
        sins->wnb = sins->wib - hwa_base::base_quat::conj(sins->qnb * hwa_base::base_att_trans::rv2q(sins->_imu.phim / 2)) * sins->eth.wnin;
        sins->fn = sins->qnb * sins->fb;
        sins->an = hwa_base::base_att_trans::rv2q(-sins->eth.wnin * nts_2) * sins->fn + sins->eth.gcc;
        sins->gcc_b = -sins->Cbn * sins->eth.gcc;
        Triple vn1 = sins->vn + sins->an * nts;
        sins->pos = sins->pos + sins->eth.v2dp(sins->vn + vn1, nts_2);
        sins->vn = vn1;
        sins->qnb = hwa_base::base_att_trans::rv2q(-sins->eth.wnin * nts) * sins->qnb * hwa_base::base_att_trans::rv2q(sins->_imu.phim);
        sins->Cnb = hwa_base::base_att_trans::q2mat(sins->qnb);
        sins->att = hwa_base::base_att_trans::m2att(sins->Cnb);
        sins->Cbn = sins->Cnb.transpose();
        sins->vb = sins->Cbn * sins->vn;
        sins->eth.Update(sins->pos, sins->vn);
        sins->pos_ecef = Geod2Cart(sins->pos, false);
        sins->Ceb = sins->eth.Cen * sins->Cnb;
        sins->Cbe = sins->Ceb.transpose();
        sins->qeb = hwa_base::base_att_trans::m2qua(sins->Ceb);
        sins->qbe = hwa_base::base_att_trans::m2qua(sins->Cbe);
        sins->ve = sins->eth.Cen * sins->vn;
        sins->ae = sins->eth.Cen * sins->an;
        Matrix temp_matrix1 = Matrix::Zero(imu_number * 3, 3);
        for (int i = 0; i < imu_number; i++) {
            temp_matrix1.block(i * 3, 0, 3, 3) = _R_imui_imu0[i].transpose() * hwa_base::askew(virtual_wm) * hwa_base::askew(_p_imui_imu0[i]) + hwa_base::askew(hwa_base::askew(virtual_wm) * _p_imui_imu0[i]);

        }
        sins->jacobian_v_bg = sins->Ceb * design_matrix * A.transpose() * temp_matrix1;
        sins->jacobian_v_etag = sins->jacobian_v_bg;

        //Pre-Integration
        if (!inswindow.empty()) {
            auto iter = inswindow.rbegin();
            auto& attribute = iter->second.Attribute;
            iter->second.ts += nts;
            attribute.delta_v += attribute.delta_q * sins->fb * nts;
            attribute.delta_p += attribute.delta_v * nts + attribute.delta_q * sins->fb * nts * nts / 2.0;
            attribute.jacobian_ba -= SO3(attribute.delta_q) * nts;
            attribute.delta_q *= I2E(hwa_base::base_att_trans::rv2q(sins->_imu.phim));
            attribute.jacobian_bg -= SO3(attribute.delta_q) * nts;
            attribute.jacobian_pos_bg += SO3(attribute.delta_q) * askew(sins->fb * nts * nts * nts / 2.0);
            attribute.jacobian_pos_ba -= attribute.jacobian_ba * nts;
        }

        double time0 = std::round(_scm_mimu.at(0).t / vis_interval) * vis_interval;
        if (vis_size > 0 && Iequal(time0, basetime)) {
            StateAugmentation();
        }

        double time1 = std::round(_scm_mimu.at(0).t / interval) * interval;
        if (Iequal(time1, _scm_mimu.at(0).t)) {
            emit NewAngleVel(time1, sins->wib);
            _OneIns Res(nq, sins->_global_variance, I2E(sins->qeb), sins->pos_ecef, sins->ve, sins->eth.gcc, sins->eth.Cen);
            Res.Attribute.Ft = Matrix::Identity(sins->_global_variance.rows(), sins->_global_variance.cols());
            Res.Attribute.Qt = Matrix::Zero(sins->_global_variance.rows(), sins->_global_variance.cols());
            inswindow[time1] = Res;
            if (inswindow.size() > WindowSize) inswindow.erase(inswindow.begin());
        }
    }

    int qt_ins_worker::_cam_feedback(const Vector& dx)
    {
        for (auto i : cam_list) {
            if (CamAttr[i]->estimate_extrinsic)
            {
                int index = _param.getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "cam" + std::to_string(i));
                if (index < 0) std::cerr << "Extrinsic Update Error!" << std::endl;
                Triple att_dx = Triple(dx(index), dx(index + 1), dx(index + 2));
                Triple crd_dx = Triple(dx(index + 3), dx(index + 4), dx(index + 5));
                Triple crd_dx1;
                hwa_base::base_quat dq_cam = hwa_base::base_att_trans::rv2q(-att_dx);    // R_i_c = (I+theta^) * R_i_c ; R_c_i = R_c_i * (I - theta);
                Eigen::Quaterniond _dq_ext;
                _dq_ext.w() = dq_cam.q0; _dq_ext.x() = dq_cam.q1; _dq_ext.y() = dq_cam.q2; _dq_ext.z() = dq_cam.q3;
                _dq_ext.normalize();

                CamAttr[i]->R_cam0_imu = CamAttr[i]->R_cam0_imu * _dq_ext.toRotationMatrix();
                CamAttr[i]->t_cam0_imu += crd_dx;    // t_c_i = t_c_i + delta_t;
                CamAttr[i]->T_cam0_imu.linear() = CamAttr[i]->R_cam0_imu;
                CamAttr[i]->T_cam0_imu.translation() = CamAttr[i]->t_cam0_imu;
                CamAttr[i]->T_cam0_imu = CamAttr[i]->T_cam0_imu;
                CamAttr[i]->t_cam0_imu = CamAttr[i]->t_cam0_imu;
                CamAttr[i]->R_cam0_imu = CamAttr[i]->R_cam0_imu;
                CamAttr[i]->T_cam1_imu = CamAttr[i]->T_cam0_imu * CamAttr[i]->T_cam0_cam1.inverse();
                CamAttr[i]->R_cam1_imu = CamAttr[i]->T_cam1_imu.linear();
                CamAttr[i]->t_cam1_imu = CamAttr[i]->T_cam1_imu.translation();

            }
            if (CamAttr[i]->estimate_t)
            {
                int index = _param.getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam" + std::to_string(i));
                if (index < 0) std::cerr << "Extrinsic dt Update Error!" << std::endl;

                CamAttr[i]->dt_cam0_imu = CamAttr[i]->dt_cam0_imu - dx(index);
            }
        }

        Eigen::Matrix4d ImuXf = Eigen::Matrix4d::Identity();
        SO3 Rotation = SO3::Identity();
        auto imu_state_iter = imu_states.begin();
        for (int i = 0; i < imu_states.size();
            ++i, ++imu_state_iter)
        {
            ImuXf = Eigen::Matrix4d::Identity();
            Rotation = SO3::Identity();
            std::string _camid = std::to_string(imu_state_iter->second.ID);

            int idx = _param.getParam(_name, hwa_base::par_type::CAM_ATT_X, _camid);
            if (idx < 0) std::cerr << "cam_feedback() idx wrong!" << std::endl;

            if (EstimatorType == NORMAL) {
                Triple att_dx = Triple(dx(idx + 0), dx(idx + 1), dx(idx + 2));
                Triple crd_dx = Triple(dx(idx + 3), dx(idx + 4), dx(idx + 5));

                hwa_base::base_quat dq_cam = hwa_base::base_att_trans::rv2q(att_dx);
                Eigen::Quaterniond _dq_cam;
                _dq_cam.w() = dq_cam.q0; _dq_cam.x() = dq_cam.q1; _dq_cam.y() = dq_cam.q2; _dq_cam.z() = dq_cam.q3;
                _dq_cam.normalize();
                imu_state_iter->second.orientation = _dq_cam * imu_state_iter->second.orientation;
                imu_state_iter->second.position -= crd_dx;
            }
            else {
                ImuXf.block(0, 0, 3, 3) = imu_state_iter->second.orientation.normalized().toRotationMatrix();
                ImuXf.block(0, 3, 3, 1) = imu_state_iter->second.position;
                int K = 1;
                Matrix X = Matrix::Identity(3 + K, 3 + K);
                SO3 R;
                SO3 Jl;
                Triple w = dx.segment<3>(idx);
                double theta = w.norm();
                SO3 I = SO3::Identity();
                if (theta < 1e-10) {
                    R = I;
                    Jl = I;
                }
                else {
                    SO3 A = hwa_base::askew(w);
                    double theta2 = theta * theta;
                    double stheta = sin(theta);
                    double ctheta = cos(theta);
                    double oneMinusCosTheta2 = (1 - ctheta) / (theta2);
                    SO3 A2 = A * A;
                    R = I + (stheta / theta) * A + oneMinusCosTheta2 * A2;
                    Jl = I + oneMinusCosTheta2 * A + ((theta - stheta) / (theta2 * theta)) * A2;
                }
                X.block<3, 3>(0, 0) = R;
                for (int i = 0; i < K; ++i) {
                    X.block<3, 1>(0, 3 + i) = dx.segment<3>(idx + 3 + 3 * i);
                }
                ImuXf = X.colPivHouseholderQr().solve(Matrix::Identity(3 + K, 3 + K)) * ImuXf;
                Rotation = ImuXf.block(0, 0, 3, 3);
                imu_state_iter->second.orientation = Rotation;
                imu_state_iter->second.orientation.normalize();
                imu_state_iter->second.position = ImuXf.block(0, 3, 3, 1);
            }
        }

        return 1;
    }

    bool qt_ins_worker::_extrinsic_init(int cam_group_id)
    {
        std::lock_guard<std::mutex> lock(ins_mutex);
        CamAttr[cam_group_id]->ex_param_num = 0;
        if (!CamAttr[cam_group_id]->estimate_extrinsic && !CamAttr[cam_group_id]->estimate_t) return true;
        if (CamAttr[cam_group_id]->estimate_extrinsic)
        {
            CamAttr[cam_group_id]->ex_param_num += 6;
            base_allpar param_extended;
            Matrix Qx_extended;
            int before_parNumber = _param.parNumber();
            for (int i = 0; i < _param.parNumber(); i++)
            {
                param_extended.addParam(_param[i]);
            }

            for (int ipar = (int)hwa_base::par_type::EX_CAM_ATT_X, i = 0; ipar <= (int)hwa_base::par_type::EX_CAM_ATT_Z; ipar++, i++)
            {
                base_par attr(_name, hwa_base::par_type(ipar), ipar, "cam" + std::to_string(cam_group_id));
                param_extended.addParam(attr);
            }
            for (int ipar = (int)hwa_base::par_type::EX_CAM_CRD_X, i = 0; ipar <= (int)hwa_base::par_type::EX_CAM_CRD_Z; ipar++, i++)
            {
                base_par pos(_name, hwa_base::par_type(ipar), ipar, "cam" + std::to_string(cam_group_id));
                param_extended.addParam(pos);
            }
            int ibefore = param_extended.getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "cam" + std::to_string(cam_group_id));
            Matrix Qx_tmp = Matrix::Zero(param_extended.parNumber(), param_extended.parNumber());
            Qx_tmp.block(0, 0, before_parNumber, before_parNumber) = sins->_global_variance;
            Qx_extended = Qx_tmp;
            for (int i = ibefore, j = 0; j < 3; i++, j++)
            {
                Qx_extended(i, i) = CamAttr[cam_group_id]->initial_cam_extrinsic_rotation_cov(j) * hwa_base::glv.deg * hwa_base::glv.deg;
            }
            for (int i = ibefore + 3, j = 0; j < 3; i++, j++)
            {
                Qx_extended(i, i) = CamAttr[cam_group_id]->initial_cam_extrinsic_translation_cov(j);
            }
            _param = param_extended;
            sins->_global_variance = Qx_extended;
            _param.reIndex();
        }

        if (CamAttr[cam_group_id]->estimate_t)
        {
            CamAttr[cam_group_id]->ex_param_num += 1;
            base_allpar param_extended;
            int before_parNumber = _param.parNumber();
            for (int i = 0; i < _param.parNumber(); i++)
            {
                param_extended.addParam(_param[i]);
            }
            base_par tc(_name, hwa_base::par_type((int)hwa_base::par_type::EXTRINSIC_T), (int)hwa_base::par_type::EXTRINSIC_T, "cam" + std::to_string(cam_group_id));
            param_extended.addParam(tc);
            int ibefore = param_extended.getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam" + std::to_string(cam_group_id));
            Matrix Qx_tmp = Matrix::Zero(param_extended.parNumber(), param_extended.parNumber());
            Qx_tmp.block(0, 0, before_parNumber, before_parNumber) = sins->_global_variance;
            sins->_global_variance = Qx_tmp;
            sins->_global_variance(ibefore, ibefore) = CamAttr[cam_group_id]->initial_cam_t_cov;
            _param = param_extended;
            _param.reIndex();
        }
        return true;
    }

    void qt_ins_worker::StateAugmentation()
    {
        Triple BLH = sins->pos;
        Triple XYZ = Geod2Cart(BLH, false);
        const SO3& R_i_n = hwa_base::base_att_trans::q2mat(sins->qnb);
        SO3 R_n_i = R_i_n.transpose();
        SO3 R_n_e = hwa_base::Cen(BLH);
        SO3 R_e_i = R_n_i * R_n_e.transpose();

        frame_count = static_cast<int>(round(basetime / vis_interval));
        imu_states[basetime].ID = frame_count;
        imu_states[basetime].orientation = R_e_i.transpose();
        imu_states[basetime].position = XYZ - initial_pos_ecef;

        std::string cam_id = std::to_string(frame_count);
        base_par att_x_par(_name, hwa_base::par_type::CAM_ATT_X, _param.parNumber() + 1, cam_id);
        _param.addParam(att_x_par);
        base_par att_y_par(_name, hwa_base::par_type::CAM_ATT_Y, _param.parNumber() + 1, cam_id);
        _param.addParam(att_y_par);
        base_par att_z_par(_name, hwa_base::par_type::CAM_ATT_Z, _param.parNumber() + 1, cam_id);
        _param.addParam(att_z_par);
        base_par crd_x_par(_name, hwa_base::par_type::CAM_CRD_X, _param.parNumber() + 1, cam_id);
        _param.addParam(crd_x_par);
        base_par crd_y_par(_name, hwa_base::par_type::CAM_CRD_Y, _param.parNumber() + 1, cam_id);
        _param.addParam(crd_y_par);
        base_par crd_z_par(_name, hwa_base::par_type::CAM_CRD_Z, _param.parNumber() + 1, cam_id);
        _param.addParam(crd_z_par);

        size_t old_rows = sins->_global_variance.rows();
        size_t old_cols = sins->_global_variance.cols();

        Matrix J = Matrix::Zero(6, old_cols);
        J.block<3, 3>(0, 0) = SO3::Identity();
        J.block<3, 3>(3, 6) = SO3::Identity();

        sins->_global_variance.conservativeResize(old_rows + 6, old_cols + 6);
        const Matrix P11 = sins->_global_variance.block(0, 0, old_rows, old_cols);
        sins->_global_variance.block(old_rows, 0, 6, old_cols) = J * P11;
        sins->_global_variance.block(0, old_cols, old_rows, 6) = sins->_global_variance.block(old_rows, 0, 6, old_cols).transpose();
        sins->_global_variance.block<6, 6>(old_rows, old_cols) = J * P11 * J.transpose();

        for (int i = 0; i < sins->_global_variance.rows(); i++)
        {
            assert(sins->_global_variance(i, i) > 0);
        }
        sins->_global_variance.block<3, 3>(old_rows, old_cols) += Matrix::Identity(3, 3) * 1e-12;
        sins->_global_variance.block<3, 3>(old_rows + 3, old_cols + 3) += Matrix::Identity(3, 3) * 1e-10;

        if (imu_states.size() > vis_size) PruneOld();
    }

    void qt_ins_worker::PruneOld()
    {
        std::string string_id = std::to_string(imu_states.begin()->second.ID);
        Matrix tmp_Qx = sins->_global_variance;

        int idx_att_x = _param.getParam(_name, hwa_base::par_type::CAM_ATT_X, string_id);
        hwa_base::Matrix_remRC(tmp_Qx, _param[idx_att_x].index, _param[idx_att_x].index);
        _param.delParam(idx_att_x);
        _param.reIndex();

        int idx_att_y = _param.getParam(_name, hwa_base::par_type::CAM_ATT_Y, string_id);
        hwa_base::Matrix_remRC(tmp_Qx, _param[idx_att_y].index, _param[idx_att_y].index);
        _param.delParam(idx_att_y);
        _param.reIndex();

        int idx_att_z = _param.getParam(_name, hwa_base::par_type::CAM_ATT_Z, string_id);
        hwa_base::Matrix_remRC(tmp_Qx, _param[idx_att_z].index, _param[idx_att_z].index);
        _param.delParam(idx_att_z);
        _param.reIndex();

        int idx_crd_x = _param.getParam(_name, hwa_base::par_type::CAM_CRD_X, string_id);
        hwa_base::Matrix_remRC(tmp_Qx, _param[idx_crd_x].index, _param[idx_crd_x].index);
        _param.delParam(idx_crd_x);
        _param.reIndex();

        int idx_crd_y = _param.getParam(_name, hwa_base::par_type::CAM_CRD_Y, string_id);
        hwa_base::Matrix_remRC(tmp_Qx, _param[idx_crd_y].index, _param[idx_crd_y].index);
        _param.delParam(idx_crd_y);
        _param.reIndex();

        int idx_crd_z = _param.getParam(_name, hwa_base::par_type::CAM_CRD_Z, string_id);
        hwa_base::Matrix_remRC(tmp_Qx, _param[idx_crd_z].index, _param[idx_crd_z].index);
        _param.delParam(idx_crd_z);
        _param.reIndex();

        sins->_global_variance = tmp_Qx;
        imu_states.erase(imu_states.begin());
    }

    void qt_ins_worker::feedback(double timestamp)
    {
        int iatt = param_of_sins.getParam(_name, hwa_base::par_type::ATT_X, "");
        auto it = inswindow.find(timestamp);
        if (it == inswindow.end()) return;
        Triple delta_eb = it->second.Xk.block(9, 0, 3, 1);
        Triple delta_db = it->second.Xk.block(12, 0, 3, 1);
        it->second.qeb = V2Q(it->second.Xk.block(0, 0, 3, 1)) * it->second.qeb;
        it->second.ve = it->second.ve - it->second.Xk.block(3, 0, 3, 1);
        it->second.pos_ecef = it->second.pos_ecef - it->second.Xk.block(6, 0, 3, 1);

        for (auto iter = std::make_reverse_iterator(it); iter != inswindow.rend(); ++iter) {
            auto next = iter.base()->second;
            auto curr = iter->second;
            auto attribute = curr.Attribute;
            iter->second.qeb = next.qeb * attribute.delta_q.conjugate() * V2Q(attribute.jacobian_bg * delta_eb).conjugate();
            iter->second.ve = next.ve - curr.qeb * (attribute.delta_v + attribute.jacobian_ba * delta_db) - curr.Cen * curr.gcc * interval;
            iter->second.pos_ecef = next.pos_ecef - curr.ve * interval - curr.Cen * curr.gcc * interval * interval / 2.0 -
                curr.qeb * (attribute.delta_p + attribute.jacobian_pos_ba * delta_db + attribute.jacobian_pos_bg * delta_eb);

            int curr_size = iter->second.global_variance.rows();
            iter->second.global_variance.block(0, 0, curr_size, curr_size) = next.global_variance.block(0, 0, curr_size, curr_size);
        }

        for (auto iter = std::next(inswindow.begin()); iter != inswindow.end(); iter++) {
            auto prev = std::prev(iter)->second;
            auto attribute = prev.Attribute;
            iter->second.qeb = prev.qeb * V2Q(attribute.jacobian_bg * delta_eb) * attribute.delta_q;
            iter->second.ve = prev.ve + prev.qeb * (attribute.delta_v + attribute.jacobian_ba * delta_db) + prev.Cen * prev.gcc * interval;
            iter->second.pos_ecef = prev.pos_ecef + prev.ve * interval + prev.Cen * prev.gcc * interval * interval / 2.0 +
                prev.qeb * (attribute.delta_p + attribute.jacobian_pos_ba * delta_db + attribute.jacobian_pos_bg * delta_eb);

            int prev_size = prev.global_variance.rows();
            int curr_size = iter->second.global_variance.rows();
            iter->second.global_variance.block(0, 0, prev_size, prev_size) = attribute.Ft * prev.global_variance * attribute.Ft.transpose() + attribute.Qt;

            if (curr_size > prev_size) {
                assert(curr_size - prev_size == 6);
                Matrix J = Matrix::Zero(6, prev_size);
                J.block<3, 3>(0, 0) = SO3::Identity();
                J.block<3, 3>(3, 6) = SO3::Identity();
                const Matrix P11 = iter->second.global_variance.block(0, 0, prev_size, prev_size);
                iter->second.global_variance.block(prev_size, 0, 6, prev_size) = J * P11;
                iter->second.global_variance.block(0, prev_size, prev_size, 6) = sins->_global_variance.block(prev_size, 0, 6, prev_size).transpose();
                iter->second.global_variance.block<6, 6>(prev_size, prev_size) = J * P11 * J.transpose();
            }
        }
        auto LastIns = inswindow.rbegin();
        auto prev = LastIns->second;
        auto attribute = prev.Attribute;

        sins->eb = sins->eb + delta_eb;
        sins->db = sins->db + delta_db;
        sins->qeb = E2I(prev.qeb * V2Q(attribute.jacobian_bg * delta_eb) * attribute.delta_q);
        sins->ve = prev.ve + prev.qeb * (attribute.delta_v + attribute.jacobian_ba * delta_db) + prev.Cen * prev.gcc * prev.ts;
        sins->pos_ecef = prev.pos_ecef + prev.ve * prev.ts + prev.Cen * prev.gcc * prev.ts * prev.ts / 2.0 +
            prev.qeb * (attribute.delta_p + attribute.jacobian_pos_ba * delta_db + attribute.jacobian_pos_bg * delta_eb);
        sins->_global_variance = attribute.Ft * prev.global_variance * attribute.Ft.transpose() + attribute.Qt;

        sins->Ceb = hwa_base::base_att_trans::q2mat(sins->qeb);
        sins->pos = Cart2Geod(sins->pos_ecef, false);
        sins->eth.Update(sins->pos, sins->vn);
        sins->vn = sins->eth.Cne * sins->ve;
        sins->qnb = hwa_base::base_att_trans::m2qua(sins->eth.Cne) * sins->qeb;
        sins->att = hwa_base::base_att_trans::q2att(sins->qnb);
        sins->Cnb = hwa_base::base_att_trans::q2mat(sins->qnb);
        sins->Cbn = sins->Cnb.transpose();
    }

    void qt_ins_worker::ZUPT_Update(double timestamp) {
        auto iter = inswindow.find(timestamp);
        if (iter == inswindow.end()) return;
        inswindow.erase(inswindow.begin(), iter);
        int n = sins->_global_variance.rows();
        Matrix Hk = Matrix::Zero(3, n);
        Vector Zk = Vector::Zero(3, 1);
        Matrix Rk = Matrix::Identity(3, 3) * ZUPT_Noise;
        int idx_vel_x = _param.getParam(_name, hwa_base::par_type::VEL_X, "");
        Hk.block(0, idx_vel_x, 3, 3) = Matrix::Identity(3, 3);
        Zk = sins->ve;
        updater._meas_update_ekf(Hk, Zk, Rk, iter->second.Xk, iter->second.global_variance);
        feedback(timestamp);
    }

    void qt_ins_worker::UpdateError() {
        _OneBatch First = EmitQueue.front();

        double Timestamp = First.Time;
        auto iter = inswindow.find(Timestamp);
        if (iter == inswindow.end()) {
            if (Timestamp < inswindow.begin()->first) {
                EmitQueue.pop();
                if (EmitQueue.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * (inswindow.begin()->first - Timestamp))));
                else if (EmitQueue.back().Time < inswindow.begin()->first) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * (inswindow.begin()->first - EmitQueue.back().Time))));
                    while (!EmitQueue.empty()) EmitQueue.pop();
                }
                return;
            }
            else if (Timestamp > inswindow.rbegin()->first) {
                return;
            }
        }
        EmitQueue.pop();
        Triple t_t_r = First.T;
        Triple t_t_e = t_r_e + R_r_e * t_t_r;
        SO3 resM = R_r_e * SO3(First.Q) * R_t_i.transpose() * SO3(iter->second.qeb).transpose();
        Eigen::Quaterniond resQ = M2Q(resM);

        try {
            Matrix Hk = Matrix::Zero(6, iter->second.global_variance.rows());
            Vector Zk = Vector::Zero(6, 1);
            Matrix Rk = Matrix::Identity(6, 6);
            Zk.block(0, 0, 3, 1) = 2.0 * Triple(resQ.x(), resQ.y(), resQ.z());
            Zk.block(3, 0, 3, 1) = t_t_e - (iter->second.pos_ecef + iter->second.qeb * t_t_i);
            Rk.block(0, 0, 3, 3) *= Tracker_Rot_noise;
            Rk.block(3, 3, 3, 3) *= Tracker_Trans_noise;
            Hk.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
            Hk.block(3, 0, 3, 3) = askew(iter->second.qeb * t_t_i);
            Hk.block(3, 6, 3, 3) = -Matrix::Identity(3, 3);
            updater._meas_update_ekf(Hk, Zk, Rk, iter->second.Xk, iter->second.global_variance);
            feedback(Timestamp);
            return;
        }
        catch (...) {
            std::cerr << " Tracker Update Failed";
            throw;
        }
    }

    void qt_ins_worker::AcceptSharedCamAttr(std::shared_ptr<hwa_vis::vis_imgproc_base> _CamAttr, int _ID) {
        CamAttr[_ID] = _CamAttr;
        _extrinsic_init(_ID);
    }

    void qt_ins_worker::Accept_tracker(_OneBatch Res) {
        EmitQueue.push(Res);
        if (!aligned) return;
        if (!aligned_track) {
            align_tracker();
            aligned_track = true;
            return;
        }
        UpdateError();
    }

    void qt_ins_worker::SetVisMeas(int cam_id, double timestamp, Matrix Hk, Vector Zk) {
        int ID = int(timestamp / vis_interval);
        int cols = Hk.cols();
        int rows = Hk.rows();
        int Vsize = (cols - nq - CamAttr[cam_id]->ex_param_num) / 6;
        int Gsize = sins->_global_variance.rows();
        MSCKF_Hk = Matrix::Zero(rows, Gsize);
        MSCKF_Zk = Zk;
        if (CamAttr[cam_id]->ex_param_num > 0) {
            if (CamAttr[cam_id]->estimate_extrinsic) {
                int idx_att_ex = _param.getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "cam" + std::to_string(cam_id));
                MSCKF_Hk.block(0, idx_att_ex, rows, 6) = Hk.block(0, 0, rows, 6);
            }
            if (CamAttr[cam_id]->estimate_t) {
                int idx_att_ext = _param.getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam" + std::to_string(cam_id));
                MSCKF_Hk.block(0, idx_att_ext, rows, 1) = Hk.block(0, CamAttr[cam_id]->ex_param_num - 1, rows, 1);
            }
        }
        int idx_att_x = _param.getParam(_name, hwa_base::par_type::CAM_ATT_X, std::to_string(ID));
        int window_size = idx_att_x - nq;
        for (auto i : cam_list) {
            window_size -= CamAttr[i]->ex_param_num;
        }
        window_size /= 6;
        MSCKF_Hk.block(0, idx_att_x - 6 * window_size, rows, 6 * window_size) = Hk.block(0, cols - 6 * window_size, rows, 6 * window_size);
        MSCKF_Rk = Matrix::Identity(Zk.size(), Zk.size()) * VIS_Noise;
    }

    void qt_ins_worker::Accept_vis(int cam_id, bool isStatic, double timestamp, const Matrix Hk1, const Vector Zk1, const Matrix Hk2, const Vector Zk2) {
        if (!aligned) return;
        if (isStatic) {
            std::cerr << timestamp << ": ZUPT Update" << std::endl;
            ZUPT_Update(timestamp);
            return;
        }
        auto iter = inswindow.find(timestamp);
        if (iter == inswindow.end()) return;

        int cols1 = Hk1.cols(), cols2 = Hk2.cols();
        int rows1 = Hk1.rows(), rows2 = Hk2.rows();
        if (rows1 == 0 && rows2 == 0) {
            std::cerr << timestamp << ": No Cam Update" << std::endl;
        }
        else if (rows1 > 0 && rows2 == 0) {
            SetVisMeas(cam_id, timestamp, Hk1, Zk1);
            updater._meas_update_ekf(MSCKF_Hk, MSCKF_Zk, MSCKF_Rk, iter->second.Xk, iter->second.global_variance);
            feedback(timestamp);
        }
        else if (rows1 == 0 && rows2 > 0) {
            SetVisMeas(cam_id, timestamp, Hk2, Zk2);
            updater._meas_update_ekf(MSCKF_Hk, MSCKF_Zk, MSCKF_Rk, iter->second.Xk, iter->second.global_variance);
            feedback(timestamp);
        }
        else {
            Vector Xk;
            SetVisMeas(cam_id, timestamp, Hk1, Zk1);
            updater._meas_update_ekf(MSCKF_Hk, MSCKF_Zk, MSCKF_Rk, Xk, iter->second.global_variance);
            iter->second.Xk += Xk;

            Xk.setZero();
            SetVisMeas(cam_id, timestamp, Hk2, Zk2);
            updater._meas_update_ekf(MSCKF_Hk, MSCKF_Zk, MSCKF_Rk, Xk, iter->second.global_variance);
            iter->second.Xk += Xk;
            feedback(timestamp);
        }
    }

    void qt_ins_worker::InitState() {
        basetime = 0;
        frame_count = 0;
        aligned = false;
        aligned_track = false;
        inswindow.clear();
        sins = std::make_shared<hwa_ins::ins_obj>(gset.get());
        sins->_mimu._num_of_imu_axiliary = _num_of_imu_axiliary;
        sins->_mimu._p_imui_imu0 = _p_imui_imu0;
        sins->_mimu._R_imui_imu0 = _R_imui_imu0;
        init_par();
    }

    void qt_ins_worker::oneCycle() {
        if (stop.load()) {
            for (int i = 0; i < num_of_ins; ++i) {
                readers[i]->stop();
            }
            for (auto& t : threads) if (t.joinable()) t.join();
            cycleTimer->stop();
            thread()->quit();
            return;
        }
        _wm.clear();
        _vm.clear();
        if (!readers[0]->GetImuTime(basetime)) return;
        for (int i = 0; i < num_of_ins; ++i) {
            Triple gyo, acc;
            _ins_scheme[i].Status = false;
            _ins_scheme[i].ts = 0;
            if (!readers[i]->isOpen()) continue;
            if (!readers[i]->Iget(basetime, gyo, acc)) continue;
            _wm[i].push_back(gyo);
            _vm[i].push_back(acc);
            _ins_scheme[i].t = basetime;
            _ins_scheme[i].ts += ts;
            _ins_scheme[i].Status = true;
            readers[i]->Ipop();
        }
        if (!aligned) {
            if (!_ins_scheme[0].Status) return;
            for (int i = 0; i < _vm[0].size(); i++) {
                vm_align.push_back(_vm[0][i]);
                wm_align.push_back(_wm[0][i]);
            }
            if (!align_static()) return;
            aligned = true;
            _ins_init();
            _merge_init();
        }
        Update(_wm, _vm, _ins_scheme);
        time_update_flex(ts, 0, inflation);

        double time0 = std::round(basetime / vis_interval) * vis_interval;
        if (Iequal(time0, basetime)) {
            emit NewIns(imu_states, sins->_global_variance);
        }
        if (Iequal(floor1w(basetime), floor2w(basetime))) {
            emit NewPose(basetime, Eigen::Quaterniond(sins->Ceb), sins->pos_ecef);
        }
    }

    void qt_ins_worker::startWork() {
        if (m_running) return;
        m_running = true;
        for (int i = 0; i < num_of_ins; ++i) {
            if (!readers[i]->start()) continue;
            _ins_scheme[i] = hwa_ins::ins_scheme();
            threads.emplace_back(&ins_reader::readThread, readers[i].get());
        }
        InitState();
        if (!readers[0]->isOpen()) {
            std::cerr << "No Core IMU !" << std::endl;
            return;
        };
        cycleTimer = new QTimer(this);
        connect(cycleTimer, &QTimer::timeout, this, &qt_ins_worker::oneCycle);
        cycleTimer->start(0);
    }

    void qt_ins_worker::stopWork()
    {
        m_running = false;
        stop = true;
    }

}