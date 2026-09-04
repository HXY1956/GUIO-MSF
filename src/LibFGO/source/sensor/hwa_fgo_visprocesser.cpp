// DBoW2 / boost headers must be processed before windows.h
#include "hwa_pg_posegraph.h"
#include "hwa_fgo_visprocesser.h"
#include "hwa_set_ign.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_fgo {
    visprocesser::visprocesser(const baseprocesser& B, int ID, base_data* data) : baseprocesser(B, VIS_NODE), cam_group_id(ID),
        vis_base(_gset.get(), ID), imgdata(dynamic_cast<vis_data*>(data)){
        beg.from_secs(dynamic_cast<set_vis*>(_gset.get())->start(cam_group_id));
        end.from_secs(dynamic_cast<set_vis*>(_gset.get())->end(cam_group_id));
        ric[0] = R_cam0_imu;
        tic[0] = t_cam0_imu;
        if (NUM_OF_CAM > 1) {
            ric[1] = R_cam0_imu * R_cam0_cam1.transpose();
            tic[1] = t_cam0_imu - ric[1] * t_cam0_cam1;
        }
        _fgo_info->_para_td[0][0] = 0.0;
		double FOCAL_LENGTH = dynamic_cast<set_vis*>(_gset.get())->cam0_intrinsics(cam_group_id)[0];
        ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
        ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
        TimeStamp = beg;

        _init_posegraph();
    };

    visprocesser::visprocesser(std::shared_ptr<set_base> gset, std::string site, int ID, base_log spdlog, base_data* data, base_time _beg, base_time _end) : baseprocesser(gset, spdlog, site, VIS_NODE, _beg, _end),
        cam_group_id(ID), vis_base(gset.get(), ID), imgdata(dynamic_cast<vis_data*>(data))
    {
        beg.from_secs(dynamic_cast<set_vis*>(_gset.get())->start(cam_group_id));
        end.from_secs(dynamic_cast<set_vis*>(_gset.get())->end(cam_group_id));
        ric[0] = R_cam0_imu;
        tic[0] = t_cam0_imu;
        if (NUM_OF_CAM > 1) {
            ric[1] = R_cam0_imu * R_cam0_cam1.transpose();
            tic[1] = t_cam0_imu - ric[1] * t_cam0_cam1;
        }
        _fgo_info->_para_td[0][0] = 0.0;
        double FOCAL_LENGTH = dynamic_cast<set_vis*>(_gset.get())->cam0_intrinsics(cam_group_id)[0];
        ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
        ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
        TimeStamp = beg;

        _init_posegraph();
    };

    void visprocesser::_init_posegraph()
    {
        if (!dynamic_cast<set_vis*>(_gset.get())->loop_closure(cam_group_id))
            return;

		auto img_proc_ptr = imgproc.get();

        posegraph_ = std::make_unique<hwa_pg::PGPoseGraph>();
        posegraph_->setCamera(R_cam0_imu, t_cam0_imu,
                              cam0_intrinsics, img_proc_ptr->cam0_distortion_coeffs, distortion2str(img_proc_ptr->cam0_distortion_model));
        posegraph_->setVocabularyPath(dynamic_cast<set_vis*>(_gset.get())->vocabulary(cam_group_id));
        posegraph_->setBriefPattern(dynamic_cast<set_vis*>(_gset.get())->brief_pattern(cam_group_id));
        std::cout << "[PG] visworker pose graph + loop closure enabled" << std::endl;
    }

    visprocesser::~visprocesser() {
        if (TimeCostDebugOutFile.is_open()) TimeCostDebugOutFile.close();
    };

    std::string camstate_id2str(const hwa_vis::CamStateIDType& id)
    {
        return std::to_string(id);
    }

    bool visprocesser::_time_valid(base_time inst) {
        double dtime;
        double insdtime = inst.sow() + inst.dsec();
        if (imgdata->load(insdtime, dtime, get_curr_imgpath(), _shm->delay)) {
            TimeStamp = base_time(TimeStamp.gwk(), dtime);
            return true;
        }
            
        return false;
    }

    bool visprocesser::load_data() {
        if (imgdata == nullptr) return false;
        double img_t = TimeStamp.sow() + TimeStamp.dsec();
        try {
            switch (processer) {
            case CPU: {
                auto iter_cpu = dynamic_cast<vis_imgproc<cv::Mat>*>(imgproc.get());
                if (!stereo) {
                    iter_cpu->imageGroup.first = img_t;
                    iter_cpu->imageGroup.second.first = cv::imread(get_curr_imgpath().img0_path, 0);
                }
                else {
                    iter_cpu->imageGroup.first = img_t;
                    iter_cpu->imageGroup.second.first = cv::imread(get_curr_imgpath().img0_path, 0);
                    iter_cpu->imageGroup.second.second = cv::imread(get_curr_imgpath().img1_path, 0);
                }
                break;
            }

            case GPU: {
                auto iter_gpu = dynamic_cast<vis_imgproc<cv::cuda::GpuMat>*>(imgproc.get());
                if (!stereo) {
                    iter_gpu->imageGroup.first = img_t;
                    iter_gpu->imageGroup.second.first.upload(cv::imread(get_curr_imgpath().img0_path, 0));
                }
                else {
                    iter_gpu->imageGroup.first = img_t;
                    iter_gpu->imageGroup.second.first.upload(cv::imread(get_curr_imgpath().img0_path, 0));
                    iter_gpu->imageGroup.second.second.upload(cv::imread(get_curr_imgpath().img1_path, 0));
                }
                break;
            }
            }
        }
        catch(...) {
			std::cerr << "Error loading image at time: " << img_t << std::endl;
            return false;
        }
        return true;
	}

    void visprocesser::load_imuobs() {
        imgproc->load_imuobs(_shm->t, _sins->_wm, _sins->_vm, _shm->ts);
    }

    bool visprocesser::addFeatureObservations()
    {
        int curr_feature_num = map_server.size();
        int tracked_feature_num = 0;

        for (const auto& feature : _pointcloud.features)
        {
            if (map_server.find(feature.id) == map_server.end())
            {
                hwa_vis::FeatureIDType feature_id = feature.id;
                map_server[feature.id] = hwa_vis::vis_feature(feature.id);
                map_server[feature.id].start_frame_id = cam_state_id;
                if (stereo)
                {
                    map_server[feature.id].observations[cam_state_id].position =
                        Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                            feature.cam1_point.x, feature.cam1_point.y);
                }
                else
                {
                    map_server[feature.id].observations[cam_state_id].position =
                        Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                            0.0, 0.0);
                }

                if (usingstereorecity && stereo)
                    map_server[feature.id].initial_depth = feature.depth;

                if (isKeyFrame)
                    map_server[feature.id].is_KeyFrame = true;

                map_server[feature.id].start_frame = _node_index().back();
                map_server[feature.id].frame_size++;
            }
            else
            {
                if (stereo)
                {
                    map_server[feature.id].observations[cam_state_id].position =
                        Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                            feature.cam1_point.x, feature.cam1_point.y);
                }
                else
                {
                    map_server[feature.id].observations[cam_state_id].position =
                        Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                            0.0, 0.0);
                }
                ++tracked_feature_num;
                map_server[feature.id].frame_size++;
            }

            map_server[feature.id].observations[cam_state_id].cur_td = _fgo_info->_para_td[0][0];
			map_server[feature.id].observations[cam_state_id].velocity = feature.velocity;
        }

        return true;
    }

    void visprocesser::resetDepth() {
        for (auto& it_per_id : map_server) {
            it_per_id.second.inv_depth = -1;
        }
    }

    void visprocesser::initializeDepth()
    {
        for (auto& it_per_id : map_server)
        {
            auto& feature = it_per_id.second;
            int used_num = feature.observations.size();
            int start_frame = feature.start_frame;
            if (!(used_num >= 2 && start_frame < _fgo_info->_window_size - 3))
                continue;

            if (feature.inv_depth > 0)
                continue;

            int node_i = start_frame;
            const auto& node_index = _node_index_copy();
            auto it = std::find(node_index.begin(), node_index.end(), node_i);
            int imu_i = (it != node_index.end())
                ? std::distance(node_index.begin(), it)
                : -1;
            int imu_j = imu_i - 1;

            Eigen::MatrixXd svd_A(2 * feature.observations.size(), 4);
            int svd_idx = 0;

            Eigen::Matrix<double, 3, 4> P0;
            Eigen::Vector3d t0 = _fgo_info->_Ps[node_i] +_fgo_info->_Rs[node_i] * tic[0];
            Eigen::Matrix3d R0 = _fgo_info->_Rs[node_i] * ric[0];
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();

            //std::cout
            //    << "\n========== Feature ==========\n";

            //std::cout
            //    << "feature id = "
            //    << it_per_id.first
            //    << std::endl;


            //std::cout
            //    << "obs size = "
            //    << feature.observations.size()
            //    << std::endl;


            //std::cout
            //    << "start frame = "
            //    << feature.start_frame
            //    << std::endl;


            //std::cout
            //    << "t0 = "
            //    << t0.transpose()
            //    << std::endl;


            //std::cout
            //    << "R0 = \n"
            //    << R0
            //    << std::endl;

            for (auto& it_per_frame : feature.observations)
            {
                imu_j++;
				int node_j = _node_index_copy()[imu_j];
                Eigen::Vector3d t1 = _fgo_info->_Ps[node_j] +_fgo_info->_Rs[node_j] * tic[0];
                Eigen::Matrix3d R1 = _fgo_info->_Rs[node_j] * ric[0];
                Eigen::Vector3d t = R0.transpose() * (t1 - t0);
				Eigen::Matrix3d R = R0.transpose() * R1; // transform from imu_j to imu_i
                Eigen::Matrix<double, 3, 4> P;  // transform from imu_i to imu_j
                P.leftCols<3>() = R.transpose();
                P.rightCols<1>() = -R.transpose() * t;
                Triple point;
                point << it_per_frame.second.position.head<2>(), 1.0;
                Eigen::Vector3d f = point.normalized();
                svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
                svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

                //std::cout
                //    << "\n---- observation ----\n";

                //std::cout
                //    << "imu_j = "
                //    << imu_j
                //    << std::endl;


                //std::cout
                //    << "t1 = "
                //    << t1.transpose()
                //    << std::endl;


                //std::cout
                //    << "relative t = "
                //    << t.transpose()
                //    << std::endl;


                //std::cout
                //    << "relative R=\n"
                //    << R
                //    << std::endl;


                //std::cout
                //    << "feature point = "
                //    << point.transpose()
                //    << std::endl;


                //std::cout
                //    << "bearing f = "
                //    << f.transpose()
                //    << std::endl;


                if (imu_i == imu_j)
                    continue;
            }
            Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
            double svd_method = svd_V[2] / svd_V[3];

            feature.inv_depth = 1.0 /svd_method;

            //std::cout
            //    << "V = "
            //    << svd_V.transpose()
            //    << std::endl;


            //std::cout
            //    << "depth = "
            //    << svd_method
            //    << std::endl;


            //std::cout
            //    << "inv depth = "
            //    << feature.inv_depth
            //    << std::endl;


            //std::cout
            //    << "============================\n";

            if (feature.inv_depth > 10)
            {
                feature.inv_depth = -1;
            }
        }
    // DEBUG(temp): inverse-depth scale statistics (remove after diagnosis)
    {
        static int dbg_call = 0;
        if (dbg_call < 20) {
            int cnt = 0;
            double sum = 0, mn = 1e9, mx = 0;
            for (auto& it : map_server) {
                const double d = it.second.inv_depth;
                if (d > 0) { cnt++; sum += d; if (d < mn) mn = d; if (d > mx) mx = d; }
            }
            if (cnt > 0)
                printf("[VIS] depth stats cnt=%d avg=%.4f min=%.4f max=%.4f\n", cnt, sum / cnt, mn, mx);
            dbg_call++;
        }
    }
    }

    void visprocesser::_write_calib()
    {
        if (!(_fcalib)) 
            return;
        if (_fcalib && fabs(TimeStamp.sod() - int(TimeStamp.sod() * 20.0) / 20.0) < 0.05)
        {
            std::ostringstream os;
            auto q_c_b = hwa_base::base_att_trans::m2qua(imgproc->R_cam0_imu);
            q_c_b.normlize(q_c_b);
            os << std::fixed << std::setprecision(6) << std::setw(20) << q_c_b.q1 << std::setw(15) << q_c_b.q2 << std::setw(15) << q_c_b.q3 << std::setw(15) << q_c_b.q0
                << std::setw(15) << imgproc->t_cam0_imu(0) << std::setw(15) << imgproc->t_cam0_imu(1) << std::setw(15) << imgproc->t_cam0_imu(2);
            os << std::setw(15) << std::setprecision(6) << dt_cam0_imu * 1000.0; //ms
            q_c_b = hwa_base::base_att_trans::m2qua(imgproc->R_cam1_imu);
            q_c_b.normlize(q_c_b);
            os << std::setw(20) << q_c_b.q1 << std::setw(15) << q_c_b.q2 << std::setw(15) << q_c_b.q3 << std::setw(15) << q_c_b.q0
                << std::setw(15) << imgproc->t_cam1_imu(0) << std::setw(15) << imgproc->t_cam1_imu(1) << std::setw(15) << imgproc->t_cam1_imu(2);
            os << std::setw(15) << std::setprecision(6) << dt_cam0_imu * 1000.0; //ms

            os << std::endl;
            _fcalib->write(os.str().c_str(), os.str().size());
        }
        return;
    }

    bool visprocesser::align_vins() {

        if (_fgo_info->rover_count < _fgo_info->_window_size - 1 || _initial_last_time == _fgo_info->_Time[_fgo_info->rover_count - 1]) {
            return 0;
        }
		_initial_last_time = _fgo_info->_Time[_fgo_info->rover_count - 1];

        for (int i = 0; i < _fgo_info->rover_count; i++) {
            cam_states[i] = hwa_vis::CamState(i);
            cam_states[i].orientation = _fgo_info->_Rs[i] * ric[0];
            cam_states[i].position = _fgo_info->_Ps[i] + _fgo_info->_Rs[i] * tic[0];
            cam_states[i].ve = _fgo_info->_Vs[i];
            cam_states[i].time = _fgo_info->_Time[i];
            cam_states[i].pre_integration = _fgo_info->_pre_integrations[i];

            Triple XYZ = cam_states[i].position;
            Triple BLH = Cart2Geod(XYZ, false);

            SO3 R_n_e = hwa_base::Cen(BLH);
            const SO3& R_i_n = R_n_e.transpose() * _fgo_info->_Rs[i];
            const SO3& R_i_c = ric[0].transpose();
            const Triple& t_c_i = tic[0];
            const Triple& t_i_c = -R_i_c * tic[0];

            cam_states[i].qnc = R_i_n * R_i_c.transpose();
            cam_states[i].qcb = R_i_c;
            cam_states[i].Tcb = t_i_c;
            cam_states[i].qbc = R_i_c.transpose();
            cam_states[i].Tbc = t_c_i;
            cam_states[i].R_e_n = R_n_e.transpose();

            base_earth eth;
			eth.Update(BLH, R_n_e.transpose() * cam_states[i].ve);
            cam_states[i].gravity = eth.gcc;
        }

		frame_count = _fgo_info->rover_count;

        if (initialStructure()) {
            align_feedback();
			resetDepth();
            initializeDepth();
            return 1;
        }
        else {
            for (int i = 0; i < _fgo_info->rover_count; i++) {
                _fgo_info->_Bgs[i] = Bgs[i];
            }
            _sins->eb = Bgs[frame_count - 1];
        }

        return 0;
    }

    void visprocesser::align_feedback() {
        _sins->eb = Bgs[frame_count - 1];
        _sins->db = Bas[frame_count - 1];
        _sins->qeb = hwa_base::base_quat(cam_states.rbegin()->second.orientation_b.w(), cam_states.rbegin()->second.orientation_b.x(),
            cam_states.rbegin()->second.orientation_b.y(), cam_states.rbegin()->second.orientation_b.z());
        _sins->ve = cam_states.rbegin()->second.ve;
        _sins->pos_ecef = cam_states.rbegin()->second.position_b;

        _sins->Ceb = hwa_base::base_att_trans::q2mat(_sins->qeb);
        _sins->pos = Cart2Geod(_sins->pos_ecef, false);
        _sins->eth.Update(_sins->pos, _sins->vn);
        _sins->vn = _sins->eth.Cne * _sins->ve;
        _sins->qnb = hwa_base::base_att_trans::m2qua(_sins->eth.Cne) * _sins->qeb;
        _sins->att = hwa_base::base_att_trans::q2att(_sins->qnb);
        _sins->Cnb = hwa_base::base_att_trans::q2mat(_sins->qnb);
        _sins->orientation = _sins->Ceb;
        _sins->velocity = _sins->ve;
        _sins->position = Geod2Cart(_sins->pos, false);

        for (int i = 0; i < _fgo_info->rover_count; i++) {
			_fgo_info->_Rs[i] = cam_states[i].orientation * ric[0].transpose();
			_fgo_info->_Ps[i] = cam_states[i].position - _fgo_info->_Rs[i] * tic[0];
            _fgo_info->_Vs[i] = cam_states[i].ve;
            _fgo_info->_Bas[i] = Bas[i];
            _fgo_info->_Bgs[i] = Bgs[i];
        }

        std::cout << " Refined Gyo Bias: " << _sins->eb.transpose() << std::endl;
        std::cout << " Refined Acc Bias: " << _sins->db.transpose() << std::endl;
        std::cout << " Rotation Matrix Rnb: " << std::endl << _sins->Cnb << std::endl;

        Triple ypr = hwa_vis::vis_base::R2ypr(_sins->Cnb);
        std::cout << "Yaw: " << ypr(0) << "Pitch: " << ypr(1) << "Roll: " << ypr(2) << std::endl;
    }

    int visprocesser::ProcessOneEpoch()
    {
        base_scopedtimer Timer("_processCamEpoch", TimeCostDebugOutFile, imgproc->TimeCostOut);
        _pointcloud = imgproc->ProcessBatch();
        _imgproc_count++;
        cam_state_id = cam_next_id++;
        pre_cam_state_id = cam_state_id - 1;

        cv::Mat frame = imgproc->get_out_img();
        std::string window_name = "Cam Group " + std::to_string(cam_group_id);
        cv::imshow(window_name, frame);
        cv::waitKey(1);

        if (!checkStaticMotion() && keyframeCheck())
        {
            _set_frame_pose();
            addFeatureObservations();
            initializeDepth();
        }
        else {
            return NO_MEAS;
        }

        return VIS_MEAS;
    }

    int visprocesser::currentPGNodeIndex() const
    {
        auto idx = _fgo_info->_Node_index.find(VIS_NODE);
        if (idx == _fgo_info->_Node_index.end() || idx->second.empty())
            return -1;
        return idx->second.back();
    }

    hwa_pg::RawKeyFrame visprocesser::buildRawKeyFrame()
    {
        hwa_pg::RawKeyFrame raw;
        int node = _node_index_copy().back();

        if (_fgo_info->rover_count <= 0 || node != _fgo_info->rover_count - 1)
            return raw;

        raw.time = _fgo_info->_Time[node];
        raw.rot = _fgo_info->_Rs[node];
        raw.pos = _fgo_info->_Ps[node];
        raw.image_path = get_curr_imgpath().img0_path;

        const double fx = cam0_intrinsics(0), fy = cam0_intrinsics(1);
        const double cx = cam0_intrinsics(2), cy = cam0_intrinsics(3);

        std::vector<cv::Point2f> norm_points;

        for (auto& it : map_server)
        {
            auto& f = it.second;
            if (f.inv_depth <= 0) continue;
            auto ob = f.observations.find(cam_state_id);
            if (ob == f.observations.end()) continue;
            int sn = f.start_frame;
            if (sn < 0 || sn >= _fgo_info->rover_count) continue;
            auto first = f.observations.find(f.start_frame_id);
            if (first == f.observations.end()) continue;

            Eigen::Vector3d pc(first->second.position(0) / f.inv_depth,
                first->second.position(1) / f.inv_depth,
                1.0 / f.inv_depth);
            Eigen::Vector3d pb = ric[0] * pc + tic[0];
            Eigen::Vector3d pw = _fgo_info->_Rs[sn] * pb + _fgo_info->_Ps[sn];
            const auto& obs = ob->second.position;

            raw.point_3d.emplace_back((float)pw.x(), (float)pw.y(), (float)pw.z());
            raw.point_2d_norm.emplace_back((float)obs(0), (float)obs(1));
            raw.point_id.push_back((double)it.first);
            norm_points.emplace_back((float)obs(0), (float)obs(1));
        }

        // convert normalized observations to (distorted) pixel coordinates for
        // the window-BRIEF matching locations
        if (!norm_points.empty()) {
            vis_stereo_lk_cpu* imgcpu = dynamic_cast<vis_stereo_lk_cpu*>(imgproc.get());
            if (imgcpu) {
                raw.point_2d_uv = imgcpu->distortPoints(
                    norm_points, imgcpu->_cam0_intrinsics,
                    imgcpu->_cam0_distortion_model, imgcpu->_cam0_distortion_coeffs);
            } else if (vis_stereo_vins_cpu* imgvins =
                       dynamic_cast<vis_stereo_vins_cpu*>(imgproc.get())) {
                raw.point_2d_uv = imgvins->distortPoints(
                    norm_points, imgvins->_cam0_intrinsics,
                    imgvins->_cam0_distortion_model, imgvins->_cam0_distortion_coeffs);
            } else {
                for (const auto& p : norm_points)
                    raw.point_2d_uv.emplace_back((float)(p.x * fx + cx),
                                                 (float)(p.y * fy + cy));
            }
        }
        return raw;
    }

    void visprocesser::updatePoseGraph()
    {
        if (!posegraph_) return;
        hwa_pg::RawKeyFrame raw = buildRawKeyFrame();
        if (raw.image_path.empty() || raw.point_3d.size() < 10)
            return;
        posegraph_->addKeyFrame(raw);

        // The pose-graph worker detects loops asynchronously. When a loop is
        // found it first runs the (slow) 4DoF optimizeGraph and only afterwards
        // publishes the corrected old-frame anchor. Wait (bounded) for that
        // anchor so the loop keyframe cannot slide out of the window while the
        // graph is being optimized; on timeout the loop is skipped.
        if (posegraph_->isReloPending())
        {
            if (!posegraph_->waitForReloReady(1000))
                return;
        }

        // poll the latest relocalization result produced by the worker thread
        double t = 0.0;
        Eigen::Vector3d old_pos;
        Eigen::Matrix3d old_rot;
        std::vector<std::pair<double, Eigen::Vector2d>> obs;
        if (posegraph_->getReloResult(t, old_pos, old_rot, obs) && !obs.empty()) {
            relo_active_ = true;
            relo_time_ = t;
            relo_old_pos_ = old_pos;
            relo_old_rot_ = old_rot;
            relo_obs_ = obs;
            std::cout << "[PG] relocalization armed: matches " << obs.size() << std::endl;
        }
    }

    void visprocesser::_addResidualBlocks(ceres::Problem& problem) {

        ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);

        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(_fgo_info->_para_ex_pose[i], SIZE_POSE, local_parameterization);
            if (!imgproc->estimate_extrinsic)
            {
                problem.SetParameterBlockConstant(_fgo_info->_para_ex_pose[i]);
            }
        }
        if (imgproc->estimate_t) {
            problem.AddParameterBlock(_fgo_info->_para_td[0], 1);
            //problem.SetParameterBlockConstant(_fgo_info->_para_td[0]);
        }

        int f_m_cnt = 0;
        int feature_index = -1;

        //std::cout << setprecision(15) << "ex_pose: " << _fgo_info->_para_ex_pose[0][0] <<" "<< _fgo_info->_para_ex_pose[0][1] << " "
        //    << _fgo_info->_para_ex_pose[0][2] << "\n" << _fgo_info->_para_ex_pose[0][3] << " " << _fgo_info->_para_ex_pose[0][4] << " "
        //    << _fgo_info->_para_ex_pose[0][5] << " "
        //    << _fgo_info->_para_ex_pose[0][6]
        //    << "\n";

        for (auto& it_per_id : map_server)
        {
			auto& feature = it_per_id.second;
            if (feature.inv_depth < 0) continue;

            int used_num = feature.observations.size();
			int start_frame = feature.start_frame;
            if (!(used_num >= 2 && start_frame < _fgo_info->_window_size - 3))
                continue;

            ++feature_index;
            if (feature_index >= 1000) break;
            problem.AddParameterBlock(_fgo_info->_para_feature[feature_index], SIZE_FEATURE);

            int node_i = start_frame;
            const auto& node_index = _node_index_copy();
            auto it = std::find(node_index.begin(), node_index.end(), node_i);
            int imu_i = (it != node_index.end())
                ? std::distance(node_index.begin(), it)
                : -1;
            int imu_j = imu_i - 1;

            Triple pts_i;
			auto feature_first_frame = feature.observations[feature.start_frame_id];
            pts_i << feature_first_frame.position.head(2), 1;

    //        for (auto iter : feature.observations) {
				//std::cout << setprecision(10) << "feature id: " << feature.id << ", frame id: " << iter.first << ", frame pos: " << start_frame++ << ", position: " << iter.second.position.transpose() << std::endl;
    //        }

            for (auto& it_next_frame : feature.observations)
            {
                feature_per_frame& feature_next_frame = it_next_frame.second;
                imu_j++;
                int node_j = _node_index_copy()[imu_j];
                if (imu_i == imu_j)
                {
                    continue;
                }
                Triple pts_j;
                pts_j << feature_next_frame.position.head(2), 1;
                if (imgproc->estimate_t)
                {
                    ProjectionTdFactor* f_td = new ProjectionTdFactor(pts_i, pts_j,
                        feature_first_frame.velocity, feature_next_frame.velocity,
                        feature_first_frame.cur_td, feature_next_frame.cur_td);

                    //std::cout << "feature_first_frame.velocity: " << feature_first_frame.velocity << std::endl;
                    //std::cout << "feature_next_frame.velocity: " << feature_next_frame.velocity << std::endl;
                    //std::cout << "feature_first_frame.cur_td: " << feature_first_frame.cur_td << std::endl;
                    //std::cout << "feature_next_frame.cur_td: " << feature_next_frame.cur_td << std::endl;

                    problem.AddResidualBlock(f_td, loss_function, _fgo_info->_para_pose[node_i], _fgo_info->_para_pose[node_j], _fgo_info->_para_ex_pose[0], _fgo_info->_para_feature[feature_index], _fgo_info->_para_td[0]);
                }
                else
                {
                    ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, _fgo_info->_para_pose[node_i], _fgo_info->_para_pose[node_j], _fgo_info->_para_ex_pose[0], _fgo_info->_para_feature[feature_index]);
                }

                //std::cout << setprecision(10);
                //std::cout << "vis coordinate [" << node_i << "]: " << pts_i.transpose() << std::endl;
                //std::cout << "vis coordinate [" << node_j << "]: " << pts_j.transpose() << std::endl;

                //double cost_save = _fgo_info->cost;
                //std::vector<double> residuals;

                //problem.Evaluate(
                //    ceres::Problem::EvaluateOptions(),
                //    &_fgo_info->cost,
                //    &residuals,
                //    nullptr,
                //    nullptr);

                //std::cout
                //    << std::fixed
                //    << std::setprecision(10)
                //    << "vis reprojection cost [" << node_i <<"," << node_j << "] = "
                //    << _fgo_info->cost - cost_save
                //    << std::endl;

                f_m_cnt++;
            }
        }

        //double cost_save = _fgo_info->cost;
        //std::vector<double> residuals;

        //problem.Evaluate(
        //    ceres::Problem::EvaluateOptions(),
        //    &_fgo_info->cost,
        //    &residuals,
        //    nullptr,
        //    nullptr);

        //std::cout
        //    << std::fixed
        //    << std::setprecision(10)
        //    << "vis reprojection cost: "
        //    << _fgo_info->cost - cost_save
        //    << std::endl;

        _addRelocalizationFactors(problem);
    }

    // Strong prior on the relocalization old-frame pose: the pose is a FREE
    // variable in the window BA (so an unreliable anchor is not silently
    // trusted), but it is pulled toward the pose-graph-corrected anchor that
    // the worker publishes after optimizeGraph.
    struct ReloPosePrior {
        ReloPosePrior(const Eigen::Vector3d& t0, const Eigen::Quaterniond& q0, double s)
            : t0_(t0), q0_(q0), s_(s) {}

        template <typename T>
        bool operator()(const T* const pose, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose);
            // pose storage is [x,y,z,qx,qy,qz,qw]
            Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
            Eigen::Quaternion<T> q0(T(q0_.w()), T(q0_.x()), T(q0_.y()), T(q0_.z()));

            residuals[0] = (t(0) - T(t0_(0))) * T(s_);
            residuals[1] = (t(1) - T(t0_(1))) * T(s_);
            residuals[2] = (t(2) - T(t0_(2))) * T(s_);

            // small-angle rotation residual between the anchor and the pose
            Eigen::Quaternion<T> dq = q0.conjugate() * q;
            residuals[3] = dq.x() * T(2.0 * s_);
            residuals[4] = dq.y() * T(2.0 * s_);
            residuals[5] = dq.z() * T(2.0 * s_);
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector3d& t0,
                                           const Eigen::Quaterniond& q0, double s) {
            return new ceres::AutoDiffCostFunction<ReloPosePrior, 6, 7>(
                new ReloPosePrior(t0, q0, s));
        }

        Eigen::Vector3d t0_;
        Eigen::Quaterniond q0_;
        double s_;
    };

    void visprocesser::_addRelocalizationFactors(ceres::Problem& problem)
    {
        if (!relo_active_ || relo_obs_.empty()) return;

        // the loop (relo) frame = current keyframe that triggered the loop;
        // find it in the sliding window by timestamp (VINS relo_frame_local_index)
        int node = -1;
        for (int i = 0; i < _fgo_info->rover_count; ++i)
        {
            if (std::fabs(_fgo_info->_Time[i] - relo_time_) < 0.01) { node = i; break; }
        }
        if (node < 0)
        {
            // relocalized node already slid out of the window
            relo_active_ = false;
            relo_obs_.clear();
            return;
        }

        // Old (loop) keyframe pose in ECEF, used as a CONSTANT anchor so the
        // sliding-window feature structure is pulled toward the old map
        // observations (VINS FAST_RELOCALIZATION anchors on the old kf pose).
        relo_old_pose_[0] = relo_old_pos_.x();
        relo_old_pose_[1] = relo_old_pos_.y();
        relo_old_pose_[2] = relo_old_pos_.z();
        Eigen::Quaterniond q_old(relo_old_rot_);
        relo_old_pose_[3] = q_old.x();
        relo_old_pose_[4] = q_old.y();
        relo_old_pose_[5] = q_old.z();
        relo_old_pose_[6] = q_old.w();
        problem.AddParameterBlock(relo_old_pose_, SIZE_POSE, new PoseLocalParameterization());
        // Free variable + strong prior anchored at the corrected old-frame pose
        // (published by the worker after optimizeGraph).
        problem.AddResidualBlock(
            ReloPosePrior::Create(
                Eigen::Vector3d(relo_old_pose_[0], relo_old_pose_[1], relo_old_pose_[2]),
                Eigen::Quaterniond(relo_old_pose_[6], relo_old_pose_[3],
                                   relo_old_pose_[4], relo_old_pose_[5]),
                10.0),
            nullptr, relo_old_pose_);

        ceres::LossFunction* loss = new ceres::CauchyLoss(1.0);

        // Same feature-index traversal as _addResidualBlocks so the correct
        // para_Feature slot is used for every matched feature.
        int feature_index = -1;
        int added = 0;
        for (auto& it_per_id : map_server)
        {
            auto& feature = it_per_id.second;
            if (feature.inv_depth < 0) continue;
            int used_num = feature.observations.size();
            int start_frame = feature.start_frame;
            if (!(used_num >= 2 && start_frame < _fgo_info->_window_size - 3))
                continue;
            ++feature_index;
            if (feature_index >= 1000) break;

            // is this feature among the loop matches?
            auto relo_it = std::find_if(relo_obs_.begin(), relo_obs_.end(),
                [&feature](const std::pair<double, Eigen::Vector2d>& m) {
                    return std::fabs(m.first - feature.id) < 0.5; });
            if (relo_it == relo_obs_.end()) continue;
            if (start_frame > node) continue;   // VINS: start_frame <= relo_frame_local_index

            // start-frame normalized observation (same as _addResidualBlocks)
            auto first = feature.observations.find(feature.start_frame_id);
            if (first == feature.observations.end()) continue;
            Triple pts_i;
            pts_i << first->second.position.head(2), 1;
            Eigen::Vector3d pts_j(relo_it->second.x(), relo_it->second.y(), 1.0);

            // reproject the start-frame feature (pose_i + inverse depth) into
            // the old keyframe camera frame and compare with the old observation
            ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss,
                _fgo_info->_para_pose[start_frame], relo_old_pose_,
                _fgo_info->_para_ex_pose[0], _fgo_info->_para_feature[feature_index]);
            ++added;
        }

        if (added == 0)
        {
            // no matched feature survived in the window; deactivate
            relo_active_ = false;
            relo_obs_.clear();
            return;
        }
        std::cout << "[PG] adding " << added
                  << " relocalization factors (old-frame anchor) on node "
                  << node << std::endl;
    }

    void visprocesser::_addMarginInfo() {

        if (!_fgo_info->time_to_margin()) return;

        int first_obs_node = _node_index_copy()[0];
        if (first_obs_node != 0) return;

        int feature_index = -1;
        ceres::LossFunction* loss_function;
        loss_function = new ceres::HuberLoss(2.0);

        for (auto& it_per_id : map_server)
        {
            auto& feature = it_per_id.second;
            if (feature.inv_depth < 0) continue;
            int used_num = feature.observations.size();
            int start_frame = feature.start_frame;
            if (!(used_num >= 2 && start_frame < _fgo_info->_window_size - 3))
                continue;
            ++feature_index;
            if (feature_index >= 1000) break;

            int node_i = start_frame;
            const auto& node_index = _node_index_copy();
            auto it = std::find(node_index.begin(), node_index.end(), node_i);
            int imu_i = (it != node_index.end())
                ? std::distance(node_index.begin(), it)
                : -1;
            if (node_i != 0) continue;
            int imu_j = imu_i - 1;

            Triple pts_i;
            auto feature_first_frame = feature.observations[feature.start_frame_id];
            pts_i << feature_first_frame.position.head(2), 1;

            for (auto& it_next_frame : feature.observations)
            {
                feature_per_frame& feature_next_frame = it_next_frame.second;
                imu_j++;
                int node_j = _node_index_copy()[imu_j];
                if (imu_i == imu_j)
                {
                    continue;
                }
                Triple pts_j;
                pts_j << feature_next_frame.position.head(2), 1;
                if (imgproc->estimate_t)
                {
                    ProjectionTdFactor* f_td = new ProjectionTdFactor(pts_i, pts_j,
                        feature_first_frame.velocity, feature_next_frame.velocity,
                        feature_first_frame.cur_td, feature_next_frame.cur_td);

                    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(f_td,
                        loss_function, vector<double*>{_fgo_info->_para_pose[node_i], _fgo_info->_para_pose[node_j],
                        _fgo_info->_para_ex_pose[0], _fgo_info->_para_feature[feature_index], _fgo_info->_para_td[0]},
                        vector<int>{0, 3});
                    _fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
                }
                else
                {
                    ProjectionFactor* f = new ProjectionFactor(pts_i, pts_j);

                    ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(f,
                        loss_function, vector<double*>{_fgo_info->_para_pose[node_i], _fgo_info->_para_pose[node_j],
                        _fgo_info->_para_ex_pose[0], _fgo_info->_para_feature[feature_index]},
                        vector<int>{0, 3});

                    _fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        for (int i = 0; i < NUM_OF_CAM; i++)
            _fgo_info->addr_shift[reinterpret_cast<long>(_fgo_info->_para_ex_pose[i])] = _fgo_info->_para_ex_pose[i];
        if (imgproc->estimate_t)
        {
            _fgo_info->addr_shift[reinterpret_cast<long>(_fgo_info->_para_td[0])] = _fgo_info->_para_td[0];
        }
    }

    void visprocesser::slide_window() {

        auto& vis_node_index = _node_index();
        if (vis_node_index.size() == 0) return;

        for (auto& it : vis_node_index) {
            it--;
        }

        if (vis_node_index[0] < 0) {
            vis_node_index.erase(vis_node_index.begin());
        }

        if (vis_node_index.size() == 0) {
			map_server.clear();
            return;
        }

        for (auto it = map_server.begin(); it != map_server.end(); )
        {
            auto& feature = it->second;
            auto& start_frame = feature.start_frame;
            start_frame--;

            if (start_frame < 0) {
                start_frame = _node_index()[0];
                feature.frame_size--;
                feature.observations.erase(feature.observations.begin());

                if (feature.frame_size > 0) {
                    feature.start_frame_id = feature.observations.begin()->first;
                    ++it;
                }
                else {
                    it = map_server.erase(it);
                }
            }
            else {
                ++it;
            }
        }
    }
}
