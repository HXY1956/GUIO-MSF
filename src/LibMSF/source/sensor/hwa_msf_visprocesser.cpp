#include "hwa_msf_visprocesser.h"
#include "hwa_set_ign.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_msf {
    visprocesser::visprocesser(const baseprocesser& B, int ID, base_data* data) : baseprocesser(B), cam_group_id(ID),
        vis_base(_gset.get(), ID), imgdata(dynamic_cast<vis_data*>(data)){
        _extrinsic_init();
        TimeCostDebugStatus = dynamic_cast<hwa_set::set_proc*>(_gset.get())->TimeCostDebug();
        make_dir("TimeCost");
        TimeCostDebugOutFile.open("TimeCost\\TimeCostCamUpdate.txt", std::ios::out | std::ios::trunc);
        beg.from_secs(dynamic_cast<set_vis*>(_gset.get())->start(cam_group_id));
        end.from_secs(dynamic_cast<set_vis*>(_gset.get())->end(cam_group_id));
        TimeStamp = beg;
        _Estimator = dynamic_cast<set_ign*>(_gset.get())->fuse_type();
    };

    visprocesser::visprocesser(std::shared_ptr<set_base> gset, std::string site, int ID, base_log spdlog, base_data* data, base_time _beg, base_time _end) : baseprocesser(gset, spdlog, site, _beg, _end),
        cam_group_id(ID), vis_base(gset.get(), ID), imgdata(dynamic_cast<vis_data*>(data))
    {
        _extrinsic_init();
        TimeCostDebugStatus = dynamic_cast<hwa_set::set_proc*>(_gset.get())->TimeCostDebug();
        make_dir("TimeCost");
        TimeCostDebugOutFile.open("TimeCost\\TimeCostCamUpdate.txt", std::ios::out | std::ios::trunc);
        beg.from_secs(dynamic_cast<set_vis*>(_gset.get())->start(cam_group_id));
        end.from_secs(dynamic_cast<set_vis*>(_gset.get())->end(cam_group_id));
        TimeStamp = beg;
        _Estimator = dynamic_cast<set_ign*>(_gset.get())->fuse_type();
    };

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
        if(imgdata->load(insdtime, dtime, get_curr_imgpath())) 
            TimeStamp = base_time(TimeStamp.gwk(), dtime);
        return std::abs(inst.diff(TimeStamp)) < _shm->delay;
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

        if (!checkStaticMotion() && keyframeCheck() && addFeatureObservations())
        {
            StateAugmentation();
            RemoveLostFeatures();
            PruneCamState();
        }
        else {
            _sins->vn = Triple::Zero();
            return NO_MEAS;
        }

        return VIS_MEAS;
    }

    void visprocesser::StateAugmentation()
    {
        base_scopedtimer timer("StateAugmentation()", TimeCostDebugOutFile, TimeCostDebugStatus);
        double dt = _sins->t - TimeStamp.sow() - TimeStamp.dsec();
        //compensate due to time 
        Triple BLH = _sins->pos - _sins->eth.v2dp(_sins->vn, dt);//BLH
        Triple XYZ = Geod2Cart(BLH, false);
        Triple imu_vel = _sins->vn - _sins->an * dt;//ENU
        const SO3& R_i_c = R_cam0_imu.transpose();
        const Triple& t_c_i = t_cam0_imu;
        const Triple& t_i_c = -R_i_c * t_cam0_imu;
        const SO3& R_i_n = hwa_base::base_att_trans::q2mat(_sins->qnb);
        SO3 R_n_i = R_i_n.transpose();
        SO3 R_n_e = hwa_base::Cen(BLH);
        SO3 R_e_i = R_n_i * R_n_e.transpose();
        Triple t_c_e = XYZ + R_n_e * R_i_n * t_c_i;
        SO3 R_e_c = R_i_c * R_e_i;
        //maintain the numerical stability of derivatives
        if (mIsFirstImg)
        {
            if (clone == hwa_vis::CAMERA) initCamPos = t_c_e;
            else initCamPos = XYZ;
            mIsFirstImg = false;
        }
        if (clone == hwa_vis::CAMERA) {
            cam_states[cam_state_id] = hwa_vis::CamState(cam_state_id);
            hwa_vis::CamState& cam_state = cam_states[cam_state_id];
            cam_state.time = TimeStamp.sod();
            cam_state.orientation = R_e_c.transpose();
            cam_state.position = t_c_e - initCamPos;
            cam_state.gravity = _sins->eth.gcc;
            cam_state.orientation_null = cam_state.orientation;
            cam_state.position_null = cam_state.position;
            cam_state.qnc = R_i_n * R_i_c.transpose();
            cam_state.qcb = R_i_c;
            cam_state.Tcb = t_i_c;
            cam_state.qbc = R_i_c.transpose();
            cam_state.Tbc = t_c_i;
            cam_state.R_e_n = R_n_e.transpose();
        }
        else if (clone == hwa_vis::IMU) {
            imu_states[cam_state_id] = hwa_vis::CamState(cam_state_id);
            hwa_vis::CamState& imu_state = imu_states[cam_state_id];
            imu_state.time = TimeStamp.sod();
            imu_state.orientation = R_e_i.transpose();
            imu_state.position = XYZ - initCamPos;
            cam_states = imuStateServer2CamStateServer(imu_states,
                imgproc->T_cam0_imu.linear(),
                imgproc->T_cam0_imu.translation());
        }

        frame_count++;
        Bgs[frame_count - 1] = _sins->eb;
        Bas[frame_count - 1] = _sins->db;

        std::string cam_id = camstate_id2str(cam_state_id);
        hwa_base::base_par att_x_par(_name, hwa_base::par_type::CAM_ATT_X, param_of_sins->parNumber(), cam_id);
        param_of_sins->addParam(att_x_par);
        hwa_base::base_par att_y_par(_name, hwa_base::par_type::CAM_ATT_Y, param_of_sins->parNumber() + 1, cam_id);
        param_of_sins->addParam(att_y_par);
        hwa_base::base_par att_z_par(_name, hwa_base::par_type::CAM_ATT_Z, param_of_sins->parNumber() + 2, cam_id);
        param_of_sins->addParam(att_z_par);

        hwa_base::base_par crd_x_par(_name, hwa_base::par_type::CAM_CRD_X, param_of_sins->parNumber() + 3, cam_id);
        param_of_sins->addParam(crd_x_par);
        hwa_base::base_par crd_y_par(_name, hwa_base::par_type::CAM_CRD_Y, param_of_sins->parNumber() + 4, cam_id);
        param_of_sins->addParam(crd_y_par);
        hwa_base::base_par crd_z_par(_name, hwa_base::par_type::CAM_CRD_Z, param_of_sins->parNumber() + 5, cam_id);
        param_of_sins->addParam(crd_z_par);
        param_of_sins->reIndex();

        size_t old_rows = _sins->Pk.rows();
        size_t old_cols = _sins->Pk.cols();

        Matrix J = Matrix::Zero(6, old_cols);

        if (clone == hwa_vis::CAMERA) {
            if (_Estimator == NORMAL) {
                //cam att to imu att
                J.block<3, 3>(0, 0) = SO3::Identity();
                //cam pos to imu att
                J.block<3, 3>(3, 0) = askew(R_n_e * R_i_n * t_c_i);
            }
            else if (_Estimator == INEKF) {
                //cam att to imu att
                J.block(0, 0, 3, 3) = SO3::Identity();
            }

            //cam pos to imu pos
            J.block<3, 3>(3, 6) = SO3::Identity();

            if (imgproc->estimate_extrinsic)
            {
                int index = param_of_sins->getParam(_name, par_type::EX_CAM_ATT_X, "");
                if (_Estimator == NORMAL) {
                    J.block<3, 3>(0, index) = -R_e_c.transpose();
                    J.block<3, 3>(3, index + 3) = -R_e_i.transpose();
                }
                else if (_Estimator == INEKF) {
                    J.block(0, index, 3, 3) = R_e_c.transpose();
                    J.block(3, index + 3, 3, 3) = -R_e_i.transpose();
                }
            }
        }
        else if (clone == hwa_vis::IMU) {
            //imu att to imu att
            J.block<3, 3>(0, 0) = SO3::Identity();
            //imu pos to imu pos
            J.block<3, 3>(3, 6) = SO3::Identity();
        }

        _sins->Pk.conservativeResize(old_rows + 6, old_cols + 6);
        _sins->Xk = Vector::Zero(_sins->Pk.rows());

        const Matrix P11 = _sins->Pk.block(0, 0, old_rows, old_cols);
        //matrix augment
        _sins->Pk.block(old_rows, 0, 6, old_cols) = J * P11;
        _sins->Pk.block(0, old_cols, old_rows, 6) = _sins->Pk.block(old_rows, 0, 6, old_cols).transpose();
        _sins->Pk.block<6, 6>(old_rows, old_cols) = J * P11 * J.transpose();

        for (int i = 0; i < _sins->Pk.rows(); i++)
        {
            if (_sins->Pk(i, i) <= 0)
            {
                std::cout << "PK warning :" << i << "," << std::setprecision(4) << _sins->Pk(i, i) << "," << param_of_sins->operator[](i).str_type() << std::endl;
                std::cin.get();
            }
        }
        _sins->Pk.block<3, 3>(old_rows, old_cols) += Matrix::Identity(3, 3) * 1e-12;
        _sins->Pk.block<3, 3>(old_rows + 3, old_cols + 3) += Matrix::Identity(3, 3) * 1e-10;
    }

    void visprocesser::RemoveLostFeatures()
    {
        base_scopedtimer timer("RemoveLostFeatures()", TimeCostDebugOutFile, TimeCostDebugStatus);
        int pointNum = 0;
        int pointInitialfail = 0;
        int pointGatingTestfail = 0;
        int pointUpdate = 0;
        int jacobian_row_size = 0;
        std::vector<hwa_vis::FeatureIDType> invalid_feature_ids(0);
        std::vector<hwa_vis::CamStateIDType> processed_feature_ids(0);

        for (auto iter = map_server.begin();
            iter != map_server.end(); ++iter)
        {
            auto& feature = iter->second;
            if (!feature.isLost)
                continue;
            if (feature.observations.size() < 5)
            {
                invalid_feature_ids.push_back(feature.id);
                //std::cout << TimeStamp.sod() << " Remove Observation Failed!" << std::endl;
                continue;
            }
            else { std::cout << feature.id << " ObservationSize: " << feature.observations.size() << std::endl; }

            pointNum++;
            if ((!feature.is_initialized) || ((feature.is_initialized) && (feature.is_initialized_NonKey)))
            {
                if (!feature.checkMotion(cam_states))
                {
                    invalid_feature_ids.push_back(feature.id);
                    pointInitialfail++;
                    std::cout << TimeStamp.sod() << " Remove Check Motion Failed!" << std::endl;
                    continue;
                }
                if (!feature.triangulatePoint(cam_states))
                {
                    invalid_feature_ids.push_back(feature.id);
                    pointInitialfail++;
                    std::cout << TimeStamp.sod() << " Remove Triangulated Failed!" << std::endl;
                    continue;
                }
                else
                {
                    if (!feature.initializePosition(cam_states))
                    {
                        invalid_feature_ids.push_back(feature.id);
                        pointInitialfail++;
                        std::cout << TimeStamp.sod() << " Remove Initiallize Failed!" << std::endl;
                        continue;
                    }
                    else
                        feature.is_initialized_NonKey = false;
                }
            }
            int observationSize = feature.observations.size();
            if (stereo) jacobian_row_size += 4 * observationSize - 3;
            else jacobian_row_size += 2 * observationSize - 3;
            processed_feature_ids.push_back(feature.id);
        }
        for (const auto& feature_id : invalid_feature_ids)
            map_server.erase(feature_id);

        if (processed_feature_ids.size() == 0)
            return;
        Matrix H_x;
        Vector r;

        H_x = Matrix::Zero(jacobian_row_size, 6 * cam_states.size() + ex_param_num);
        r = Vector::Zero(jacobian_row_size);
        int stack_cntr = 0;
        for (const auto& feature_id : processed_feature_ids)
        {
            auto& feature = map_server[feature_id];
            std::vector<hwa_vis::CamStateIDType> cam_state_ids(0);
            for (const auto& measurement : feature.observations)
            {
                if (cam_states[measurement.first].isKeyFrame)
                    cam_state_ids.push_back(measurement.first);
            }
            Matrix H_xj;
            Vector r_j;

            if (!featureJacobian(feature.id, cam_state_ids, H_xj, r_j))
            {
                std::cout << TimeStamp.sod() << " Remove Feature Failed!" << std::endl;
                continue;
            }
            if (GatingTest(H_xj, r_j, cam_state_ids.size() - 1))
            {
                H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
                r.segment(stack_cntr, r_j.rows()) = r_j;
                stack_cntr += H_xj.rows();
            }
            else
            {
                std::cout << TimeStamp.sod() << " Gating Test Fail" << std::endl;
                pointGatingTestfail++;
            }
            if (stack_cntr > 1500)
                break;
        }
        pointUpdate = pointNum - pointInitialfail - pointGatingTestfail;
        H_x.conservativeResize(stack_cntr, H_x.cols());
        r.conservativeResize((Eigen::Index)stack_cntr);

        meas_update(H_x, r);
        for (const auto& feature_id : processed_feature_ids)
        {
            map_server.erase(feature_id);
        }
        return;
    }

    void visprocesser::PruneCamState()
    {
        base_scopedtimer timer("PruneCamState()", TimeCostDebugOutFile, TimeCostDebugStatus);
        if (KeyFrameBuffer.size() < max_camstate_size)
            return;
        int pointNum = 0;
        int pointInitialfail = 0;
        int pointGatingTestfail = 0;
        int pointUpdate = 0;
        std::vector<hwa_vis::CamStateIDType> rm_cam_state_ids(0);
        findRedundantCamStates(rm_cam_state_ids);
        int jacobian_row_size = 0;

        for (auto& item : map_server)
        {
            auto& feature = item.second;
            std::vector<hwa_vis::CamStateIDType> involved_cam_state_ids(0);
            for (const auto& cam_id : rm_cam_state_ids)
            {
                if (feature.observations.find(cam_id) !=
                    feature.observations.end())
                    involved_cam_state_ids.push_back(cam_id);
            }

            if (involved_cam_state_ids.size() == 0)
                continue;
            if (involved_cam_state_ids.size() == 1)
            {
                feature.observations.erase(involved_cam_state_ids[0]);
                continue;
            }
            if (involved_cam_state_ids.size() == 2)
            {
                feature.observations.erase(involved_cam_state_ids[0]);
                feature.observations.erase(involved_cam_state_ids[1]);
                continue;
            }

            pointNum++;
            if ((!feature.is_initialized) || ((feature.is_initialized) && (feature.is_initialized_NonKey)))
            {
                if (!stereo && !feature.checkMotion(cam_states))
                {

                    for (const auto& cam_id : involved_cam_state_ids)
                        feature.observations.erase(cam_id);
                    pointInitialfail++;
                    //   std::cout << TimeStamp.sod() << " Prune Check Motion Failed!" << std::endl;
                    continue;

                }
                if (!feature.triangulatePoint(cam_states))
                {

                    for (const auto& cam_id : involved_cam_state_ids)
                        feature.observations.erase(cam_id);
                    pointInitialfail++;
                    // std::cout << TimeStamp.sod() << " Prune Triangulated Failed!" << std::endl;
                    continue;
                }
                else
                {
                    if (!feature.initializePosition(cam_states))
                    {

                        for (const auto& cam_id : involved_cam_state_ids)
                            feature.observations.erase(cam_id);
                        pointInitialfail++;
                        //   std::cout << TimeStamp.sod() << " Prune Intialize Failed!" << std::endl;
                        continue;
                    }
                    else
                        feature.is_initialized_NonKey = false;
                }
            }

            if (stereo) jacobian_row_size += 4 * involved_cam_state_ids.size() - 3;
            else    jacobian_row_size += 2 * involved_cam_state_ids.size() - 3;
        }

        Matrix H_x;
        H_x = Matrix::Zero(jacobian_row_size, 6 * cam_states.size() + ex_param_num);
        Vector r = Vector::Zero(jacobian_row_size);

        int stack_cntr = 0;
        for (auto& item : map_server)
        {
            auto& feature = item.second;


            std::vector<hwa_vis::CamStateIDType> involved_cam_state_ids(0);
            for (const auto& cam_id : rm_cam_state_ids)
            {
                if (feature.observations.find(cam_id) !=
                    feature.observations.end())
                    involved_cam_state_ids.push_back(cam_id);
            }

            if (involved_cam_state_ids.size() == 0 || involved_cam_state_ids.size() == 1)
                continue;


            Matrix H_xj;
            Vector r_j;

            if (!featureJacobian(feature.id, involved_cam_state_ids, H_xj, r_j))
            {
                for (const auto& cam_id : involved_cam_state_ids)
                    feature.observations.erase(cam_id);
                std::cout << TimeStamp.sod() << " Prune Feature Failed!" << std::endl;
                continue;
            }

            if (GatingTest(H_xj, r_j, involved_cam_state_ids.size()))
            {
                H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
                r.segment(stack_cntr, r_j.rows()) = r_j;
                stack_cntr += H_xj.rows();
            }
            else
            {
                pointGatingTestfail++;
            }
            for (const auto& cam_id : involved_cam_state_ids)
                feature.observations.erase(cam_id);
        }

        H_x.conservativeResize(stack_cntr, H_x.cols());
        r.conservativeResize(stack_cntr);
        pointUpdate = pointNum - pointInitialfail - pointGatingTestfail;

        meas_update(H_x, r);

        for (const auto& cam_id : rm_cam_state_ids)
        {
            hwa_vis::CamStateIDType _cam_id = cam_id;
            std::string string_id = camstate_id2str(_cam_id);
            Matrix tmp_Qx = _sins->Pk;
            Vector _dx = _sins->Xk;

            int idx_att_x = param_of_sins->getParam(_name, hwa_base::par_type::CAM_ATT_X, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_att_x).index, param_of_sins->operator[](idx_att_x).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_att_x).index);
            param_of_sins->delParam(idx_att_x);
            param_of_sins->reIndex();

            int idx_att_y = param_of_sins->getParam(_name, hwa_base::par_type::CAM_ATT_Y, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_att_y).index, param_of_sins->operator[](idx_att_y).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_att_y).index);
            param_of_sins->delParam(idx_att_y);
            param_of_sins->reIndex();

            int idx_att_z = param_of_sins->getParam(_name, hwa_base::par_type::CAM_ATT_Z, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_att_z).index, param_of_sins->operator[](idx_att_z).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_att_z).index);
            param_of_sins->delParam(idx_att_z);
            param_of_sins->reIndex();

            int idx_crd_x = param_of_sins->getParam(_name, hwa_base::par_type::CAM_CRD_X, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_crd_x).index, param_of_sins->operator[](idx_crd_x).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_crd_x).index);
            param_of_sins->delParam(idx_crd_x);
            param_of_sins->reIndex();

            int idx_crd_y = param_of_sins->getParam(_name, hwa_base::par_type::CAM_CRD_Y, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_crd_y).index, param_of_sins->operator[](idx_crd_y).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_crd_y).index);
            param_of_sins->delParam(idx_crd_y);
            param_of_sins->reIndex();

            int idx_crd_z = param_of_sins->getParam(_name, hwa_base::par_type::CAM_CRD_Z, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_crd_z).index, param_of_sins->operator[](idx_crd_z).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_crd_z).index);
            param_of_sins->delParam(idx_crd_z);
            param_of_sins->reIndex();

            _sins->Pk = tmp_Qx;
            _sins->Xk = _dx;
            cam_states.erase(cam_id);
            if (clone == hwa_vis::IMU) {
                imu_states.erase(cam_id);
            }
            frame_count--;
        }
        return;
    }

    bool visprocesser::GatingTest(const Matrix& H, const Vector& r, const int& dof)
    {
        int par_size = param_of_sins->parNumber();
        int obs_size = H.rows();
        int N = cam_states.size();

        Matrix All_H = Matrix::Zero(H.rows(), par_size);
        assert(obs_size == r.rows());

        auto cam_state_iter = cam_states.begin();
        for (int i = 0; i < cam_states.size();
            ++i, ++cam_state_iter)
        {
            std::string cam_id = camstate_id2str(cam_state_iter->first);
            int idx = param_of_sins->getParam(_name, hwa_base::par_type::CAM_ATT_X, cam_id);

            All_H.block(0, idx, obs_size, 6) = H.block(0, 6 * i + ex_param_num, obs_size, 6);
        }
        if (imgproc->estimate_extrinsic)
        {
            int idx = param_of_sins->getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "");
            All_H.block(0, idx, obs_size, 6) = H.block(0, 0, obs_size, 6);

        }
        if (imgproc->estimate_t)
        {
            int idx = param_of_sins->getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam0");
            All_H.block(0, idx, obs_size, 1) = H.block(0, ex_param_num - 1, obs_size, 1);
        }
        ;
        Matrix P1 = All_H * _sins->Pk * All_H.transpose();
        Matrix P2 = feature_observation_noise * Matrix::Identity(All_H.rows(), All_H.rows());
        double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);
        if (gamma < chi_squared_test_table[dof]) {
            return true;
        }
        else {
            return false;
        }
    }

    void visprocesser::meas_update(const Matrix& H, const Vector& r)
    {
        if (H.rows() == 0 || r.rows() == 0) {
            std::cout << "Bad Visual" << std::endl;
            return;
        }

        int par_size = param_of_sins->parNumber();
        int obs_size = H.rows();

        Matrix All_H = Matrix::Zero(H.rows(), par_size);
        assert(obs_size == r.rows());
        auto cam_state_iter = cam_states.begin();
        for (int i = 0; i < cam_states.size();
            ++i, ++cam_state_iter)
        {
            std::string cam_id = camstate_id2str(cam_state_iter->first);
            int idx = param_of_sins->getParam(_name, hwa_base::par_type::CAM_ATT_X, cam_id);
            All_H.block(0, idx, obs_size, 6) = H.block(0, 6 * i + ex_param_num, obs_size, 6);
        }
        if (imgproc->estimate_extrinsic)
        {
            int idx = param_of_sins->getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "");
            All_H.block(0, idx, obs_size, 6) = H.block(0, 0, obs_size, 6);
            if (num_of_cam == 2 && imgproc->estimate_extrinsic_allcam)
            {
                idx = param_of_sins->getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "cam1");
                All_H.block(0, idx, obs_size, 6) = H.block(0, 6, obs_size, 6);
            }
        }
        if (imgproc->estimate_t)
        {
            int idx = param_of_sins->getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam0");
            All_H.block(0, idx, obs_size, 1) = H.block(0, ex_param_num - 1, obs_size, 1);
            //TODO: multi-cam t
            if (num_of_cam == 2 && imgproc->estimate_t_allcam)
            {
                idx = param_of_sins->getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam0");
                All_H.block(0, idx, obs_size, 1) = H.block(0, ex_param_num - 2, obs_size, 1);
                idx = param_of_sins->getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam1");
                All_H.block(0, idx, obs_size, 1) = H.block(0, ex_param_num - 1, obs_size, 1);
            }
        }

        if (All_H.rows() > All_H.cols())
        {
            Eigen::HouseholderQR<Matrix> qr_helper(All_H);
            Matrix Q = qr_helper.householderQ();
            Matrix Q1;
            Q1 = Q.leftCols(par_size);
            _sins->Hk = Q1.transpose() * All_H;
            _sins->Zk = Q1.transpose() * r;
        }
        else
        {
            _sins->Hk = All_H;
            _sins->Zk = r;
        }
        
        _sins->Rk = feature_observation_noise * Matrix::Identity(
            _sins->Hk.rows(), _sins->Hk.rows());
        Vector delta_x;

        _Updater._meas_update(_sins->Hk, _sins->Zk, _sins->Rk, delta_x, _sins->Pk);

        m_out("_sins->Zk", _sins->Zk);
        m_out("_sins->Rk", _sins->Rk);
        m_out("_sins->Pk", _sins->Pk);
        m_out("_sins->Xk", delta_x);

        _sins->Xk += delta_x;
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

    bool visprocesser::_extrinsic_init()
    {
        if (!imgproc->estimate_extrinsic && !imgproc->estimate_t) return true;
        //rotation and translation
        if (imgproc->estimate_extrinsic)
        {
            try
            {
                hwa_base::base_allpar param_extended;
                Matrix Qx_extended;
                int before_parNumber = param_of_sins->parNumber();

                // attitude        
                for (int ipar = (int)hwa_base::par_type::EX_CAM_ATT_X, i = 0; ipar <= (int)hwa_base::par_type::EX_CAM_ATT_Z; ipar++, i++)
                {
                    hwa_base::base_par attr(_name, hwa_base::par_type(ipar), ipar, "");
                    param_of_sins->addParam(attr);
                }
                // position 
                for (int ipar = (int)hwa_base::par_type::EX_CAM_CRD_X, i = 0; ipar <= (int)hwa_base::par_type::EX_CAM_CRD_Z; ipar++, i++)
                {
                    hwa_base::base_par pos(_name, hwa_base::par_type(ipar), ipar, "");
                    param_of_sins->addParam(pos);
                }

                if (num_of_cam == 2 && imgproc->estimate_extrinsic_allcam)
                {
                    for (int ipar = (int)hwa_base::par_type::EX_CAM_ATT_X, i = 0; ipar <= (int)hwa_base::par_type::EX_CAM_ATT_Z; ipar++, i++)
                    {
                        hwa_base::base_par attr(_name, hwa_base::par_type(ipar), ipar, "cam1");
                        param_of_sins->addParam(attr);
                    }
                    // position 
                    for (int ipar = (int)hwa_base::par_type::EX_CAM_CRD_X, i = 0; ipar <= (int)hwa_base::par_type::EX_CAM_CRD_Z; ipar++, i++)
                    {
                        hwa_base::base_par pos(_name, hwa_base::par_type(ipar), ipar, "cam1");
                        param_of_sins->addParam(pos);
                    }
                }

                int ibefore = param_of_sins->getParam(_name, hwa_base::par_type::EX_CAM_ATT_X, "");

                Matrix Qx_tmp = Matrix::Zero(param_of_sins->parNumber(), param_of_sins->parNumber());
                Qx_tmp.block(0, 0, before_parNumber, before_parNumber) = _sins->Pk;

                for (int i = ibefore, j = 0; j < 3; i++, j++)
                {
                    Qx_tmp(i, i) = imgproc->initial_cam_extrinsic_rotation_cov(j) * hwa_base::glv.deg * hwa_base::glv.deg;
                }
                for (int i = ibefore + 3, j = 0; j < 3; i++, j++)
                {
                    Qx_tmp(i, i) = imgproc->initial_cam_extrinsic_translation_cov(j);
                }

                if (num_of_cam == 2 && imgproc->estimate_extrinsic_allcam)
                {
                    for (int i = ibefore + 6, j = 0; j < 3; i++, j++)
                    {
                        Qx_tmp(i, i) = imgproc->initial_cam_extrinsic_rotation_cov(j) * hwa_base::glv.deg * hwa_base::glv.deg;
                    }
                    for (int i = ibefore + 9, j = 0; j < 3; i++, j++)
                    {
                        Qx_tmp(i, i) = imgproc->initial_cam_extrinsic_translation_cov(j);
                    }
                }
                _sins->Pk = Qx_tmp;
                param_of_sins->reIndex();
            }
            catch (...)
            {
                return false;
            }
        }

        if (imgproc->estimate_t)
        {
            try
            {
                int before_parNumber = param_of_sins->parNumber();

                hwa_base::base_par tc(_name, hwa_base::par_type((int)hwa_base::par_type::EXTRINSIC_T), (int)hwa_base::par_type::EXTRINSIC_T, "cam0");
                param_of_sins->addParam(tc);

                if (num_of_cam == 2 && imgproc->estimate_t_allcam)
                {
                    hwa_base::base_par tc(_name, hwa_base::par_type((int)hwa_base::par_type::EXTRINSIC_T), (int)hwa_base::par_type::EXTRINSIC_T, "cam1");
                    param_of_sins->addParam(tc);
                }

                int ibefore = param_of_sins->getParam(_name, hwa_base::par_type::EXTRINSIC_T, "cam0");
                Matrix Qx_tmp = Matrix::Zero(param_of_sins->parNumber(), param_of_sins->parNumber());
                ;
                Qx_tmp.block(0, 0, before_parNumber, before_parNumber) = _sins->Pk;
                _sins->Pk = Qx_tmp;

                _sins->Pk(ibefore, ibefore) = imgproc->initial_cam_t_cov;

                if (num_of_cam == 2 && imgproc->estimate_t_allcam)
                {
                    _sins->Pk(ibefore + 1, ibefore + 1) = imgproc->initial_cam_t_cov;
                }
                param_of_sins->reIndex();
            }
            catch (...)
            {
                return false;
            }
        }
        return true;
    }

    bool visprocesser::align_vins() {
        if (frame_count > 0) {
            double image_time = cam_states[cam_state_id].time;
            double dt = _sins->t - image_time;
            if (dt >= 0) {
                if (dt < 1.0 / imu_frequency) {
                    if (pre_cam_state_id >= 0) {
                        cam_states[pre_cam_state_id].pre_integration->processIMU(1.0 / imu_frequency - dt, _sins->obs_fb,
                            _sins->obs_wib, Bgs[frame_count - 2], Bas[frame_count - 2]);
                    }
                    cam_states[cam_state_id].pre_integration->processIMU(dt, _sins->obs_fb, _sins->obs_wib,
                        Bgs[frame_count - 1], Bas[frame_count - 1]);
                }
                else {
                    cam_states[cam_state_id].pre_integration->processIMU(1.0 / imu_frequency, _sins->obs_fb, _sins->obs_wib,
                        Bgs[frame_count - 1], Bas[frame_count - 1]);
                }
            }
        }
        if (frame_count == max_camstate_size - 1) {
            if (initialStructure()) {
                align_feedback();
                return 1;
            }
        }
        return 0;
    }

    void visprocesser::align_feedback() {
        _sins->eb = Bgs[frame_count - 1];
        _sins->db = Bas[frame_count - 1];
        _sins->qeb = hwa_base::base_quat(cam_states.rbegin()->second.orientation_b.w(), cam_states.rbegin()->second.orientation_b.x(),
            cam_states.rbegin()->second.orientation_b.y(), cam_states.rbegin()->second.orientation_b.z());
        _sins->ve = cam_states.rbegin()->second.ve;
        _sins->pos_ecef = cam_states.rbegin()->second.position_b + initCamPos;

        _sins->Ceb = hwa_base::base_att_trans::q2mat(_sins->qeb);
        _sins->pos = Cart2Geod(_sins->pos_ecef, false);
        _sins->eth.Update(_sins->pos, _sins->vn);
        _sins->vn = _sins->eth.Cne * _sins->ve;
        _sins->qnb = hwa_base::base_att_trans::m2qua(_sins->eth.Cne) * _sins->qeb;
        _sins->att = hwa_base::base_att_trans::q2att(_sins->qnb);
        _sins->Cnb = hwa_base::base_att_trans::q2mat(_sins->qnb);
        _sins->orientation = _sins->Ceb.transpose();
        _sins->velocity = _sins->ve;
        _sins->position = Geod2Cart(_sins->pos, false);

        std::cout << " Refined Gyo Bias: " << _sins->eb.transpose() << std::endl;
        std::cout << " Refined Acc Bias: " << _sins->db.transpose() << std::endl;
        std::cout << " Rotation Matrix Rnb: " << std::endl << _sins->Cnb << std::endl;

        Triple ypr = hwa_vis::vis_base::R2ypr(_sins->Cnb);
        std::cout << "Yaw: " << ypr(0) << "Pitch: " << ypr(1) << "Roll: " << ypr(2) << std::endl;
    }

    void visprocesser::_feed_back() {
        if (imgproc->estimate_extrinsic)
        {
            int index = param_of_sins->getParam(_name, par_type::EX_CAM_ATT_X, "");
            if (index < 0) cerr << "Extrinsic Update Error!" << endl;
            Eigen::Vector3d att_dx = Eigen::Vector3d(_sins->Xk(index), _sins->Xk(index + 1), _sins->Xk(index + 2));
            Eigen::Vector3d crd_dx = Eigen::Vector3d(_sins->Xk(index + 3), _sins->Xk(index + 4), _sins->Xk(index + 5));
            Eigen::Vector3d crd_dx1;
            base_quat dq_cam = base_att_trans::rv2q(-att_dx);    // R_i_c = (I+theta^) * R_i_c ; R_c_i = R_c_i * (I - theta);
            Eigen::Quaterniond _dq_ext;
            _dq_ext.w() = dq_cam.q0; _dq_ext.x() = dq_cam.q1; _dq_ext.y() = dq_cam.q2; _dq_ext.z() = dq_cam.q3;
            _dq_ext.normalize();
            R_cam0_imu = this->R_cam0_imu * _dq_ext.toRotationMatrix();
            t_cam0_imu += crd_dx;    // t_c_i = t_c_i + delta_t;

            T_cam0_imu.linear() = R_cam0_imu;
            T_cam0_imu.translation() = t_cam0_imu;
            imgproc->T_cam0_imu = T_cam0_imu;
            imgproc->t_cam0_imu = t_cam0_imu;
            imgproc->R_cam0_imu = R_cam0_imu;

            if (num_of_cam == 2 && imgproc->estimate_extrinsic_allcam)
            {
                Eigen::Isometry3d T_c1_imu = imgproc->T_cam1_imu;
                Eigen::Matrix3d R_c1_imu = T_c1_imu.linear();
                Eigen::Vector3d t_c1_imu = T_c1_imu.translation();

                index = param_of_sins->getParam(_name, par_type::EX_CAM_ATT_X, "cam1");
                if (index < 0) cerr << "Extrinsic Update Error!" << endl;
                att_dx = Eigen::Vector3d(_sins->Xk(index), _sins->Xk(index + 1), _sins->Xk(index + 2));
                crd_dx = Eigen::Vector3d(_sins->Xk(index + 3), _sins->Xk(index + 4), _sins->Xk(index + 5));

                dq_cam = base_att_trans::rv2q(att_dx);
                _dq_ext.w() = dq_cam.q0; _dq_ext.x() = dq_cam.q1; _dq_ext.y() = dq_cam.q2; _dq_ext.z() = dq_cam.q3;
                _dq_ext.normalize();

                R_c1_imu = R_c1_imu * _dq_ext.toRotationMatrix();
                crd_dx1 = R_c1_imu * (-crd_dx);
                t_c1_imu -= crd_dx1;

                // cam1 in t_gvis is _cam0^cam1
                // cam1 in imgproc are _cam0^cam1 and _cam1^imu
                imgproc->R_cam1_imu = R_c1_imu;
                imgproc->t_cam1_imu = t_c1_imu;
                imgproc->T_cam1_imu.linear() = imgproc->R_cam1_imu;
                imgproc->T_cam1_imu.translation() = imgproc->t_cam1_imu;

                T_cam0_cam1 = (imgproc->T_cam1_imu.inverse()) * T_cam0_imu;
                R_cam0_cam1 = T_cam0_cam1.linear();
                t_cam0_cam1 = T_cam0_cam1.translation();


                imgproc->T_cam0_cam1 = T_cam0_cam1;
                imgproc->R_cam0_cam1 = R_cam0_cam1;
                imgproc->t_cam0_cam1 = t_cam0_cam1;
            }
            else
            {
                imgproc->T_cam1_imu = imgproc->T_cam0_imu * imgproc->T_cam0_cam1.inverse();
                imgproc->R_cam1_imu = imgproc->T_cam1_imu.linear();
                imgproc->t_cam1_imu = imgproc->T_cam1_imu.translation();
            }

        }
        if (imgproc->estimate_t)
        {
            int index = param_of_sins->getParam(_name, par_type::EXTRINSIC_T, "cam0");
            if (index < 0) cerr << "Extrinsic dt Update Error!" << endl;
            dt_cam0_imu = dt_cam0_imu - _sins->Xk(index);
            //for (auto i : imu_states)
            //{
            //    auto& imu_state = i.second;
            //    t_gquat dq_cam = t_gbase::rv2q(-_sins->Xk(index) * imu_state.wib);
            //    Eigen::Quaterniond _dq_cam;
            //    _dq_cam.w() = dq_cam.q0; _dq_cam.x() = dq_cam.q1; _dq_cam.y() = dq_cam.q2; _dq_cam.z() = dq_cam.q3;
            //    _dq_cam.normalize();
            //    imu_state.orientation = _dq_cam * imu_state.orientation;
            //    imu_state.position = imu_state.position + sins.ve * _sins->Xk(index);
            //}
        }
        if (cam_states.size() > 0 && clone == CAMERA)
        {
            Eigen::Matrix4d CamXf = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d Rotation = Eigen::Matrix3d::Identity();
            auto cam_state_iter = cam_states.begin();
            for (int i = 0; i < cam_states.size();
                ++i, ++cam_state_iter)
            {
                CamXf = Eigen::Matrix4d::Identity();
                Rotation = Eigen::Matrix3d::Identity();
                CamStateIDType cur_id = cam_state_iter->first;
                string _camid = camstate_id2str(cur_id);

                int index = param_of_sins->getParam(_name, par_type::CAM_ATT_X, _camid);
                if (index < 0) cerr << "cam_feedback() idx wrong!" << endl;

                if (_Estimator == NORMAL) {
                    Triple att_dx = Triple(_sins->Xk(index), _sins->Xk(index + 1), _sins->Xk(index + 2));
                    Triple crd_dx = Triple(_sins->Xk(index + 3), _sins->Xk(index + 4), _sins->Xk(index + 5));

                    base_quat dq_cam = base_att_trans::rv2q(att_dx);
                    Eigen::Quaterniond _dq_cam;
                    _dq_cam.w() = dq_cam.q0; _dq_cam.x() = dq_cam.q1; _dq_cam.y() = dq_cam.q2; _dq_cam.z() = dq_cam.q3;
                    _dq_cam.normalize();
                    cam_state_iter->second.orientation = _dq_cam * cam_state_iter->second.orientation;
                    cam_state_iter->second.position -= crd_dx;
                }
                else {
                    //CamXf.block(0, 0, 3, 3) = cam_state_iter->second.orientation.normalized().toRotationMatrix().transpose();
                    CamXf.block(0, 0, 3, 3) = cam_state_iter->second.orientation.normalized().toRotationMatrix();
                    CamXf.block(0, 3, 3, 1) = cam_state_iter->second.position;

                    int K = 1;
                    Matrix X = Matrix::Identity(3 + K, 3 + K);
                    Eigen::Matrix3d R;
                    Eigen::Matrix3d Jl;
                    Eigen::Vector3d w = _sins->Xk.segment<3>(index);
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
                        X.block<3, 1>(0, 3 + i) = _sins->Xk.segment<3>(index + 3 + 3 * i);
                    }

                    //sins.Xf = X * sins.Xf;
                    CamXf = X.colPivHouseholderQr().solve(Matrix::Identity(3 + K, 3 + K)) * CamXf;
                    //Rotation = CamXf.block(0, 0, 3, 3).transpose();
                    Rotation = CamXf.block(0, 0, 3, 3);

                    cam_state_iter->second.orientation = Rotation;
                    cam_state_iter->second.orientation.normalize();
                    cam_state_iter->second.position = CamXf.block(0, 3, 3, 1);

                }
                /*cout << " ID " << cam_state_iter->first << "  " << setiosflags(ios::fixed) << setprecision(4) << cam_state_iter->second.position.transpose() << endl;*/
            }
        }
        else if (imu_states.size() > 0 && clone == IMU)
        {
            Eigen::Matrix4d ImuXf = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d Rotation = Eigen::Matrix3d::Identity();
            auto imu_state_iter = imu_states.begin();
            for (int i = 0; i < imu_states.size();
                ++i, ++imu_state_iter)
            {
                ImuXf = Eigen::Matrix4d::Identity();
                Rotation = Eigen::Matrix3d::Identity();
                CamStateIDType cur_id = imu_state_iter->first;
                string _camid = camstate_id2str(cur_id);

                int index = param_of_sins->getParam(_name, par_type::CAM_ATT_X, _camid);
                if (index < 0) cerr << "cam_feedback() idx wrong!" << endl;

                if (_Estimator == NORMAL) {
                    Triple att_dx = Triple(_sins->Xk(index), _sins->Xk(index + 1), _sins->Xk(index + 2));
                    Triple crd_dx = Triple(_sins->Xk(index + 3), _sins->Xk(index + 4), _sins->Xk(index + 5));

                    base_quat dq_cam = base_att_trans::rv2q(att_dx);
                    Eigen::Quaterniond _dq_cam;
                    _dq_cam.w() = dq_cam.q0; _dq_cam.x() = dq_cam.q1; _dq_cam.y() = dq_cam.q2; _dq_cam.z() = dq_cam.q3;
                    _dq_cam.normalize();
                    imu_state_iter->second.orientation = _dq_cam * imu_state_iter->second.orientation;
                    imu_state_iter->second.position -= crd_dx;
                }
                else {
                    //CamXf.block(0, 0, 3, 3) = cam_state_iter->second.orientation.normalized().toRotationMatrix().transpose();
                    ImuXf.block(0, 0, 3, 3) = imu_state_iter->second.orientation.normalized().toRotationMatrix();
                    ImuXf.block(0, 3, 3, 1) = imu_state_iter->second.position;

                    int K = 1;
                    Matrix X = Matrix::Identity(3 + K, 3 + K);
                    Eigen::Matrix3d R;
                    Eigen::Matrix3d Jl;
                    Eigen::Vector3d w = _sins->Xk.segment<3>(index);
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
                        X.block<3, 1>(0, 3 + i) = _sins->Xk.segment<3>(index + 3 + 3 * i);
                    }

                    //sins.Xf = X * sins.Xf;
                    ImuXf = X.colPivHouseholderQr().solve(Matrix::Identity(3 + K, 3 + K)) * ImuXf;
                    //Rotation = CamXf.block(0, 0, 3, 3).transpose();
                    Rotation = ImuXf.block(0, 0, 3, 3);

                    imu_state_iter->second.orientation = Rotation;
                    imu_state_iter->second.orientation.normalize();
                    imu_state_iter->second.position = ImuXf.block(0, 3, 3, 1);
                }
            }
        }
        _sins->_gv_sav = _sins->Pk;
        _sins->pos_store = _sins->pos;
        _sins->qnb_store = _sins->qnb;
        if (frame_count == 0) return;
        Bgs[frame_count - 1] = _sins->eb;
        Bas[frame_count - 1] = _sins->db;
    }
}