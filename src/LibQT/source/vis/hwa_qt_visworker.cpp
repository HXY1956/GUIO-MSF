#include "hwa_qt_visworker.h"
#include "hwa_set_ign.h"

namespace hwa_qt {
    qt_vis_worker::qt_vis_worker(hwa_set::set_base* gset, int ID, QObject* parent) : QObject(parent),
        cam_group_id(ID)
    {
        EstimatorType = dynamic_cast<hwa_set::set_ign*>(gset)->fuse_type();
        TimeCostDebugStatus = dynamic_cast<hwa_set::set_proc*>(gset)->TimeCostDebug();
        TimeCostDebugOutFile.open("TimeCostCamUpdate.txt", std::ios::out | std::ios::trunc);
        vis_interval = dynamic_cast<hwa_set::set_proc*>(gset)->visinterval();
        vis_size = dynamic_cast<hwa_set::set_vis*>(gset)->max_cam_state_size();
        CamAttr = std::make_shared<hwa_vis::vis_imgproc<cv::cuda::GpuMat>>(gset, cam_group_id);
        emit SharedCamAttr(CamAttr, ID);
        static_threshold = dynamic_cast<hwa_set::set_vis*>(gset)->static_threshold();
        translation_threshold[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->translation_threshold();
        huber_epsilon[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->huber_epsilon();
        estimation_precision[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->estimation_precision();
        initial_damping[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->initial_damping();
        outler_loop_max_iteration[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->outler_loop_max_iteration();
        inner_loop_max_iteration[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->inner_loop_max_iteration();
        T_cam0_cam1[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->T_cam0_cam1(ID);
        stereo[ID] = dynamic_cast<hwa_set::set_vis*>(gset)->stereo(ID);
    };

    qt_vis_worker::~qt_vis_worker() {
        if (TimeCostDebugOutFile.is_open()) TimeCostDebugOutFile.close();
    };

    void qt_vis_worker::Accept_OpenStatus(bool flag) {
        isOpen = flag;
    }

    void qt_vis_worker::Accept_pts(double timestamp, hwa_vis::PointCloud msg) {
        std::lock_guard<std::mutex> lock(map_mtx);
        if (checkStaticMotion(msg)) {
            emit NewUpdate(cam_group_id, true, timestamp);
            return;
        }
        Clouds[timestamp] = msg;
        addFeatureObservations(timestamp, msg);
    }

    void qt_vis_worker::Accept_ins(Mstate _ins_states, Matrix Pk) {
        std::lock_guard<std::mutex> lock(map_mtx);
        if (!isOpen) return;
        ins_states = _ins_states;
        double timestamp = ins_states.rbegin()->first;
        _global_variance = Pk;
        Matrix Hk1, Hk2;
        Vector Zk1, Zk2;
        RemoveLostFeatures(Hk1, Zk1);
        PruneCamState(Hk2, Zk2);
        emit NewUpdate(cam_group_id, false, timestamp, Hk1, Zk1, Hk2, Zk2);
    }

    Mstate qt_vis_worker::Imu2Cam(const Mstate& imu_states, const SO3 R_c_i, const Triple t_c_i)
    {
        Mstate cam_states = imu_states;
        auto cam_state_iter = cam_states.begin();
        for (int i = 0; i < cam_states.size();
            ++i, ++cam_state_iter)
        {
            cam_state_iter->second.position = cam_state_iter->second.position + cam_state_iter->second.orientation * t_c_i;
            cam_state_iter->second.orientation = cam_state_iter->second.orientation * Eigen::Quaterniond(R_c_i);
        }
        return cam_states;
    }

    bool qt_vis_worker::checkStaticMotion(const hwa_vis::PointCloud& msg)
    {
        base_scopedtimer timer("checkStaticMotion()", TimeCostDebugOutFile, TimeCostDebugStatus);
        double fx = CamAttr->cam0_intrinsics(0); double fy = CamAttr->cam0_intrinsics(1);
        double cx = CamAttr->cam0_intrinsics(2); double cy = CamAttr->cam0_intrinsics(3);

        double mean_motion = 0.0;
        int common_feature_size = 0;
        for (auto pts : msg.features)
        {
            int feature_id = pts.id;
            auto iter = map_server.find(feature_id);

            if (iter != map_server.end())
            {
                double x1, y1, x2, y2;
                x1 = fx * pts.cam0_point.x + cx;
                y1 = fy * pts.cam0_point.y + cy;
                x2 = fx * iter->second.observations.rbegin()->second(0) + cx;
                y2 = fy * iter->second.observations.rbegin()->second(1) + cy;
                double dx = x2 - x1;
                double dy = y2 - y1;
                mean_motion += sqrt(dx * dx + dy * dy);
                common_feature_size++;
            }
        }
        mean_motion = mean_motion / common_feature_size;

        if (mean_motion < static_threshold)
        {
            return true;
        }
        return false;
    }

    void qt_vis_worker::addFeatureObservations(double timestamp, const hwa_vis::PointCloud& msg)
    {
        base_scopedtimer timer("addFeatureObservations()", TimeCostDebugOutFile, TimeCostDebugStatus);

        int curr_feature_num = map_server.size();
        int tracked_feature_num = 0;
        int ID = int(timestamp / vis_interval);

        for (const auto& feature : msg.features)
        {
            if (map_server.find(feature.id) != map_server.end())
            {
                ++tracked_feature_num;
            }
            else {
                map_server[feature.id] = _OneFeature();
                map_server[feature.id].id = feature.id;
                map_server[feature.id].usbID = cam_group_id;
            }

            if (CamAttr->stereo) {
                map_server[feature.id].observations[ID] =
                    Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                        feature.cam1_point.x, feature.cam1_point.y);
            }
            else {
                map_server[feature.id].observations[ID] =
                    Eigen::Vector4d(feature.cam0_point.x, feature.cam0_point.y,
                        0.0, 0.0);
            }
        }
        for (auto iter = map_server.begin();
            iter != map_server.end(); ++iter)
        {
            auto& feature = iter->second;
            if (feature.observations.find(ID) ==
                feature.observations.end())
            {
                feature.isLost = true;
            }
        }
        double tracking_rate = static_cast<double>(tracked_feature_num) / static_cast<double>(curr_feature_num);
        std::cout << timestamp << ":  Tracking Rate: " << tracking_rate << std::endl;
        return;
    }

    std::set<int> qt_vis_worker::JudgeArea(_OneFeature feature, std::vector<int> a) {
        std::set<int> res;
        for (auto i : a) {
            if (feature.observations.find(i) != feature.observations.end())
                res.insert(i);
        }
        return res;
    }

    Mstate qt_vis_worker::ExtractFromMap(Mstate _cam_states, std::set<int> intersection) {
        Mstate res;
        for (auto iter = _cam_states.begin(); iter != _cam_states.end(); iter++) {
            int ID = iter->second.ID;
            double timestamp = iter->first;
            if (intersection.find(ID) != intersection.end())
                res[timestamp] = iter->second;
        }
        return res;
    }

    bool qt_vis_worker::RemoveLostFeatures(Matrix& H_x, Vector& r)
    {
        base_scopedtimer timer("RemoveLostFeatures()", TimeCostDebugOutFile, TimeCostDebugStatus);
        int pointNum = 0;
        int pointInitialfail = 0;
        int pointGatingTestfail = 0;
        int pointUpdate = 0;
        int jacobian_row_size = 0;
        std::vector<hwa_vis::FeatureIDType> invalid_feature_ids(0);
        std::vector<int> processed_feature_ids(0);

        cam_states = Imu2Cam(ins_states,
            CamAttr->T_cam0_imu.linear(),
            CamAttr->T_cam0_imu.translation());
        int curr_id = ins_states.rbegin()->second.ID;
        int ins_size = ins_states.size();
        std::vector<int> ins_id;
        ins_id.reserve(ins_size);
        for (int i = curr_id; i > curr_id - ins_size; --i) ins_id.push_back(i);

        for (auto iter = map_server.begin();
            iter != map_server.end(); ++iter)
        {
            auto& feature = iter->second;
            std::set<int> Intersection = JudgeArea(feature, ins_id);
            if (!feature.isLost || Intersection.size() == 0)
                continue;
            if (Intersection.size() < 5)
            {
                invalid_feature_ids.push_back(feature.id);
                continue;
            }
            pointNum++;
            if ((!feature.is_initialized) || ((feature.is_initialized) && (feature.is_initialized_NonKey)))
            {
                if (!feature.checkMotion(Intersection))
                {
                    invalid_feature_ids.push_back(feature.id);
                    pointInitialfail++;
                    std::cout << TimeStamp << " Remove Check Motion Failed!" << std::endl;
                    continue;
                }
                Mstate _cam_states = ExtractFromMap(cam_states, Intersection);
                if (!feature.triangulatePoint(_cam_states))
                {
                    invalid_feature_ids.push_back(feature.id);
                    pointInitialfail++;
                    std::cout << TimeStamp << " Remove Triangulated Failed!" << std::endl;
                    continue;
                }
                else
                {
                    if (!feature.initializePosition(_cam_states))
                    {
                        invalid_feature_ids.push_back(feature.id);
                        pointInitialfail++;
                        std::cout << TimeStamp << " Remove Initiallize Failed!" << std::endl;
                        continue;
                    }
                    else
                        feature.is_initialized_NonKey = false;
                }
            }
            int observationSize = Intersection.size();
            if (CamAttr->stereo) jacobian_row_size += 4 * observationSize - 3;
            else jacobian_row_size += 2 * observationSize - 3;
            processed_feature_ids.push_back(feature.id);
        }
        for (const auto& feature_id : invalid_feature_ids)
            map_server.erase(feature_id);

        if (processed_feature_ids.size() == 0)
            return false;

        H_x = Matrix::Zero(jacobian_row_size, 6 * cam_states.size() + CamAttr->ex_param_num);
        r = Vector::Zero(jacobian_row_size);
        int stack_cntr = 0;
        for (const auto& feature_id : processed_feature_ids)
        {
            auto& feature = map_server[feature_id];
            std::set<int> Intersection = JudgeArea(feature, ins_id);
            std::vector<int> cam_state_ids(0);
            for (int i : Intersection)
            {
                cam_state_ids.push_back(i);
            }
            Matrix H_xj;
            Vector r_j;

            if (!featureJacobian(feature.id, cam_state_ids, H_xj, r_j))
            {
                std::cout << TimeStamp << " Remove Feature Failed!" << std::endl;
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
                std::cout << TimeStamp << " Gating Test Fail" << std::endl;
                pointGatingTestfail++;
            }
            if (stack_cntr > 1500)
                break;
        }
        H_x.conservativeResize(stack_cntr, H_x.cols());
        r.conservativeResize((Eigen::Index)stack_cntr);
        if (H_x.rows() == 0) return false;

        for (const auto& feature_id : processed_feature_ids)
        {
            map_server.erase(feature_id);
        }
        return true;
    }

    bool qt_vis_worker::PruneCamState(Matrix& H_x, Vector& r)
    {
        base_scopedtimer timer("PruneCamState()", TimeCostDebugOutFile, TimeCostDebugStatus);
        if (ins_states.size() < vis_size)
            return false;
        int pointNum = 0;
        int pointInitialfail = 0;
        int pointGatingTestfail = 0;
        int pointUpdate = 0;
        int jacobian_row_size = 0;
        int rm_cam_id = ins_states.begin()->second.ID;

        cam_states = Imu2Cam(ins_states,
            CamAttr->T_cam0_imu.linear(),
            CamAttr->T_cam0_imu.translation());
        int curr_id = ins_states.rbegin()->second.ID;
        int ins_size = ins_states.size();
        std::vector<int> ins_id;
        ins_id.reserve(ins_size);
        for (int i = curr_id; i > curr_id - ins_size; --i) ins_id.push_back(i);

        std::vector<int> processed_feature_ids(0);
        for (auto& item : map_server)
        {
            auto& feature = item.second;
            std::set<int> Intersection = JudgeArea(feature, ins_id);
            if (Intersection.find(rm_cam_id) == Intersection.end()) continue;
            pointNum++;

            if ((!feature.is_initialized) || ((feature.is_initialized) && (feature.is_initialized_NonKey)))
            {
                if (!feature.checkMotion())
                {
                    feature.observations.erase(rm_cam_id);
                    pointInitialfail++;
                    continue;

                }
                if (!feature.triangulatePoint(cam_states))
                {
                    feature.observations.erase(rm_cam_id);
                    pointInitialfail++;
                    continue;
                }
                else
                {
                    if (!feature.initializePosition(cam_states))
                    {
                        feature.observations.erase(rm_cam_id);
                        pointInitialfail++;
                        continue;
                    }
                    else
                        feature.is_initialized_NonKey = false;
                }
            }

            if (CamAttr->stereo) jacobian_row_size += 4 * Intersection.size() - 3;
            else jacobian_row_size += 2 * Intersection.size() - 3;
            processed_feature_ids.push_back(feature.id);
            feature.is_initialized = false;
        }

        H_x = Matrix::Zero(jacobian_row_size, 6 * cam_states.size() + CamAttr->ex_param_num);
        r = Vector::Zero(jacobian_row_size);

        int stack_cntr = 0;
        for (const auto& feature_id : processed_feature_ids)
        {
            auto& feature = map_server[feature_id];
            std::set<int> Intersection = JudgeArea(feature, ins_id);
            std::vector<int> cam_state_ids(0);
            for (int i : Intersection)
            {
                cam_state_ids.push_back(i);
            }
            Matrix H_xj;
            Vector r_j;

            if (!featureJacobian(feature.id, cam_state_ids, H_xj, r_j))
            {
                std::cout << TimeStamp << " Remove Feature Failed!" << std::endl;
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
                std::cout << TimeStamp << " Gating Test Fail" << std::endl;
                pointGatingTestfail++;
            }

            if (stack_cntr > 1500)
                break;
        }
        H_x.conservativeResize(stack_cntr, H_x.cols());
        r.conservativeResize((Eigen::Index)stack_cntr);
        if (H_x.rows() == 0) return false;

        for (const auto& feature_id : processed_feature_ids)
        {
            auto& feature = map_server[feature_id];
            feature.observations.erase(rm_cam_id);
        }

        return true;
    }

    bool qt_vis_worker::GatingTest(const Matrix& H, const Vector& r, const int& dof)
    {
        int obs_size = H.rows();
        int N = cam_states.size();
        int par_size = 15 + 6 * N + CamAttr->ex_param_num;

        Matrix All_H = Matrix::Zero(H.rows(), par_size);
        assert(obs_size == r.rows());

        auto cam_state_iter = cam_states.begin();
        for (int i = 0; i < cam_states.size();
            ++i, ++cam_state_iter)
        {
            int idx = 15 + CamAttr->ex_param_num + 6 * i;
            All_H.block(0, idx, obs_size, 6) = H.block(0, 6 * i + CamAttr->ex_param_num, obs_size, 6);
        }
        if (CamAttr->estimate_extrinsic)
        {
            All_H.block(0, 15, obs_size, 6) = H.block(0, 0, obs_size, 6);
        }
        if (CamAttr->estimate_t)
        {
            All_H.block(0, 15 + CamAttr->ex_param_num - 1, obs_size, 1) = H.block(0, CamAttr->ex_param_num - 1, obs_size, 1);
        }
        Matrix P1 = All_H * _global_variance * All_H.transpose();
        Matrix P2 = CamAttr->feature_observation_noise * Matrix::Identity(All_H.rows(), All_H.rows());
        double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);
        if (gamma < chi_squared_test_table[dof]) {
            return true;
        }
        else {
            return false;
        }
    }

    void qt_vis_worker::meas_update(Matrix& H, Vector& r)
    {
        int obs_size = H.rows();
        int param_size = H.cols();
        int par_size = 15 + param_size;

        Matrix All_H = Matrix::Zero(H.rows(), par_size);
        assert(obs_size == r.rows());
        All_H.block(0, 15, obs_size, param_size) = H;

        if (All_H.rows() > All_H.cols())
        {
            Eigen::HouseholderQR<Matrix> qr_helper(All_H);
            Matrix Q = qr_helper.householderQ();
            Matrix Q1;
            Q1 = Q.leftCols(par_size);
            H = Q1.transpose() * All_H;
            r = Q1.transpose() * r;
        }
        else
        {
            H = All_H;
        }
    }

    bool qt_vis_worker::featureJacobian(const int feature_id,
        const std::vector<int>& cam_state_ids,
        Matrix& H_x, Vector& r)
    {

        const auto& feature = map_server[feature_id];
        std::vector<int> valid_cam_state_ids(0);
        for (const auto& cam_id : cam_state_ids)
        {
            if (feature.observations.find(cam_id) == feature.observations.end())
                continue;

            valid_cam_state_ids.push_back(cam_id);
        }

        int jacobian_row_size = 0;
        if (CamAttr->stereo) jacobian_row_size = 4 * valid_cam_state_ids.size();
        else jacobian_row_size = 2 * valid_cam_state_ids.size();

        Matrix H_xj;
        H_xj = Matrix::Zero(jacobian_row_size, cam_states.size() * 6 + CamAttr->ex_param_num);

        Matrix H_fj = Matrix::Zero(jacobian_row_size, 3);
        Vector r_j = Vector::Zero(jacobian_row_size);
        int stack_cntr = 0;
        if (CamAttr->stereo)
        {
            for (const auto& cam_id : valid_cam_state_ids)
            {
                Eigen::Matrix<double, 4, 6> H_xi = Eigen::Matrix<double, 4, 6>::Zero();
                Eigen::Matrix<double, 4, 6> H_xi_ex = Eigen::Matrix<double, 4, 6>::Zero();
                Eigen::Matrix<double, 4, 3> H_fi = Eigen::Matrix<double, 4, 3>::Zero();
                Eigen::Vector4d r_i = Eigen::Vector4d::Zero();

                if (CamAttr->estimate_extrinsic) measurementJacobianEX(cam_id, feature.id, H_xi, H_xi_ex, H_fi, r_i);
                else measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

                auto cam_state_iter = cam_states.find(cam_id);
                int cam_state_cntr;

                if (CamAttr->estimate_extrinsic) {
                    H_xj.block<4, 6>(stack_cntr, 0) = H_xi_ex;
                    cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter) + 1;
                }
                else
                    cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter);

                H_xj.block<4, 6>(stack_cntr, 6 * cam_state_cntr) = H_xi;
                H_fj.block<4, 3>(stack_cntr, 0) = H_fi;
                r_j.segment<4>(stack_cntr) = r_i;
                stack_cntr += 4;
            }
        }
        else
        {
            for (const auto& cam_id : valid_cam_state_ids)
            {
                Eigen::Matrix<double, 2, 6> H_xi = Eigen::Matrix<double, 2, 6>::Zero();
                Eigen::Matrix<double, 2, 3> H_fi = Eigen::Matrix<double, 2, 3>::Zero();
                Eigen::Matrix<double, 2, 6> H_xi_ex = Eigen::Matrix<double, 2, 6>::Zero();
                Eigen::Vector2d r_i = Eigen::Vector2d::Zero();

                if (CamAttr->estimate_extrinsic) measurementJacobianEX(cam_id, feature.id, H_xi, H_xi_ex, H_fi, r_i);
                else measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

                auto cam_state_iter = cam_states.find(cam_id);

                int cam_state_cntr;
                if (CamAttr->estimate_extrinsic) {
                    H_xj.block<2, 6>(stack_cntr, 0) = H_xi_ex;
                    cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter) + 1;
                }
                else
                    cam_state_cntr = std::distance(cam_states.begin(), cam_state_iter);


                H_xj.block<2, 6>(stack_cntr, 6 * cam_state_cntr) = H_xi;
                H_fj.block<2, 3>(stack_cntr, 0) = H_fi;
                r_j.segment<2>(stack_cntr) = r_i;
                stack_cntr += 2;
            }
        }

        Eigen::JacobiSVD<Matrix> svd_helper(H_fj, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Matrix A = svd_helper.matrixU().rightCols(
            jacobian_row_size - 3);

        H_x = A.transpose() * H_xj;
        r = A.transpose() * r_j;

        return true;
    }

    void qt_vis_worker::measurementJacobianEX(
        const int& cam_state_id,
        const int& feature_id,
        Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 6>& H_x_ex, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r)
    {
        const CamInfo& imu_state = ins_states[cam_state_id];
        const CamInfo& cam_state = cam_states[cam_state_id];
        const _OneFeature& feature = map_server[feature_id];

        SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
        const Triple& t_c0_w = cam_state.position;
        const Triple& t_i_w = imu_state.position;

        SO3 R_c0_i = CamAttr->T_cam0_imu.linear();
        SO3 R_i_c0 = R_c0_i.transpose();
        SO3 R_c0_c1 = CamAttr->T_cam0_cam1.linear();
        Triple t_c0_c1 = CamAttr->T_cam0_cam1.translation();
        SO3 R_w_c1 = CamAttr->T_cam0_cam1.linear() * R_w_c0;
        Triple t_c1_w = t_c0_w - R_w_c1.transpose() * t_c0_c1;

        const Triple& p_w = feature.position;
        const Eigen::Vector4d& z = feature.observations.find(cam_state_id)->second;

        Triple p_c0 = R_w_c0 * (p_w - t_c0_w);
        Triple p_c1 = R_w_c1 * (p_w - t_c1_w);

        Eigen::Matrix<double, 4, 3> dz_dpc0 = Eigen::Matrix<double, 4, 3>::Zero();
        dz_dpc0(0, 0) = 1 / p_c0(2);
        dz_dpc0(1, 1) = 1 / p_c0(2);
        dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
        dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

        Eigen::Matrix<double, 4, 3> dz_dpc1 = Eigen::Matrix<double, 4, 3>::Zero();
        dz_dpc1(2, 0) = 1 / p_c1(2);
        dz_dpc1(3, 1) = 1 / p_c1(2);
        dz_dpc1(2, 2) = -p_c1(0) / (p_c1(2) * p_c1(2));
        dz_dpc1(3, 2) = -p_c1(1) / (p_c1(2) * p_c1(2));

        Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 3, 6> dpc1_dxc = Eigen::Matrix<double, 3, 6>::Zero();

        Eigen::Matrix<double, 3, 6> dpc0_dxc_ex = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 3, 6> dpc1_dxc_ex = Eigen::Matrix<double, 3, 6>::Zero();

        if (EstimatorType == NORMAL) {

            dpc0_dxc.leftCols(3) = R_w_c0 * hwa_base::askew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = R_w_c1 * hwa_base::askew(p_w - t_i_w);
            dpc1_dxc.rightCols(3) = R_w_c1;

            dpc0_dxc_ex.leftCols(3) = -hwa_base::askew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            dpc1_dxc_ex.leftCols(3) = -hwa_base::askew(R_w_c1 * (p_w - t_c1_w)) + hwa_base::askew(t_c0_c1) * R_c0_c1;
            dpc1_dxc_ex.rightCols(3) = -R_c0_c1 * R_i_c0;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
        else if (EstimatorType == INEKF) {
            dpc0_dxc.leftCols(3) = -R_w_c0 * hwa_base::askew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = -R_w_c1 * hwa_base::askew(p_w + R_w_c1.transpose() * CamAttr->T_cam0_cam1.translation());
            dpc1_dxc.rightCols(3) = R_w_c1;

            dpc0_dxc_ex.leftCols(3) = -hwa_base::askew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            dpc1_dxc_ex.leftCols(3) = -hwa_base::askew(R_w_c1 * (p_w - t_c1_w)) + hwa_base::askew(t_c0_c1) * R_c0_c1;
            dpc1_dxc_ex.rightCols(3) = -R_c0_c1 * R_i_c0;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }

        SO3 dpc0_dpg = R_w_c0;
        SO3 dpc1_dpg = R_w_c1;

        H_x = dz_dpc0 * dpc0_dxc + dz_dpc1 * dpc1_dxc;
        H_f = dz_dpc0 * dpc0_dpg + dz_dpc1 * dpc1_dpg;
        H_x_ex = dz_dpc0 * dpc0_dxc_ex + dz_dpc1 * dpc1_dxc_ex;
    }


    void qt_vis_worker::measurementJacobian(
        const int& cam_state_id,
        const int& feature_id,
        Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r)
    {
        const CamInfo& imu_state = ins_states[cam_state_id];
        const CamInfo& cam_state = cam_states[cam_state_id];
        const _OneFeature& feature = map_server[feature_id];


        //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
        SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
        const  Triple& t_c0_w = cam_state.position;
        const  Triple& t_i_w = imu_state.position;


        SO3 R_c0_c1 = CamAttr->T_cam0_cam1.linear();
        SO3 R_w_c1 = CamAttr->T_cam0_cam1.linear() * R_w_c0;
        Triple t_c1_w = t_c0_w - R_w_c1.transpose() * CamAttr->T_cam0_cam1.translation();

        const Triple& p_w = feature.position;
        const  Eigen::Vector4d& z = feature.observations.find(cam_state_id)->second;

        Triple p_c0 = R_w_c0 * (p_w - t_c0_w);
        Triple p_c1 = R_w_c1 * (p_w - t_c1_w);

        Eigen::Matrix<double, 4, 3> dz_dpc0 = Eigen::Matrix<double, 4, 3>::Zero();
        dz_dpc0(0, 0) = 1 / p_c0(2);
        dz_dpc0(1, 1) = 1 / p_c0(2);
        dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
        dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

        Eigen::Matrix<double, 4, 3> dz_dpc1 = Eigen::Matrix<double, 4, 3>::Zero();
        dz_dpc1(2, 0) = 1 / p_c1(2);
        dz_dpc1(3, 1) = 1 / p_c1(2);
        dz_dpc1(2, 2) = -p_c1(0) / (p_c1(2) * p_c1(2));
        dz_dpc1(3, 2) = -p_c1(1) / (p_c1(2) * p_c1(2));

        Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 3, 6> dpc1_dxc = Eigen::Matrix<double, 3, 6>::Zero();

        if (EstimatorType == NORMAL) {
            dpc0_dxc.leftCols(3) = R_w_c0 * hwa_base::askew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = R_w_c1 * hwa_base::askew(p_w - t_i_w);
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }
        else if (EstimatorType == INEKF) {
            dpc0_dxc.leftCols(3) = -R_w_c0 * hwa_base::askew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc1_dxc.leftCols(3) = -R_w_c1 * hwa_base::askew(p_w + R_w_c1.transpose() * CamAttr->T_cam0_cam1.translation());
            dpc1_dxc.rightCols(3) = R_w_c1;

            r = z - Eigen::Vector4d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2),
                p_c1(0) / p_c1(2), p_c1(1) / p_c1(2));
        }


        SO3 dpc0_dpg = R_w_c0;
        SO3 dpc1_dpg = R_w_c1;

        H_x = dz_dpc0 * dpc0_dxc + dz_dpc1 * dpc1_dxc;
        H_f = dz_dpc0 * dpc0_dpg + dz_dpc1 * dpc1_dpg;
    }


    void qt_vis_worker::measurementJacobianEX(
        const int& cam_state_id,
        const int& feature_id,
        Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 6>& H_x_ex, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r)
    {
        const CamInfo& imu_state = ins_states[cam_state_id];
        const Triple& t_i_w = imu_state.position;
        SO3 R_c0_i = CamAttr->T_cam0_imu.linear();
        SO3 R_i_c0 = R_c0_i.transpose();

        const CamInfo& cam_state = cam_states[cam_state_id];
        const _OneFeature& feature = map_server[feature_id];

        //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
        SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
        const Triple& t_c0_w = cam_state.position;

        const Triple& p_w = feature.position;
        const Eigen::Vector2d z = Eigen::Vector2d(feature.observations.find(cam_state_id)->second(0),
            feature.observations.find(cam_state_id)->second(1));

        Triple p_c0 = R_w_c0 * (p_w - t_c0_w);
        Eigen::Matrix<double, 2, 3> dz_dpc0 = Eigen::Matrix<double, 2, 3>::Zero();
        dz_dpc0(0, 0) = 1 / p_c0(2);
        dz_dpc0(1, 1) = 1 / p_c0(2);
        dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
        dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

        Eigen::Matrix<double, 3, 6> dpc0_dxc_ex = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();

        if (EstimatorType == NORMAL) {

            dpc0_dxc.leftCols(3) = R_w_c0 * hwa_base::askew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc0_dxc_ex.leftCols(3) = -hwa_base::askew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
        else if (EstimatorType == INEKF) {
            dpc0_dxc.leftCols(3) = -R_w_c0 * hwa_base::askew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            dpc0_dxc_ex.leftCols(3) = -hwa_base::askew(R_w_c0 * (p_w - t_c0_w));
            dpc0_dxc_ex.rightCols(3) = -R_i_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }

        SO3 dpc0_dpg = R_w_c0;
        H_x = dz_dpc0 * dpc0_dxc;
        H_f = dz_dpc0 * dpc0_dpg;
    }

    void qt_vis_worker::measurementJacobian(
        const int& cam_state_id,
        const int& feature_id,
        Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r)
    {
        const CamInfo& imu_state = ins_states[cam_state_id];
        const Triple& t_i_w = imu_state.position;

        const CamInfo& cam_state = cam_states[cam_state_id];
        const _OneFeature& feature = map_server[feature_id];

        //SO3 R_w_c0 = cam_state.orientation.toRotationMatrix();
        SO3 R_w_c0 = cam_state.orientation.toRotationMatrix().transpose();
        const Triple& t_c0_w = cam_state.position;

        const Triple& p_w = feature.position;
        const Eigen::Vector2d z = Eigen::Vector2d(feature.observations.find(cam_state_id)->second(0),
            feature.observations.find(cam_state_id)->second(1));

        Triple p_c0 = R_w_c0 * (p_w - t_c0_w);


        Eigen::Matrix<double, 2, 3> dz_dpc0 = Eigen::Matrix<double, 2, 3>::Zero();
        dz_dpc0(0, 0) = 1 / p_c0(2);
        dz_dpc0(1, 1) = 1 / p_c0(2);
        dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2) * p_c0(2));
        dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2) * p_c0(2));

        Eigen::Matrix<double, 3, 6> dpc0_dxc = Eigen::Matrix<double, 3, 6>::Zero();


        if (EstimatorType == NORMAL) {

            dpc0_dxc.leftCols(3) = R_w_c0 * hwa_base::askew(p_w - t_i_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }
        else if (EstimatorType == INEKF) {
            dpc0_dxc.leftCols(3) = -R_w_c0 * hwa_base::askew(p_w);
            dpc0_dxc.rightCols(3) = R_w_c0;

            r = z - Eigen::Vector2d(p_c0(0) / p_c0(2), p_c0(1) / p_c0(2));
        }


        SO3 dpc0_dpg = R_w_c0;

        H_x = dz_dpc0 * dpc0_dxc;
        H_f = dz_dpc0 * dpc0_dpg;
    }

    void qt_vis_worker::readChisquare_test()
    {
        const double CHI_SQUARED_TABLE[1000] = {
            NAN,0.003932,0.102587,0.351846,0.710723,1.145476,1.635383,2.167350,2.732637,3.325113,3.940299,4.574813,5.226029,5.891864,6.570631,7.260944,7.961646,8.671760,9.390455,10.117013,
            10.850811,11.591305,12.338015,13.090514,13.848425,14.611408,15.379157,16.151396,16.927875,17.708366,18.492661,19.280569,20.071913,20.866534,21.664281,22.465015,23.268609,24.074943,24.883904,25.695390,
            26.509303,27.325551,28.144049,28.964717,29.787477,30.612259,31.438995,32.267622,33.098077,33.930306,34.764252,35.599864,36.437093,37.275893,38.116218,38.958027,39.801278,40.645933,41.491954,42.339308,
            43.187958,44.037874,44.889024,45.741377,46.594905,47.449581,48.305378,49.162270,50.020233,50.879243,51.739278,52.600315,53.462333,54.325312,55.189231,56.054072,56.919817,57.786447,58.653945,59.522294,
            60.391478,61.261482,62.132291,63.003888,63.876261,64.749396,65.623278,66.497895,67.373234,68.249284,69.126030,70.003463,70.881571,71.760343,72.639768,73.519835,74.400535,75.281858,76.163793,77.046332,
            77.929465,78.813184,79.697479,80.582343,81.467767,82.353742,83.240262,84.127317,85.014902,85.903008,86.791628,87.680755,88.570382,89.460503,90.351111,91.242200,92.133763,93.025794,93.918287,94.811237,
            95.704637,96.598482,97.492766,98.387485,99.282632,100.178202,101.074191,101.970593,102.867404,103.764618,104.662231,105.560239,106.458637,107.357420,108.256584,109.156124,110.056038,110.956320,111.856966,112.757973,
            113.659337,114.561053,115.463118,116.365529,117.268281,118.171372,119.074797,119.978553,120.882637,121.787046,122.691775,123.596823,124.502186,125.407860,126.313843,127.220131,128.126722,129.033613,129.940801,130.848283,
            131.756057,132.664118,133.572466,134.481097,135.390009,136.299198,137.208663,138.118401,139.028410,139.938687,140.849230,141.760036,142.671103,143.582429,144.494011,145.405848,146.317937,147.230276,148.142863,149.055696,
            149.968773,150.882091,151.795649,152.709445,153.623476,154.537742,155.452239,156.366967,157.281923,158.197105,159.112512,160.028141,160.943992,161.860062,162.776350,163.692854,164.609572,165.526502,166.443644,167.360995,
            168.278554,169.196320,170.114290,171.032463,171.950839,172.869414,173.788188,174.707160,175.626327,176.545689,177.465244,178.384991,179.304928,180.225055,181.145368,182.065869,182.986554,183.907423,184.828474,185.749707,
            186.671120,187.592712,188.514481,189.436426,190.358547,191.280841,192.203309,193.125948,194.048758,194.971737,195.894884,196.818199,197.741680,198.665326,199.589135,200.513108,201.437243,202.361538,203.285994,204.210608,
            205.135380,206.060309,206.985394,207.910634,208.836028,209.761574,210.687273,211.613123,212.539123,213.465273,214.391571,215.318016,216.244608,217.171346,218.098229,219.025255,219.952425,220.879738,221.807192,222.734786,
            223.662521,224.590394,225.518406,226.446555,227.374841,228.303263,229.231820,230.160512,231.089337,232.018295,232.947385,233.876607,234.805959,235.735442,236.665053,237.594793,238.524661,239.454656,240.384777,241.315025,
            242.245397,243.175894,244.106514,245.037258,245.968124,246.899112,247.830221,248.761451,249.692800,250.624269,251.555856,252.487562,253.419385,254.351325,255.283380,256.215552,257.147839,258.080240,259.012755,259.945383,
            260.878124,261.810977,262.743942,263.677018,264.610204,265.543500,266.476906,267.410421,268.344044,269.277775,270.211613,271.145558,272.079609,273.013766,273.948028,274.882395,275.816866,276.751441,277.686119,278.620900,
            279.555783,280.490768,281.425855,282.361042,283.296330,284.231717,285.167204,286.102791,287.038475,287.974258,288.910138,289.846116,290.782190,291.718361,292.654628,293.590990,294.527447,295.463998,296.400644,297.337384,
            298.274217,299.211143,300.148162,301.085272,302.022475,302.959769,303.897153,304.834628,305.772194,306.709849,307.647593,308.585427,309.523349,310.461359,311.399458,312.337643,313.275916,314.214276,315.152721,316.091253,
            317.029871,317.968574,318.907362,319.846234,320.785191,321.724231,322.663356,323.602563,324.541853,325.481226,326.420681,327.360218,328.299836,329.239536,330.179316,331.119177,332.059118,332.999139,333.939240,334.879420,
            335.819679,336.760016,337.700432,338.640926,339.581498,340.522147,341.462873,342.403677,343.344556,344.285512,345.226544,346.167651,347.108834,348.050092,348.991424,349.932831,350.874312,351.815868,352.757497,353.699199,
            354.640974,355.582822,356.524743,357.466736,358.408801,359.350937,360.293146,361.235425,362.177775,363.120196,364.062688,365.005249,365.947881,366.890582,367.833353,368.776192,369.719101,370.662078,371.605124,372.548238,
            373.491420,374.434669,375.377986,376.321370,377.264821,378.208339,379.151923,380.095574,381.039290,381.983072,382.926920,383.870833,384.814812,385.758855,386.702962,387.647135,388.591371,389.535671,390.480036,391.424463,
            392.368954,393.313508,394.258125,395.202805,396.147547,397.092352,398.037218,398.982146,399.927136,400.872188,401.817300,402.762474,403.707708,404.653003,405.598359,406.543774,407.489250,408.434785,409.380380,410.326035,
            411.271749,412.217521,413.163353,414.109243,415.055192,416.001199,416.947264,417.893387,418.839568,419.785806,420.732101,421.678454,422.624864,423.571330,424.517853,425.464433,426.411068,427.357760,428.304508,429.251312,
            430.198171,431.145085,432.092055,433.039079,433.986159,434.933293,435.880482,436.827725,437.775022,438.722373,439.669779,440.617237,441.564750,442.512316,443.459934,444.407606,445.355331,446.303109,447.250939,448.198822,
            449.146756,450.094743,451.042782,451.990873,452.939015,453.887209,454.835453,455.783750,456.732097,457.680495,458.628944,459.577443,460.525993,461.474593,462.423243,463.371943,464.320693,465.269492,466.218341,467.167240,
            468.116187,469.065184,470.014230,470.963325,471.912468,472.861660,473.810900,474.760189,475.709526,476.658910,477.608343,478.557823,479.507351,480.456926,481.406549,482.356219,483.305936,484.255699,485.205510,486.155367,
            487.105271,488.055221,489.005218,489.955260,490.905349,491.855484,492.805664,493.755890,494.706161,495.656478,496.606840,497.557248,498.507700,499.458197,500.408739,501.359326,502.309957,503.260632,504.211352,505.162116,
            506.112924,507.063776,508.014672,508.965612,509.916595,510.867621,511.818691,512.769805,513.720961,514.672160,515.623402,516.574687,517.526015,518.477385,519.428798,520.380253,521.331750,522.283290,523.234871,524.186494,
            525.138160,526.089866,527.041615,527.993404,528.945236,529.897108,530.849022,531.800976,532.752972,533.705008,534.657085,535.609203,536.561361,537.513560,538.465799,539.418078,540.370398,541.322757,542.275156,543.227596,
            544.180074,545.132593,546.085151,547.037748,547.990385,548.943061,549.895776,550.848530,551.801323,552.754155,553.707025,554.659934,555.612882,556.565868,557.518893,558.471956,559.425057,560.378196,561.331373,562.284588,
            563.237841,564.191131,565.144459,566.097825,567.051228,568.004669,568.958146,569.911661,570.865213,571.818802,572.772428,573.726091,574.679790,575.633526,576.587299,577.541108,578.494953,579.448835,580.402753,581.356707,
            582.310697,583.264723,584.218785,585.172883,586.127016,587.081185,588.035390,588.989630,589.943905,590.898216,591.852562,592.806943,593.761359,594.715810,595.670296,596.624817,597.579373,598.533963,599.488588,600.443247,
            601.397940,602.352668,603.307431,604.262227,605.217058,606.171922,607.126821,608.081753,609.036719,609.991719,610.946752,611.901820,612.856920,613.812054,614.767221,615.722422,616.677656,617.632923,618.588223,619.543556,
            620.498922,621.454320,622.409752,623.365216,624.320712,625.276242,626.231803,627.187398,628.143024,629.098683,630.054374,631.010097,631.965852,632.921639,633.877458,634.833309,635.789191,636.745106,637.701051,638.657029,
            639.613038,640.569078,641.525150,642.481253,643.437388,644.393553,645.349750,646.305977,647.262236,648.218525,649.174846,650.131197,651.087578,652.043991,653.000434,653.956907,654.913411,655.869946,656.826510,657.783105,
            658.739730,659.696386,660.653071,661.609786,662.566531,663.523307,664.480112,665.436946,666.393811,667.350705,668.307628,669.264581,670.221564,671.178576,672.135617,673.092688,674.049788,675.006917,675.964075,676.921262,
            677.878478,678.835723,679.792997,680.750299,681.707631,682.664991,683.622379,684.579796,685.537242,686.494716,687.452219,688.409750,689.367309,690.324897,691.282512,692.240156,693.197828,694.155528,695.113255,696.071011,
            697.028794,697.986606,698.944445,699.902311,700.860206,701.818127,702.776077,703.734054,704.692058,705.650089,706.608148,707.566234,708.524348,709.482488,710.440655,711.398850,712.357071,713.315320,714.273595,715.231897,
            716.190226,717.148582,718.106964,719.065373,720.023809,720.982271,721.940759,722.899274,723.857815,724.816383,725.774976,726.733596,727.692243,728.650915,729.609613,730.568338,731.527088,732.485864,733.444666,734.403494,
            735.362348,736.321228,737.280133,738.239063,739.198020,740.157002,741.116009,742.075042,743.034100,743.993183,744.952292,745.911426,746.870586,747.829770,748.788980,749.748214,750.707474,751.666758,752.626068,753.585402,
            754.544761,755.504145,756.463554,757.422987,758.382446,759.341928,760.301436,761.260967,762.220524,763.180104,764.139709,765.099339,766.058992,767.018670,767.978372,768.938099,769.897849,770.857624,771.817422,772.777245,
            773.737091,774.696962,775.656856,776.616774,777.576716,778.536682,779.496671,780.456684,781.416720,782.376781,783.336864,784.296971,785.257102,786.217256,787.177433,788.137634,789.097858,790.058105,791.018376,791.978669,
            792.938986,793.899326,794.859688,795.820074,796.780483,797.740915,798.701369,799.661847,800.622347,801.582870,802.543415,803.503984,804.464575,805.425188,806.385824,807.346483,808.307164,809.267868,810.228594,811.189342,
            812.150113,813.110906,814.071721,815.032559,815.993418,816.954300,817.915204,818.876130,819.837078,820.798048,821.759040,822.720054,823.681090,824.642147,825.603227,826.564328,827.525451,828.486596,829.447762,830.408950,
            831.370159,832.331391,833.292643,834.253917,835.215213,836.176530,837.137868,838.099228,839.060609,840.022011,840.983434,841.944879,842.906345,843.867832,844.829340,845.790869,846.752419,847.713990,848.675582,849.637196,
            850.598829,851.560484,852.522160,853.483856,854.445573,855.407311,856.369070,857.330849,858.292649,859.254469,860.216310,861.178172,862.140054,863.101956,864.063879,865.025822,865.987786,866.949770,867.911774,868.873798,
            869.835843,870.797908,871.759993,872.722098,873.684223,874.646369,875.608534,876.570720,877.532925,878.495150,879.457395,880.419660,881.381945,882.344250,883.306575,884.268919,885.231283,886.193666,887.156070,888.118493,
            889.080935,890.043397,891.005879,891.968380,892.930901,893.893441,894.856000,895.818579,896.781177,897.743795,898.706432,899.669088,900.631763,901.594458,902.557171,903.519904,904.482656,905.445427,906.408217,907.371027,
            908.333855,909.296702,910.259568,911.222453,912.185357,913.148279,914.111221,915.074181,916.037160,917.000158,917.963175,918.926210,919.889264,920.852336,921.815427,922.778537,923.741665,924.704812,925.667977,926.631161,
        };
        for (int i = 1; i < 1000; i++)
        {
            chi_squared_test_table[i] = CHI_SQUARED_TABLE[i];
        }
    }

}
