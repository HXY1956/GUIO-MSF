#include "hwa_fgo_lidarprocesser.h"

using namespace std;

namespace hwa_fgo {
    lidarprocesser::lidarprocesser(const baseprocesser& B, base_data* data) : baseprocesser(B, LIDAR_NODE),
        lidar_base(_gset.get()), _lidardata(dynamic_cast<lidar_data*>(data)) {
        beg.from_secs(dynamic_cast<set_lidar*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_lidar*>(_gset.get())->end());
        TimeStamp = beg;
        keyframe_trans_thresh = dynamic_cast<set_lidar*>(_gset.get())->keyframe_trans_thresh();
        keyframe_rot_thresh = dynamic_cast<set_lidar*>(_gset.get())->keyframe_rot_thresh();
    };

    lidarprocesser::lidarprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog, base_data* data, base_time _beg, base_time _end) : baseprocesser(gset, spdlog, site, LIDAR_NODE,_beg, _end),
        lidar_base(_gset.get()), _lidardata(dynamic_cast<lidar_data*>(data))
    {
        beg.from_secs(dynamic_cast<set_lidar*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_lidar*>(_gset.get())->end());
        TimeStamp = beg;
        keyframe_trans_thresh = dynamic_cast<set_lidar*>(_gset.get())->keyframe_trans_thresh();
        keyframe_rot_thresh = dynamic_cast<set_lidar*>(_gset.get())->keyframe_rot_thresh();
    };

    lidarprocesser::~lidarprocesser() {
        if (_global_map.isRun())
            _global_map.stop();
    };

    int lidarprocesser::ProcessOneEpoch()
    {
        double run_epoch = TimeStamp.sow() + TimeStamp.dsec();
        _lidarframe = lidarproc->PreProcessPointCloud(run_epoch);

        return LIDAR_MEAS;
    }

    bool lidarprocesser::_time_valid(base_time inst)
    {
        double _lidar_t = 0;
        if(_lidardata->load(inst.sow() + inst.dsec(), _lidar_t, _lidar_path))
            TimeStamp = base_time(TimeStamp.gwk(), _lidar_t);
        return std::abs(inst.diff(TimeStamp)) < _shm->delay;
    }

    bool lidarprocesser::load_data() {
        lidarproc->load_lidarobs(TimeStamp.sow() + TimeStamp.dsec(), _lidar_path);
        return true;
    }

    bool lidarprocesser::checkKeyframe(){
        double run_epoch = TimeStamp.sow() + TimeStamp.dsec();
        double dt = _sins->t - run_epoch;

        Triple BLH = _sins->pos - _sins->eth.v2dp(_sins->vn, dt);
        Triple XYZ = Geod2Cart(BLH, false);

        Triple imu_vel = _sins->vn - _sins->an * dt;//ENU
        const Eigen::Matrix3d& R_i_l = R_lidar_imu.transpose();
        const Triple& t_l_i = t_lidar_imu;
        const Eigen::Matrix3d& R_i_n = base_att_trans::q2mat(_sins->qnb);
        Eigen::Matrix3d R_n_i = R_i_n.transpose();
        Eigen::Matrix3d R_n_e = Cen(BLH);
        Eigen::Matrix3d R_e_i = R_n_i * R_n_e.transpose();
        Triple t_l_e = XYZ + R_n_e * R_i_n * t_l_i;
        Eigen::Matrix3d R_e_l = R_i_l * R_e_i;

        _lidarframe.id = lidar_state_id;
        _lidarframe.R_l_e = R_e_l.transpose();
        _lidarframe.t_l_e = t_l_e;
        _lidarframe.empty = false;

        isKeyframe = false;
        if (mIsFirstLidar) {
            _firstframe = _lidarframe;
            _lastframe = _lidarframe;
            mIsFirstLidar = false;
            isKeyframe = true;
            return true;
        }

        Triple trans_vec = _lidarframe.t_l_e - _lastframe.t_l_e;
        //std::cout << "cur std::vector: " << std::fixed << std::setprecision(3) << _lidarframe.t_l_e.transpose() << "\n";
        //std::cout << "last std::vector: " << std::fixed << std::setprecision(3) << _lastframe.t_l_e.transpose() << "\n";
        //std::cout << "trans std::vector: "<<std::fixed<<std::setprecision(3) << trans_vec.transpose() << "\n";
        double trans = trans_vec.norm();
        Eigen::Matrix3d dR = _lastframe.R_l_e.transpose() * _lidarframe.R_l_e;
        double rot = Eigen::AngleAxisd(dR).angle();
        if (trans > keyframe_trans_thresh || rot > keyframe_rot_thresh)
        {
            _lastframe = _lidarframe;
            isKeyframe = true;
        }

        return isKeyframe;
    }

    bool lidarprocesser::_gatingTest(Matrix& H, Vector& r, const int& dof, bool is_scan)
    {
        int obs_count = H.rows();

        if (H.rows() == 0) return false;

        assert(H.rows() == r.size());
        std::vector<int> indics;
        int pass_count = 0;
        for (int i = 0; i < obs_count; i++)
        {
            if (lidarproc->estimate_extrinsic)
            {
                assert(H.cols() == lidar_states.size() * 6 + 6);
            }
            else
            {
                assert(H.cols() == lidar_states.size() * 6);
            }
            Matrix onerow_H = H.block(i, 0, 1, H.cols());
            Vector onerow_r = r.block(i, 0, 1, r.cols());
            int par_size = param_of_sins->parNumber();
            Matrix All_H = Matrix::Zero(onerow_H.rows(), par_size);

            auto lidar_state_iter = lidar_states.begin();
            for (int i = 0; i < lidar_states.size();
                ++i, ++lidar_state_iter)
            {
                std::string lidar_id = to_string(lidar_state_iter->first);
                int idx = param_of_sins->getParam(_name, par_type::LIDAR_ATT_X, lidar_id);
                if (lidarproc->estimate_extrinsic)
                {
                    All_H.block(0, idx, 1, 6) = onerow_H.block(0, 6 * i + 6, 1, 6);
                }
                else
                {
                    All_H.block(0, idx, 1, 6) = onerow_H.block(0, 6 * i, 1, 6);
                }
            }
            Matrix P1 = All_H * _sins->Pk * All_H.transpose();
            double P1_number = P1(0, 0);
            Matrix P2;
            if (is_scan)
                P2 = scan_observation_noise * Matrix::Identity(All_H.rows(), All_H.rows());
            else
                P2 = map_observation_noise * Matrix::Identity(All_H.rows(), All_H.rows());
            double P2_number = P2(0, 0);
            double P_number = 1 / (P1_number + P2_number);

            double onerow_r_number = onerow_r(0);

            // calculate weighted average of the residuals
            double direct_gamma = onerow_r_number * P_number * onerow_r_number;

            if (direct_gamma < chi_squared_test_table[dof])
            {
                indics.push_back(1);
                pass_count++;
            }
            else
            {
                indics.push_back(0);
            }
        }
        cout << "all_obs_size:" << r.size() << "  pass_size:" << pass_count << "  pass_per:" << double(pass_count) / double(r.size()) << endl;
        cout << "before_remove:" << H.rows() << "," << r.size() << endl;
        for (int i = 0; i < indics.size(); i++)
        {
            if (indics.at(i) == 0)
            {
                removeRow(H, i);
                removeRow(r, i);
            }
        }
        cout << "after_remove:" << H.rows() << "," << r.size() << endl;
        return true;
    }

    void lidarprocesser::add_lidarframe(LidarFrame& frame)
    {
        if (lidar_buffer.size() < window_size + 1)
        {
            lidar_buffer.push_back(frame);
        }
        else
        {
            cout << "there is some thing error with lidar_buffer size!" << endl;
            getchar();
        }
    }

    void lidarprocesser::getAllOdoResidual(Matrix& H, Vector& r, bool use_3d, float ther)
    {
        cout << "lidar buffer size:" << lidar_buffer.size() << endl;
        for (int i = 1; i < lidar_buffer.size() - 1; i++)
        {
            Matrix temp_H;
            Vector temp_r;
            _lidarOdo.process(lidar_buffer[i], lidar_buffer[i + 1]);
            lidarMeasurementJacobian(_lidarOdo.lidarOdoObs, temp_H, temp_r, i, use_3d, ther);

            if (i == 1)
            {
                H = temp_H;
                r = temp_r;
            }
            else
            {
                Matrix vTemp_H;
                Vector vTemp_r;

                vTemp_H.resize(H.rows() + temp_H.rows(), H.cols());
                vTemp_r.resize(r.size() + temp_r.size());

                vTemp_H << H,
                    temp_H;
                vTemp_r << r,
                    temp_r;

                H.resize(vTemp_H.rows(), vTemp_H.cols());
                r.resize(vTemp_r.rows(), vTemp_r.cols());
                H = vTemp_H;
                r = vTemp_r;
            }
        }
    }

    void lidarprocesser::ProjOdoResidual(Matrix& H, Vector& r, bool use_3d, float ther)
    {
        int buf_size = lidar_buffer.size();
        int cur_id = buf_size - 1;

        for (int i = 1; i < lidar_buffer.size() - 1; i++)
        {
            Matrix tmp_H; Vector tmp_r;

            auto cur_project_lidarframe = lidar_buffer[i];
            int project_id = i;

            _lidarOdo.process(cur_project_lidarframe, lidar_buffer[cur_id]);
            lidarMeasurementJacobian(_lidarOdo.lidarOdoObs, tmp_H, tmp_r, project_id, cur_id, use_3d, ther);
            if (i == 1)
            {
                H = tmp_H;
                r = tmp_r;
            }
            else
            {
                Matrix back_H = H;
                Vector back_r = r;
                H.resize(back_H.rows() + tmp_H.rows(), back_H.cols());
                r.resize(back_r.size() + tmp_r.size());
                H << back_H,
                    tmp_H;
                r << back_r,
                    tmp_r;
            }
        }

        _gatingTest(H, r, 2, true);
    }

    void lidarprocesser::build_PPHR(std::vector<LidarFrame>& buffer, map<int, std::map<int, int>>& indexs, Matrix& H_x, Vector& r)
    {
        int eft_buf_size = buffer.size() - 2;
        int eft_number = 0;
        int eft_gatingtest_number = 0;
        int stack_cntr = 0;

        int max_jacobian_row_size = 3 * eft_buf_size * indexs.size();

        if (lidarproc->estimate_extrinsic)
        {
            H_x = Matrix::Zero(max_jacobian_row_size, 6 * lidar_states.size() + 6);
            r = Vector::Zero(max_jacobian_row_size);
        }
        else
        {
            H_x = Matrix::Zero(max_jacobian_row_size, 6 * lidar_states.size());
            r = Vector::Zero(max_jacobian_row_size);
        }

        map<int, std::map<int, int>>::iterator iter;
        for (iter = indexs.begin(); iter != indexs.end(); iter++)
        {
            Matrix H_xj;
            Vector r_j;

            if (planarpatchJacobian(buffer, iter->first, iter->second, H_xj, r_j))
            {
                eft_number++;
            }
            if (_gatingTest(H_xj, r_j, r_j.rows(), scan_observation_noise))
            {
                H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
                r.segment(stack_cntr, r_j.rows()) = r_j;
                stack_cntr += H_xj.rows();
                eft_gatingtest_number++;
            }
        }

        cout << "buildHR_ids_size( full observation):" << eft_number << endl;
        cout << "pass_gatingtest_jacobian_size:" << stack_cntr << endl;
        H_x.conservativeResize(stack_cntr, H_x.cols());
        r.conservativeResize((Eigen::Index)stack_cntr);
    }

    bool lidarprocesser::_gatingTest(Matrix& H, Vector& r, const int& dof, double obs_noise)
    {
        int par_size = param_of_sins->parNumber();
        int obs_size = H.rows();
        int N = lidar_buffer.size();

        Matrix All_H = Matrix::Zero(H.rows(), par_size);

        assert(obs_size == r.rows());

        auto lidar_state_iter = lidar_states.begin();
        for (int i = 0; i < lidar_states.size();
            ++i, ++lidar_state_iter)
        {
            std::string lidar_id = to_string(lidar_state_iter->first);
            int idx = param_of_sins->getParam(_name, par_type::LIDAR_ATT_X, lidar_id);
            if (lidarproc->estimate_extrinsic)
            {
                All_H.block(0, idx, obs_size, 6) = H.block(0, 6 * i + 6, obs_size, 6);
            }
            else
            {
                All_H.block(0, idx, obs_size, 6) = H.block(0, 6 * i, obs_size, 6);
            }
        }
        Matrix P1 = All_H * _sins->Pk * All_H.transpose();

        Matrix P2 = obs_noise * Matrix::Identity(All_H.rows(), All_H.rows());
        // calculate weighted average of the residuals
        double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);

  //      std::cout << "gating test rx: " << std::fixed << std::setprecision(3) << r.transpose() << " < " << chi_squared_test_table[dof] << " ?" << endl;
  //      std::cout << "gating test Px:\n" << std::fixed << std::setprecision(2) << (P1 + P2).inverse() << "\n";
		//std::cout << "gating test detail: " << std::fixed << std::setprecision(3) << gamma << " < " << chi_squared_test_table[dof] << " ?" << endl;

        if (gamma < chi_squared_test_table[dof]) {
            return true;
        }
        else {
            return false;
        }
    }

    bool lidarprocesser::planarpatchJacobian(std::vector<LidarFrame>& buffer, int point_id, std::map<int, int> corr_ids, Matrix& H, Vector& r)
    {
        int par_number = 0;
        if (lidarproc->estimate_extrinsic) par_number = buffer.size() * 6 + 6;
        else par_number = buffer.size() * 6;

        int jacobian_row_size = 3 * corr_ids.size();
        Matrix H_phi = Matrix::Zero(jacobian_row_size, 3);
        Matrix H_x = Matrix::Zero(jacobian_row_size, par_number);
        Vector r_j = Vector::Zero(jacobian_row_size);

        int stack_cntr = 0;
        for (auto iter : corr_ids)
        {
            Matrix H_nd_j = Matrix::Zero(3, 4);
            Matrix H_phi_j = Matrix::Zero(4, 3);
            Matrix H_x_j = Matrix::Zero(3, par_number);
            auto& oldest_lidar = buffer.at(1);
            auto& cur_lidar = buffer.at(iter.first);

            Eigen::Matrix3d R_old_cur = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e;
            Triple t_old_cur = cur_lidar.R_l_e.transpose() * (oldest_lidar.t_l_e - cur_lidar.t_l_e);
            Triple t_cur_old = oldest_lidar.R_l_e.transpose() * (cur_lidar.t_l_e - oldest_lidar.t_l_e);

            auto& oldest_id = point_id;
            auto& oldest_nc = oldest_lidar.ncs.at(oldest_id);
            auto& oldest_pc = oldest_lidar.pcs.at(oldest_id);
            auto& cur_nc = cur_lidar.ncs.at(iter.second);
            auto& cur_pc = cur_lidar.pcs.at(iter.second);

            Triple oldest_phi = oldest_nc * (oldest_nc.transpose() * oldest_pc);
            Triple oldest_n = oldest_phi / oldest_phi.norm();
            Triple cur_phi = cur_nc * (cur_nc.transpose() * cur_pc);
            Triple cur_n = cur_phi / cur_phi.norm();
            Triple residual = cur_phi - R_old_cur * oldest_n * (oldest_phi.norm() - t_cur_old.transpose() * oldest_n);

            //H_nd_j.block(0, 0, 3, 3) = R_old_cur * (Eigen::Matrix3d::Identity() * (oldest_phi.norm() - t_cur_old.transpose() * oldest_n) - oldest_n * t_cur_old.transpose());
            //H_nd_j.block(0, 3, 3, 1) = R_old_cur * oldest_n;
            //H_phi_j.block(0, 0, 3, 3) = 1.0 / oldest_phi.norm() * (Eigen::Matrix3d::Identity() - oldest_n * oldest_n.transpose());
            //H_phi_j.block(3, 0, 1, 3) = oldest_n.transpose();

            if (lidarproc->estimate_extrinsic)
            {
				// jacobian on oldest lidar state R_l_e and t_l_e;
                H_x_j.block(0, 6 + 1 * 6, 3, 3) = 
                    - cur_lidar.R_l_e.transpose() * skew(oldest_lidar.R_l_e * oldest_n * (oldest_phi.norm() - oldest_n.transpose() * t_cur_old))
                    - R_old_cur * oldest_n * oldest_n.transpose() * oldest_lidar.R_l_e.transpose() * skew(cur_lidar.t_l_e - oldest_lidar.t_l_e);
                H_x_j.block(0, 6 + 1 * 6 + 3, 3, 3) =
                    - R_old_cur * oldest_n * oldest_n.transpose() * oldest_lidar.R_l_e.transpose();
                // jacobian on current lidar state R_l_e and t_l_e;
                H_x_j.block(0, 6 + iter.first * 6, 3, 3) =
                    cur_lidar.R_l_e.transpose() * skew(oldest_lidar.R_l_e * oldest_n * (oldest_phi.norm() - oldest_n.transpose() * t_cur_old));
                H_x_j.block(0, 6 + iter.first * 6 + 3, 3, 3) =
                    R_old_cur * oldest_n * oldest_n.transpose() * oldest_lidar.R_l_e.transpose();
            }
            else
            {
                // jacobian on oldest lidar state R_l_e and t_l_e;
                H_x_j.block(0, 6, 3, 3) =
                    - cur_lidar.R_l_e.transpose() * skew(oldest_lidar.R_l_e * oldest_n * (oldest_phi.norm() - oldest_n.transpose() * t_cur_old))
                    - R_old_cur * oldest_n * oldest_n.transpose() * oldest_lidar.R_l_e.transpose() * skew(cur_lidar.t_l_e - oldest_lidar.t_l_e);
                H_x_j.block(0, 6 + 3, 3, 3) =
                    - R_old_cur * oldest_n * oldest_n.transpose() * oldest_lidar.R_l_e.transpose();
                // jacobian on current lidar state R_l_e and t_l_e;
                H_x_j.block(0, iter.first * 6, 3, 3) =
                    cur_lidar.R_l_e.transpose() * skew(oldest_lidar.R_l_e * oldest_n * (oldest_phi.norm() - oldest_n.transpose() * t_cur_old));
                H_x_j.block(0, iter.first * 6 + 3, 3, 3) =
                    R_old_cur * oldest_n * oldest_n.transpose() * oldest_lidar.R_l_e.transpose();
            }

            H_phi.block(stack_cntr, 0, 3, 3) = H_nd_j * H_phi_j;
            H_x.block(stack_cntr, 0, 3, par_number) = H_x_j;
            r_j.segment<3>(stack_cntr) = residual;
            stack_cntr += 3;
        }

        //Eigen::JacobiSVD<Matrix> svd_helper(H_phi, Eigen::ComputeFullU | Eigen::ComputeThinV);
        //Matrix A = svd_helper.matrixU().rightCols(jacobian_row_size - 3);

        //std::cout << "before marginalization Hx:\n "<<std::fixed<<std::setprecision(3) << H_x << "\n";
        //std::cout << "before marginalization Rx:\n " << std::fixed << std::setprecision(3) << r_j.transpose() << "\n";

        //H = A.transpose() * H_x;
        //r = A.transpose() * r_j;

        //std::cout << "feature matrix:\n " << std::fixed << std::setprecision(3) << H_phi << "\n";
        //std::cout << "marginalization matrix:\n " << std::fixed << std::setprecision(3) << A.transpose() << "\n";
        //std::cout << "after marginalization Hx:\n " << std::fixed << std::setprecision(3) << H << "\n";
        //std::cout << "after marginalization Rx:\n " << std::fixed << std::setprecision(3) << r.transpose() << "\n";

        H = H_x;
        r = r_j;

        return true;
    }

    void lidarprocesser::removeLidar(std::vector<LidarStateIDType>& rm_lidar_state_ids)
    {
        if (rm_lidar_state_ids.size() == 0)
            return;
        removeState(rm_lidar_state_ids);
        removeIDandBuffer(rm_lidar_state_ids);
    }

    void lidarprocesser::removeState(std::vector<LidarStateIDType>& rm_lidar_state_ids)
    {
        if (rm_lidar_state_ids.size() <= 0)
            return;
        for (const auto& lidar_id : rm_lidar_state_ids)
        {
            Matrix tmp_Qx = _sins->Pk;
            Vector _dx = _sins->Xk;
            LidarStateIDType _lidar_id = lidar_id;
            std::string string_id = to_string(_lidar_id);
            int idx_att_x = param_of_sins->getParam(_name, hwa_base::par_type::LIDAR_ATT_X, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_att_x).index, param_of_sins->operator[](idx_att_x).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_att_x).index);
            param_of_sins->delParam(idx_att_x);
            param_of_sins->reIndex();

            int idx_att_y = param_of_sins->getParam(_name, hwa_base::par_type::LIDAR_ATT_Y, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_att_y).index, param_of_sins->operator[](idx_att_y).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_att_y).index);
            param_of_sins->delParam(idx_att_y);
            param_of_sins->reIndex();

            int idx_att_z = param_of_sins->getParam(_name, hwa_base::par_type::LIDAR_ATT_Z, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_att_z).index, param_of_sins->operator[](idx_att_z).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_att_z).index);
            param_of_sins->delParam(idx_att_z);
            param_of_sins->reIndex();

            int idx_crd_x = param_of_sins->getParam(_name, hwa_base::par_type::LIDAR_CRD_X, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_crd_x).index, param_of_sins->operator[](idx_crd_x).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_crd_x).index);
            param_of_sins->delParam(idx_crd_x);
            param_of_sins->reIndex();

            int idx_crd_y = param_of_sins->getParam(_name, hwa_base::par_type::LIDAR_CRD_Y, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_crd_y).index, param_of_sins->operator[](idx_crd_y).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_crd_y).index);
            param_of_sins->delParam(idx_crd_y);
            param_of_sins->reIndex();

            int idx_crd_z = param_of_sins->getParam(_name, hwa_base::par_type::LIDAR_CRD_Z, string_id);
            hwa_base::Matrix_remRC(tmp_Qx, param_of_sins->operator[](idx_crd_z).index, param_of_sins->operator[](idx_crd_z).index);
            hwa_base::remR(_dx, param_of_sins->operator[](idx_crd_z).index);
            param_of_sins->delParam(idx_crd_z);
            param_of_sins->reIndex();

            _sins->Pk = tmp_Qx;
            _sins->Xk = _dx;
            lidar_states.erase(lidar_id);
        }
    }

    void lidarprocesser::removeIDandBuffer(std::vector<LidarStateIDType>& rm_lidar_state_ids)
    {
        std::vector<LidarFrame> vTemplidar;
        bool isRemove;
        for (int i = 0; i < lidar_buffer.size(); i++)
        {
            isRemove = false;

            for (int j = 0; j < rm_lidar_state_ids.size(); j++)
            {
                if (lidar_buffer[i].id == rm_lidar_state_ids[j])
                {
                    isRemove = true;
                }
            }
            if (!isRemove)
            {
                vTemplidar.push_back(lidar_buffer[i]);
            }
        }
        lidar_buffer = vTemplidar;
    }

    int lidarprocesser::updateLidarBuffer()
    {
        if (lidar_states.size() == 0) return 1;
        if (lidar_buffer.size() != lidar_states.size())
        {
            cout << "Error:the quantity in lidar_states and lidar_buffer is not same!" << endl;
            getchar();
            return false;
        }
        auto lidar_state_iter = lidar_states.begin();
        for (int i = 0; i < lidar_states.size();
            ++i, ++lidar_state_iter)
        {
            lidar_buffer[i].R_l_e = lidar_state_iter->second.orientation;
            lidar_buffer[i].t_l_e = lidar_state_iter->second.position;
        }

        if (use_map)
        {
            _lidarmap.first_R_l_e = lidar_states.begin()->second.orientation.toRotationMatrix();
            _lidarmap.first_t_l_e = lidar_states.begin()->second.position;
        }
        return 1;
    }
}

