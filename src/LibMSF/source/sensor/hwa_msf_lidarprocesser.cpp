#include "hwa_msf_lidarprocesser.h"

using namespace std;

namespace hwa_msf {
    lidarprocesser::lidarprocesser(const baseprocesser& B, base_data* data) : baseprocesser(B),
        lidar_base(_gset.get()), _lidardata(dynamic_cast<lidar_data*>(data)) {
        beg.from_secs(dynamic_cast<set_lidar*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_lidar*>(_gset.get())->end());
        TimeStamp = beg;
    };

    lidarprocesser::lidarprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog, base_data* data, base_time _beg, base_time _end) : baseprocesser(gset, spdlog, site, _beg, _end),
        lidar_base(_gset.get()), _lidardata(dynamic_cast<lidar_data*>(data))
    {
        beg.from_secs(dynamic_cast<set_lidar*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_lidar*>(_gset.get())->end());
        TimeStamp = beg;
    };

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

    void lidarprocesser::StateAugmentation()
    {
        double run_epoch = TimeStamp.sow() + TimeStamp.dsec();
        double dt = _sins->t - run_epoch;

        //compensate due to time 
        Triple BLH = _sins->pos - _sins->eth.v2dp(_sins->vn, dt);//BLH
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
        //maintain the numerical stability of derivatives

        lidar_states[lidar_state_id] = LIDARState(lidar_state_id);
        LIDARState& lidar_state = lidar_states[lidar_state_id];

        lidar_state.time = run_epoch;
        lidar_state.orientation = R_e_l.transpose();
        lidar_state.position = t_l_e;

        auto iter_lidar = --lidar_states.end();
        _lidarframe.id = lidar_state_id;
        _lidarframe.R_l_e = iter_lidar->second.orientation;
        _lidarframe.t_l_e = iter_lidar->second.position;
        _lidarframe.empty = false;
        ///< distort correction
        if (use_corrdistort)
        {
            _lidarOdo.removeDistortion(_lastframe, _lidarframe);
        }

        if (lidar_buffer.size() < window_size + 1)
        {
            lidar_buffer.push_back(_lidarframe);
        }
        else
        {
            cout << "there is some thing error with lidar_buffer size!" << endl;
            getchar();
        }

        string lidar_id = to_string(lidar_state.id);
        base_par att_x_par(_name, par_type::LIDAR_ATT_X, param_of_sins->parNumber(), lidar_id);
        param_of_sins->addParam(att_x_par);
        base_par att_y_par(_name, par_type::LIDAR_ATT_Y, param_of_sins->parNumber() + 1, lidar_id);
        param_of_sins->addParam(att_y_par);
        base_par att_z_par(_name, par_type::LIDAR_ATT_Z, param_of_sins->parNumber() + 2, lidar_id);
        param_of_sins->addParam(att_z_par);

        base_par crd_x_par(_name, par_type::LIDAR_CRD_X, param_of_sins->parNumber() + 3, lidar_id);
        param_of_sins->addParam(crd_x_par);
        base_par crd_y_par(_name, par_type::LIDAR_CRD_Y, param_of_sins->parNumber() + 4, lidar_id);
        param_of_sins->addParam(crd_y_par);
        base_par crd_z_par(_name, par_type::LIDAR_CRD_Z, param_of_sins->parNumber() + 5, lidar_id);
        param_of_sins->addParam(crd_z_par);
        param_of_sins->reIndex();
        //int ins_size = nq;

        //time update
        size_t old_rows = _sins->Pk.rows();
        size_t old_cols = _sins->Pk.cols();
        Matrix J = Matrix::Zero(6, old_cols);

        if (_Estimator == NORMAL) {
            //lidar att to imu att
            J.block<3, 3>(0, 0) = SO3::Identity();
            //lidar pos to imu att
            J.block<3, 3>(3, 0) = askew(R_n_e * R_i_n * t_l_i);
        }
        else if (_Estimator == INEKF) {
            //lidar att to imu att
            J.block(0, 0, 3, 3) = SO3::Identity();
        }

        //lidar pos to imu pos
        J.block<3, 3>(3, 6) = SO3::Identity();

        if (lidarproc->estimate_extrinsic)
        {
            int index = param_of_sins->getParam(_name, par_type::EX_LIDAR_ATT_X, "");
            if (_Estimator == NORMAL) {
                J.block<3, 3>(0, index) = -R_e_l.transpose();
                J.block<3, 3>(3, index + 3) = -R_e_i.transpose();
            }
            else if (_Estimator == INEKF) {
                J.block(0, index, 3, 3) = R_e_l.transpose();
                J.block(3, index + 3, 3, 3) = -R_e_i.transpose();
            }
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
                cout << "PK warning :" << i << "," << setprecision(6) << _sins->Pk(i, i) << endl;
                cin.get();
            }
        }
        //keep positive definite of matrix
        _sins->Pk.block<3, 3>(old_rows, old_cols) += Matrix::Identity(3, 3) * 1e-12;
        _sins->Pk.block<3, 3>(old_rows + 3, old_cols + 3) += Matrix::Identity(3, 3) * 1e-10;
    }

    int lidarprocesser::ProcessOneEpoch()
    {
        double run_epoch = TimeStamp.sow() + TimeStamp.dsec();
        _lidarframe = lidarproc->PreProcessPointCloud(run_epoch);

        if (!use_scan && !use_map && !use_pp)
        {
            std::cout << "Warning: if lidar measurement is needed, you should chose at least one mode from LIDAR_SCAN or LIDAR_MAP!" << std::endl;
            getchar();
            return NO_MEAS;
        }
        if (_lidarframe.fullCloud.points.size() <= 0)
        {
            std::cout << "Error: There are no lidar points read in!" << endl;
            getchar();
            return NO_MEAS;
        }

        lidar_state_id = lidar_next_id++; //put this ahead

        if (mIsFirstLidar)
        {
            double dt = _sins->t - run_epoch;

            //compensate due to time 
            Triple BLH = _sins->pos - _sins->eth.v2dp(_sins->vn, dt);//BLH
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


            _lidarframe.R_l_e = R_e_l.transpose();
            _lidarframe.t_l_e = t_l_e;
            _lidarframe.empty = false;

            _lastframe = _lidarframe;
            mIsFirstLidar = false;

            return LIDAR_MEAS;
        }

        StateAugmentation();

        if (use_pp)
        {
            if (!_lidarOdo.systemInited_)
            {
                _lidarOdo.systemInited_ = true;

                return LIDAR_MEAS;
            }
            else
            {
                if (lidar_buffer.size() > 3)
                {
                    Matrix H;
                    Vector r;
                    _lidarOdo.data_association(lidar_buffer);
                    build_PPHR(lidar_buffer, _lidarOdo.associations, H, r);
                    _meas_update(H, r, true);
                }
            }
        }


        if (use_scan)
        {
            if (!_lidarOdo.systemInited_)
            {
                _lidarOdo.systemInited_ = true;
            }
            else
            {
                Matrix H;
                Vector r;

                if (lidar_buffer.size() > 2)
                {
                    ProjOdoResidual(H, r, false);
                    _meas_update(H, r, true);
                }
            }
        }

        if (use_map)
        {

            if (lidar_buffer.size() >= window_size + 1)
            {
                if (!_lidarmap.systemInited_)
                {
                    _lidarmap.process(lidar_buffer.at(0));
                    _lidarmap.addPointcloudToMap(lidar_buffer.at(0));
                    //_lidarmap.addPointcloudToMap(_lidarframe);
                }
                else
                {
                    //_lidarmap.process(lidar_buffer.at(1));
                    _lidarmap.process(_lidarframe);
                    Matrix H;
                    Vector r;
                    lidarMeasurementJacobian(_lidarmap.lidarMapObs, H, r, window_size, false, 1);
                    _gatingTest(H, r, 2, false);
                    _meas_update(H, r, false);
                    _lidarmap.addPointcloudToMap(lidar_buffer.at(1));

                }
            }
        }

        vector<LidarStateIDType> rm_lidar_state_ids(0);
        if (lidar_buffer.size() >= window_size + 1)
        {

            //remove the second oldest lidarframe

            rm_lidar_state_ids.push_back((++lidar_buffer.begin())->id);
        }
        //remove corresponding lidar frame
        removeLidar(rm_lidar_state_ids);

        _lastframe = _lidarframe;

        return LIDAR_MEAS;
    }

    void lidarprocesser::_feed_back()
    {
        if (lidarproc->estimate_extrinsic)
        {
            int index = param_of_sins->getParam(_name, par_type::EX_LIDAR_ATT_X, "");
            if (index < 0) cerr << "Extrinsic Update Error!" << endl;
            Triple att_dx = Triple(_sins->Xk(index), _sins->Xk(index + 1), _sins->Xk(index + 2));
            Triple crd_dx = Triple(_sins->Xk(index + 3), _sins->Xk(index + 4), _sins->Xk(index + 5));
            base_quat dq_lidar = base_att_trans::rv2q(-att_dx);
            Eigen::Quaterniond _dq_ext;
            _dq_ext.w() = dq_lidar.q0; _dq_ext.x() = dq_lidar.q1; _dq_ext.y() = dq_lidar.q2; _dq_ext.z() = dq_lidar.q3;
            _dq_ext.normalize();
            this->R_lidar_imu = R_lidar_imu * _dq_ext;
            this->t_lidar_imu -= crd_dx;
            this->T_lidar_imu.linear() = R_lidar_imu;
            this->T_lidar_imu.translation() = t_lidar_imu;
            lidarproc->T_lidar_imu = this->T_lidar_imu;
            lidarproc->R_lidar_imu = this->R_lidar_imu;
            lidarproc->t_lidar_imu = this->t_lidar_imu;

        }
        if (lidar_states.size() > 0)
        {
            auto lidar_state_iter = lidar_states.begin();
            for (int i = 0; i < lidar_states.size();
                ++i, ++lidar_state_iter)
            {
                LidarStateIDType cur_id = lidar_state_iter->first;
                string _lidarid = to_string(cur_id);

                int idx = param_of_sins->getParam(_name, par_type::LIDAR_ATT_X, _lidarid);
                if (idx < 0) cerr << "lidar_feedback() idx wrong!" << endl;

                Triple att_dx = Triple(_sins->Xk(idx), _sins->Xk(idx + 1), _sins->Xk(idx + 2));
                Triple crd_dx = Triple(_sins->Xk(idx + 3), _sins->Xk(idx + 4), _sins->Xk(idx + 5));


                base_quat dq_lidar = base_att_trans::rv2q(-att_dx);
                Eigen::Quaterniond _dq_lidar;
                _dq_lidar.w() = dq_lidar.q0; _dq_lidar.x() = dq_lidar.q1; _dq_lidar.y() = dq_lidar.q2; _dq_lidar.z() = dq_lidar.q3;
                _dq_lidar.normalize();
                lidar_state_iter->second.orientation = lidar_state_iter->second.orientation * _dq_lidar;
                lidar_state_iter->second.position -= crd_dx;
            }
        }
        if (updateLidarBuffer() < 0) cerr << "update lidar buffer wrong!" << endl;
    }

    void lidarprocesser::_meas_update(const Matrix& H, const Vector& r, bool isscan)
    {
        double lidar_obs_noise = 0;
        if (H.rows() == 0 || r.rows() == 0)
            return;
        cout << "!!!lidar_meas_up!!!:" << endl;
        if (lidarproc->estimate_extrinsic)
            assert(H.cols() == 6 * lidar_states.size() + 6);
        else
            assert(H.cols() == 6 * lidar_states.size());
        int par_size = param_of_sins->parNumber();
        int obs_size = H.rows();
        int N = lidar_states.size();

        Matrix All_H = Matrix::Zero(H.rows(), par_size);
        assert(N == int(H.cols() / 6));
        assert(obs_size == r.rows());

        if (isscan)
        {
            lidar_obs_noise = scan_observation_noise;
        }
        else
        {
            lidar_obs_noise = map_observation_noise;
        }

        auto lidar_state_iter = lidar_states.begin();
        for (int i = 0; i < lidar_states.size();
            ++i, ++lidar_state_iter)
        {
            string lidar_id = to_string(lidar_state_iter->first);
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

        _sins->Rk = Matrix::Identity(obs_size, obs_size) * lidar_obs_noise;
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

        _Updater._meas_update(_sins->Hk, _sins->Zk, _sins->Rk, _sins->Xk, _sins->Pk);
    }

    bool lidarprocesser::_gatingTest(Matrix& H, Vector& r, const int& dof, bool is_scan)
    {
        int obs_count = H.rows();
        assert(H.rows() == r.size());
        vector<int> indics;
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
                string lidar_id = to_string(lidar_state_iter->first);
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

        //LidarFrame project_lidarframe = _lastframe;
        int buf_size = lidar_buffer.size();
        int id = buf_size - 2;

        if (buf_size <= 3)
        {
            //lidar_buffer[buf_size-1] -> project_lidarframe
            _lidarOdo.process(lidar_buffer[buf_size - 2], lidar_buffer[buf_size - 1]);
            lidarMeasurementJacobian(_lidarOdo.lidarOdoObs, H, r, id, use_3d, ther);

        }
        else
        {

            for (int i = 1; i < lidar_buffer.size() - 1; i++)
            {
                Matrix tmp_H; Vector tmp_r;

                auto cur_project_lidarframe = lidar_buffer[i];
                int project_id = i;
                int cur_id = buf_size - 1;

                _lidarOdo.process(cur_project_lidarframe, lidar_buffer[buf_size - 1]);
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
        }

        _gatingTest(H, r, 2, true);
    }

    void lidarprocesser::build_PPHR(vector<LidarFrame>& buffer, map<int, vector<int>>& indexs, Matrix& H_x, Vector& r)
    {


        int eft_buf_size = buffer.size() - 2;
        int eft_number = 0;
        int eft_gatingtest_number = 0;
        int stack_cntr = 0;

        int max_jacobian_row_size = (3 * eft_buf_size - 3) * indexs.size();

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


        map<int, vector<int>>::iterator iter;
        for (iter = indexs.begin(); iter != indexs.end(); iter++)
        {
            if (iter->second.size() == eft_buf_size)
            {

                Matrix H_xj;
                Vector r_j;

                if (planarpatchJacobian(buffer, iter->first, iter->second, H_xj, r_j))
                {
                    eft_number++;

                }
                //lsy change
                if (_gatingTest(H_xj, r_j, buffer.size() - 1, scan_observation_noise))
                {
                    H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
                    r.segment(stack_cntr, r_j.rows()) = r_j;
                    stack_cntr += H_xj.rows();

                    eft_gatingtest_number++;
                }
                else
                {

                    //cout << "not_gatingtest:" << endl;

                }
            }

        }

        cout << "max_jacobian_row_size:" << max_jacobian_row_size << endl;
        cout << "pass_gatingtest_jacobian_size:" << stack_cntr << endl;

        cout << "max_process_ids_size:" << indexs.size() << endl;
        cout << "buildHR_ids_size( full observation):" << eft_number << endl;
        cout << "pass_gating_test_ids_size:" << eft_gatingtest_number << endl;
        H_x.conservativeResize(stack_cntr, H_x.cols());
        r.conservativeResize((Eigen::Index)stack_cntr);
        //cin.get();

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
            string lidar_id = to_string(lidar_state_iter->first);
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

        if (gamma < chi_squared_test_table[dof]) {
            return true;
        }
        else {
            return false;
        }
    }

    bool lidarprocesser::planarpatchJacobian(vector<LidarFrame>& buffer, int point_id, vector<int> corr_ids, Matrix& H, Vector& r)
    {

        assert(corr_ids.size() == buffer.size() - 2);
        vector<Eigen::Matrix3d> R_old_curs;
        vector<Triple> t_old_curs;
        vector<Triple> t_curs_old;
        for (int i = 0; i < corr_ids.size(); i++)
        {
            auto& oldest_lidar = buffer.at(1);
            auto& cur_lidar = buffer.at(2 + i);


            Eigen::Matrix3d R_old_cur = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e;
            Triple t_old_cur = cur_lidar.R_l_e.transpose() * (oldest_lidar.t_l_e - cur_lidar.t_l_e);
            Triple t_cur_old = oldest_lidar.R_l_e.transpose() * (cur_lidar.t_l_e - oldest_lidar.t_l_e);

            R_old_curs.push_back(R_old_cur);
            t_old_curs.push_back(t_old_cur);
            t_curs_old.push_back(t_cur_old);
        }


        int par_number = 0;
        if (lidarproc->estimate_extrinsic) par_number = buffer.size() * 6 + 6;
        else par_number = buffer.size() * 6;

        int jacobian_row_size = 3 * corr_ids.size();
        Matrix H_phi = Matrix::Zero(jacobian_row_size, 3);
        Matrix H_x = Matrix::Zero(jacobian_row_size, par_number);
        Vector r_j = Vector::Zero(jacobian_row_size);

        int stack_cntr = 0;
        for (int i = 0; i < corr_ids.size(); i++)
        {
            Matrix H_nd_j = Matrix::Zero(3, 4);
            Matrix H_phi_j = Matrix::Zero(4, 3);
            Matrix H_x_j = Matrix::Zero(3, par_number);
            auto& oldest_lidar = buffer.at(1);
            auto& cur_lidar = buffer.at(2 + i);

            auto& oldest_id = point_id;
            auto& cur_id = corr_ids.at(i);
            auto& oldest_nc = oldest_lidar.ncs.at(oldest_id);
            auto& oldest_pc = oldest_lidar.pcs.at(oldest_id);
            auto& cur_nc = cur_lidar.ncs.at(cur_id);
            auto& cur_pc = cur_lidar.pcs.at(cur_id);

            Eigen::Matrix3d& R_old_cur = R_old_curs.at(i);
            Triple& t_old_cur = t_old_curs.at(i);
            Triple& t_cur_old = t_curs_old.at(i);

            Triple oldest_phi = oldest_nc * (oldest_nc.transpose() * oldest_pc);
            Triple oldest_n = oldest_phi / oldest_phi.norm();

            Triple cur_phi = cur_nc * (cur_nc.transpose() * cur_pc);
            Triple cur_n = cur_phi / cur_phi.norm();

            //Triple residual = cur_phi - R_old_cur * oldest_n*(oldest_phi.norm() - t_cur_old.transpose()*oldest_n);
            Triple residual = R_old_cur * oldest_n * (oldest_phi.norm() - t_cur_old.transpose() * oldest_n) - cur_phi;
            //cout << "recal_res:::  "<<i<<"  " << residual.transpose() << endl;

            H_nd_j.block(0, 0, 3, 3) = R_old_cur * (Eigen::Matrix3d::Identity() * (oldest_phi.norm() - t_cur_old.transpose() * oldest_n) - oldest_n * t_cur_old.transpose());
            H_nd_j.block(0, 3, 3, 1) = R_old_cur * oldest_n;
            H_phi_j.block(0, 0, 3, 3) = 1.0 / oldest_phi.norm() * (Eigen::Matrix3d::Identity() - oldest_n * oldest_n.transpose());
            H_phi_j.block(3, 0, 1, 3) = oldest_n.transpose();

            double tmp_d = (oldest_phi.norm() - (oldest_lidar.R_l_e.transpose() * (cur_lidar.t_l_e - oldest_lidar.t_l_e)).transpose() * oldest_n);
            if (lidarproc->estimate_extrinsic)
            {
                //Eigen::Matrix3d tmp = oldest_lidar.R_l_e* oldest_n * (cur_lidar.t_l_e - oldest_lidar.t_l_e).transpose()*oldest_lidar.R_l_e*skew(oldest_n);
                //H_x_j.block(0, 2 * 6, 3, 3) = cur_lidar.R_l_e.transpose()*oldest_lidar.R_l_e*oldest_n*(cur_lidar.t_l_e - oldest_lidar.t_l_e).transpose()*oldest_lidar.R_l_e*skew(oldest_n)
                //    -tmp_d * cur_lidar.R_l_e.transpose()*oldest_lidar.R_l_e*skew(oldest_n);
                Eigen::Matrix3d a1 = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e;
                Eigen::Matrix3d a2 = oldest_n * (cur_lidar.t_l_e.transpose() - oldest_lidar.t_l_e.transpose()) * oldest_lidar.R_l_e * skew(oldest_n);
                Eigen::Matrix3d a3 = -skew(oldest_n * oldest_phi.norm()) + skew(oldest_n * (cur_lidar.t_l_e.transpose() - oldest_lidar.t_l_e.transpose()) * oldest_lidar.R_l_e * oldest_n);
                H_x_j.block(0, 2 * 6, 3, 3) = a1 * (a2 + a3);
                H_x_j.block(0, 2 * 6 + 3, 3, 3) = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e * oldest_n * (oldest_lidar.R_l_e * oldest_n).transpose();

                H_x_j.block(0, 2 * 6 + (i + 1) * 6, 3, 3) = tmp_d * skew(cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e * oldest_n);
                H_x_j.block(0, 2 * 6 + (i + 1) * 6 + 3, 3, 3) = -cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e * oldest_n * (oldest_lidar.R_l_e * oldest_n).transpose();
            }
            else
            {
                /*H_x_j.block(0, 1 * 6, 3, 3) = cur_lidar.R_l_e.transpose()*oldest_lidar.R_l_e*oldest_n*(cur_lidar.t_l_e - oldest_lidar.t_l_e).transpose()*oldest_lidar.R_l_e*skew(oldest_n)
                    - tmp_d * cur_lidar.R_l_e.transpose()*oldest_lidar.R_l_e*skew(oldest_n);*/
                Eigen::Matrix3d a1 = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e;
                Eigen::Matrix3d a2 = oldest_n * (cur_lidar.t_l_e.transpose() - oldest_lidar.t_l_e.transpose()) * oldest_lidar.R_l_e * skew(oldest_n);
                Eigen::Matrix3d a3 = -skew(oldest_n * oldest_phi.norm()) + skew(oldest_n * (cur_lidar.t_l_e.transpose() - oldest_lidar.t_l_e.transpose()) * oldest_lidar.R_l_e * oldest_n);
                H_x_j.block(0, 1 * 6, 3, 3) = a1 * (a2 + a3);
                H_x_j.block(0, 1 * 6 + 3, 3, 3) = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e * oldest_n * (oldest_lidar.R_l_e * oldest_n).transpose();

                H_x_j.block(0, 1 * 6 + (i + 1) * 6, 3, 3) = tmp_d * skew(cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e * oldest_n);
                H_x_j.block(0, 1 * 6 + (i + 1) * 6 + 3, 3, 3) = -cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e * oldest_n * (oldest_lidar.R_l_e * oldest_n).transpose();
            }

            H_phi.block<3, 3>(stack_cntr, 0) = H_nd_j * H_phi_j;
            H_x.block(stack_cntr, 0, 3, par_number) = H_x_j;
            r_j.segment<3>(stack_cntr) = residual;

            stack_cntr += 3;
        }

        //cout << "stack_cntr:" << stack_cntr << endl;
        //cout << "r_j_row" << jacobian_row_size << endl;

        Eigen::JacobiSVD<Matrix> svd_helper(H_phi, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Matrix A = svd_helper.matrixU().rightCols(jacobian_row_size - 3);

        H = A.transpose() * H_x;
        r = A.transpose() * r_j;

        return true;
    }

    void lidarprocesser::removeLidar(vector<LidarStateIDType>& rm_lidar_state_ids)
    {
        if (rm_lidar_state_ids.size() == 0)
            return;
        removeState(rm_lidar_state_ids);
        removeIDandBuffer(rm_lidar_state_ids);
    }

    void lidarprocesser::removeState(vector<LidarStateIDType>& rm_lidar_state_ids)
    {
        if (rm_lidar_state_ids.size() <= 0)
            return;
        for (const auto& lidar_id : rm_lidar_state_ids)
        {
            Matrix tmp_Qx = _sins->Pk;
            Vector _dx = _sins->Xk;
            LidarStateIDType _lidar_id = lidar_id;
            string string_id = to_string(_lidar_id);
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

    void lidarprocesser::removeIDandBuffer(vector<LidarStateIDType>& rm_lidar_state_ids)
    {
        //remove corresponding ids
        //vector<LidarStateIDType> vTemp;
        vector<LidarFrame> vTemplidar;
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
                //vTemp.push_back(FrameBuffer[i]);
                vTemplidar.push_back(lidar_buffer[i]);
            }
        }
        //FrameBuffer = vTemp;
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
        _lidarframe.R_l_e = (--lidar_states.end())->second.orientation;
        _lidarframe.t_l_e = (--lidar_states.end())->second.position;
        _lastframe.R_l_e = (--(--lidar_states.end()))->second.orientation;
        _lastframe.t_l_e = (--(--lidar_states.end()))->second.position;

        if (use_map)
        {
            _lidarmap.first_R_l_e = lidar_states.begin()->second.orientation.toRotationMatrix();
            _lidarmap.first_t_l_e = lidar_states.begin()->second.position;
        }
        return 1;
    }
}

