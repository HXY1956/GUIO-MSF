#include "hwa_msf_uwbprocesser.h"
#include "hwa_set_ign.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_msf{
    uwbprocesser::uwbprocesser(const baseprocesser& B, base_data* data) : baseprocesser(B){
        make_dir("output\\range.txt");
        make_dir("output\\position.txt");
        make_dir("output\\pdop.txt");
        make_dir("output\\enudop.txt");
        std::ofstream("output\\range.txt", std::ios::out | std::ios::trunc);
        std::ofstream("output\\position.txt", std::ios::out | std::ios::trunc);
        std::ofstream("output\\pdop.txt", std::ios::out | std::ios::trunc);
        std::ofstream("output\\enudop.txt", std::ios::out | std::ios::trunc);
        _outfile["range"].open("output\\range.txt", std::ios::app);
        _outfile["position"].open("output\\position.txt", std::ios::app);
        _outfile["pdop"].open("output\\pdop.txt", std::ios::app);
        _outfile["enudop"].open("output\\enudop.txt", std::ios::app);
        beg.from_secs(dynamic_cast<set_uwb*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_uwb*>(_gset.get())->end());
        TimeStamp = beg;
        lever = dynamic_cast<set_ign*>(_gset.get())->uwb_lever();
    };

    uwbprocesser::uwbprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog, base_data* data, base_time _beg, base_time _end) : baseprocesser(gset, spdlog, site, _beg, _end)
    {
        make_dir("output\\range.txt");
        make_dir("output\\position.txt");
        make_dir("output\\pdop.txt");
        make_dir("output\\enudop.txt");
        std::ofstream("output\\range.txt", std::ios::out | std::ios::trunc);
        std::ofstream("output\\position.txt", std::ios::out | std::ios::trunc);
        std::ofstream("output\\pdop.txt", std::ios::out | std::ios::trunc);
        std::ofstream("output\\enudop.txt", std::ios::out | std::ios::trunc);
        _outfile["range"].open("output\\range.txt", std::ios::app);
        _outfile["position"].open("output\\position.txt", std::ios::app);
        _outfile["pdop"].open("output\\pdop.txt", std::ios::app);
        _outfile["enudop"].open("output\\enudop.txt", std::ios::app);
        beg.from_secs(dynamic_cast<set_uwb*>(_gset.get())->start());
        end.from_secs(dynamic_cast<set_uwb*>(_gset.get())->end());
        TimeStamp = beg;
        lever = dynamic_cast<set_ign*>(_gset.get())->uwb_lever();
    };

    uwbprocesser::~uwbprocesser() {
        _outfile["range"].close();
        _outfile["position"].close();
        _outfile["pdop"].close();
        _outfile["enudop"].close();
    };

    void uwbprocesser::timesynchronization(base_time t) {
        if (TimeStamp < t) {
            int nEpo = round(t.diff(TimeStamp) / _ts);
            if (_ts > 1) {
                TimeStamp.add_secs(int(_ts * nEpo));  //  < 1Hz data
            }
            else {
                TimeStamp.add_dsec(_ts * nEpo);       //  >=1Hz data
            }
        }
    }

    bool uwbprocesser::_time_valid(base_time inst)
    {
        double crt = inst.sow() + inst.dsec();
        while (inst.diff(TimeStamp) >= _ts)
        {
            if (_ts > 1) TimeStamp.add_secs(int(_ts));  // =<1Hz data
            else         TimeStamp.add_dsec(_ts);       //  >1Hz data
        }
        if ((abs(inst.diff(TimeStamp)) < _shm->delay && inst >= TimeStamp)
            || abs(inst.diff(TimeStamp)) < 1e-6)
        {
            return true;
        }
        return false;
    }

    MEAS_TYPE uwbprocesser::_getPOS(base_time uwbt, base_posdata::data_pos& pos, MEAS_INFO& m)
    {
        MEAS_TYPE res_type;
        double crt = uwbt.sow() + uwbt.dsec();
        base_time runEpoch = uwbt;
        int irc = uwb_proc::ProcessOneEpoch(runEpoch);
        if (irc < 0) {
            return MEAS_TYPE::NO_MEAS;
        }

        m.MeasVel = Eigen::Vector3d::Zero(); m.MeasPos = _get_pos(); m.tmeas = crt;
        m._Cov_MeasVn = Eigen::Vector3d::Zero(); m._Cov_MeasPos = _get_pos_std().cwiseProduct(_get_pos_std());

        _sins->pos = Cart2Geod(m.MeasPos, false);
        _sins->vn = Cen(_sins->pos).transpose() * m.MeasVel;

        res_type = MEAS_TYPE::POS_MEAS;
        if (!double_eq(m.MeasVel.norm(), 0.0))
            res_type = POS_VEL_MEAS;

        pos.pos = _get_pos();
        pos.t = crt;
        pos.vn = Eigen::Vector3d::Zero();

        return res_type;
    }

    bool uwbprocesser::_init() {
        return true;
    }

    int uwbprocesser::_setMeas()
    {
        hwa_map_id_uwbnode uwb_epo = _get_uwb_epo();
        int valid_num = 0, obs_crt = 0;
        std::map<std::string, Triple> base_info = _get_anchor_info();
        UWB_WEIGHT wgt_type = _get_wgt_type();
        Triple pos_uwb_ecef = _sins->pos_ecef + _sins->Ceb * lever;
        double x = pos_uwb_ecef(0), y = pos_uwb_ecef(1), z = pos_uwb_ecef(2);
        Matrix H0 = Matrix::Zero(1, 3);
        anchorname.clear();
        for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
        {
            anchorname.push_back(base_info.find(it->first)->first);

            if (_outfile["position"].is_open()) {
                _outfile["position"] << TimeStamp.sod() << " " << base_info.find(it->first)->first << " " << it->second.range << " " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << pos_uwb_ecef.transpose();
                _outfile["position"] << std::setiosflags(std::ios::fixed) << std::setprecision(4) << " "
                    << pow(meas_range_std * sqrt(pow(10.0, -(it->second.SNR) / 10.0)), 2) << std::endl;
            }
            else {
                std::cerr << "Unable to open file" << std::endl;
            }

            Triple base_crd_vector = base_info.find(it->first)->second;
            double dist = (pos_uwb_ecef - base_crd_vector).norm();
            double xu = base_crd_vector(0), yu = base_crd_vector(1), zu = base_crd_vector(2);

            H0 = (pos_uwb_ecef - base_crd_vector).transpose() / dist;
            _sins->Zk(obs_crt, 0) = it->second.range - dist;

            if (_outfile["range"].is_open()) {
                _outfile["range"] << std::setiosflags(std::ios::fixed) << std::setprecision(4) << TimeStamp.sod() 
                    << " " << base_info.find(it->first)->first << " " << _sins->Zk(obs_crt, 0) << " " << pos_uwb_ecef.transpose()
                    << " " << base_crd_vector.transpose() << " " << it->second.range << " " << dist << std::endl;
            }

            if (_Estimator == INEKF) {
                _sins->Hk.block(obs_crt, 0, 1, 3) = H0 * hwa_base::askew(pos_uwb_ecef - _sins->initial_pos); //att
                _sins->Hk.block(obs_crt, 3, 1, 3) = Matrix::Zero(1, 3); //ve
                _sins->Hk.block(obs_crt, 6, 1, 3) = -H0; //pe
            }
            else if (_Estimator == NORMAL) {
                _sins->Hk.block(obs_crt, 6, 1, 3) = -H0;
                _sins->Hk.block(obs_crt, 0, 1, 3) = H0 * hwa_base::askew(_sins->Ceb * lever);
            }
            //Rk
            switch (wgt_type)
            {
            case hwa_uwb::UWB_WEIGHT::EQUAL:
                _sins->Rk(obs_crt, obs_crt) = pow(meas_range_std, 2);
                break;
            case hwa_uwb::UWB_WEIGHT::SNR:
                _sins->Rk(obs_crt, obs_crt) = pow(meas_range_std * sqrt(pow(10.0, -(it->second.SNR) / 10.0)), 2);
                break;
            case hwa_uwb::UWB_WEIGHT::ANTIRANGE:
                _sins->Rk(obs_crt, obs_crt) = 1e-2;
                break;
            case hwa_uwb::UWB_WEIGHT::SUCCESSRATE:
                _sins->Rk(obs_crt, obs_crt) = 1 / pow(it->second.successrate * 5, 2);
                break;
            default:
                break;
            }

            auto index = find(_nlos.begin(), _nlos.end(), it->first);
            if (index != _nlos.end())
            {
                _sins->Rk(obs_crt, obs_crt) = _sins->Rk(obs_crt, obs_crt) * barrior;
            }

            obs_crt++;
            valid_num++;
            valid_node.push_back(it->first);
        }
        return valid_num;
    }

    bool uwbprocesser::load_data() {
        return _prepareData(TimeStamp);
    }

    int uwbprocesser::ProcessOneEpoch()
    {
        valid_node.clear();
        int dim = _sins->Pk.rows();
        nr = _get_nr();
        _sins->Pk_Sav = _sins->Pk;
        _sins->Hk = Matrix::Zero(nr, dim);
        _sins->Zk = Vector::Zero(nr);
        _sins->Rk = Matrix::Zero(nr, nr);
        _sins->Xk = Vector::Zero(dim);
        int valid_num = _setMeas();
        if (valid_num == 0) 
            return NO_MEAS;

        Matrix Kkm;
        Vector v_norm;
        Vector X_sav = _sins->Xk;
        int outlier_index = -1;
        int outlier = -1;
        double vtpv = 0.0, lambda;

        //Estimate Nlos;
        Triple pos_uwb_ecef = _sins->pos_ecef + _sins->Ceb * lever;
        _extract_value();
        if (SDP) {
            std::vector<double> mu(valid_num, 0.0);
            nlos_est(mu, pos_uwb_ecef);
            for (int i = 0; i < _sins->Zk.size(); i++) {
                _sins->Zk[i] -= mu[i];
                CrtRange[i] = IniRange[i] - mu[i];
            }
        }
        _Updater.setRecpos(pos_uwb_ecef);
        _Updater.setAncpos(AnchorPos);
        _Updater.setRange(CrtRange);

        // Measurement Update
        for (int i = 0; i < _iter; i++)
        {
            if (outlier >= 0)
            {
                _sins->Rk(outlier, outlier) = _sins->Rk(outlier, outlier) * barrior;
                outlier = -1;
            }
            _Updater._meas_update(_sins->Hk, _sins->Zk, _sins->Rk, _sins->Xk, _sins->Pk);
            if (posterior) {
                _posterioriTest(_sins->Hk, _sins->Rk, _sins->Zk, _sins->Xk, _sins->Pk, v_norm, vtpv);
                if (_outlierDetect(v_norm, outlier, lambda) >= 0) _sins->Pk = _sins->Pk_Sav;
            }
            if (outlier < 0) break;
            _sins->Xk = X_sav;
            _sins->Pk = _sins->Pk_Sav;
        }

        //Calculate Dop
        double cof = pow(_sins->Rk.block(0, 0 ,valid_num, valid_num).determinant(), 1.0 / valid_num);
        Triple Geo_pos = Cart2Geod(_sins->pos_ecef, false);
        SO3 Cne;
        Cne(0, 0) = -sin(Geo_pos(1)); Cne(0, 1) = cos(Geo_pos(1)); Cne(0, 2) = 0.0;
        Cne(1, 0) = -sin(Geo_pos(0)) * cos(Geo_pos(1)); Cne(1, 1) = -sin(Geo_pos(0)) * sin(Geo_pos(1)); Cne(1, 2) = cos(Geo_pos(0));
        Cne(2, 0) = cos(Geo_pos(0)) * cos(Geo_pos(1)); Cne(2, 1) = cos(Geo_pos(0)) * sin(Geo_pos(1)); Cne(2, 2) = sin(Geo_pos(0));
        SO3 Qxyzi, Qxyz;
        Qxyzi << (_sins->Hk.block(0, 6, valid_num, 3).transpose()) * _sins->Hk.block(0, 6, valid_num, 3);
        Qxyz << Qxyzi.inverse();
        SO3 Qenu = Cne * Qxyz * Cne.transpose();
        Qenu = Qenu / cof;
        double edop = sqrt(Qenu(0, 0));
        double ndop = sqrt(Qenu(1, 1));
        double udop = sqrt(Qenu(2, 2));
        _sins->hdop = sqrt(Qenu(0, 0) + Qenu(1, 1));
        _sins->vdop = sqrt(Qenu(2, 2));
        uwb_pdop = _sins->pdop = sqrt(Qxyz(0, 0) + Qxyz(1, 1) + Qxyz(2, 2));
        if (_outfile["pdop"].is_open()) {
            _outfile["pdop"] << TimeStamp.sod() << std::setprecision(2) << " " << _sins->pdop << std::endl;
        }
        if (_outfile["enupdop"].is_open()) {
            _outfile["enupdop"] << TimeStamp.sod() << std::setprecision(2) << " " << edop << " " << ndop << " " << udop << std::endl;
        }
        return UWB_MEAS;
    }

    void uwbprocesser::_feed_back() {
    }
}

