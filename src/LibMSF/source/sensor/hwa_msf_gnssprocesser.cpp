#include "hwa_msf_gnssprocesser.h"

namespace hwa_msf {
	gnssprocesser::gnssprocesser(const baseprocesser& B, std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc) : baseprocesser(B),
        gnss_proc_spp(site, gset.get(), spdlog),
		gnss_proc_pvtflt(site, site_base, gset.get(), spdlog, allproc) {
        lever = dynamic_cast<set_ign*>(gset.get())->gnss_lever();
	}

	gnssprocesser::gnssprocesser(std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc, base_time _beg, base_time _end) :
		baseprocesser(gset, spdlog, site, _beg, _end),
        gnss_proc_spp(site, gset.get(), spdlog),
		gnss_proc_pvtflt(site, site_base, gset.get(), spdlog, allproc) {
        lever = dynamic_cast<set_ign*>(gset.get())->gnss_lever();
	};

    bool gnssprocesser::_init()
    {
        try
        {
            Symmetric Qx_extended;
            int size_bef = param_of_sins->parNumber();
            int icrdz_gnss = _param->getParam(_site, par_type::CRD_Z, "");
            for (int i = icrdz_gnss + 1; i < _param->parNumber(); i++)
            {
                param_of_sins->addParam(_param->operator[](i));
            }
            Qx_extended.resize(param_of_sins->parNumber()); 
            Qx_extended.setZero();
            //Qx_extended.matrixW().block(0, 0, size_bef, size_bef) = _sins->Pk.block(0, 0, size_bef, size_bef);

            if (size_bef < param_of_sins->parNumber())
            {
                int length = param_of_sins->parNumber() - size_bef;
                Qx_extended.matrixW().block(size_bef, size_bef, length, length) =
                    _Qx.SymSubMatrix(icrdz_gnss + 1, _param->parNumber() - 1).matrixR();
            }
            for (int i = 0; i < size_bef; i++)
            {
                Qx_extended.set(1.0, i, i);
            }

            _param = param_of_sins;
            _Qx = Qx_extended;
            _param->reIndex();
        }
        catch (...)
        {
            return false;
        }

        return true;
    }

    void gnssprocesser::timesynchronization(base_time t) {
        if (TimeStamp < t) {
            int nEpo = round(t.diff(TimeStamp) / _sampling + 0.5);
            if (_sampling > 1) {
                TimeStamp.add_secs(int(_sampling * nEpo));  //  < 1Hz data
            }
            else {
                TimeStamp.add_dsec(_sampling * nEpo);       //  >=1Hz data
            }
        }
    }

    bool gnssprocesser::load_data() {
        _data.erase(_data.begin(), _data.end());

        if (_isClient) {
            if (!_gInterpol->interpolAug(TimeStamp, _data, _data_base)) {
                return false;
            }
        }

        _slip_detect(TimeStamp);

        if (!_isClient) _data = _gobs->obs(_site, TimeStamp);

        if (_data.size() > 0) {
            if (_gallbias) {
                for (auto& itdata : _data) {
                    itdata.apply_bias(_gallbias);
                }
            }
        }
        else {
            if (baseprocesser::_spdlog) {
                SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, std::string("gintegration:  ") + _site + TimeStamp.str_ymdhms(" no observation found at epoch: "));
            }
            return false;
        }

        std::vector<gnss_data_sats>::iterator it = _data.begin();
        std::string double_freq = "";
        std::string single_freq = "";

        std::ostringstream obsqualityInfo; obsqualityInfo.str("");
        obsqualityInfo << "> " << std::setw(6) << TimeStamp.sod() << std::endl;
        _sat_freqs.clear();
        while (it != _data.end())
        {
            GOBSBAND b1 = _band_index[it->gsys()][FREQ_1];
            GOBSBAND b2 = _band_index[it->gsys()][FREQ_2];

            auto obsL1 = it->select_phase(b1);
            auto obsL2 = it->select_phase(b2);
            auto obsP1 = it->select_range(b1);
            auto obsP2 = it->select_range(b2);
            auto snrL1 = it->getobs(pl2snr(obsP1)) > it->getobs(pl2snr(obsL1)) ? it->getobs(pl2snr(obsP1)) : it->getobs(pl2snr(obsL1));
            auto snrL2 = it->getobs(pl2snr(obsP2)) > it->getobs(pl2snr(obsL2)) ? it->getobs(pl2snr(obsP2)) : it->getobs(pl2snr(obsL2));
            auto ele = it->ele_deg();
            //auto snrL1 = it->obs_S(b1);
            //std::cerr << ele << std::setw(6) << std::endl;

            if ((obsL1 == GOBS::X && obsL2 != GOBS::X) || (obsL1 != GOBS::X && obsL2 == GOBS::X))
            {
                single_freq += "  " + it->sat();
                _sat_freqs[it->sat()] = "1";
            }

            if (obsL1 != GOBS::X && obsL2 != GOBS::X)
            {
                double_freq += "  " + it->sat();
                _sat_freqs[it->sat()] = "2";
            }
            obsqualityInfo << it->sat();
            if (obsP1 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
            else obsqualityInfo << std::setw(4) << "1";
            if (obsL1 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
            else obsqualityInfo << std::setw(4) << "1";
            if (obsP2 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
            else obsqualityInfo << std::setw(4) << "1";
            if (obsL2 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
            else obsqualityInfo << std::setw(4) << "1";
            obsqualityInfo << std::setw(10) << std::setprecision(4) << snrL1
                << std::setw(10) << std::setprecision(4) << snrL2 << std::endl;
            ++it;
        }
        if (_obsqualityfile)
        {
            _obsqualityfile->write(obsqualityInfo.str().c_str(), obsqualityInfo.str().size());
            _obsqualityfile->flush();
        }

        if (_isBase)
        {
            if (!_isClient) {
                _data_base.erase(_data_base.begin(), _data_base.end());
                _data_base = _gobs->obs(_site_base, TimeStamp);
            }
            if (_data_base.size() > 0) {
                if (_gallbias) {
                    for (auto& itdata : _data_base) {
                        itdata.apply_bias(_gallbias);
                    }
                }
            }
            else {
                if (baseprocesser::_spdlog) {
                    SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, std::string("gintegration:  ") + _site_base + TimeStamp.str_ymdhms(" no base observation found at epoch: "));
                }
                return false;
            }
        }

        if (_data.size() == 0) 
            return false;

        return true;
    }

    bool gnssprocesser::_time_valid(base_time inst)
    {
        bool res_valid = false;
        double crt = inst.sow() + inst.dsec();
        TimeStamp = _gobs->load(_site, crt);
        if ((abs(inst.diff(TimeStamp)) < _shm->delay && inst >= TimeStamp)
            || abs(inst.diff(TimeStamp)) < 1e-6) 
            return true;
        return false;
    }

    MEAS_TYPE gnssprocesser::_getPOS(base_time gst, base_posdata::data_pos& pos, MEAS_INFO& m)
    {
        MEAS_TYPE res_type;
        double crt = gst.sow() + gst.dsec();
        base_time runEpoch = _gobs->load(_site, crt);
        int irc = gnss_proc_pvtflt::ProcessOneEpoch(runEpoch);
        if (irc < 0) {
            return MEAS_TYPE::NO_MEAS;
        }
        _get_result(runEpoch, pos);

        m.MeasVel = pos.vn; m.MeasPos = pos.pos; m.tmeas = pos.t;
        m._Cov_MeasVn = pos.Rvn; m._Cov_MeasPos = pos.Rpos;

        _sins->pos = Cart2Geod(m.MeasPos, false);
        _sins->vn = Cen(_sins->pos).transpose() * m.MeasVel;

        res_type = MEAS_TYPE::POS_MEAS;
        if (!double_eq(m.MeasVel.norm(), 0.0))
            res_type = POS_VEL_MEAS;

        //if (pos.PDOP > _shm->max_pdop) res_type = NO_MEAS;
        if (pos.nSat < _shm->min_sat) res_type = NO_MEAS;
        //if (_isBase && !pos.amb_state) res_type = NO_MEAS;

        return res_type;
    }

    void gnssprocesser::_prt_port(base_time instime)
    {
        std::set<std::string> ambs = _param->amb_prns();
        int nsat = ambs.size();
        // get amb status
        std::string amb = "Float";
        if (_amb_state)amb = "Fixed";
        Eigen::Vector3d XYZ_INS = _sins->pos_ecef + _sins->Ceb * lever;
        std::ostringstream os;
        os << std::fixed << std::setprecision(4) << " "
            << " " << instime.sow() + instime.dsec()
            << std::fixed << std::setprecision(4)
            << " " << std::setw(15) << XYZ_INS[0]          // [m]
            << " " << std::setw(15) << XYZ_INS[1]          // [m]
            << " " << std::setw(15) << XYZ_INS[2]          // [m]
            << " " << std::setw(10) << _sins->vn[0]          // [m]
            << " " << std::setw(10) << _sins->vn[1]          // [m]
            << " " << std::setw(10) << _sins->vn[2]          // [m]
            << std::fixed << std::setprecision(4)
            << " " << std::setw(10) << _sins->att(0) / glv.deg
            << " " << std::setw(10) << _sins->att(1) / glv.deg
            << " " << std::setw(10) << -_sins->att(2) / glv.deg
            << std::fixed << std::setprecision(4)
            << " " << std::setw(10) << _sins->eb(0) / glv.dph
            << " " << std::setw(10) << _sins->eb(1) / glv.dph
            << " " << std::setw(10) << _sins->eb(2) / glv.dph
            << std::fixed << std::setprecision(4)
            << " " << std::setw(10) << _sins->db(0) / glv.mg
            << " " << std::setw(10) << _sins->db(1) / glv.mg
            << " " << std::setw(10) << _sins->db(2) / glv.mg
            << std::fixed << std::setprecision(0)
            << " " << std::setw(5) << nsat            // nsat
            << std::fixed << std::setprecision(2)
            << " " << std::setw(5) << _dop.pdop()            // pdop
            << std::fixed << std::setprecision(2)
            << " " << std::setw(8) << amb
            << std::setw(10) << (_amb_state ? _ambfix->get_ratio() : 0.0)
            << std::endl;
        if (_maptcp.find(FLT_OUT) != _maptcp.end())_maptcp[FLT_OUT]->run_send(os.str());
    }

    int gnssprocesser::_prt_ins_kml(base_time instime)
    {
        if (!_kml)
            return 0;
        Eigen::Vector3d Geo_pos = Eigen::Vector3d(_sins->pos(0) / glv.deg, _sins->pos(1) / glv.deg, _sins->pos(2));

        double crt = instime.sow() + instime.dsec();
        Eigen::Vector3d Qpos = _sins->Pk.block(6, 6, 3, 3).diagonal(), Qvel = _sins->Pk.block(3, 3, 3, 3).diagonal();
        Eigen::Vector3d position = _sins->pos_ecef, velocity = _sins->ve;
        base_posdata::data_pos posdata = base_posdata::data_pos{ crt, position, velocity, Qpos, Qvel, 1.3, int(_data.size()), _amb_state };

        if (_kml) {
            std::ostringstream out;
            out << std::fixed << std::setprecision(11) << " " << std::setw(0) << Geo_pos[1] << ',' << Geo_pos[0];
            std::string val = out.str();

            xml_node root = _doc;
            xml_node node = this->_default_node(root, _root.c_str());
            xml_node document = node.child("Document");
            xml_node last_child = document.last_child();
            xml_node placemark = document.insert_child_after("Placemark", last_child);
            std::string q = "#P" + _quality_grade(posdata);
            this->_default_node(placemark, "styleUrl", q.c_str());
            this->_default_node(placemark, "time", base_type_conv::int2str(instime.sow()).c_str());
            xml_node point = this->_default_node(placemark, "Point");
            this->_default_node(point, "coordinates", val.c_str()); // for point
            xml_node description = placemark.append_child("description");
            description.append_child(pugi::node_cdata).set_value(_gen_kml_description(instime, posdata).c_str());
            xml_node TimeStamp = placemark.append_child("TimeStamp");
            std::string time = base_type_conv::trim(instime.str_ymd()) + "T" + base_type_conv::trim(instime.str_hms()) + "Z";
            this->_default_node(TimeStamp, "when", time.c_str());

            xml_node Placemark = document.child("Placemark");
            xml_node LineString = Placemark.child("LineString");
            this->_default_node(LineString, "coordinates", val.c_str(), false);  // for line
        }

        return 1;
    }

    void gnssprocesser::_feed_back() {
        for (unsigned int iPar = 0; iPar < _param->parNumber(); iPar++) {
            if (_param->operator[](iPar).parType == par_type::TRP)
            {
                std::string site = _param->operator[](iPar).site;
                Triple Ell, XYZ;
                if (_param->getCrdParam(site, XYZ) > 0) {
                }
                else { XYZ = _gallobj->obj(site)->crd_arp(_epoch); }
                xyz2ell(XYZ, Ell, false);
                if (site == _site && _gModel->tropoModel() != 0)
                    _param->operator[](iPar).apriori(_gModel->tropoModel()->getZHD(Ell, _epoch));
                else if (site == _site_base && _gModel_base->tropoModel() != 0)
                    _param->operator[](iPar).apriori(_gModel_base->tropoModel()->getZHD(Ell, _epoch));
            }
        }
        for (unsigned int Par = 0; Par < _param->parNumber(); Par++) {
            if (_param->operator[](Par).parType == par_type::CRD_X || _param->operator[](Par).parType == par_type::CRD_Y || _param->operator[](Par).parType == par_type::CRD_Z)
            {
                _param->operator[](Par).value(_param->operator[](Par).value() - _sins->Xk(_param->operator[](Par).index));
            }
            else
            {
                _param->operator[](Par).value(_param->operator[](Par).value() + _sins->Xk(_param->operator[](Par).index));
            }
        }
    }

    int gnssprocesser::_merge(Matrix& A)
    {
        try
        {
            int size = _param->parNumber();
            int nobs = A.rows();
            Eigen::Vector3d pos_gnss_ecef = _sins->pos_ecef + _sins->Ceb * lever;
            // A 
            if (_Estimator == NORMAL) {
                int icrdx = _param->operator[](_param->getParam(_site, par_type::CRD_X, "")).index;
                A.block(0, icrdx, nobs, 3) = -A.block(0, icrdx, nobs, 3);
                int iattx = _param->operator[](_param->getParam(_site, par_type::ATT_X, "")).index;
                A.block(0, iattx, nobs, 3) = A.block(0, icrdx, nobs, 3) * askew(_sins->Ceb * lever);
            }
            else if (_Estimator == INEKF ) {
                int icrdx = _param->operator[](_param->getParam(_site, par_type::CRD_X, "")).index;
                A.block(0, icrdx, nobs, 3) = -A.block(0, icrdx, nobs, 3);
                int iattx = _param->operator[](_param->getParam(_site, par_type::ATT_X, "")).index;
                A.block(0, iattx, nobs, 3) = A.block(0, icrdx, nobs, 3) * askew(pos_gnss_ecef - _sins->initial_pos);
            }
        }
        catch (...)
        {
            if (baseprocesser::_spdlog) SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, std::string("gintegration:  ") + "GNSS and INS infomation fusion failed!");
            return 0;
        }
        return 1;
    }

    void gnssprocesser::valid_ins_constraint() {
        Eigen::Vector3d XYZ_INS = _sins->pos_ecef + _sins->Ceb * lever;
        int crd_idx = param_of_sins->getParam(_site, par_type::CRD_X, "");
        Triple XYZ(XYZ_INS(0), XYZ_INS(1), XYZ_INS(2));
        Triple Qxyz(sqrt(_Qx(crd_idx, crd_idx)), sqrt(_Qx(crd_idx + 1, crd_idx + 1)),
            sqrt(_Qx(crd_idx + 2, crd_idx + 2)));
        _external_pos(XYZ, Qxyz);
    }

    int gnssprocesser::ProcessOneEpoch() {
        _timeUpdate(TimeStamp);
        _Qx.matrixW() = _sins->Pk;
        if (_grec == nullptr)
        {
            if (baseprocesser::_spdlog)
                SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, "No receiver settings available!!!");
            return -1;
        }

        _epoch = TimeStamp;
        _amb_state = false;

        if (!_crd_xml_valid())
            _sig_init_crd = 100.0;

        Symmetric P;
        Vector v_norm;
        Symmetric Qsav, QsavBP;
        base_allpar XsavBP;
        double vtpv;
        std::string outlier = "";
        _cntrep = 0; // number of iterations caused by outliers
        _crt_SNR.clear();
        _crt_ObsLevel.clear();

        // select obs for tb log (rover)
        if (_slip_model == SLIPMODEL::TURBO_EDIT && !_turbo_liteMode)
        {
            if (!this->_post_turbo_select_obs(TimeStamp, _data))
                return -1;
        }

        do
        {
            _remove_sat(outlier);

            if (TimeStamp.sow() + TimeStamp.dsec() == 285381) {
                std::cerr << "DEBUG\n";
            }

            valid_ins_constraint();

            if (_prepareData() < 0)
            {
                if(_initialized) _predict(TimeStamp);
                _sins->Pk = _Qx.matrixR();
                return NO_MEAS;
            }

            QsavBP = _Qx;
            XsavBP = *_param;

            _predict(TimeStamp);

            _initialized = true;
            _pos_constrain = false;

            if (_data.size() < _minsat)
            {
                if (baseprocesser::_spdlog)
                    SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, "Not enough visible satellites!");
                _restore(QsavBP, XsavBP);
                _sins->Pk = _Qx.matrixR();
                return NO_MEAS;
            }

            unsigned int nObs = _data.size();
            unsigned int mult = 1;
            if (_observ == OBSCOMBIN::RAW_DOUBLE)
            {
                mult = 2;
                nObs *= 2;
            }
            if (_observ == OBSCOMBIN::RAW_ALL /*|| _observ == OBSCOMBIN::RAW_MIX*/)
            {
                mult = 2;
                nObs *= 5;
            } // reservation for 5 freq - not used raws will be removed
            if (_phase)
            {
                mult *= 2;
                nObs *= 2;
            } // code + phase
            if (_observ == OBSCOMBIN::IF_P1)
            {
                mult = 2;
                nObs *= 3;
            } // IF + P1(不增加参数，多一倍观测值）

            /*G01 P1 P2 P3... L1 L2L3*/
            unsigned int nPar = _param->parNumber();
            unsigned int iobs = 1;

            _frqNum.clear();
            _obs_index.clear();

            if (_isBase)
            {
                dynamic_cast<gnss_model_comb_dd*>(&(*_base_model))->set_base_data(&_data_base);
                dynamic_cast<gnss_model_comb_dd*>(&(*_base_model))->set_rec_info(_gallobj->obj(_site_base)->crd_arp(_epoch), _vBanc(3), _vBanc_base(3));
            }
            // use combmodel
            gnss_proc_lsq_equationmatrix equ;
            iobs = _cmp_equ(equ);

            equ.chageNewMat(_sins->Hk, P, _sins->Zk, nPar);
            _sins->Xk.resize(nPar);
            _sins->Xk.setZero();

            _obs_index.clear();
            _generateObsIndex(equ);

            if (iobs < _minsat * mult)
            {
                if (baseprocesser::_spdlog)
                    SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, "Not enough processable observations!");
                _restore(QsavBP, XsavBP);
                _sins->Pk = _Qx.matrixR();
                return -1;
            }
            if (_isBase || _sd_sat)
            {
                if (_combineDD(_sins->Hk, P, _sins->Zk) < 0) {
                    _sins->Pk = _Qx.matrixR();
                    return -1;
                }
            }

            Qsav = _Qx;
            if (!_merge(_sins->Hk)) {
                _sins->Xk.setZero(); 
                _sins->Pk = _Qx.matrixR();
                if (baseprocesser::_spdlog)
                    SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, std::string("gintegration:  ") + _site + TimeStamp.str_ymdhms(" merge failed at epoch: "));
                return NO_MEAS;
            }

            _sins->Rk = P.matrixR().jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
                .solve(Matrix::Identity(P.rows(), P.cols()));

            //std::cout << "time"<<TimeStamp.sow() +TimeStamp.dsec()<<"\n";
            //m_out("A", _sins->Hk);
            //m_out("P", P.matrixR());
            //m_out("l", _sins->Zk);
            //m_out("DEBUG[0] _Qx", _Qx.matrixR());

            try
            {
                //if (_Updater._meas_update(_sins->Hk, _sins->Zk, _sins->Rk, _sins->Xk, _Qx.matrixW()) < 0) {
                //    _filter->update(_sins->Hk, _sins->Rk, _sins->Zk, _sins->Xk, _Qx);
                //}
                _filter->update(_sins->Hk, _sins->Rk, _sins->Zk, _sins->Xk, _Qx);
                //m_out("_dx", _sins->Xk);
                //m_out("DEBUG[1] _Qx", _Qx.matrixR());
            }

            catch (...)
            {
                if (baseprocesser::_spdlog)
                    SPDLOG_LOGGER_INFO(baseprocesser::_spdlog, "gnss_proc_pvtflt", " filter update failed!");
                _Qx = Qsav;
                _sins->Pk = _Qx.matrixR();
                return -1;
            }

            for (size_t iPar = 0; iPar < _param->parNumber(); iPar++) {
                if (_param->operator[](iPar).parType == par_type::AMB_IF ||
                    _param->operator[](iPar).parType == par_type::AMB_L1 ||
                    _param->operator[](iPar).parType == par_type::AMB_L2 ||
                    _param->operator[](iPar).parType == par_type::AMB_L3 ||
                    _param->operator[](iPar).parType == par_type::AMB_L4 ||
                    _param->operator[](iPar).parType == par_type::AMB_L5) {
                    std::string sat = _param->operator[](iPar).prn;
                    if (_newAMB.find(sat) != _newAMB.end() && _cntrep == 1) {
                        if (_newAMB[sat] == 1) _Qx.matrixW()(iPar, iPar) += 10;
                        if (_newAMB[sat] == 2 && _Qx.get(iPar, iPar) > 0.01) _Qx.matrixW()(iPar, iPar) += 1;
                        _newAMB[sat]++;
                    }
                }
            }
            _posterioriTest(_sins->Hk, _sins->Rk, _sins->Zk, _sins->Xk, _Qx, v_norm, vtpv);
        } while (_outlierDetect(v_norm, Qsav, outlier) != 0);
        
        _filter->add_data(*_param, _sins->Xk, _Qx, _sig_unit, Qsav);
        _filter->add_data(_sins->Hk, P, _sins->Zk);
        _filter->add_data(vtpv, _sins->Hk.rows(), _sins->Hk.cols());

        for (int i = 0; i < _param->parNumber(); i++)
        {
            std::cout << std::fixed << std::setw(20) << " Float EPO  " << std::setw(20) << _filter->param()[i].str_type() + "  " 
                << std::setw(20) << std::setprecision(5) << _filter->param()[i].value() 
                << std::setw(15) << std::setprecision(5) << _filter->dx()(i) 
                << std::setw(20) << std::setprecision(5) << _filter->param()[i].value() + _filter->dx()(i) 
                << std::setw(20) << std::setprecision(5) << _filter->stdx()(i) << std::endl;
        }

        _sins->Pk = _Qx.matrixR();
        _amb_resolution();

        return GNSS_MEAS;
    }
}