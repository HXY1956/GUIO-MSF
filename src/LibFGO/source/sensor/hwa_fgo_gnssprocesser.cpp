#include "hwa_fgo_gnssprocesser.h"
#include "hwa_fgo_factor_initial_pose.h"
#include "hwa_gnss_model_precisebiasgpp.h"
#include "hwa_base_timecost.h"

using namespace std;

namespace hwa_fgo {
	gnssprocesser::gnssprocesser(const baseprocesser& B, std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc) : baseprocesser(B, GNSS_NODE),
        gnss_proc_spp(site, gset.get(), spdlog),
		gnss_proc_pvtflt(site, site_base, gset.get(), spdlog, allproc) {

        lever = dynamic_cast<set_ign*>(gset.get())->gnss_lever();
		gnss_model_bias* precise_bias(new gnss_model_precise_biasgpp(_allproc, baseprocesser::_spdlog, gset.get()));
		_gbias_model = precise_bias;
		cur_sat_prn.clear();
		_all_para_win.delAllParam();
		_DD_msg.clear();
		_vDD_msg.clear();
		shared_ptr<gnss_amb_manager> amb_m(new gnss_amb_manager(_band_index));
		_amb_manager = amb_m;
		_pos_constrain = false;
		_cntrep = 0;
		int sign = 1;
		if (_sampling > 1)
			_gtime_interval = int(sign * _sampling);
		else
			_gtime_interval = sign * _sampling;
	}

	gnssprocesser::gnssprocesser(std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc, base_time _beg, base_time _end) :
		baseprocesser(gset, spdlog, site, GNSS_NODE,_beg, _end),
        gnss_proc_spp(site, gset.get(), spdlog),
		gnss_proc_pvtflt(site, site_base, gset.get(), spdlog, allproc) {

        lever = dynamic_cast<set_ign*>(gset.get())->gnss_lever();
		gnss_model_bias* precise_bias(new gnss_model_precise_biasgpp(_allproc, baseprocesser::_spdlog, gset.get()));
		_gbias_model = precise_bias;
		cur_sat_prn.clear();
		_all_para_win.delAllParam();
		_DD_msg.clear();
		_vDD_msg.clear();
		shared_ptr<gnss_amb_manager> amb_m(new gnss_amb_manager(_band_index));
		_amb_manager = amb_m;
		_pos_constrain = false;
		_cntrep = 0;
		int sign = 1;
		if (_sampling > 1)
			_gtime_interval = int(sign * _sampling);
		else
			_gtime_interval = sign * _sampling;
	};

	gnssprocesser::~gnssprocesser()
	{
		delete _last_gnss_info;
		delete _gbias_model;
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

			if (_gallobj != nullptr) {
				auto it_data = _data.begin();
				while (it_data != _data.end()) {
					string sat_id = it_data->sat();
					shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(sat_id);


					if (sat_obj == nullptr) {
						//std::cout << "remove satellite " + sat_id + " due to missing object\n";
						it_data = _data.erase(it_data);
					}
					else {
						shared_ptr<gnss_data_pcv> sat_pcv = sat_obj->pcv(TimeStamp);
						if (sat_pcv == nullptr) {
							//std::cout << "remove satellite " + sat_id + " due to missing PCV data\n";
							it_data = _data.erase(it_data);
						}
						else {
							++it_data;
						}

						//++it_data;
					}
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

				if (_gallobj != nullptr) {
					auto it_base = _data_base.begin();
					while (it_base != _data_base.end()) {
						string sat_id = it_base->sat();
						shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(sat_id);

						if (sat_obj == nullptr) {
							it_base = _data_base.erase(it_base);
						}
						else {
							shared_ptr<gnss_data_pcv> sat_pcv = sat_obj->pcv(TimeStamp);
							if (sat_pcv == nullptr) {
								std::cout << "remove base satellite " + sat_id + " due to missing PCV data\n";
								it_base = _data_base.erase(it_base);
							}
							else {
								++it_base;
							}
							//++it_base;
						}
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

        if (abs(inst.diff(TimeStamp)) < 1e-3) {
            time_lock = true;
            return true;
        }

        if (time_lock) {
			time_lock = false;
            return false;
        }

        if ((abs(inst.diff(TimeStamp)) < _shm->delay && inst >= TimeStamp)) 
            return true;

        return false;
    }

    MEAS_TYPE gnssprocesser::_getPOS(base_time gst, base_posdata::data_pos& pos, MEAS_INFO& m)
    {
        MEAS_TYPE res_type;
        double crt = gst.sow() + gst.dsec();
        base_time runEpoch = _gobs->load(_site, crt);

        double temp_t = runEpoch.sow() + runEpoch.dsec();

        int irc = gnss_proc_pvtflt::ProcessOneEpoch(runEpoch);
        if (irc < 0) {
            return MEAS_TYPE::NO_MEAS;
        }
        _get_result(runEpoch, pos);

        m.MeasVel = pos.vn; m.MeasPos = pos.pos; m.tmeas = pos.t;
        m._Cov_MeasVn = pos.Rvn; m._Cov_MeasPos = pos.Rpos;

        _sins->pos = Cart2Geod(m.MeasPos, false);
        _sins->vn = Cen(_sins->pos).transpose() * m.MeasVel;

        std::cout << TimeStamp.str_ymdhms("Pos Debug[0] Pos: ") << std::fixed << std::setprecision(6) << _sins->pos.transpose() << "; Vel: " << _sins->vn.transpose() << "\n";


        res_type = MEAS_TYPE::POS_MEAS;
        if (!double_eq(m.MeasVel.norm(), 0.0))
            res_type = POS_VEL_MEAS;

        //if (pos.PDOP > _shm->max_pdop) res_type = NO_MEAS;
        if (pos.nSat < _shm->min_sat) res_type = NO_MEAS;
        //if (_isBase && !pos.amb_state) res_type = NO_MEAS;

        time_lock = true;

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

	MEAS_TYPE gnssprocesser::_get_gnss_measurements()
	{
		_timeUpdate(TimeStamp);
		_epoch = TimeStamp;
		_amb_state = false;

		Triple crdapr = _grec->crd_arp(TimeStamp);
		if (double_eq(crdapr[0], 0.0) && double_eq(crdapr[1], 0.0) &&
			double_eq(crdapr[2], 0.0)) {
			_valid_crd_xml = false;
		}
		else {
			_valid_crd_xml = true;
		}
		if (!_valid_crd_xml) _sig_init_crd = 100.0;

		Eigen::Vector3d XYZ_INS = _sins->pos_ecef + _sins->Ceb * lever;
		Triple XYZ(XYZ_INS(0), XYZ_INS(1), XYZ_INS(2));
		_external_pos(XYZ, Triple());

		if (_prepareData() < 0)
		{
			if (baseprocesser::_spdlog) SPDLOG_LOGGER_ERROR(baseprocesser::_spdlog, string("t_gfgo_gins "), ("_prepareData Failed!"));
			cur_sat_prn.clear();
			_initial_prior = true;
			return NO_MEAS;
		}
		if (_isBase)
			_set_rec_info(_gallobj->obj(_site_base)->crd_arp(TimeStamp), _vBanc(3), _vBanc_base(3));


		if (_data.size() < _minsat)
		{
			if (baseprocesser::_spdlog) SPDLOG_LOGGER_ERROR(baseprocesser::_spdlog, string("t_gfgo_gins "), ("Not enough visible satellites!"));
			return  NO_MEAS;
		}

		_get_initial_value(); //for current epoch	

		return GNSS_MEAS;
	}

	void gnssprocesser::_gnss_amb_resolution()
	{
		if (is_first) {
			is_first = false;
			return;
		}
		if (_last_gnss_info->valid)
		{
			_pre_amb_resolution();
			_amb_resolution();
		}
	}

	void gnssprocesser::clearWindow()
	{
		if (_last_gnss_info != nullptr)
			delete _last_gnss_info;
		_last_gnss_info = nullptr;
		cur_sat_prn.clear();
		_DD_msg.clear();
		_vDD_msg.clear();
		_amb_manager->clearState();
		_fgo_info->rover_count = 0;
		_global_sat_id = -1;
		_global_amb_id = -1;
		_initial_prior = true;
		_amb_state = false;
		memset(_fgo_info->_para_amb, 0, sizeof(_fgo_info->_para_amb));
		memset(_fgo_info->_para_CRD, 0, sizeof(_fgo_info->_para_CRD));
	}

	bool gnssprocesser::_get_gdata(const base_time& now, vector<gnss_data_sats>* data_rover, vector<gnss_data_sats>* data_base)
	{
		if (/*_data.size()*/ _getData(now, data_rover, false) == 0)
		{
			if (baseprocesser::_spdlog)
			{
				SPDLOG_LOGGER_DEBUG(baseprocesser::_spdlog, _site + now.str_ymdhms(" no observation found at epoch: "));
			}
			return false;
		}
		// apply dcb
		if (_gallbias)
		{
			for (auto& itdata : _data)
			{
				itdata.apply_bias(_gallbias);
			}
		}

		vector<gnss_data_sats>::iterator it = _data.begin();
		string double_freq = "";
		string single_freq = "";


		_sat_freqs.clear();
		while (it != _data.end())
		{
			GOBSBAND b1 = _band_index[it->gsys()][FREQ_1];
			GOBSBAND b2 = _band_index[it->gsys()][FREQ_2];

			auto obsL1 = it->select_phase(b1);
			auto obsL2 = it->select_phase(b2);

			if (obsL1 == GOBS::X && obsL2 != GOBS::X || obsL1 != GOBS::X && obsL2 == GOBS::X)
			{
				single_freq += "  " + it->sat();
				_sat_freqs[it->sat()] = "1";
			}

			if (obsL1 != GOBS::X && obsL2 != GOBS::X)
			{
				double_freq += "  " + it->sat();
				_sat_freqs[it->sat()] = "2";
			}

			++it;
		}


		if (_isBase)
		{

			if (/*_data_base.size()*/_getData(now, data_base, true) == 0)
			{
				if (baseprocesser::_spdlog)
				{
					SPDLOG_LOGGER_DEBUG(baseprocesser::_spdlog, _site_base + now.str_ymdhms(" no observation found at epoch: "));
				}
				return false;
			}
			// apply dcb
			if (_gallbias)
			{
				for (auto& itdata_base : _data_base)
				{
					itdata_base.apply_bias(_gallbias);
				}
			}
		}


		if (_gallobj != nullptr) {
			auto it_data = _data.begin();
			while (it_data != _data.end()) {
				string sat_id = it_data->sat();
				shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(sat_id);


				if (sat_obj == nullptr) {
					if (baseprocesser::_spdlog) {
						SPDLOG_LOGGER_DEBUG(baseprocesser::_spdlog, "remove satellite " + sat_id + " due to missing object");
					}
					it_data = _data.erase(it_data);
				}
				else {
					shared_ptr<gnss_data_pcv> sat_pcv = sat_obj->pcv(now);
					if (sat_pcv == nullptr) {
						if (baseprocesser::_spdlog) {
							SPDLOG_LOGGER_DEBUG(baseprocesser::_spdlog, "remove satellite " + sat_id + " due to missing PCV data");
						}
						it_data = _data.erase(it_data);
					}
					else {
						++it_data;
					}
				}
			}


			if (_isBase && data_base != nullptr) {
				auto it_base = data_base->begin();
				while (it_base != data_base->end()) {
					string sat_id = it_base->sat();
					shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(sat_id);

					if (sat_obj == nullptr) {
						it_base = data_base->erase(it_base);
					}
					else {
						shared_ptr<gnss_data_pcv> sat_pcv = sat_obj->pcv(now);
						if (sat_pcv == nullptr) {
							it_base = data_base->erase(it_base);
						}
						else {
							++it_base;
						}
					}
				}
			}
		}


		return true;
	}

	void gnssprocesser::_get_initial_value()
	{
		if (_phase)
		{
			_syncAmb();
		}
		_predictCrd();
		_predictAmb();
		_initialized = true;
	}

	void gnssprocesser::_set_initial_value()
	{
		base_allpar params_add;
		_gtemp_params(*_param, params_add);
		_map_basedata.insert(make_pair(TimeStamp, _data_base));
		_map_param.insert(make_pair(TimeStamp, params_add));
		int lateset_obs_rover = _node_index_copy().back();
		//_rover_window[_rover_count]->setSatData(_data);
		vector<string> cur_sat_list;
		for (auto it : _data) cur_sat_list.push_back(it.sat());

		if (cur_sat_prn.empty())
			cout << "Refill current satellite list !" << endl;
		map<string, int>::iterator iter_cur_prn = cur_sat_prn.begin();  // Last list of sats
		auto cur_sat_map = _amb_manager->getSatMap();

		for (; iter_cur_prn != cur_sat_prn.end();)
		{
			string sat = iter_cur_prn->first;
			auto it_find = find_if(cur_sat_list.begin(), cur_sat_list.end(), [sat](const string& sat_name)
				{
					return sat_name == sat;
				});
			auto it_find1 = find_if(cur_sat_map.begin(), cur_sat_map.end(), [sat](const auto& sat_pair)
				{
					return sat_pair.second->_sat_name == sat;
				});

			if (it_find == cur_sat_list.end())  // If a satellite present in the previous epoch is absent in the current epoch, it indicates a loss of lock.
			{
				iter_cur_prn = cur_sat_prn.erase(iter_cur_prn);
				cout << "sat : " << sat << " lost tracking!!!" << endl;
			}
			else if (it_find1 == cur_sat_map.end())  // If a satellite present in the previous epoch is removed from the current satellite map, it indicates a new start..
			{
				iter_cur_prn = cur_sat_prn.erase(iter_cur_prn);
				cout << "sat : " << sat << " need initialization!!!" << endl;
			}
			else
				iter_cur_prn++;
		}

		for (auto it : _data)
		{
			string sat_name = it.sat();
			auto it_find = cur_sat_prn.find(sat_name);
			if (it_find == cur_sat_prn.end())
			{
				_global_sat_id++;
				cur_sat_prn[sat_name] = _global_sat_id;
				_amb_manager->addNewSat(TimeStamp, lateset_obs_rover, _global_sat_id, _global_amb_id, it, *_param);
			}
			else
			{
				if (it.islip())
				{
					cur_sat_prn.erase(sat_name);
					_global_sat_id++;
					cur_sat_prn[sat_name] = _global_sat_id;
					_amb_manager->addNewSat(TimeStamp, lateset_obs_rover, _global_sat_id, _global_amb_id, it, *_param);
				}
				else
					_amb_manager->addRover(TimeStamp.sow(), sat_name, lateset_obs_rover);

			}

		}
		_amb_manager->get_last_epoch_sats(TimeStamp.sow());
		assert(_amb_manager->cur_sats.size() == _data.size());
	}

	bool gnssprocesser::_gtemp_params(base_allpar& params, base_allpar& params_temp)
	{
		params_temp = params;
		base_par par_x_base; par_x_base.site = _site_base; par_x_base.parType = par_type::CRD_X; par_x_base.value(_gcrd_base[0]);
		base_par par_y_base; par_y_base.site = _site_base; par_y_base.parType = par_type::CRD_Y; par_y_base.value(_gcrd_base[1]);
		base_par par_z_base; par_z_base.site = _site_base; par_z_base.parType = par_type::CRD_Z; par_z_base.value(_gcrd_base[2]);
		base_par par_clk_rover; par_clk_rover.site = _site; par_clk_rover.parType = par_type::CLK; par_clk_rover.value(_gclk_rover);
		base_par par_clk_base; par_clk_base.site = _site_base; par_clk_base.parType = par_type::CLK; par_clk_base.value(_gclk_base);
		params_temp.addParam(par_x_base);
		params_temp.addParam(par_y_base);
		params_temp.addParam(par_z_base);
		params_temp.addParam(par_clk_rover);
		params_temp.addParam(par_clk_base);
		params_temp.reIndex();
		return true;
	}

	void gnssprocesser::_set_rec_info(const Triple& xyz_base, double clk_rover, double clk_base)
	{
		_gcrd_base = xyz_base;
		_gclk_rover = clk_rover;
		_gclk_base = clk_base;
	}

	int gnssprocesser::_combine_DD()
	{
		int flag = -1;
		_select_ref_sat();
		auto it_dd = _DD_msg.begin();
		while (it_dd != _DD_msg.end())
		{
			if (!_get_DD_data(*it_dd, _data_base))
			{
				cout << "[" << it_dd->rover_ref_sat.sat() << "-" << it_dd->rover_nonref_sat.sat() << it_dd->obs_type << "," << it_dd->freq << "]" << "DD msg wrong!" << endl;
				_DD_msg.erase(it_dd);
				continue;
			}

			it_dd++;
		}
		if (!_DD_msg.empty())
		{
			_vDD_msg.push_back(_DD_msg);
			flag = 1;
		}
		return flag;
	}

	void gnssprocesser::_select_ref_sat()
	{
		_DD_msg.clear();
		//_obs_index.clear();
		_sat_ref.clear();
		set<string> sysall = dynamic_cast<set_gen*>(_set)->sys();
		bool isSetRefSat = dynamic_cast<set_amb*>(_set)->isSetRefSat();
		bool isPhaseProcess = true;
		for (int obslevel = _obs_level; obslevel <= 3; obslevel++)
		{
			for (auto sys_iter = sysall.begin(); sys_iter != sysall.end(); sys_iter++)
			{
				enum GSYS sys = gnss_sys::str2gsys(*sys_iter);
				vector<GOBSBAND> band = dynamic_cast<set_gnss*>(_set)->band(sys);
				int nf = 5;
				if (band.size())
					nf = band.size();
				if (_observ == OBSCOMBIN::IONO_FREE)
					nf = 1;
				FREQ_SEQ f;
				string sat_ref;
				for (FREQ_SEQ freq = FREQ_1; freq <= 2 * nf; freq = (FREQ_SEQ)(freq + 1))
				{
					if (freq <= nf)
					{ //phase equations
						isPhaseProcess = true;
						f = freq;
					}
					else
					{ //code equations
						isPhaseProcess = false;
						f = (FREQ_SEQ)(freq - nf);
					}
					if (f > _frequency)
						continue;
					string sat;
					gnss_data_sats obs_sat_ref;
					enum GSYS gs;
					if (!isSetRefSat || (_observ == OBSCOMBIN::RAW_MIX && !isPhaseProcess))
						sat_ref.clear();
					//sat_ref.empty();
					if (sat_ref.empty())
					{
						for (auto it = _data.begin(); it != _data.end(); it++)
						{
							string sat = it->sat();

							gs = it->gsys();
							if (gs == QZS)
								gs = GPS;
							if (gs != sys)
								continue;
							if ((gs == BDS) && gnss_sys::bds_geo(sat))
								continue;
							if (/*(_observ == RAW_ALL||_observ == IONO_FREE||_observ == RAW_DOUBLE)
							//	 &&*/
								!_reset_amb && !_reset_par && it->islip())
								continue;

							GOBSBAND b = _band_index[gs][f];
							if (isPhaseProcess)
							{
								if (!it->band_avail(true).count(b))
								{
									continue;
								}
							}
							else
							{
								if (!it->band_avail(true).count(b) || !it->band_avail(false).count(b))
								{
									continue;
								}
							}
							int base_flag = 0;
							for (auto it_base = _data_base.begin(); it_base != _data_base.end(); it_base++)
							{
								if (it_base->sat() != sat)
								{
									continue;
								}

								if (isPhaseProcess)
								{
									if (it_base->band_avail(true).count(b))
									{
										base_flag = 1;
									}
								}
								else
								{
									if (it_base->band_avail(true).count(b) && it_base->band_avail(false).count(b))
									{
										base_flag = 1;
									}
								}
							}
							if (!base_flag)
							{
								continue;
							}


							if (sat_ref.empty())
							{
								sat_ref = sat;
								obs_sat_ref = *it;
								continue;
							}

							double e = it->ele_deg();
							double e2 = obs_sat_ref.ele_deg();
							if (e >= e2)
							{
								sat_ref = sat;
								sat_ref = it->sat();
								obs_sat_ref = *it;
							}

						} //end select sat_ref
					}
					if (sat_ref.empty())
						continue;
					if (_ipSatRep[sys] != "" && sat_ref != _ipSatRep[sys])
					{
						cerr << "refsat bug" << endl;
						continue;
					}
					if (freq == FREQ_1)
						_sat_ref.insert(sat_ref);
					gnss_data_sats ref_sat_data;
					auto it = find_if(_data.begin(), _data.end(), [sat_ref](gnss_data_sats it)
						{
							return it.sat() == sat_ref;
						});
					assert(it != _data.end());
					ref_sat_data = *it;
					for (auto it = _data.begin(); it != _data.end(); it++)
					{
						sat = it->sat();
						if (sat == sat_ref)
						{
							continue;
						}
						gs = it->gsys();
						if (gs == QZS)
							gs = GPS;
						if (gs != sys)
							continue;
						GOBSTYPE obstype = TYPE_C;
						if (isPhaseProcess)
							obstype = GOBSTYPE::TYPE_L;
						//_obs_index.push_back(make_pair(it->sat(), make_pair(f, obstype)));
						// delete unrecorded observations, added by hyChang
						GOBSBAND b = _band_index[gs][f];
						if (isPhaseProcess)
						{
							if (!it->band_avail(true).count(b))
							{
								continue;
							}
						}
						else
						{
							if (!it->band_avail(true).count(b) || !it->band_avail(false).count(b))
							{
								continue;
							}
						}
						int base_flag = 0;
						for (auto it_base = _data_base.begin(); it_base != _data_base.end(); it_base++)
						{
							if (it_base->sat() != sat)
							{
								continue;
							}

							if (isPhaseProcess)
							{
								if (it_base->band_avail(true).count(b))
								{
									base_flag = 1;
								}
							}
							else
							{
								if (it_base->band_avail(true).count(b) && it_base->band_avail(false).count(b))
								{
									base_flag = 1;
								}
							}
						}
						if (!base_flag)
						{
							continue;
						}

						DDEquMsg dd_msg(ref_sat_data, *it, obstype, f);
						//if (gins_window_size>0)
						//{
						//	dd_msg.ref_sat_global_id = cur_sat_prn[ref_sat_data.sat()];
						//	dd_msg.nonref_sat_global_id = cur_sat_prn[it->sat()];
						//}
						//else
						//{
						dd_msg.ref_sat_global_id = _amb_manager->get_sat_id(ref_sat_data.sat());
						//dd_msg.ref_sat_global_id = cur_sat_prn[ref_sat_data.sat()];
						dd_msg.nonref_sat_global_id = _amb_manager->get_sat_id(it->sat());
						//}

						assert(dd_msg.ref_sat_global_id != -1);
						assert(dd_msg.nonref_sat_global_id != -1);

						_DD_msg.push_back(dd_msg);

					} //end sat
				}      //end f
			}          //end sys
		}
	}

	bool gnssprocesser::_get_DD_data(DDEquMsg& dd_msg, vector<gnss_data_sats> base_sat_data)
	{
		gnss_data_sats rover_ref_sat = dd_msg.rover_ref_sat;
		gnss_data_sats rover_nonref_sat = dd_msg.rover_nonref_sat;
		FREQ_SEQ   freq = dd_msg.freq;
		GOBSTYPE   obstype = dd_msg.obs_type;
		gnss_data_sats base_ref_sat, base_nonref_sat;
		if (base_sat_data.empty()) return false;
		int ref_i = 0;
		int nonref_i = 0;

		for (auto it : base_sat_data)
		{
			if (it.sat() == rover_ref_sat.sat())
			{
				ref_i = 1;
				base_ref_sat = it;
			}
			if (it.sat() == rover_nonref_sat.sat())
			{
				nonref_i = 1;
				base_nonref_sat = it;
			}
		}
		if (!ref_i || !nonref_i)
		{
			cout << "no common view between base and rover!!!" << endl;
			return false;
		}
		map<FREQ_SEQ, GOBSBAND> crt_bands = _band_index[rover_ref_sat.gsys()];
		if (crt_bands.empty()) return false;
		if (freq > _frequency) return false;
		dd_msg.band = crt_bands[freq];
		dd_msg.base_ref_sat = base_ref_sat;
		dd_msg.base_nonref_sat = base_nonref_sat;
		dd_msg.time = rover_ref_sat.epoch();
		dd_msg.base_site = _site_base;
		dd_msg.rover_site = _site;
		return true;
	}

	bool gnssprocesser::_getRobustFixedPosition()
	{
		std::set<std::string> ambs = _param->amb_prns();
		int nsat = ambs.size();

		std::cout << std::boolalpha;
		std::cout << "\n================ Robust Fixed Position Check ================\n";
		std::cout << "Ambiguity state : "
			<< _amb_state
			<< "        [" << (_amb_state ? "PASS" : "FAIL") << "]\n";

		std::cout << "Satellite number: "
			<< nsat
			<< " (>10 required)"
			<< "    [" << (nsat > 10 ? "PASS" : "FAIL") << "]\n";

		std::cout << "AR Ratio        : "
			<< std::fixed << std::setprecision(2)
			<< _ambfix->get_ratio()
			<< " (>5.00 required)"
			<< "    [" << (_ambfix->get_ratio() > 5.0 ? "PASS" : "FAIL") << "]\n";

		bool valid = _amb_state &&
			nsat > 10 &&
			_ambfix->get_ratio() > 5.0;

		std::cout << "-------------------------------------------------------------\n";
		std::cout << "Final Result    : "
			<< (valid ? "FIXED POSITION" : "RETURN ZERO")
			<< "\n";
		std::cout << "=============================================================\n";

		return valid;
	}

	bool gnssprocesser::_remove_outlier_sat(const pair<string, int>& outlier)
	{
		if (outlier.first != " ")
		{

			if (_amb_manager->cur_sats.size() > _minsat)
			{
				pair<string, int> sat_id = outlier;

				int latest_obs_index = _node_index_copy().size() - 1;
				int latest_obs_epoch = _node_index_copy()[latest_obs_index];
				auto it_DD = _vDD_msg[latest_obs_index].begin();
				int sat_global_id = outlier.second;

				while (it_DD != _vDD_msg[latest_obs_index].end())
				{
					if (it_DD->ref_sat_global_id == sat_global_id || it_DD->nonref_sat_global_id == sat_global_id)
					{
						it_DD = _vDD_msg[latest_obs_index].erase(it_DD);
						continue;
					}

					it_DD++;

				}

				_amb_manager->removeSat(sat_global_id, latest_obs_epoch);
				_amb_manager->get_last_epoch_sats(TimeStamp.sow());//update cur sat map

				_last_gnss_info->valid = false;
				auto it = cur_sat_prn.find(sat_id.first);
				if (it != cur_sat_prn.end())
					cur_sat_prn.erase(sat_id.first);
				return true;
			}
			else return false;
		}

		return true;
	}

	bool gnssprocesser::_pre_amb_resolution()
	{
		base_allpar construct_para = _all_para_win;
		int nobs_total, npar_number;
		Matrix A_fgo;
		Symmetric P_fgo;
		Vector l_fgo, dx_fgo;
		Symmetric Qx0_fgo, Qx_fgo;
		double vtpv_fgo;
		nobs_total = _last_gnss_info->linearized_jacobians.rows();
		npar_number = _last_gnss_info->linearized_jacobians.cols();
		assert(npar_number == construct_para.parNumber());
		//cout << "nobs_total：" << nobs_total << endl;
		//cout << "npar_number：" << npar_number << endl;
		A_fgo.resize(nobs_total, npar_number);
		A_fgo.setZero();
		P_fgo.resize(nobs_total);
		P_fgo.setZero();
		l_fgo.resize(nobs_total);
		l_fgo.setZero();
		dx_fgo.resize(npar_number);
		dx_fgo.setZero();
		Qx0_fgo.resize(npar_number);
		Qx0_fgo.setZero();
		Qx_fgo.resize(npar_number);
		Qx_fgo.setZero();
		_sig_unit = _last_gnss_info->sig_unit;
		vtpv_fgo = _last_gnss_info->vtpv;
		//for Qx	
		for (int i = 0; i < npar_number; i++)
		{
			for (int j = 0; j < npar_number; j++)
			{
				//Qx_fgo(i + 1, j + 1) = _last_gnss_info->Qx(i, j);
				double val = _last_gnss_info->Qx(i, j);
				Qx0_fgo.set(val, i, j);	
			}
		}

		Eigen::IOFormat fmt(
			4,                  // 小数位数
			0,                  // StreamPrecision=0，使用上面的精度
			", ",               // 元素分隔符
			"\n",               // 行分隔符
			"[ ",               // 行开始
			" ]",               // 行结束
			"[\n",              // 整个矩阵开始
			"\n]"
		);

		std::cout << "Qx =\n" << _last_gnss_info->Qx.format(fmt) << std::endl;

		Qx_fgo = Qx0_fgo;
		//for A
		for (int i = 0; i < nobs_total; i++)
		{
			for (int j = 0; j < npar_number; j++)
			{
				A_fgo(i, j) = _last_gnss_info->linearized_jacobians(i, j);
			}
		}
		//for P
		for (int i = 0; i < nobs_total; i++)
		{
			for (int j = 0; j < nobs_total; j++)
			{
				double val = _last_gnss_info->weight(i, j);
				P_fgo.set(val, i, j);
			}
		}
		//for l	
		for (int i = 0; i < nobs_total; i++)
		{
			l_fgo(i) = _last_gnss_info->linearized_residuals(i);
		}
		_filter->add_data(construct_para, dx_fgo, Qx_fgo, _sig_unit, Qx0_fgo);
		_filter->add_data(A_fgo, P_fgo, l_fgo);
		_filter->add_data(vtpv_fgo, nobs_total, npar_number);

		return true;;
	}

	void gnssprocesser::_slide_window()
	{
	}

	int gnssprocesser::_gobs_outlier_detection(pair<string, int>& outlier)
	{
		pair<string, int> sat_id;
		int idx = -1;
		if (_last_gnss_info->valid)
		{
			Eigen::VectorXd v = _last_gnss_info->v_norm;
			//std::cout << v << endl;
			int nobs = v.rows();
			double max = 0.0;

			for (int i = 0; i < nobs; i++)
			{
				if (fabs(v(i)) > max && fabs(v(i)) > _max_res_norm)
				{
					max = fabs(v(i));
					idx = i;
				}
			}
			if (idx >= 0)
			{
				sat_id = _gnss_obs_index[idx].first;
				outlier = sat_id;
				int id = sat_id.second;
				auto it_find = find_if(_removed_sats.begin(), _removed_sats.end(), [id](pair<string, int>& sat_id)
					{
						return sat_id.second == id;

					});
				if (it_find == _removed_sats.end())
					_removed_sats.push_back(sat_id);

				string obsType = gobstype2str(_gnss_obs_index[idx].second.second);
				ostringstream os;

				std::cout << _site << " outlier (" << obsType << _gnss_obs_index[idx].second.first << ") " << sat_id.first
					<< " v: " << fixed << setw(16) << right << setprecision(3) << max << endl;

				if (baseprocesser::_spdlog)
					SPDLOG_LOGGER_ERROR(baseprocesser::_spdlog, string("gpvtfgo "), TimeStamp.str_ymdhms(" epoch ") + os.str());
			}

			if (_vDD_msg[_node_index_copy().size() - 1].size() - _removed_sats.size() <= 2)
				idx = -1;

		}

		return idx;
	}

	bool gnssprocesser::_check_outlier(const string& sat)
	{
		auto it_find = find_if(outlier_sats.begin(), outlier_sats.end(), [sat](const string& sat_name)
			{
				return sat_name == sat;

			});
		if (it_find != outlier_sats.end())
			return true;
		else
			return false;
	}

	void gnssprocesser::_posteriori_test(ceres::Problem& problem)
	{
		_all_para_win.delAllParam();
		ceres::LossFunction* loss_function;
		loss_function = new ceres::HuberLoss(_loss_func_value);
		//loss_function = new ceres::CauchyLoss(_loss_func_value);
		GNSSInfo* gnss_info = new GNSSInfo();
		int _node_index = _node_index_copy().back();
		int _node_size = _node_index_copy().size();

		vector<DDEquMsg> dd_msg = _vDD_msg[_node_size - 1];
		base_time crt = dd_msg.begin()->time;

		if (dd_msg.size() >= 1)
		{
			//constrcut window para_index
			vector<vector<int>> pose_para_col_index;
			vector<vector<int>> amb_para_col_index;
			int total_para_size = 0;
			vector<double*> _parameter_blocks;
			vector<par_type> crd_partype{ par_type::CRD_X ,par_type::CRD_Y ,par_type::CRD_Z };
			vector<par_type> att_partype{ par_type::ATT_X ,par_type::ATT_Y ,par_type::ATT_Z };
			vector<int> posei;
			base_quat qi = base_quat(_fgo_info->_para_pose[_node_index][6], _fgo_info->_para_pose[_node_index][3], _fgo_info->_para_pose[_node_index][4], _fgo_info->_para_pose[_node_index][5]);
			//qi.normlize(qi);
			Eigen::Vector3d rv = base_att_trans::q2rv(qi);
			_parameter_blocks.push_back(_fgo_info->_para_pose[_node_index]);
			for (int j = 0; j < 6; j++)
			{
				if (j < 3)
				{
					base_par par_crd;
					par_crd.site = _site;
					par_crd.parType = crd_partype[j];
					par_crd.value(_fgo_info->_para_pose[_node_index][j]);
					par_crd.beg = TimeStamp;
					par_crd.end = TimeStamp;
					par_crd.index = j;
					_all_para_win.addParam(par_crd);
				}
				if (j >= 3)
				{
					base_par par_att;
					par_att.site = _site;
					par_att.parType = att_partype[j - 3];
					par_att.value(rv(j - 3));
					par_att.beg = TimeStamp;
					par_att.end = TimeStamp;
					par_att.index = j;
					_all_para_win.addParam(par_att);
				}
				int id = j;
				posei.push_back(id);
				total_para_size = total_para_size + 1;
			}
			pose_para_col_index.push_back(posei);

			int amb_index_start = 6;
			map<int, int> amb_col_id;
			int amb_size = -1;
			_gnss_obs_index.clear();
			int unused_DD_nums = 0;

			map<base_time, vector<gnss_data_sats>>::const_iterator base_iter = _map_basedata.find(crt);
			map<base_time, base_allpar>::const_iterator param_iter = _map_param.find(crt);
			if (base_iter != _map_basedata.end() && param_iter != _map_param.end()) {

				for (auto& dd_iter : dd_msg)
				{
					if (!_get_DD_data(dd_iter, base_iter->second))
					{
						unused_DD_nums++;
						continue;
					}
					pair<string, string> base_rover_site = make_pair(dd_iter.base_site, dd_iter.rover_site);
					pair<FREQ_SEQ, GOBSBAND> freq_band = make_pair(dd_iter.freq, dd_iter.band);
					vector<pair<gnss_data_sats, gnss_data_sats>> DD_sat_data;
					DD_sat_data.push_back(make_pair(dd_iter.base_ref_sat, dd_iter.rover_ref_sat));
					DD_sat_data.push_back(make_pair(dd_iter.base_nonref_sat, dd_iter.rover_nonref_sat));

					GOBSTYPE  obstype = dd_iter.obs_type;
					_gnss_obs_index.push_back(make_pair(make_pair(dd_iter.rover_nonref_sat.sat(), dd_iter.nonref_sat_global_id), make_pair(dd_iter.freq, obstype)));
					if (obstype == GOBSTYPE::TYPE_C)
					{
						PseudorangeDDINGFactor* pinsf = new PseudorangeDDINGFactor(dd_iter.time, base_rover_site, param_iter->second, DD_sat_data, _gbias_model, freq_band, lever);
						GNSSResidualBlockInfo* residual_block_info = new GNSSResidualBlockInfo(pinsf, NULL, vector<double*> {_fgo_info->_para_pose[_node_index]});
						map<long, vector<int>> para_index;
						para_index[reinterpret_cast<long>(_fgo_info->_para_pose[_node_index])] = pose_para_col_index[0];
						gnss_info->addResidualBlockInfo(residual_block_info, para_index);
					}
					//if (obstype == GOBSTYPE::TYPE_L)
					//{
					//	int index = dd_iter.ref_sat_global_id;
					//	int nonindex = dd_iter.nonref_sat_global_id;
					//	vector<int> amb_id12(2);
					//	amb_id12[0] = _amb_manager->getAmbSearchIndex(make_pair(dd_iter.ref_sat_global_id, dd_iter.freq));
					//	amb_id12[1] = _amb_manager->getAmbSearchIndex(make_pair(dd_iter.nonref_sat_global_id, dd_iter.freq));

					//	if (amb_id12[0] == -1 || amb_id12[1] == -1)
					//		continue;

					//	for (int i = 0; i < amb_id12.size(); i++)
					//	{
					//		int amb_id = amb_id12[i];
					//		if (amb_col_id.find(amb_id) == amb_col_id.end())
					//		{
					//			amb_size++;
					//			int amb_para_id = amb_index_start + amb_size;
					//			amb_col_id[amb_id] = amb_para_id;
					//			amb_para_col_index.push_back(vector<int> {amb_para_id});
					//			_amb_manager->addGpara(_all_para_win, amb_id, _fgo_info->_para_amb[amb_id][0]);
					//			_parameter_blocks.push_back(_fgo_info->_para_amb[amb_id]);
					//		}
					//	}
					//	map<long, vector<int>> para_index;
					//	//cout << "para_col_index: " << para_col_index.size() << " id1: " << _rover_count + 1+ id1 << " id2: " << _rover_count + 1+id2 << endl;
					//	double* addr1 = _fgo_info->_para_amb[amb_id12[0]];
					//	double* addr2 = _fgo_info->_para_amb[amb_id12[1]];
					//	para_index[reinterpret_cast<long>(_fgo_info->_para_pose[_node_index])] = pose_para_col_index[0];
					//	para_index[reinterpret_cast<long>(addr1)] = vector<int>{ amb_col_id[amb_id12[0]] };
					//	para_index[reinterpret_cast<long>(addr2)] = vector<int>{ amb_col_id[amb_id12[1]] };
					//	CarrierphaseDDINGFactor* linsf = new CarrierphaseDDINGFactor(dd_iter.time, base_rover_site, param_iter->second, DD_sat_data, _gbias_model, freq_band, lever);
					//	GNSSResidualBlockInfo* residual_block_info = new GNSSResidualBlockInfo(linsf, NULL, vector<double*> {_fgo_info->_para_pose[_node_index], _fgo_info->_para_amb[amb_id12[0]], _fgo_info->_para_amb[amb_id12[1]]});
					//	gnss_info->addResidualBlockInfo(residual_block_info, para_index);
					//}
				}
			}

			total_para_size = total_para_size + amb_size + 1;
			//construct variances by ceres solver		
			ceres::Covariance::Options options_co;
			options_co.algorithm_type = ceres::DENSE_SVD;
			options_co.min_reciprocal_condition_number = 1e-50;
			//options_co.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
			//options_co.apply_loss_function = false; //optional, true or false is depended on the reliability of covariance
			ceres::Covariance covariance(options_co);
			std::vector<const double*> covariance_blocks; //all parameter_blocks		
			for (int i = 0; i < _parameter_blocks.size(); i++)
			{
				covariance_blocks.push_back(_parameter_blocks[i]);
			}
			Matrix Qx = Matrix::Zero(total_para_size + 1, total_para_size + 1);
			try
			{
				covariance.Compute(covariance_blocks, &problem);
				covariance.GetCovarianceMatrix(covariance_blocks, Qx.data());
				Matrix_remRC(Qx, 6, 6);
			}
			catch (...)
			{
				cout << "Covariance Compute Failed" << endl;
			}

			if (_vDD_msg[_node_index_copy().size() - 1].size() - unused_DD_nums >= 1)
			{
				gnss_info->constructEqu_fromCeres(Qx);
			}
			else
			{
				cout << "DD_msgs: " << _vDD_msg[_node_index_copy().size() - 1].size() << "   unused_DD_nums: " << unused_DD_nums << endl;
			}
		}
		if (_last_gnss_info) delete _last_gnss_info;
		_last_gnss_info = gnss_info;

	}

	gnssprocesser::DDEquMsg::DDEquMsg(const gnss_data_sats& _ref_sat, const gnss_data_sats& _nonref_sat, const GOBSTYPE& _obs_type, const FREQ_SEQ& _freq)
		: rover_ref_sat(_ref_sat),
		rover_nonref_sat(_nonref_sat),
		obs_type(_obs_type),
		freq(_freq),
		ref_sat(_ref_sat.sat()),
		nonref_sat(_nonref_sat.sat())
	{
		this->time = _nonref_sat.epoch();
	}

	int gnssprocesser::ProcessOneEpoch() {
		if (_get_gnss_measurements() == NO_MEAS) 
			return 0;

		_set_frame_pose();
		_set_initial_value();

		if (_combine_DD() < 0)
		{
			if (baseprocesser::_spdlog) SPDLOG_LOGGER_ERROR(baseprocesser::_spdlog, string("t_gfgo_gins "), ("Combining the Double-Difference Pairs Failed!"));
			cur_sat_prn.clear();
			_initial_prior = true;
			return  0;
		}

		return 1;
	}

	void gnssprocesser::_addResidualBlocks(ceres::Problem& problem) {

		TicToc fgo_gins;
		_removed_sats.clear();

		if (_node_index_copy().size() == 0) return;
		int latest_obs_index = _node_index_copy().size() - 1;

		ceres::LossFunction* loss_function = new ceres::HuberLoss(_loss_func_value);

		_obs_index.clear();

		//std::cout << "Current Time: " << TimeStamp.str_ymdhms() << std::endl;

		std::vector<double> residuals;

		for (int i = 0; i <= latest_obs_index; i++)
		{
			double cost_save = _fgo_info->cost;
			int obs_node = _node_index_copy()[i];
			vector<DDEquMsg> DD_tmp = _vDD_msg[i];
			base_time crt = DD_tmp.begin()->time;
			map<base_time, vector<gnss_data_sats>>::const_iterator base_iter = _map_basedata.find(crt);
			map<base_time, base_allpar>::const_iterator param_iter = _map_param.find(crt);                        
			if (base_iter == _map_basedata.end() || param_iter == _map_param.end())
				continue;
			int dd_equ_count = 0;
			for (auto& dd_iter : DD_tmp)
			{
				if (!_get_DD_data(dd_iter, base_iter->second)) continue;
				pair<string, string> base_rover_site = make_pair(dd_iter.base_site, dd_iter.rover_site);
				pair<FREQ_SEQ, GOBSBAND> freq_band = make_pair(dd_iter.freq, dd_iter.band);
				vector<pair<gnss_data_sats, gnss_data_sats>> DD_sat_data;
				DD_sat_data.push_back(make_pair(dd_iter.base_ref_sat, dd_iter.rover_ref_sat));
				DD_sat_data.push_back(make_pair(dd_iter.base_nonref_sat, dd_iter.rover_nonref_sat));

				GOBSTYPE  obstype = dd_iter.obs_type;
				_obs_index.push_back(make_pair(dd_iter.rover_nonref_sat.sat(), make_pair(dd_iter.freq, obstype)));
				if (obstype == GOBSTYPE::TYPE_C)
				{
					PseudorangeDDINGFactor* pinsf = new PseudorangeDDINGFactor(dd_iter.time, base_rover_site, param_iter->second, DD_sat_data, _gbias_model, freq_band, lever);
					problem.AddResidualBlock(pinsf, NULL, _fgo_info->_para_pose[obs_node]);
				}
				if (obstype == GOBSTYPE::TYPE_L)
				{
					int id1, id2;
					id1 = _amb_manager->getAmbSearchIndex(make_pair(dd_iter.ref_sat_global_id, dd_iter.freq));
					id2 = _amb_manager->getAmbSearchIndex(make_pair(dd_iter.nonref_sat_global_id, dd_iter.freq));
					if (id1 == -1 || id2 == -1)
						continue;
					CarrierphaseDDINGFactor* linsf = new CarrierphaseDDINGFactor(dd_iter.time, base_rover_site, param_iter->second, DD_sat_data, _gbias_model, freq_band, lever);
					problem.AddResidualBlock(linsf, NULL, _fgo_info->_para_pose[obs_node], _fgo_info->_para_amb[id1], _fgo_info->_para_amb[id2]);
				}
				dd_equ_count++;
			}
			assert(dd_equ_count == DD_tmp.size());

			//problem.Evaluate(
			//	ceres::Problem::EvaluateOptions(),
			//	&_fgo_info->cost,
			//	&residuals,
			//	nullptr,
			//	nullptr);

			//std::cout
			//	<< std::fixed
			//	<< std::setprecision(10)
			//	<< "range cost [" << i << "] = "
			//	<< _fgo_info->cost - cost_save
			//	<< std::endl;
		}

		if (1)
		{
			for (int i = 0; i < _amb_manager->ambiguity_ids.size(); i++)
			{
				int amb_id = _amb_manager->ambiguity_ids[i];
				double initial_amb = _fgo_info->_para_amb[amb_id][0];
				InitialGnssAMB* amb_prior = new InitialGnssAMB(initial_amb);
				problem.AddResidualBlock(amb_prior, NULL, _fgo_info->_para_amb[amb_id]);

				std::cout << "AMB[" << std::setw(3) << amb_id << "] = "
					<< std::fixed << std::setprecision(6)
					<< initial_amb << std::endl;
			}
		}

		//double cost_save = _fgo_info->cost;
		//problem.Evaluate(
		//	ceres::Problem::EvaluateOptions(),
		//	&_fgo_info->cost,
		//	&residuals,
		//	nullptr,
		//	nullptr);

		//std::cout
		//	<< "amb cost = "
		//	<< _fgo_info->cost - cost_save
		//	<< std::endl;
	}

	void gnssprocesser::_addMarginInfo() {

		if (!_fgo_info->time_to_margin()) return;

		if (_node_index_copy().size() == 0) return;
		int first_obs_node = _node_index_copy()[0];
		if (first_obs_node != 0) return;

		std::vector<DDEquMsg> dd_msg = _vDD_msg[0];
		base_time crt = dd_msg.begin()->time;
		std::map<base_time, vector<gnss_data_sats>>::const_iterator base_iter = _map_basedata.find(crt);
		std::map<base_time, base_allpar>::const_iterator param_iter = _map_param.find(crt);
		if (base_iter != _map_basedata.end() && param_iter != _map_param.end()) {
			for (auto& dd_iter : dd_msg)
			{
				if (!_get_DD_data(dd_iter, base_iter->second)) continue;
				pair<string, string> base_rover_site = make_pair(dd_iter.base_site, dd_iter.rover_site);
				pair<FREQ_SEQ, GOBSBAND> freq_band = make_pair(dd_iter.freq, dd_iter.band);
				vector<pair<gnss_data_sats, gnss_data_sats>> DD_sat_data;
				DD_sat_data.push_back(make_pair(dd_iter.base_ref_sat, dd_iter.rover_ref_sat));
				DD_sat_data.push_back(make_pair(dd_iter.base_nonref_sat, dd_iter.rover_nonref_sat));

				GOBSTYPE  obstype = dd_iter.obs_type;
				if (obstype == GOBSTYPE::TYPE_C)
				{
					PseudorangeDDINGFactor* pinsf = new PseudorangeDDINGFactor(dd_iter.time, base_rover_site, param_iter->second, DD_sat_data, _gbias_model, freq_band, lever);
					ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(pinsf, NULL, vector<double*>{_fgo_info->_para_pose[0]}, vector<int>{0});
					_fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
				}
				if (obstype == GOBSTYPE::TYPE_L)
				{
					int id1, id2;
					id1 = _amb_manager->getAmbSearchIndex(make_pair(dd_iter.ref_sat_global_id, dd_iter.freq));
					id2 = _amb_manager->getAmbSearchIndex(make_pair(dd_iter.nonref_sat_global_id, dd_iter.freq));
					if (id1 == -1 || id2 == -1)
						continue;
					vector<int> drop_set{ 0 };
					if (_amb_manager->getAmbStartRoverID(id1) == 0 && _amb_manager->getAmbStartRoverID(id2) == 0)
					{
						if (_amb_manager->getAmbEndRoverID(id1) == 0)
							drop_set.push_back(1);
						if (_amb_manager->getAmbEndRoverID(id2) == 0)
							drop_set.push_back(2);

						CarrierphaseDDINGFactor* linsf = new CarrierphaseDDINGFactor(dd_iter.time, base_rover_site, param_iter->second, DD_sat_data, _gbias_model, freq_band, lever);
						ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(linsf, NULL, vector<double*>{_fgo_info->_para_pose[0], _fgo_info->_para_amb[id1], _fgo_info->_para_amb[id2]}, drop_set);
						_fgo_info->marginalization_info->addResidualBlockInfo(residual_block_info);
					}
				}
			}
		}
		
		vector<int> cur_amb = _amb_manager->getCurWinAmb();
		for (int i = 0; i < cur_amb.size(); i++)
		{
			_fgo_info->addr_shift[reinterpret_cast<long>(_fgo_info->_para_amb[cur_amb[i]])] = _fgo_info->_para_amb[cur_amb[i]];
		}
	}

	void gnssprocesser::slide_window() {

		if (_node_index_copy().size() == 0) return;
		auto& gnss_node_index = _node_index();

		if (gnss_node_index.size() == 0) return;

		for (auto& it : gnss_node_index) {
			it--;
		}

		if (gnss_node_index[0] < 0) {
			gnss_node_index.erase(gnss_node_index.begin());
			_vDD_msg.erase(_vDD_msg.begin());
		}

		_amb_manager->slidingWindow();
	}
}