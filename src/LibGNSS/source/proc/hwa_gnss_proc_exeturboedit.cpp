#include "hwa_gnss_proc_exeturboedit.h"

using namespace std;

namespace hwa_gnss
{
    gnss_proc_exeturboedit::gnss_proc_exeturboedit(set_base *set, base_log spdlog, gnss_all_obs *gobs, gnss_all_nav *gnav) : _set(set),
                                                                                                           _spdlog(spdlog),
                                                                                                           _allobs(gobs),
                                                                                                           _allnav(gnav),
                                                                                                           _allobj(nullptr),
                                                                                                           _check_sf(false),
                                                                                                           _check_pc(true),
                                                                                                           _amb_output(true),
                                                                                                           _simulation(false),
                                                                                                           _check_short(true),
                                                                                                           _allambflag(nullptr),
                                                                                                           _site_kinematic(false),
                                                                                                           _check_statistics(true),
                                                                                                           _outliers_proc(set, spdlog)
    {
        // common intialization
        _initCommonProcess();

        // init post-processing / real-time-processing
        if (_liteMode || _realtime)
            _initRealTimeProcess();

        else
            _initPostProcess();

        // output control infomation
        _outputControlInfo();
    }

    gnss_proc_exeturboedit::gnss_proc_exeturboedit(set_base *set, base_log spdlog,  base_all_proc *gdata) : _set(set),
                                                                                            _spdlog(spdlog),
                                                                                            _check_sf(false),
                                                                                            _check_pc(true),
                                                                                            _amb_output(true),
                                                                                            _simulation(false),
                                                                                            _check_short(true),
                                                                                            _allambflag(nullptr),
                                                                                            _site_kinematic(false),
                                                                                            _check_statistics(true),
                                                                                            _outliers_proc(set, spdlog)
    {
        // get data
        _allobs = dynamic_cast<gnss_all_obs *>((*gdata)[base_data::ALLOBS]);
        _allnav = dynamic_cast<gnss_all_nav *>((*gdata)[base_data::GRP_EPHEM]);
        _allobj = dynamic_cast<gnss_all_obj *>((*gdata)[base_data::ALLOBJ]);

        // common intialization
        _initCommonProcess();

        // init post-processing / real-time-processing
        if (_liteMode || _realtime)
            _initRealTimeProcess();

        else
            _initPostProcess();

        // output control infomation
        _outputControlInfo();
    }

    gnss_proc_exeturboedit::gnss_proc_exeturboedit(set_base* set, base_log spdlog,  base_all_proc* gdata, std::string site, bool isBase) : _set(set),
                                                                                                         _spdlog(spdlog),
                                                                                                         _crt_rec(site),
                                                                                                         _isBase(isBase),
                                                                                                         _check_sf(false),
                                                                                                         _check_pc(true),
                                                                                                         _amb_output(true),
                                                                                                         _simulation(false),
                                                                                                         _check_short(true),
                                                                                                         _allambflag(nullptr),
                                                                                                         _site_kinematic(false),
                                                                                                         _check_statistics(true),
                                                                                                         _outliers_proc(set, spdlog)
    {
        // get data
        _allobs = dynamic_cast<gnss_all_obs*>((*gdata)[base_data::ALLOBS]);
        _allnav = dynamic_cast<gnss_all_nav*>((*gdata)[base_data::GRP_EPHEM]);
        _allobj = dynamic_cast<gnss_all_obj*>((*gdata)[base_data::ALLOBJ]);

        // common intialization
        _initCommonProcess();

        // init post-processing / real-time-processing
        if (_liteMode || _realtime)
            _initRealTimeProcess();

        else
            _initPostProcess();

        // output control infomation
        _outputControlInfo();
    }

    gnss_proc_exeturboedit::gnss_proc_exeturboedit(set_base* set, base_log spdlog,  base_all_proc* gdata, std::string site) : _set(set),
        _spdlog(spdlog),
        _crt_rec(site),
        _check_sf(false),
        _check_pc(true),
        _amb_output(true),
        _simulation(false),
        _check_short(true),
        _allambflag(nullptr),
        _site_kinematic(false),
        _check_statistics(true),
        _outliers_proc(set, spdlog)
    {
        // get data
        _allobs = dynamic_cast<gnss_all_obs*>((*gdata)[base_data::ALLOBS]);
        _allnav = dynamic_cast<gnss_all_nav*>((*gdata)[base_data::GRP_EPHEM]);
        _allobj = dynamic_cast<gnss_all_obj*>((*gdata)[base_data::ALLOBJ]);

        // common intialization
        _initCommonProcess();

        // init post-processing / real-time-processing
        if (_liteMode || _realtime)
            _initRealTimeProcess();

        else
            _initPostProcess();

        // output control infomation
        _outputControlInfo();
    }

    gnss_proc_exeturboedit::~gnss_proc_exeturboedit()
    {
        if (_allambflag)
        {
            delete _allambflag;
            _allambflag = nullptr;
        }

        for (auto it : _rec_list)
        {

            if (_sppflt[it])
            {
                delete _sppflt[it];
                _sppflt[it] = nullptr;
            }
        }

        if (_debug_turbo)
        {
            if (_debug_turbo->is_open())
                _debug_turbo->close();

            delete _debug_turbo;
            _debug_turbo = nullptr;
        }

        if (_cycleslipfile)
        {
            if (_cycleslipfile->is_open())
            {
                _cycleslipfile->close();
            };
            delete _cycleslipfile;
        }
    }

    bool gnss_proc_exeturboedit::ProcessBatch(std::string site, const base_time &begT, const base_time &endT, double sampling)
    {
        _crt_rec = site;
        _interval = sampling;

        // Check Time Duration
        if (begT > endT) //lvhb begT>endT
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Error in {} time settings : beg time {} >= end time {}", _crt_rec, begT.str_ymdhms(), endT.str_ymdhms());
            return false;
        }

        // Real_time Turboedit
        if (_realtime || _liteMode)
        {
            if (!_realtimeProcess(begT, endT))
                return false;
        }
        else
        {
            // =================================================
            if (!_amb_output)
                return true; //用于不覆盖原有log文件
            // =================================================

            if (_rec_list.find(_crt_rec) == _rec_list.end())
                return false;

            if (_allambflag)
            {
                set<std::string> sites = _allambflag->getSiteList();
                if (sites.find(_crt_rec) != sites.end())
                    return false;
            }

            // Check simulation and kinematic
            if (_simulation && _site_kinematic)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Error in {}, Not support kinematic solution for simulation data", _crt_rec);
                return false;
            }

            _epo_num = floor((_end_time - _beg_time) / sampling) + 1;

            // loop band combination
            for (unsigned int i = 1; i <= _post_all_band.size(); i++)
            {
                _post_crt_band = _post_all_band[i];
                _cycleslip[_crt_rec].push_back(std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit()));

                if (!_postProcess())
                    return false;
            }
        }
        return true;
    }

    void gnss_proc_exeturboedit::setGLOFreqID(gnss_all_satinfo *gsatpars)
    {
        if (gsatpars)
        {
            gnss_data_satparam sat_param;
            for (auto sat : _sat_list)
            {
                if (sat.substr(0, 1) != "R")
                    continue;

                if (gsatpars->getSatinfoUsed(sat, _beg_time, _end_time, sat_param))
                {
                    _glonass_freq_id[sat] = sat_param.fid();
                }
            }
        }
    }

    void gnss_proc_exeturboedit::set_site_crd(std::string site, Triple crd)
    {
        std::map<std::string, Triple>::iterator it;
        it = _rec_crds.find(site);
        if (it == _rec_crds.end())
        {
            _rec_crds[site] = crd;
        }
    }

    Triple gnss_proc_exeturboedit::get_site_crd(std::string site)
    {
        std::map<std::string, Triple>::iterator it;
        it = _rec_crds.find(site);
        if (it == _rec_crds.end())
        {
            return _rec_crds[site];
        }
        else
            return Triple(0, 0, 0);
    }

    void gnss_proc_exeturboedit::setSingleEpoData(std::vector<gnss_data_sats> *input_data)
    {
        //modified
        _inputEpoData.clear();
        _map_nppdata_idx.clear();

        return;
    }

    base_log gnss_proc_exeturboedit::spdlog()
    {
        return _spdlog;
    }

    void gnss_proc_exeturboedit::spdlog(base_log spdlog)
    {
        // set spdlog
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }

    void gnss_proc_exeturboedit::_outputControlInfo()
    {
        std::stringstream str2;
        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
        {

            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "***********************************************");
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "+++  Control Parameters from XML file  +++     ");
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "RealtimeMode :  {}                             ", base_type_conv::bl2str(_realtime));
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "LiteMode     :  {}                             ", base_type_conv::bl2str(_liteMode));
            }
        }
        if (!_realtime && !_liteMode)
        {
            // SYS & Bands
            str2.str("");
            for (auto sys : _sys_list)
            {
                GSYS gsys = gnss_sys::str2gsys(sys);
                std::vector<GOBSBAND> v_band = dynamic_cast<set_gnss *>(_set)->band(gsys);

                str2 << sys + "->";
                if (_simulation && gsys == GSYS::LEO)
                    continue;

                for (unsigned int i = 1; i <= _post_all_band.size(); i++)
                {
                    if (_post_all_band[i].find(gsys) == _post_all_band[i].end())
                        continue;

                    str2 << "BAND_" + gobsband2str(_post_all_band[i][gsys].first) +
                                "+BAND_" + gobsband2str(_post_all_band[i][gsys].second) + "  ";
                }
                str2 << "  ";
            }

            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "System&Bands :  {}                             ", str2.str());
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "Kinematic    :  {}                             ", base_type_conv::bl2str(_site_kinematic));
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "Simulation   :  {}                             ", base_type_conv::bl2str(_simulation));
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "Use Ephemeris:  {}                             ", base_type_conv::bl2str(_ephemeris));
            }

            if (_ephemeris)
            {
                str2.str("  ");
                std::vector<std::string> inp = dynamic_cast<set_inp *>(_set)->inputs("rinexn");
                for (auto iter : inp)
                    str2 << iter << "  ";

                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "Ephemeris    :  {}                             ", str2.str());

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _min_elev;

                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "ElevationMask:  {}  degrees                    ", str2.str());
            }
            // PC

            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Check PC     :  " + base_type_conv::bl2str(_check_pc));
            if (_check_pc)
            {

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _pc_limit;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "PCLimitation :  " + str2.str() + " meters");
            }
            // LX-CX

            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Check LX-CX  :  " + base_type_conv::bl2str(_check_sf));
            if (_check_sf)
            {

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _sf_limit;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "LXLimitation :  " + str2.str() + " meters");
            }
        }
        // MW

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "TurboEdit(MW):  " + base_type_conv::bl2str(_check_mw));
        if (_check_mw)
        {

            str2.str("");
            str2 << std::fixed << left << setw(7) << setprecision(2) << _mw_limit;
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "MWLimitation :  " + str2.str() + " cycles");
        }
        // GF
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "TurboEdit(GF):  " + base_type_conv::bl2str(_check_gf));
        if (_check_gf)
        {
            if (!_realtime && !_liteMode)
            {

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _gf_limit;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "GFLimitation :  " + str2.str() + " cycles");

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _gf_rms_limit;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "GFRmslimit   :  " + str2.str() + " cycles");
            }
            else
            {

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _gf_limit;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "GFLimitation :  " + str2.str() + " meters");
            }
        }

        // check gap
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "Check Gap    :  " + base_type_conv::bl2str(_check_gap));
        if (_check_gap)
        {

            str2.str("");
            str2 << std::fixed << left << setw(7) << _gap_limit;
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "GapLimit     :  " + str2.str() + " epochs ");
        }

        // check short
        if (!_realtime && !_liteMode)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Check Short  :  " + base_type_conv::bl2str(_check_short));
            if (_check_short)
            {

                str2.str("");
                str2 << std::fixed << left << setw(7) << _short_limit;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "ShortLimit   :  " + str2.str() + " epochs ");
            }
            // check statistics
            if (_check_statistics)
            {

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _max_mean_namb;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "MaxMeanAmbs  :  " + str2.str());

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _min_percent;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "MinPercent>4 :  " + str2.str() + " %");

                str2.str("");
                str2 << std::fixed << left << setw(7) << setprecision(2) << _min_mean_nprn;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "MinMeanPrns  :  " + str2.str());
            }
        }
    }

    void gnss_proc_exeturboedit::_initCommonProcess()
    {
        // Global Settings
        _beg_time = dynamic_cast<set_gen *>(_set)->beg();
        _end_time = dynamic_cast<set_gen *>(_set)->end();
        _sys_list = dynamic_cast<set_gen *>(_set)->sys();  // processing systems
        _rec_list = dynamic_cast<set_gen *>(_set)->recs(); // processing receivers
        for (auto it : _sys_list)                            // processing satellites
        {
            GSYS gsys = gnss_sys::str2gsys(it);
            set<std::string> sat_temp = dynamic_cast<set_gnss *>(_set)->sat(gsys);
            _sat_list.insert(sat_temp.begin(), sat_temp.end()); // insert satellites
        }
        // exclude satellites mainly For simulation
        // real data can control processed sats by module <gps>/<bds>/<gal>/..., also useful for real data
        set<std::string> sat_rm = dynamic_cast<set_gen *>(_set)->sat_rm();

        for (auto iter : sat_rm)
        {
            _sat_list.erase(iter);
        }

        // TurboEdit Settings
        _realtime = dynamic_cast<set_gproc *>(_set)->realtime();
        _min_elev = dynamic_cast<set_gproc *>(_set)->minimum_elev();
        _liteMode = dynamic_cast<set_turboedit *>(_set)->liteMode();
        _check_mw = dynamic_cast<set_turboedit *>(_set)->checkMW(_mw_limit);                // cycle for post/real_time
        _check_gf = dynamic_cast<set_turboedit *>(_set)->checkGF(_gf_limit, _gf_rms_limit); // meter for real_time/cycle for post
        _check_gap = dynamic_cast<set_turboedit *>(_set)->checkGap(_gap_limit);
        _ephemeris = dynamic_cast<set_turboedit *>(_set)->isEphemeris(); // 适用于年份较老的动态数据，提供GLONASS_freq_ID
        _site_kinematic = dynamic_cast<set_gproc *>(_set)->pos_kin();
        _observ = dynamic_cast<set_gproc*>(_set)->obs_combin();

        if (!_allnav)
            _ephemeris = false;

        if (!_check_gap)
			_gap_limit = 999999;

		// SPPFLT initialization
		for (auto it : _rec_list)
		{
			if (_sppflt[it])
			{
				delete _sppflt[it];
				_sppflt[it] = nullptr;
			}

			_sppflt[it] = new gnss_proc_sppflt(it, _set, _spdlog);
			_sppflt[it]->setDAT(_allobs, _allnav);
			_sppflt[it]->setOBJ(_allobj);
		}

    }

    void gnss_proc_exeturboedit::_initRealTimeProcess()
    {
        // 平滑窗口
        _smooth_window = dynamic_cast<set_turboedit *>(_set)->smoothWindows();
        // GF设置 参考 张小红《构建阈值模型改善TurboEdit实时周跳探测》
        // 对于每一组数据，GF组合历元差与高度角成反比，主要原因是卫星高度角低时，多路径效应和观测噪声的影响较大，因此，需要在上述初始阈
        // 值中引入一个与卫星高度角有关的加权因子，目前在前面乘以一个比较大的因子来替代
        // MW 采用自适应
        _check_sf = dynamic_cast<set_turboedit*>(_set)->checkSingleFreq(_sf_limit);

        double sample = dynamic_cast<set_gen *>(_set)->sampling();
        if (sample <= 1)
        {
            if (double_eq(_gf_limit, 0.0))
                _gf_limit = 0.05; ///< meters

            if (double_eq(_mw_limit, 0.0))
                _mw_limit = 3.0; ///< cycles
        }
        else if (sample <= 20)
        {

            if (double_eq(_gf_limit, 0.0))
                _gf_limit = 0.1 / 20.0 * sample + 0.05; ///< meters

            if (double_eq(_mw_limit, 0.0))
                _mw_limit = 2.5 / 20.0 * sample + 2.5; ///< cycles
        }
        else if (sample <= 60)
        {
            if (double_eq(_gf_limit, 0.0))
                _gf_limit = 0.15; ///< meters

            if (double_eq(_mw_limit, 0.0))
                _mw_limit = 4.0; ///< cycles
        }
        else if (sample <= 100)
        {
            if (double_eq(_gf_limit, 0.0))
                _gf_limit = 0.25; ///< meters

            if (double_eq(_mw_limit, 0.0))
                _mw_limit = 5.0; ///< cycles
        }
        else
        {
            if (double_eq(_gf_limit, 0.0))
                _gf_limit = 0.3; ///< meters

            if (double_eq(_mw_limit, 0.0))
                _mw_limit = 6.0; ///< cycles
        }

        _frequency = dynamic_cast<set_gproc *>(_set)->frequency();

        if (_ephemeris)
        {
            _glonass_freq_id = _allnav->glo_freq_num();
        }

        // Get site coordinates from XML/SNX files
        for (const auto &it : _rec_list)
        {
            // XML
            _rec_crds[it] = dynamic_cast<set_rec *>(_set)->get_crd_xyz(it);
            // SNX
            if (_rec_crds[it].isZero() && _allobj)
            {
                std::shared_ptr<gnss_data_obj> _grec = _allobj->obj(it);
                Triple crd(0.0, 0.0, 0.0);
                Triple std(0.0, 0.0, 0.0);
                if (_grec)
                {
                    _grec->get_recent_crd(_beg_time, 0.1, crd, std);
                    _rec_crds[it] = crd;
                }
            }
        }

        if (1) // lvhb shut down the outputting file in 202011
        {
            if (_debug_turbo)
            {
                if (_debug_turbo->is_open())
                    _debug_turbo->close();

                delete _debug_turbo;
                _debug_turbo = nullptr;
            }
        }
        else
        {
            _debug_turbo = new base_iof;
            _debug_turbo->tsys(base_time::GPS);
            _debug_turbo->mask("debug_turbo.spdlog");
            _debug_turbo->append(dynamic_cast<set_out *>(_set)->append());
        }

        // set output cycle slip file
        std::string tmp;
        tmp = dynamic_cast<set_turboedit*>(_set)->outputs("cycleslip");
        if (!tmp.empty())
        {
            std::string site_tmp = _crt_rec;
            transform(site_tmp.begin(), site_tmp.end(), site_tmp.begin(), ::tolower);
            base_type_conv::substitute(tmp, "$(rec)", site_tmp, false);
            _cycleslipfile = new base_iof;
            _cycleslipfile->tsys(base_time::GPS);
            _cycleslipfile->mask(tmp);
            _cycleslipfile->append(dynamic_cast<set_out*>(_set)->append());
        }
        //_receiverType = dynamic_cast<set_gproc*>(_set)->get_receiverType();
    }

    bool gnss_proc_exeturboedit::_realtimeProcess(const base_time &begT, const base_time &endT)
    {

        _crt_time = begT;
        std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData;
        while (_crt_time <= endT)
        {
            // 如果设为false，需要在每个历元周跳探测前传入测站坐标
            if (_ephemeris && (_site_kinematic || _rec_crds[_crt_rec].isZero()))
            {
                if (_siteSPP(_crt_rec, _crt_time))
                    _rec_crds[_crt_rec] = _sppflt[_crt_rec]->getCrd(_crt_time);
            }

            if (_inputEpoData.size() == 0)
                epoData = _allobs->obs_pt(_crt_rec, _crt_time);
            else
                epoData = _inputEpoData;

            if (epoData.size() == 0)
            {
                _crt_time = _crt_time + _interval;
                continue;
            }

            if (_cycleslipfile)
            {
                std::ostringstream cycleslipInfo; cycleslipInfo.str("");
                cycleslipInfo << "> " << setw(6) << _crt_time.sod() << std::endl;
                _cycleslipfile->write(cycleslipInfo.str().c_str(), cycleslipInfo.str().size());
                _cycleslipfile->flush();
            }

            for (auto &oneObs : epoData)
            {
                std::string sat = oneObs->sat();
                GSYS gsys = oneObs->gsys();
                std::string site = oneObs->site();

                if (site != _crt_rec)
                    continue; // For Npp, maybe no use

                // set glonass Frequency ID
                if (sat.substr(0, 1) == "R" && oneObs->channel() >= DEF_CHANNEL)
                {
                    if (_glonass_freq_id.end() == _glonass_freq_id.find(sat)) continue;
                    oneObs->channel(_glonass_freq_id.at(sat));
                }

                if (_crt_time > _beg_time)
                {
                    if (_epoDataPre[_crt_rec].find(sat) != _epoDataPre[_crt_rec].end())
                    {
                        // Range / Doppler outliers Detection
                        _outliers_proc.flagRangeOutliers(_epoDataPre[_crt_rec][sat], oneObs, _interval);
                        set<GOBSBAND> bands = oneObs->band_avail(true);
                        if (bands.size() == 1)
                        { 
                            _checkSlipWithDoppler(_epoDataPre[_crt_rec][sat], oneObs);
                        }
                    }
                }
                if (_frequency >= 1)
                {
                    set<GOBSBAND> bands = oneObs->band_avail(true);
                    std::vector<GOBSBAND> sorted_bands = sort_band(gsys, bands);
                    for (unsigned int i = 1; i < sorted_bands.size(); ++i)
                    {
                        if (sorted_bands[0] == sorted_bands[i])
                            continue;
                        _realTimeMWGF_new(oneObs, sorted_bands[0], sorted_bands[i]);
                    }

                    if ((_frequency == 1 || _observ == OBSCOMBIN::RAW_MIX))
                        _checkSlipWithHighDiff(oneObs);
                }

                _epoDataPre[_crt_rec][sat] = oneObs;
            }
            // Next
            _crt_time = _crt_time + _interval;
        }
        return true;
    }

    void gnss_proc_exeturboedit::_realTimeMWGF(std::shared_ptr<gnss_data_obs_manager> oneObs, GOBSBAND b1, GOBSBAND b2)
    {
        std::string sat = oneObs->sat();
        //// get the set<GOBS>
        //set<GOBS> obs_vec1 = oneObs->obs_phase(b1);
        //set<GOBS> obs_vec2 = oneObs->obs_phase(b2);
        // add select Phase
        // 避免 L1C+L2L的周跳对L1C+L2W造成影响（即 L1C L2L判定为周跳，但L2W无周跳， 实际上选用的为L1C L2W）
        GOBS obs_L1 = oneObs->select_phase(b1, true);
        GOBS obs_L2 = oneObs->select_phase(b2, true);
        set<GOBS> obs_vec1, obs_vec2;

        if (obs_L1 != GOBS::X)
            obs_vec1.insert(obs_L1);

        if (obs_L2 != GOBS::X)
            obs_vec2.insert(obs_L2);

        gnss_data_obs gobs_P1 = gnss_data_obs(oneObs->select_range(b1, true));
        gnss_data_obs gobs_P2 = gnss_data_obs(oneObs->select_range(b2, true));

        double distance, sat_clk;
        double sat_elev = _min_elev;
        double GF_factor = 1.0;
        if (this->_getSatInfo(sat, distance, sat_elev, sat_clk))
        {
            oneObs->addele(sat_elev * D2R);
            if (sat_elev < _min_elev)
                sat_elev = _min_elev;

            if (sat_elev < 30)
                GF_factor = sqrt(sin(D2R * 30) / sin(D2R * sat_elev));
        }
        else
        {
            GF_factor = 5.0;
        }

        std::ostringstream os;
        os.str("");

        for (auto obs1 : obs_vec1)
        {
            gnss_data_obs gobs_L1 = gnss_data_obs(obs1);

            for (auto obs2 : obs_vec2)
            {
                gnss_data_obs gobs_L2 = gnss_data_obs(obs2);

                double crt_MW_val = oneObs->MW_cycle(gobs_L1, gobs_L2, gobs_P1, gobs_P2); // MW combination [cycles]
                double crt_GF_val = oneObs->L4(gobs_L1, gobs_L2);                         // Geometry-free combination [meters]

                os << std::fixed << setw(10) << " MW[c] " << oneObs->sat() << "  " << setw(6) << _crt_time.sod()
                   << "  " << gobs2str(obs1) << "  " << gobs2str(obs2)
                   << "  " << gobs2str(gobs_P1.gobs()) << "  " << gobs2str(gobs_P2.gobs())
                   << setw(15) << setprecision(4) << crt_MW_val << std::endl;

                os << std::fixed << setw(10) << " GF[m] " << oneObs->sat() << "  " << setw(6) << _crt_time.sod()
                   << "  " << gobs2str(obs1) << "  " << gobs2str(obs2)
                   << setw(15) << setprecision(4) << crt_GF_val << std::endl;

                // 如果某卫星某参考GOBS/第二个GOBS不存在（开始），重新再计算
                if (_rt_data[_crt_rec][sat][obs1].epo_num.find(obs2) == _rt_data[_crt_rec][sat][obs1].epo_num.end() || _rt_data[_crt_rec][sat][obs1].mwslip[obs2] || _rt_data[_crt_rec][sat][obs1].gfslip[obs2] || oneObs->getlli(obs_L1) >= 1 || oneObs->getlli(obs_L2) >= 1)
                {
                    _rt_data[_crt_rec][sat][obs1].epo_num[obs2] = 1;
                    _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;
                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2] = std::make_pair(crt_MW_val, 0.0);

                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].clear();
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(0.0);
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                    continue;
                }

                // 数据中断与前后历元的比较，存在BUG
                // 如  epoch->(1,2,3,4,20,21,22,...),MW->(-3.22,-3.24,-3.26,-3.25,-33.52,-33.42,-33.47,...)
                double dt = _crt_time.diff(_rt_data[_crt_rec][sat][obs1].last_time[obs2]);
                double dif_MW = crt_MW_val - _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].back();
                // 数据中断且MW前后时刻变化大视为模糊度重置
                if ((dt > _interval && abs(dif_MW) > 8) || (_check_gap && dt > _gap_limit * _interval))
                {
                    _rt_data[_crt_rec][sat][obs1].epo_num[obs2] = 1;
                    _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;
                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2] = std::make_pair(crt_MW_val, 0.0);

                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].clear();
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(0.0);
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    std::ostringstream msg;
                    msg << std::fixed << setw(10) << " Data Gap or Max_diff_MW : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                        << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << dif_MW << setw(16) << setprecision(3) << dt
                        << setw(16) << setprecision(3) << _gap_limit * _interval;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());

                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                    continue;
                }

                _rt_data[_crt_rec][sat][obs1].epo_num[obs2] += 1;
                _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;

                int epo_num = _rt_data[_crt_rec][sat][obs1].epo_num[obs2];

                if (_check_gf)
                {
                    double pre_GF_val = _rt_data[_crt_rec][sat][obs1].GF[obs2];

                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;

                    if (abs(crt_GF_val - pre_GF_val) > _gf_limit * GF_factor)
                    {
                        _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = true;
                        std::ostringstream msg;
                        msg << std::fixed << setw(10) << " GF cycle slip : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                            << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << pre_GF_val << setw(16) << setprecision(3) << crt_GF_val
                            << setw(16) << setprecision(3) << (crt_GF_val - pre_GF_val) << setw(16) << setprecision(3) << _gf_limit * GF_factor;
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                    }
                }

                if (_check_mw)
                {
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    double sigma;
                    double pre_MW_val = _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first;
                    double pre_MW_std = _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second;
                    sigma = sqrt(pre_MW_std);

                    if (epo_num <= _smooth_window)
                    {
                        double smooth_MW_val = pre_MW_val * (epo_num - 1) / epo_num + crt_MW_val / epo_num;
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                        double smooth_MW_std = pre_MW_std * (epo_num - 1) / epo_num + pow(crt_MW_val - pre_MW_val, 2) / epo_num;
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;
                    }
                    else
                    {
                        double smooth_MW_val = 0.0;
                        for (int i = epo_num - _smooth_window; i <= epo_num - 1; i++)
                        {
                            smooth_MW_val += _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2][i] / _smooth_window;
                        }
                        double smooth_MW_std = 0.0;
                        for (int i = epo_num - _smooth_window + 1; i <= epo_num; i++)
                        {
                            smooth_MW_std += pow(_rt_data[_crt_rec][sat][obs1].origin_MWs[obs2][i] - smooth_MW_val, 2) / _smooth_window;
                        }
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;
                    }

                    // 自适应阈值
                    double k;
                    if (sigma >= 0.0 && sigma <= 0.4)
                        k = 8.0 - 5.0 * sigma;

                    else if (sigma > 0.4 && sigma <= 0.6)
                        k = 6.0;

                    else if (sigma > 0.6 && sigma <= 1.0)
                        k = 6.0 - 2.5 * (sigma - 0.6);

                    else
                        k = 5.0;

                    // outliers
                    if (oneObs->getobs(gobs_P1.gobs()) < 1.9e7 || oneObs->getobs(gobs_P2.gobs()) < 1.9e7)
                    {
                        k = 1.e9;
                    }
                    else if (oneObs->getoutliers(gobs_P1.gobs()) > 0 || oneObs->getoutliers(gobs_P2.gobs()) > 0)
                    {
                        k *= 2;
                    }

                    _mw_limit = k * sigma;

                    if (abs(crt_MW_val - pre_MW_val) > _mw_limit && !double_eq(_mw_limit, 0.0))
                    {
                        _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = true;
                        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
                        {
                            std::ostringstream msg;
                            msg << std::fixed << setw(10) << " MW cycle slip : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                                << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << pre_MW_val << setw(16) << setprecision(3) << crt_MW_val
                                << setw(16) << setprecision(3) << (crt_MW_val - pre_MW_val) << setw(16) << setprecision(3) << _mw_limit;
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                        }
                    }
                }

                // 任一不合格就要设为周跳
                if (_rt_data[_crt_rec][sat][obs1].gfslip[obs2] || _rt_data[_crt_rec][sat][obs1].mwslip[obs2])
                {
                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                }
            }
        }

        // Print flt results
        if (_debug_turbo)
        {

            _debug_turbo->write(os.str().c_str(), os.str().size());
            _debug_turbo->flush();
        }
    }

    void gnss_proc_exeturboedit::_realTimeMWGF_test(std::shared_ptr<gnss_data_obs_manager> oneObs, GOBSBAND b1, GOBSBAND b2)
    {
        std::string sat = oneObs->sat();
        // get the set<GOBS>
        set<GOBS> obs_vec1 = oneObs->obs_phase(b1);
        set<GOBS> obs_vec2 = oneObs->obs_phase(b2);
        gnss_data_obs gobs_P1 = gnss_data_obs(oneObs->select_range(b1, true));
        gnss_data_obs gobs_P2 = gnss_data_obs(oneObs->select_range(b2, true));
        //double wlength1 = oneObs->wavelength(b1);//L1波长
        // double fac1 = oneObs->frequency(b1);
        // double fac2 = oneObs->frequency(b2);
        // double lamw = CLIGHT / (fac1 - fac2); //MW wavelength

        //xiongyun test
        double distance, elev, sat_clk;
        double fact = 1;
        if (_getSatInfo(sat, distance, elev, sat_clk))
        {
            //if (_interval >= 29.5)
            //{
            //    if (_interval <= 2.0)        fact = 1.0;
            //    else if (_interval <= 4.0)    fact = 1.25;
            //    else if (_interval <= 6.0)    fact = 1.5;
            //    else                    fact = 2.0;
            //}

            if (elev < _min_elev)
            {
                elev = _min_elev;
            }
        }

        for (auto obs1 : obs_vec1)
        {
            gnss_data_obs gobs_L1 = gnss_data_obs(obs1);

            for (auto obs2 : obs_vec2)
            {
                gnss_data_obs gobs_L2 = gnss_data_obs(obs2);

                //double crt_MW_val = oneObs->MW_cycle(gobs_L1, gobs_L2, gobs_P1, gobs_P2);     // MW combination [cycles]
                //crt_MW_val *= lamw;
                double crt_MW_val = -oneObs->MW(gobs_L1, gobs_L2); // MW combination [meters]
                double crt_GF_val = oneObs->L4(gobs_L1, gobs_L2);  // Geometry-free combination [meters]

                // 如果某卫星某参考GOBS以及第二个GOBS不存在（开始）+ 数据中断，重新再计算
                if (_rt_data[_crt_rec][sat][obs1].epo_num.find(obs2) == _rt_data[_crt_rec][sat][obs1].epo_num.end() || 
                   (_check_gap && _crt_time.diff(_rt_data[_crt_rec][sat][obs1].last_time[obs2]) > _gap_limit * _interval) || 
                   _rt_data[_crt_rec][sat][obs1].mwslip[obs2] ||
                   _rt_data[_crt_rec][sat][obs1].gfslip[obs2])
                {
                    _rt_data[_crt_rec][sat][obs1].epo_num[obs2] = 1;
                    _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;
                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2] = std::make_pair(crt_MW_val, 0.0);

                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].clear();
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                    continue;
                }

                double dt = _crt_time.diff(_rt_data[_crt_rec][sat][obs1].last_time[obs2]);
                _rt_data[_crt_rec][sat][obs1].epo_num[obs2] += 1;
                _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;

                int epo_num = _rt_data[_crt_rec][sat][obs1].epo_num[obs2];

                if (_check_gf)
                {
                    double pre_GF_val = _rt_data[_crt_rec][sat][obs1].GF[obs2];
                    //double pre_GF_std = _rt_data[_crt_site][sat][obs1].smooth_GF[obs2].second;

                    //double smooth_GF_val = pre_GF_val * (epo_num - 1) / epo_num + crt_GF_val / epo_num;
                    //_rt_data[_crt_site][sat][obs1].smooth_GF[obs2].first = smooth_GF_val;
                    //double smooth_GF_std = pre_GF_std * (epo_num - 1) / epo_num + pow(crt_GF_val - pre_GF_val, 2) / epo_num;
                    //_rt_data[_crt_site][sat][obs1].smooth_GF[obs2].second = smooth_GF_std;

                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;

                    //xiongyun test
                    //if (elev < 15)
                    //{
                    //    _gf_limit = -1 * elev * 1.0 / 15.0 + 2;
                    //}
                    //else
                    //{
                    //    _gf_limit = 1;
                    //}
                    //int gapepo = round(dt / _interval);
                    //if (gapepo >= 3 && dt >= 90.0)
                    //{
                    //    _gf_limit = min(_gf_limit * gapepo, 0.35);
                    //}

                    //rtppp

                    if (dt <= 1.0)
                        _gf_limit = 0.05;

                    else if (dt <= 20.0)
                        _gf_limit = (0.05 * 2) / (20.0 - 0.0) * dt + 0.05;

                    else if (dt <= 60.0)
                        _gf_limit = 0.05 * 3;
                    else if (dt <= 100.0)
                        _gf_limit = 0.05 * 5;

                    else
                        _gf_limit = 0.05 * 7;

                    if (elev < 15)
                    {
                        _gf_limit = -_gf_limit * 0.1 * elev + 3 * _gf_limit; // unit: meter
                    }

                    //if (abs(crt_GF_val - pre_GF_val) > _gf_limit * 5)
                    if (abs(crt_GF_val - pre_GF_val) > _gf_limit)
                    {
                        _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = true;
                        std::ostringstream msg;
                        msg << std::fixed << setw(10) << " GF cycle slip : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                            << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << pre_GF_val << setw(16) << setprecision(3) << crt_GF_val
                            << setw(16) << setprecision(3) << (crt_GF_val - pre_GF_val) << setw(16) << setprecision(3) << _gf_limit * 5;
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                    }
                }

                if (_check_mw)
                {
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    double sigma;
                    double pre_MW_val = _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first;
                    double pre_MW_std = _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second;
                    sigma = sqrt(pre_MW_std);

                    //if (epo_num < _smooth_window)
                    //{
                    //    double smooth_MW_val = pre_MW_val * (epo_num - 1) / epo_num + crt_MW_val / epo_num;
                    //    _rt_data[_crt_site][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                    //    double smooth_MW_std = pre_MW_std * (epo_num - 1) / epo_num + pow(crt_MW_val - pre_MW_val, 2) / epo_num;
                    //    _rt_data[_crt_site][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;
                    //}
                    //else
                    //{
                    //    double smooth_MW_val = 0.0;
                    //    for (int i = epo_num - _smooth_window; i < epo_num - 1; i++)
                    //    {
                    //        smooth_MW_val += _rt_data[_crt_site][sat][obs1].origin_MWs[obs2][i] / _smooth_window;
                    //    }
                    //    double smooth_MW_std = 0.0;
                    //    for (int i = epo_num - _smooth_window; i < epo_num; i++)
                    //    {
                    //        smooth_MW_std += pow(_rt_data[_crt_site][sat][obs1].origin_MWs[obs2][i] - smooth_MW_val, 2) / _smooth_window;
                    //    }
                    //    _rt_data[_crt_site][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                    //    _rt_data[_crt_site][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;

                    //}

                    // 自适应阈值
                    double k = 4;
                    //if (sigma >= 0.0 && sigma <= 0.4) k = 8.0 - 5.0 * sigma;
                    //else if (sigma > 0.4 && sigma <= 0.6) k = 6.0;
                    //else if (sigma > 0.6 && sigma <= 1.0) k = 6.0 - 2.5 * (sigma - 0.6);
                    //else                                  k = 5.0;

                    // outliers
                    if (oneObs->getobs(gobs_P1.gobs()) < 1.9e7 || oneObs->getobs(gobs_P2.gobs()) < 1.9e7)
                    {
                        k = 1.e9;
                    }
                    else if (oneObs->getoutliers(gobs_P1.gobs()) > 0 || oneObs->getoutliers(gobs_P2.gobs()) > 0)
                    {
                        k *= 2;
                    }
                    //// outliers
                    //if (oneObs->getoutliers(gobs_P1.gobs()) > 0 || oneObs->getoutliers(gobs_P2.gobs()) > 0)
                    //{
                    //    k *= 2;
                    //}
                    //_mw_limit = k * sigma;

                    //全部平滑
                    double smooth_MW_val = pre_MW_val * (epo_num - 1) / epo_num + crt_MW_val / epo_num;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                    double smooth_MW_std = pre_MW_std * (epo_num - 1) / epo_num + pow(crt_MW_val - pre_MW_val, 2) / epo_num;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;
                    //xiongyun test
                    //if (elev >= 20) _mw_limit = 1;
                    //else           _mw_limit = -1 * 0.1 * elev + 3;

                    //rtppp
                    //_mw_limit = 4 * sigma; 0429
                    _mw_limit = k * sigma;
                    //xiongyun test
                    //if (abs(crt_MW_val - pre_MW_val) > _mw_limit && !double_eq(_mw_limit, 0.0))
                    if (abs(crt_MW_val - pre_MW_val) > _mw_limit * fact && !double_eq(_mw_limit, 0.0))
                    {
                        _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = true;
                        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
                        {
                            std::ostringstream msg;
                            msg << std::fixed << setw(10) << " MW cycle slip : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                                << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << pre_MW_val << setw(16) << setprecision(3) << crt_MW_val
                                << setw(16) << setprecision(3) << (crt_MW_val - pre_MW_val) << setw(16) << setprecision(3) << _mw_limit;
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                        }
                    }
                }

                // 任一不合格就要设为周跳
                if (_rt_data[_crt_rec][sat][obs1].gfslip[obs2] || _rt_data[_crt_rec][sat][obs1].mwslip[obs2])
                {
                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                }
            }
        }
    }

    void gnss_proc_exeturboedit::_realTimeMWGF_new(std::shared_ptr<gnss_data_obs_manager> oneObs, GOBSBAND b1, GOBSBAND b2)
    {
        std::string sat = oneObs->sat();
        //// get the set<GOBS>
        //set<GOBS> obs_vec1 = oneObs->obs_phase(b1);
        //set<GOBS> obs_vec2 = oneObs->obs_phase(b2);
        // add select Phase
        // aviod the cycle slip on L1C+L2L influence L1C+L2W 
        // (if judge cycle slip on L1C+L2L, but there isn't cycle slip on L2W, and we actually use L1C+L2W)
        GOBS obs_L1 = oneObs->select_phase(b1, true);
        GOBS obs_L2 = oneObs->select_phase(b2, true);
        set<GOBS> obs_vec1, obs_vec2;

        if (obs_L1 != GOBS::X)
            obs_vec1.insert(obs_L1);

        if (obs_L2 != GOBS::X)
            obs_vec2.insert(obs_L2);

        gnss_data_obs gobs_P1 = gnss_data_obs(oneObs->select_range(b1, true));
        gnss_data_obs gobs_P2 = gnss_data_obs(oneObs->select_range(b2, true));

        double distance, sat_clk;
        double sat_elev = _min_elev;
        double scale_factor = 1;
        double thres_MW = 0, thres_GF = 0;
        if (this->_getSatInfo(sat, distance, sat_elev, sat_clk))
        {
            oneObs->addele(sat_elev * D2R);
            if (sat_elev < _min_elev)
                sat_elev = _min_elev;
        }

        std::ostringstream os; os.str("");
        std::ostringstream cycleslipInfo; cycleslipInfo.str("");

        for (auto obs1 : obs_vec1)
        {
            gnss_data_obs gobs_L1 = gnss_data_obs(obs1);
            
            for (auto obs2 : obs_vec2)
            {
                gnss_data_obs gobs_L2 = gnss_data_obs(obs2);

                double crt_MW_val = oneObs->MW_cycle(gobs_L1, gobs_L2, gobs_P1, gobs_P2); // MW combination [cycles]
                double crt_GF_val = oneObs->L4(gobs_L1, gobs_L2);                         // Geometry-free combination [meters]

                os << std::fixed << setw(10) << " MW[c] " << oneObs->sat() << "  " << setw(6) << _crt_time.sod()
                   << "  " << gobs2str(obs1) << "  " << gobs2str(obs2)
                   << "  " << gobs2str(gobs_P1.gobs()) << "  " << gobs2str(gobs_P2.gobs())
                   << setw(15) << setprecision(4) << crt_MW_val << std::endl;

                os << std::fixed << setw(10) << " GF[m] " << oneObs->sat() << "  " << setw(6) << _crt_time.sod()
                   << "  " << gobs2str(obs1) << "  " << gobs2str(obs2)
                   << setw(15) << setprecision(4) << crt_GF_val << std::endl;

                if (_cycleslipfile)
                {
                    cycleslipInfo << std::fixed << oneObs->sat() << setw(15) << setprecision(4) << crt_MW_val
                        << setw(15) << setprecision(4) << crt_GF_val << "        # MW[c]    GF[m]("
                        << gobs2str(obs1) << " " << gobs2str(obs2) << " "
                        << gobs2str(gobs_P1.gobs()) << " " << gobs2str(gobs_P2.gobs()) << ")" << std::endl;
                }
                

                // If the satellite, or the reference GOBS, or the second GOBS isn't exist, data break, Then recalculate.
                if (_rt_data[_crt_rec][sat][obs1].epo_num.find(obs2) == _rt_data[_crt_rec][sat][obs1].epo_num.end() || _rt_data[_crt_rec][sat][obs1].mwslip[obs2] || _rt_data[_crt_rec][sat][obs1].gfslip[obs2] || oneObs->getlli(obs_L1) >= 1 || oneObs->getlli(obs_L2) >= 1)
                {
                    _rt_data[_crt_rec][sat][obs1].epo_num[obs2] = 1;
                    _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;
                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2] = std::make_pair(crt_MW_val, 0.0);

                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].clear();
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(0.0);
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                    continue;
                }

                // the compare between before and after epoch when there is data break. There is BUG. 
                // eg:  epoch->(1,2,3,4,20,21,22,...),MW->(-3.22,-3.24,-3.26,-3.25,-33.52,-33.42,-33.47,...)
                double dt = _crt_time.diff(_rt_data[_crt_rec][sat][obs1].last_time[obs2]);
                double dif_MW = crt_MW_val - _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].back();
                bool badRange = oneObs->getoutliers(gobs_P1.gobs()) >= 1 || oneObs->getoutliers(gobs_P2.gobs()) >= 1;
                // If data break or MW-value change hugely, regard as ambiguity reset.
                if ((dt > _interval && abs(dif_MW) > 8 && !badRange) || 
                    (_check_gap && dt > _gap_limit * _interval))
                {
                    _rt_data[_crt_rec][sat][obs1].epo_num[obs2] = 1;
                    _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = false;
                    _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;
                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;
                    _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2] = std::make_pair(crt_MW_val, 0.0);
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].clear();
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(0.0);
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    std::ostringstream msg;
                    msg << std::fixed << setw(10) << " Data Gap or Max_diff_MW : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                        << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << dif_MW << setw(16) << setprecision(3) << dt
                        << setw(16) << setprecision(3) << _gap_limit * _interval;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());

                    cycleslipInfo << std::fixed << "# Data-Gap-or-Max_diff_MW: " << setprecision(3) << dif_MW << setw(16) 
                        << setprecision(3) << dt << setw(16) << setprecision(3) << _gap_limit * _interval << std::endl;

                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                    continue;
                }

                int gapNum = dt / _interval - 1;

                if (_check_gf)
                {
                    double pre_GF_val = _rt_data[_crt_rec][sat][obs1].GF[obs2];

                    _rt_data[_crt_rec][sat][obs1].GF[obs2] = crt_GF_val;

                    if (sat_elev < 15)
                    {
                        thres_GF = -_gf_limit * 1.0 / 15.0 * sat_elev + 2 * _gf_limit;
                    }
                    else
                    {
                        thres_GF = _gf_limit;
                    }

                    if (dt >= 90 && gapNum >= 3)
                    {
                        thres_GF = min(thres_GF * gapNum, 0.35);
                    }

                    if (abs(crt_GF_val - pre_GF_val) > thres_GF)
                    {
                        _rt_data[_crt_rec][sat][obs1].gfslip[obs2] = true;
                        std::ostringstream msg;
                        msg << std::fixed << setw(10) << " GF cycle slip : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                            << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << pre_GF_val << setw(16) << setprecision(3) << crt_GF_val
                            << setw(16) << setprecision(3) << (crt_GF_val - pre_GF_val) << setw(16) << setprecision(3) << thres_GF;
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                        cycleslipInfo << std::fixed << "# GF-cycle-slip: " << setw(16) << setprecision(3) << (crt_GF_val - pre_GF_val)
                            << setw(16) << setprecision(3) << thres_GF << std::endl;
                    }
                }

                if (_check_mw && !badRange)
                {
                    _rt_data[_crt_rec][sat][obs1].epo_num[obs2] += 1;
                    _rt_data[_crt_rec][sat][obs1].last_time[obs2] = _crt_time;

                    int epo_num = _rt_data[_crt_rec][sat][obs1].epo_num[obs2];
                    _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2].push_back(crt_MW_val);

                    double pre_MW_val = _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first;
                    double pre_MW_std = _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second;

                    if (epo_num <= _smooth_window)
                    {
                        double smooth_MW_val = pre_MW_val * (epo_num - 1) / epo_num + crt_MW_val / epo_num;
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                        double smooth_MW_std = pre_MW_std * (epo_num - 1) / epo_num + pow(crt_MW_val - pre_MW_val, 2) / epo_num;
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;
                    }
                    else
                    {
                        double smooth_MW_val = 0.0;
                        for (int i = epo_num - _smooth_window; i <= epo_num - 1; i++)
                        {
                            smooth_MW_val += _rt_data[_crt_rec][sat][obs1].origin_MWs[obs2][i] / _smooth_window;
                        }
                        double smooth_MW_std = 0.0;
                        for (int i = epo_num - _smooth_window + 1; i <= epo_num; i++)
                        {
                            smooth_MW_std += pow(_rt_data[_crt_rec][sat][obs1].origin_MWs[obs2][i] - smooth_MW_val, 2) / _smooth_window;
                        }
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].first = smooth_MW_val;
                        _rt_data[_crt_rec][sat][obs1].smooth_MW[obs2].second = smooth_MW_std;
                    }

                    if (sat_elev < 20)
                        thres_MW = -_mw_limit * 0.1 * sat_elev + 3 * _mw_limit;

                    else
                        thres_MW = _mw_limit;

                    if (gapNum <= 2)
                        scale_factor = 1.0;

                    else if (gapNum <= 4)
                        scale_factor = 1.25;
                    else if (gapNum <= 8)
                        scale_factor = 1.5;

                    else
                        scale_factor = 2.0;

                    if (abs(crt_MW_val - pre_MW_val) > thres_MW * scale_factor && !double_eq(thres_MW, 0.0))
                    {
                        _rt_data[_crt_rec][sat][obs1].mwslip[obs2] = true;
                        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
                        {
                            std::ostringstream msg;
                            msg << std::fixed << setw(10) << " MW cycle slip : " << _crt_time.str_hms() << "   " << oneObs->sat() << "   " << gobs2str(obs1)
                                << "   " << gobs2str(obs2) << setw(16) << setprecision(3) << pre_MW_val << setw(16) << setprecision(3) << crt_MW_val
                                << setw(16) << setprecision(3) << (crt_MW_val - pre_MW_val) << setw(16) << setprecision(3) << thres_MW;
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                            cycleslipInfo << std::fixed << "# MW-cycle-slip: " << setw(16) << setprecision(3) << (crt_MW_val - pre_MW_val) 
                                << setw(16) << setprecision(3) << thres_MW << std::endl;
                        }
                    }
                }

                // Set cycle slip, if any one is not qualified.
                if (_rt_data[_crt_rec][sat][obs1].gfslip[obs2] || _rt_data[_crt_rec][sat][obs1].mwslip[obs2])
                {
                    if (_satdata_npp)
                    {
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs1, 1);
                        (*_satdata_npp)[_map_nppdata_idx[_crt_rec][sat]].addlli(obs2, 1);
                    }
                    else
                    {
                        oneObs->addlli(obs1, 1);
                        oneObs->addlli(obs2, 1);
                    }
                }
            }
        }

        // Print turboedit results
        if (_debug_turbo)
        {
            _debug_turbo->write(os.str().c_str(), os.str().size());
            _debug_turbo->flush();
        }
        if (_cycleslipfile)
        {
            _cycleslipfile->write(cycleslipInfo.str().c_str(), cycleslipInfo.str().size());
            _cycleslipfile->flush();
        }
    }

    void gnss_proc_exeturboedit::_checkSlipWithDoppler(std::shared_ptr<gnss_data_obs_manager> ObsPre, std::shared_ptr<gnss_data_obs_manager> Obs)
    {
        if (Obs->band_avail(true).size() != 1)
            return;
        // 受到速度的影响 Todo 修改
        // 参考彭文杰博士论文，后续进行具体值的修改

        double epo_factor = 1.0;
        double diff_time = _crt_time.diff(ObsPre->epoch());
        double nepoch = diff_time / _interval;
        if (nepoch <= 3.0)
            epo_factor = 1.0;

        else if (nepoch <= 6.0)
            epo_factor = 1.5;

        else if (nepoch <= 12.0)
            epo_factor = 2.0;

        else
            epo_factor = 2.5;

        set<GOBS> obs_vec;
        set<GOBSBAND> bands = Obs->band_avail(true);
        for (auto it : bands)
        {
            GOBS obs_L = Obs->select_phase(it, true);

            if (obs_L != GOBS::X)
                obs_vec.insert(obs_L);
        }

        std::ostringstream os;
        os.str("");
        for (auto obs_L : obs_vec)
        {
            // epoch Gap Judge
            if (_check_gap && nepoch > _gap_limit)
            {
                Obs->addlli(obs_L, 1);

                std::ostringstream msg;
                msg << std::fixed << setw(10) << " cycle slip : " << _crt_time.str_hms() << "   " << Obs->sat() << "   " << gobs2str(obs_L)
                    << "   epoch gap !!! ";
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                continue;
            }

            double crt_phase = Obs->getobs(obs_L);    // cycle
            double pre_phase = ObsPre->getobs(obs_L); // cycle

            if (double_eq(pre_phase, NULL_GOBS) || double_eq(crt_phase, NULL_GOBS))
                continue;

            GOBSBAND band = str2gobsband(gobs2str(obs_L));
            GOBS obs_range = Obs->select_range(band, true);
            double crt_range = Obs->getobs(obs_range);    // m
            double pre_range = ObsPre->getobs(obs_range); // m
            if (double_eq(crt_range, NULL_GOBS) || double_eq(pre_range, NULL_GOBS))
                continue;

            double wavelen = Obs->wavelength(band);
            double delta_PL = (crt_range - crt_phase * wavelen) - (pre_range - pre_phase * wavelen);

            os << std::fixed << setw(10) << " PL[m] " << Obs->sat() << "  " << setw(6) << ObsPre->epoch().sod() << setw(6) << _crt_time.sod() << "  " << gobs2str(obs_L) << "  " << gobs2str(obs_range) << setw(15) << setprecision(4) << delta_PL << std::endl;

            if (Obs->getoutliers(obs_range) < 1 && ObsPre->getoutliers(obs_range) < 1 && abs(delta_PL) > 15 * epo_factor)
            {
                Obs->addlli(obs_L, 1);
                std::ostringstream msg;
                msg << std::fixed << setw(10) << " cycle slip [PL]: " << _crt_time.str_hms() << "   " << Obs->sat() << "   " << gobs2str(obs_L) << "  " << gobs2str(obs_range)
                    << setw(16) << setprecision(3) << delta_PL << " meters ";
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());

                continue;
            }

            std::string strGOBS = gobs2str(obs_L);
            strGOBS.replace(0, 1, "D");
            GOBS obs_doppler = str2gobs(strGOBS);
            double crt_doppler = Obs->getobs(obs_doppler);    // cycle / s
            double pre_doppler = ObsPre->getobs(obs_doppler); // cycle / s
            if (double_eq(crt_doppler, NULL_GOBS) || double_eq(pre_doppler, NULL_GOBS))
                continue;

            double delta_LD = (crt_phase - pre_phase) + 0.5 * (crt_doppler + pre_doppler) * diff_time;

            os << std::fixed << setw(10) << " LD[c] " << Obs->sat() << "  " << setw(6) << ObsPre->epoch().sod() << setw(6) << _crt_time.sod() << "  " << gobs2str(obs_L) << "  " << gobs2str(obs_doppler) << setw(15) << setprecision(4) << delta_LD << std::endl;

            // 为了避免极少数情况下多普勒观测值存在粗差并将相位观测值误判为存在周跳,阈值设为5周
            if (Obs->getoutliers(obs_doppler) < 1 && ObsPre->getoutliers(obs_doppler) < 1 && abs(delta_LD) > 5 * epo_factor)
            {
                Obs->addlli(obs_L, 1);
                std::ostringstream msg;
                msg << std::fixed << setw(10) << " cycle slip [LD] : " << _crt_time.str_hms() << "   " << Obs->sat() << "   " << gobs2str(obs_L) << "  " << gobs2str(obs_doppler)
                    << setw(16) << setprecision(3) << delta_LD << " cycle ";
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
            }
        }

        // Print flt results
        if (_debug_turbo)
        {

            _debug_turbo->write(os.str().c_str(), os.str().size());
            _debug_turbo->flush();
        }
    }

    void gnss_proc_exeturboedit::_checkSlipWithHighDiff(std::shared_ptr<gnss_data_obs_manager> Obs)
    {
        if (Obs->band_avail(true).size() != 1)
            return;

        set<GOBS> obs_vec;
        set<GOBSBAND> bands = Obs->band_avail(true);
        for (auto it : bands)
        {
            GOBS obs_L = Obs->select_phase(it, true);

            if (obs_L != GOBS::X)
                obs_vec.insert(obs_L);
        }

        std::ostringstream os;
        os.str("");
        for (auto obs_L : obs_vec)
        {
            double crt_phase = Obs->getobs(obs_L); // cycle
            GOBSBAND band = str2gobsband(gobs2str(obs_L));

            if (_epo_diff_time.find(_crt_rec) == _epo_diff_time.end() ||
                _epo_diff_time[_crt_rec].find(Obs->sat()) == _epo_diff_time[_crt_rec].end() ||
                _epo_diff_time[_crt_rec][Obs->sat()].find(band) == _epo_diff_time[_crt_rec][Obs->sat()].end() ||
                _crt_time.diff(_epo_diff_time[_crt_rec][Obs->sat()][band]) > 1.1 ||
                double_eq(crt_phase, NULL_GOBS) || Obs->getlli(obs_L) >= 1)
            {
                _epo_diff[_crt_rec][Obs->sat()][band].clear();
                _epo_diff[_crt_rec][Obs->sat()][band].push_back(crt_phase);
                _epo_diff_time[_crt_rec][Obs->sat()][band] = _crt_time;
                continue;
            }

            // double delta = 0.0;

            if (_epo_diff[_crt_rec][Obs->sat()][band].size() < 3)
            {
                _epo_diff[_crt_rec][Obs->sat()][band].push_back(crt_phase);
                _epo_diff_time[_crt_rec][Obs->sat()][band] = _crt_time;
            }
            else
            {
                double delta = crt_phase - 3 * _epo_diff[_crt_rec][Obs->sat()][band][2] +
                               3 * _epo_diff[_crt_rec][Obs->sat()][band][1] - _epo_diff[_crt_rec][Obs->sat()][band][0];
                if (abs(delta) > _sf_limit)
                {
                    Obs->addlli(obs_L, 1);
                    _epo_diff[_crt_rec][Obs->sat()][band].clear();
                    _epo_diff[_crt_rec][Obs->sat()][band].push_back(crt_phase);
                    _epo_diff_time[_crt_rec][Obs->sat()][band] = _crt_time;
                    std::ostringstream msg;
                    msg << std::fixed << setw(10) << " cycle slip [3rd-diff]: " << _crt_time.str_hms() << "   " << Obs->sat() << "   " << gobs2str(obs_L) << setw(16) << setprecision(3) << delta << " cycles ";
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, msg.str());
                }
                else
                {
                    _epo_diff[_crt_rec][Obs->sat()][band].erase(_epo_diff[_crt_rec][Obs->sat()][band].begin());
                    _epo_diff[_crt_rec][Obs->sat()][band].push_back(crt_phase);
                    _epo_diff_time[_crt_rec][Obs->sat()][band] = _crt_time;
                }

                os << std::fixed << setw(10) << " LLL[c] " << Obs->sat() << "  " << setw(6) << _crt_time.sod() << "  " << gobs2str(obs_L) << "  " << setw(15) << setprecision(4) << delta << "  " + _crt_rec << std::endl;
            }
        }

        // Print flt results
        if (_debug_turbo)
        {

            _debug_turbo->write(os.str().c_str(), os.str().size());
            _debug_turbo->flush();
        }
    }

    void gnss_proc_exeturboedit::_initPostProcess()
    {
        // TurboEdit Settings for post-processing
        _site_kinematic = dynamic_cast<set_gproc *>(_set)->pos_kin();
        _simulation = dynamic_cast<set_gproc *>(_set)->simulation();
        _amb_output = dynamic_cast<set_turboedit *>(_set)->isAmbOutput();
        _check_sf = dynamic_cast<set_turboedit *>(_set)->checkSingleFreq(_sf_limit);
        _check_pc = dynamic_cast<set_turboedit *>(_set)->checkPC(_pc_limit);
        _check_short = dynamic_cast<set_turboedit *>(_set)->checkShort(_short_limit);
        _check_statistics = dynamic_cast<set_turboedit *>(_set)->checkStatistics(_min_percent, _min_mean_nprn, _max_mean_namb);

        // Single frequency and combination are not compatible
        if (_check_mw || _check_gf)
            _check_sf = false;

        if (!_check_short)
            _short_limit = 0;

        // set GNSS bands
        for (auto it : _sys_list)
        {
            GSYS gsys = gnss_sys::str2gsys(it);
            if (gsys == GSYS::LEO)
                gsys = GSYS::GPS;

            set<GOBSBAND> s_band; // insert bands
            std::vector<GOBSBAND> v_band = dynamic_cast<set_gnss *>(_set)->band(gsys);
            for (auto it_band : v_band)
                s_band.insert(it_band);

            //v_band = sort_band(gsys, s_band);

            _freq_index[gsys] = dynamic_cast<set_gnss *>(_set)->freq_index(gsys);

            if (_check_sf)
            {
                _post_all_band[1][gsys] = std::make_pair(v_band[0], v_band[0]);
            }
            else
            {
                for (unsigned int i = 1; i < v_band.size(); ++i)
                {
                    _post_all_band[i][gsys] = std::make_pair(v_band[0], v_band[i]);
                }
            }
        }

        // Get site coordinates from XML/rinex O files
        set<std::string> rec_rm;
        for (const auto &it : _rec_list)
        {
            if (_site_kinematic)
            {

                _rec_crds[it] = Triple(0, 0, 0);
                continue;
            }
            // XML
            _rec_crds[it] = dynamic_cast<set_rec *>(_set)->get_crd_xyz(it);
            // SNX
            if (_rec_crds[it].isZero() && _allobj)
            {
                std::shared_ptr<gnss_data_obj> _grec = _allobj->obj(it);
                Triple crd(0.0, 0.0, 0.0);
                Triple std(0.0, 0.0, 0.0);
                if (_grec)
                {
                    _grec->get_recent_crd(_beg_time, 0.1, crd, std);
                    _rec_crds[it] = crd;
                }
            }
            // Rinex O
            if (_rec_crds[it].isZero() && _allobs)
            {
                _rec_crds[it] = _allobs->getsitecrd(it);
            }
            // delete it
            if (_rec_crds[it].isZero())
            {
                rec_rm.insert(it);
            }
        }

        for (auto it : rec_rm)
        {
            _rec_list.erase(it);
            if (_sppflt[it])
            {
                delete _sppflt[it];
                _sppflt[it] = nullptr;
            } // SPPFLT delete
        }

        // Renew satlist according brdm files
        if (_ephemeris)
        {
            // clean invalid/duplicate NAV messages
            _allnav->clean_invalid();
            _allnav->clean_duplicit();
            _glonass_freq_id = _allnav->glo_freq_num();
            set<std::string> sat_xml = _sat_list;
            _sat_list.clear();
            set<std::string> sat_temp = _allnav->satellites();
            set_intersection(sat_xml.begin(), sat_xml.end(), sat_temp.begin(), sat_temp.end(), inserter(_sat_list, _sat_list.begin()));
        }

        // Set scaling factor for large cycle checks
        _scaling_factor["LX"] = 1.0; // single frequency
        _scaling_factor["MW"] = 5.0;
        if (_simulation)
        {
            _ephemeris = false; // if simulation, noy use ephemeris
            _scaling_factor["GF"] = 20.0;
        }
        else
        {
            _scaling_factor["GF"] = 5.0;
        }

        // init ambflag

        if (_allambflag)
        {
            delete _allambflag;
            _allambflag = nullptr;
        }

        _allambflag = new gnss_all_ambflag(_spdlog, base_data::AMBFLAG);
    }

    bool gnss_proc_exeturboedit::_postProcess()
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _crt_rec + " --- Start TURBOEDIT --- ");

        // Set spdlog suffix
        std::string log_suffix = ".log";
        std::string cerr_out = " Process Band : ";
        for (auto it : _sys_list)
        {
            GSYS gsys = gnss_sys::str2gsys(it);
            if (_post_crt_band.find(gsys) == _post_crt_band.end())
                continue;
            if (_simulation && gsys == GSYS::LEO)
                continue;
            std::string str_freq1 = gfreqseq2str(_freq_index[gsys][_post_crt_band[gsys].first]);
            std::string str_freq2 = gfreqseq2str(_freq_index[gsys][_post_crt_band[gsys].second]);
            std::string str_freq = "L" + str_freq1 + str_freq2;
            if (str_freq1 > "2" || str_freq2 > "2")
                log_suffix = ".log" + str_freq1 + str_freq2;

            cerr_out += " " + it + "->BAND_" + gobsband2str(_post_crt_band[gsys].first) + " ";

            if (!_check_sf)
                cerr_out += "+ BAND_" + gobsband2str(_post_crt_band[gsys].second) + " ";
        }

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, cerr_out);

        // Init turboEdit Information for each site
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, " --- Init Turboedit Information --- ");
        _initPostTurboInfo();

        //  Basic check
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, " --- Basic Check --- ");
        _postBasicCheck();

        // Remove short piece and flag gap
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, " --- Check Gap and Remove Short --- ");
        SPDLOG_LOGGER_INFO(_spdlog, " --- First remove short, short_limit = " + base_type_conv::dbl2str(_short_limit * _interval / 2.0) + "s --- ");

        for (auto sat : _sat_list)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "## remove short for satellite, " + sat);
            while (true)
            {
                if (!_postCheckGapShort(_short_limit * _interval / 2.0, sat))
                    break;
            }
        }

        // check_lwlg check WN and LG for all satellites
        if (!_simulation && (_check_gf || _check_mw))
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, " --- Turboedit Check --- ");
            SPDLOG_LOGGER_INFO(_spdlog, " --- check MW or GF for all satellites --- ");
            for (auto sat : _sat_list)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "## turboedit for satellite " + sat);
                _postCheckMWGF(sat);
            }
            // remove short piece and flag gap after turboedit check
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, " --- Check Gap and Remove Short --- ");
            SPDLOG_LOGGER_INFO(_spdlog, " --- Second remove short, short_limit = " + base_type_conv::dbl2str(_short_limit * _interval / 2.0) + "s --- ");
            for (auto sat : _sat_list)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "## remove short for satellite, " + sat);
                while (true)
                {
                    if (!_postCheckGapShort(_short_limit * _interval / 2.0, sat))
                        break;
                }
            }
        }

        // make the decision whether the staion is usable and write the report
        // in case there are two many ambiguities, try to remove some of the short pieces by incresing `ength_short`
        std::string decision = "";
        double new_length_short = _short_limit * _interval;
        while (new_length_short < 7200)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, " --- Print Process Report --- ");
            _postPrintReport(decision);

            if (decision != "BAD too many AMBs")
                break;

            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, " --- Check Gap and Remove Short --- ");

            new_length_short *= 1.2;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Too many ambiguities and try to remove some short piece. short_length=" + base_type_conv::dbl2str(new_length_short) + "s");

            for (auto sat : _sat_list)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "## remove short for satellite, " + sat);
                while (true)
                {
                    if (!_postCheckGapShort(new_length_short, sat))
                        break;
                }
            }
        }

        // set spdlog-files
        _allambflag->clearSiteAmbFlag(_crt_rec);
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, " --- Set Single Ambflag --- ");

        if (!_postSetAmbflag())
            return false;

        // write spdlog-files
        if (_amb_output)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, " --- Encoder Product: " + _crt_rec + " --- ");

            if (!_postEncoderProduct(log_suffix))
                return false;
        }
        return true;
    }

    void gnss_proc_exeturboedit::_initPostTurboInfo()
    {
        // re-initialize _post_data/_post_time
        int iepo;
        int flag = _setFlag(0, "NODATA");
        for (auto sat : _sat_list)
        {
            for (iepo = 1; iepo <= _epo_num; iepo++)
            {
                _post_data[sat].obs[iepo][1] = 0.0;   // geometry-free combination  cycle
                _post_data[sat].obs[iepo][2] = 0.0;   // Melbourne-Wuebenna combination cycle
                _post_data[sat].obs[iepo][3] = 0.0;   // ionosphere-free combination for phase cycle
                _post_data[sat].obs[iepo][4] = 0.0;   // ionosphere-free combination for range m
                _post_data[sat].obs[iepo][5] = 0.0;   // residuals of geometry-free combination
                _post_data[sat].obs[iepo][6] = 999.0; // sigma of geometry-free combination
                _post_data[sat].elev[iepo] = 0.0;
                _post_data[sat].iflag[iepo] = flag;
            }
            // init last valid epoch for every satellite
            _post_data[sat].last_epo = 0;
        }

        for (iepo = 1; iepo <= _epo_num; iepo++)
        {

            _post_time[iepo] = _beg_time + _interval * (iepo - 1);
        }
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "Turboedit Information Initialization Success For Post-Processing!");
    }

    bool gnss_proc_exeturboedit::_siteSPP(const std::string &site, const base_time &now)
    {
        // SPPFlt for station
        if (_sppflt[site]->processBatch(now, now) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Kinematic SPPFlt Fail at " + now.str_ymdhms());
            return false;
        }
        return true;
    }

    void gnss_proc_exeturboedit::_postBasicCheck()
    {
        _crt_time = _beg_time;
        _crt_epo = 1;
        int flag_tmp;
        while (_crt_time <= _end_time)
        {
            // std::cerr << _crt_time.str_ymdhms() << std::endl;

            // For Kinematic Stations
            if ((_site_kinematic || _rec_crds[_crt_rec].isZero()) && _ephemeris)
            {
                if (_siteSPP(_crt_rec, _crt_time))
                    _rec_crds[_crt_rec] = _sppflt[_crt_rec]->getCrd(_crt_time);
            }

            /* get observation */
            std::vector<gnss_data_sats> crt_obs = _allobs->obs(_crt_rec, _crt_time);

            // loop observations
            for (auto &it_obs : crt_obs)
            {
                std::string sat = it_obs.sat();
                GSYS sys = it_obs.gsys();

                // exclude sats not in _sat_list
                if (_sat_list.find(sat) == _sat_list.end())
                    continue;

                // with data, set its flag to good at first
                _post_data[sat].iflag[_crt_epo] = DATAFLAG::GOOD;

                // set glonass Frequency ID
                if (sat.substr(0, 1) == "R" && it_obs.channel() >= DEF_CHANNEL)
                {
                    it_obs.channel(_glonass_freq_id.at(sat));
                }

                //Todo
                //check wether after shadow for GPS of BlockType II / IIA

                // check observation exist or not, and obs combination
                if (!_postPrepareObs(it_obs))
                    continue;

                if (!_simulation && _ephemeris)
                {
                    double distance, elev, sat_clk;

                    // compute satellite position, distance, elevation
                    if (!_getSatInfo(sat, distance, elev, sat_clk))
                    {
                        flag_tmp = _post_data[sat].iflag[_crt_epo];
                        _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "NOBRD");
                        continue;
                    }

                    _post_data[sat].obs[_crt_epo][4] -= (distance - sat_clk);
                    _post_data[sat].elev[_crt_epo] = elev;

                    if (elev < _min_elev)
                    {
                        flag_tmp = _post_data[sat].iflag[_crt_epo];
                        _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "LOWELE");
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, sat + " has low elevation = " + base_type_conv::dbl2str(elev) + " at Epoch " + base_type_conv::int2str(_crt_epo));
                        continue;
                    }
                }

                //check widelane and ionosphere for detecting large cycle slips
                _postCheckLargeSlips(sat);

                // check loss-of-lock indicator
                if (it_obs.getlli(_post_phase_obs[sys][0]) >= 1 || it_obs.getlli(_post_phase_obs[sys][1]) >= 1)
                {
                    flag_tmp = _post_data[sat].iflag[_crt_epo];
                    _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "LLI");
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Exist loss-of-lock For " + sat + " in Epoch " + base_type_conv::int2str(_crt_epo));
                }

            } // end loop observations

            // check range
            if (!_simulation && _ephemeris && _check_pc)
                _postCheckRange();

            // Update time & epoch
            _crt_time = _crt_time + _interval;
            _crt_epo = floor((_crt_time - _beg_time) / _interval) + 1;
        }
    }

    bool gnss_proc_exeturboedit::_postPrepareObs(gnss_data_obs_manager &obsdata)
    {
        GSYS gsys = obsdata.gsys();
        std::string sat = obsdata.sat();

        // For GLONASS when 3 frequency
        if (_post_crt_band.find(gsys) == _post_crt_band.end())
            return false;

        GOBS obsL1 = obsdata.select_phase(_post_crt_band[gsys].first);
        GOBS obsP1 = obsdata.select_range(_post_crt_band[gsys].first);
        double P1 = obsdata.getobs(obsP1);
        double L1 = obsdata.getobs(obsL1);
        _post_phase_obs[gsys].clear();
        _post_phase_obs[gsys].push_back(obsL1);
        _post_phase_obs[gsys].push_back(obsL1);

        // check single frequency
        if (_check_sf)
        {
            if (double_eq(P1, 0.0) || double_eq(L1, 0.0))
            {
                int flag_tmp = _post_data[sat].iflag[_crt_epo];
                _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "NO4");
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, sat + " lack of full data at Epoch " + base_type_conv::int2str(_crt_epo));
                return false;
            }
            _post_data[sat].obs[_crt_epo][3] = L1 * obsdata.frequency(_post_crt_band[gsys].first); // L1 [m]
            _post_data[sat].obs[_crt_epo][4] = P1;                                                 // P1 [m]
            _post_data[sat].obs[_crt_epo][1] = _post_data[sat].obs[_crt_epo][3] - P1;              // L1-P1 [m]
        }
        // check MW+GF
        if (_check_mw || _check_gf)
        {
            GOBS obsL2 = obsdata.select_phase(_post_crt_band[gsys].second);
            GOBS obsP2 = obsdata.select_range(_post_crt_band[gsys].second);
            double P2 = obsdata.getobs(obsP2);
            double L2 = obsdata.getobs(obsL2);
            _post_phase_obs[gsys].back() = obsL2;

            if (double_eq(P1, 0.0) || double_eq(L1, 0.0) || double_eq(P2, 0.0) || double_eq(L2, 0.0))
            {
                int flag_tmp = _post_data[sat].iflag[_crt_epo];
                _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "NO4");
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, sat + " lack of full data at Epoch " + base_type_conv::int2str(_crt_epo));
                return false;
            }

            gnss_data_obs gobsL1 = gnss_data_obs(obsL1);
            gnss_data_obs gobsL2 = gnss_data_obs(obsL2);
            gnss_data_obs gobsP1 = gnss_data_obs(obsP1);
            gnss_data_obs gobsP2 = gnss_data_obs(obsP2);

            _post_data[sat].obs[_crt_epo][1] = obsdata.L4_cycle(gobsL1, gobsL2);                 // Geometry-free combination [cycle]
            _post_data[sat].obs[_crt_epo][2] = obsdata.MW_cycle(gobsL1, gobsL2, gobsP1, gobsP2); // MW combination [cycle]
            _post_data[sat].obs[_crt_epo][3] = obsdata.L3_cycle(gobsL1, gobsL2);                 // LC combination [cycle]
            _post_data[sat].obs[_crt_epo][4] = obsdata.P3(gobsP1, gobsP2);                       // PC combination [m]
        }

        return true;
    }

    bool gnss_proc_exeturboedit::_getSatInfo(std::string sat, double &distance, double &elev, double &sat_clk)
    {
        bool valid = false;
        double sat_crd[3];
        Triple sat_crd_Triple, site_sat_vector, site_crd_Triple;
        double delay = 0.0;
        base_time sat_time;
        std::shared_ptr<gnss_data_eph> geph;
        site_crd_Triple = _rec_crds[_crt_rec];
        while (true)
        {
            sat_time = _crt_time - delay;
            geph = _allnav->find(sat, sat_time, false); // add false to not mind healthy status, test by glfeng
            // no broadcast ephemeris
            if (!geph)
                break;
            // invalid broadcast ephemeris
            if (fabs(sat_time.diff(geph->epoch()) / 3600.0) > 4.0)
                break;

            // can't get sat_crd from broadcast ephemeris
            if (geph->pos(sat_time, sat_crd, NULL, NULL, false) < 0)
                break; // add false to not mind healthy status, test by glfeng

            sat_crd_Triple = Triple(sat_crd);
            if (site_crd_Triple.isZero() || site_crd_Triple.array().isNaN().any())
                break;

            site_sat_vector = sat_crd_Triple - site_crd_Triple;
            distance = site_sat_vector.norm();

            if (abs(distance / CLIGHT - delay) < 1E-9)
            {
                valid = true;
                break;
            }

            delay = distance / CLIGHT;
        }

        // Compute elevation & sat clks
        if (valid)
        {
            if (geph->clk(sat_time, &sat_clk, NULL, NULL, false) >= 0) // add false to not mind healthy status, test by glfeng
            {
                sat_clk *= CLIGHT;
                // CRS
                elev = (site_crd_Triple[0] * site_sat_vector[0] + site_crd_Triple[1] * site_sat_vector[1] + site_crd_Triple[2] * site_sat_vector[2]) / site_crd_Triple.norm() / distance;
                elev = 90.0 - acos(elev) * 180.0 / hwa_pi;
                
                return true;
            }
        }

        distance = 0.0;
        sat_clk = 0.0, elev = 0.0;
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, sat + " lack of suitable broadcast ephemeris at Time " + _crt_time.str_ymdhms());
        return false;
    }

    void gnss_proc_exeturboedit::_postCheckLargeSlips(std::string sat)
    {
        // check widelane and ionosphere for detecting large cycle slips
        if (_post_data[sat].last_epo >= 1)
        {
            int flag_tmp;

            double dummy;
            if (_check_mw) // widelane check [cycle]
            {
                dummy = fabs(_post_data[sat].obs[_crt_epo][2] - _post_data[sat].obs[_post_data[sat].last_epo][2]);
                if (dummy > _mw_limit * _scaling_factor["MW"])
                {
                    flag_tmp = _post_data[sat].iflag[_crt_epo];
                    _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "MWJUMP");
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Bad Widelane at Epoch " + base_type_conv::int2str(_crt_epo) + " for " + sat + ", it residuals : " + base_type_conv::dbl2str(dummy));
                }
            }
            if (_check_gf)
            {
                // ionosphere check [cycle]
                dummy = fabs(_post_data[sat].obs[_crt_epo][1] - _post_data[sat].obs[_post_data[sat].last_epo][1]) / _crt_time.diff(_post_time[_post_data[sat].last_epo]);
                if (!_simulation && _post_data[sat].elev[_crt_epo] < 45.0)
                {
                    dummy *= sin(_post_data[sat].elev[_crt_epo] / 180.0 * hwa_pi);
                }
                if (dummy > _gf_limit * _scaling_factor["GF"])
                {
                    flag_tmp = _post_data[sat].iflag[_crt_epo];
                    _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "GFJUMP");
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Bad Ionosphere at Epoch " + base_type_conv::int2str(_crt_epo) + " for " + sat + ", its residuals and ele: " + base_type_conv::dbl2str(dummy) + "  " +
                                                         base_type_conv::dbl2str(_post_data[sat].elev[_crt_epo]));
                }
            }
            if (_check_sf)
            {
                dummy = fabs(_post_data[sat].obs[_crt_epo][1] - _post_data[sat].obs[_post_data[sat].last_epo][1]); // [m]
                if (dummy > _sf_limit * _scaling_factor["LX"])
                {
                    flag_tmp = _post_data[sat].iflag[_crt_epo];
                    _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "MWJUMP");
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Bad LX-CX at Epoch " + base_type_conv::int2str(_crt_epo) + " " + sat + " residuals : " + base_type_conv::dbl2str(dummy));
                }
            }
        }
        _post_data[sat].last_epo = _crt_epo;
    }

    void gnss_proc_exeturboedit::_postCheckRange()
    {
        std::vector<double> pc_res;
        std::vector<std::string> useful_sats;
        for (auto it_sat : _sat_list)
        {
            if (!_isTrue(_post_data[it_sat].iflag[_crt_epo], "OK"))
                continue;

            pc_res.push_back(_post_data[it_sat].obs[_crt_epo][4]);
            useful_sats.push_back(it_sat);
        }

        int flag_tmp;
        if (pc_res.size() == 1 || (pc_res.size() == 2 && fabs(pc_res[0] - pc_res[1]) > 40.0))
        {
            for (auto sat : useful_sats)
            {
                flag_tmp = _post_data[sat].iflag[_crt_epo];
                _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "PCBAD");
            }
            return;
        }

        double mean = 0.0;
        double sig = 0.0;
        std::vector<double> res_tmp;

        for (auto it : pc_res)
        {
            mean += it;
        }

        mean /= pc_res.size();

        for (auto it : pc_res)
        {
            sig += pow(it - mean, 2);
            res_tmp.push_back(fabs(it - mean));
        }
        sig = sqrt(sig / (pc_res.size() - 1));

        double sig1 = 2000;
        double mean1 = 0.0;
        bool isFound = true;
        set<std::string> bad_sats;
        while (isFound && sig1 > 1000 && pc_res.size() >= 3)
        {
            // find out the max.residial
            auto maxRes = max_element(res_tmp.begin(), res_tmp.end());
            int maxPos = distance(res_tmp.begin(), maxRes);
            double MaxValue = pc_res[maxPos];

            // get mean and sigma after removal of the largest residual.
            mean1 = 0.0;
            sig1 = 0.0;
            res_tmp.clear();

            for (auto it : pc_res)
            {
                if (double_eq(it, MaxValue))
                    continue;

                mean1 += it / (pc_res.size() - 1);
            }

            for (auto it : pc_res)
            {
                if (double_eq(it, MaxValue))
                    continue;

                sig1 += (it - mean1) * (it - mean1);
                res_tmp.push_back(fabs(it - mean1));
            }
            sig1 = sqrt(sig1 / (pc_res.size() - 2));

            isFound = sig / sig1 > 2.5 || fabs(mean - mean1) > 200.0 || fabs(MaxValue - mean1) > (3 * (sig1 > 100.0 ? sig1 : 100.0));
            if (!isFound)
                continue;

            mean = mean1;
            sig = sig1;
            bad_sats.insert(useful_sats[maxPos]);
            pc_res.erase(pc_res.begin() + maxPos);
            useful_sats.erase(useful_sats.begin() + maxPos);
        }

        int nbad = 0;
        for (auto sat : _sat_list)
        {
            if (bad_sats.find(sat) == bad_sats.end())
                continue;

            if (fabs(_post_data[sat].obs[_crt_epo][4]) <= _pc_limit)
                continue;

            flag_tmp = _post_data[sat].iflag[_crt_epo];
            _post_data[sat].iflag[_crt_epo] = _setFlag(flag_tmp, "PCBAD");
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Flagged Bad PC " + sat + "  at Epoch " + base_type_conv::int2str(_crt_epo) + "  pc_residuals and threshold: " +
                                                 base_type_conv::dbl2str(_post_data[sat].obs[_crt_epo][4]) + base_type_conv::dbl2str(_pc_limit));
            nbad++;
        }

        if (nbad >= 3)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Too many bad ranges, be sure the coordinates used are good enough!");
        }

        if (pc_res.size() != 0)
            _post_time[_crt_epo] = _post_time[_crt_epo] - mean / CLIGHT;

        return;
    }

    bool gnss_proc_exeturboedit::_postCheckGapShort(double length_short, std::string sat)
    {
        // remove short piece
        bool is_removed = false;

        std::vector<int> search_flag_data;
        search_flag_data.push_back(-999); // no use
        int last_valid_epo = 0;
        int iepo = 0;
        // mark large gap and first epoch
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {
            search_flag_data.push_back(_post_data[sat].iflag[iepo]);
            if (!_isTrue(_post_data[sat].iflag[iepo], "OK"))
                continue; // not usable data

            if (last_valid_epo == 0)
            {
                _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "GAP");
            }
            else
            {
                if (_post_time[iepo].diff(_post_time[last_valid_epo]) > _gap_limit * _interval)
                {
                    _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "GAP");
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Flagged gap at " + base_type_conv::int2str(iepo) + ", last good epoch  " + base_type_conv::int2str(last_valid_epo));
                }
            }
            last_valid_epo = iepo;
            search_flag_data[iepo] = _post_data[sat].iflag[iepo]; // update iflag
        }

        // no usable data
        if (last_valid_epo <= 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "No data to remove: " + sat);
            return is_removed;
        }

        int j = 1;
        int i, k;
        while (j <= _epo_num)
        {

            // find the first ambiguity
            i = _findFlag(j, _epo_num, search_flag_data, "AMB", "+");
            if (i == -999)
                break;

            // find the next ambiguity or end of the data set
            j = _findFlag(i + 1, _epo_num, search_flag_data, "AMB", "+");

            if (j == -999)
                j = _epo_num + 1;

            // find backwards the last usable epoch
            k = _findFlag(i, j - 1, search_flag_data, "OK", "-");

            // count the number of usable
            int ngood = 1; // for epoch i
            for (iepo = i + 1; iepo <= k; iepo++)
            {
                if (_isTrue(search_flag_data[iepo], "OK"))
                    ngood++;
            }

            // Removed short
            if (_post_time[k].diff(_post_time[i + 1]) < length_short || ngood * _interval <= length_short / 2)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Removed short, from epoch " + base_type_conv::int2str(i) + " to " + base_type_conv::int2str(k));
                for (iepo = i; iepo <= k; iepo++)
                {
                    if (_isTrue(search_flag_data[iepo], "OK"))
                    {
                        _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "SHRT");
                        search_flag_data[iepo] = _post_data[sat].iflag[iepo];
                    }
                }
                is_removed = true;
            }

            j = k + 1;
        }
        return is_removed;
    }

    void gnss_proc_exeturboedit::_postCheckMWGF(std::string sat)
    {
        int nused = 0;
        for (int iepo = 1; iepo <= _epo_num; iepo++)
        {
            if (!_isTrue(_post_data[sat].iflag[iepo], "OK"))
                continue;

            nused++;
        }

        if (nused > 0)
        {

            if (_check_mw)
                _postEditWidelane(sat);

            if (_check_gf)
                _postCheckIonosphere(sat);
        }
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, " --- After MW+GF check, short_limit = " + base_type_conv::dbl2str(_short_limit * _interval) + "s --- ");
        while (true)
        {
            if (!_postCheckGapShort(_short_limit * _interval, sat))
                break;
        }
        return;
    }

    void gnss_proc_exeturboedit::_postEditWidelane(std::string sat)
    {
        std::vector<int> search_flg;
        search_flg.push_back(999); // no use
        int iepo = 0;
        // simply check for jump
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {
            search_flg.push_back(3);
            if (!_isTrue(_post_data[sat].iflag[iepo], "OK"))
                continue; // OK = GOOD+AMB

            search_flg[iepo] = 1;
            if (!_isTrue(_post_data[sat].iflag[iepo], "MWJUMP") && !_isTrue(_post_data[sat].iflag[iepo], "GAP")) // 排除上述情况，剩下的基本就是正常的MW值
            {
                search_flg[iepo] = 0;
            }
        }

        //  sequential mean of NW and flag possible are apply to detect detect cycle slips
        //          ( by testing  dabs(nwi-meani) > limit cycles)
        // if a data point is flagged, mean calculation is re-initialized by finding the
        // next two points with a dif.less than limit.
        int kobs = 0, last_epo = 0;
        double mean_mw, mw_diff, mw_adjacent_epo;
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {

            if (search_flg[iepo] > 1)
                continue; // data with bad point

            if (search_flg[iepo] == 1)
                kobs = 0; // possible exsit LWJUMP/GAP, piece start

            if (last_epo >= 1)
            {
                mw_adjacent_epo = _post_data[sat].obs[last_epo][2] - _post_data[sat].obs[iepo][2];
            }

            if (kobs == 0) // first point. piece start
            {
                mean_mw = _post_data[sat].obs[iepo][2];
                search_flg[iepo] = 1;
                kobs = 1;
            }
            else if (kobs == 1) // second point
            {
                mw_diff = _post_data[sat].obs[iepo][2] - mean_mw;
                if (fabs(mw_diff) <= _mw_limit) // two points are ok
                {
                    kobs++;
                    mean_mw = mean_mw + mw_diff / kobs;
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Widelane bad point, " + base_type_conv::int2str(iepo) + "  " + base_type_conv::dbl2str(mw_adjacent_epo));
                    search_flg[last_epo] = 2; // bad point = 2
                    search_flg[iepo] = 1;     // piece start = 1
                    mean_mw = _post_data[sat].obs[iepo][2];
                }
            }
            else // third or later point
            {
                mw_diff = _post_data[sat].obs[iepo][2] - mean_mw;
                if (fabs(mw_diff) <= _mw_limit ||
                    (fabs(mw_diff) <= 1.4 * _mw_limit && fabs(mw_adjacent_epo) <= 0.5))
                {
                    kobs++;
                    mean_mw = mean_mw + mw_diff / kobs;
                    if (search_flg[last_epo] == 1)
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Widelane bad point, " + base_type_conv::int2str(last_epo) + "  " + base_type_conv::dbl2str(mw_adjacent_epo));
                        search_flg[last_epo] = 2;
                    }
                }
                else // possible cycle slip
                {
                    if (search_flg[last_epo] == 1) // set crt epo as second point
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Widelane possible jump, " + base_type_conv::int2str(last_epo));
                        kobs = 1;
                        mean_mw = _post_data[sat].obs[last_epo][2];
                        mw_diff = _post_data[sat].obs[iepo][2] - mean_mw;
                        if (fabs(mw_diff) <= _mw_limit)
                        {
                            kobs++;
                            mean_mw = mean_mw + mw_diff / kobs;
                        }
                        else // set crt epo as bad point
                        {
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, "Widelane bad point, " + base_type_conv::int2str(iepo) + "  " + base_type_conv::dbl2str(mw_adjacent_epo));
                            search_flg[last_epo] = 2;
                            search_flg[iepo] = 1;
                            mean_mw = _post_data[sat].obs[iepo][2];
                        }
                    }
                    else
                    {
                        search_flg[iepo] = 1;
                    }
                }
            }
            last_epo = iepo;
        }

        // set flag
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {
            if (search_flg[iepo] == 1)
            {
                if (!_isTrue(_post_data[sat].iflag[iepo], "MWJUMP") && !_isTrue(_post_data[sat].iflag[iepo], "GAP"))
                {
                    _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "MWJUMP");
                }
            }
            else if (search_flg[iepo] == 2)
            {
                _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "MWBAD");
            }
        }
    }

    void gnss_proc_exeturboedit::_postCheckIonosphere(std::string sat)
    {
        std::vector<int> search_flg;
        search_flg.push_back(999); // no use, for counts
        int iepo = 0;
        // simply check for jump
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {
            search_flg.push_back(2);
            if (!_isTrue(_post_data[sat].iflag[iepo], "OK"))
                continue;

            search_flg[iepo] = 1;
            if (!_isTrue(_post_data[sat].iflag[iepo], "GFJUMP"))
                search_flg[iepo] = 0; // Not LGJUMP
        }

        // loop epochs
        int ibeg, iend, i; // first/last usable epoch, useful epo_num, loop useful epochs
        std::vector<double> inp_lg, inp_ti, inp_res;
        std::vector<int> inp_flg;
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {

            if (search_flg[iepo] == 2)
                continue; // exclude bad point

            // search backward 5 epochs flagged as ok
            if (_searchEpoch(search_flg, iepo, 0, "-", ibeg) == 0)
            {
                _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "GFJUMP");
                continue;
            }

            // search forwards, be aware that we have at least one usable point iepo.
            _searchEpoch(search_flg, iepo, _epo_num + 1, "+", iend);

            // total epochs
            // kobs = iend - ibeg + 1;
            // insert relavant elements
            inp_lg.clear();
            inp_ti.clear();
            inp_flg.clear();
            inp_res.clear();

            for (i = ibeg; i <= iend; i++)
            {
                inp_lg.push_back(_post_data[sat].obs[i][1]);
                inp_res.push_back(_post_data[sat].obs[i][5]);
                inp_flg.push_back(search_flg[i]);
                inp_ti.push_back(_post_time[i].diff(_beg_time));
            }

            // check for jump within the selected piece
            double rms = 0.0;
            bool MaybeNotjump = _checkForJump("LGC", inp_ti, inp_lg, inp_flg, inp_res, rms);
            // update search_flg
            for (i = ibeg; i <= iend; i++)
            {
                search_flg[i] = inp_flg[i - ibeg];
            }
            // true, valid check , may not exsit LG jump
            if (MaybeNotjump)
            {
                // update turbo data
                _post_data[sat].obs[iepo][5] = inp_res[iepo - ibeg];
                _post_data[sat].obs[iepo][6] = rms;
                // residual control only
                if (double_eq(_gf_rms_limit, 0.0) && fabs(_post_data[sat].obs[iepo][5]) <= _gf_limit)
                {

                    continue;
                }
                // with rms control
                if (!double_eq(_gf_rms_limit, 0.0) && rms <= _gf_rms_limit &&
                    fabs(_post_data[sat].obs[iepo][5]) <= (3 * rms > _gf_limit ? 3 * rms : _gf_limit))
                {

                    continue;
                }
            }

            _post_data[sat].iflag[iepo] = _setFlag(_post_data[sat].iflag[iepo], "GFJUMP");
        }
    }

    bool gnss_proc_exeturboedit::_checkForJump(std::string strflag, const std::vector<double> &ti,
                                        const std::vector<double> &values, std::vector<int> &flag, std::vector<double> &res, double &rms)
    {
        // with coefficients of polynomial, degree = 2
        std::vector<double> coeff;
        coeff.push_back(0.0);
        coeff.push_back(0.0);

        coeff.push_back(0.0);
        coeff.push_back(0.0);

        std::vector<int> ipt;                                        // with information about used epoch, default is -1
        int kobs = ti.size();                                   // number of obs
        // int epo0 = round(fmod(ti[0], 86400.0) / _interval) + 1; // sod in a single day
        double pre_rms = 1.e10;                                 // previous rms
        int degree = 0, nflag = 0, iter = 0, iepo = 0;

        bool iteration = true;
        while (iteration)
        {
            if (!_polynomialFit(degree, ti, values, flag, coeff, res, ipt, rms))
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Ploydf error!");
                return false;
            }

            // detecting flagged number
            nflag = 0;
            for (iepo = 0; iepo < kobs; iepo++)
            {
                if (ipt[iepo] == -1)
                    continue;

                if (fabs(res[iepo]) > (3 * rms > 0.30 ? 3 * rms : 0.30)) // residuals with 3 times RMS
                {
                    if (flag[iepo] == 0)
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, strflag + " flagged(b) : at Epoch " + base_type_conv::int2str(epo0 + iepo) + "   " + base_type_conv::dbl2str(ti[iepo]) + "  [max(0.30,3rms)] -- 3rms=" + base_type_conv::dbl2str(3 * rms) + " -- residuals=" + base_type_conv::dbl2str(res[iepo]));
                        nflag++;
                        flag[iepo] = 1;
                    }
                }
            }
            // Number of iterations
            iter++;
            iteration = false;

            if (nflag > 0)
            {
                iteration = true;
            }
            else if (rms > 0.3) // rms threshold (unit error)
            {
                iteration = true;
                if (rms / pre_rms > 0.95) // have not obvious change, add degree
                {
                    degree++;
                    iter = 1;
                }

                if (degree > 2)
                    iteration = false;
            }

            pre_rms = rms;
            if (iteration && iter <= 2)
                iteration = true;
        }

        return true;
    }

    bool gnss_proc_exeturboedit::_polynomialFit(int degree, const std::vector<double> &X, const std::vector<double> &Y,
                                         const std::vector<int> &L, std::vector<double> &C, std::vector<double> &V, std::vector<int> &ipt, double &rms)
    {
        ipt.clear(); // ipt  re-initialization for next time
        // normalisation of independent argument
        double Xmax = -1.e20;
        double Xmin = 1.e20;
        int N = X.size();
        int m, i, j, k;
        // searching the Max.  and  min.  Independent argument values
        for (i = 0; i < N; i++)
        {
            ipt.push_back(-1); // flag used epoch
            // L: 0: best - NO LG/LC/.. JUMP data; 1: Followed by best - OK data[Jump + GOOD]; 2: Not OK
            // Preset Independent variable search range
            if (L[i] <= 1)
            {
                Xmax = X[i] > Xmax ? X[i] : Xmax;
                Xmin = X[i] < Xmin ? X[i] : Xmin;
                V[i] = 0.0;
            }
        }
        // No way to search
        if (Xmax == Xmin)
            return false;

        double dx = (Xmax + Xmin) / 2.0;
        double ft = 2.0 / (Xmax - Xmin);

        // initial importent parameter

        Matrix Q;
        Vector F;
        Vector A;

        if (degree > 0)
        {
            Q.resize(degree, degree);
            Q.setZero();

            F.resize(degree);
            F.setZero();

            A.resize(degree);
            A.setZero();
        }

        int neffective_obs = 0; // number of obs

        // loop over all functional values
        double Xnorm, Xnorj;
        for (j = N - 1; j >= 1; j--) // From the last to the second
        {
            if (L[j] == 0) // get first usable functional value, must best
            {
                // searching next usable functional value to form difference
                m = j - 1;
                while (m >= 0 && L[m] > 1)
                    m--;

                if (m != -1) // get two obs
                {
                    Xnorj = (X[j] - dx) * ft;
                    Xnorm = (X[m] - dx) * ft;
                    neffective_obs++; // Number of observations
                    if (degree <= 0)
                        continue;

                    // computaion of a-matrix (row number j)
                    for (k = 0; k < degree; k++)
                    {
                        A(k) = pow(Xnorm, k + 1) - pow(Xnorj, k + 1); // equal to coefficient matrix
                    }
                    // contribution to q - matrix
                    Q += A * A.transpose(); // equal to BTPB
                    // contribution to right hand side
                    for (i = 0; i < degree; i++)
                    {
                        F(i) += A(i) * (Y[m] - Y[j]); // equal to BTPL
                    }
                }
            }
        }

        // not enough obs < number of parameters
        if (neffective_obs < degree)
            return false;

        // solution std::vector (polynomial coefficients) equal to inv(BTPB)BTPL
        // Check Matrix singularity
        if (degree > 0)
        {
            Matrix solution;
            if (double_eq(Q.maxCoeff(), 0.0))
                return false;

            try
            {
                solution << Q.inverse() * F;
            }
            catch (exception e)
            {
                return false;
            }

            for (i = 0; i < degree; i++)
            {
                C[i] = solution(i, 0);
            }
        }

        //  residuals, rms
        rms = 0.0;
        double Vmax = 0.0, ys = 0.0;
        for (j = N - 1; j >= 1; j--)
        {
            if (L[j] <= 1) // same ways ..... like before
            {
                // searching next usable functional value to form difference
                m = j - 1;
                while (m >= 0 && L[m] > 1)
                    m--;

                if (m != -1)
                {
                    Xnorj = (X[j] - dx) * ft;
                    Xnorm = (X[m] - dx) * ft;

                    ys = 0.0;
                    if (degree > 0)
                    {
                        for (k = 1; k <= degree; k++)
                        {
                            ys += C[k - 1] * (pow(Xnorm, k) - pow(Xnorj, k));
                        }
                    }

                    V[j] = ys - (Y[m] - Y[j]); // get residuals

                    if (L[j] == 0) // calculate rms with best data
                    {
                        if (fabs(V[j]) > Vmax)
                            Vmax = fabs(V[j]);

                        rms += V[j] * V[j];
                    }

                    ipt[j] = m;
                }
            }
        }

        double rms1;
        if (neffective_obs - degree != 0 && rms > 0.0) // judge Degrees of freedom to compute Unit weight error
        {
            Vmax = rms - Vmax * Vmax;
            rms = sqrt(rms / (neffective_obs - degree)); // Unit weight error
            if (neffective_obs - degree - 1 != 0 && Vmax > 0.0)
            {
                rms1 = sqrt(Vmax / (neffective_obs - degree - 1));
                if (rms / rms1 > 2.0)
                    rms = rms1;
            }

            return true;
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "BAD RMS, " + base_type_conv::dbl2str(rms) +
                                                 "  number of effective obs and degree  " + base_type_conv::int2str(neffective_obs) + "  " + base_type_conv::int2str(degree));
            return false;
        }
    }

    void gnss_proc_exeturboedit::_postPrintReport(std::string &decision)
    {
        std::map<int, std::map<std::string, int>> sat_state; // five state/sat/number
        std::map<int, int> state_sum;              // five sate/number
        std::map<std::string, bool> lfirst_sat;         // first_sats
        int useful_sat = 0;                   // used sat number
        int iepo;
        for (auto sat : _sat_list)
        {
            lfirst_sat[sat] = true;
            for (iepo = 1; iepo <= _epo_num; iepo++)
            {
                if (_isTrue(_post_data[sat].iflag[iepo], "NODATA"))
                    continue;

                sat_state[1][sat] += 1; // 1: Total observatons
                if (_isTrue(_post_data[sat].iflag[iepo], "NO4"))
                {
                    sat_state[4][sat] += 1; // 4: Incomplete observations
                }
                else if (_isTrue(_post_data[sat].iflag[iepo], "LOWELE"))
                {
                    sat_state[5][sat] += 1; // 5: Low elevation observations
                }
                else if (!_isTrue(_post_data[sat].iflag[iepo], "OK"))
                {
                    sat_state[2][sat] += 1; // 2: No-brd+Bad-brd+Short+PC/LW/LG-Bad+Shadow
                }
                else if (!_isTrue(_post_data[sat].iflag[iepo], "GOOD"))
                {

                    if (lfirst_sat[sat])
                        lfirst_sat[sat] = false;

                    else
                        sat_state[3][sat] += 1; // 3: OK-GOOD[Gap+LWJump+LGJump+LLI], means new ambiguities
                }
            }

            // how many observations for TOTAL,NO4,LOWELE,OK,GOOD
            if (sat_state[1][sat] == 0)
                continue;

            useful_sat++;
            state_sum[1] += sat_state[1][sat];
            state_sum[2] += sat_state[2][sat];
            state_sum[3] += sat_state[3][sat];
            state_sum[4] += sat_state[4][sat];
            state_sum[5] += sat_state[5][sat];
        }

        std::map<int, int> nobs_epo; // nbos each epoch
        std::map<int, int> nepo_nsv; // e.g 1 obs have 3 epochs, 2 obs have 10 epochs, ...

        int kepo4 = 0;
        int kepo4_nobs = 0;
        int nepo_input = 1;
        bool linput;
        for (int i = 1; i <= _epo_num; i++)
        {
            linput = false;
            nobs_epo[i] = 0;
            for (auto sat : _sat_list)
            {
                if (!_isTrue(_post_data[sat].iflag[i], "NODATA"))
                {
                    linput = true;
                }

                if (_isTrue(_post_data[sat].iflag[i], "OK"))
                {
                    nobs_epo[i] += 1;
                }
            }

            if (linput)
                nepo_input++;
            nepo_nsv[nobs_epo[i]] += 1;
            if (nobs_epo[i] >= 4)
            {
                kepo4++;
                kepo4_nobs += nobs_epo[i];
            }
        }

        double mean_namb_per_prn = 0.0;

        if (useful_sat != 0)
            mean_namb_per_prn = state_sum[3] / (useful_sat * 1.0);

        double mean_nprn_per_epo = 0.0;

        if (kepo4 != 0)
            mean_nprn_per_epo = kepo4_nobs / (kepo4 * 1.0);

        double percent_kepo4 = 0.0;
        // i.e. useful_sat != 0

        if (state_sum[1] != 0)
            percent_kepo4 = kepo4 / (nepo_input * 1.0);

        decision = "GOOD";
        if (_check_statistics)
        {
            if (percent_kepo4 < _min_percent * 1.e-2)
            {
                decision = "BAD too many removed";
            }
            else if (mean_namb_per_prn > _max_mean_namb)
            {
                decision = "BAD too many AMBs";
            }
            else if (mean_nprn_per_epo < _min_mean_nprn)
            {
                decision = "BAD too small mean nprn";
            }
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "+++  Summary  +++");
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, " Site and Evaluation : " + _crt_rec + "   " + decision);
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, " #EPOCH > 4_SAT      : " + base_type_conv::dbl2str(percent_kepo4 * 100) + "%");
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, " MEAN_#AMB_PER_PRN   : " + base_type_conv::dbl2str(mean_namb_per_prn));
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, " MEAN_#PRN_PER_EPOCH : " + base_type_conv::dbl2str(mean_nprn_per_epo));
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, " NUMBER_OF_obs       :    #TOTAL:   " + base_type_conv::int2str(state_sum[1]) +
                                             "   #REMOVED:   " + base_type_conv::int2str(state_sum[2]) + "   #LOWELEV:   " + base_type_conv::int2str(state_sum[5]) +
                                             "   #AMB:   " + base_type_conv::int2str(state_sum[3]));
        std::stringstream str_prn, str_total, str_removed, str_no4, str_lowelev, str_amb;
        str_prn << " PRN      TOTAL";
        str_total << " #TOTAL   " << setw(5) << state_sum[1];
        str_removed << " #REMOVED " << setw(5) << state_sum[2];
        str_no4 << " #NO_4TYP " << setw(5) << state_sum[4];
        str_lowelev << " #LOWELEV " << setw(5) << state_sum[5];
        str_amb << " #AMB     " << setw(5) << state_sum[3];
        for (auto sat : _sat_list)
        {
            if (sat_state[1][sat] == 0)
                continue;

            str_prn << "  " + sat;
            str_total << setw(5) << sat_state[1][sat];
            str_removed << setw(5) << sat_state[2][sat];
            str_no4 << setw(5) << sat_state[4][sat];
            str_lowelev << setw(5) << sat_state[5][sat];
            str_amb << setw(5) << sat_state[3][sat];
        }
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_prn.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_total.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_removed.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_no4.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_lowelev.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_amb.str());

        std::stringstream str_svobs, str_epoch;
        str_svobs << " #SV obs. ";
        str_epoch << " #epoch   ";
        for (int j = 0; j <= 36; j++)
        {
            str_svobs << setw(5) << j;
            str_epoch << setw(5) << nepo_nsv[j];
        }
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_svobs.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, str_epoch.str());
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "---  Summary  ---");
    }

    bool gnss_proc_exeturboedit::_postSetAmbflag()
    {
        std::map<std::string, std::map<int, int>> local_flag;
        std::map<std::string, std::vector<int>> search_flag_data;
        std::map<int, int> namb_epo;
        int iepo, jepo, kepo;
        for (auto sat : _sat_list)
        {
            search_flag_data[sat].push_back(999);
            for (iepo = 1; iepo <= _epo_num; iepo++)
            {
                search_flag_data[sat].push_back(_post_data[sat].iflag[iepo]);
                local_flag[sat][iepo] = 0;
                namb_epo[iepo] = 0;
            }
        }

        // Ambiguities infomation flagged at local_flag
        // local_flag[sat][iepo]==iepo BAD data
        // local_flag[sat][iepo] < 0   AMB data
        // local_flag[sat][iepo] >iepo DEL data
        gnss_data_ambflag_head ambflag_head;
        ambflag_head.new_rm_obs = 0;
        ambflag_head.new_amb = 0;
        ambflag_head.avaiable_obs = 0;
        for (auto sat : _sat_list)
        {
            iepo = 0;
            while (iepo < _epo_num)
            {
                iepo++;
                // check obs or not
                if (_isTrue(_post_data[sat].iflag[iepo], "NODATA"))
                    continue;

                // select bad point
                if (!_isTrue(_post_data[sat].iflag[iepo], "OK"))
                {
                    jepo = iepo;
                    // Traverse from the current epoch to the end, if the data is OK, jump out
                    while (jepo <= _epo_num && !_isTrue(_post_data[sat].iflag[jepo], "OK"))
                    {
                        // Delete an observation if it exists
                        if (!_isTrue(_post_data[sat].iflag[jepo], "NODATA"))
                        {
                            ambflag_head.new_rm_obs += 1;
                            local_flag[sat][iepo] = jepo; // iepo->jepo  bad duration
                        }
                        jepo++;
                    }
                    iepo = jepo - 1; // Minus 1 because jepo adds 1
                }
                else // OK data
                {
                    if (!_isTrue(_post_data[sat].iflag[iepo], "GOOD")) // OK-GOOD：insert AMB because of GAP/JUMP
                    {
                        //  find the next ambiguity or end of the data set
                        jepo = _findFlag(iepo + 1, _epo_num, search_flag_data[sat], "AMB", "+");

                        if (jepo == -999)
                            jepo = _epo_num + 1;

                        // find backwards the last usable epoch
                        jepo = _findFlag(iepo + 1, jepo - 1, search_flag_data[sat], "GOOD", "-");
                        if (jepo == -999)
                            jepo = iepo;

                        // accumulate namb per epoch
                        for (kepo = iepo; kepo <= jepo; kepo++)
                        {
                            namb_epo[kepo] += 1;
                        }
                        // new amb num
                        ambflag_head.new_amb += 1;

                        local_flag[sat][iepo] = -jepo; // New amb 是负数
                    }
                    ambflag_head.avaiable_obs += 1;
                }
            }
        }

        // update ambflag header
        std::string sat = *(_sat_list.begin());
        ambflag_head.intv = _interval;
        ambflag_head.duration = _end_time.diff(_beg_time);
        ambflag_head.beg_mjd = _beg_time.mjd();
        ambflag_head.beg_sod = _beg_time.sod() + _beg_time.dsec();
        int max_nobs = -1;
        for (iepo = 1; iepo <= _epo_num; iepo++)
        {
            max_nobs = max_nobs > namb_epo[iepo] ? max_nobs : namb_epo[iepo];
        }
        ambflag_head.max_amb_1epo = max_nobs;

        // add ambflag_head
        _allambflag->addAmbFlagHead(_crt_rec, ambflag_head);
        _cycleslip[_crt_rec].back()->set_active_amb(_crt_rec, ambflag_head.max_amb_1epo);
        // add ambflag data
        if (!_postSetAmbflagData(local_flag))
            return false;

        else
            return true;
    }

    bool gnss_proc_exeturboedit::_postSetAmbflagData(std::map<std::string, std::map<int, int>> &sat_epo_beg_end)
    {
        gnss_data_ambflag_data AmbFlag_data;
        base_time beg, end;
        for (int iepo = 1; iepo <= _epo_num; iepo++)
        {
            for (std::string sat : _sat_list)
            {
                AmbFlag_data.reason = "RN_     ";
                if (sat_epo_beg_end[sat][iepo] == iepo) // End epoch is stored
                {
                    AmbFlag_data.identify = "BAD";
                    AmbFlag_data.iflag = "5";
                    AmbFlag_data.reason = _strFlagDescription(sat, "BAD", iepo);
                }
                else if (sat_epo_beg_end[sat][iepo] > iepo)
                {
                    AmbFlag_data.identify = "DEL";
                    AmbFlag_data.iflag = "6";
                    AmbFlag_data.reason = _strFlagDescription(sat, "DEL", iepo);
                }
                else if (sat_epo_beg_end[sat][iepo] < 0)
                {
                    AmbFlag_data.identify = "AMB";
                    AmbFlag_data.iflag = "1";
                    AmbFlag_data.reason = _strFlagDescription(sat, "AMB", iepo);
                    sat_epo_beg_end[sat][iepo] = -sat_epo_beg_end[sat][iepo];
                }

                if (AmbFlag_data.reason == "RN_Unkown_Amb" || AmbFlag_data.reason == "RN_Unkown_Bad")
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, _crt_rec + " " + AmbFlag_data.reason + " at Epoch  " + base_type_conv::int2str(iepo) +
                                                         +"  " + sat + ", iflag= " + base_type_conv::int2str(_post_data[sat].iflag[iepo]));
                    return false;
                }

                if (AmbFlag_data.reason != "RN_     ")
                {
                    AmbFlag_data.beg_epo = iepo;
                    AmbFlag_data.end_epo = sat_epo_beg_end[sat][iepo];
                    AmbFlag_data.C1 = 0.0;
                    AmbFlag_data.C2 = 0.0;
                    _allambflag->addAmbFlagData(_crt_rec, sat, AmbFlag_data);
                    beg = _beg_time + (AmbFlag_data.beg_epo - 1) * _interval;
                    end = _beg_time + (AmbFlag_data.end_epo - 1) * _interval;
                    _cycleslip[_crt_rec].back()->add_ambflag(_crt_rec, sat, AmbFlag_data.identify, beg, end);
                }
            }
        }
        return true;
    }

    std::string gnss_proc_exeturboedit::_strFlagDescription(std::string sat, std::string str3, int epo)
    {
        std::string str_back = "     ";
        if (str3 == "BAD")
        {
            if (_isTrue(_post_data[sat].iflag[epo], "LOWELE"))
                str_back = "lowelevation";

            else if (_isTrue(_post_data[sat].iflag[epo], "SHRT"))
                str_back = "shortpiece";

            else if (_isTrue(_post_data[sat].iflag[epo], "MWBAD"))
                str_back = "badwidelane";

            else if (_isTrue(_post_data[sat].iflag[epo], "GFBAD"))
                str_back = "badionosphere";

            else if (_isTrue(_post_data[sat].iflag[epo], "NO4"))
                str_back = "lessthan4obs";

            else if (_isTrue(_post_data[sat].iflag[epo], "PCBAD"))
                str_back = "badrange";

            else if (_isTrue(_post_data[sat].iflag[epo], "ASHADOW"))
                str_back = "aftershadow";

            else if (_isTrue(_post_data[sat].iflag[epo], "NOBRD"))
                str_back = "nobrd";

            else
                str_back = "Unkown_Bad";
        }
        else if (str3 == "AMB")
        {
            if (_isTrue(_post_data[sat].iflag[epo], "GAP"))
                str_back = "biggap";

            else if (_isTrue(_post_data[sat].iflag[epo], "MWJUMP"))
                str_back = "widelanejump";

            else if (_isTrue(_post_data[sat].iflag[epo], "GFJUMP"))
                str_back = "iono.jump";

            else if (_isTrue(_post_data[sat].iflag[epo], "LLI"))
                str_back = "flaginrinex";

            else
                str_back = "Unkown_Amb";
        }
        else // DEL
        {
            str_back = "mixed";
        }
        str_back = "RN_" + str_back;
        return str_back;
    }

    bool gnss_proc_exeturboedit::_postEncoderProduct(std::string logsuffix)
    {
        if (_allambflag->getAllAmbFlag().find(_crt_rec) == _allambflag->getAllAmbFlag().end())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "No ambflag info : " + _crt_rec);
            return false;
        }

        std::string strambflagpath = dynamic_cast<set_out *>(_set)->outputs("ambflag_dir");

        if (strambflagpath.empty())
            strambflagpath = "file://log_tb";
        if (ACCESS(strambflagpath.substr(7).c_str(), 0) != 0)
        {
            MKDIR(strambflagpath.substr(7).c_str());
        }

        std::string site = _crt_rec;
        transform(site.begin(), site.end(), site.begin(), ::tolower);
        std::stringstream str;
        str << strambflagpath << PATH_SEPARATOR << site << setw(3) << setfill('0') << _beg_time.doy() << "0."
            << setw(2) << setfill('0') << _beg_time.yr() << "o" + logsuffix;

        base_io *gout = new base_file(_spdlog);
        //set I/O
        gout->path(str.str());
        gout->spdlog(_spdlog);

        //set coder
        base_coder *gcoder = new gnss_coder_ambflag(nullptr, "", 4096);

        gcoder->clear();
        gcoder->spdlog(_spdlog);
        gcoder->path(str.str());
        gcoder->add_data("IDX", _allambflag);
        gout->coder(gcoder);
        // write
        gout->run_write();

        delete gout;
        gout = nullptr;

        delete gcoder;
        gcoder = nullptr;
        return true;
    }

    int gnss_proc_exeturboedit::_str2DataFlag(std::string data_flag)
    {
        std::string tmp = data_flag;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

        if (tmp == "GOOD")
            return DATAFLAG::GOOD;

        else if (tmp == "GFJUMP")
            return DATAFLAG::GFJUMP;

        else if (tmp == "MWJUMP")
            return DATAFLAG::MWJUMP;

        else if (tmp == "GAP")
            return DATAFLAG::GAP;

        else if (tmp == "LLI")
            return DATAFLAG::LLI;

        else if (tmp == "PCBAD")
            return DATAFLAG::PCBAD;

        else if (tmp == "NOBRD")
            return DATAFLAG::NOBRD;

        else if (tmp == "GFBAD")
            return DATAFLAG::GFBAD;

        else if (tmp == "MWBAD")
            return DATAFLAG::MWBAD;

        else if (tmp == "ASHADOW")
            return DATAFLAG::ASHADOW;

        else if (tmp == "SHRT")
            return DATAFLAG::SHRT;

        else if (tmp == "LOWELE")
            return DATAFLAG::LOWELE;

        else if (tmp == "NO4")
            return DATAFLAG::NO4;

        else if (tmp == "NODATA")
            return DATAFLAG::NODATA;

        else
            return -1;
    }

    int gnss_proc_exeturboedit::_setFlag(int iflag, std::string type)
    {
        std::string trim_type = base_type_conv::trim(type);
        transform(trim_type.begin(), trim_type.end(), trim_type.begin(), ::toupper);

        int set_flag = iflag;

        if (trim_type == "OK")
        {
            for (int i = 16; i <= 31; i++)
            {
                set_flag = set_flag & ~(1 << (i + 1 - 1)); // 把右数第i + 1位变成0
            }
        }
        else if (trim_type == "GOOD")
        {
            for (int i = 0; i <= 31; i++)
            {
                set_flag = set_flag & ~(1 << (i + 1 - 1)); // 把右数第i + 1位变成0
            }
        }
        else
        {
            set_flag = set_flag | (1 << (_str2DataFlag(trim_type) + 1 - 1)); // 把右数第str2DataFlag(trim_type) + 1位变成1
        }
        return set_flag;
    }

    int gnss_proc_exeturboedit::_findFlag(int istart, int istop, const std::vector<int> &flg, std::string flag_type, std::string direction)
    {
        int i1, i2, istep;

        if (direction == "+")
        {

            i1 = istart;
            i2 = istop;
            istep = 1;
        }
        else
        {

            i1 = istop;
            i2 = istart;
            istep = -1;
        }

        int iepo = -999; // if can't find, return the -1
        for (int i = i1; i != i2 + istep; i += istep)
        {

            if (!_isTrue(flg[i], flag_type))
            {

                continue;
            }
            iepo = i;
            break;
        }

        return iepo;
    }

    bool gnss_proc_exeturboedit::_isTrue(int iflag, std::string type)
    {
        bool istrue = false;
        std::string trim_type = base_type_conv::trim(type);
        transform(trim_type.begin(), trim_type.end(), trim_type.begin(), ::toupper);

        if (trim_type == "OK")
        {
            // note: z'FFFF0000' = -65536
            istrue = (iflag & -65536) == 0 ? true : false;
        }
        else if (trim_type == "GOOD")
        {
            istrue = iflag == 0 ? true : false;
        }
        else if (trim_type == "AMB")
        {
            if ((iflag & -65536) == 0 && iflag != 0)
            {
                istrue = true;
            }
        }
        else
        {
            int tmp = iflag | (1 << (_str2DataFlag(trim_type) + 1 - 1));
            istrue = tmp == iflag ? true : false;
        }

        return istrue;
    }

    int gnss_proc_exeturboedit::_searchEpoch(const std::vector<int> &flg, int crt_epo, int limit_epo, std::string mode, int &use_epo)
    {
        int jepo, prefix;
        int max_epo_num;
        if (mode == "-") // backward "-"
        {
            jepo = crt_epo - 1;
            max_epo_num = 30;
            prefix = -1;
        }
        if (mode == "+") // forward "-"
        {
            jepo = crt_epo;
            max_epo_num = 29;
            prefix = 1;
        }

        int npoint = 0;
        use_epo = crt_epo;
        while (true)
        {
            if (npoint >= 5)
                break;

            if (abs(crt_epo - jepo) > max_epo_num)
                break;

            if (jepo == limit_epo)
                break;

            if (flg[jepo] < 2)
            {

                npoint++;
                use_epo = jepo;
            }
            jepo = jepo + prefix;
        }

        return npoint;
    }

}
