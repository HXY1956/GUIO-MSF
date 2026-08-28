#include "hwa_fgo_client.h"
using namespace hwa_set;
using namespace hwa_gnss;
using namespace hwa_uwb;
using namespace hwa_vis;
using namespace std;

namespace hwa_fgo {
    fgo_client::fgo_client(std::string site, std::string site_base, base_time _beg, base_time _end, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* data) : _spdlog(spdlog)
    {
        _ign_type = dynamic_cast<set_ign*>(gset.get())->_ign_type_();
        startenv = str2startenv(dynamic_cast<set_ign*>(gset.get())->start_env());
        align_type = dynamic_cast<set_ign*>(gset.get())->align_type();
        _aligned = align_type == NONE ? true : false;
        if(_aligned) insworker->set_solver_flag(NON_LINEAR);
        UseGnss = dynamic_cast<set_ign*>(gset.get())->GNSS() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::GUVI_TCI || _ign_type == IGN_TYPE::GVI_TCI || _ign_type == IGN_TYPE::GI_TCI || _ign_type == IGN_TYPE::GI_LCI || _ign_type == IGN_TYPE::GLVI_TCI || _ign_type == IGN_TYPE::GLI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseUwb = dynamic_cast<set_ign*>(gset.get())->UWB() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::UVI_TCI || _ign_type == IGN_TYPE::UI_LCI || _ign_type == IGN_TYPE::UI_TCI || _ign_type == IGN_TYPE::ULI_TCI || _ign_type == IGN_TYPE::ULVI_TCI || _ign_type == IGN_TYPE::GUI_TCI || _ign_type == IGN_TYPE::GUVI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseVis = dynamic_cast<set_ign*>(gset.get())->VISION() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::VIO_TCI || _ign_type == IGN_TYPE::VIO_LCI || _ign_type == IGN_TYPE::UVI_TCI || _ign_type == IGN_TYPE::ULVI_TCI || _ign_type == IGN_TYPE::GVI_TCI || _ign_type == IGN_TYPE::GUVI_TCI || _ign_type == IGN_TYPE::GLVI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseLidar = dynamic_cast<set_ign*>(gset.get())->LIDAR() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::LIO_TCI || _ign_type == IGN_TYPE::LVI_TCI || _ign_type == IGN_TYPE::ULI_TCI || _ign_type == IGN_TYPE::ULVI_TCI || _ign_type == IGN_TYPE::GLVI_TCI || _ign_type == IGN_TYPE::GLI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseOdo = dynamic_cast<set_ign*>(gset.get())->Odo();
        UseNhc = dynamic_cast<set_ign*>(gset.get())->NHC();
        UseZupt = dynamic_cast<set_ign*>(gset.get())->ZUPT();
        UseHgt = dynamic_cast<set_ign*>(gset.get())->Hgt();

        baseworker = baseprocesser(gset, spdlog, site, _beg, _end);

        margworker = std::make_unique<margprocesser>(baseworker);
		all_workers.push_back(margworker.get());

        if (UseIns) {
            insworker = std::make_unique<insprocesser>(baseworker, data->operator[](base_data::ID_TYPE::IMUDATA));
            all_workers.push_back(insworker.get());
        }

        if (UseUwb) {
            uwbworker = std::make_unique<uwbprocesser>(baseworker, data->operator[](base_data::ID_TYPE::UWBDATA));
            all_workers.push_back(uwbworker.get());
        }

        if (UseLidar) {
            lidarworker = std::make_unique<lidarprocesser>(baseworker, data->operator[](base_data::ID_TYPE::LIDARDATA));
            all_workers.push_back(lidarworker.get());
        }
            
        if (UseGnss) {
            gnssworker = std::make_unique<gnssprocesser>(baseworker, site, site_base, gset, spdlog, data);
            all_workers.push_back(gnssworker.get());
        }
            
        if(UseVis)
            for (int i = 0; i < dynamic_cast<set_vis*>(gset.get())->num_of_cam_group(); i++) {
                visworker[i] = std::make_unique<visprocesser>(baseworker, i, data->operator[](base_data::ID_TYPE::CAMDATA));
                all_workers.push_back(visworker[i].get());
            }

        if (UseZupt) {
            zuptworker = std::make_unique<zuptprocesser>(baseworker);
            all_workers.push_back(zuptworker.get());
        }

        if (UseNhc) {
            nhcworker = std::make_unique<nhcprocesser>(baseworker);
            all_workers.push_back(nhcworker.get());
        }
    }

    int fgo_client::ProcessBatchFB()
    {
		TicToc t_total;
        if (!this->fgo_client::_init()) {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "gipn_client", "init failed");
            return -1;
        }

        this->PreTimeSynchronization();

        while (true)
        {
            if (insworker->Time() > insworker->_end()) 
                break;

            insworker->load_data();

            if (!_aligned) {
                _aligned = align_process();
            }

            if (_aligned && initial_merge)
                merge_init();

            insworker->ProcessOneEpoch();

            if (!_getMeas()) 
                continue;

            for (auto it = _Meas_Type.begin(); it != _Meas_Type.end(); it++)
            {
                switch (*it)
                {
                case GNSS_MEAS:
                {
                    TicToc t_gnss;
                    irc = gnssworker->ProcessOneEpoch();
                    double time = gnssworker->dTime();
                    std::cout << std::fixed << std::setprecision(3) << time << " GNSS SPENT: " << t_gnss.toc() << "\n";
                    break;
                }

                case VIS_MEAS:
                {
                    TicToc t_vis;
                    irc = visworker[0]->ProcessOneEpoch();
                    double time = visworker[0]->dTime();
                    if(irc == VIS_MEAS) 
                        std::cout << std::fixed << std::setprecision(3) << time << "[KEY FRAME] VIS SPENT: " << t_vis.toc() << "\n";
                    else
                        std::cout << std::fixed << std::setprecision(3) << time << " VIS SPENT: " << t_vis.toc() << "\n";
                    break;
                }
                case UWB_MEAS:
                {
                    TicToc t_uwb;
                    irc = uwbworker->ProcessOneEpoch();
                    double time = uwbworker->dTime();
                    std::cout << std::fixed << std::setprecision(3) << time << " UWB SPENT: " << t_uwb.toc() << "\n";
                    break;
                }
                case LIDAR_MEAS:
                {
                    TicToc t_lidar;
                    irc = lidarworker->ProcessOneEpoch();
                    double time = lidarworker->dTime();
                    std::cout << std::fixed << std::setprecision(3) << time << " LIDAR SPENT: " << t_lidar.toc() << "\n";
                    break;
                }
                default:
                    break;
                }

                if (std::next(it) == _Meas_Type.end() && new_node() && _time_to_margin()) {
                    
                    if (!_aligned) {
                        this->slide_window();
                        break;
                    }

                    TicToc t_opt;

                    if (UseGnss && gnssworker->is_last_node())
                        this->optimization_with_poterior();
                    else
                        this->optimization();
                    TicToc t_marg;
                    this->marginalizaiton();
                    this->slide_window();
                    this->feed_back();
                    this->reset();

                    std::cout << "Total SPENT: " << t_opt.toc() << "\n";
                }
            }

            if (!_aligned)
                continue;

            if (_Meas_Type.size()) {

                if (UseGnss)
                    gnssworker->_prt_port(insworker->Time());

                insworker->UpdateViewer();

                double percent = insworker->Time().diff(insworker->_beg()) / insworker->_end().diff(insworker->_beg()) * 100.0;
                cerr << "\r" << insworker->Time().str_ymdhms("Processing Epoch: ") << " Meas = " << meas2str(*_Meas_Type.begin()) << fixed << setprecision(1) << setw(6) << percent << "%";
            }
            if (insworker->dsec() < insworker->_delay()) {

                if(UseVis)
                    visworker[0]->_write_calib();

                this->write2file();
            }
        }

        std::cout << "Total SPENT: " << t_total.toc() << "\n";

        return 1;
    }

    void fgo_client::_vector_to_double(){
        for (auto worker : all_workers) {
			worker->_fgo_vector_to_double();
        }
    }

    void fgo_client::_double_to_vector() {
        for (auto worker : all_workers) {
            worker->_fgo_double_to_vector();
        }
    }

    void fgo_client::optimization_with_poterior() {

        std::pair<string, int>  outlier = make_pair(" ", -1);

        do
        {
             _vector_to_double();
             //prtState();
             double cost = 0;
            ceres::Problem problem;
            gnssworker->_remove_outlier_sat(outlier);

            for (auto worker : all_workers) {
                worker->_addResidualBlocks(problem);
            }

            ceres::Solver::Options options;
            options.minimizer_progress_to_stdout = true;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.trust_region_strategy_type = ceres::DOGLEG;
            options.max_num_iterations = 5;
            //options.num_threads = std::thread::hardware_concurrency();
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << endl;

            gnssworker->_posteriori_test(problem);

        } while (gnssworker->_gobs_outlier_detection(outlier) >= 0);

        gnssworker->_gnss_amb_resolution();
        _double_to_vector();
        isGNSSUpdate = true;
    }

    void fgo_client::optimization() {

        ceres::Problem problem;

        _vector_to_double();
        //prtState();

        for(auto worker : all_workers) {
            worker->_addResidualBlocks(problem);
        }
		TicToc t_opt;
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_solver_time_in_seconds = 0.05;
        options.max_num_iterations = 8;
        //options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        //options.use_nonmonotonic_steps = false;
        //options.min_trust_region_radius = options.max_trust_region_radius = 1e6;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << endl;
		std::cout << "Optimization SPENT: " << t_opt.toc() << "\n";

        _double_to_vector();
        isGNSSUpdate = false;
    }

    void fgo_client::marginalizaiton() {

        margworker->reset();

        for (auto worker : all_workers) {
            worker->_addMarginInfo();
        }

		margworker->ProcessOneEpoch();
    }

    void fgo_client::slide_window() {
        for (auto worker : all_workers) {
            worker->slide_window();
        }
    }
}

namespace hwa_fgo {
    int fgo_client::_init()
    {
        if ((UseIns && !insworker->_init()) || (UseGnss && !gnssworker->_init()) || (UseUwb && !uwbworker->_init()))
        {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, std::string("t_gipn_client:  ") + "initialize ipn_client filter failed!!! ");
            return -1;
        }
        return 1;
    }

    void fgo_client::PreTimeSynchronization() {

        switch (startenv) {

        case INDOOR:

            if (UseUwb) {

                if (insworker->_beg() < uwbworker->_beg()) {
                    insworker->erase_bef(uwbworker->_beg());
                }
                else
                {
                    uwbworker->ProcessBatch(uwbworker->Time(), insworker->Time());
                    uwbworker->timesynchronization(insworker->Time());
                }
            }

            if (UseGnss)
                gnssworker->timesynchronization(insworker->Time());

            break;

        case OUTDOOR:

            if (UseGnss) {

                if (insworker->_beg() < gnssworker->_beg()) {
                    insworker->erase_bef(gnssworker->_beg());
                    gnssworker->set_obs(gnssworker->_beg(), gnssworker->_end());
                }
                else
                {
                    gnssworker->gnss_proc_pvtflt::processBatch(gnssworker->Time(), insworker->Time(), false);
                    gnssworker->timesynchronization(insworker->Time());
                }
            }

            if (UseUwb)
                uwbworker->timesynchronization(insworker->Time());

            break;
        }
    }

    bool fgo_client::align_process() {

        Flag = NO_MEAS;

        if (UseIns)
            insworker->_getPOS(posdata);

        if (UseGnss && gnssworker->_time_valid(insworker->Time()) && gnssworker->load_data())
            Flag = gnssworker->_getPOS(insworker->Time(), posdata, measinfo);

        if (UseUwb && ((UseGnss && Flag == NO_MEAS) || !UseGnss) && uwbworker->_time_valid(insworker->Time()) && uwbworker->load_data())
            Flag = uwbworker->_getPOS(insworker->Time(), posdata, measinfo);

        return cascaded_align(posdata.pos, posdata.vn, Flag);
    }

    bool fgo_client::cascaded_align(Triple pos, Triple vel, MEAS_TYPE _Flag)
    {
        insworker->MeasCrt();

        Eigen::Vector3d blh = Cart2Geod(pos, false);

        Eigen::Vector3d vn = vel;

        insworker->set_posvel(blh, vn);

        bool ok = false;

        if (align_type == hwa_ins::VINS) {
            ok = visworker[0]->align_vins();
        }

        else if (align_type == hwa_ins::TRACK) {
            ok = trackworker->align_track();
        }

        else if (align_type == STC_AGN) {
            ok = insworker->align_coarse();
        }

        else if (align_type == VEL_AGN && _Flag != NO_MEAS) {
            if (SQRT(SQR(vn(0)) + SQR(vn(1))) > 2)
                ok = insworker->align_vva(vn);
        }
        else if (align_type == POS_AGN && _Flag != NO_MEAS) {
            ok = insworker->align_pva(pos);
        }

        if (ok) {
            std::cerr << "Alignment finished successfully" << std::endl;
            std::cerr << "TimeStamp: " << insworker->Time().sow() + insworker->Time().dsec() << "\n";
            insworker->_aligned = true;
            insworker->set_solver_flag(NON_LINEAR);
        }

        return ok;
    }

    bool fgo_client::_getMeas()
    {
        _Meas_Type.clear(); Flag = NO_MEAS;

        double ins_crt = insworker->Time().sow() + insworker->Time().dsec();

        if (UseVis) visworker[0]->load_imuobs();

		//USED FOR VINS ALIGNMENT, ONLY VIS MEAS IS USED FOR ALIGNMENT
        if (baseworker._get_solver_flag() == INITIAL && align_type == hwa_ins::VINS) {
            if (UseVis && visworker[0]->_time_valid(insworker->Time()) && visworker[0]->load_data() && visworker[0]->timecheck()) {
                _Meas_Type.insert(MEAS_TYPE::VIS_MEAS);
            }
            return _Meas_Type.size() > 0;
        }

        if (UseGnss && gnssworker->_time_valid(insworker->Time()) && gnssworker->load_data() && gnssworker->timecheck()) {
            _Meas_Type.insert(MEAS_TYPE::GNSS_MEAS);
        }

        if (UseUwb && uwbworker->_time_valid(insworker->Time()) && uwbworker->load_data() && uwbworker->timecheck()) {
            _Meas_Type.insert(MEAS_TYPE::UWB_MEAS);
        }

        if (UseVis && visworker[0]->_time_valid(insworker->Time()) && visworker[0]->load_data() && visworker[0]->timecheck()) {
            _Meas_Type.insert(MEAS_TYPE::VIS_MEAS);
        }

        if (UseLidar && lidarworker->_time_valid(insworker->Time()) && lidarworker->load_data() && lidarworker->timecheck()) {
            _Meas_Type.insert(MEAS_TYPE::LIDAR_MEAS);
        }

        MEAS_TYPE meas_type = insworker->meas_state();
        if (double_eq(std::fabs(ins_crt - int(ins_crt)), 0.0))
        {
            if (UseZupt && meas_type == ZUPT_MEAS) {
                zuptworker->insert(insworker->Time());
            }
            else if (UseNhc && meas_type == NHC_MEAS) {
                nhcworker->insert(insworker->Time());
            }
            else
            {
                baseworker.motion_insert(insworker->Time(), MOTION_TYPE::m_default);
            }
        }

        if (double_eq(fabs(ins_crt - int(ins_crt)), 0.001) && insworker->MimuMeas())
        {
            _Meas_Type.insert(MEAS_TYPE::R_MIMU_MEAS);
            _Meas_Type.insert(MEAS_TYPE::P_MIMU_MEAS);
        }

        return _Meas_Type.size() > 0;
    }

    void fgo_client::merge_init() {

        if (UseGnss && startenv == OUTDOOR)
            insworker->merge_init(gnssworker->get_site_pos(), gnssworker->get_lever());

        if (UseUwb && startenv == INDOOR)
            insworker->merge_init(uwbworker->get_site_pos(), uwbworker->get_lever());

        initial_merge = false;
    };

    void fgo_client::write2file() {
        std::string amb = "Float";
        double pdop = 99.0;
        int nsat = 0;
        int nanchor = 0;
        int ratio = 0;
        std::string meas = "INS";
        if (_Meas_Type.size()) meas = meas2str(*_Meas_Type.begin());

        if (UseGnss && meas == "GNSS") {
            std::set<std::string> ambs = gnssworker->ambs_name();
            nsat = ambs.size();
            if (gnssworker->get_amb_state()) amb = "Fixed";
            gnssworker->_prt_ins_kml(insworker->Time());
            pdop = gnssworker->get_pdop();
            ratio = gnssworker->get_amb_state() ? gnssworker->get_ratio() : 0.0;
        }
        if (UseUwb && meas == "UWB") {
            pdop = uwbworker->get_pdop();
            nanchor = uwbworker->get_anchor_number();
        }
        if (pdop > 100 || std::isnan(pdop)) pdop = 99;

        std::ostringstream os;
        insworker->prt_sins(os);

        os << fixed << setprecision(0)
            << " " << setw(10) << meas            // meas
            << " " << setw(5) << nsat     // nsat
            << " " << setw(5) << nanchor
            << fixed << setprecision(2)
            << " " << setw(8) << pdop     // pdop
            << " " << setw(8) << amb
            << setw(10) << ratio;
        os << endl;
        insworker->write_sins(os);
        os.str("");
    }
}
