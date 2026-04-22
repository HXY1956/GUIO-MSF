#include "hwa_msf_client.h"
using namespace hwa_set;
using namespace hwa_gnss;
using namespace hwa_uwb;
using namespace hwa_vis;
using namespace std;

namespace hwa_msf {
    msf_client::msf_client(std::string site, std::string site_base, base_time _beg, base_time _end, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* data) : _spdlog(spdlog)
    {
        int cam_number = dynamic_cast<set_vis*>(gset.get())->num_of_cam_group();
        baseworker = baseprocesser(gset, spdlog, site, _beg, _end);
        insworker = std::make_unique<insprocesser>(baseworker, data->operator[](base_data::ID_TYPE::IMUDATA));
        uwbworker = std::make_unique<uwbprocesser>(baseworker, data->operator[](base_data::ID_TYPE::UWBDATA));
        lidarworker = std::make_unique<lidarprocesser>(baseworker, data->operator[](base_data::ID_TYPE::LIDARDATA));
        gnssworker = std::make_unique<gnssprocesser>(baseworker, site, site_base, gset, spdlog, data);
        trackworker = std::make_unique<trackprocesser>(baseworker);
        for (int i = 0; i < cam_number; i++) {
            visworker[i] = std::make_unique<visprocesser>(baseworker, i, data->operator[](base_data::ID_TYPE::CAMDATA));
        }
        _ign_type = dynamic_cast<set_ign*>(gset.get())->_ign_type_();
        startenv = str2startenv(dynamic_cast<set_ign*>(gset.get())->start_env());
        align_type = dynamic_cast<set_ign*>(gset.get())->align_type();
        _aligned = align_type == NONE ? true : false;
        UseGnss = dynamic_cast<set_ign*>(gset.get())->GNSS() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::GUVI_TCI || _ign_type == IGN_TYPE::GVI_TCI || _ign_type == IGN_TYPE::GI_TCI || _ign_type == IGN_TYPE::GI_LCI || _ign_type == IGN_TYPE::GLVI_TCI || _ign_type == IGN_TYPE::GLI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseUwb = dynamic_cast<set_ign*>(gset.get())->UWB() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::UVI_TCI || _ign_type == IGN_TYPE::UI_LCI || _ign_type == IGN_TYPE::UI_TCI || _ign_type == IGN_TYPE::ULI_TCI || _ign_type == IGN_TYPE::ULVI_TCI || _ign_type == IGN_TYPE::GUI_TCI || _ign_type == IGN_TYPE::GUVI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseVis = dynamic_cast<set_ign*>(gset.get())->VISION() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::VIO_TCI || _ign_type == IGN_TYPE::VIO_LCI || _ign_type == IGN_TYPE::UVI_TCI || _ign_type == IGN_TYPE::ULVI_TCI || _ign_type == IGN_TYPE::GVI_TCI || _ign_type == IGN_TYPE::GUVI_TCI || _ign_type == IGN_TYPE::GLVI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
		UseLidar = dynamic_cast<set_ign*>(gset.get())->LIDAR() && (_ign_type == IGN_TYPE::IGN_DEFAULT || _ign_type == IGN_TYPE::LIO_TCI || _ign_type == IGN_TYPE::LVI_TCI || _ign_type == IGN_TYPE::ULI_TCI || _ign_type == IGN_TYPE::ULVI_TCI || _ign_type == IGN_TYPE::GLVI_TCI || _ign_type == IGN_TYPE::GLI_TCI || _ign_type == IGN_TYPE::GULVI_TCI);
        UseOdo = dynamic_cast<set_ign*>(gset.get())->Odo();
        UseNhc = dynamic_cast<set_ign*>(gset.get())->NHC();
        UseZupt = dynamic_cast<set_ign*>(gset.get())->ZUPT();
        UseHgt = dynamic_cast<set_ign*>(gset.get())->Hgt();
    }

    int msf_client::ProcessBatchFB()
    {
		TicToc t_total;
        if (!this->msf_client::_init()) {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "gipn_client", "init failed");
            return -1;
        }

        this->PreTimeSynchronization();

        while (true)
        {
            if (insworker->Time() > insworker->_end()) break;
            insworker->load_data();
            if (!this->align_process()) continue;
            if (initial_merge) this->merge_init();
            insworker->ProcessOneEpoch();
            if (!this->_getMeas()) continue;
            for (auto it = _Meas_Type.begin(); it != _Meas_Type.end(); it++)
            {
                switch (*it)
                {
                case GNSS_MEAS:
                {
                    TicToc t_gnss;
                    irc = gnssworker->ProcessOneEpoch();
                    std::cout << gnssworker->dTime() << " GNSS SPENT: " << t_gnss.toc() << "\n";
                    break;
                }

                case VIS_MEAS:
                {
                    TicToc t_vis;
                    irc = visworker[0]->ProcessOneEpoch();
                    std::cout << visworker[0]->dTime() << " VIS SPENT: " << t_vis.toc() << "\n";
                    break;
                }
                case UWB_MEAS:
                {
                    TicToc t_uwb;
                    irc = uwbworker->ProcessOneEpoch();
                    std::cout << uwbworker->dTime() << " UWB SPENT: " << t_uwb.toc() << "\n";
                    break;
                }
                case LIDAR_MEAS:
                {
                    TicToc t_lidar;
                    irc = lidarworker->ProcessOneEpoch();
                    std::cout << lidarworker->dTime() << " LIDAR SPENT: " << t_lidar.toc() << "\n";
                    break;
                }
                default:
                    break;
                }
                if (irc != NO_MEAS) this->crt_feed_back();
            }

            if (_Meas_Type.size()) {
                gnssworker->_prt_port(insworker->Time());
                insworker->UpdateViewer();
            }

            if (insworker->dsec() < insworker->_delay()) {
                visworker[0]->_write_calib();
                this->write2file();
            }
        }
        std::cout << "Total SPENT: " << t_total.toc() << "\n";
        return 1;
    }

    int msf_client::_init()
    {
        if ((UseIns && !insworker->_init()) || (UseGnss && !gnssworker->_init()) || (UseUwb && !uwbworker->_init()))
        {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, string("t_gipn_client:  ") + "initialize ipn_client filter failed!!! ");
            return -1;
        }
        return 1;
    }

    void msf_client::PreTimeSynchronization() {
        if (UseIns) {
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
                if (UseGnss) {
                    gnssworker->timesynchronization(insworker->Time());
                }
                break;
            case OUTDOOR:
                if (UseGnss) {
                    if (insworker->_beg() < gnssworker->_beg()) {
                        insworker->erase_bef(gnssworker->_beg());
                    }
                    else
                    {
                        gnssworker->gnss_proc_pvtflt::processBatch(gnssworker->Time(), insworker->Time(), false);
                        gnssworker->timesynchronization(insworker->Time());
                    }
                }
                if (UseUwb) {
                    uwbworker->timesynchronization(insworker->Time());
                }
                break;
            }
        }
    }

    bool msf_client::align_process() {
        if (!_aligned)
        {
            Flag = NO_MEAS;

            if (UseGnss && gnssworker->_time_valid(insworker->Time()) && gnssworker->load_data())
                Flag = gnssworker->_getPOS(gnssworker->Time(), posdata, measinfo);

            if (UseUwb && ((UseGnss && Flag == NO_MEAS) || !UseGnss) && uwbworker->_time_valid(insworker->Time()) && uwbworker->load_data())
                Flag = uwbworker->_getPOS(insworker->Time(), posdata, measinfo);

            _aligned = cascaded_align(posdata.pos, posdata.vn, Flag);
        }
        return _aligned;
    }

    bool msf_client::cascaded_align(Triple pos, Triple vel, MEAS_TYPE _Flag)
    {
        insworker->MeasCrt();
        Eigen::Vector3d blh = Cart2Geod(pos, false);
        Eigen::Vector3d vn = Cen(blh).transpose() * vel;
        insworker->set_posvel(blh, vn);

        bool ok = false;

        if (align_type == hwa_ins::VINS) {
            ok = visworker[0]->align_vins();
        }
        else if (align_type == hwa_ins::TRACK) {
            ok = trackworker->align_track();
        }
        else if (align_type == STC_AGN)
        {
            ok = insworker->align_coarse();
        }
        else if (align_type == VEL_AGN && _Flag != NO_MEAS)
        {
            if (SQRT(SQR(vn(0)) + SQR(vn(1))) > 2)
                ok = insworker->align_vva(vn);
        }
        else if (align_type == POS_AGN && _Flag != NO_MEAS)
        {
            ok = insworker->align_pva(pos);
        }
        if (ok) {
            std::cerr << "Alignment finished successfully" << std::endl;
        }

        return ok;
    }

    bool msf_client::_getMeas()
    {
        _Meas_Type.clear(); Flag = NO_MEAS; visworker[0]->load_imuobs();
        double ins_crt = insworker->Time().sow() + insworker->Time().dsec();
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
        if (double_eq(fabs(ins_crt - int(ins_crt)), 0.001)) {
            if (UseNhc && meas_type == NHC_MEAS)
                _Meas_Type.insert(NHC_MEAS);
            if (UseZupt && meas_type == ZUPT_MEAS)
            {
                _Meas_Type.insert(MEAS_TYPE::ZUPT_MEAS);
                _Meas_Type.insert(MEAS_TYPE::YAW_MEAS);
            }
        }

        if (double_eq(fabs(ins_crt - int(ins_crt)), 0.001) && insworker->MimuMeas())
        {
            _Meas_Type.insert(MEAS_TYPE::R_MIMU_MEAS);
            _Meas_Type.insert(MEAS_TYPE::P_MIMU_MEAS);
        }

        return _Meas_Type.size() > 0;
    }

    void msf_client::merge_init() {
        if (UseGnss && startenv == OUTDOOR)
            insworker->merge_init(gnssworker->get_site_pos(), gnssworker->get_lever(), gnssworker->_get_Pk(), GNSS);

        if (UseUwb && startenv == INDOOR)
            insworker->merge_init(uwbworker->get_site_pos(), uwbworker->get_lever(), uwbworker->_get_Pk(), UWB);
        initial_merge = false;
    };

    void msf_client::write2file() {
        std::set<std::string> ambs = gnssworker->ambs_name();
        int nsat = ambs.size();

        std::string amb = "Float";
        if (gnssworker->get_amb_state()) amb = "Fixed";

        std::string meas = "INS";
        if (_Meas_Type.size()) meas = meas2str(*_Meas_Type.begin());

        double pdop = 99.0;
        if (meas == "UWB") pdop = uwbworker->get_pdop();
        else if (meas == "GNSS") pdop = gnssworker->get_pdop();
        if (pdop > 100 || std::isnan(pdop)) pdop = 99;

        std::ostringstream os;
        gnssworker->_prt_ins_kml(insworker->Time());
        insworker->prt_sins(os);

        os << fixed << setprecision(0)
            << " " << setw(10) << meas            // meas
            << " " << setw(5) << nsat     // nsat
            << " " << setw(5) << uwbworker->get_anchor_number()
            << fixed << setprecision(2)
            << " " << setw(8) << pdop     // pdop
            << " " << setw(8) << amb
            << setw(10) << (gnssworker->get_amb_state() ? gnssworker->get_ratio() : 0.0);
        os << endl;
        insworker->write_sins(os);
        os.str("");
    }

    void msf_client::crt_feed_back() {
        if (UseGnss) gnssworker->_feed_back();
        if (UseUwb) uwbworker->_feed_back();
        if (UseVis) visworker[0]->_feed_back();
        if (UseLidar) lidarworker->_feed_back();
        if (UseIns) insworker->_feed_back();
    }
}
