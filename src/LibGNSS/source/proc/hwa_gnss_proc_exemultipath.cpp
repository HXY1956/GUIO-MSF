#include "hwa_gnss_proc_Exemultipath.h"

namespace hwa_gnss
{
    gnss_proc_EXEMULTIPATH::gnss_proc_EXEMULTIPATH(std::string site, set_base* set, base_log spdlog,  base_all_proc* gdata) :
        _set(set),
        _spdlog(spdlog),
        _crt_rec(site)
    {
        _initCommonProcess();
        _allobs = dynamic_cast<gnss_all_obs*>((*gdata)[base_data::ALLOBS]);
        _allnav = dynamic_cast<gnss_all_nav*>((*gdata)[base_data::GRP_EPHEM]);
        _allobj = dynamic_cast<gnss_all_obj*>((*gdata)[base_data::ALLOBJ]);
        _gturboedit = std::make_shared<gnss_proc_exeturboedit>(set, spdlog, gdata,_crt_rec);
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
        _obscomb = dynamic_cast<set_gproc*>(_set)->obs_combin();
        // Get site coordinates from XML/SNX files
        for (const auto& it : _rec_list)
        {
            _rec_crds[it] = dynamic_cast<set_rec*>(_set)->get_crd_xyz(it);
            if (_rec_crds[it].isZero() && _allobj)
            { // SNX
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
    }

    gnss_proc_EXEMULTIPATH::~gnss_proc_EXEMULTIPATH()
    {
        for (auto it : _rec_list)
        {
            if (_sppflt[it]) { delete _sppflt[it]; _sppflt[it] = nullptr; }
        }

        if (_multipathfile)
        {
            if (_multipathfile->is_open()) _multipathfile->close();
            delete _multipathfile;
            _multipathfile = nullptr;
        }
        if (_cycleslipflagfile)
        {
            if (_cycleslipflagfile->is_open()) _cycleslipflagfile->close();
            delete _cycleslipflagfile;
            _cycleslipflagfile = nullptr;
        }
        if (_noisefile)
        {
            if (_noisefile->is_open()) _noisefile->close();
            delete _noisefile;
            _noisefile = nullptr;
        }
    }

    void gnss_proc_EXEMULTIPATH::_initCommonProcess()
    {
        _interval = dynamic_cast<set_gen*>(_set)->sampling();
        _beg_time = dynamic_cast<set_gen*>(_set)->beg();
        _end_time = dynamic_cast<set_gen*>(_set)->end();
        _sys_list = dynamic_cast<set_gen*>(_set)->sys();
        _rec_list = dynamic_cast<set_gen*>(_set)->recs();
        _site_kinematic = dynamic_cast<set_gproc*>(_set)->pos_kin();
        _obscomb = dynamic_cast<set_gproc*>(_set)->obs_combin();
        _minimum_elev = dynamic_cast<set_gproc*>(_set)->minimum_elev();
        _rec_list_base = dynamic_cast<set_gen*>(_set)->list_base();
        if (_rec_list_base.empty())
            _site_base = "";
        else
            _site_base = _rec_list_base[0];     ///< all base is the first base
        for (auto it : _sys_list)
        {
            GSYS gsys = gnss_sys::str2gsys(it);
            std::set<std::string> sat_temp = dynamic_cast<set_gnss*>(_set)->sat(gsys);
            _sat_list.insert(sat_temp.begin(), sat_temp.end());
            _band_index[gsys] = dynamic_cast<set_gnss*>(_set)->band_index(gsys);
        }
        std::set<std::string> sat_rm = dynamic_cast<set_gen*>(_set)->sat_rm();

        for (auto iter : sat_rm)
        {
            _sat_list.erase(iter);
        }

        std::string savepath = dynamic_cast<set_out*>(_set)->outputs("obsquality_dir");
        if (!savepath.empty())
        {
            std::string site_tmp = _crt_rec;
            std::stringstream tmp_file;
            savepath += site_tmp;
            if (ACCESS(savepath.substr(7).c_str(), 0) != 0)
            {
                MKDIR(savepath.substr(7).c_str());
            }
            transform(site_tmp.begin(), site_tmp.end(), site_tmp.begin(), ::tolower);
            tmp_file << savepath << PATH_SEPARATOR << site_tmp << "-mp.txt";
            _multipathfile = new base_iof;
            _multipathfile->tsys(base_time::GPS);
            _multipathfile->mask(tmp_file.str());
            tmp_file.str("");
            tmp_file << savepath << PATH_SEPARATOR << site_tmp << "-tbflag.txt";
            _cycleslipflagfile = new base_iof;
            _cycleslipflagfile->tsys(base_time::GPS);
            _cycleslipflagfile->mask(tmp_file.str());
            tmp_file.str("");
            tmp_file << savepath << PATH_SEPARATOR << site_tmp << "-noise.txt";
            _noisefile = new base_iof;
            _noisefile->tsys(base_time::GPS);
            _noisefile->mask(tmp_file.str());
        }
    }

    void gnss_proc_EXEMULTIPATH::_add_rho_azel(const std::string& site_name, Triple& xyz_s, const Triple& xyz_r, gnss_data_sats& obs_sat)
    { // TODO
        Triple xyz_rho = xyz_s - xyz_r;
        Triple ell_r, neu_s;
        xyz2ell(xyz_r, ell_r, false);
        xyz2neu(ell_r, xyz_rho, neu_s);
        // Earth rotation correction
        Triple xRec;
        double rho0 = (xyz_r - xyz_s).norm();
        double dPhi = OMEGA * rho0 / CLIGHT;
        xRec[0] = xyz_r[0] * cos(dPhi) - xyz_r[1] * sin(dPhi);
        xRec[1] = xyz_r[1] * cos(dPhi) + xyz_r[0] * sin(dPhi);
        xRec[2] = xyz_r[2];

        double tmp = (xyz_s - xRec).norm();
        obs_sat.addrho(tmp);
        double NE2 = neu_s[0] * neu_s[0] + neu_s[1] * neu_s[1];
        double ele = acos(sqrt(NE2) / tmp);
        if (neu_s[2] < 0.0) { ele *= -1.0; }
        if (sqrt(NE2) / tmp > 1.0)
            obs_sat.addele(0.0);
        else obs_sat.addele(ele);

        double azi = atan2(neu_s[1], neu_s[0]);
        if (azi < 0) azi += 2 * hwa_pi;
        obs_sat.addazi_sat(azi);
        //if (!_use_ecl) obs_sat.addecl(_lastEcl);
        return;
    }

    bool gnss_proc_EXEMULTIPATH::_getSatInfo(std::string sat, double& distance, double& elev, double& sat_clk)
    {
        bool valid = false;
        double sat_crd[3];
        Triple xyz_s, site_sat_vector, xyz_r;
        double delay = 0.0;
        base_time sat_time;
        std::shared_ptr<gnss_data_eph> geph;
        xyz_r = _rec_crds[_crt_rec];
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

            xyz_s = Triple(sat_crd);
            if (xyz_r.isZero())
                break;

            site_sat_vector = xyz_s - xyz_r;
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
                /*elev = (site_crd_Triple[0] * site_sat_vector[0] + site_crd_Triple[1] * site_sat_vector[1] + site_crd_Triple[2] * site_sat_vector[2]) / site_crd_Triple.norm() / distance;
                elev = 90.0 - acos(elev) * 180.0 / hwa_pi;*/
                Triple xyz_rho = xyz_s - xyz_r;
                Triple ell_r, neu_s;
                xyz2ell(xyz_r, ell_r, false);
                xyz2neu(ell_r, xyz_rho, neu_s);
                // Earth rotation correction
                Triple xRec;
                double rho0 = (xyz_r - xyz_s).norm();
                double dPhi = OMEGA * rho0 / CLIGHT;
                xRec[0] = xyz_r[0] * cos(dPhi) - xyz_r[1] * sin(dPhi);
                xRec[1] = xyz_r[1] * cos(dPhi) + xyz_r[0] * sin(dPhi);
                xRec[2] = xyz_r[2];

                double tmp = (xyz_s - xRec).norm();
                double NE2 = neu_s[0] * neu_s[0] + neu_s[1] * neu_s[1];
                elev = acos(sqrt(NE2) / tmp) * 180.0 / hwa_pi;
                if (neu_s[2] < 0.0) { elev *= -1.0; }
                return true;
            }
        }

        distance = 0.0;
        sat_clk = 0.0, elev = 0.0;
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, sat + " lack of suitable broadcast ephemeris at Time " + _crt_time.str_ymdhms());
        return false;
    }
    
    bool gnss_proc_EXEMULTIPATH::_siteSPP(const std::string& site, const base_time& now)
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

    int gnss_proc_EXEMULTIPATH::_combineMP(gnss_data_sats& satdata)
    {
        std::string sat = satdata.sat();
        double obs_intv = _interval;
        double band_num;
        double MP1 = 0, MP2 = 0, MP3 = 0;
        bool slip_L1 = false, slip_L2 = false, slip_L3 = false, islip;
        GOBSBAND b1, b2, b3;
        std::vector < GOBSBAND > band = dynamic_cast<set_gnss*>(_set)->band(satdata.gsys());
        base_time crt_time = _crt_time;
        base_time pre_time = crt_time - obs_intv;
        if (crt_time == _beg_time || _mp_orig[pre_time].find(sat) == _mp_orig[pre_time].end())
        { // initialization the first-1 epoch
            _mp_orig[pre_time][sat][1] = 0.0;    // the number of epoch in an arc on band 1
            _mp_orig[pre_time][sat][2] = 0.0;    // the value of multipath on band 1
            _mp_orig[pre_time][sat][3] = 0.0;    // the number of epoch in an arc on band 2
            _mp_orig[pre_time][sat][4] = 0.0;    // the value of multipath on band 2
            _mp_orig[pre_time][sat][5] = 0.0;    // the number of epoch in an arc on band 3
            _mp_orig[pre_time][sat][6] = 0.0;    // the value of multipath on band 3
        }
        band_num = band.size();
        b1 = band[0];
        b2 = band[1];
        if (satdata.gsys() == BDS) b2 = sat > "C16" ? BAND_6 : BAND_7;  // BDS2: 27  BDS3: 26
        if (band_num > 2) { b3 = band[2]; }
        gnss_data_obs gobsP1 = gnss_data_obs(satdata.select_range(b1));
        gnss_data_obs gobsP2 = gnss_data_obs(satdata.select_range(b2));
        gnss_data_obs gobsL1 = gnss_data_obs(satdata.select_phase(b1));
        gnss_data_obs gobsL2 = gnss_data_obs(satdata.select_phase(b2));
        gnss_data_obs gobsP3 = gnss_data_obs(satdata.select_range(b3));
        gnss_data_obs gobsL3 = gnss_data_obs(satdata.select_phase(b3));
        
        if (gobsP1.valid() && gobsL1.valid()) slip_L1 = satdata.getlli(gobsL1.gobs()) >= 1;
        if (gobsP1.valid() && gobsL1.valid() && gobsL2.valid())
        {
            MP1 = satdata.MP(gobsP1, gobsL1, gobsL2);
        }
        if (gobsP2.valid() && gobsL2.valid()) slip_L2 = satdata.getlli(gobsL2.gobs()) >= 1;
        if (gobsP2.valid() && gobsL2.valid() && gobsL1.valid())
        {
            MP2 = satdata.MP(gobsP2, gobsL2, gobsL1);
        }
        if (gobsP3.valid() && gobsL3.valid()) slip_L3 = satdata.getlli(gobsL3.gobs()) >= 1;
        if (gobsP3.valid() && gobsL3.valid() && gobsL1.valid())
        {
            MP3 = satdata.MP(gobsP3, gobsL3, gobsL1);
        }
        
        islip = slip_L1 || slip_L2;
        if (!islip)
        {   // very very compact!
            _mp_orig[crt_time][sat][2] = MP1;
            _mp_orig[crt_time][sat][1] = double_eq(MP1, 0.0) ? 0.0
                : _mp_orig[crt_time][sat][1] = _mp_orig[pre_time][sat][1] + 1;    // the orig index is zero, so...
            _mp_orig[crt_time][sat][4] = MP2;
            _mp_orig[crt_time][sat][3] = double_eq(MP2, 0.0) ? 0.0
                : _mp_orig[crt_time][sat][3] = _mp_orig[pre_time][sat][3] + 1;    // the orig index is zero, so...
        }
        else
        {
            _mp_orig[crt_time][sat][1] = 1;
            _mp_orig[crt_time][sat][2] = MP1;
            _mp_orig[crt_time][sat][3] = 1;
            _mp_orig[crt_time][sat][4] = MP2;
        }
        islip = slip_L1 || slip_L3;
        if (!islip)
        {
            _mp_orig[crt_time][sat][6] = MP3;
            _mp_orig[crt_time][sat][5] = double_eq(MP3, 0.0) ? 0.0
                : _mp_orig[crt_time][sat][5] = _mp_orig[pre_time][sat][5] + 1;
        }
        return 0;
    }

    int gnss_proc_EXEMULTIPATH::_prepareDataMP(const std::string& ssite, std::vector<gnss_data_sats>& sdata)
    {
        std::vector<gnss_data_sats>::iterator iter = sdata.begin();
        std::set<std::string> sat_rm = dynamic_cast<set_gen*>(_set)->sat_rm();
        while (iter != sdata.end()) 
        {
            // except sat from config file
            std::string satname = iter->sat();
            if (sat_rm.find(satname) != sat_rm.end()) 
            {
                iter = sdata.erase(iter);
                continue;
            }
            GSYS gs = iter->gsys();

            //GOBSBAND b1, b2;
            GOBSBAND b1 = _band_index[gs][FREQ_1];
            GOBSBAND b2 = _band_index[gs][FREQ_2];
            if (iter->gsys() == BDS) b2 = satname > "C16" ? BAND_6 : BAND_7;  // BDS2: 27  BDS3: 26
            iter->spdlog(_spdlog);
            // check data availability lvhb added in 20200525
            auto l1 = iter->select_phase(b1);
            auto l2 = iter->select_phase(b2);
            auto p1 = iter->select_range(b1);
            auto p2 = iter->select_range(b2);

            // check data availability
            if (p1 == X || l1 == X) 
            {
                iter = sdata.erase(iter);
                continue;
            }

            double P3, L3;// no use now, 20220518 hlgou
            if (_obscomb == OBSCOMBIN::RAW_SINGLE) 
            {
                P3 = iter->obs_C(p1);
                L3 = iter->obs_L(l1);
            }
            else if (_obscomb == OBSCOMBIN::IF_P1 || _obscomb == OBSCOMBIN::RAW_MIX)
            {
                if (p2 == X || l2 == X) 
                {
                    P3 = iter->obs_C(p1);
                    L3 = iter->obs_L(l1);
                }
                else 
                {
                    P3 = iter->P3(p1, p2);
                    L3 = iter->L3(l1, l2);
                }
            }
            else 
            {
                //xjhan
                if (p2 == X || l2 == X) 
                {
                    iter = sdata.erase(iter);
                    continue;
                }
                P3 = iter->P3(p1, p2);
                L3 = iter->L3(l1, l2);
            }

            if (_satPos(_crt_time, *iter) < 0)
            {
                if (_spdlog) 
                {
                    std::ostringstream str;
                    str << "prepareData: erasing data since _satPos failed, "
                        << "epo: " << _crt_time.str_hms() << ", " <<
                        "prn: " << iter->sat();
                    SPDLOG_LOGGER_ERROR(_spdlog, str.str());
                }
            }
            iter++;
        }
        //Compute sat elevation and rho, // no use now, 20220518 hlgou
        //Triple xyz_r=_rec_crds[_crt_rec];
        //if (xyz_r.isZero() && _spdlog)
        //{
        //    std::ostringstream str;
        //    str << "prepareData failed! Beacause receiver coordinate is zero, "
        //            << "epo: " << _crt_time.str_hms() << std::endl;
        //    SPDLOG_LOGGER_ERROR(_spdlog, str.str());
        //    return -1;
        //}
        //while (iter != sdata.end()) 
        //{
        //    Triple  xyz_s;
        //    xyz_s = iter->satcrd();
        //    _add_rho_azel(ssite, xyz_s, xyz_r, *iter);
        //    iter++;
        //}//end sdata
        return 0;
    }

    int gnss_proc_EXEMULTIPATH::_satPos(base_time& epo, gnss_data_sats& gsatdata)
    {
        std::string satname = gsatdata.sat();
        int i = 0;
        if (dynamic_cast<set_gproc*>(_set)->realtime())
            i = gsatdata.addprd_realtime(_allnav);
        else
            i = gsatdata.addprd(_allnav); 
        if (i < 0) return i;
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_outputMP()
    {
        _outputMP_header();
        std::ostringstream multipathInfo; multipathInfo.str("");
        std::ostringstream tempInfo; tempInfo.str("");
        base_time crt_time = _beg_time;
        int Gnums, Enums, Cnums, GECnums, Allnums;
        std::string prn = "";
        double mp1, mp2, mp3, ele;
        double snrp1, snrl1, snrb1, snrp2, snrl2, snrb2;
        int p1flag, l1flag, p2flag, l2flag;
        while (crt_time < _end_time || crt_time == _end_time)
        {
            std::vector<gnss_data_sats> obs_data = _allobs->obs(_crt_rec, crt_time);
            Gnums = Enums = Cnums = GECnums = Allnums = 0;
            Allnums = _epo_avaisatlist[crt_time].size();
            tempInfo.str("");
            if (obs_data.size() == 0)
            {
                crt_time.add_secs(int(_interval));
                continue;
            }
            for (auto iter=obs_data.begin(); iter!=obs_data.end(); iter++)
            {
                prn = iter->sat();
                ele = iter->gnss_data_obs_manager::getele() * R2D;
                if (_epo_avaisatlist[crt_time].find(prn) != _epo_avaisatlist[crt_time].end())
                {
                    ele = _epo_avaisatlist[crt_time][prn];
                }
                GOBSBAND b1 = _band_index[iter->gsys()][FREQ_1];
                GOBSBAND b2 = _band_index[iter->gsys()][FREQ_2];
                if (iter->gsys() == BDS) b2 = (prn > "C16") ? BAND_6 : BAND_7;  // BDS2: 27  BDS3: 26
                if (iter->gsys() == GPS) Gnums += 1;
                else if (iter->gsys() == GAL) Enums += 1;
                else if (iter->gsys() == BDS) Cnums += 1;
                GECnums += 1;

                auto obsL1 = iter->select_phase(b1); auto obsL2 = iter->select_phase(b2);
                auto obsP1 = iter->select_range(b1); auto obsP2 = iter->select_range(b2);
                p1flag = obsP1 == GOBS::X ? 0 : 1;
                l1flag = obsL1 == GOBS::X ? 0 : 1;
                p2flag = obsP2 == GOBS::X ? 0 : 1;
                l2flag = obsL2 == GOBS::X ? 0 : 1;
                snrp1 = iter->getobs(pl2snr(obsP1)); snrl1 = iter->getobs(pl2snr(obsL1));
                snrb1 = snrp1 > snrl1 ? snrp1 : snrl1;
                snrp2 = iter->getobs(pl2snr(obsP2)); snrl2 = iter->getobs(pl2snr(obsL2));
                snrb2 = snrp2 > snrl2 ? snrp2 : snrl2;
                mp1 = mp2 = mp3 = 0.0;
                if (_mp_avearc[crt_time].find(prn) != _mp_avearc[crt_time].end()) 
                { 
                    mp1 = _mp_avearc[crt_time][prn][2]; 
                    mp2 = _mp_avearc[crt_time][prn][4];
                    mp3 = _mp_avearc[crt_time][prn][6];
                }
                if (abs(mp1) > 50) mp1 = 0.0;
                if (abs(mp2) > 50) mp2 = 0.0;
                if (abs(mp3) > 50) mp3 = 0.0;
                tempInfo << prn << std::fixed << std::setw(7) << p1flag << l1flag << p2flag << l2flag
                    << std::setw(10) << std::setprecision(3) << ele
                    << std::setw(10) << std::setprecision(2) << snrb1 << std::setw(10) << std::setprecision(3) << mp1
                    << std::setw(10) << std::setprecision(2) << snrb2 << std::setw(10) << std::setprecision(3) << mp2 << std::endl;
            }
            
            multipathInfo << "> " << crt_time.mjd() << " " << crt_time.sod() << std::setw(10) << Allnums 
                << std::setw(10) << GECnums << std::setw(10) << Gnums << std::setw(10) << Enums << std::setw(10) << Cnums << std::endl << tempInfo.str();

            crt_time.add_secs(int(_interval));
        }
        if (_multipathfile)
        {
            _multipathfile->write(multipathInfo.str().c_str(), multipathInfo.str().size());
            _multipathfile->flush();
        }
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_outputMP_header()
    {
        std::ostringstream multipathInfo; multipathInfo.str("");
        std::string temp = "";
        multipathInfo << "# " << std::setw(10) << _crt_rec << std::setw(48) << "" << "MARKER NAME" << std::endl;
        multipathInfo << "# ";
        int leave, count = 0;
        for (auto sys : _sys_list)
        {
            multipathInfo << std::setw(8) << sys;
            count += 8;
        }
        leave = 58 - count;
        multipathInfo << std::setw(leave) << "" << "SYS" << std::endl;
        multipathInfo << "# MJD" << std::setw(8) << "SOD" << std::setw(8) << "IDEAR" << std::setw(8) << "ALL"
            << std::setw(8) << "GPS" << std::setw(8) << "GAL" << std::setw(8) << "BDS" << std::setw(7) << "" << "TIME / SATNUMS" << std::endl;
        multipathInfo << "# PRN" << std::setw(8) << "AVAI" << std::setw(8) << "ELE" << std::setw(8) << "F1S" 
            << std::setw(8) << "F1M" << std::setw(8) << "F2S" << std::setw(8) << "F2M" << std::setw(7) << "" << "DATA TYPES" << std::endl;
        multipathInfo << "# " << std::left << std::setw(18) << "F:Frequency" << std::setw(20) << "M:Multi-path" << std::setw(20) << "S:SNR" << "COMMENTS" << std::endl;
        multipathInfo << "# " << std::setw(58) << "" << "END OF HEADER" << std::endl;
        if (_multipathfile)
        {
            _multipathfile->write(multipathInfo.str().c_str(), multipathInfo.str().size());
            _multipathfile->flush();
        }
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_outputNoise_header()
    {
        std::ostringstream noiseInfo; noiseInfo.str("");
        std::string temp = "";
        noiseInfo << "# " << std::left << std::setw(10) << _crt_rec << std::setw(48) << "" << "MARKER NAME" << std::endl;
        noiseInfo << "# " << std::left << std::setw(10) << _site_base << std::setw(48) << "" << "BASE SITE NAME" << std::endl;
        noiseInfo << "# ";
        int leave, count = 0;
        for (auto sys : _sys_list)
        {
            noiseInfo << std::left << std::setw(8) << sys;
            count += 8;
        }
        leave = 58 - count;
        noiseInfo << std::setw(leave) << "" << "SYS" << std::endl;
        noiseInfo << "# MJD" << std::setw(8) << "SOD" << std::setw(8) << "IDEAR" << std::setw(8) << "ALL"
            << std::setw(8) << "GPS" << std::setw(8) << "GAL" << std::setw(8) << "BDS" << std::setw(7) << "" << "TIME / SATNUMS" << std::endl;
        noiseInfo << "# PRN" << std::setw(8) << "AVAI" << std::setw(8) << "ELE" << std::setw(8) << "F1S"
            << std::setw(8) << "F2S" << std::setw(8) << "F1M" << std::setw(8) << "F2M" << std::setw(7) << "" << "DATA TYPES" << std::endl;
        noiseInfo << "#    " << std::setw(8) << "F1PN" << std::setw(8) << "F2PN" << std::setw(8) << "F1LN"
            << std::setw(8) << "F2LN" << std::setw(8) << "F1DN" << std::setw(8) << "F2DN" << std::setw(7) << "" << "DATA TYPES" << std::endl;
        noiseInfo << "#    " << std::setw(8) << "F1LN" << std::setw(8) << "F2LN" << std::setw(8) << "F1DN"
            << std::setw(8) << "F2DN" << std::setw(8) << "" << std::setw(8) << "" << std::setw(7) << "" << "DATA TYPES of BASE SITE" << std::endl;
        noiseInfo << "# " << std::left << std::setw(18) << "F:Frequency" << std::setw(20) << "M:Multi-path" << std::setw(20) << "N:Noise" << "COMMENTS" << std::endl;
        noiseInfo << "# " << std::left << std::setw(18) << "P:Code" << std::setw(20) << "L:Phase" << std::setw(20) << "D:Doppler" << "COMMENTS" << std::endl;
        noiseInfo << "# " << std::setw(58) << "" << "END OF HEADER" << std::endl;
        if (_noisefile)
        {
            _noisefile->write(noiseInfo.str().c_str(), noiseInfo.str().size());
            _noisefile->flush();
        }
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_outputNoise()
    {
        _outputNoise_header();
        std::ostringstream noiseInfo; noiseInfo.str("");
        std::ostringstream tempInfo; tempInfo.str("");
        base_time crt_time = _beg_time;
        int Gnums, Enums, Cnums, GECnums, Allnums;
        std::string prn = "";
        double mp1, mp2, mp3, ele;
        double snrp1, snrl1, snrb1, snrp2, snrl2, snrb2;
        double noisep1, noisep2, noisel1, noisel2, noised1, noised2;
        double noisel1_base, noisel2_base, noised1_base, noised2_base;
        int p1flag, l1flag, p2flag, l2flag;
        while (crt_time < _end_time)
        {
            std::vector<gnss_data_sats> obs_data = _allobs->obs(_crt_rec, crt_time);
            Gnums = Enums = Cnums = GECnums = Allnums = 0;
            Allnums = _epo_avaisatlist[crt_time].size();
            tempInfo.str("");
            if (obs_data.size() == 0)
            {
                crt_time.add_secs(int(_interval));
                continue;
            }
            for (auto iter = obs_data.begin(); iter != obs_data.end(); iter++)
            {
                prn = iter->sat();
                ele = iter->gnss_data_obs_manager::getele() * R2D;
                if (_epo_avaisatlist[crt_time].find(prn) != _epo_avaisatlist[crt_time].end())
                {
                    ele = _epo_avaisatlist[crt_time][prn];
                }
                GOBSBAND b1 = _band_index[iter->gsys()][FREQ_1];
                GOBSBAND b2 = _band_index[iter->gsys()][FREQ_2];
                if (iter->gsys() == BDS) b2 = (prn > "C16") ? BAND_6 : BAND_7;  // BDS2: 27  BDS3: 26
                if (iter->gsys() == GPS) Gnums += 1;
                else if (iter->gsys() == GAL) Enums += 1;
                else if (iter->gsys() == BDS) Cnums += 1;
                GECnums += 1;

                auto obsL1 = iter->select_phase(b1); auto obsL2 = iter->select_phase(b2);
                auto obsP1 = iter->select_range(b1); auto obsP2 = iter->select_range(b2);
                p1flag = obsP1 == GOBS::X ? 0 : 1;
                l1flag = obsL1 == GOBS::X ? 0 : 1;
                p2flag = obsP2 == GOBS::X ? 0 : 1;
                l2flag = obsL2 == GOBS::X ? 0 : 1;
                snrp1 = iter->getobs(pl2snr(obsP1)); snrl1 = iter->getobs(pl2snr(obsL1));
                snrb1 = snrp1 > snrl1 ? snrp1 : snrl1;
                snrp2 = iter->getobs(pl2snr(obsP2)); snrl2 = iter->getobs(pl2snr(obsL2));
                snrb2 = snrp2 > snrl2 ? snrp2 : snrl2;
                mp1 = mp2 = mp3 = 0.0;
                if (_mp_avearc[crt_time].find(prn) != _mp_avearc[crt_time].end())
                {
                    mp1 = _mp_avearc[crt_time][prn][2];
                    mp2 = _mp_avearc[crt_time][prn][4];
                    mp3 = _mp_avearc[crt_time][prn][6];
                }
                // If the mp_value is too big, then std::set it zero.
                if (abs(mp1) > 50) mp1 = 0.0;
                if (abs(mp2) > 50) mp2 = 0.0;
                if (abs(mp3) > 50) mp3 = 0.0;

                noisep1 = noisep2 = noisel1 = noisel2 = noised1 = noised2 = 0;
                if (_noise_code[crt_time].find(prn) != _noise_code[crt_time].end())
                {
                    if (_noise_code[crt_time][prn].find(b1) != _noise_code[crt_time][prn].end())
                        noisep1 = _noise_code[crt_time][prn][b1][1];
                    if (_noise_code[crt_time][prn].find(b2) != _noise_code[crt_time][prn].end())
                        noisep2 = _noise_code[crt_time][prn][b2][1];
                }
                if (_noise_phase[crt_time].find(prn) != _noise_phase[crt_time].end())
                {
                    if (_noise_phase[crt_time][prn].find(b1) != _noise_phase[crt_time][prn].end())
                    {
                        noisel1 = _noise_phase[crt_time][prn][b1][1];
                        noised1 = _noise_phase[crt_time][prn][b1][2];
                    } 
                    if (_noise_phase[crt_time][prn].find(b2) != _noise_phase[crt_time][prn].end())
                    {
                        noisel2 = _noise_phase[crt_time][prn][b2][1];
                        noised2 = _noise_phase[crt_time][prn][b2][2];
                    }
                }
                noisel1_base = noisel2_base = noised1_base = noised2_base = 0;
                if (_noise_phase_base[crt_time].find(prn) != _noise_phase[crt_time].end())
                {   // base station noise.
                    if (_noise_phase_base[crt_time][prn].find(b1) != _noise_phase_base[crt_time][prn].end())
                    {
                        noisel1_base = _noise_phase_base[crt_time][prn][b1][1];
                        noised1_base = _noise_phase_base[crt_time][prn][b1][2];
                    }
                    if (_noise_phase_base[crt_time][prn].find(b2) != _noise_phase_base[crt_time][prn].end())
                    {
                        noisel2_base = _noise_phase_base[crt_time][prn][b2][1];
                        noised2_base = _noise_phase_base[crt_time][prn][b2][2];
                    }
                }

                tempInfo << prn << std::fixed << std::setw(7) << p1flag << l1flag << p2flag << l2flag
                    << std::setw(10) << std::setprecision(3) << ele << std::setw(10) << std::setprecision(2) << snrb1 << std::setw(10) << std::setprecision(2) << snrb2
                    << std::setw(10) << std::setprecision(3) << mp1 << std::setw(10) << std::setprecision(3) << mp2
                    << std::setw(10) << std::setprecision(3) << noisep1 << std::setw(10) << std::setprecision(3) << noisep2
                    << std::setw(10) << std::setprecision(3) << noisel1 << std::setw(10) << std::setprecision(3) << noisel2
                    << std::setw(10) << std::setprecision(3) << noised1 << std::setw(10) << std::setprecision(3) << noised2
                    << std::setw(10) << std::setprecision(3) << noisel1_base << std::setw(10) << std::setprecision(3) << noisel2_base
                    << std::setw(10) << std::setprecision(3) << noised1_base << std::setw(10) << std::setprecision(3) << noised2_base
                    << std::endl;
            }
            noiseInfo << "> " << crt_time.mjd() << " " << crt_time.sod() << std::setw(10) << Allnums
                << std::setw(10) << GECnums << std::setw(10) << Gnums << std::setw(10) << Enums << std::setw(10) << Cnums << std::endl << tempInfo.str();

            crt_time.add_secs(int(_interval));
        }
        if (_noisefile)
        {
            _noisefile->write(noiseInfo.str().c_str(), noiseInfo.str().size());
            _noisefile->flush();
        }
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_outputMP_test()
    {
        std::ostringstream multipathInfo; multipathInfo.str("");
        for (auto it = _mp_orig.begin(); it != _mp_orig.end(); it++)
        {
            base_time crt_time = it->first;
            multipathInfo << "> " << std::setw(6) << crt_time.sod() << std::endl;
            for (auto itsat = (it->second).begin(); itsat != (it->second).end(); itsat++)
            {
                std::string prn = itsat->first;
                if (_mp_orig[crt_time][prn][2] == 0 && _mp_orig[crt_time][prn][4] == 0 /*&& _MP3[t][prn][1] == 0*/) continue;
                multipathInfo << prn << std::fixed 
                    << std::setw(5) << std::setprecision(0) << _mp_orig[crt_time][prn][1] << std::setw(10) << std::setprecision(3) << _mp_orig[crt_time][prn][2]
                    << std::setw(5) << std::setprecision(0) << _mp_orig[crt_time][prn][3] << std::setw(10) << std::setprecision(3) << _mp_orig[crt_time][prn][4] << std::endl;
                //<< " " << _MP3[t][prn][1] << std::setw(15) << right << _MP3[t][prn][4] << " "
            }
        }
        if (_multipathfile)
        {
            _multipathfile->write(multipathInfo.str().c_str(), multipathInfo.str().size());
            _multipathfile->flush();
        }

        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_output_slip_flag()
    {
        std::ostringstream slipflagInfo; slipflagInfo.str("");
        base_time crt_time = _beg_time;
        while (crt_time < _end_time || crt_time == _end_time)
        {
            _data.erase(_data.begin(), _data.end());
            _data = _allobs->obs(_crt_rec, crt_time);
            if (_data.size() == 0)
            {
                crt_time.add_secs(int(_interval));
                continue;
            }
            slipflagInfo << "> " << crt_time.mjd() << "  " << crt_time.sod() << std::endl;
            for (auto it = _data.begin(); it != _data.end(); it++)
            {
                GOBSBAND b1, b2;
                std::vector < GOBSBAND > band = dynamic_cast<set_gnss*>(_set)->band((*it).gsys());
                int band_num = band.size();
                b1 = band[0];
                b2 = band[1];
                gnss_data_obs gobsP1 = gnss_data_obs((*it).select_range(b1));
                gnss_data_obs gobsP2 = gnss_data_obs((*it).select_range(b2));
                gnss_data_obs gobsL1 = gnss_data_obs((*it).select_phase(b1));
                gnss_data_obs gobsL2 = gnss_data_obs((*it).select_phase(b2));
                //double ele = it->gnss_data_obs_manager::getele() * R2D;
                int slip = 0;
                if ((*it).getlli(gobsL1.gobs()) >= 1 || (*it).getlli(gobsL2.gobs()) >= 1)
                    slip = 1;
                slipflagInfo << (*it).sat() << " " << slip << std::endl;
            }
            crt_time.add_secs(int(_interval));
        }
        if (_cycleslipflagfile)
        {
            _cycleslipflagfile->write(slipflagInfo.str().c_str(), slipflagInfo.str().size());
            _cycleslipflagfile->flush();
        }
        return 1;

    }

    bool gnss_proc_EXEMULTIPATH::_average_mp_post()
    {
        for (auto iter_time = _mp_orig.begin(); iter_time != _mp_orig.end(); iter_time++)
        {
            base_time crt_time = iter_time->first;
            auto allsat = iter_time->second;
            for (auto iter_sat=allsat.begin(); iter_sat != allsat.end(); iter_sat++)
            {
                std::string prn = iter_sat->first;
                auto mpdata = iter_sat->second;
                if (double_eq(mpdata[1], 1.0)) _average_mp_post_freq(crt_time, prn, 1);
                if (double_eq(mpdata[3], 1.0)) _average_mp_post_freq(crt_time, prn, 3);
                if (double_eq(mpdata[5], 1.0)) _average_mp_post_freq(crt_time, prn, 5);
            }
        }
        return 1;

    }

    bool gnss_proc_EXEMULTIPATH::_average_mp_post_freq(const base_time& crt_time, const std::string& prn, const int& i)
    {
        base_time tmp_time = crt_time, tmp1_time = crt_time;
        int j = i + 1;
        std::vector<double> tmp_mp; tmp_mp.push_back(_mp_orig[crt_time][prn][j]);
        double sum_mp = _mp_orig[crt_time][prn][j];
        tmp_time.add_secs(int(_interval));
        while (true)
        {
            if (_mp_orig[tmp_time][prn][i] > 1)
            {
                tmp_mp.push_back(_mp_orig[tmp_time][prn][j]);
                sum_mp += _mp_orig[tmp_time][prn][j];
                tmp_time.add_secs(int(_interval));
            }
            else break;
            if (tmp_time > _end_time)
            {
                tmp_time.del_secs(int(_interval));
                break;
            }
        }
        if (tmp_mp.size() <= 1)
        {
            SPDLOG_LOGGER_DEBUG(_spdlog, "gnss_proc_EXEMULTIPATH::_average_mp_post_freq: only one value int the arc, remove!");
            return false;
        }
        double mean_mp = sum_mp / tmp_mp.size();
        while (tmp1_time <= tmp_time)
        {
            _mp_avearc[tmp1_time][prn][i] = _mp_orig[tmp1_time][prn][i];
            _mp_avearc[tmp1_time][prn][j] = _mp_orig[tmp1_time][prn][j] - mean_mp;
            tmp1_time.add_dsec(int(_interval));
        }
        return true;
    }

    int gnss_proc_EXEMULTIPATH::_phaseHighDiff(const base_time& now)
    {
        std::string prn = "";
        std::map<FREQ_SEQ, GOBSBAND> crt_bands;
        double d_res_l, d_res_d;
        _data.erase(_data.begin(), _data.end());
        _data = _allobs->obs(_crt_rec, now);
        for (auto it = _data.begin(); it != _data.end(); ++it)
        {
            prn = it->sat();
            crt_bands = _band_index[it->gsys()];
            for (const auto& iter : crt_bands)
            {
                GOBSBAND band = iter.second;
                d_res_l = d_res_d = 0;
                if (it->gsys() == BDS && band == BAND_7 && prn > "C16") band = BAND_6;
                if (it->gsys() == BDS && band == BAND_6 && prn <= "C16") band = BAND_7;
                _phaseHighDiffFreq(it, band, d_res_l, d_res_d);

                _noise_phase[_crt_time][prn][band][1] = d_res_l;
                _noise_phase[_crt_time][prn][band][2] = d_res_d;
            }
        }
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_phaseHighDiffBase(const base_time& now)
    {
        _data_base.erase(_data_base.begin(), _data_base.end());
        _data_base = _allobs->obs(_site_base, now);
        std::string prn = "";
        std::map<FREQ_SEQ, GOBSBAND> crt_bands;
        double d_res_l, d_res_d;
        for (auto it = _data_base.begin(); it != _data_base.end(); ++it)
        {
            prn = it->sat();
            crt_bands = _band_index[it->gsys()];
            for (const auto& iter : crt_bands)
            {
                GOBSBAND band = iter.second;
                d_res_l = d_res_d = 0;
                if (it->gsys() == BDS && band == BAND_7 && prn > "C16") band = BAND_6;
                if (it->gsys() == BDS && band == BAND_6 && prn <= "C16") band = BAND_7;
                _phaseHighDiffFreq(it, band, d_res_l, d_res_d);
                _noise_phase_base[_crt_time][prn][band][1] = d_res_l;
                _noise_phase_base[_crt_time][prn][band][2] = d_res_d;
            }
        }
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_phaseHighDiffFreq(std::vector<gnss_data_sats>::iterator &it, const GOBSBAND &band, double& d_res_l, double& d_res_d)
    {
        std::string prn = it->sat();
        auto  obsL = it->select_phase(band);
        std::string strGOBS = gobs2str(obsL);
        strGOBS.replace(0, 1, "D");
        GOBS obsD = str2gobs(strGOBS);
        double l_value = it->getobs(obsL);
        double d_value = it->obs_D(obsD);
        /// phase 3rd diff
        if (!double_eq(l_value, 0.0))
        {
            if (_res_l.find(prn) == _res_l.end() || _res_l[prn].find(band) == _res_l[prn].end())
            {
                std::vector<double> v(0);
                _res_l[prn][band] = v;
            }
            _res_l[prn][band].push_back(l_value);
        }
        else
        {
            _res_l[prn][band].clear();
            _res_d[prn][band].clear();
        }

        // doppler 2rd diff
        if (_res_d.find(it->sat()) == _res_d.end() || _res_l[prn].find(band) == _res_l[prn].end())
        {
            std::vector<double> v(0);
            _res_d[prn][band] = v;
        }
        _res_d[prn][band].push_back(d_value);

        if (!double_eq(l_value, 0.0) && /*!double_eq(d_value,0.0)&&*/   _res_l[prn][band].size() > 4 && _res_d[prn][band].size() > 3)
        {
            _res_l[prn][band].erase(_res_l[prn][band].begin());
            _res_d[prn][band].erase(_res_d[prn][band].begin());
            /*std::cerr << it->epoch().sod() << " " << it->sat() << " " << _res_l[prn][band][3] 
                << " " << _res_l[prn][band][2] << " " << _res_l[prn][band][1] << " " << _res_l[prn][band][0] << std::endl;*/
            d_res_l = _res_l[prn][band][3] - 3 * _res_l[prn][band][2] + 3 * _res_l[prn][band][1] - _res_l[prn][band][0];
            d_res_d = _res_d[prn][band][2] - 2 * _res_d[prn][band][1] + _res_d[prn][band][0];
            // std::cerr << d_res_l << " " << _res_d[prn][band][1] << " " << d_res_d << std::endl;
        }
        if (abs(d_res_l) > 50) d_res_l = 0;
        if (abs(d_res_d) > 50) d_res_d = 0;
        return 1;
    }

    int gnss_proc_EXEMULTIPATH::_codeSigleDiff_site(const base_time& now)
    {
        // ues or not?
        _data.erase(_data.begin(), _data.end());
        _data = _allobs->obs(_crt_rec, now);
        _data_base.erase(_data_base.begin(), _data_base.end());
        _data_base = _allobs->obs(_site_base, now);

        auto it = _data.begin();
        std::ostringstream noiseInfo; noiseInfo.str("");
        std::map<FREQ_SEQ, GOBSBAND> crt_bands;
        std::string prn = "";
        int sat_num_total = 0;
        std::map <std::string, double> rec_clock;     /// single difference between sites for the same satellite
        std::map < std::string, int> sat_num;         /// 
        for (it = _data.begin(); it != _data.end(); )
        { // Just to get the receiver clock.
            gnss_data_sats* it_base = NULL;
            /// find common sat 
            prn = it->sat();
            int i = 0;
            for (i = 0; i < _data_base.size(); i++)
            {
                if (prn == (_data_base)[i].sat())
                {
                    it_base = &(_data_base)[i];
                    break;
                }
            }
            if (i == _data_base.size())
            {
                /// not commom 
                it++;
                continue;
            }

            crt_bands = _band_index[it->gsys()];
            if ((_obscomb == OBSCOMBIN::RAW_SINGLE /*|| _observ == RAW_MIX*/) && crt_bands.size() > 1)
            {
                auto first_band = (*crt_bands.begin());
                crt_bands.clear();
                crt_bands.insert(first_band);
            }
            for (const auto& iter : crt_bands)
            {
                GOBSBAND band = iter.second;
                auto  obsP = it->select_range(band);
                double p_value = it->getobs(obsP);
                obsP = it_base->select_range(band);
                double p_value_base = it_base->getobs(obsP);
                if (!double_eq(p_value, 0.0) && !double_eq(p_value_base, 0.0))
                {
                    rec_clock[it->sys()] += (p_value - p_value_base);     // receiver clock
                    sat_num[it->sys()]++;
                }
            }
            ++it;
        }

        for (auto itt = rec_clock.begin(); itt != rec_clock.end(); itt++)
        {
            std::string sys = itt->first;
            itt->second /= sat_num[sys];            // The receiver clock of the satellites of the same system is same.
            sat_num_total += sat_num[sys];
        }
        for (it = _data.begin(); it != _data.end(); )
        {
            gnss_data_sats* it_base = NULL;

            /// find common sat 
            int i = 0;
            for (i = 0; i < _data_base.size(); i++)
            {
                if (it->sat() == (_data_base)[i].sat())
                {
                    it_base = &(_data_base)[i];
                    break;
                }
            }
            if (i == _data_base.size())
            {
                /// not commom 
                it++;
                continue;
            }
            prn = it->sat();
            crt_bands = _band_index[it->gsys()];

            for (const auto& iter : crt_bands)
            {
                GOBSBAND band = iter.second;
                if (it->gsys() == BDS && band == BAND_7 && prn > "C16") band = BAND_6;
                if (it->gsys() == BDS && band == BAND_6 && prn <= "C16") band = BAND_7;
                auto  obsP = it->select_range(band);
                double p_value = it->getobs(obsP);
                obsP = it->select_range(band);
                double p_value_base = it_base->getobs(obsP);

                if (!double_eq(p_value, 0.0) && !double_eq(p_value_base, 0.0))
                {
                    double d_res_p = p_value - p_value_base - rec_clock[it->sys()];
                    _noise_code[now][prn][band][1] = d_res_p;
                }
            }
            it++;
        }
        return 1;
    }

    bool gnss_proc_EXEMULTIPATH::ExtractMP(std::string site, const base_time& beg_r, const base_time& end_r)
    {
        double percent = 0;
        double ele, distance, sat_clk;
        int satnums = 0;
        std::string prn = "";
        _crt_time = beg_r;
        std::cerr << _crt_rec << ": Start ExtractMP Processing: " << _crt_time.str_ymdhms() << " " << _end_time.str_ymdhms() << std::endl;
        while (_crt_time < end_r || _crt_time == end_r)
        {
            percent = _crt_time.diff(_beg_time) / _end_time.diff(_beg_time) * 100.0;
            std::cerr << "\r" << site << "   " << _crt_time.str_ymdhms() << std::fixed << std::setw(6) << std::setprecision(1) << percent << "%";
            _gturboedit->ProcessBatch(_crt_rec, _crt_time, _crt_time, _interval);
            if (_site_kinematic || _rec_crds[_crt_rec].isZero())
            {
                _rec_crds[_crt_rec] = _gturboedit->get_site_crd(site);
                if (_rec_crds[_crt_rec].isZero())
                {
                    if (_siteSPP(_crt_rec, _crt_time))
                        _rec_crds[_crt_rec] = _sppflt[_crt_rec]->getCrd(_crt_time);
                }
            }
            satnums = 0;
            for (auto it = _sat_list.begin(); it != _sat_list.end(); ++it)
            {   //insert the all satellites whose ele > 0.
                ele = distance = sat_clk = 0;
                prn = it->c_str();
                if (_getSatInfo(prn, distance, ele, sat_clk))
                {
                    if (ele > _minimum_elev)
                        _epo_avaisatlist[_crt_time].insert(std::make_pair(prn, ele));
                }

            }
            _data.erase(_data.begin(), _data.end());
            _data = _allobs->obs(_crt_rec, _crt_time);
            if (_data.size() == 0)
            {
                _crt_time.add_secs(int(_interval));
                continue;
            }
            _prepareDataMP(_crt_rec, _data);
            for (auto it = _data.begin(); it != _data.end(); ++it)
            {
                _combineMP(*it);
            }
            _crt_time.add_secs(int(_interval));
        }
        _average_mp_post();
        _outputMP();
        _output_slip_flag();
        //_outputMP_test();

        return true;
    }

    bool gnss_proc_EXEMULTIPATH::ExtractNoise(std::string site, const base_time& begT, const base_time& endT)
    {
        double percent = 0;
        base_time now(begT);
        _crt_rec = site;
        std::cerr << _crt_rec << ": Start ExtractNoise Processing: " << now.str_ymdhms() << " " << begT.str_ymdhms() << std::endl;
        while (now < endT || now == endT)
        {
            _crt_time = now;
            percent = _crt_time.diff(begT) / endT.diff(begT) * 100.0;
            std::cerr << "\r" << site << "   " << _crt_time.str_ymdhms() << std::fixed << std::setw(6) << std::setprecision(1) << percent << "%";
            _phaseHighDiff(now);
            if(_site_base!="")
                _codeSigleDiff_site(now);
            now.add_dsec(_interval);
        }

        // for Base station.
        now = begT;
        _res_l.clear(); _res_d.clear();
        if (_site_base != "")
        {
            while (now < endT || now == endT)
            {
                _crt_time = now;
                _phaseHighDiffBase(now);
                now.add_dsec(_interval);
            }

        }
        _outputNoise();
        return true;
    }

    bool gnss_proc_EXEMULTIPATH::ProcessBatch(std::string site, const base_time& beg_r, const base_time& end_r)
    {
        ExtractMP(site, beg_r, end_r);
        std::cerr << "" << std::endl;
        ExtractNoise(site, beg_r, end_r);
        std::cerr << "" << std::endl;
        return true;
    }
}