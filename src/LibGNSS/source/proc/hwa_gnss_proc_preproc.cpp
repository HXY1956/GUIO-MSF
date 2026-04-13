#include "hwa_gnss_proc_preproc.h"
#include "hwa_base_common.h"
#include "hwa_base_timesync.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_model_Bancroft.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    gnss_proc_preproc::gnss_proc_preproc(gnss_all_obs *obs, set_base *settings)
        : _spdlog(nullptr)
    {
        if (nullptr == settings)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _set = settings;
        }
        _obs = obs;
        //_msoffset.clear();
        _beg_end = true;

        _sys = dynamic_cast<set_gen *>(_set)->sys();
        _scl = dynamic_cast<set_gen *>(_set)->sampling_scalefc();
        _sat = dynamic_cast<set_gnss *>(_set)->sat();
        if (_sat.size() == 0)
        {
            hwa_map_sats _gnss_sats = gnss_sats();
            hwa_map_sats::const_iterator itGNS;
            for (itGNS = _gnss_sats.begin(); itGNS != _gnss_sats.end(); ++itGNS)
            {
                GSYS gsys = itGNS->first;
                set<std::string> sats = _gnss_sats[gsys];
                for (set<std::string>::iterator it = sats.begin(); it != sats.end(); ++it)
                    _sat.insert(*it);
            }
        }
        _sigCode = dynamic_cast<set_gnss *>(_set)->sigma_C(GPS);
        _sigPhase = dynamic_cast<set_gnss *>(_set)->sigma_L(GPS);
        _sigCode_GLO = dynamic_cast<set_gnss *>(_set)->sigma_C(GLO);
        _sigPhase_GLO = dynamic_cast<set_gnss *>(_set)->sigma_L(GLO);
        _sigCode_GAL = dynamic_cast<set_gnss *>(_set)->sigma_C(GAL);
        _sigPhase_GAL = dynamic_cast<set_gnss *>(_set)->sigma_L(GAL);
        _sigCode_BDS = dynamic_cast<set_gnss *>(_set)->sigma_C(BDS);
        _sigPhase_BDS = dynamic_cast<set_gnss *>(_set)->sigma_L(BDS);
        _sigCode_QZS = dynamic_cast<set_gnss *>(_set)->sigma_C(QZS);
        _sigPhase_QZS = dynamic_cast<set_gnss *>(_set)->sigma_L(QZS);
        _sigCode_IRN = dynamic_cast<set_gnss *>(_set)->sigma_C(IRN);
        _sigPhase_IRN = dynamic_cast<set_gnss *>(_set)->sigma_L(IRN);

        set<std::string> rec_list = dynamic_cast<set_gen *>(_set)->rec_all();
        for (auto rec : rec_list)
        {
            for (set<std::string>::iterator it = _sys.begin(); it != _sys.end(); it++)
            {
                _dI[rec][*it] = 0.0;
            }
            for (set<std::string>::iterator it = _sat.begin(); it != _sat.end(); it++)
            {
                _dI[rec][*it] = 0.0;
            }
            _firstEpo[rec] = true;
        }

        //   if( _set ){ // AVOID gsetproc !
        //     _sigCode  = dynamic_cast<set_gproc*>(_set)->sigma_C();
        //     _sigPhase = dynamic_cast<set_gproc*>(_set)->sigma_L();
        //   }
    }

    // Destructor
    // ----------
    gnss_proc_preproc::~gnss_proc_preproc()
    {
    }

    void gnss_proc_preproc::spdlog(base_log spdlog)
    {
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
    // run preprocessing from time beg until time end (main method)
    // --------------------------------------------------------------
    // rename by zzwu
    int gnss_proc_preproc::ProcessBatch(std::string site, const base_time &beg_r, const base_time &end_r, double sampl, bool sync, bool save)
    {

        int sign = 1;
        base_time beg;
        base_time end;

        if (!_beg_end)
        {
            beg = end_r;
            end = beg_r;
            sign = -1;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Preprocessing in end -> begin direction!");
        }
        else
        {
            beg = beg_r;
            end = end_r;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Preprocessing in begin -> end direction!");
        }

        this->_site = site;
        //   std::cerr << "start prep for " << _site << std::endl;
        if (_obs->nepochs(_site) <= 1)
            return -1;

        //bool firstEpo = true;
        std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData;
        //std::vector<gnss_data_obs_manager>             epoDataPre;

        double CJ = 0.0; // integer ms clk jump

        double subint = 0.1;
        if (_scl > 0)
            subint = 1.0 / _scl;
        if (sampl > 1)
            subint = pow(10, floor(log10(sampl)));
        // std::cerr << "subint 2:" << subint << " _scl: " << _scl << std::endl;

        bool time_loop = true;
        base_time epoch(beg);
        while (time_loop)
        {
            if (_beg_end && (epoch < end || epoch == end))
                time_loop = true;
            else if (_beg_end && epoch > end)
                time_loop = false;

            if (!_beg_end && (epoch > end || epoch == end))
                time_loop = true;
            else if (!_beg_end && epoch < end)
                time_loop = false;

            //     std::cout << epoch.str_ymdhms("epoch: ")+base_type_conv::dbl2str(epoch.dsec()) << " sampl:" << sampl << std::endl;

            // synchronization
            if (sync)
            {
                if (!time_sync(epoch, sampl, _scl, _spdlog))
                {
                    //        std::cerr << epoch.str_ymdhms("synchronize: ")+base_type_conv::dbl2str(epoch.dsec()) << " add: " << subint/100 << " smpl:" << sampl << std::endl;
                    epoch.add_dsec(subint / 100); // add_dsec used for synchronization!
                    continue;
                }
            }

            if (sampl >= 1)
                epoch.reset_dsec(); //  LOW-RATE (>=1Hz), i.e. if not HIGH-RATE !!

            if (_inputEpoData.size() == 0)
                epoData = _obs->obs_pt(_site, epoch);
            else
                epoData = _inputEpoData;

            if (epoData.size() == 0 || !time_loop)
            {
                if (sampl >= 1)
                    epoch.add_secs(sign * (int)sampl); // =<1Hz data
                else
                    epoch.add_dsec(sign * sampl); //  >1Hz data
                continue;
            }

            if (/*firstEpo*/ _firstEpo[_site])
            {
                for (unsigned int i = 0; i < epoData.size(); i++)
                    _epoDataPre[_site].push_back(*epoData[i]);
                //firstEpo = false;
                _firstEpo[_site] = false;
            }

            // test data gap
            if (save)
            {
                std::shared_ptr<gnss_data_obs_manager> g = *(epoData.begin());
                double diffEpo = g->epoch() - _epoDataPre[_site].begin()->epoch();
                if (/*!firstEpo*/ !_firstEpo[_site] && diffEpo > sampl + DIFF_SEC(sampl))
                {
                    _gapReport(epoData);
                }
                _compare(_epoDataPre[_site], epoData);
            }

            double Ns = epoData.size(); // total number of viewed satellites
            double Na = Ns;             // number of satellite involved into clk jump detection
            double n = 0;               // number of detected jumps in particular epoch
            _sumS = 0;

            std::shared_ptr<gnss_data_obs_manager> obs2;

            for (std::vector<std::shared_ptr<gnss_data_obs_manager>>::iterator it = epoData.begin(); it != epoData.end(); ++it)
            {
                if (*it == 0)
                {
                    //     std::cout << "*it = 0" << std::endl;
                    Na--;
                    continue;
                }

                obs2 = *it;
                gnss_data_obs_manager *obs1 = 0;

                std::vector<gnss_data_obs_manager>::iterator itPre;
                for (itPre = _epoDataPre[_site].begin(); itPre != _epoDataPre[_site].end(); ++itPre)
                {
                    if ((*it)->sat() == (itPre)->sat())
                    {
                        obs1 = &(*itPre);
                    }
                }

                if (obs1 == 0)
                {
                    //     std::cout << "obs1 = 0" << std::endl;
                    Na--;
                    continue;
                } // sat not found in previous epoch

                gnss_sys gsys(obs1->gsys());
                if (gsys.gsys() != GPS)
                    Na--; // clk jump is determined only from GPS

                std::string satname = obs1->sat();
                if (_sat.find(satname) == _sat.end())
                    continue; // sat excluded in config

                // Cycle slip detection and repair
                if (sync)
                {
                    if (abs(obs1->epoch() - obs2->epoch()) <= sampl + 1 && _slip(obs1, obs2) > -1)
                    {
                        _transform(obs2, save); // 1 second reserve for obs sync
                    }
                }
                else if (_slip(obs1, obs2) > -1)
                    _transform(obs2, save);

                // Clock jump detection and repair
                int irc_jmp = _jumps(obs1, obs2);
                if (irc_jmp == 1)
                    n++;
                if (irc_jmp == -1)
                    Na--;
            } // end sats loop
            //std::cout << epoDataPre[0].epoch().hour() << "  " << epoDataPre[0].epoch().mins() << "  " << epoDataPre[0].epoch().secs() << "  " << epoDataPre[0].sat() << std::endl;

            _epoDataPre[_site].clear();

            for (unsigned int i = 0; i < epoData.size(); i++)
                _epoDataPre[_site].push_back(*epoData[i]);

            if (n == Na)
            { // clk jump effect at all sats in the same way
                double M = (1e3 * _sumS) / (CLIGHT * n);
                double Js = abs(M - floor(M + 0.5));

                if (Js <= 1e-5)
                {                         // it is clock jump instead of common slip
                    CJ += floor(M + 0.5); // ms jump
                    if (save)
                        _mbreaks[_site][epoch] = (int)CJ; // store for logging
                    _remove_slip(epoData);
                }
            }

            if (CJ != 0.0)
            {
                _repair(epoData, 1e-3 * CJ * CLIGHT); // corrected carrier phase
                                                      //        std::cout << "Opravil jsem epoData: " << epoch.str_hms() << " o hodnotu " << CJ << " ms" << std::endl;
            }

            if (_satdata_npp)
                _updateNppData(epoData, !double_eq(CJ, 0.0));

            if (sampl >= 1)
                epoch.add_secs(sign * (int)sampl); // =<1Hz data
            else
                epoch.add_dsec(sign * sampl); //  >1Hz data
        }

        //   std::cerr << "Preprocess for " << site << " finished" << std::endl;
        return 1;
    }

    void gnss_proc_preproc::setSingleEpoData(std::vector<gnss_data_sats> *input_data)
    {
        //if (!input_data) return;

        //if (_satdata_npp) _satdata_npp = nullptr;
        //_satdata_npp = input_data;

        //int nsize = _satdata_npp->size();

        //_inputEpoData.clear();
        //_map_nppdata_idx.clear();
        //for (unsigned int i = 0; i < nsize; i++)
        //{
        //    std::shared_ptr<gnss_data_obs_manager> gobs(new gnss_data_sats((*_satdata_npp)[i]));
        //    _inputEpoData.push_back(gobs);
        //    _map_nppdata_idx[gobs->site()][gobs->sat()] = i;
        //}

        // modified
        _inputEpoData.clear();
        _map_nppdata_idx.clear();

        return;
    }

    // Compute threshold for cycle slip
    // -------------------------------------
    double gnss_proc_preproc::_slipThreshold(std::string LC, hwa_spt_obsmanager obs, gnss_data_band &band1, gnss_data_band &band2, double sampl)
    {

        double iono = 0.4 / 60; // maximum iono jump/sec  ??? set via XML ???

        double f1 = obs->frequency(band1.band()); // first  band frequency
        double f2 = obs->frequency(band2.band()); // second band frequency
        double k1 = f1 / (f1 - f2);
        double k2 = f2 / (f1 - f2);
        double k3 = f1 / (f1 + f2);
        double k4 = f2 / (f1 + f2);

        //  double alfa = (f1*f1)/(f1*f1 - f2*f2);

        double sigCode = 0;
        double sigPhase = 0;

        GSYS gs = obs->gsys();
        switch (gs)
        {
        case GPS:
            sigCode = _sigCode;
            sigPhase = _sigPhase;
            break;
        case GLO:
            sigCode = _sigCode_GLO;
            sigPhase = _sigPhase_GLO;
            break;
        case GAL:
            sigCode = _sigCode_GAL;
            sigPhase = _sigPhase_GAL;
            break;
        case BDS:
            sigCode = _sigCode_BDS;
            sigPhase = _sigPhase_BDS;
            break;
        case QZS:
            sigCode = _sigCode_QZS;
            sigPhase = _sigPhase_QZS;
            break;
        case IRN:
            sigCode = _sigCode_IRN;
            sigPhase = _sigPhase_IRN;
            break;
        case SBS:
            sigCode = _sigCode;
            sigPhase = _sigPhase;
            break;
        case GNS:
            sigCode = _sigCode;
            sigPhase = _sigPhase;
            break;
        default:
            sigCode = 0.0;
            sigPhase = 0.0;
        }

#ifdef DEBUG
        std::cout << "sigma phase = " << sigPhase << std::endl;
        std::cout << "iono*sample = " << iono * sampl << std::endl;
        std::cout << "sample = " << sampl << std::endl;
#endif

        if (LC.compare("L4") == 0)
        {
            return 2.5 * sqrt(4 * sigPhase * sigPhase) + iono * sampl;
        }
        else if (LC.compare("MW") == 0)
        {
            double bias = 0.05;
            double mltp = 0.3;
            return 2.5 * sqrt(2 * k1 * k1 * sigPhase * sigPhase +
                              2 * k2 * k2 * sigPhase * sigPhase +
                              2 * k3 * k3 * sigCode * sigCode +
                              2 * k4 * k4 * sigCode * sigCode + bias + mltp);
        }
        else if (LC.compare("WL") == 0)
        {
            double bias = 0.05;
            return 2.5 * sqrt(2 * k1 * k1 * sigPhase * sigPhase +
                              2 * k2 * k2 * sigPhase * sigPhase + bias);
        }
        else
            return 0.0;
    }

    // check coherency between range and phase caused by clock jump
    // --------------------------------------------------------------
    int gnss_proc_preproc::_jumps(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2)
    {

        std::string prn = gobs2->sat();
        GSYS GS = gobs1->gsys();

        if (GS != GPS)
            return -1;

        set<GOBSBAND> freq_1 = gobs1->band_avail();
        set<GOBSBAND> freq_2 = gobs2->band_avail();

        if (freq_1.size() < 2 || freq_2.size() < 2)
        {
            return -1;
        }

        GOBSBAND band;
        set<GOBSBAND>::reverse_iterator it = freq_1.rbegin();
        // int b1_1 = gnss_sys::freq2band(gobs1->gsys(), *it);
        band = *it; // gnss_sys::freq2band(gobs1->gsys(), *it);
        gnss_data_band b1_1(band, ATTR);
        it++;
        // int b2_1 = gnss_sys::freq2band(gobs1->gsys(), *it);
        band = *it; // gnss_sys::freq2band(gobs1->gsys(), *it);
        gnss_data_band b2_1(band, ATTR);

        it = freq_2.rbegin();
        // int b1_2 = gnss_sys::freq2band(gobs2->gsys(), *it);
        band = *it; // gnss_sys::freq2band(gobs2->gsys(), *it);
        gnss_data_band b1_2(band, ATTR);
        it++;
        // int b2_2 = gnss_sys::freq2band(gobs2->gsys(), *it);
        band = *it; // gnss_sys::freq2band(gobs2->gsys(), *it);
        gnss_data_band b2_2(band, ATTR);

        // j-epoch
        double L1j = gobs2->obs_L(b1_2); // [m]
        double L2j = gobs2->obs_L(b2_2); // [m]
        double P1j = gobs2->obs_C(b1_2); // [m]
        double P2j = gobs2->obs_C(b2_2); // [m]

        // i-epoch
        double L1i = gobs1->obs_L(b1_1); // [m]
        double L2i = gobs1->obs_L(b2_1); // [m]
        double P1i = gobs1->obs_C(b1_1); // [m]
        double P2i = gobs1->obs_C(b2_1); // [m]

        int nl = 0;
        double dl = 0.0;
        if (!double_eq(L1j, 0.0) && !double_eq(L1i, 0.0))
        {
            dl += L1j - L1i;
            nl++;
        }
        if (!double_eq(L2j, 0.0) && !double_eq(L2i, 0.0))
        {
            dl += L2j - L2i;
            nl++;
        }
        if (nl == 0)
            return -1;
        dl /= nl;

        int np = 0;
        double dp = 0.0;
        if (!double_eq(P1j, 0.0) && !double_eq(P1i, 0.0))
        {
            dp += P1j - P1i;
            np++;
        }
        if (!double_eq(P2j, 0.0) && !double_eq(P2i, 0.0))
        {
            dp += P2j - P2i;
            np++;
        }
        if (np == 0)
            return -1;
        dp /= np;

        double S = dp - dl;                 // jump detection observable
        double sig = 5;                     // sigma of S
        double k = 1e-3 * CLIGHT - 3 * sig; // jump threshold

#ifdef DEBUG
        std::cout << gobs2->epoch().str_hms() << " " << gobs2->sat() << " S: " << S << " dp: " << dp << " dl: " << dl << " k: " << k << std::endl;
#endif

        if (abs(S) >= k)
        { // candidate of clock jump
            _sumS += S;
            return 1;
        }
        return 0;
    }

    // Set navigation file (rinexn or sp3)
    // --------------------------
    void gnss_proc_preproc::setNav(gnss_all_nav *nav)
    {
        this->_nav = nav;
    }

    // Repair phase due to clk jump
    // -----------------------------------
    void gnss_proc_preproc::_repair(std::vector<hwa_spt_obsmanager> epoData, double dL)
    {

        for (std::vector<hwa_spt_obsmanager>::iterator it = epoData.begin(); it != epoData.end(); it++)
        {
            (*it)->mod_L(dL, X); // modify all phases by dL [m]
        }
    }

    // Set site name
    // -------------------
    void gnss_proc_preproc::setSite(std::string site)
    {
        this->_site = site;
    }

    // Get std::map with cysle slips
    // ---------------------------------
    std::map<base_time, std::map<std::string, std::map<GOBS, double>>> gnss_proc_preproc::getSlips()
    {
        return _mslips[_site];
    }

    // Get std::map with cysle slips : gap version
    // ---------------------------------
    std::map<base_time, std::map<std::string, std::map<GOBS, int>>> gnss_proc_preproc::getSlipsGap()
    {
        return _mslipsGap[_site];
    }

    // Get std::map with clk jumps
    // ---------------------------------
    std::map<base_time, int> gnss_proc_preproc::getClkjump()
    {
        return _mbreaks[_site];
    }

    // Transform dN to slips on individual band
    // ----------------------------------
    int gnss_proc_preproc::_transform(hwa_spt_obsmanager gobs, bool save)
    {

        int d = _v_lcslp.size();
        if (d <= 1)
            return -1;

        unsigned int maxit = 1;
        for (t_vec_slp::iterator it = _v_lcslp.begin(); it != _v_lcslp.end(); it++)
        {
            if (it->second.size() > maxit)
                maxit = it->second.size();
        }

        for (unsigned int tmp = 1; tmp <= maxit; tmp++)
        {
            Matrix M(d, d);
            M.setZero();
            Vector S(d);
            S.setZero();
            std::map<GOBS, double> m_orig;

            bool trans = false;

            for (int i = 0; i < d - 1; i++)
            {
                // find narr-lane slp
                if (_v_lcslp.find(i + 10) != _v_lcslp.end())
                {
                    S(d - 1) = -_v_lcslp[i + 10].begin()->val;
                    if (!double_eq(S(d - 1), 0.0))
                        trans = true;
                    m_orig[_v_lcslp[i + 10].begin()->obs1.gobs()] = 0;
                    m_orig[_v_lcslp[i + 10].begin()->obs2.gobs()] = 0;

                    if (_v_lcslp[i + 10].size() >= 2)
                    {
                        _v_lcslp[i + 10].erase(_v_lcslp[i + 10].begin());
                    }

                    M(d - 1, 0) = 2;
                    M(d - 1, 1) = -1;
                }

                // find wide-lane slps
                if (_v_lcslp.find(i) != _v_lcslp.end())
                {

                    M(i, d - 1) = -1;
                    M(i, d - 2 - i) = 1;

                    S(i) = -_v_lcslp[i].begin()->val;

                    m_orig[_v_lcslp[i].begin()->obs1.gobs()] = 0;
                    m_orig[_v_lcslp[i].begin()->obs2.gobs()] = 0;
                    if (_v_lcslp[i].size() >= 2)
                    {
                        _v_lcslp[i].erase(_v_lcslp[i].begin());
                    }

                    if (!double_eq(S(d - 2 - i), 0.0))
                        trans = true;
                }
            }

            //std::cout << gobs->sat() << " " << gobs->epoch().str_hms() << std::endl;
            //std::cout << "M = " << M << std::endl;
            //std::cout << "S = " << S << std::endl;

            Vector O(S);
            O.setZero();

            if (trans)
            {
                O = M.inverse() * S;
                set<GOBSBAND> bands;
                for (std::map<GOBS, double>::iterator it = m_orig.begin(); it != m_orig.end(); it++)
                {
                    gnss_data_obs s;
                    s.gobs(it->first);
                    //      int f = gnss_sys::band2freq(gobs->gsys(), s.band());
                    bands.insert(s.band());
                }

                std::vector<GOBSBAND> vec_bnd = sort_band(gobs->gsys(), bands);

                int i = 0;
                for (auto it = vec_bnd.begin(); it != vec_bnd.end(); it++, i++)
                {
                    //      GOBSBAND band = gnss_sys::freq2band(gobs->gsys(), *it);
                    for (std::map<GOBS, double>::iterator it2 = m_orig.begin(); it2 != m_orig.end(); it2++)
                    {
                        gnss_data_obs s;
                        s.gobs(it2->first);

                        if (s.band() == *it)
                        {
                            it2->second = O(i);
                            // save lli to gnss_data_obs_manager
                            gobs->addlli(it2->first, 1);
                            gobs->addslip(it2->first, int(it2->second));
                        }
                    }
                }

                if (save)
                    _save(gobs, m_orig);
            }
        }

        return 1;
    }

    // Save estimated cycle slips
    // ----------------------------
    void gnss_proc_preproc::_save(hwa_spt_obsmanager gobs, const std::map<GOBS, double> &slips)
    {
        base_time epo = gobs->epoch();
        std::string prn = gobs->sat();

        for (std::map<GOBS, double>::const_iterator it = slips.begin(); it != slips.end(); it++)
        {
            GOBS g = it->first;
            double slp = it->second;
            //      std::cout << "preproc test: " << gobs->epoch().str_hms() << " " << gobs->sat() << " " << gobs2str(g) << "  " << slp << std::endl;
            if (!double_eq(slp, 0.0))
                _mslips[_site][epo][prn][g] = slp;
        }
    }

    // remove slips from gobsgnss* and _mslips
    // ---------------------------------------
    void gnss_proc_preproc::_remove_slip(std::vector<hwa_spt_obsmanager> gobs)
    {

        base_time epo = gobs[0]->epoch();

        std::map<base_time, std::map<std::string, std::map<GOBS, double>>>::iterator itEPO;
        itEPO = _mslips[_site].find(epo);
        if (itEPO != _mslips[_site].end())
            _mslips[_site].erase(itEPO);

        for (std::vector<hwa_spt_obsmanager>::iterator itG = gobs.begin(); itG != gobs.end(); itG++)
        {
            set<GOBSBAND> freq = (*itG)->band_avail();
            for (auto itF = freq.begin(); itF != freq.end(); itF++)
            {
                //   int band = gnss_sys::freq2band( (*itG)->gsys(), *itF);
                //   GOBSBAND band = gnss_sys::freq2band( (*itG)->gsys(), *itF);
                gnss_data_band(*itF, ATTR);
                //   GOBS g = (*itG)->pha_id(*itF);
                GOBS g = (*itG)->id_phase(*itF);
                (*itG)->addlli(g, 0);
            }
        }
    }

    // only common items remain
    // -----------------------------------
    void gnss_proc_preproc::_common(set<GOBSBAND> &set1, set<GOBSBAND> &set2)
    {

        for (auto it1 = set1.begin(); it1 != set1.end();)
        {
            auto it2 = set2.find(*it1);
            if (it2 == set2.end())
            {
                auto tmp = it1;
                ++it1;
                set1.erase(tmp);
            }
            else
                ++it1;
        }

        for (auto it1 = set2.begin(); it1 != set2.end();)
        {
            auto it2 = set1.find(*it1);
            if (it2 == set1.end())
            {
                auto tmp = it1;
                ++it1;
                set2.erase(tmp);
            }
            else
                ++it1;
        }
    }

    // only common items remain
    // -----------------------------------
    void gnss_proc_preproc::_common(set<GOBS> &set1, set<GOBS> &set2)
    {

        for (set<GOBS>::iterator it1 = set1.begin(); it1 != set1.end();)
        {
            set<GOBS>::iterator it2 = set2.find(*it1);
            if (it2 == set2.end())
            {
                set<GOBS>::iterator tmp = it1;
                ++it1;
                set1.erase(tmp);
            }
            else
                ++it1;
        }

        for (set<GOBS>::iterator it1 = set2.begin(); it1 != set2.end();)
        {
            set<GOBS>::iterator it2 = set1.find(*it1);
            if (it2 == set1.end())
            {
                set<GOBS>::iterator tmp = it1;
                ++it1;
                set2.erase(tmp);
            }
            else
                ++it1;
        }
    }

    // check phase cycl slips
    // --------------------------------
    int gnss_proc_preproc::_slip(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2)
    {

        bool slip = false;

        set<GOBSBAND> bands_t1 = gobs1->band_avail();
        set<GOBSBAND> bands_t2 = gobs2->band_avail();

        this->_common(bands_t1, bands_t2);

        if (bands_t1.size() != bands_t2.size())
        {
            std::cerr << "ERROR: problem in gnss_proc_preprocc::_common" << std::endl;
            return -1;
        }

        if (bands_t1.size() <= 1)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, gobs1->epoch().str_ymdhms("Not enough bands available: ") + " " + gobs1->sat());
            return -1;
        }

        int nfreq = bands_t1.size();

        gnss_data_obs s1;
        gnss_data_obs s2;
        gnss_data_obs sF;
        gnss_data_obs s_narr;

        // sort according to wavelenght
        std::vector<GOBSBAND> sorted_t1 = sort_band(gobs1->gsys(), bands_t1);
        std::vector<GOBSBAND> sorted_t2 = sort_band(gobs2->gsys(), bands_t2);

        std::vector<GOBSBAND>::reverse_iterator itFRQ = sorted_t1.rbegin();
        //   int b = gnss_sys::freq2band(gobs1->gsys(), *itFRQ);

        set<GOBS> gf2_t1 = gobs1->obs_phase(*itFRQ);
        set<GOBS> gf2_t2 = gobs2->obs_phase(*itFRQ);
        this->_common(gf2_t1, gf2_t2); // signals for wide-lane

        //second freq: for narrow-lane
        std::vector<GOBSBAND>::iterator it_narr = sorted_t1.begin();
        if (nfreq >= 2)
            it_narr++;
        //   int b_narr = gnss_sys::freq2band(gobs1->gsys(), *it_narr);
        set<GOBS> gNL_t1 = gobs1->obs_phase(*it_narr);
        set<GOBS> gNL_t2 = gobs2->obs_phase(*it_narr);
        this->_common(gNL_t1, gNL_t2); // signals for wide-lane

        _m_lcslp.clear();
        _v_lcslp.clear();

        for (int i = 0; i < nfreq - 1; i++)
        {
            ++itFRQ;

            //    b = gnss_sys::freq2band(gobs1->gsys(), *itFRQ);
            set<GOBS> gf1_t1 = gobs1->obs_phase(*itFRQ);
            set<GOBS> gf1_t2 = gobs2->obs_phase(*itFRQ);

            this->_common(gf1_t1, gf1_t2); // reference signal - last one

            //shift reference band if no common signals
            if (gf2_t1.size() == 0)
            {
                //       int b = gnss_sys::freq2band(gobs1->gsys(), *itFRQ);
                gf2_t1 = gobs1->obs_phase(*itFRQ);
                gf2_t2 = gobs2->obs_phase(*itFRQ);
                this->_common(gf2_t1, gf2_t2); // signals for wide-lane
                i--;
                nfreq--;
                continue;
            }

            // find freq for extra-wide-lane
            std::vector<GOBSBAND>::reverse_iterator itFF;
            set<GOBS> gFF_t1;
            set<GOBS> gFF_t2;
            if (i > 0)
            {
                std::vector<GOBSBAND>::reverse_iterator itFF = itFRQ;
                --itFF;
                //     b = gnss_sys::freq2band(gobs1->gsys(), *itFF);
                gFF_t1 = gobs1->obs_phase(*itFF);
                gFF_t2 = gobs2->obs_phase(*itFF);
                this->_common(gFF_t1, gFF_t2); // freq for extra-widelane
            }

            set<GOBS>::iterator itGOBSFF = gFF_t1.begin();
            set<GOBS>::iterator itGOBSf2 = gf2_t1.begin();
            set<GOBS>::iterator itGOBSNL = gNL_t1.begin();

            bool endFF = true;
            bool endNL = true;

            for (set<GOBS>::iterator itGOBSf1 = gf1_t1.begin();;)
            {
                if (gf1_t1.size() == 0 || gf2_t1.size() == 0)
                {
                    _v_lcslp.clear();
                    break;
                }

                s1.gobs(*itGOBSf1);
                s2.gobs(*itGOBSf2);

                gnss_data_obs_pair gobs_pair(s1, s2);

                double diff = 0;
                double lam = CLIGHT / gobs1->frequency_lc(s1.band(), 1, s2.band(), -1);

                if (i == 0)
                {
                    //std::cout << "MW test " << gobs2->epoch().str_hms() << " " << gobs2->sat() << std::endl;
                    double lct1 = gobs1->MW(s1, s2);
                    double lct2 = gobs2->MW(s1, s2);
                    diff = (lct2 - lct1) / lam;
                    //std::cout << gobs1->epoch().str_hms() << " " << gobs2->epoch().str_hms() << " " << gobs1->sat() << " " << gobs2str(s1.gobs()) << "  " << gobs2str(s2.gobs())
                    //     << std::fixed << setprecision(3) << " MW_t1: " << lct1 << " MW_t2: " << lct2 << " diff: " << diff << std::endl;
                    gnss_data_band band1 = s1.gband();
                    gnss_data_band band2 = s2.gband();
                    //      double thr = _slipThreshold("MW", gobs2, band1, band2) / lam;
                    double wlSlp = 0;
                    if (fabs(diff) > 2)
                    { //thr) {
                        slip = true;
                        wlSlp = round(diff);
                    }
                    gobs_pair.val = wlSlp;
                    _v_lcslp[i].push_back(gobs_pair);
                    _m_lcslp[i][gobs_pair] = wlSlp;
                }
                else
                {
                    //std::cout << "WL test " << std::endl;
                    // Compute wide-lane time differenced observations
                    double lct1 = gobs1->LWL(s1, s2);
                    double lct2 = gobs2->LWL(s1, s2);
                    double dWL = lct2 - lct1;

                    //std::cout << gobs1->sat() << " " << gobs2str(s1.gobs()) << "  " << gobs2str(s2.gobs()) << " ";
                    //std::cout << std::fixed << setprecision(3) << " WL_t1: " << lct1 << " WL_t2: " << lct2 << " dWL: " << dWL << std::endl;

                    gnss_data_band band1 = s1.gband();
                    gnss_data_band band2 = s2.gband();
                    //      double thrWL = _slipThreshold("WL", gobs2, band1, band2) / lam;

                    // Compute extra-wide-lane time differenced observatins
                    sF.gobs(*itGOBSFF);
                    lct1 = gobs1->LWL(sF, s2);
                    lct2 = gobs2->LWL(sF, s2);
                    double dEWL = lct2 - lct1;
                    gobs_pair.obs1 = sF;

                    // find extra-wide-lane cycle slip from previous cascade
                    double ewlSlp = 0;
                    int x = i - 1;
                    ewlSlp = _findSlp(x, gobs_pair);
                    double elam = 0;
                    elam = CLIGHT / gobs1->frequency_lc(gobs_pair.obs1.band(), 1, gobs_pair.obs2.band(), -1);
                    gnss_data_band bandF = sF.gband();
                    //      double thrEWL = _slipThreshold("WL", gobs2, bandF, band2) / elam;
                    diff = (dEWL - dWL + ewlSlp * elam) / lam;
                    //std::cout << gobs1->sat() << " " << gobs2str(sF.gobs()) << "  " << gobs2str(s2.gobs()) << std::fixed << setprecision(3) << " EWL_t1: " << lct1 << " EWL_t2: " << lct2 << " diff: " << diff << std::endl;
                    //      double thr = thrEWL + thrWL;
                    gobs_pair.obs1 = s1;
                    double wlSlp = 0;
                    if (fabs(diff) > 2)
                    { //thr) {
                        slip = true;
                        wlSlp = round(diff);
                    }
                    gobs_pair.val = wlSlp;
                    _v_lcslp[i].push_back(gobs_pair);
                    _m_lcslp[i][gobs_pair] = wlSlp;
                }

                // narrow-lane slip
                if (i == nfreq - 2)
                {
                    //std::cout << "NL test " << std::endl;
                    if (gf1_t1.size() == 0 || gNL_t1.size() == 0)
                        break;
                    double lct1 = gobs1->LWL(s1, s2);
                    double lct2 = gobs2->LWL(s1, s2);
                    double dWL = lct2 - lct1;

                    // find wide-lane slip from last cascade
                    double wlSlp = 0;
                    wlSlp = _findSlp(i, gobs_pair);

                    s_narr.gobs(*itGOBSNL);
                    lct1 = gobs1->LNL(s1, s_narr);
                    lct2 = gobs2->LNL(s1, s_narr);
                    double dNL = lct2 - lct1;
                    gobs_pair.obs1 = s1;
                    gobs_pair.obs2 = s_narr;
                    double nlam = CLIGHT / gobs1->frequency_lc(gobs_pair.obs1.band(), 2, gobs_pair.obs2.band(), -1);
                    double disf = _disf(gobs1, gobs2, s1, s_narr);
                    if (!slip)
                        _iono(gobs1, gobs2, s1, s_narr);
                    std::string prn = gobs1->sat();

                    diff = (dWL - dNL - disf * _dI[_site][prn] + wlSlp * lam) / nlam;
                    //std::cout << gobs1->epoch().str_hms() << " " << gobs2->epoch().str_hms() << " " << gobs1->sat() << " " << gobs2str(s1.gobs()) << "  " << gobs2str(s_narr.gobs())
                    //       << " " << diff << " " << nlam << "  " << dWL << "  " << dNL << " " << wlSlp << "  " << disf*_dI[_site][prn] << " " << disf << " " << _dI[prn] << std::endl;
                    double nlSlp = 0;
                    if (fabs(diff) > 2)
                    {
                        nlSlp = round(diff);
                    }
                    //std::cout << gobs1->sat() << " " << gobs2str(s1.gobs()) << "  " << gobs2str(s2.gobs()) << std::fixed << setprecision(3) << " NL_t1: " << lct1 << " NL_t2: " << lct2 << " diff: " << diff << std::endl;
                    gobs_pair.val = nlSlp;
                    _v_lcslp[i + 10].push_back(gobs_pair);
                    _m_lcslp[i + 10][gobs_pair] = nlSlp;
                }

                if (i > 0)
                {
                    ++itGOBSFF;
                    if (itGOBSFF == gFF_t1.end())
                        endFF = true;
                    else
                        endFF = false;
                }
                if (i == nfreq - 2)
                {
                    ++itGOBSNL;
                    if (itGOBSNL == gNL_t1.end())
                        endNL = true;
                    else
                        endNL = false;
                }

                ++itGOBSf1;
                ++itGOBSf2;
                if (itGOBSf1 == gf1_t1.end() && itGOBSf2 == gf2_t1.end() && endFF && endNL)
                    break;
                if (itGOBSf1 == gf1_t1.end())
                    --itGOBSf1;
                if (itGOBSf2 == gf2_t1.end())
                    --itGOBSf2;
                if (i > 0 && endFF)
                    --itGOBSFF;
                if (i == nfreq - 2 && endNL)
                    --itGOBSNL;
            }
        }

        return 1;
    }

    // Costructor for gnss_data_obs_pair
    // ----------------------------
    gnss_data_obs_pair::gnss_data_obs_pair(gnss_data_obs &gobs1, gnss_data_obs &gobs2)
        : obs1(gobs1),
          obs2(gobs2)
    {
    }

    // operator< for gnss_data_obs_pair
    // --------------------------
    bool gnss_data_obs_pair::operator<(const gnss_data_obs_pair &t) const
    {
        return (this->obs1.attr() < t.obs1.attr() ||

                this->obs2.attr() < t.obs2.attr());
    }

    // ionosphere scale factor: wl - nl
    // --------------------------------------
    double gnss_proc_preproc::_disf(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2, gnss_data_obs &s1, gnss_data_obs &s2)
    {
        double isf_wl = gobs1->isf_lc(s1.band(), 1, s2.band(), -1);
        double isf_nl = gobs1->isf_lc(s1.band(), 2, s2.band(), -1);
        double disf = isf_wl - isf_nl;

        return disf;
    }

    // ionodphere
    // -----------------------------------------
    void gnss_proc_preproc::_iono(gnss_data_obs_manager *gobs1, hwa_spt_obsmanager gobs2, gnss_data_obs &s1, gnss_data_obs &s2)
    {
        double k = pow(gobs1->frequency(s1.band()), 2) / pow(gobs1->frequency(s2.band()), 2);

        double Lb1_1 = gobs1->obs_L(s1);
        double Lb2_1 = gobs1->obs_L(s2);
        double Lb1_2 = gobs2->obs_L(s1);
        double Lb2_2 = gobs2->obs_L(s2);
        double dL = Lb1_2 - Lb2_2 - Lb1_1 + Lb2_1;

        double dI = -dL / (k - 1);

        std::string prn = gobs1->sat();

#ifdef DEBUG
        std::cout << "iono: " << dL << " " << Lb1_2 << " " << Lb2_2 << " " << Lb1_1 << " " << Lb2_1 << std::endl;
#endif

        _dI[_site][prn] = dI;

        //   ddI = _dI[_site][prn] - dI;
    }

    // find wl slip
    // ----------------------------------
    double gnss_proc_preproc::_findSlp(int &i, gnss_data_obs_pair &gpair)
    {
        double wlSlp = 0;
        if (_m_lcslp.find(i) != _m_lcslp.end())
        {
            if (_m_lcslp[i].find(gpair) != _m_lcslp[i].end())
            {
                wlSlp = _m_lcslp[i][gpair];
            }
        }

        return wlSlp;
    }

    // report epo data as slips due to epoch data gap
    // ------------------------------------
    void gnss_proc_preproc::_gapReport(std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData)
    {

        for (std::vector<std::shared_ptr<gnss_data_obs_manager>>::iterator it = epoData.begin(); it != epoData.end(); it++)
        {
            base_time epo = (*it)->epoch();
            std::string prn = (*it)->sat();
            std::vector<GOBS> allobs = (*it)->obs();
            for (std::vector<GOBS>::iterator itGOBS = allobs.begin(); itGOBS != allobs.end(); itGOBS++)
            {
                if (gobs_phase(*itGOBS))
                    _mslipsGap[_site][epo][prn][*itGOBS] = 1;
            }
        }
    }

    // report epo data as slips due to satellite data gap
    // ------------------------------------
    void gnss_proc_preproc::_gapReport(std::shared_ptr<gnss_data_obs_manager> data)
    {

        base_time epo = data->epoch();
        std::string prn = data->sat();
        std::vector<GOBS> allobs = data->obs();
        for (std::vector<GOBS>::iterator itGOBS = allobs.begin(); itGOBS != allobs.end(); itGOBS++)
        {
            if (gobs_phase(*itGOBS))
                _mslipsGap[_site][epo][prn][*itGOBS] = 2;
        }
    }

    // compare two epoch data
    // --------------------------------
    void gnss_proc_preproc::_compare(std::vector<gnss_data_obs_manager> data1, std::vector<std::shared_ptr<gnss_data_obs_manager>> data2)
    {

        for (std::vector<std::shared_ptr<gnss_data_obs_manager>>::iterator it2 = data2.begin(); it2 != data2.end(); it2++)
        {
            base_time epo = (*it2)->epoch();
            std::string prn = (*it2)->sat();
            bool foundPRN = false;
            for (std::vector<gnss_data_obs_manager>::iterator it1 = data1.begin(); it1 != data1.end(); it1++)
            {
                if ((*it2)->sat() == it1->sat())
                {
                    foundPRN = true;
                    std::vector<GOBS> vgobs1 = it1->obs();
                    std::vector<GOBS> vgobs2 = (*it2)->obs();
                    for (std::vector<GOBS>::iterator itGOBS2 = vgobs2.begin(); itGOBS2 != vgobs2.end(); itGOBS2++)
                    {
                        bool foundGOBS = false;
                        for (std::vector<GOBS>::iterator itGOBS1 = vgobs1.begin(); itGOBS1 != vgobs1.end(); itGOBS1++)
                        {
                            if (*itGOBS2 == *itGOBS1)
                            {
                                foundGOBS = true;
                                break;
                            }
                        }
                        if (!foundGOBS && gobs_phase(*itGOBS2))
                            _mslipsGap[_site][epo][prn][*itGOBS2] = 3;
                    }
                    break;
                }
            }
            if (!foundPRN)
                _gapReport(*it2);
        }
    }

    void gnss_proc_preproc::_updateNppData(std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData, bool clock_jump)
    {
        for (unsigned int i = 0; i < epoData.size(); i++)
        {
            std::string gsat = epoData[i]->sat();
            std::vector<GOBS> vec_obs = epoData[i]->obs();
            for (auto obs : vec_obs)
            {
                if (!gnss_data_obs(obs).is_phase())
                    continue;

                if (epoData[i]->getlli(obs) >= 1)
                    (*_satdata_npp)[_map_nppdata_idx[_site][gsat]].addlli(obs, 1);

                if (!clock_jump)
                    continue;

                double phase = epoData[i]->getobs(obs);

                (*_satdata_npp)[_map_nppdata_idx[_site][gsat]].addobs(obs, phase);
            }
        }
    }

} // namespace
