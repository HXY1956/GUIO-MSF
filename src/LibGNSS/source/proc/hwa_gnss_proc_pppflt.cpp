#include <iostream>
#include <sstream>
#include <chrono>
#include "hwa_gnss_proc_pppflt.h"
#include "hwa_gnss_proc_preproc.h"
#include "hwa_gnss_model_ppp.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_timesync.h"
#include "hwa_gnss_coder_bncobs.h"
#include "hwa_gnss_all_prec.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    // constructor
    // ----------
    gnss_proc_pppflt::gnss_proc_pppflt(std::string mark, set_base *gset)
        : gnss_proc_spp(mark, gset),
        gnss_proc_ppp(mark, gset),
        gnss_proc_sppflt(mark, gset),
        base_xml("kml"),
        _read(false),
        _flt(0),
        _enufile(0),
        _gpggafile(0),
        _smt(0),
        _kml(false),
        _restarted(false),
        _beg_end(true)
    {
        // Nothing but std::cout a std::string here

        gnss_proc_ppp::_get_settings();

        this->_setOut(); // set output && identify if smoothing requested

        _epoch.tsys(base_time::GPS);

        _grdStoModel = new gnss_model_random_walk();
        _ambStoModel = new gnss_model_random_walk();
        _grdStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_grd());
        _ambStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_amb());

        int npar = _param.parNumber();

        // Add tropospheric gradient parameters
        if (_tropo_grad)
        {
            base_par par_grdN(_site, par_type::GRD_N, ++npar, "");
            base_par par_grdE(_site, par_type::GRD_E, ++npar, "");
            par_grdN.setMF(_grd_mf);
            par_grdN.setMF(_ztd_mf);
            par_grdE.setMF(_grd_mf);
            par_grdE.setMF(_ztd_mf);
            _param.addParam(par_grdN);
            _param.addParam(par_grdE);
        }
    }

    gnss_proc_pppflt::gnss_proc_pppflt(std::string mark, set_base *gset, base_log spdlog)
        : gnss_proc_spp(mark, gset, spdlog),
        gnss_proc_ppp(mark, gset, spdlog),
        gnss_proc_sppflt(mark, gset, spdlog),
        base_xml("kml"),
        _read(false),
        _flt(0),
        _enufile(0),
        _gpggafile(0),
        _smt(0),
        _kml(false),
        _restarted(false),
        _beg_end(true)
    {
        //gnss_proc_ppp::_get_settings();
        this->_setOut(); // set output && identify if smoothing requested
        _epoch.tsys(base_time::GPS);
        _grdStoModel = new gnss_model_random_walk();
        _ambStoModel = new gnss_model_random_walk();
        _grdStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_grd());
        _ambStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_amb());
        int npar = _param.parNumber();

        if (_tropo_grad)
        {
            base_par par_grdN(_site, par_type::GRD_N, ++npar, "");
            base_par par_grdE(_site, par_type::GRD_E, ++npar, "");
            par_grdN.setMF(_grd_mf);
            par_grdN.setMF(_ztd_mf);
            par_grdE.setMF(_grd_mf);
            par_grdE.setMF(_ztd_mf);
            _param.addParam(par_grdN);
            _param.addParam(par_grdE);
        }

        npar = _param.parNumber();

        // Filling init parameter covariance matrix
        Symmetric tmp = _Qx;
        _Qx.resize(_param.parNumber());
        _Qx.setZero();

        for (int i = 0; i < tmp.rows(); i++)
            _Qx.set(tmp(i, i),i, i);

        for (size_t i = 0; i < _param.parNumber(); i++)
        {
            if (_param[i].parType == par_type::GRD_N)
                _Qx.set(_sig_init_grd * _sig_init_grd, i, i);
            else if (_param[i].parType == par_type::GRD_E)
                _Qx.set(_sig_init_grd * _sig_init_grd, i, i);
            else if (_param[i].parType == par_type::P1P2G_REC)
                _Qx.set(100.0 * 100, i, i);
            else if (_param[i].parType == par_type::P1P2E_REC)
                _Qx.set(100.0 * 100, i, i);
        }

        _reset_amb = dynamic_cast<set_flt *>(_set)->reset_amb();
        _reset_par = dynamic_cast<set_flt *>(_set)->reset_par();

    } // end constructor

    // destructor
    // ----------
    gnss_proc_pppflt::~gnss_proc_pppflt()
    {

        if (_grdStoModel)
            delete _grdStoModel;

        if (_flt)
        {
            if (_flt->is_open())
            {
                _flt->close();
            };
            delete _flt;
        }
        //LX added the enufile
        if (_enufile)
        {
            if (_enufile->is_open())
            {
                _enufile->close();
            };
            delete _enufile;
        }
        //LvHB added the GPGGA file
        if (_gpggafile)
        {
            if (_gpggafile->is_open())
            {
                _gpggafile->close();
            };
            delete _gpggafile;
        }
    if (_obsqualityfile)
    {
        if (_obsqualityfile->is_open())
        {
            _obsqualityfile->close();
        };
        delete _obsqualityfile;
    }

        std::map<OFMT, base_io_tcp *>::const_iterator it = _maptcp.begin();
        while (it != _maptcp.end())
        {
            delete it->second;
            ++it;
        }

        this->write(_kml_name);
    }

    // Process observation batch
    // ----------
    int gnss_proc_pppflt::processBatch(const base_time &beg_r, const base_time &end_r, bool prtOut)
    {
        // check for site, no object means no station to be processed
        if (_grec == nullptr)
        {
            std::ostringstream os;
            os << "ERROR: No object found (" << _site << "). Processing terminated!!! " << beg_r.str_ymdhms() << " -> " << end_r.str_ymdhms() << std::endl;
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, os.str());
            return -1;
        }

        int sign = 1;
        base_time begT;
        base_time endT;

        // check the processing direction
        if (!_beg_end)
        { 
            //from begin to end
            begT = end_r;
            endT = beg_r;
            sign = -1;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "gnss_proc_pppflt", "Filtering in end -> begin direction!");
        }
        else
        { 
            // from end to begin
            begT = beg_r;
            endT = end_r;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "gnss_proc_pppflt", "Filtering in begin -> end direction!");
        }

        // set the observation, navigation and station data
        _dop.set_data(_gnav, _gobs, _site);
        // check whether running or not
        _running = true;
        // Confirm the parameters to be estimated and set site name for all pars in gallpar
        _param.setSite(_site);
        // intval scale factor and sampling
        double subint = 0.1;
        if (_scale > 0)
            subint = 1.0 / _scale;
        if (_sampling > 1)
            subint = pow(10, floor(log10(_sampling)));
        // Get the settings for filter
        std::string smtModStr(dynamic_cast<set_flt *>(_set)->method_smt());
        std::string fltModStr(dynamic_cast<set_flt *>(_set)->method_flt());
        int smt_delay = dynamic_cast<set_flt *>(_set)->smt_delay();

        // Init current time
        base_time now(begT);

#ifdef DEBUG
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, begT.str_ymdhms("Filtering: ") + endT.str_ymdhms(" "));
#endif

        // Cout the information for beginning
        std::cout << _site << ": Start filtering: " << _site << " " << now.str_ymdhms() << " " << endT.str_ymdhms() << std::endl;
        // Peugeot of Continuous Cycle
        bool time_loop = true;
        //   high_resolution_clock::time_point chrono_t1 = high_resolution_clock::now();
        while (time_loop)
        {
            // Check whether the cycle is valid or not
            // case : begin to end
            if (_beg_end && (now < endT || now == endT))
                time_loop = true;
            else if (_beg_end && now > endT)
            {
                time_loop = false;
                break;
            }
            // case : end to begin
            if (!_beg_end && (now > endT || now == endT))
                time_loop = true;
            else if (!_beg_end && now < endT)
            {
                time_loop = false;
                break;
            }

            // synchronization
            if (now != endT)
            {
                if (!time_sync(now, _sampling, _scale, _spdlog))
                {                                    // now.sod()%(int)_sampling != 0 ){
                    now.add_dsec(sign * subint / 100); // add_dsec used for synchronization!

                    continue;
                }
                if (_sampling > 1)
                    now.reset_dsec();
            }
            // added by zhShen
            if (dynamic_cast<set_gproc *>(_set)->realtime())
            {
                if (dynamic_cast<set_gproc *>(_set)->realtime())
                {
                    while (now > _gobs->end_obs(_site))
                        base_time::gmsleep(10);
                    while (!dynamic_cast<gnss_all_prec *>(_gnav)->corr_avali(now))
                        base_time::gmsleep(10);
                }
                gnss_proc_preproc prep(_gobs, _set);
                prep.spdlog(_spdlog);
                base_time epo_prev = now - 10.0;
                if (_sampling > 1)
                    epo_prev.add_secs(-int(sign * 2 * _sampling)); // =<1Hz data
                else
                    epo_prev.add_dsec(-sign * 2 * _sampling); //  >1Hz data
                prep.ProcessBatch(_site, epo_prev, now, _sampling * 2, false);
            }
            // clean/collect/filter epoch data
            _data.erase(_data.begin(), _data.end());
            // Get obs data for new epoch
            _data = _gobs->obs(_site, now);
            // Check whether the data for current epoch is empty or not
            if (_data.size() == 0)
            {
                if (_sampling > 1)
                    now.add_secs(int(sign * _sampling)); // =<1Hz data
                else
                    now.add_dsec(sign * _sampling); //  >1Hz data
                continue;
            }
            // Update time
            base_time obsEpo = _data.begin()->epoch();
            _timeUpdate(obsEpo);

            // reseting params according to conf
            if (_reset_amb > 0)
            {
                if (now.sod() % _reset_amb == 0)
                {
                    _reset_ambig();
                }
            }
            if (_reset_par > 0)
            {
                if (now.sod() % _reset_par == 0)
                {
                    _reset_param();
                }
            }

            // save apriory coordinates
            _saveApr(obsEpo, _param, _Qx);

#ifdef _WIN32
            if (system("cls") != 0)
            {
                spdlog::error("Failed run systerm");
            }
#else
            if (system("clear") != 0)
            {
                spdlog::error("Failed run systerm");
            }
#endif
            std::cerr << now.str() << std::endl;
            // base_time::gmsleep(1000*1.5);
            int irc_epo = _processEpoch(obsEpo);

            if (irc_epo < 0)
            {
                _success = false;
                _removeApr(obsEpo);

                if (_sampling > 1)
                    now.add_secs(int(sign * _sampling)); // =<1Hz data
                else
                    now.add_dsec(sign * _sampling); //  >1Hz data

                  // repeat preprocessing in case of skipped epoch -> sampling changed
                gnss_proc_preproc prep(_gobs, _set);
                prep.spdlog(_spdlog);
                base_time epo_prev = now;
                if (_sampling > 1)
                    epo_prev.add_secs(-int(sign * 2 * _sampling)); // =<1Hz data
                else
                    epo_prev.add_dsec(-sign * 2 * _sampling); //  >1Hz data
                prep.ProcessBatch(_site, epo_prev, now, _sampling * 2, false);
                //       std::cout << "New preprocessing due to skipping epoch: " << epo_prev.str_hms() << " " << now.str_hms() << " " << _sampling*2 << std::endl;

                continue;
            }
            else
                _success = true;


            std::ostringstream os;
            bool savePrd = true;
            if (_smooth)
                savePrd = false;
            _prtOut(obsEpo, _param, _Qx, _data, os, line, savePrd);

            // Print flt results
            if (_flt && prtOut)
            {
                _flt->write(os.str().c_str(), os.str().size());
                _flt->flush();
            }

            _initialized = true;

            if (_sampling > 1)
                now.add_secs(int(sign * _sampling)); // =<1Hz data
            else
                now.add_dsec(sign * _sampling); //  >1Hz data

              // RTS Smooth
            if (_smooth)
            {
                if (smt_delay != 0 && now.sod() % smt_delay == 0)
                {
                    _backflt(prtOut);
                    _vfltdata.clear();
                }
            }

        } //end now

        _running = false;
        if (_smooth && smt_delay == 0)
        {
            _backflt(prtOut);
        }
        return 1;
    }

    // Process observation batch in forward <-> backward direction
    // ----------
    int gnss_proc_pppflt::processBatchFB(const base_time &beg_r, const base_time &end_r)
    {

        this->processBatch(beg_r, end_r, false);

        _beg_end = false;

        this->processBatch(beg_r, end_r, true);

        _beg_end = true;

        return 1;
    }

    // ----------------------------------------------------------------------------------
    // PROTECTED
    // ----------------------------------------------------------------------------------
    //

    // Update time for RDW processes
    // ---------------------------
    void gnss_proc_pppflt::_timeUpdate(const base_time &epo)
    {
        gnss_proc_sppflt::_timeUpdate(epo);

        //tropo gradient
        _grdStoModel->updateTime(epo);
        _grdStoModel->setTcurr(epo);

        // amb rndwk
        _ambStoModel->updateTime(epo);
        _ambStoModel->setTcurr(epo);
    }

    // Process one epoch
    // -----------------------------
    int gnss_proc_pppflt::_processEpoch(const base_time &runEpoch)
    {

        int irc = gnss_proc_sppflt::_processEpoch(runEpoch);
        return irc;
    }

    // add one satellite data to A, l, P - carrier phase
    // ---------------------------
    int gnss_proc_pppflt::_addObsL(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P)
    {
        gnss_sys gsys(satdata.gsys());
        GSYS gs = gsys.gsys();

        GOBSBAND b1, b2, b3, b4, b5;
        b1 = b2 = b3 = b4 = b5 = BAND;

        if (_auto_band)
        { // automatic dual band selection -> for Anubis purpose
            std::set<GOBSBAND> bands = satdata.band_avail();
            auto itBAND = bands.begin();
            if (bands.size() < 2)
                return -1;
            b1 = *itBAND;
            itBAND++;
            b2 = *itBAND;
        }
        else
        { // select fix defined band according the table
            b1 = gnss_sys::band_priority(gs, FREQ_1);
            b2 = gnss_sys::band_priority(gs, FREQ_2);
            b3 = gnss_sys::band_priority(gs, FREQ_3);
            b4 = gnss_sys::band_priority(gs, FREQ_4);
            b5 = gnss_sys::band_priority(gs, FREQ_5);
        }

        gnss_data_obs gobs1, gobs2, gobs3, gobs4, gobs5;

        _getgobs(satdata.sat(), TYPE_L, b1, gobs1);
        _getgobs(satdata.sat(), TYPE_L, b2, gobs2);
        _getgobs(satdata.sat(), TYPE_L, b3, gobs3);
        _getgobs(satdata.sat(), TYPE_L, b4, gobs4);
        _getgobs(satdata.sat(), TYPE_L, b5, gobs5);

        if (_phase)
        {

            bool com = true;
            if (_gnav)
                com = _gnav->com();

            double LIF, L1, L2, L3, L4, L5;
            LIF = L1 = L2 = L3 = L4 = L5 = 0.0;

            LIF = satdata.L3(gobs1, gobs2);
            L1 = satdata.obs_L(gobs1);
            L2 = satdata.obs_L(gobs2);
            if (b3 != BAND)
                L3 = satdata.obs_L(gobs3);
            if (b4 != BAND)
                L4 = satdata.obs_L(gobs4);
            if (b5 != BAND)
                L5 = satdata.obs_L(gobs5);

            if (_observ == OBSCOMBIN::IONO_FREE && double_eq(LIF, 0.0))
                return -1;
            if (_observ == OBSCOMBIN::RAW_SINGLE && double_eq(L1, 0.0))
                return -1;
            if ((_observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL) && (double_eq(L1, 0.0) || double_eq(L2, 0.0)))
                return -1;

            double weight_coef = _weightObs(satdata, gobs1);

            int addIobs = 1;
            if (L3 > 0)
                addIobs++;
            if (L4 > 0)
                addIobs++;
            if (L5 > 0)
                addIobs++;

            double sigPhase = 0.0;
            switch (gs)
            {
            case GPS:
                sigPhase = _sigPhaseGPS;
                break;
            case GLO:
                sigPhase = _sigPhaseGLO;
                break;
            case GAL:
                sigPhase = _sigPhaseGAL;
                break;
            case BDS:
                sigPhase = _sigPhaseBDS;
                break;
            case QZS:
                sigPhase = _sigPhaseQZS;
                break;
            default:
                sigPhase = 0.0;
            }

            // Create weight matrix
            P(iobs) = weight_coef * (1 / (sigPhase * sigPhase));
            if (_observ == OBSCOMBIN::RAW_DOUBLE ||
                _observ == OBSCOMBIN::RAW_ALL)
                P(iobs + 1) = weight_coef * (1 / (sigPhase * sigPhase));
            if (_observ == OBSCOMBIN::RAW_ALL)
                for (int i = 2; i <= addIobs; i++)
                    P(iobs + i) = weight_coef * (1 / (sigPhase * sigPhase));

            GOBS type1, type2, type3, type4, type5;

            if (gobs1.gattr().attr() == ATTR)
                type1 = satdata.id_phase(gobs1.gband().band());
            else
                type1 = gobs1.gobs();
            if (gobs2.gattr().attr() == ATTR)
                type2 = satdata.id_phase(gobs2.gband().band());
            else
                type2 = gobs2.gobs();
            if (gobs3.gattr().attr() == ATTR)
                type3 = satdata.id_phase(gobs3.gband().band());
            else
                type3 = gobs3.gobs();
            if (gobs4.gattr().attr() == ATTR)
                type4 = satdata.id_phase(gobs4.gband().band());
            else
                type4 = gobs4.gobs();
            if (gobs5.gattr().attr() == ATTR)
                type5 = satdata.id_phase(gobs5.gband().band());
            else
                type5 = gobs5.gobs();

            for (size_t iPar = 0; iPar < _param.parNumber(); iPar++)
            {
                if (_param[iPar].prn.compare(satdata.sat()) == 0)
                {

                    double modObsL = 0.0;

                    double obs_L = 0.0;
                    if (_param[iPar].parType == par_type::AMB_IF)
                    {
                        obs_L = LIF;
                        modObsL = _gModel->cmpObs(_epoch, _param, satdata, gobs1, com);
                    }
                    else if (_param[iPar].parType == par_type::AMB_L1)
                    {
                        obs_L = L1;
                        modObsL = _gModel->cmpObs(_epoch, _param, satdata, gobs1, com);
                    }
                    else if (_param[iPar].parType == par_type::AMB_L2)
                    {
                        obs_L = L2;
                        modObsL = _gModel->cmpObs(_epoch, _param, satdata, gobs2, com);
                    }
                    else if (_param[iPar].parType == par_type::AMB_L3)
                    {
                        obs_L = L3;
                        modObsL = _gModel->cmpObs(_epoch, _param, satdata, gobs3, com);
                    }
                    else if (_param[iPar].parType == par_type::AMB_L4)
                    {
                        obs_L = L4;
                        modObsL = _gModel->cmpObs(_epoch, _param, satdata, gobs4, com);
                    }
                    else if (_param[iPar].parType == par_type::AMB_L5)
                    {
                        obs_L = L5;
                        modObsL = _gModel->cmpObs(_epoch, _param, satdata, gobs5, com);
                    }

                    if (modObsL < 0)
                        return -1;

                    // Reset ambiguity in case of cycle slip
                    satdata.addslip(false);
                    if ((_param[iPar].parType == par_type::AMB_L1 && satdata.getlli(type1) >= 1) ||
                        (_param[iPar].parType == par_type::AMB_L2 && satdata.getlli(type2) >= 1) ||
                        (_param[iPar].parType == par_type::AMB_L3 && satdata.getlli(type3) >= 1) ||
                        (_param[iPar].parType == par_type::AMB_L4 && satdata.getlli(type4) >= 1) ||
                        (_param[iPar].parType == par_type::AMB_L5 && satdata.getlli(type5) >= 1) ||
                        (_param[iPar].parType == par_type::AMB_IF && (satdata.getlli(type1) >= 1 || satdata.getlli(type2) >= 1)))
                    {
                        satdata.addslip(true);
                        std::string prn = satdata.sat();
                        _param[iPar].value(obs_L - modObsL);
                        //std::cout << "cycle slip !!!!" << std::endl;
                        _Qx.set(_sigAmbig* _sigAmbig, iPar, iPar);
                        _newAMB[prn] = 1;
                        if (_smooth)
                            _slips.insert(prn);
                        //              std::cout << "Cycle slip: " << prn << " " << gobs2str(type1) << " " << gobs2str(type2) << " " << _epoch.str_hms() << std::fixed << setprecision(3) << " " << _param[iPar].value() << " " << LIF << " " << modObsL << std::endl;
                        if (_smooth)
                        {
                            Symmetric Qp = _fltdata.Qp();
                            Qp.set(_sigAmbig* _sigAmbig, iPar, iPar);
                            _fltdata.Qp(Qp);
                        }
                    }

                    // Create reduced measurement (prefit residuals)
                    if (_param[iPar].parType == par_type::AMB_L1 || _param[iPar].parType == par_type::AMB_IF)
                    {
                        l(iobs) = obs_L - _param[iPar].value() - modObsL;
                        /*  std::cout << "obs_L modObsL amb" << setprecision(5)
                          << setw(20) << obs_L
                          << setw(20) << modObsL
                          << setw(20) << _param[iPar].value() << std::endl;*/
                    }
                    if ((_observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL) && _param[iPar].parType == par_type::AMB_L2)
                    {
                        l(iobs + 1) = obs_L - _param[iPar].value() - modObsL;
                    }
                    if (_observ == OBSCOMBIN::RAW_ALL)
                    {
                        if (_param[iPar].parType == par_type::AMB_L3)
                            l(iobs + 2) = obs_L - _param[iPar].value() - modObsL;
                        if (_param[iPar].parType == par_type::AMB_L4)
                            l(iobs + 3) = obs_L - _param[iPar].value() - modObsL;
                        if (_param[iPar].parType == par_type::AMB_L5)
                            l(iobs + 4) = obs_L - _param[iPar].value() - modObsL;
                    }

                    // check satellite post shadow period
                    if (satdata.ecl())
                        return -1;
                }
            }

            // Create first design matrix
            for (unsigned int ipar = 0; ipar < _param.parNumber(); ipar++)
            {
                A(iobs, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs1);
                if (_observ == OBSCOMBIN::RAW_DOUBLE ||
                    _observ == OBSCOMBIN::RAW_ALL)
                    A(iobs + 1, ipar) = _param[ipar - 1].partial(satdata, _epoch, ell, gobs2);
                if (_observ == OBSCOMBIN::RAW_ALL)
                {
                    for (int i = 2; i <= addIobs; i++)
                    {
                        if (i == 2)
                            A(iobs + i, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs3);
                        if (i == 3)
                            A(iobs + i, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs4);
                        if (i == 4)
                            A(iobs + i, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs5);
                    }
                }
            }

            if (_observ == OBSCOMBIN::RAW_DOUBLE)
                iobs += 2;
            else if (_observ == OBSCOMBIN::RAW_ALL)
                iobs = iobs + addIobs + 1;
            else
                iobs++;
        }

        return 1;
    }

    // Applying tides
    // -------------------------------------------------------
    int gnss_proc_pppflt::_apply_tides(base_time &_epoch, Triple &xRec)
    {
        //gtrace("gnss_proc_pppflt::_apply_tides");

        if (_tides)
        {
            xRec = xRec + _tides->tide_earth(_epoch, xRec) + _tides->load_ocean(_epoch, _site, xRec) + _tides->tide_pole() + _tides->load_atmosph();
        }
        else
            return -1;
        //std::cout << std::fixed << setprecision(10) << "xRec = " << xRec[0] << " " << xRec[1] << " " << xRec[2] << std::endl;
        return 1;
    }

    // Predict state std::vector and covariance matrix
    // =======================================================
    void gnss_proc_pppflt::_predict()
    {
        // Predict coordinates, clock and troposphere
        // state std::vector is the same, covariance matrix is predicted with noise
        int i;

        gnss_proc_sppflt::_predict();

        i = _param.getParam(_site, par_type::GRD_N, "");
        if (i >= 0)
        {
            if (_cntrep == 1 && _initialized)
                _Qx.matrixW()(i, i) += _grdStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _grdStoModel->getQ();
        }

        i = _param.getParam(_site, par_type::GRD_E, "");
        if (i >= 0)
        {
            if (_cntrep == 1 && _initialized)
                _Qx.matrixW()(i, i) += _grdStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _grdStoModel->getQ();
        }

        // Predict ambiguity
        // Add or remove ambiguity parameter and appropriate rows/columns covar. matrix
        if (_phase)
            _syncAmb();

        //   std::cout << "Predicted Q: \n" << _Qx << std::endl;
        //   std::cout << "Predicted X: \n" << _param << std::endl;

        // Create _Noise matrix (Diag)
        if (_smooth)
        {
            Matrix tmp = _Noise.matrixR();
            _Noise.resize(_Qx.rows());
            _Noise.setZero();
            for (int i = 0; i < _Noise.cols(); i++)
            {
                if (i >= tmp.cols())
                    break;
                _Noise.matrixW()(i, i) = tmp(i, i);
            }
        }

    } // end prediction

    // Satelite position
    // ----------
    int gnss_proc_pppflt::_satPos(base_time &epo, gnss_data_sats &gsatdata)
    {
        if (gnss_proc_sppflt::_satPos(epo, gsatdata) < 0)
            return -1;
        return 1;
    }

    // Synchronize ambiguity with respect to observation data.
    // First new ambiguity parameters are added.
    // Second old ambiguity parameters are deleted;
    // -----------------------------------------------------
    void gnss_proc_pppflt::_syncAmb()
    {
        //gtrace("gnss_proc_pppflt::_syncAmb");

        for (std::map<std::string, int>::iterator it = _newAMB.begin(); it != _newAMB.end();)
        {
            if (it->second > 2)
                _newAMB.erase(it++);
            else
                ++it;
        }

        // Add ambiguity parameter and appropriate rows/columns covar. matrix
        set<std::string> mapPRN;

        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); ++it)
        { // loop over all satellites
            mapPRN.insert(it->sat());

            GSYS gs = it->gsys();

            GOBSBAND b1, b2, b3, b4, b5;
            b1 = b2 = b3 = b4 = b5 = BAND;

            if (_auto_band)
            { // automatic dual band selection -> for Anubis purpose
                set<GOBSBAND> bands = it->band_avail();
                auto itBAND = bands.begin();
                if (bands.size() < 2)
                    continue;
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
            }
            else
            { // select fix defined band according the table
                b1 = gnss_sys::band_priority(gs, FREQ_1);
                b2 = gnss_sys::band_priority(gs, FREQ_2);
                b3 = gnss_sys::band_priority(gs, FREQ_3);
                b4 = gnss_sys::band_priority(gs, FREQ_4);
                b5 = gnss_sys::band_priority(gs, FREQ_5);
            }

            gnss_data_obs gobs1, gobs2, gobs3, gobs4, gobs5;

            _getgobs(it->sat(), TYPE_L, b1, gobs1);
            _getgobs(it->sat(), TYPE_L, b2, gobs2);
            _getgobs(it->sat(), TYPE_L, b3, gobs3);
            _getgobs(it->sat(), TYPE_L, b4, gobs4);
            _getgobs(it->sat(), TYPE_L, b5, gobs5);

            double LIF, L1, L2, L3, L4, L5;
            LIF = L1 = L2 = L3 = L4 = L5 = 0.0;

            LIF = it->L3(gobs1, gobs2);
            L1 = it->obs_L(gobs1);
            L2 = it->obs_L(gobs2);
            if (b3 != BAND)
                L3 = it->obs_L(gobs3);
            if (b4 != BAND)
                L4 = it->obs_L(gobs4);
            if (b5 != BAND)
                L5 = it->obs_L(gobs5);

            if (_observ == OBSCOMBIN::RAW_SINGLE && double_eq(L1, 0.0))
                continue;
            if (_observ == OBSCOMBIN::RAW_DOUBLE && (double_eq(L1, 0.0) || double_eq(L2, 0.0)))
                continue;

            bool com = true;
            if (_gnav)
                com = _gnav->com();

            if (_observ == OBSCOMBIN::IONO_FREE && _param.getParam(_site, par_type::AMB_IF, it->sat()) < 0)
            {

                base_par newPar(it->site(), par_type::AMB_IF, _param.parNumber() + 1, it->sat());
                double cmpObs = _gModel->cmpObs(_epoch, _param, *it, gobs1, com);
                if (cmpObs < 0)
                    continue;

                newPar.value(LIF - cmpObs); // first ambiguity value

                _param.addParam(newPar);
                _newAMB[it->sat()] = 1;

                _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sigAmbig * _sigAmbig;
                //std::cout << "ADD AMB IF " << it->sat() << " " << it->epoch().str_ymdhms() << std::endl;
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "AMB_IF was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
            }
            else if (_observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL)
            {
                int newAmb = 0;
                // add L1 amb - everytime when RAW observations
                if (_param.getParam(_site, par_type::AMB_L1, it->sat()) < 0)
                {
                    double cmpObs = _gModel->cmpObs(_epoch, _param, *it, gobs1, com);
                    if (cmpObs < 0)
                        continue;
                    base_par newPar1(it->site(), par_type::AMB_L1, _param.parNumber() + 1, it->sat());
                    newPar1.value(L1 - cmpObs);
                    _param.addParam(newPar1);
                    _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                    _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sigAmbig * _sigAmbig;
                    //         std::cout << "ADD AMB L1 " << it->sat() << " " << it->epoch().str_ymdhms()  << " " << L1 - cmpObs << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "RAW AMB_L1 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                }

                // add L2 amb - when RAW DOUBLE or ALL frequency
                if (_observ != OBSCOMBIN::RAW_SINGLE && _param.getParam(_site, par_type::AMB_L2, it->sat()) < 0)
                {
                    double cmpObs = _gModel->cmpObs(_epoch, _param, *it, gobs2, com);
                    if (cmpObs < 0)
                        continue;
                    base_par newPar2(it->site(), par_type::AMB_L2, _param.parNumber() + 1, it->sat());
                    newPar2.value(L2 - cmpObs);
                    _param.addParam(newPar2);
                    _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                    _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sigAmbig * _sigAmbig;
                    //         std::cout << "ADD AMB L2 " << it->sat() << " " << it->epoch().str_ymdhms() << " " << L2 - cmpObs << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "RAW AMB_L2 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                }

                // add L3 amb - when RAW ALL frequency
                if (_observ != OBSCOMBIN::RAW_SINGLE && _observ != OBSCOMBIN::RAW_DOUBLE && _param.getParam(_site, par_type::AMB_L3, it->sat()) < 0 && L3 > 0.0)
                {
                    double cmpObs = _gModel->cmpObs(_epoch, _param, *it, gobs3, com);
                    if (cmpObs < 0)
                        continue;
                    base_par newPar3(it->site(), par_type::AMB_L3, _param.parNumber() + 1, it->sat());
                    newPar3.value(L3 - cmpObs);
                    _param.addParam(newPar3);
                    _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                    _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sigAmbig * _sigAmbig;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "RAW AMB_L3 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                }

                // add L4 amb - when RAW ALL frequency
                if (_observ != OBSCOMBIN::RAW_SINGLE && _observ != OBSCOMBIN::RAW_DOUBLE && _param.getParam(_site, par_type::AMB_L4, it->sat()) < 0 && L4 > 0.0)
                {
                    double cmpObs = _gModel->cmpObs(_epoch, _param, *it, gobs4, com);
                    if (cmpObs < 0)
                        continue;
                    base_par newPar4(it->site(), par_type::AMB_L4, _param.parNumber() + 1, it->sat());
                    newPar4.value(L4 - cmpObs);
                    _param.addParam(newPar4);
                    _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                    _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sigAmbig * _sigAmbig;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "RAW AMB_L4 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                }

                // add L5 amb - when RAW ALL frequency
                if (_observ != OBSCOMBIN::RAW_SINGLE && _observ != OBSCOMBIN::RAW_DOUBLE && _param.getParam(_site, par_type::AMB_L5, it->sat()) < 0 && L5 > 0.0)
                {
                    double cmpObs = _gModel->cmpObs(_epoch, _param, *it, gobs5, com);
                    if (cmpObs < 0)
                        continue;
                    base_par newPar5(it->site(), par_type::AMB_L5, _param.parNumber() + 1, it->sat());
                    newPar5.value(L5 - cmpObs);
                    _param.addParam(newPar5);
                    _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                    _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sigAmbig * _sigAmbig;
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "RAW AMB_L5 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                }

                if (newAmb)
                    _newAMB[it->sat()] = 1;
            }

        } // end loop over all observations

        // Remove ambiguity parameter and appropriate rows/columns covar. matrix

        for (unsigned int i = 0; i < _param.parNumber(); i++)
        {
            if (_param[i].parType == par_type::AMB_IF ||
                _param[i].parType == par_type::AMB_L1 ||
                _param[i].parType == par_type::AMB_L2 ||
                _param[i].parType == par_type::AMB_L3 ||
                _param[i].parType == par_type::AMB_L4 ||
                _param[i].parType == par_type::AMB_L5)
            {

                set<std::string>::iterator prnITER = mapPRN.find(_param[i].prn);
                if (prnITER == mapPRN.end())
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "AMB will be removed! For Sat PRN " + _param[i].prn + " Epoch: " + _epoch.str_ymdhms());

#ifdef DEBUG
                    std::cout << "AMB will be removed! For Sat PRN " << _param[i].prn
                        << " Epoch: " << _epoch.str_ymdhms() << std::endl;
#endif

                    _newAMB.erase(_param[i].prn);

                    _Qx.Matrix_remRC(_param[i].index, _param[i].index);
                    _param.delParam(i);
                    _param.reIndex();
                    i--;
                }
            }
        }

        return;
    }

    // Save apriory coordinates to products
    // -----------------------------
    void gnss_proc_pppflt::_saveApr(base_time &epoch, base_allpar &X, const Symmetric &Q)
    {
        //gtrace("gnss_proc_pppflt::_saveApr");

        // get CRD params
        Triple xyz;
        X.getCrdParam(_site, xyz);

        // CRD using eccentricities
        Triple xyz_ecc = xyz - _grec->eccxyz(epoch); // MARKER + ECC = ARP

        // get CRD rms  (XYZ)
        double Xrms = 0.0, Yrms = 0.0, Zrms = 0.0;
        int icrdx = _param.getParam(_site, par_type::CRD_X, "");
        int icrdy = _param.getParam(_site, par_type::CRD_Y, "");
        int icrdz = _param.getParam(_site, par_type::CRD_Z, "");
        if (Q(icrdx, icrdx) < 0)
            Xrms = -1;
        else
            Xrms = sqrt(Q(icrdx, icrdx));
        if (Q(icrdy, icrdy) < 0)
            Yrms = -1;
        else
            Yrms = sqrt(Q(icrdy, icrdy));
        if (Q(icrdz, icrdz) < 0)
            Zrms = -1;
        else
            Zrms = sqrt(Q(icrdz, icrdz));
        Triple crd_rms(Xrms, Yrms, Zrms);

        if (_allprod != 0)
        {
            std::shared_ptr<gnss_prod> prdcrd = _allprod->get(_site, base_data::POS, epoch);
            if (prdcrd)
            {
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->apr(xyz_ecc);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->apr_rms(crd_rms);
            }
            else
            {
                prdcrd = std::make_shared<gnss_prod_crd>(_spdlog, epoch);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->apr(xyz_ecc);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->apr_rms(crd_rms);
                _allprod->add(prdcrd, _site);
            }
        }
    }

    // Remove apriory coordinates from products
    // -----------------------------
    void gnss_proc_pppflt::_removeApr(base_time &epoch)
    {

        if (_allprod != 0)
        {
            _allprod->rem(_site, base_data::POS, epoch);
        }
    }

    // Set output & return if smooth
    // -----------------------------
    void gnss_proc_pppflt::_prtOut(base_time &epoch, base_allpar &X, const Symmetric &Q, std::vector<gnss_data_sats> &data, std::ostringstream &os, xml_node &node, bool saveProd)
    {

        // get CRD params
        Triple xyz, ell;
        X.getCrdParam(_site, xyz);
        xyz2ell(xyz, ell, false);

        // CRD using eccentricities
        Triple xyz_ecc = xyz - _grec->eccxyz(epoch); // MARKER + ECC = ARP

        // get CRD rms  (XYZ)
        double Xrms = 0.0, Yrms = 0.0, Zrms = 0.0;
        double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;
        if (Q(0, 0) < 0)
            Xrms = -1;
        else
            Xrms = sqrt(Q(0, 0));
        if (Q(1, 1) < 0)
            Yrms = -1;
        else
            Yrms = sqrt(Q(1, 1));
        if (Q(2, 2) < 0)
            Zrms = -1;
        else
            Zrms = sqrt(Q(2, 2));
        Triple crd_rms(Xrms, Yrms, Zrms);
        cov_xy = Q(1, 0);
        cov_xz = Q(2, 0);
        cov_yz = Q(2, 1);

        // get CLK param
        double clk = 0.0;
        int iclk = X.getParam(_site, par_type::CLK, "");
        if (iclk >= 0)
            clk = X[iclk].value();

        // get ZTD params
        double ZHD = 0.0, ZWD = 0.0, ZTD = 0.0, ZWDrms = 0.0;
        int iztd = X.getParam(_site, par_type::TRP, "");
        if (iztd >= 0)
        {
            ZHD = X[iztd].apriori(); //_gModel->tropoModel()->getZHD(ell, epoch);
            ZWD = X[iztd].value();
            ZTD = ZHD + ZWD;
            if (Q(iztd, iztd) < 0)
                ZWDrms = -1;
            else
                ZWDrms = sqrt(Q(iztd, iztd)); // _param is indexed from 0, but Q from 1 !
        }
        else
        {
            // test purpose only (if ZTD not estimated)
            if (_gModel->tropoModel() != 0)
            {
                ZWD = _gModel->tropoModel()->getZWD(ell, epoch);
                ZHD = _gModel->tropoModel()->getZHD(ell, epoch);
                ZTD = ZHD + ZWD;
            }
        }

        // get Tropo gradient
        int ign = X.getParam(_site, par_type::GRD_N, "");
        int ige = X.getParam(_site, par_type::GRD_E, "");
        double grdN = 0.0, grdE = 0.0;
        //  double rmsN = 0.0, rmsE = 0.0;
        if (ign >= 0 && ige >= 0)
        {
            grdN = X[ign].value();
            grdE = X[ige].value();
            //    rmsN = sqrt(Q(ign+1,ign+1));                  // _param is indexed from 0, but Q from 1 !
            //    rmsE = sqrt(Q(ige+1,ige+1));                  // _param is indexed from 0, but Q from 1 !
        }

        set<std::string> sat_list;
        for (auto it = _data.begin(); it != _data.end(); it++)
            sat_list.insert(it->sat());
        _dop.set_sats(sat_list);
        double gdop = -1;
        if (_dop.calculate(epoch, xyz) >= 0)
            gdop = _dop.gdop();

        set<std::string> ambs = X.amb_prns();
        int nsat = ambs.size();

        if (_allprod != 0 && saveProd)
        {
            std::shared_ptr<gnss_prod> prdcrd = _allprod->get(_site, base_data::POS, epoch);
            if (prdcrd)
            {
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->xyz(xyz_ecc);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->xyz_rms(crd_rms);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->cov(COV_XY, cov_xy);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->cov(COV_XZ, cov_xz);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->cov(COV_YZ, cov_yz);
            }
            else
            {
                prdcrd = std::make_shared<gnss_prod_crd>(_spdlog, epoch);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->xyz(xyz_ecc);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->xyz_rms(crd_rms);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->cov(COV_XY, cov_xy);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->cov(COV_XZ, cov_xz);
                std::dynamic_pointer_cast<gnss_prod_crd>(prdcrd)->cov(COV_YZ, cov_yz);
                _allprod->add(prdcrd, _site);
            }

            Triple Ell, XYZ;
            if (_param.getCrdParam(_site, XYZ) > 0)
            {
            }
            else if (_valid_crd_xml)
            {
                XYZ = _grec->crd_arp(_epoch);
            }
            xyz2ell(XYZ, Ell, false);
            int itrp = _param.getParam(_site, par_type::TRP, "");
            if (itrp >= 0)
                _param[itrp].apriori(_gModel->tropoModel()->getZHD(Ell, _epoch));
        }

        // write params
        if (_kml)
        {
            std::string val = base_type_conv::dbl2str(ell[1] * R2D, 11) + "," + base_type_conv::dbl2str(ell[0] * R2D, 11);
            this->_default_node(node, "coordinates", val.c_str(), false);
        }

        std::string str_dsec = base_type_conv::dbl2str(epoch.dsec());

        os <<
            // "=FLT " <<
            std::fixed << setprecision(4)
            //<< epoch.str_ymdhms() << str_dsec.substr(2) << setprecision(4)
            << epoch.str() << setprecision(4)
            // << " " << epoch.sow() + epoch.dsec()
            //<< " " << setw(13) << ell[0] * R2D          // [m]
            //<< " " << setw(13) << ell[1] * R2D         // [m]
            //<< " " << setw(13) << ell[2]       // [m]
            << " " << setw(13) << xyz_ecc[0] // [m]
            << " " << setw(13) << xyz_ecc[1] // [m]
            << " " << setw(13) << xyz_ecc[2] // [m]
            << " " << setw(7) << ZTD         // [m]
            << " " << setw(7) << grdN * 1000 // [mm]
            << " " << setw(7) << grdE * 1000 // [mm]
            << " " << setw(15) << clk        //<< (clk/CLIGHT)*1e6             // [m]
            << std::fixed << setprecision(5)
            << " " << setw(9) << Xrms   // [m]
            << " " << setw(9) << Yrms   // [m]
            << " " << setw(9) << Zrms   // [m]
            << " " << setw(7) << ZWDrms // [m]
            << std::fixed << setprecision(0)
            << " " << setw(2) << nsat // nsat
            << std::fixed << setprecision(1)
            << " " << setw(3) << gdop // gdop
            << std::fixed << setprecision(1)
            << " " << setw(5) << _sig_unit // m0
            << std::endl;

        return;
    }

    // Set output & return if smooth
    // -----------------------------
    void gnss_proc_pppflt::_setOut()
    {
        std::string tmp;
        tmp = dynamic_cast<set_out *>(_set)->outputs("flt");
        if (!tmp.empty() && !_read)
        {
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            _flt = new base_iof;
            _flt->tsys(base_time::GPS);
            _flt->mask(tmp);
            _flt->append(dynamic_cast<set_out *>(_set)->append());

            //if (!_flt->is_open())
            //{
            //    throw_logical_error("can not open : " + tmp);
            //}
        }

        //LX added
        // set enu file
        tmp = dynamic_cast<set_out *>(_set)->outputs("enu");
        if (!tmp.empty() && !_read)
        {
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            _enufile = new base_iof;
            _enufile->tsys(base_time::GPS);
            _enufile->mask(tmp);
            _enufile->append(dynamic_cast<set_out *>(_set)->append());
        }
    tmp = dynamic_cast<set_out*>(_set)->outputs("obsquality");
    if (!tmp.empty() && !_read)
    {
        base_type_conv::substitute(tmp, "$(rec)", _site, false);
        _obsqualityfile = new base_iof;
        _obsqualityfile->tsys(base_time::GPS);
        _obsqualityfile->mask(tmp);
        _obsqualityfile->append(dynamic_cast<set_out*>(_set)->append());
    }

        std::string port = dynamic_cast<set_out *>(_set)->output_port("flt");
        if (port.size() > 0)
        {
            base_io_tcp *flt = new base_io_tcp(base_type_conv::str2int(port), _spdlog);
            _maptcp.insert(std::make_pair(FLT_OUT, flt));
        }

        tmp = dynamic_cast<set_out *>(_set)->outputs("kml");
        if (!tmp.empty() && !_read)
        {
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            _kml_name = tmp;
            _kml = true;

            xml_node node, document, placemark, multigeometry;
            xml_node root = _doc;
            node = this->_default_node(root, _root.c_str());

            document = this->_default_node(node, "Document");

            std::string mode;
            std::string smooth = dynamic_cast<set_out *>(_set)->outputs("smt");
            if (!smooth.empty())
                mode = "_SMT";
            else
                mode = "_FLT";
            std::string desc = _site + mode;
            this->_default_node(document, "name", desc.c_str(), false);

            // for style in document
            // note(zhshen): P1-P3 is set to Fix Sol, and P4-P6 is set to Float Sol;
            xml_node doc_style1 = this->_default_node(document, "Style");
            std::string id1 = "P1";
            this->_default_attr(doc_style1, "id", id1);
            xml_node IconStyle = this->_default_node(doc_style1, "IconStyle");
            this->_default_node(IconStyle, "color", "ff00ff64");  // for std::fixed point
            this->_default_node(IconStyle, "scale", "0.7");
            xml_node Icon = this->_default_node(IconStyle, "Icon");
            this->_default_node(Icon, "href", "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png");

            xml_node BalloonStyle = this->_default_node(doc_style1, "BalloonStyle");
            this->_default_node(BalloonStyle, "color", "ffd5f3fa");  // for info
            xml_node text = BalloonStyle.append_child("text");
            std::string str("<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>");
            text.append_child(pugi::node_cdata).set_value(str.c_str());



            xml_node doc_style2 = document.append_copy(doc_style1);
            std::string id2 = "P2";
            this->_default_attr(doc_style2, "id", id2, true);
            IconStyle = this->_default_node(doc_style2, "IconStyle");
            this->_default_node(IconStyle, "color", "ff78c800", true);

            xml_node doc_style3 = document.append_copy(doc_style1);
            std::string id3 = "P3";
            this->_default_attr(doc_style3, "id", id3, true);
            IconStyle = this->_default_node(doc_style3, "IconStyle");
            this->_default_node(IconStyle, "color", "ffff9600", true);


            xml_node doc_style4 = document.append_copy(doc_style1);
            std::string id4 = "P4";
            this->_default_attr(doc_style4, "id", id4, true);
            IconStyle = this->_default_node(doc_style4, "IconStyle");
            this->_default_node(IconStyle, "color", "ffff6496", true);


            xml_node doc_style5 = document.append_copy(doc_style1);
            std::string id5 = "P5";
            this->_default_attr(doc_style5, "id", id5, true);
            IconStyle = this->_default_node(doc_style5, "IconStyle");
            this->_default_node(IconStyle, "color", "ffff00ff", true);


            xml_node doc_style6 = document.append_copy(doc_style1);
            std::string id6 = "P6";
            this->_default_attr(doc_style6, "id", id6, true);
            IconStyle = this->_default_node(doc_style6, "IconStyle");
            this->_default_node(IconStyle, "color", "ff0000ff", true);

            // for style in placemark
            placemark = this->_default_node(document, "Placemark");
            this->_default_node(placemark, "name", "Trajection", false);
            xml_node style = this->_default_node(placemark, "Style");
            xml_node LineStyle = this->_default_node(style, "LineStyle");
            this->_default_node(LineStyle, "color", "ff00ffff", false);
            this->_default_node(LineStyle, "width", "2", false);
            // multigeometry = this->_default_node(placemark, "MultiGeometry");
            line = this->_default_node(placemark, "LineString");
            this->_default_node(line, "coordinates");
        }

        //LvHB added in 20200312
        // set GPGGA file
        tmp = dynamic_cast<set_out *>(_set)->outputs("gpgga");
        if (!tmp.empty() && !_read)
        {
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            _gpggafile = new base_iof;
            _gpggafile->tsys(base_time::UTC);
            _gpggafile->mask(tmp);
            _gpggafile->append(dynamic_cast<set_out *>(_set)->append());
        }

        _read = true;
        return;
    }

    // Run smoothing
    // ---------------------
    void gnss_proc_pppflt::_backflt(bool prtOut)
    {

        return;
    }

} // namespace

// reset ambiguity
// ------------------
void gnss_proc_pppflt::_reset_ambig()
{
    std::vector<int> ind = _param.delAmb();
    _Qx.Matrix_rem(ind);
}

// Check and update  ambiguity  struct when the "_prepareData" called failure
// Called when the observation is not enough or else
// ------------------
void gnss_proc_pppflt::_check_ambig()
{
    // Find number of ambiguities in the observation data
    set<std::string> mapPRN;
    std::vector<gnss_data_sats>::iterator it;
    for (it = _data.begin(); it != _data.end(); ++it)
    { // loop over all observations
        mapPRN.insert(it->sat());
    }

    //Delete the ambiguity parameter is the satellite is missed in the epoch
    for (size_t i = 0; i < _param.parNumber(); i++)
    {
        if (_param[i].parType == par_type::AMB_IF)
        {

            set<std::string>::iterator prnITER = mapPRN.find(_param[i].prn);
            if (prnITER == mapPRN.end())
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "AMB will be removed for missed Sat PRN " + _param[i].prn + " Epoch: " + _epoch.str_ymdhms());

                _newAMB.erase(_param[i].prn);
                _Qx.Matrix_remRC(_param[i].index, _param[i].index);
                _param.delParam(i);
                _param.reIndex();
                i--; //One ambiguity is deleted, i --
            }
        }
    }
}

// reset all params
// ------------------
void gnss_proc_pppflt::_reset_param()
{
    std::vector<int> ind = _param.delAmb();
    _Qx.Matrix_rem(ind);
    _initialized = false;
}
