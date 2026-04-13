#include "hwa_gnss_proc_sppflt.h"
#include "hwa_base_timesync.h"
#include "hwa_gnss_prod_clk.h"
#include "hwa_gnss_model_spp.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    gnss_proc_sppflt::gnss_proc_sppflt(std::string mark, set_base *set)
        : gnss_proc_spp(mark, set),
          _minsat(static_cast<size_t>(SPP_MINSAT)),
          _filter(0),
          _smooth(false),
          _n_NPD_flt(0),
          _n_all_flt(0),
          _n_NPD_smt(0),
          _n_all_smt(0)
    {

        gnss_proc_spp::_get_settings();

        _smooth = dynamic_cast<set_flt *>(_set)->smooth();

        std::string fltModStr(dynamic_cast<set_flt *>(_set)->method_flt());
        if (fltModStr.compare("kalman") == 0)
            _filter = new gnss_proc_kalman();
        else if (fltModStr.compare("srcf") == 0)
            _filter = new gnss_proc_srf();

        _trpStoModel = new gnss_model_random_walk();
        _trpStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_ztd());

        //_ionStoModel = new gnss_model_white_noise(dynamic_cast<set_flt*>(_set)->noise_vion());
        _ionStoModel = new gnss_model_random_walk();
        _ionStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_vion());
        //LX added for GPS ifb

        _gpsStoModel = new gnss_model_random_walk();
        _gpsStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_gps());

        _gloStoModel = new gnss_model_random_walk();
        _gloStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_glo());

        _galStoModel = new gnss_model_random_walk();
        _galStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_gal());

        _bdsStoModel = new gnss_model_random_walk();
        _bdsStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_bds());

        _qzsStoModel = new gnss_model_random_walk();
        _qzsStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_qzs());

        _clkStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_clk());
        _crdStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_crd());

        _vBanc.resize(4);
        _vBanc.setZero();

        int ipar = 0;

        //added by xiongyun
        std::string refclk = dynamic_cast<set_gproc *>(_set)->ref_clk();
        if (refclk != "")
        {
            _crd_est = CONSTRPAR::EST;
        }

        // Add coordinates parameters
        if (_crd_est != CONSTRPAR::FIX)
        {
            _param.addParam(base_par(_site, par_type::CRD_X, ++ipar, ""));
            _param.addParam(base_par(_site, par_type::CRD_Y, ++ipar, ""));
            _param.addParam(base_par(_site, par_type::CRD_Z, ++ipar, ""));
        }

        // Add receiver clock parameter
        _param.addParam(base_par(_site, par_type::CLK, ++ipar, ""));

        // Add tropospheric wet delay parameter
        if (_tropo_est)
        {
            base_par par_trp(_site, par_type::TRP, ++ipar, "");
            par_trp.setMF(_ztd_mf);
            _param.addParam(par_trp);
        }

        // Filling init parameter covariance matrix
        _Qx.resize(_param.parNumber());
        _Qx.matrixW().setZero();
        double crdInit = _sig_init_crd;
        double ztdInit = _sig_init_ztd;
        for (unsigned int i = 0; i < _param.parNumber(); i++)
        {
            if (_param[i - 1].parType == par_type::CRD_X)
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            else if (_param[i - 1].parType == par_type::CRD_Y)
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            else if (_param[i - 1].parType == par_type::CRD_Z)
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            else if (_param[i - 1].parType == par_type::CLK)
                _Qx.matrixW()(i, i) = _clkStoModel->getQ() * _clkStoModel->getQ();
            else if (_param[i - 1].parType == par_type::TRP)
                _Qx.matrixW()(i, i) = ztdInit * ztdInit;
        }

        _resid_type = dynamic_cast<set_gproc *>(_set)->residuals();
        _auto_band = dynamic_cast<set_gproc *>(_set)->auto_band();
        _cbiaschar = dynamic_cast<set_gproc *>(_set)->cbiaschar();
        _frequency = dynamic_cast<set_gproc *>(_set)->frequency();

        _success = true;
        _ifb3_init = false;
        _ifb4_init = false;
        _ifb5_init = false;

    } // end constructor

    gnss_proc_sppflt::gnss_proc_sppflt(std::string mark, set_base *set, base_log spdlog,std::string mode)
        : gnss_proc_spp(mark, set, spdlog, mode),
          _minsat(static_cast<size_t>(SPP_MINSAT)),
          _filter(0),
          _smooth(false),
          _n_NPD_flt(0),
          _n_all_flt(0),
          _n_NPD_smt(0),
          _n_all_smt(0)
    {
        gnss_proc_spp::_get_settings();
        _smooth = dynamic_cast<set_flt *>(_set)->smooth();
        std::string fltModStr(dynamic_cast<set_flt *>(_set)->method_flt());
        if (fltModStr.compare("kalman") == 0)
            _filter = new gnss_proc_kalman();
        else if (fltModStr.compare("srcf") == 0)
            _filter = new gnss_proc_srf();
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + " not correct filter setting - check XML-config");
        }
        _trpStoModel = new gnss_model_random_walk();
        _trpStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_ztd());
        _ionStoModel = new gnss_model_random_walk();
        _ionStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_vion());
        _gpsStoModel = new gnss_model_random_walk();
        _gpsStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_gps());

        _gloStoModel = new gnss_model_random_walk();
        _gloStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_glo());

        _galStoModel = new gnss_model_random_walk();
        _galStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_gal());

        _bdsStoModel = new gnss_model_random_walk();
        _bdsStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_bds());

        _qzsStoModel = new gnss_model_random_walk();
        _qzsStoModel->setq(dynamic_cast<set_flt *>(_set)->rndwk_qzs());

        _clkStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_clk());
        _crdStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_crd());

        _vBanc.resize(4);
        _vBanc.setZero();

        int ipar = 0;
        std::string refclk = dynamic_cast<set_gproc *>(_set)->ref_clk();
        if (refclk != "")
        {
            _crd_est = CONSTRPAR::EST;
        }

        // Add coordinates parameters
        if (_crd_est != CONSTRPAR::FIX)
        {
            _param.addParam(base_par(_site, par_type::CRD_X, ++ipar, ""));
            _param.addParam(base_par(_site, par_type::CRD_Y, ++ipar, ""));
            _param.addParam(base_par(_site, par_type::CRD_Z, ++ipar, ""));
        }

        // Add receiver clock parameter
        _param.addParam(base_par(_site, par_type::CLK, ++ipar, ""));

        // Add tropospheric wet delay parameter
        if (_tropo_est)
        {
            base_par par_trp(_site, par_type::TRP, ++ipar, "");
            par_trp.setMF(_ztd_mf);
            _param.addParam(par_trp);
        }

        // Filling init parameter covariance matrix
        _Qx.resize(_param.parNumber());
        _Qx.setZero();
        double crdInit = _sig_init_crd;
        double ztdInit = _sig_init_ztd;
        for (unsigned int i = 0; i < _param.parNumber(); i++)
        {
            if (_param[i].parType == par_type::CRD_X)
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            else if (_param[i].parType == par_type::CRD_Y)
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            else if (_param[i].parType == par_type::CRD_Z)
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            else if (_param[i].parType == par_type::CLK)
                _Qx.matrixW()(i, i) = _clkStoModel->getQ() * _clkStoModel->getQ();
            else if (_param[i].parType == par_type::TRP)
                _Qx.matrixW()(i, i) = ztdInit * ztdInit;
        }

        _resid_type = dynamic_cast<set_gproc *>(_set)->residuals();
        _auto_band = dynamic_cast<set_gproc *>(_set)->auto_band();
        _cbiaschar = dynamic_cast<set_gproc *>(_set)->cbiaschar();
        _frequency = dynamic_cast<set_gproc *>(_set)->frequency();

        _success = true;
        _ifb3_init = false;
        _ifb4_init = false;
        _ifb5_init = false;

		// tyx debug
		//freq_index and band_index 
		_band_index[GPS] = dynamic_cast<set_gnss *>(_set)->band_index(GPS);
		_band_index[GAL] = dynamic_cast<set_gnss *>(_set)->band_index(GAL);
		_band_index[GLO] = dynamic_cast<set_gnss *>(_set)->band_index(GLO);
		_band_index[BDS] = dynamic_cast<set_gnss *>(_set)->band_index(BDS);
		_band_index[QZS] = dynamic_cast<set_gnss *>(_set)->band_index(QZS);

		_freq_index[GPS] = dynamic_cast<set_gnss *>(_set)->freq_index(GPS);
		_freq_index[GAL] = dynamic_cast<set_gnss *>(_set)->freq_index(GAL);
		_freq_index[GLO] = dynamic_cast<set_gnss *>(_set)->freq_index(GLO);
		_freq_index[BDS] = dynamic_cast<set_gnss *>(_set)->freq_index(BDS);
		_freq_index[QZS] = dynamic_cast<set_gnss *>(_set)->freq_index(QZS);

    } // end constructor
    // destructor
    // ----------
    gnss_proc_sppflt::~gnss_proc_sppflt()
    {

        if (_trpStoModel)
        {
            delete _trpStoModel;
            _trpStoModel = nullptr;
        }
        if (_filter)
        {
            delete _filter;
            _filter = nullptr;
        }
        if (_gloStoModel)
        {
            delete _gloStoModel;
            _gloStoModel = nullptr;
        }
        if (_galStoModel)
        {
            delete _galStoModel;
            _galStoModel = nullptr;
        }
        if (_bdsStoModel)
        {
            delete _bdsStoModel;
            _bdsStoModel = nullptr;
        }
        if (_qzsStoModel)
        {
            delete _qzsStoModel;
            _qzsStoModel = nullptr;
        }
        if (_clkStoModel)
        {
            delete _clkStoModel;
            _clkStoModel = nullptr;
        }
        if (_crdStoModel)
        {
            delete _crdStoModel;
            _crdStoModel = nullptr;
        }
        if (_ionStoModel)
        {
            delete _ionStoModel;
            _ionStoModel = nullptr;
        }
        if (_gpsStoModel)
        {
            delete _gpsStoModel;
            _gpsStoModel = nullptr;
        }
    }

    // Process observation batch
    // ----------
    int gnss_proc_sppflt::processBatch(const base_time &beg, const base_time &end)
    {

        if (beg > end)
        {
            std::cerr << _site << " - processing not started [beg > end]\n";
            return -1;
        }
        _dop.set_data(_gnav, _gobs, _site);

        base_time begT(beg);
        base_time endT(end);

        _param.setSite(_site);
        Triple xyz, neu, ell;

        double subint = 0.1;
        if (_scale > 0)
            subint = 1.0 / _scale;
        if (_sampling > 1)
            subint = pow(10, floor(log10(_sampling)));

        int sign = int(fabs(_sampling) / _sampling);

        base_time now(begT);

        base_time timeT(now);

        //   std::cout << _site << ": Start filtering: " << _site << " " << now.str_ymdhms() << std::endl;
        while (now < end || now == end)
        {
            // synchronization of now != end
            if (now != end)
            {
                if (!time_sync(now, _sampling, _scale, _spdlog))
                {                                      // now.sod()%(int)_sampling != 0 ){
                    now.add_dsec(sign * subint / 100); // add_dsec used for synchronization!
                    continue;
                }
                if (_sampling > 1)
                    now.reset_dsec();
            }

            // clean/collect/filter epoch data
            _data.erase(_data.begin(), _data.end());
            _data = _gobs->obs(_site, now);

            if (_data.size() == 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _site + now.str_ymdhms(" no observation found at epoch: "));
                if (_sampling > 1)
                    now.add_secs(int(sign * _sampling)); // =<1Hz data
                else
                    now.add_dsec(sign * _sampling); //  >1Hz data
                continue;
            }

            base_time obsEpo = _data.begin()->epoch();
            _timeUpdate(obsEpo);

            int irc_epo = _processEpoch(obsEpo);

            if (irc_epo < 0)
            {
                _success = false;
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _site + now.str_ymdhms(" epoch ") + " was not calculated");

                if (_sampling > 1)
                    now.add_secs(int(sign * _sampling)); // =<1Hz data
                else
                    now.add_dsec(sign * _sampling); //  >1Hz data
                continue;
            }
            else
                _success = true;

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _site + now.str_ymdhms(" processing epoch: "));

            if (_param.getCrdParam(_site, xyz) <= 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _site + now.str_ymdhms(" No coordinates included in params: "));
            }

            _map_crd[obsEpo] = xyz; // add by glfeng

            double clk = 0.0;
            double clk_std = 0.0;
            int iclk = _param.getParam(_site, par_type::CLK, "", FIRST_TIME, LAST_TIME);
            if (iclk >= 0)
            {
                clk_std = _Qx.matrixW()(_param[iclk].index, _param[iclk].index);
                clk = _param[iclk].value();
            }

            xyz2ell(xyz, ell, false);

            if (_allprod != 0)
            {
                std::shared_ptr<gnss_prod_crd> prd_crd = std::make_shared<gnss_prod_crd>(_spdlog, now); // now is rounded value !!
                std::shared_ptr<gnss_prod_clk> prd_clk = std::make_shared<gnss_prod_clk>(_spdlog, now); // now is rounded value !!
                prd_crd->xyz(xyz);
                prd_clk->clk(clk, clk_std);
                prd_crd->nSat(_nSat);
                prd_crd->nSat_excl(_nSat_excl);

                // store DOP
                std::set<std::string> sat_list;
                for (auto it = _data.begin(); it != _data.end(); it++)
                    sat_list.insert(it->sat());
                _dop.set_sats(sat_list);

                _dop.calculate(now, xyz);
                double gdop = _dop.gdop();
                double pdop = _dop.pdop();
                double hdop = _dop.hdop();
                double vdop = _dop.vdop();

                prd_crd->set_val("GDOP", gdop);
                prd_crd->set_val("PDOP", pdop);
                prd_crd->set_val("HDOP", hdop);
                prd_crd->set_val("VDOP", vdop);

                _allprod->add(prd_crd, _site);
                _allprod->add(prd_clk, _site);
            }

            _initialized = true;

            if (_sampling > 1)
                now.add_secs(int(sign * _sampling)); // =<1Hz data
            else
                now.add_dsec(sign * _sampling); //  >1Hz data
        }
        return 1;
    }

    // Process one epoch
    // -----------------------------
    int gnss_proc_sppflt::_processEpoch(const base_time &runEpoch)
    {

#ifdef DEBUG
        std::cout << "gsppflt: Epoch - full data size: " << _data.size() << " "
             << runEpoch.str_hms() << std::endl;
#endif

        // Check whether the receiver is valid or not
        if (_grec == nullptr)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "No receiver settings available!!!");
            return -1;
        }
        // Get the current epoch
        _epoch = runEpoch;
        // Get the prior coordinates
        Triple crdapr = _grec->crd_arp(_epoch);

        if ((double_eq(crdapr[0], 0.0) && double_eq(crdapr[1], 0.0) &&
            double_eq(crdapr[2], 0.0)) || crdapr.array().isNaN().any())
        {
            _valid_crd_xml = false;
        }
        else
        {
            _valid_crd_xml = true;
        }

        if (!_valid_crd_xml)
            _sig_init_crd = 100.0;

        Matrix A;
        Diag P;
        Vector l, dx, dx_el;
        Vector v_orig, v_norm;
        Symmetric Qsav, QsavBP;
        std::vector<gnss_data_sats>::iterator it;
        std::vector<GOBSTYPE> obstypes;
        base_allpar XsavBP;

        _cntrep = 0; // number of iterations caused by outliers

        do
        {
            if (_prepareData() < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Preparing data failed!");
                return -1;
            }

#ifdef DEBUG
            std::cout << "gsppflt: Epoch - proc data size: " << _data.size() << " " << runEpoch.str_hms() << std::endl;
            std::cout.flush();
#endif

            QsavBP.matrixW() = _Qx.matrixR();
            XsavBP = _param;
            _predict();

            if (_data.size() < _minsat)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Not enough visible satellites!");
                _restore(QsavBP, XsavBP);
                return -1;
            }

            Triple groundEll, groundXYZ;

            if (_param.getCrdParam(_site, groundXYZ, FIRST_TIME, LAST_TIME) > 0)
            {
            }
            else if (_valid_crd_xml)
            {
                groundXYZ = _grec->crd_arp(_epoch);
            }

            xyz2ell(groundXYZ, groundEll, false);

            // define a number of measurements
            unsigned int nObs = _data.size();
            unsigned int mult = 1;
            if (_observ == OBSCOMBIN::RAW_DOUBLE)
            {
                mult = 2;
                nObs *= 2;
            }
            if (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
            {
                mult = 2;
                nObs *= 5;
            } // reservation for 5 freq - not used raws will be removed
            if (_observ == OBSCOMBIN::IF_P1)
            {
                mult = 2;
                nObs *= 3;
            }
            if (_phase)
            {
                mult *= 2;
                nObs *= 2;
            } // code + phase

#ifdef DEBUG
            std::cout << "gsppflt: nObs: " << _data.size() << " " << nObs << " " << runEpoch.str_hms() << std::endl;
            std::cout.flush();
#endif

            unsigned int nPar = _param.parNumber();
            A.resize(nObs, nPar);
            A.setZero();
            l.resize(nObs);
            l.setZero();
            P.resize(nObs);
            P.setZero();
            dx.resize(nPar);
            dx.setZero();
            unsigned int iobs = 0;

            _frqNum.clear();

            // Create matrices and std::vectors for estimation
            // ----------------------------------------------------------------------------------
            // loop over all measurement in current epoch

            for (it = _data.begin(); it != _data.end();)
            {

                if (_addObsP(*it, iobs, groundEll, A, l, P) > 0)
                    obstypes.push_back(TYPE_C);
                else
                {
                    it = _data.erase(it);
                    continue;
                } // just return next iterator

                if (_addObsL(*it, iobs, groundEll, A, l, P) > 0)
                    obstypes.push_back(TYPE_L);
                else
                {
                    it = _data.erase(it);
                    continue;
                } // just return next iterator

                ++it; // increase iterator if not erased
            }

            //    std::cout << _epoch.str_hms() << std::endl;
            //    std::cout << "Predict cov: \n" << std::fixed << setprecision(0) << _Qx.matrixW() << std::endl;

			// tyx debug
			//std::cerr << iobs << "   " << _minsat * mult << std::endl;

            if (iobs < _minsat * mult)
            {
                //added by xiongyun to get erase sat
                for (it = _data.begin(); it != _data.end();)
                {
                    auto it_sat = find(_outlier_sat.begin(), _outlier_sat.end(), it->sat());
                    if (it_sat != _outlier_sat.end())
                    {
                        _outlier_sat.push_back(it->sat());
                    }
                    it++;
                }
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Not enough processable observations!");
                _restore(QsavBP, XsavBP);
                return -1;
            }

            // delete zero rows/cols
            A = A.block(0, 0, iobs, A.cols()).eval();
            P.matrixW() = P.matrixR().block(0,0,iobs,iobs).eval();
            l = l.segment(0, iobs).eval();

            Qsav.matrixW() = _Qx.matrixR();

            _filter->update(A, P, l, dx, _Qx);

            // increasing variance after update in case of introducing new ambiguity
            for (size_t iPar = 0; iPar < _param.parNumber(); iPar++)
            {
                if (_param[iPar].parType == par_type::AMB_IF)
                {
                    std::string sat = _param[iPar].prn;
                    if (_newAMB.find(sat) != _newAMB.end() && _cntrep == 1)
                    {
                        if (_newAMB[sat] == 1)
                            _Qx.matrixW()(iPar, iPar) += 10;
                        if (_newAMB[sat] == 2 && _Qx.matrixW()(iPar, iPar) > 0.01)
                            _Qx.matrixW()(iPar, iPar) += 1;
                        _newAMB[sat]++;
                    }
                }
            }

            // post-fit residuals
            v_orig = l - A * dx;

            // normalized post-fit residuals
            Matrix Qv = A * _Qx.matrixR() * A.transpose() + P.matrixR().inverse();
            v_norm.resize(v_orig.rows());
            for (int i = 0; i < v_norm.rows(); i++)
            {
                v_norm(i) = sqrt(1 / Qv(i, i)) * v_orig(i);
            }

            int freedom = A.rows() - A.cols();
            if (freedom < 1)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "No redundant observations!");
                //    _restore(QsavBP, XsavBP);
                //    return -1;
                freedom = 1;
            }

            Vector vtPv = v_orig.transpose() * P.matrixR() * v_orig;
            _sig_unit = vtPv(0) / freedom;

            /* Matrix N = A.transpose()*P*A; */
            /* Matrix Ql = A*N.i()*A.transpose(); */
            /* std::cout << std::fixed << setprecision(0) << Ql << std::endl; */
            /* int ooo; cin >> ooo; */

#ifdef DEBUG
            //    Symmetric Corr = cov2corr(_Qx.matrixW());
            std::cout << "Used satellites: ";
            for (unsigned int i = 0; i < _data.size(); i++)
                std::cout << _data[i].sat() << " ";
            std::cout << std::endl;
            std::cout << "Used parameters: ";
            for (unsigned int i = 0; i < _param.parNumber(); i++)
                std::cout << _param[i].str_type() << "_" << _param[i].prn << " ";
            std::cout << std::endl;
            std::cout << std::fixed << setprecision(3)
                 << _epoch.str_ymdhms() << std::endl
                 //         << " m0: " << setw(7) << _sig_unit << std::endl
                 << " dx: " << setw(7) << dx
                 //         << " l.rows(): " << l.rows() << std::endl
                 //         << setw(7) << l
                 //         << " v_orig.rows(): " << v_orig.rows()
                 //         << " v_orig: "  << setw(7) << v_orig
                 //         << " v_norm: "  << setw(7) << v_norm
                 //         << " A(" << A.rows() << "," << A.cols() << "):" << setw(7) << std::endl << A
                 //         << " P: "  << setw(7) << P
                 //         << " Q: "  << setw(7) << _Qx.matrixW()
                 //         << " C: "  << setw(7) << Corr
                 << " Param " << setw(7) << _param
                 << std::endl;
            std::cout.flush();

            // for(int i = 1; i <= _param.parNumber(); i++) std::cout << _param[_param.getParam(i)].str_type() << " "; std::cout << std::endl;
            // for(int i = 1; i <= _param.parNumber(); i++) std::cout << _param[_param.getParam(i)].str_type() << " " << Corr.row(i) << " "; std::cout << std::endl;

            //int ooo; cin >> ooo;
#endif

            // save post-fit residuals
            _save_residuals(v_norm, _data, RESIDTYPE::RES_NORM);
            _save_residuals(v_orig, _data, RESIDTYPE::RES_ORIG);

        } while (_gModel->outlierDetect(_data, _Qx, Qsav) != 0);

        _outlier_sat = dynamic_cast<gnss_model_spp *>(_gModel)->get_outlier_sat();

        // test purpose - print L1, L2, L3 residuals
        /* for(auto it = _data.begin(); it != _data.end(); it++){ */
        /*   std::vector<double> vres = it->residuals(RES_ORIG, TYPE_L); */
        /*   std::cout << it->epoch().str_ymdhms() << " " << it->sat(); */
        /*   for(auto itRES = vres.begin(); itRES != vres.end(); itRES++){ */
        /*      std::cout << std::fixed << setprecision(3) << setw(10) << *itRES; */
        /*   } */
        /*   if(vres.size() < 3) std::cout << setw(10) << 0.0; */
        /*   std::cout << std::endl; */
        /* } */

        _nSat_excl = _nSat - _data.size(); // number of excluded satellites due to different reasons

        try
        {
            _n_all_flt++;
            Matrix L = _Qx.matrixR().llt().matrixL();
        }
        catch (...)
        {
            //     std::cout << _site << " " << _epoch.str_ymdhms() << " NPDException " << std::endl;
            _n_NPD_flt++;
        }
        // end of covariance matrix testing

        if (_data.size() < _minsat)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + " skipped: " + base_type_conv::int2str(_data.size()) + " < _minsat)");
            //    std::cout << "Epoch " << _epoch.str_hms() << " not calculated due to _minsat." << std::endl;
            _restore(QsavBP, XsavBP);
            return -1;
        }

        // edit and save ZHD - for post-fit residual
        // (Must be before updating crd due to maintain consistency with a priory)
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
        {
            if (_gModel->tropoModel() != 0)
            {
                _param[itrp].apriori(_gModel->tropoModel()->getZHD(Ell, _epoch));
            }
        }

        // Edit and save parematres
        for (unsigned int iPar = 0; iPar < _param.parNumber(); iPar++)
        {
            _param[iPar].value(_param[iPar].value() + dx(_param[iPar].index));
        }

        return 1;
    }

    // weight coef for observations
    // ---------------------------
    double gnss_proc_sppflt::_weightObs(gnss_data_sats &satdata, gnss_data_obs &go)
    {
        double weight_coef = 0;

        switch (_weight)
        {
        case OBSWEIGHT::DEF_OBS_WEIGHT:
            std::cerr << "gsppflt: WeightObs (default) should not happened!\n";
            break;
        case OBSWEIGHT::EQUAL:
            weight_coef = 1;
            break;
        case OBSWEIGHT::SINEL: //weight_coef = sin(satdata.ele()); break;
            if (satdata.ele() * 180.0 / hwa_pi > 40)
                weight_coef = 1.0;
            else
                weight_coef = sin(satdata.ele()) * sin(satdata.ele());
            break;
        case OBSWEIGHT::SINEL2:
            weight_coef = pow(sin(satdata.ele()), 2);
            break;
        case OBSWEIGHT::SINEL4:
            weight_coef = pow(sin(satdata.ele()), 4);
            break;
        case OBSWEIGHT::PARTELE:
            if (satdata.ele_leo_deg() > 30)
                weight_coef = 1.0;
            else
                weight_coef = sin(satdata.ele_leo_deg()) * sin(satdata.ele_leo_deg());
            break;
        case OBSWEIGHT::CODPHA:
            if (go.is_code())
                weight_coef = pow(sin(satdata.ele()), 4);
            if (go.is_phase())
                weight_coef = pow(sin(satdata.ele()), 2);
            if (go.is_doppler())
                weight_coef = pow(sin(satdata.ele()), 2);
            break;
        case OBSWEIGHT::SNR: // SNR=SINEL in sppflt
            if (satdata.ele() * 180.0 / hwa_pi > 40)
                weight_coef = 1.0;
            else
                weight_coef = sin(satdata.ele()) * sin(satdata.ele());
            break;
        default:
            weight_coef = 0.0;
            break;
        }

        return weight_coef;
    }

    // add pseudo observations as a constrains
    // ---------------------------
    int gnss_proc_sppflt::_addPseudoZTD(unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P)
    {

        // Design Matrix
        if (_epoch > _ztd_begStat && _epoch < _ztd_endStat)
        {
#ifdef DEBUG
            std::cout << _epoch.str_ymdhms(" epoch: ") << std::endl;
            std::cout << _ztd_begStat.str_ymdhms("_ztd_begStat: ") << std::endl;
            std::cout << _ztd_endStat.str_ymdhms("_ztd_endStat: ") << std::endl;
#endif

            Matrix_addRC(A, A.rows() + 1, 0);
            int i = _param.getParam(_site, par_type::TRP, "");
            A(A.rows(), i + 1) = 1;

            // tropo (wet part) is constrained to this value
            double TRP_fix = 0;
            if (_gModel->tropoModel() != 0)
                TRP_fix = _aprox_ztd_xml - _gModel->tropoModel()->getZHD(ell, _epoch);

            // Reduced measurement
            addR(l, l.rows() + 1);
            l(l.rows()) = _param[i].value() - TRP_fix;

            // weight matrix
            P.Matrix_addRC(P.rows() + 1);
            P.matrixW()(P.rows() - 1, P.cols() - 1) = 99999;

            iobs++;
        }

        return 1;
    }

    // add data to A, l, P - code
    // ---------------------------
    int gnss_proc_sppflt::_addObsP(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P)
    {

        gnss_sys gsys(satdata.gsys());
        GSYS gs = gsys.gsys();

        GOBSBAND b1, b2, b3, b4, b5;
        b1 = b2 = b3 = b4 = b5 = BAND;

        if (_auto_band)
        { // automatic dual band selection -> for Anubis purpose
            std::set<GOBSBAND> bands = satdata.band_avail(_phase);
            auto itBAND = bands.begin();
            if (bands.size() < 2)
                return -1;
            b1 = *itBAND;
            itBAND++;
            b2 = *itBAND;
        }
        else
        { // select fix defined band according the table
			//b1 = gnss_sys::band_priority(gs, FREQ_1);
			//b2 = gnss_sys::band_priority(gs, FREQ_2);
			//b3 = gnss_sys::band_priority(gs, FREQ_3);
			//b4 = gnss_sys::band_priority(gs, FREQ_4);
			//b5 = gnss_sys::band_priority(gs, FREQ_5);
			// tyx debug
			b1 = _band_index[gs][FREQ_1];
			b2 = _band_index[gs][FREQ_2];
			b3 = _band_index[gs][FREQ_3];
			b4 = _band_index[gs][FREQ_4];
			b5 = _band_index[gs][FREQ_5];
        }

        gnss_data_obs gobs1, gobs2, gobs3, gobs4, gobs5;
        _getgobs(satdata.sat(), TYPE_C, b1, gobs1);
        _getgobs(satdata.sat(), TYPE_C, b2, gobs2);
        _getgobs(satdata.sat(), TYPE_C, b3, gobs3);
        _getgobs(satdata.sat(), TYPE_C, b4, gobs4);
        _getgobs(satdata.sat(), TYPE_C, b5, gobs5);
        //std::cout << satdata.epoch().str_hms() << " " << satdata.sat() << " " << gobs2str(satdata.id_range(gobs1.band())) << " "
        //                                                                 << gobs2str(satdata.id_range(gobs2.band())) << " "
        //                                                                 << gobs2str(satdata.id_range(gobs3.band())) << " "
        //                                                                 << gobs2str(satdata.id_range(gobs4.band())) << " "
        //                                                                 << gobs2str(satdata.id_range(gobs5.band())) << std::endl;

        double PIF, P1, P2, P3, P4, P5;
        PIF = P1 = P2 = P3 = P4 = P5 = 0.0;

        PIF = satdata.P3(gobs1, gobs2);
        P1 = satdata.obs_C(gobs1);
        P2 = satdata.obs_C(gobs2);
        if (b3 != BAND)
            P3 = satdata.obs_C(gobs3);
        if (b4 != BAND)
            P4 = satdata.obs_C(gobs4);
        if (b5 != BAND)
            P5 = satdata.obs_C(gobs5);

        if (_observ == OBSCOMBIN::IONO_FREE && double_eq(PIF, 0.0))
            return -1;
        if (_observ == OBSCOMBIN::RAW_SINGLE && double_eq(P1, 0.0))
            return -1;
        if ((_observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL) && (double_eq(P1, 0.0) || double_eq(P2, 0.0)))
            return -1;
        if (_observ == OBSCOMBIN::RAW_MIX && double_eq(P1, 0.0) /*&& double_eq(P2, 0.0)*/)
            return -1;

        double weight_coef_b1 = _weightObs(satdata, gobs1);
        double weight_coef_b2 = _weightObs(satdata, gobs2);
        double weight_coef_b3 = _weightObs(satdata, gobs3);
        double weight_coef_b4 = _weightObs(satdata, gobs4);
        double weight_coef_b5 = _weightObs(satdata, gobs5);
        double weight_coef = 0;

        if (_observ == OBSCOMBIN::IONO_FREE)
        {
            double koef1 = 0.0;
            double koef2 = 0.0;
            satdata.coef_ionofree(b1, koef1, b2, koef2);
            weight_coef = (weight_coef_b1 + weight_coef_b2) / 2;
        }
        else
            weight_coef = weight_coef_b1;

        int addIobs = 1;
        if (P3 > 0)
            addIobs++;
        if (P4 > 0)
            addIobs++;
        if (P5 > 0)
            addIobs++;
        if (_observ == OBSCOMBIN::RAW_MIX)
        {
            if (double_eq(P2, 0.0))
                addIobs = 0;
            else
                addIobs = 1;
        }

        int usedFrq = addIobs + 1;
        if (_observ == OBSCOMBIN::IONO_FREE || _observ == OBSCOMBIN::RAW_SINGLE)
            _frqNum[satdata.sat()] = 1;
        if (_observ == OBSCOMBIN::RAW_DOUBLE)
            _frqNum[satdata.sat()] = 2;
        if (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
            _frqNum[satdata.sat()] = usedFrq;

        double sigCode = 0.0;
        switch (gs)
        {
        case GPS:
            sigCode = _sigCodeGPS;
            break;
        case GLO:
            sigCode = _sigCodeGLO;
            break;
        case GAL:
            sigCode = _sigCodeGAL;
            break;
        case BDS:
            sigCode = _sigCodeBDS;
            break;
        case QZS:
            sigCode = _sigCodeQZS;
            break;
        default:
            sigCode = 0.0;
        }

        // Create weight matrix
        P.matrixW()(iobs, iobs) = weight_coef * (1 / (sigCode * sigCode));
        if (_observ == OBSCOMBIN::RAW_DOUBLE ||
            _observ == OBSCOMBIN::RAW_ALL)
        {
            P.matrixW()(iobs + 1, iobs + 1) = weight_coef_b2 * (1 / (sigCode * sigCode));
        }
        if (_observ == OBSCOMBIN::RAW_ALL)
        {
            for (int i = 2; i <= addIobs; i++)
            {
                if (i == 2)
                    P.matrixW()(iobs + i, iobs + i) = weight_coef_b3 * (1 / (sigCode * sigCode));
                if (i == 3)
                    P.matrixW()(iobs + i, iobs + i) = weight_coef_b4 * (1 / (sigCode * sigCode));
                if (i == 4)
                    P.matrixW()(iobs + i, iobs + i) = weight_coef_b5 * (1 / (sigCode * sigCode));
            }
        }
        if (_observ == OBSCOMBIN::RAW_MIX)
        {
            for (int i = 1; i <= addIobs; i++)
            {
                if (i == 1)
                    P.matrixW()(iobs + i, iobs + i) = weight_coef_b2 * (1 / (sigCode * sigCode));
            }
        }
        bool com = true;
        if (_gnav)
            com = _gnav->com();

        //Create reduced measurements (prefit residuals)
        double modObsP = 0.0;
        if (_observ == OBSCOMBIN::IONO_FREE)
        {
            modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs1, com);
            if (modObsP < 0)
                return -1;
            _applyDCB(satdata, PIF, &gobs1, &gobs2);
            l(iobs) = PIF - modObsP;
            //std::cout << "PIF modObsP:" << setprecision(5) << std::fixed
            // << setw(20) << PIF
            // << setw(20) << modObsP << std::endl;
        }
        else if (_observ == OBSCOMBIN::RAW_SINGLE)
        {
            modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs1, com);
            if (modObsP < 0)
                return -1;
            _applyDCB(satdata, P1, &gobs1);
            l(iobs) = P1 - modObsP;
        }
        else if (_observ == OBSCOMBIN::RAW_DOUBLE)
        {
            modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs1, com);
            if (modObsP < 0)
                return -1;
            _applyDCB(satdata, P1, &gobs1);
            l(iobs) = P1 - modObsP;
            modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs2, com);
            if (modObsP < 0)
                return -1;
            _applyDCB(satdata, P2, &gobs2);
            l(iobs + 1) = P2 - modObsP;
        }
        else if (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
        {
            for (int i = 0; i <= addIobs; i++)
            {
                double Pi = 0.0;
                if (i == 0)
                {
                    modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs1, com);
                    _applyDCB(satdata, P1, &gobs1);
                    Pi = P1;
                }
                if (i == 1)
                {
                    modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs2, com);
                    _applyDCB(satdata, P2, &gobs2);
                    Pi = P2;
                }
                if (i == 2)
                {
                    modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs3, com);
                    Pi = P3;
                }
                if (i == 3)
                {
                    modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs4, com);
                    Pi = P4;
                }
                if (i == 4)
                {
                    modObsP = _gModel->cmpObs(_epoch, _param, satdata, gobs5, com);
                    Pi = P5;
                }
                if (double_eq(modObsP, -1.0) || double_eq(Pi, 0.0))
                    continue; // double_eq(modObsP, -1.0) modified by zhshen
                l(iobs + i) = Pi - modObsP;
            }
        }

        // Create first design matrix
        if (gs == GLO ||
            gs == GPS ||
            gs == GAL ||
            gs == BDS ||
            gs == QZS)
        {

            for (unsigned int ipar = 0; ipar < _param.parNumber(); ipar++)
            {
                A(iobs, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs1);
                if (_observ == OBSCOMBIN::RAW_DOUBLE ||
                    _observ == OBSCOMBIN::RAW_ALL)
                    A(iobs + 1, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs2);
                if (_observ == OBSCOMBIN::RAW_MIX && addIobs == 1)
                    A(iobs + 1, ipar) = _param[ipar].partial(satdata, _epoch, ell, gobs2);
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
        }
        else
            return -1;

#ifdef DEBUG
        std::cout << satdata.sat() << std::fixed << setprecision(10)
             << "  PIF: (" << BAND_1 << "," << BAND_2 << ") :" << setw(12) << satdata.P3(b1, b2)
             << "  cmpObs: " << setw(12) << modObsP
             << std::endl;
#endif

        if (_observ == OBSCOMBIN::RAW_DOUBLE)
            iobs += 2;
        else if (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
            iobs = iobs + addIobs + 1;
        else
            iobs++;

        return 1;
    }

    // add data to A, l, P - carrier phase
    // ---------------------------
    int gnss_proc_sppflt::_addObsL(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P)
    {

        return 1;
    }

    // Prepare data:
    // filter, bancroft, members in gsatdata
    // ----------
    int gnss_proc_sppflt::_prepareData()
    {

        std::vector<gnss_data_sats>::iterator it = _data.begin();
        while (it != _data.end())
        {
            switch (_gnss)
            {
            case GNS:
                break;
            case GPS:
                if (it->gsys() != GPS)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            case GLO:
                if (it->gsys() != GLO)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            case GAL:
                if (it->gsys() != GAL)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            case BDS:
                if (it->gsys() != BDS)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            case QZS:
                if (it->gsys() != QZS)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            case IRN:
                if (it->gsys() != IRN)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            case SBS:
                if (it->gsys() != SBS)
                {
                    it = _data.erase(it);
                    continue;
                }
                break;
            default:
                continue;
            }
            it++;
        }

        Matrix BB;

        BB.resize(_data.size(), 4);
        BB.setZero();
        int iobs = 0;
        std::vector<gnss_data_sats>::iterator iter = _data.begin();

        _nSat = _data.size(); // configured GNSS is considered (see erase in above swich)

        while (iter != _data.end())
        {

            GSYS gs = iter->gsys();

            GOBSBAND b1, b2;

            if (_auto_band)
            {
                std::set<GOBSBAND> bands;
                bands = iter->band_avail(_phase);
                auto itBAND = bands.begin();
                if (bands.size() < 2)
                {
                    iter++;
                    continue;
                }
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
            }
            else
            {
                //b1 = gnss_sys::band_priority(gs, FREQ_1);
                //b2 = gnss_sys::band_priority(gs, FREQ_2);
				// tyx debug
				b1 = _band_index[gs][FREQ_1];
				b2 = _band_index[gs][FREQ_2];
            }

            iter->spdlog(_gspdlog);

            // check data availability
            double P3 = iter->P3(b1, b2);
            double L3 = iter->L3(b1, b2);

            if (double_eq(P3, 0.0) && (_observ == OBSCOMBIN::IF_P1 || _observ == OBSCOMBIN::RAW_MIX || _observ == OBSCOMBIN::RAW_SINGLE))
            {
                L3 = iter->obs_L(gnss_data_band(b1, GOBSATTR::ATTR));
                if (!double_eq(L3, 0.0))
                    P3 = iter->obs_C(gnss_data_band(b1, GOBSATTR::ATTR));
            }

#ifdef DEBUG
            std::cerr << "iter->epoch/_epoch: " << iter->epoch().str_hms() << _epoch.str_hms(" / ")
                 << " " << gnss_sys::gsys2str(gs) << " " << iter->sat() << " " << iter->channel()
                 << " " << std::fixed << setprecision(3) << P3 << " " << L3 << std::endl;
#endif

            if (double_eq(L3, 0.0) && _phase)
            {
                iter = _data.erase(iter); // !!!! OK u vektoru funguje bez ++ (erase zvysuje pointer !)
                continue;
            }

            if (double_eq(P3, 0.0))
            {
                iter = _data.erase(iter); // !!!! OK u vektoru funguje bez ++ (erase zvysuje pointer !)
                continue;
            }

            if (_satPos(_epoch, *iter) < 0)
            {
#ifdef DEBUG
                std::cerr << "prepareData: sat pos not calculated "
                     << _epoch.str_ymdhms(_site + " " + iter->sat() + " " + base_type_conv::int2str(_data.size()) + " ")
                     << std::endl;
                std::cerr.flush();
#endif
                iter = _data.erase(iter); // !!!! OK u vektoru funguje bez ++ (erase zvysuje pointer !)
            }
            else
            {
                BB(iobs, 0) = iter->satcrd()[0];
                BB(iobs, 1) = iter->satcrd()[1];
                BB(iobs, 2) = iter->satcrd()[2];
                //BB(iobs, 4) = iter->P3(b1, b2) + iter->clk();
                BB(iobs, 3) = P3 + iter->clk();
                iobs++;
                iter++;
                continue;
            }
        }

        if (_data.size() < _minsat)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _epoch.str_ymdhms(_site + " epoch ") + " skipped (Bancroft not calculated: " + base_type_conv::int2str(_data.size()) + " < _minsat)");
            //      std::cout << gnss_sys::gsys2str(_gnss) << " " << _epoch.str_ymdhms( _site + " epoch ") << " " << base_type_conv::int2str(_data.size()) <<  " skipped (Bancroft not calculated: _data.size < _minsat)" << std::endl;
            return -1;
        }

        BB = BB.block(0,0,iobs,BB.cols()).eval(); // delete zero rows

        if (BB.rows() < static_cast<int>(_minsat))
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _epoch.str_ymdhms(_site + " epoch ") + " skipped (Bancroft not calculated: BB.rows < _minsat)");
            //      std::cout << _epoch.str_ymdhms( _site + " epoch ") << base_type_conv::int2str(_data.size()) <<  " skipped (Bancroft not calculated: _data.size < _minsat)" << std::endl;
            return -1;
        }

        _vBanc.setZero();
        gbancroft(BB, _vBanc);

        if (!_initialized && (_vBanc.segment(0, 3) - _grec->crd_arp(_epoch)).norm() > 1000)
            _valid_crd_xml = false;

        if (_valid_crd_xml && !_initialized)
        {
            _vBanc(0) = _grec->crd_arp(_epoch)[0];
            _vBanc(1) = _grec->crd_arp(_epoch)[1];
            _vBanc(2) = _grec->crd_arp(_epoch)[2];
        }

        Triple test_xyz{ _vBanc[0], _vBanc[1], _vBanc[2] };
        Triple test_ell;
        xyz2ell(test_xyz, test_ell, true);
        double radius = test_xyz.norm();
        if (radius < B_WGS - 500)
        {
            std::string warning = "WARNING: Unexpected site (" + _site + ") coordinates from Bancroft. Orbits/clocks or code observations should be checked.";
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, warning + _epoch.str_ymdhms(" Epoch "));
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, warning + _epoch.str_ymdhms(" Epoch "));
            if (!_phase)
            {
                return -1;
            }
        }

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << std::endl;
        std::cout.flush();
        //   std::cout << "\nBB\n" << std::fixed << setprecision(10) << BB << std::endl;
        std::cout << "vBanc\n"
             << std::fixed << setprecision(10) << _vBanc << std::endl;
        std::cout.flush();
        //   std::cout << "Clk [mili sec]" << std::fixed << setprecision(10) << _vBanc(4)*1000/CLIGHT << std::endl;
        //   std::cout << "_Qx.matrixW()\n"  << std::fixed << setprecision(3)<< _Qx.matrixW() << std::endl;    std::cout.flush();
        //   int ooo; cin >> ooo;
#endif

        //Compute sat elevation and rho
        iter = _data.begin();
        while (iter != _data.end())
        {
            Triple xyz_r, xyz_s, neu_s;
            xyz_s = iter->satcrd();
            if (_crd_est == CONSTRPAR::FIX && _valid_crd_xml)
            {
                xyz_r = _grec->crd_arp(_epoch);
            }
            else if (_crd_est == CONSTRPAR::EST || _crd_est == CONSTRPAR::KIN || _crd_est == CONSTRPAR::SIMU_KIN)
            {
                if (!_initialized)
                {
                    xyz_r = _vBanc.segment(0, 3);
                }
                else
                {
                    if (_pos_kin || _crd_est == CONSTRPAR::KIN || _crd_est == CONSTRPAR::SIMU_KIN)
                    {
                        xyz_r = _vBanc.segment(0, 3);
                    }
                    else
                        _param.getCrdParam(_site, xyz_r);
                }
            }
            else
            {
                std::cerr << "Undefined CRD constraining" << std::endl;
                return -1;
            }

#ifdef DEBUG
            std::cout << "Coordinates: " << iter->sat() << " " << iter->epoch().str_hms()
                 << std::fixed << setprecision(10) << std::endl
                 << " X_R " << setw(12) << xyz_r[0]
                 << " Y_R " << setw(12) << xyz_r[1]
                 << " Z_R " << setw(12) << xyz_r[2] << std::endl
                 << " X_S " << setw(12) << xyz_s[0]
                 << " Y_S " << setw(12) << xyz_s[1]
                 << " Z_S " << setw(12) << xyz_s[2] << std::endl;
            std::cout.flush();
            int ooo;
            cin >> ooo;
#endif

            Triple xyz_rho = xyz_s - xyz_r;
            Triple ell_r;
            xyz2ell(xyz_r, ell_r, false);

            xyz2neu(ell_r, xyz_rho, neu_s);

#ifdef DEBUG // modified by zhshen \
             // debug
             /*std::cout << _epoch.str_ymdhms() << std::endl
                << std::fixed << setprecision(5)
                << setw(20) << xyz_r << std::endl;*/
#endif       // DEBUG

            // Earth rotation correction
            Triple xRec;
            double rho0 = sqrt(pow(xyz_r[0] - xyz_s[0], 2) + pow(xyz_r[1] - xyz_s[1], 2) + pow(xyz_r[2] - xyz_s[2], 2));
            double dPhi = OMEGA * rho0 / CLIGHT;
            xRec[0] = xyz_r[0] * cos(dPhi) - xyz_r[1] * sin(dPhi);
            xRec[1] = xyz_r[1] * cos(dPhi) + xyz_r[0] * sin(dPhi);
            xRec[2] = xyz_r[2];

#ifdef DEBUG // modified by zhshen              \
             // debug                           \
             //std::cout << std::fixed << setprecision(5) \
             //    << setw(20) << xRec << std::endl;

#endif // !1

            // Apply tides
            _apply_tides(_epoch, xRec);

            double tmp = (iter->satcrd() - xRec).norm();

#ifdef DEBUG // modified by zhshen                 \
             // debug                              \
             //std::cout << std::fixed << setprecision(5)    \
             //    << setw(20) << xRec \
             //    << setw(20) << iter->satcrd() << std::endl;
#endif

            iter->addrho(tmp);
            //std::cout << iter->sat() << " " << iter->epoch().str_ymdhms() << std::fixed << setprecision(3) << " " << neu_s[0] << " " << neu_s[1] << " " << neu_s[2] << std::endl;
            double NE2 = neu_s[0] * neu_s[0] + neu_s[1] * neu_s[1];
            double ele = acos(sqrt(NE2) / iter->rho());
            if (neu_s[2] < 0.0)
            {
                ele *= -1.0;
            }

            if (sqrt(NE2) / iter->rho() > 1.0)
                iter->addele(0.0);
            else
                iter->addele(ele);

            double azi = atan2(neu_s[1], neu_s[0]);
            if (azi < 0)
                azi += 2 * hwa_pi;
            iter->addazi_rec(azi);
            if (!_use_ecl)
                iter->addecl(_lastEcl);

            // check elevation cut-off
            if (iter->ele_deg() < _minElev)
            {
                std::ostringstream os;
                os << "Erasing " << iter->sat() << " data due to low elevation angle (ele = " << std::fixed << std::setprecision(1) << iter->ele_deg()
                   << ") " << iter->epoch().str_ymdhms();
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, os.str());
                iter = _data.erase(iter); // !!!! zveda iterator !!!!
                if (_data.size() < _minsat)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + base_type_conv::int2str(_data.size()) + " skipped (data.size < _minsat)");
                    return -1;
                }
                continue;
            }

            // Printing beta and orbit angles for deep verbosity
            std::ostringstream os;
            os << iter->sat() << " " << iter->epoch().str_ymdhms() << " " << std::fixed << std::setprecision(1) << iter->beta() << " " << iter->orb_angle() << std::endl;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, os.str());

            // check satellite eclipsing
            if (iter->ecl())
            {
                std::ostringstream os;
                os << "Erasing " << iter->sat() << " data due to satellite eclipsing (beta = " << std::fixed << std::setprecision(1) << iter->beta()
                   << " ,orbit angle = " << iter->orb_angle() << ") " << iter->epoch().str_ymdhms();
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, os.str());
                iter = _data.erase(iter); // !!!! zveda iterator !!!!
                if (_data.size() < _minsat)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + base_type_conv::int2str(_data.size()) + " skipped (data.size < _minsat)");
                    return -1;
                }
            }
            else
                iter++;
        }
        return 1;
    }

    // Tides are not applied in SPP
    // ---------------------------------
    int gnss_proc_sppflt::_apply_tides(base_time &_epoch, Triple &xRec)
    {
        return 1;
    }

    // Predict state std::vector and covariance matrix
    // =======================================================
    void gnss_proc_sppflt::_predict()
    {

        _cntrep++;

        // add/remove inter system bias
        _syncSys();

        // add/remove ionosphere delay
        _syncIono();

        // add/remove inter-frequency biases
        if (_frequency >= 3)
            _syncIFB();

        _Noise.resize(_Qx.matrixW().rows());
        _Noise.setZero();

        // Predict coordinates, clock and troposphere
        // state std::vector is the same, covariance matrix is predicted with noise
        int i;

        double crdInit = _sig_init_crd;
        double ztdInit = _sig_init_ztd;

        i = _param.getParam(_site, par_type::CRD_X, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _param[i].value(_vBanc(0));
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            }
            else
            {
                if (_epoch > _crd_begStat && _epoch < _crd_endStat)
                {
                    if (_smooth)
                        _Noise.matrixW()(i, i) = 0.0;
                }
                else
                {
                    if (_pos_kin)
                        _param[i].value(_vBanc(0));
                    if (_cntrep == 1 && _success)
                        _Qx.matrixW()(i, i) += _crdStoModel->getQ() * _crdStoModel->getQ();
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
                }
            }
        }

        i = _param.getParam(_site, par_type::CRD_Y, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _param[i].value(_vBanc(1));
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            }
            else
            {
                if (_epoch > _crd_begStat && _epoch < _crd_endStat)
                {
                    if (_smooth)
                        _Noise.matrixW()(i, i) = 0.0;
                }
                else
                {
                    if (_pos_kin)
                        _param[i].value(_vBanc(1));
                    if (_cntrep == 1 && _success)
                        _Qx.matrixW()(i, i) += _crdStoModel->getQ() * _crdStoModel->getQ();
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
                }
            }
        }

        i = _param.getParam(_site, par_type::CRD_Z, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _param[i].value(_vBanc(2));
                _Qx.matrixW()(i, i) = crdInit * crdInit;
            }
            else
            {
                if (_epoch > _crd_begStat && _epoch < _crd_endStat)
                {
                    if (_smooth)
                        _Noise.matrixW()(i, i) = 0.0;
                }
                else
                {
                    if (_pos_kin)
                        _param[i].value(_vBanc(2));
                    if (_cntrep == 1 && _success)
                        _Qx.matrixW()(i, i) += _crdStoModel->getQ() * _crdStoModel->getQ();
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
                }
            }
        }

        i = _param.getParam(_site, par_type::CLK, "");
        if (i >= 0)
        {
            _param[i].value(_vBanc(3));
            for (unsigned int jj = 0; jj < _param.parNumber(); jj++)
                _Qx.matrixW()(i, jj) = 0.0;
            _Qx.matrixW()(i, i) = _clkStoModel->getQ() * _clkStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _clkStoModel->getQ() * _clkStoModel->getQ();
        }

        i = _param.getParam(_site, par_type::GLO_ISB, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _Qx.matrixW()(i, i) = _sig_init_glo * _sig_init_glo;
            }
            else
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += _gloStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _gloStoModel->getQ();
            }
        }

        i = _param.getParam(_site, par_type::GLO_ifcb, "", FIRST_TIME, LAST_TIME);
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _Qx.matrixW()(i, i) = _sig_init_glo * _sig_init_glo;
            }
            else
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += _gloStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _gloStoModel->getQ();
            }
        }

        i = _param.getParam(_site, par_type::GAL_ISB, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _Qx.matrixW()(i, i) = _sig_init_gal * _sig_init_gal;
            }
            else
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += _galStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _galStoModel->getQ();
            }
        }

        i = _param.getParam(_site, par_type::BDS_ISB, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _Qx.matrixW()(i, i) = _sig_init_bds * _sig_init_bds;
            }
            else
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += _bdsStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _bdsStoModel->getQ();
            }
        }

        i = _param.getParam(_site, par_type::QZS_ISB, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                _Qx.matrixW()(i, i) = _sig_init_qzs * _sig_init_qzs;
            }
            else
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += _qzsStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _qzsStoModel->getQ();
            }
        }

        i = _param.getParam(_site, par_type::TRP, "");
        if (i >= 0)
        {
            Triple Ell, XYZ;

            if (_param.getCrdParam(_site, XYZ) > 0)
            {
            }
            else if (_valid_crd_xml)
            {
                XYZ = _grec->crd_arp(_epoch);
            }
            xyz2ell(XYZ, Ell, false);
            // ZTD apriori (=ZHD) is updated every epoch
            if (_gModel->tropoModel() != 0)
            {
                _param[i].apriori(_gModel->tropoModel()->getZHD(Ell, _epoch));
            }

            if (!_initialized || _Qx.matrixW()(i, i) == 0.0)
            {
                if (_valid_ztd_xml)
                {
                    _param[i].value(_aprox_ztd_xml - _gModel->tropoModel()->getZHD(Ell, _epoch));
                }
                else
                {
                    if (_gModel->tropoModel() != 0)
                    {
                        _param[i].value(_gModel->tropoModel()->getZWD(Ell, _epoch));
                    }
                }

                _Qx.matrixW()(i, i) = ztdInit * ztdInit;
            }
            else
            {
                if (_epoch > _ztd_begStat && _epoch < _ztd_endStat)
                {
                    if (_smooth)
                        _Noise.matrixW()(i, i) = 0.0;
                }
                else
                {
                    if (_cntrep == 1)
                        _Qx.matrixW()(i, i) += _trpStoModel->getQ();
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _trpStoModel->getQ();
                }
            }
        }

        // predict ionosphere delay
        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); it++)
        {
            i = _param.getParam(_site, par_type::VION, it->sat());
            if (i >= 0)
            {
                if (_gion)
                { // Introduce new apriory value from IONEX if available
                    Triple site_xyz(0.0, 0.0, 0.0);
                    Triple site_ell(0.0, 0.0, 0.0);
                    Triple ipp_ell(0.0, 0.0, 0.0);

                    _param.getCrdParam(_site, site_xyz);
                    if (site_xyz.isZero())
                        site_xyz = _vBanc.segment(0, 3);
                    xyz2ell(site_xyz, site_ell, false);
                    ell2ipp(&*it, site_ell, ipp_ell);

                    // use iono-free combination instead ! //
                    //      double ionomodel = _gion->iono(ipp_ell[0] * R2D, ipp_ell[1] * R2D, it->epoch());
                    double ionomodel = 1.0; // ( ionomodel * 40.28 * 1e16 ) / (G01_F*G01_F);

                    _param[i].apriori(ionomodel);
                }

                if (_cntrep == 1 &&
                    !double_eq(_Qx.matrixW()(i, i), _sig_init_vion * _sig_init_vion))
                {
                    _Qx.matrixW()(i, i) += _ionStoModel->getQ() * _ionStoModel->getQ();
                }

                // if (!_initialized || _Qx.matrixW()(i, i) == 0.0) {
                //   std::cout << "DEBUG " << it->sat() << " " << it->epoch().str_ymdhms() << std::endl;
                //   _Qx.matrixW()(i, i) =  _sig_init_vion * _sig_init_vion;
                // }
                // else {
                //   if (_cntrep == 1) _Qx.matrixW()(i, i) += _ionStoModel->getQ() * _ionStoModel->getQ();
                //   if (_smooth) _Noise(i, i) = _ionStoModel->getQ() * _ionStoModel->getQ();
                // }
            }

            i = _param.getParam(_site, par_type::SION, it->sat());
            if (i >= 0)
            {
                if (_cntrep == 1 && !double_eq(_Qx.matrixW()(i, i), _sig_init_vion * _sig_init_vion))
                {
                    _Qx.matrixW()(i, i) += _ionStoModel->getQ() * _ionStoModel->getQ();
                }
            }
        }

        // predict inter-frequency clock bias
        for (it = _data.begin(); it != _data.end(); it++)
        {
            i = _param.getParam(_site, par_type::IFCB_F3, it->sat());
            if (i >= 0)
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += 0.001;
                if (_smooth)
                    _Noise.matrixW()(i, i) = 0.001;
            }
            i = _param.getParam(_site, par_type::IFCB_F4, it->sat());
            if (i >= 0)
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += 0.001;
                if (_smooth)
                    _Noise.matrixW()(i, i) = 0.001;
            }
            i = _param.getParam(_site, par_type::IFCB_F5, it->sat());
            if (i >= 0)
            {
                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += 0.001;
                if (_smooth)
                    _Noise.matrixW()(i, i) = 0.001;
            }
        }
    }

    // Restore state and covariance matrix
    // ---------------------
    void gnss_proc_sppflt::_restore(const Symmetric &QsavBP, const base_allpar &XsavBP)
    {
        _Qx = QsavBP;
        _param = XsavBP;
    }

    // Satelite position
    // ----------
    // TODO:Single Freq Set Option
    int gnss_proc_sppflt::_satPos(base_time &epo, gnss_data_sats &gsatdata)
    {


        std::string satname = gsatdata.sat();

        int i;
        auto nppmodel = dynamic_cast<set_gproc *>(_set)->npp_model();
        int base_size = dynamic_cast<set_gen *>(_set)->list_base().size();
        // modified by zhShen
        if (dynamic_cast<set_gproc *>(_set)->realtime() && base_size == 0 && nppmodel != NPP_MODEL::URTK)
            i = gsatdata.addprd_realtime(_gnav); //add sat crd and clk
        else
            i = gsatdata.addprd(_gnav); //add sat crd and clk

        if (i < 0)
            return i;

        return 1;
    }

    // Get satellite number in epoch for specific system
    // -----------------------------
    int gnss_proc_sppflt::_numSat(GSYS gsys)
    {
        int num = 0;

        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); it++)
        {
            GSYS satSys = it->gsys();
            if (satSys == gsys)
                num++;
            else
                continue;
        }

        return num;
    }

    // Update time for RDW processes
    // ---------------------------
    void gnss_proc_sppflt::_timeUpdate(const base_time &epo)
    {
        // tropo delay
        _trpStoModel->updateTime(epo);
        _trpStoModel->setTcurr(epo);

        // iono delay
        _ionStoModel->updateTime(epo);
        _ionStoModel->setTcurr(epo);

        // GLO ISB
        _gloStoModel->updateTime(epo);
        _gloStoModel->setTcurr(epo);

        // GAL ISB
        _galStoModel->updateTime(epo);
        _galStoModel->setTcurr(epo);

        // BDS ISB
        _bdsStoModel->updateTime(epo);
        _bdsStoModel->setTcurr(epo);

        // QZS ISB
        _qzsStoModel->updateTime(epo);
        _qzsStoModel->setTcurr(epo);
    }

    // Sync inter-GNSS systems bias
    // -----------------------------------
    void gnss_proc_sppflt::_syncSys()
    {

        // modified by zhshen
        if (_data.size() == 0)
            return;
        //  set<std::string> mapPRN;
        bool obsGps = false;
        bool obsGlo = false;
        bool obsGal = false;
        bool obsBds = false;
        bool obsQzs = false;
        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); it++)
        { // loop over all observations

            if (it->gsys() == GPS)
                obsGps = true;
            if (it->gsys() == GLO)
                obsGlo = true;
            if (it->gsys() == GAL)
                obsGal = true;
            if (it->gsys() == BDS)
                obsBds = true;
            if (it->gsys() == QZS)
                obsQzs = true;
        }

        bool onlyGlo = false;
        bool onlyGal = false;
        bool onlyBds = false;
        bool onlyQzs = false;

        if (obsGlo && !obsGps && !obsGal && !obsBds && !obsQzs)
            onlyGlo = true;
        if (obsGal && !obsGps && !obsGlo && !obsBds && !obsQzs)
            onlyGal = true;
        if (obsBds && !obsGps && !obsGlo && !obsGal && !obsQzs)
            onlyBds = true;
        if (obsQzs && !obsGps && !obsGlo && !obsGal && !obsBds)
            onlyQzs = true;

        bool parGlo = false;
        bool parGal = false;
        bool parBds = false;
        bool parQzs = false;
        for (unsigned int i = 0; i < _param.parNumber(); i++)
        {
            if (_param[i].site != _site)
                continue;
            if (_param[i].parType == par_type::GLO_ISB)
                parGlo = true;
            if (_param[i].parType == par_type::GAL_ISB)
                parGal = true;
            if (_param[i].parType == par_type::BDS_ISB)
                parBds = true;
            if (_param[i].parType == par_type::QZS_ISB)
                parQzs = true;
        }

        // Add GLO ISB parameter
        if (!parGlo && obsGlo && !onlyGlo)
        {
            base_par newPar(_data.begin()->site(), par_type::GLO_ISB, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sig_init_glo * _sig_init_glo;
#ifdef DEBUG
            std::cout << "GLO_ISB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add GAL ISB parameter
        if (!parGal && obsGal && !onlyGal)
        {
            base_par newPar(_data.begin()->site(), par_type::GAL_ISB, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sig_init_gal * _sig_init_gal;
#ifdef DEBUG
            std::cout << "GAL_ISB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add BDS ISB parameter
        if (!parBds && obsBds && !onlyBds)
        {
            base_par newPar(_data.begin()->site(), par_type::BDS_ISB, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sig_init_bds * _sig_init_bds;
#ifdef DEBUG
            std::cout << "BDS_ISB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add QZS ISB parameter
        if (!parQzs && obsQzs && !onlyQzs)
        {
            base_par newPar(_data.begin()->site(), par_type::QZS_ISB, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sig_init_qzs * _sig_init_qzs;
#ifdef DEBUG
            std::cout << "QZS_ISB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove GLO ISB paremeter
        if (parGlo && !obsGlo)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::GLO_ISB, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "GLO_ISB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove GAL ISB paremeter
        if (parGal && !obsGal)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::GAL_ISB, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "GAL_ISB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove BDS ISB paremeter
        if (parBds && !obsBds)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::BDS_ISB, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "BDS_ISB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove QZS ISB paremeter
        if (parQzs && !obsQzs)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::QZS_ISB, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "QZS_ISB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

    } // void

    // Add/Remove ionosphere delay
    void gnss_proc_sppflt::_syncIono()
    {

        _param.reIndex();

        // Add ionosphere parameter and appropriate rows/columns covar. matrix
        std::set<std::string> mapPRN;

        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); it++)
        { // loop over all tracked satellites
            mapPRN.insert(it->sat());

            if (_observ == OBSCOMBIN::IONO_FREE || _observ == OBSCOMBIN::IF_P1)
                return;

            if (_observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_MIX)
            {
                /*
                // add ionosphere vertical delay - not in case of single frequency
                if (_param.getParam(_site, par_type::VION, it->sat()) < 0 && _iono_est)
                {
                    base_par parVION(it->site(), par_type::VION, _param.parNumber() + 1, it->sat());
                    if (_gion)
                    {
                        // apriori from iono model
                        Triple site_xyz(0.0, 0.0, 0.0);
                        Triple site_ell(0.0, 0.0, 0.0);
                        Triple ipp_ell(0.0, 0.0, 0.0);

                        _param.getCrdParam(_site, site_xyz);
                        if (site_xyz.isZero()) site_xyz[_vBanc);
                        xyz2ell(site_xyz, site_ell, false);
                        ell2ipp(*it, site_ell, ipp_ell);

                        // use iono-free combination instead ! //
                        //  double ionomodel = _gion->iono(ipp_ell[0] * R2D, ipp_ell[1] * R2D, it->epoch());
                        //  ionomodel = ( ionomodel * 40.28 * 1e16 ) / (G01_F*G01_F);

                        double ionomodel = 1.0;
                        parVION.apriori(ionomodel);
                        parVION.value(0.0);

                    }
                    else
                    {
                        parVION.apriori(1.0);  // apriori as a std::fixed value = 1 m
                        parVION.value(0.0);
                    }

                    _param.addParam(parVION);
                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = _sig_init_vion * _sig_init_vion;
                    //        std::cout << "Pridavam VION " << it->sat() << " " << it->epoch().str_hms() << " " << parVION.apriori() << " " << _Qx.matrixW()(_param.parNumber(), _param.parNumber()) << std::endl;
                }
                */
                // add ionosphere vertical delay - not in case of single frequency
                //std::cout << _iono_est << std::endl;
                if (_param.getParam(_site, par_type::SION, it->sat()) < 0 && _iono_est)
                {
                    base_par parSION(it->site(), par_type::SION, _param.parNumber() + 1, it->sat());

                    parSION.apriori(1.0); // apriori as a std::fixed value = 1 m
                    parSION.value(0.0);

                    _param.addParam(parSION);
                    _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
                    _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = _sig_init_vion * _sig_init_vion;
                    //     std::cout << "Pridavam SION " << it->sat() << " " << it->epoch().str_hms() << " " << parSION.value() << " " << _Qx.matrixW()(_param.parNumber(), _param.parNumber()) << std::endl;
                }
            }

        } // end loop over all observations

        // Remove params and appropriate rows/columns covar. matrix
        for (unsigned int i = 0; i <= _param.parNumber() - 1; i++)
        {
            if (_param[i].parType == par_type::VION)
            {
                std::string sat = _param[i].prn;

                std::set<std::string>::iterator prnITER = mapPRN.find(sat);
                if (prnITER == mapPRN.end())
                {

                    Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
                    _param.delParam(i);
                    _param.reIndex();
                    i--;
                }
            }
        }
        for (unsigned int i = 0; i <= _param.parNumber() - 1; i++)
        {
            if (_param[i].parType == par_type::SION)
            {
                if (_param[i].site != _site)
                    continue;
                std::string sat = _param[i].prn;

                std::set<std::string>::iterator prnITER = mapPRN.find(sat);
                if (prnITER == mapPRN.end())
                {

                    Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
                    _param.delParam(i);
                    _param.reIndex();
                    i--;
                }
            }
        }

        return;
    }

    // Add/Remove inter-freq. biases
    void gnss_proc_sppflt::_syncIFB()
    {

        // modified by zhshen
        if (_data.size() == 0)
            return;
        //  set<std::string> mapPRN;
        bool obsGps = false;
        bool obsGal = false;
        bool obsBds = false;
        bool obsQzs = false;
        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); it++)
        { // loop over all observations

            if (it->gsys() == GPS)
                obsGps = true;
            if (it->gsys() == GAL)
                obsGal = true;
            if (it->gsys() == BDS)
                obsBds = true;
            if (it->gsys() == QZS)
                obsQzs = true;
        }

        bool parGps = false;
        bool parGal = false;
        bool parBds = false;
        bool parQzs = false;

        for (unsigned int i = 0; i < _param.parNumber(); i++)
        {

            if (_param[i].parType == par_type::IFB_GPS)
                parGps = true;
            if (_param[i].parType == par_type::IFB_GAL)
                parGal = true;
            if (_param[i].parType == par_type::IFB_BDS)
                parBds = true;
            if (_param[i].parType == par_type::IFB_QZS)
                parQzs = true;
        }

        // Add GLO ISB parameter
        if (!parGps && obsGps)
        {
            base_par newPar(_data.begin()->site(), par_type::IFB_GPS, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000.0 * 3000;
#ifdef DEBUG
            std::cout << "GPS_IFB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add GAL ISB parameter
        if (!parGal && obsGal)
        {
            base_par newPar(_data.begin()->site(), par_type::IFB_GAL, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000.0 * 3000;
            ;
#ifdef DEBUG
            std::cout << "GAL_IFB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add BDS ISB parameter
        if (!parBds && obsBds)
        {
            base_par newPar(_data.begin()->site(), par_type::IFB_BDS, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;
            ;
#ifdef DEBUG
            std::cout << "BDS_IFB was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add QZS ISB parameter
        if (!parQzs && obsQzs)
        {
            base_par newPar(_data.begin()->site(), par_type::IFB_QZS, _param.parNumber() + 1, "");
            newPar.value(0.0);
            _param.addParam(newPar);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000.0 * 3000;
            ;
#ifdef DEBUG
            std::cout << "QZS_QZS was added!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove GLO ISB paremeter
        if (parGps && !obsGps)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::IFB_GPS, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "GPS_IFB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove GAL ISB paremeter
        if (parGal && !obsGal)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::IFB_GAL, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "GAL_IFB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove BDS ISB paremeter
        if (parBds && !obsBds)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::IFB_BDS, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "BDS_IFB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove QZS ISB paremeter
        if (parQzs && !obsQzs)
        {
            int i = _param.getParam(_data.begin()->site(), par_type::IFB_QZS, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG
            std::cout << "QZS_IFB was removed!"
                 << " Epoch: "
                 << _epoch.str_ymdhms() << std::endl;
#endif
        }
    }
    // tyx : only double or more frequency can use this fuction
    void gnss_proc_sppflt::_syncRcb()
    {
        // modified by zhshen
        if (_data.size() == 0)return;
        //  set<std::string> mapPRN; 
        bool obsGps = false;
        bool obsGal = false;
        bool obsBds = false;
        bool obsQzs = false;
        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); it++) {       // loop over all observations

            if (it->gsys() == GPS) obsGps = true;
            if (it->gsys() == GAL) obsGal = true;
            if (it->gsys() == BDS) obsBds = true;
            if (it->gsys() == QZS) obsQzs = true;
        }

        bool parGps = false;
        bool parGal = false;
        bool parBds = false;
        bool parQzs = false;

        for (unsigned int i = 0; i < _param.parNumber(); i++) {
            if (_param[i].site != _site)
                continue;
            if (_param[i].parType == par_type::RCB_GPS_1 || _param[i].parType == par_type::RCB_GPS_2) parGps = true;
            if (_param[i].parType == par_type::RCB_GAL_1 || _param[i].parType == par_type::RCB_GAL_2) parGal = true;
            if (_param[i].parType == par_type::RCB_BDS_1 || _param[i].parType == par_type::RCB_BDS_2) parBds = true;
            if (_param[i].parType == par_type::RCB_QZS_1 || _param[i].parType == par_type::RCB_QZS_2) parQzs = true;
        }

        // Add GPS RCB parameter
        if (!parGps && obsGps) {
            base_par newPar1(_data.begin()->site(), par_type::RCB_GPS_1, _param.parNumber() + 1, "");
            newPar1.value(0.0);
            _param.addParam(newPar1);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;

            base_par newPar2(_data.begin()->site(), par_type::RCB_GPS_2, _param.parNumber() + 1, "");
            newPar2.value(0.0);
            _param.addParam(newPar2);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;
#ifdef DEBUG   
            std::cout << "GPS_RCB was added!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add GAL RCB parameter
        if (!parGal && obsGal) {
            base_par newPar1(_data.begin()->site(), par_type::RCB_GAL_1, _param.parNumber() + 1, "");
            newPar1.value(0.0);
            _param.addParam(newPar1);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;

            base_par newPar2(_data.begin()->site(), par_type::RCB_GAL_2, _param.parNumber() + 1, "");
            newPar2.value(0.0);
            _param.addParam(newPar2);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;
#ifdef DEBUG   
            std::cout << "GAL_RCB was added!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add BDS RCB parameter
        if (!parBds && obsBds) {
            base_par newPar1(_data.begin()->site(), par_type::RCB_BDS_1, _param.parNumber() + 1, "");
            newPar1.value(0.0);
            _param.addParam(newPar1);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;

            base_par newPar2(_data.begin()->site(), par_type::RCB_BDS_2, _param.parNumber() + 1, "");
            newPar1.value(0.0);
            _param.addParam(newPar2);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;
#ifdef DEBUG   
            std::cout << "BDS_RCB was added!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Add QZS RCB parameter
        if (!parQzs && obsQzs) {
            base_par newPar1(_data.begin()->site(), par_type::RCB_QZS_1, _param.parNumber() + 1, "");
            newPar1.value(0.0);
            _param.addParam(newPar1);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;

            base_par newPar2(_data.begin()->site(), par_type::RCB_QZS_2, _param.parNumber() + 1, "");
            newPar2.value(0.0);
            _param.addParam(newPar2);
            _Qx.Matrix_addRC(_param.parNumber() - 1, _param.parNumber() - 1);
            _Qx.matrixW()(_param.parNumber() - 1, _param.parNumber() - 1) = 3000 * 3000;
#ifdef DEBUG   
            std::cout << "QZS_RCB was added!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif
        }

        // Remove GPS ISB paremeter
        if (parGps && !obsGps) {
            int i = _param.getParam(_data.begin()->site(), par_type::RCB_GPS_1, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();

            i = _param.getParam(_data.begin()->site(), par_type::RCB_GPS_2, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG   
            std::cout << "GPS_RCB was removed!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif     
        }

        // Remove BDS ISB paremeter
        if (parBds && !obsBds) {
            int i = _param.getParam(_data.begin()->site(), par_type::RCB_BDS_1, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();

            i = _param.getParam(_data.begin()->site(), par_type::RCB_BDS_2, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG   
            std::cout << "BDS_RCB was removed!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif     
        }

        // Remove GAL ISB paremeter
        if (parGal && !obsGal) {
            int i = _param.getParam(_data.begin()->site(), par_type::RCB_GAL_1, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();

            i = _param.getParam(_data.begin()->site(), par_type::RCB_GAL_2, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG   
            std::cout << "GAL_RCB was removed!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif     
        }

        // Remove QZS ISB paremeter
        if (parQzs && !obsQzs) {
            int i = _param.getParam(_data.begin()->site(), par_type::RCB_QZS_1, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();

            i = _param.getParam(_data.begin()->site(), par_type::RCB_QZS_2, "");
            Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
            _param.delParam(i);
            _param.reIndex();
#ifdef DEBUG   
            std::cout << "QZS_RCB was removed!" << " Epoch: "
                << _epoch.str_ymdhms() << std::endl;
#endif     
        }
    }

    //    void gnss_proc_sppflt::_syncIFB()
    //    {
    //
    //
    //        set<std::string> mapPRN;
    //
    //        std::vector<gnss_data_sats>::iterator it;
    //        for (it = _data.begin(); it != _data.end(); it++) {       // loop over all tracked satellites
    //            mapPRN.insert(it->sat());
    //
    //            if (_observ == IONO_FREE) return;
    //
    //            if (_observ == RAW_ALL) {
    //
    //                // add inter-frequency code bias for FREQ_3 (if not already)
    //                if (!_ifb3_init && it->contain_freq(FREQ_3) && _param.getParam(_site, par_type::IFB_C3, "") < 0) {
    //                    base_par parIFB(it->site(), par_type::IFB_C3, _param.parNumber() + 1, "");
    //                    parIFB.value(0.0);
    //                    _param.addParam(parIFB);
    //                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
    //                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = 100 * 100;
    //                    _ifb3_init = true;
    //                }
    //
    //                // add inter-frequency code bias for FREQ_4 (if not already)
    //                if (!_ifb4_init && it->contain_freq(FREQ_4) && _param.getParam(_site, par_type::IFB_C4, "") < 0) {
    //                    base_par parIFB(it->site(), par_type::IFB_C4, _param.parNumber() + 1, "");
    //                    parIFB.value(0.0);
    //                    _param.addParam(parIFB);
    //                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
    //                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = 100 * 100;
    //                    _ifb4_init = true;
    //                }
    //
    //                // add inter-frequency code bias for FREQ_5 (if not already)
    //                if (!_ifb5_init && it->contain_freq(FREQ_5) && _param.getParam(_site, par_type::IFB_C5, "") < 0) {
    //                    base_par parIFB(it->site(), par_type::IFB_C5, _param.parNumber() + 1, "");
    //                    parIFB.value(0.0);
    //                    _param.addParam(parIFB);
    //                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
    //                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = 100 * 100;
    //                    _ifb5_init = true;
    //                }
    //                // add inter-frequency clock bias for FREQ_3
    //
    //                if (it->contain_freq(FREQ_3) && _param.getParam(_site, par_type::IFCB_F3, it->sat()) < 0) {
    //                    base_par parIFCB(it->site(), par_type::IFCB_F3, _param.parNumber() + 1, it->sat());
    //                    parIFCB.value(0.0);
    //                    _param.addParam(parIFCB);
    //                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
    //                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = 1 * 1;
    //                }
    //
    //                // add inter-frequency clock bias for FREQ_4
    //                if (it->contain_freq(FREQ_4) && _param.getParam(_site, par_type::IFCB_F4, it->sat()) < 0) {
    //                    base_par parIFCB(it->site(), par_type::IFCB_F4, _param.parNumber() + 1, it->sat());
    //                    parIFCB.value(0.0);
    //                    _param.addParam(parIFCB);
    //                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
    //                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = 1 * 1;
    //                }
    //
    //                // add inter-frequency clock bias for FREQ_5
    //                if (it->contain_freq(FREQ_5) && _param.getParam(_site, par_type::IFCB_F5, it->sat()) < 0) {
    //                    base_par parIFCB(it->site(), par_type::IFCB_F5, _param.parNumber() + 1, it->sat());
    //                    parIFCB.value(0.0);
    //                    _param.addParam(parIFCB);
    //                    _Qx.Matrix_addRC(_param.parNumber(), _param.parNumber());
    //                    _Qx.matrixW()(_param.parNumber(), _param.parNumber()) = 1 * 1;
    //                }
    //
    //            }
    //
    //        }  // end loop over all satellites
    //
    //        // Remove params and appropriate rows/columns covar. matrix
    //        for (unsigned int i = 0; i <= _param.parNumber() - 1; i++) {
    //            if (_param[i].parType == par_type::IFCB_F3 ||
    //                _param[i].parType == par_type::IFCB_F4 ||
    //                _param[i].parType == par_type::IFCB_F5) {
    //
    //                std::string sat = _param[i].prn;
    //
    //                set<std::string>::iterator prnITER = mapPRN.find(sat);
    //                if (prnITER == mapPRN.end()) {
    //
    //                    Matrix_remRC(_Qx.matrixW(), _param[i].index, _param[i].index);
    //                    _param.delParam(i);
    //                    _param.reIndex();
    //                    i--;
    //                }
    //            }
    //        }
    //
    //        return;
    //    }

    // get gnss_data_obs instance for particular prn, band, and attribute
    int gnss_proc_sppflt::_getgobs(std::string prn, GOBSTYPE type, GOBSBAND band, gnss_data_obs &gobs)
    {
        std::map<std::string, std::map<GOBSBAND, GOBSATTR>>::iterator itPRN = _signals.find(prn);
        if (itPRN == _signals.end())
        {
            _signals[prn][band] = ATTR;
            itPRN = _signals.find(prn);
        }

        std::map<GOBSBAND, GOBSATTR>::iterator itBAND = itPRN->second.find(band);
        if (itBAND == itPRN->second.end())
        {
            _signals[prn][band] = ATTR;
        }

        GOBSATTR attr = _signals[prn][band];

        gobs.type(type);
        gobs.band(band);
        gobs.attr(attr);

        return 1;
    }

    // save observations residuals
    void gnss_proc_sppflt::_save_residuals(Vector &v, std::vector<gnss_data_sats> &data, RESIDTYPE restype)
    {
        int mult = 1;
        if (_phase)
            mult = 2;
        int k = 0;

        for (auto it = data.begin(); it != data.end(); it++)
        {

            std::vector<double> res;

            auto itFrq = _frqNum.find(it->sat());
            if (itFrq == _frqNum.end())
                continue;

            // separate frequencies
            for (int i = 0; i < itFrq->second * mult; i++)
            {
                res.push_back(v(i + k));
            }
            k += itFrq->second * mult;

            // clear all previously stored residuals
            it->clear_res(restype);

            // separate code and phase and storing
            unsigned int sz = res.size();
            if (_phase)
            {
                for (unsigned int i = 0; i < sz / 2; i++)
                    it->addres(restype, TYPE_C, res[i]);
                for (unsigned int i = sz / 2; i < sz; i++)
                    it->addres(restype, TYPE_L, res[i]);
            }
            else
                for (unsigned int i = 0; i < sz; i++)
                    it->addres(restype, TYPE_C, res[i]);
        }
    }

    // Apply DCB correction
    int gnss_proc_sppflt::_applyDCB(gnss_data_sats &satdata, double &P, gnss_data_obs *gobs1, gnss_data_obs *gobs2)
    {
        GSYS gsys = satdata.gsys();
        GOBS g1_R2, g1_R3, g1_ref;
        GOBS g2_R2, g2_R3, g2_ref;
        ;
        g1_R2 = g1_R3 = g2_R2 = g2_R3 = X;
        if (gsys == GPS || gsys == GLO)
        {
            g1_R2 = P1;
            g1_R3 = C1W;
            g2_R2 = P2;
            g2_R3 = C2W;
        }
        else if (gsys == GAL)
        {
            g1_R2 = C1;
            g1_R3 = C1X;
            g2_R2 = C5;
            g2_R3 = C5X;
        }
        else if (gsys == BDS)
        {
            g1_R2 = C1;
            g1_R3 = C1I;
            g2_R2 = C7;
            g2_R3 = C7I;
        }

        GOBS g1 = X;
        GOBS g2 = X;

        if (gobs1->attr() == ATTR)
            g1 = satdata.id_range(gobs1->band()); // automatic GOBS selection
        else
            g1 = gobs1->gobs(); // specific GOBS

        if (gobs2 != 0)
        {
            if (gobs2->attr() == ATTR)
                g2 = satdata.id_range(gobs2->band()); // automatic GOBS selection
            else
                g2 = gobs2->gobs(); // specific GOBS
        }

        FREQ_SEQ freq1 = gnss_sys::band2freq(satdata.gsys(), gobs1->band());

        double corr1 = 0.0;
        double corr2 = 0.0;

        base_time epoch = satdata.epoch();
        std::string sat = satdata.sat();

        // conversion 2char <-> 3char signals when needed
        gnss_data_obs tmp1(g1);
        gnss_data_obs tmp2;
        if (gobs2)
            tmp2.gobs(g2);
        // force 2char signals
        if (_cbiaschar == CBIASCHAR::CHAR2 && (gsys == GPS || gsys == GLO))
        {
            g1 = tmp1.gobs2CH(gsys);
            if (gobs2)
                g2 = tmp2.gobs2CH(gsys);
            g1_ref = g1_R2;
            g2_ref = g2_R2;
            // force 3char sigals
        }
        else if (_cbiaschar == CBIASCHAR::CHAR3)
        {
            g1 = tmp1.gobs3CH();
            if (gobs2)
                g2 = tmp2.gobs3CH();
            g1_ref = g1_R3;
            g2_ref = g2_R3;
            // original signals
        }
        else if (_cbiaschar == CBIASCHAR::ORIG || gsys != GPS || gsys != GLO)
        {
            if (g1 < 1000)
            {
                g1_ref = g1_R3;
                g2_ref = g2_R3;
            } // 3char
            else
            {
                g1_ref = g1_R2;
                g2_ref = g2_R2;
            } // 2char
        }

        if (_gallbias)
        {
            if (freq1 == FREQ_1 && g1 != g1_ref)
            {
                corr1 = _gallbias->get(epoch, sat, g1_ref, g1);
            }
            if (freq1 == FREQ_2 && g1 != g2_ref)
            {
                corr1 = _gallbias->get(epoch, sat, g2_ref, g1);
            }

            if (gobs2)
            {
                FREQ_SEQ freq2 = gnss_sys::band2freq(satdata.gsys(), gobs2->band());
                if (freq2 == FREQ_1 && g2 != g1_ref)
                {
                    corr2 = _gallbias->get(epoch, sat, g1_ref, g2);
                }
                if (freq2 == FREQ_2 && g2 != g2_ref)
                {
                    corr2 = _gallbias->get(epoch, sat, g2_ref, g2);
                }
            }
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: Code biases container not available!");
        }

        if (gobs2 != 0)
        {
            double alfa, beta;
            alfa = beta = 0.0;
            satdata.coef_ionofree(gobs1->band(), alfa, gobs2->band(), beta);
            //     std::cout << "Opravuji Code obs: " << satdata.sat() << " " << satdata.epoch().str_ymdhms() << " "
            //          << gobs2str(g1) << " " << corr1/CLIGHT*1e9 << " " << gobs2str(g2) << " " << corr2/CLIGHT*1e9 << std::endl;
            if (_observ == OBSCOMBIN::IONO_FREE)
                P += alfa * corr1 + beta * corr2;
        }
        else
        {
            P += corr1;
        }

        return 1;
    }

    // get crd copy by glfeng from spplsq
    Triple gnss_proc_sppflt::getCrd(const base_time &time)
    {
        if (_map_crd.find(time) != _map_crd.end())
        {
            return _map_crd[time];
        }
        auto left_crd = _map_crd.lower_bound(time);
        auto right_crd = left_crd;
        if (_map_crd.size() > 0)
        {
            left_crd--;
        }
        if (left_crd != _map_crd.end() && right_crd != _map_crd.end())
        {
            return (left_crd->second + right_crd->second) / 2;
        }
        else if (left_crd != _map_crd.end())
        {
            return left_crd->second;
        }
        else if (right_crd != _map_crd.end())
        {
            return right_crd->second;
        }
        else
        {
            return Triple(0.0, 0.0, 0.0);
        }
    }

} // namespace
