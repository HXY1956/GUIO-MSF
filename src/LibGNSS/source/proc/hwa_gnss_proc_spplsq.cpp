#include "hwa_gnss_proc_Spplsq.h"
#include "hwa_set_param.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    gnss_proc_spplsq::gnss_proc_spplsq(std::string mark, set_base *set, gnss_all_obj *obj, base_log spdlog, bool dynamic)
        : gnss_proc_spp(mark, set, spdlog),
          _minsat(static_cast<size_t>(SPP_MINSAT)),
          _glsq(nullptr),
          _result(0),
          _dynamics(dynamic)
    {

        // std::set obj
        setOBJ(obj);

        // std::set site && inono free
        dynamic_cast<gnss_model_spp *>(_gModel)->setrec(obj->obj(_site));
        if (_observ != OBSCOMBIN::RAW_MIX && _observ != OBSCOMBIN::RAW_SINGLE)
        {
            dynamic_cast<gnss_model_spp *>(_gModel)->reset_observ(OBSCOMBIN::IONO_FREE);
            _observ = OBSCOMBIN::IONO_FREE;
        }
        _sig_ion = dynamic_cast<set_par *>(set)->sigSion(_site);

        // get std::set from setfile
        _get_own_settings();

        // std::set output
        _setOut();
    }

    gnss_proc_spplsq::~gnss_proc_spplsq()
    {

        if (_glsq)
        {
            delete _glsq;
            _glsq = nullptr;
        }

        if (_result)
        {
            if (_result->is_open())
            {
                _result->close();
            }
            delete _result;
            _result = nullptr;
        }
    }

    int gnss_proc_spplsq::processBatch(const base_time &beg, const base_time &end)
    {
        _obs_num.clear();

        int suc_nepoch = 0;
        if (beg > end)
        {
            std::cerr << _site << " - processing not started [beg > end]\n";
            return -1;
        }

        // std::set the _dop
        _dop.set_data(_gnav, _gobs, _site);

        // define the time begin/end/intv/now
        base_time begT(beg);
        base_time endT(end);
        base_time now(beg);

        // time loop
        while (now <= end)
        {
            int invalid_equation = ProcessOneEpoch(now, NULL);
            if (invalid_equation <= 0)
            {
                now = now + _sampling;
                continue;
            }
            // update time
            now = now + _sampling;
            suc_nepoch++;
        }

        return suc_nepoch;
    }

    int gnss_proc_spplsq::ProcessOneEpoch(const base_time &now, std::vector<gnss_data_sats> *vec_obs)
    {
        // get the obs !! This is the Raw obs !!!
        _data.erase(_data.begin(), _data.end());

        if (vec_obs)
            _data = *vec_obs;
        else
            _data = _gobs->obs(_site, now);

        // no obs
        if (_data.size() == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _site + now.str_ymdhms(" no observation found at epoch: "));
            return -1;
        }

        // apply dcb by lvhb 20200803
        if (_gallbias)
        {
            for (auto &itdata : _data)
            {
                itdata.apply_bias(_gallbias);
            }
        }

        int invalid_equation = _processEpoch(now);
        if (invalid_equation <= 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, now.str_ymdhms("havo no equations at the epoch "));
            return -1;
        }

        return 1;
    }

    double gnss_proc_spplsq::getRecClk(const base_time &time)
    {
        auto param = _glsq->_x_solve;
        int i = param.getParam(_site, par_type::CLK, "");
        return i < 0 ? 0.0 : param[i].value();
    }

    Triple gnss_proc_spplsq::getvel(const base_time &time)
    {
        auto param = _glsq->_x_solve;
        int i = param.getParam(_site, par_type::VEL_X, "");
        int j = param.getParam(_site, par_type::VEL_Y, "");
        int k = param.getParam(_site, par_type::VEL_Z, "");
        Triple vel(param[i].value(), param[j].value(), param[k].value());
        return vel;
    }

    Triple gnss_proc_spplsq::getCrd(const base_time &time)
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

    Triple gnss_proc_spplsq::getAvergeCrd()
    {
        Triple crd_aver(0.0, 0.0, 0.0);
        for (auto iter_crd = _map_crd.begin(); iter_crd != _map_crd.end(); iter_crd++)
        {
            crd_aver += iter_crd->second;
        }
        crd_aver /= _map_crd.size();
        return crd_aver;
    }

    std::set<std::string> gnss_proc_spplsq::outcomsat(std::map<std::string, gnss_data_sats> &satdata)
    {
        std::set<std::string> com_sat;

        for (std::vector<gnss_data_sats>::iterator it = _data.begin(); it != _data.end(); it++)
        {
            std::string comsat = it->sat();
            com_sat.insert(comsat);
            satdata[comsat] = *it;
        }

        return com_sat;
    }

    std::map<base_time, int> gnss_proc_spplsq::getobsnum()
    {
        return _obs_num;
    }

    int gnss_proc_spplsq::_processEpoch(const base_time &runEpoch)
    {

        // construct lsq estimator
        Triple crd_init(0.0, 0.0, 0.0);
        double clk_init = 0.0;
        //crd_init = _grec->crd(runEpoch);
        int invalid_equation = 0;
        int count_iter = 0;
        Triple vel_init(0.0, 0.0, 0.0);
        double vclk_init = 0.0;
        // iterator
        while (count_iter <= 10)
        {
            // construct lsq
            if (_glsq)
            {
                delete _glsq;
                _glsq = nullptr;
            }
            _glsq = new gnss_proc_lsqbase(nullptr);
            base_par parx(_site, par_type::CRD_X, 0, "");
            base_par pary(_site, par_type::CRD_Y, 1, "");
            base_par parz(_site, par_type::CRD_Z, 2, "");
            base_par parclk(_site, par_type::CLK, 3, "");
            parx.value(crd_init[0]);
            pary.value(crd_init[1]);
            parz.value(crd_init[2]);
            parclk.value(clk_init);

            _glsq->add_parameter(parx);
            _glsq->add_parameter(pary);
            _glsq->add_parameter(parz);
            _glsq->add_parameter(parclk);
            if (_dynamics)
            {
                base_par parv_x(_site, par_type::VEL_X, 4, "");
                base_par parv_y(_site, par_type::VEL_Y, 5, "");
                base_par parv_z(_site, par_type::VEL_Z, 6, "");
                base_par parv_clk(_site, par_type::CLK_RAT, 7, "");
                parv_x.value(vel_init[0]);
                parv_y.value(vel_init[1]);
                parv_z.value(vel_init[2]);
                parv_clk.value(vclk_init);
                _glsq->add_parameter(parv_x);
                _glsq->add_parameter(parv_y);
                _glsq->add_parameter(parv_z);
                _glsq->add_parameter(parv_clk);
            }
            if (_sys.size() >= 2)
            {
                std::string ref_sys;
                if (_sys.count("GPS") > 0)
                    ref_sys = "GPS";
                else if (_sys.count("LEO") > 0)
                    ref_sys = "LEO"; //xjhan
                else if (_sys.count("GAL") > 0)
                    ref_sys = "GAL";
                else if (_sys.count("BDS") > 0)
                    ref_sys = "BDS";
                else if (_sys.count("QZS") > 0)
                    ref_sys = "QZS";
                else
                    ref_sys = "GLO";

                base_par par_isbifb;
                for (auto it_sys : _sys)
                {
                    GSYS crt_sys = gnss_sys::str2gsys(it_sys);
                    switch (crt_sys)
                    {
                    case GPS:
                        break;
                    case GAL:
                        par_isbifb = base_par(_site, par_type::GAL_ISB, _glsq->_x_solve.parNumber() + 1, "");
                        par_isbifb.value(0.0);
                        _glsq->add_parameter(par_isbifb);
                        break;
                    case BDS:
                        par_isbifb = base_par(_site, par_type::BDS_ISB, _glsq->_x_solve.parNumber() + 1, "");
                        par_isbifb.value(0.0);
                        _glsq->add_parameter(par_isbifb);
                        break;
                    case LEO: //xjhan
                        par_isbifb = base_par(_site, par_type::LEO_ISB, _glsq->_x_solve.parNumber() + 1, "");
                        par_isbifb.value(0.0);
                        _glsq->add_parameter(par_isbifb);
                        break;

                    case QZS:
                        par_isbifb = base_par(_site, par_type::QZS_ISB, _glsq->_x_solve.parNumber() + 1, "");
                        par_isbifb.value(0.0);
                        _glsq->add_parameter(par_isbifb);
                        break;
                    case GLO:
                        for (auto itdata : _data)
                        {
                            std::string sat = itdata.sat();
                            if (sat.substr(0, 1) != "R")
                                continue;
                            par_isbifb = base_par(_site, par_type::GLO_IFB, _glsq->_x_solve.parNumber() + 1, sat);
                            par_isbifb.value(0.0);
                            _glsq->add_parameter(par_isbifb);
                        }
                        break;
                    default:
                        break;
                    }
                }
            }

            // Process this Epoch and get the equations
            Matrix B;
            Diag P;
            Vector l;
            // Check whether the receiver is valid or not
            if (_grec == nullptr)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "No receiver settings available!!!");
                return -1;
            }
            // Get the current epoch
            _epoch = runEpoch;

            //  the fist estimator of _data (erase some unused data)
            if (_prepareData() == 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "This epoch have no useful data");
                return -1;
            }

            // for obs std::set spdlog && std::set IONO
            for (auto iter = _data.begin(); iter != _data.end(); iter++)
            {
                iter->spdlog(_spdlog);
                if (_observ == OBSCOMBIN::RAW_MIX)
                {
                    base_par parSION(_site, par_type::SION, _glsq->_x_solve.parNumber() + 1, iter->sat());
                    parSION.value(0.0);
                    _glsq->add_parameter(parSION);
                }
            }

            // Get the parameter
            base_allpar &param = _glsq->_x_solve;
            Triple groundEll, groundXYZ;
            param.getCrdParam(_site, groundXYZ);
            xyz2ell(groundXYZ, groundEll, false);

            // define a number of measurements
            unsigned int nObs = _data.size();
            unsigned int mult = 1;
            //if (_observ == RAW_DOUBLE) { mult = 2; nObs *= 2; }
            //if (_observ == RAW_ALL) { mult = 2; nObs *= 5; } // reservation for 5 freq - not used raws will be removed
            if (_observ == OBSCOMBIN::RAW_MIX)
            {
                mult = 2;
                nObs *= 5;
            } // reservation for 5 freq - not used raws will be removed
            if (_phase)
            {
                mult *= 2;
                nObs *= 2;
            } // code + phase

            unsigned int nPar = param.parNumber();

            // resize the Matrix B l P
            if (_dynamics)
                nObs *= 5;
            B.resize(nObs, nPar);
            B.setZero();
            l.resize(nObs);
            l.setZero();
            P.resize(nObs);
            P.setZero();

            // count for useful obs
            unsigned int iobs = 0;

            // loop over the obs data generate Matrix
            // TODO Maybe the obstype should be optional?
            for (auto it = _data.begin(); it != _data.end();)
            {
                if (_addObsP(*it, iobs, groundEll, B, l, P) <= 0)
                {
                    it = _data.erase(it);
                    continue;
                }
                invalid_equation++;
                if (_addObsL(*it, iobs, groundEll, B, l, P) <= 0)
                {
                    it = _data.erase(it);
                    continue;
                }
                if (_dynamics)
                {
                    if (_addObsD(*it, iobs, groundEll, B, l, P) <= 0)
                    {
                        it = _data.erase(it);
                        continue;
                    }
                }
                it++;
            }

            //invalid_equation = iobs;
            B = B.block(0,0,iobs,B.cols());
            P.matrixW() = P.matrixR().block(0, 0, iobs, iobs);
            l = l.segment(0, iobs);

            if (invalid_equation < 4)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _site + runEpoch.str_ymdhms("no enough equations:"));
                break;
            }

            // add the new equations
            _glsq->add_equation(B, P, l, runEpoch);

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _site + runEpoch.str_ymdhms("processing epoch:"));

            try
            {
                _glsq->solve_NEQ();
            }
            catch (...)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _site + _epoch.str_ymdhms() + "_glsq->solve_NEQ failed");

                invalid_equation = -1;
                break;
            }
            Triple dx = (_glsq->dx()).segment(0, 3);
            Triple crd_temp(0.0, 0.0, 0.0);
            _glsq->_x_solve.getCrdParam(_site, crd_temp);
            crd_temp += dx;
            if ((_glsq->dx()).segment(0, 4).norm() < 1E-4)
            {
                if (_dynamics && count_iter != 10)
                {
                    if ((_glsq->dx()).segment(4, 4).norm() < 1E-6)
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
            crd_init = crd_temp;
            clk_init += _glsq->dx()(3);

            if (_dynamics)
            {
                Triple dv = (_glsq->dx()).segment(4, 3);
                int i = param.getParam(_site, par_type::VEL_X, "");
                int j = param.getParam(_site, par_type::VEL_Y, "");
                int k = param.getParam(_site, par_type::VEL_Z, "");
                vel_init[0] = param[i].value();
                vel_init[1] = param[j].value();
                vel_init[2] = param[k].value();
                vel_init += dv;
                vclk_init += _glsq->dx()(7);
            }
            count_iter++;
            _obs_num[runEpoch] = P.rows();
        }

        if (invalid_equation >= 4 && count_iter <= 10)
        {
            _map_crd[runEpoch] = crd_init;
            _prepareData();
            return invalid_equation;
        }
        else
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, runEpoch.str_ymdhms() + ": solve fail!!");
            return -1;
        }

        return -1;
    }

    int gnss_proc_spplsq::_get_own_settings()
    {
        _sys = dynamic_cast<set_gen *>(_set)->sys();
        _beg = dynamic_cast<set_gen *>(_set)->beg();
        _end = dynamic_cast<set_gen *>(_set)->end();
        _pos_kin = dynamic_cast<set_gproc *>(_set)->pos_kin();
        _crd_est = dynamic_cast<set_gproc *>(_set)->crd_est();

        //freq_index and band_index
        _band_index[GPS] = dynamic_cast<set_gnss *>(_set)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(_set)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(_set)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(_set)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(_set)->band_index(QZS);

        return 1;
    }

    int gnss_proc_spplsq::_prepareData()
    {

        std::vector<gnss_data_sats>::iterator it = _data.begin();

        // erase the unused data of gnss system unused
        it = _data.begin();
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

        // ============= add for spp parameters initialation glfeng
        Matrix BB;

        BB.resize(_data.size(), 4);
        BB.setZero();
        int iobs = 0;
        // =============
        // filter the obs according to the obs && the sat

        // if obs is used ?
        // TODO: the obs type is LC PC;it should be optional
        it = _data.begin();
        while (it != _data.end())
        {
            GSYS gs = it->gsys();
            GOBSBAND b1, b2;

            //b1 = gnss_sys::band_priority(gs, FREQ_1);
            //b2 = gnss_sys::band_priority(gs, FREQ_2);
            //lvhb modified
            b1 = _band_index[gs][FREQ_1];
            b2 = _band_index[gs][FREQ_2];

            double obsP = it->P3(b1, b2);

            if (double_eq(obsP, 0.0) && (_observ == OBSCOMBIN::RAW_MIX || _observ == OBSCOMBIN::RAW_SINGLE))
            {
                /*if(!double_eq(it->obs_L(gnss_data_band(b1, GOBSATTR::ATTR)), 0.0))*/
                obsP = it->obs_C(gnss_data_band(b1, GOBSATTR::ATTR));
            }

            if (double_eq(obsP, 0.0))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "have no PIF erase the sat:" + it->sat());
                it = _data.erase(it);
                continue;
            }

            if (_satPos(_epoch, *it) < 0)
            {
                it = _data.erase(it);
                continue;
            }

            BB(iobs, 0) = it->satcrd()[0];
            BB(iobs, 1) = it->satcrd()[1];
            BB(iobs, 2) = it->satcrd()[2];
            //BB(iobs, 4) = it->P3(b1, b2) + it->clk();
            BB(iobs, 3) = obsP + it->clk();
            iobs++;

            it++;
        }
        //lvhb
        //std::cout << "bb" << std::endl;
        //std::cout << std::fixed << std::setprecision(6) << std::setw(15) << BB << std::endl;

        int i = _glsq->_x_solve.getParam(_site, par_type::CRD_X, "");
        int j = _glsq->_x_solve.getParam(_site, par_type::CRD_Y, "");
        int k = _glsq->_x_solve.getParam(_site, par_type::CRD_Z, "");
        int m = _glsq->_x_solve.getParam(_site, par_type::CLK, "");

        if (double_eq(_glsq->_x_solve[i].value(), 0.0) && double_eq(_glsq->_x_solve[j].value(), 0.0) &&
            double_eq(_glsq->_x_solve[k].value(), 0.0))
        {
            if (_data.size() < _minsat)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _epoch.str_ymdhms(_site + " epoch ") + " skipped (Bancroft not calculated: " + base_type_conv::int2str(_data.size()) + " < _minsat)");
                return -1;
            }

            BB = BB.block(0, 0, iobs, BB.cols()); // delete zero rows

            if (BB.rows() < static_cast<int>(_minsat))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, _epoch.str_ymdhms(_site + " epoch ") + " skipped (Bancroft not calculated: BB.rows < _minsat)");
                return -1;
            }

            Vector vBanc;
            gbancroft(BB, vBanc);

            _glsq->_x_solve[i].value(vBanc(0));
            _glsq->_x_solve[j].value(vBanc(1));
            _glsq->_x_solve[k].value(vBanc(2));
            _glsq->_x_solve[m].value(vBanc(3));
        }

        // if sattelite is used?
        //    relative have been thought in clk
        it = _data.begin();
        while (it != _data.end())
        {
            if (it->addprd(_gnav) < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "have no eph erase the sat:" + it->sat());
                it = _data.erase(it);
                continue;
            }
            it++;
        }

        // elvation
        // compute sat elevation and rho
        // TODO  real time should change
        it = _data.begin();
        while (it != _data.end())
        {
            Triple xyz_r, xyz_s, neu_s;
            xyz_s = it->satcrd();
            // ARP Correction
            //xyz_r = _grec->crd_arp(_epoch);
            _glsq->_x_solve.getCrdParam(_site, xyz_r);

            Triple xyz_rho = xyz_s - xyz_r;
            Triple ell_r;
            xyz2ell(xyz_r, ell_r, false);

            xyz2neu(ell_r, xyz_rho, neu_s);

            // Earth rotation correction
            Triple xRec;
            double rho0 = sqrt(pow(xyz_r[0] - xyz_s[0], 2) + pow(xyz_r[1] - xyz_s[1], 2) + pow(xyz_r[2] - xyz_s[2], 2));
            double dPhi = OMEGA * rho0 / CLIGHT;
            xRec[0] = xyz_r[0] * cos(dPhi) - xyz_r[1] * sin(dPhi);
            xRec[1] = xyz_r[1] * cos(dPhi) + xyz_r[0] * sin(dPhi);
            xRec[2] = xyz_r[2];

            // Apply tides
            _apply_tides(_epoch, xRec);

            double tmp = (it->satcrd() - xRec).norm();

            it->addrho(tmp);

            double NE2 = neu_s[0] * neu_s[0] + neu_s[1] * neu_s[1];
            double ele = acos(sqrt(NE2) / it->rho());
            if (neu_s[2] < 0.0)
            {
                ele *= -1.0;
            }

            if (sqrt(NE2) / it->rho() > 1.0)
                it->addele(0.0);
            else
                it->addele(ele);

            double azi = atan2(neu_s[1], neu_s[0]);
            if (azi < 0)
                azi += 2 * hwa_pi;
            it->addazi_rec(azi);

            it++;
        }

        // TODO add the res

        return _data.size();
    }

    int gnss_proc_spplsq::_addObsP(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &B, Vector &OMC, Diag &P)
    {

        // TODO obstype should be optional !!
        // TODO Maybeneed our own obstype
        // just use the PC here
        //  This part  for getting the obs according to the OBS
        gnss_sys gsys(satdata.gsys());
        GSYS gs = gsys.gsys();

        GOBSBAND b1, b2;
        //b1 = gnss_sys::band_priority(gs, FREQ_1);
        //b2 = gnss_sys::band_priority(gs, FREQ_2);
        //lvhb modified
        b1 = _band_index[gs][FREQ_1];
        b2 = _band_index[gs][FREQ_2];

        gnss_data_obs gobs1(TYPE_C, b1, ATTR);
        gnss_data_obs gobs2(TYPE_C, b2, ATTR);

        int addObs = 0;
        double Pobs = satdata.P3(gobs1, gobs2);
        double P1 = satdata.obs_C(gobs1);
        double P2 = satdata.obs_C(gobs2);

        if (_observ == OBSCOMBIN::IONO_FREE && double_eq(Pobs, 0.0))
            return -1;
        if (_observ == OBSCOMBIN::RAW_SINGLE && double_eq(P1, 0.0))
            return -1;
        if (_observ == OBSCOMBIN::RAW_MIX && double_eq(P1, 0.0) && double_eq(P2, 0.0))
            return -1;

        bool com = true;
        if (_gnav)
            com = _gnav->com();

        // this part for getting the resuids l (gpsmod)
        // TODO:: applyDCB
        //_applyDCB(satdata, Pobs, _observ);
        if (_observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_MIX)
        {
            double modelPobs = _gModel->cmpObs(_epoch, _glsq->_x_solve, satdata, gobs1, com);
            if (modelPobs < 0)
                return -1;
            OMC(iobs) = P1 - modelPobs;

            if (_observ == OBSCOMBIN::RAW_MIX && !double_eq(P2, 0.0))
            {
                addObs = 1;
                modelPobs = _gModel->cmpObs(_epoch, _glsq->_x_solve, satdata, gobs2, com);
                if (modelPobs < 0)
                    addObs = 0;
                else
                    OMC(iobs + addObs) = P2 - modelPobs;
            }
        }
        else
        {
            double modelPobs = _gModel->cmpObs(_epoch, _glsq->_x_solve, satdata, gobs1, com);
            if (modelPobs < 0)
                return -1;
            OMC(iobs) = Pobs - modelPobs;
        }

        // This part for getting the  Weight P
        // TODO Maybe edit the _weight
        if (_observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_MIX)
        {
            double weightObs = _weightObs(satdata, _observ, &gobs1);
            if (weightObs < 0)
                return -1;
            P(iobs) = weightObs;
            P(iobs + addObs) = weightObs;
        }
        else
        {
            double weightObs = _weightObs(satdata, _observ, &gobs1, &gobs2);
            if (weightObs < 0)
                return -1;
            P(iobs) = weightObs;
        }

        // This part for getting the  Design Matrix B
        base_allpar &param = _glsq->_x_solve;
        for (unsigned int ipar = 0; ipar < param.parNumber(); ipar++)
        {
            B(iobs, ipar) = param[ipar].partial(satdata, _epoch, ell, gobs1);
            if (_observ == OBSCOMBIN::RAW_MIX && addObs == 1)
                B(iobs + 1, ipar) = param[ipar].partial(satdata, _epoch, ell, gobs2);
        }

        // Only have PC
        iobs++;
        iobs += addObs; // greater than 0 only RAW_MIX.

        return 1;
    }

    // SPP have no use of phase
    int gnss_proc_spplsq::_addObsL(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &B, Vector &OMC, Diag &P)
    {

        return 1;
    }

    int gnss_proc_spplsq::_addObsD(gnss_data_sats &satdata, unsigned int &iobs, Triple &XYZ, Matrix &A, Vector &l, Diag &P)
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
            //b1 = gnss_sys::band_priority(gs, FREQ_1);
            //b2 = gnss_sys::band_priority(gs, FREQ_2);
            //b3 = gnss_sys::band_priority(gs, FREQ_3);
            //b4 = gnss_sys::band_priority(gs, FREQ_4);
            //b5 = gnss_sys::band_priority(gs, FREQ_5);

            //lvhb modified
            b1 = _band_index[gs][FREQ_1];
            b2 = _band_index[gs][FREQ_2];
            b3 = _band_index[gs][FREQ_3];
            b4 = _band_index[gs][FREQ_4];
            b5 = _band_index[gs][FREQ_5];
        }

        //gnss_data_obs gobs1, gobs2, gobs3, gobs4, gobs5;
        gnss_data_obs gobs1(TYPE_D, b1, ATTR);
        gnss_data_obs gobs2(TYPE_D, b2, ATTR);
        gnss_data_obs gobs3(TYPE_D, b3, ATTR);
        gnss_data_obs gobs4(TYPE_D, b4, ATTR);
        gnss_data_obs gobs5(TYPE_D, b5, ATTR);

        double D1, D2, D3, D4, D5;
        D1 = D2 = D3 = D4 = D5 = 0.0;
        std::vector<int> index_freq(5, 0);
        D1 = satdata.obs_D(gobs1);
        D2 = satdata.obs_D(gobs2);
        if (b3 != BAND)
            D3 = satdata.obs_D(gobs3);
        if (b4 != BAND)
            D4 = satdata.obs_D(gobs4);
        if (b5 != BAND)
            D5 = satdata.obs_D(gobs5);

        double weightObs = _weightObs(satdata, _observ, &gobs1, &gobs2);
        //double weight_coef = _weightObs(satdata, gobs1);
        int obsnum = 0;
        if (!double_eq(D1, 0))
        {
            obsnum++;
            index_freq[0] = 1;
        }
        if (!double_eq(D2, 0))
        {
            obsnum++;
            index_freq[1] = 1;
        }
        if (!double_eq(D3, 0))
        {
            obsnum++;
            index_freq[2] = 1;
        }
        if (!double_eq(D4, 0))
        {
            obsnum++;
            index_freq[3] = 1;
        }
        if (!double_eq(D5, 0))
        {
            obsnum++;
            index_freq[4] = 1;
        }

        if (obsnum == 0)
            return -1;

        //Create reduced measurements (prefit residuals)
        double modObsD = 0.0;
        int n = 0;
        auto &param = _glsq->_x_solve;
        for (int i = 0; i < index_freq.size(); i++)
        {
            double Di = 0.0;
            if (!index_freq[i])
                continue;
            if (i == 0)
            {
                modObsD = _gModel->cmpObsD(_epoch, param, satdata, gobs1);
                Di = D1;
            }
            if (i == 1)
            {
                modObsD = _gModel->cmpObsD(_epoch, param, satdata, gobs2);
                Di = D2;
            }
            if (i == 2)
            {
                modObsD = _gModel->cmpObsD(_epoch, param, satdata, gobs3);
                Di = D3;
            }
            if (i == 3)
            {
                modObsD = _gModel->cmpObsD(_epoch, param, satdata, gobs4);
                Di = D4;
            }
            if (i == 4)
            {
                modObsD = _gModel->cmpObsD(_epoch, param, satdata, gobs5);
                Di = D5;
            }
            /*if (fabs(-Di - modObsD) > 0.1)
            {
                obsnum--;
                continue;
            }*/
            if (double_eq(modObsD, 0.0) || double_eq(Di, 0.0))
                continue;
            l(iobs + n) = -Di - modObsD;
            n++;
        }

        // Create weight matrix
        for (int i = 0; i < obsnum; i++)
            P(iobs + i) = weightObs;

        // Create first design matrix
        double A_conf = 0.0;
        int i = param.getParam(_site, par_type::VEL_X, "");
        int j = param.getParam(_site, par_type::VEL_Y, "");
        int k = param.getParam(_site, par_type::VEL_Z, "");
        Triple groundVel(param[i].value(), param[j].value(), param[k].value());

        n = 0;
        for (int i = 0; i < index_freq.size(); i++)
        {
            if (!index_freq[i])
                continue;
            for (unsigned int ipar = 0; ipar < param.parNumber(); ipar++)
            {
                if (i == 0)
                    A_conf = param[ipar].partial_doppler(satdata, XYZ, groundVel);
                if (i == 1)
                    A_conf = param[ipar].partial_doppler(satdata, XYZ, groundVel);
                if (i == 2)
                    A_conf = param[ipar].partial_doppler(satdata, XYZ, groundVel);
                if (i == 3)
                    A_conf = param[ipar].partial_doppler(satdata, XYZ, groundVel);
                if (i == 4)
                    A_conf = param[ipar].partial_doppler(satdata, XYZ, groundVel);
                if (double_eq(A_conf, 0.0))
                    continue;
                if (param[ipar].parType == par_type::CRD_X || param[ipar].parType == par_type::CRD_Y || param[ipar].parType == par_type::CRD_Z)
                    A_conf = 0.0;
                A(iobs + n, ipar) = A_conf;
            }
            n++;
        }

        iobs = iobs + obsnum;

        return 1;
    }

    // Satelite position
    // ----------
    int gnss_proc_spplsq::_satPos(base_time &epo, gnss_data_sats &gsatdata)
    {

        std::string satname = gsatdata.sat();

        int i;

        int base_size = dynamic_cast<set_gen *>(_set)->list_base().size();
        auto nppmodel = dynamic_cast<set_gproc *>(_set)->npp_model(); //lvhb added
        // modified by zhShen
        if (dynamic_cast<set_gproc *>(_set)->realtime() && !base_size && nppmodel != NPP_MODEL::URTK)
            i = gsatdata.addprd_realtime(_gnav); //add sat crd and clk
        else
            i = gsatdata.addprd(_gnav); //add sat crd and clk

        if (i < 0)
            return i;

        return 1;
    }

    int gnss_proc_spplsq::_apply_tides(base_time &_epoch, Triple &xRec)
    {
        // spp don't apply tides
        return 1;
    }

    double gnss_proc_spplsq::_weightObs(gnss_data_sats &satdata, OBSCOMBIN obstype, gnss_data_obs *go1, gnss_data_obs *go2)
    {
        // TODO according to obs type
        // This weight is computing according to the Panda

        GSYS gs = satdata.gsys();
        GOBSTYPE type = go1->type();

        double factor = 1.0;
        double sigRange = 0.0;
        // get the sys sigRange
        if (type == TYPE_C)
        {
            switch (gs)
            {
            case GPS:
                sigRange = _sigCodeGPS;
                break;
            case GLO:
                sigRange = _sigCodeGLO;
                break;
            case GAL:
                sigRange = _sigCodeGAL;
                break;
            case BDS:
                sigRange = _sigCodeBDS;
                break;
            case QZS:
                sigRange = _sigCodeQZS;
                break;
            default:
                sigRange = 0.0;
                return -1;
            }
        }
        else if (type == TYPE_L)
        {
            switch (gs)
            {
            case GPS:
                sigRange = _sigPhaseGPS;
                break;
            case GLO:
                sigRange = _sigPhaseGLO;
                break;
            case GAL:
                sigRange = _sigPhaseGAL;
                break;
            case BDS:
                sigRange = _sigPhaseBDS;
                break;
            case QZS:
                sigRange = _sigPhaseQZS;
                break;
            default:
                sigRange = 0.0;
                return -1;
            }
        }
        else if (type == TYPE_D)
        {
            switch (gs)
            {
            case GPS:
                sigRange = _sigDopplerGPS;
                break;
            case GLO:
                sigRange = _sigDopplerGLO;
                break;
            case GAL:
                sigRange = _sigDopplerGAL;
                break;
            case BDS:
                sigRange = _sigDopplerBDS;
                break;
            case QZS:
                sigRange = _sigDopplerQZS;
                break;
            default:
                sigRange = 0.0;
            }
        }
        else
        {
            return -1;
        }

        // get the _weight factor
        std::string strGOBS = "";
        switch (_weight)
        {
        case OBSWEIGHT::DEF_OBS_WEIGHT:
            std::cerr << "gspplsq: WeightObs (default) should not happened!\n";
            break;
        case OBSWEIGHT::EQUAL:
            factor *= 1;
            break;
        case OBSWEIGHT::SINEL:
            factor *= 1.0 / 2.0 / sin(satdata.ele());
            break;
        case OBSWEIGHT::SINEL2:
            factor *= 1.0 / 2.0 / pow(sin(satdata.ele()), 2);
            break;
        case OBSWEIGHT::SINEL4:
            factor *= 1.0 / 2.0 / pow(sin(satdata.ele()), 4);
            break;
        case OBSWEIGHT::PARTELE:
            if (satdata.ele_deg() <= 30.0)
            {
                factor *= (1.0 / 2.0 / sin(satdata.ele()));
            }
            else
                factor *= 1.0;
            break;
        case OBSWEIGHT::SNR:
            strGOBS = gobs2str(satdata.select_range(go1->band(), true));
            strGOBS.replace(0, 1, "S");
            factor *= sqrt(134.02 * pow(10, -(satdata.getobs(str2gobs(strGOBS)) / 17.91)));
            break;
        default:
            std::cerr << "gspplsq: we can't deal with this WeightObs method!\n";
            return -1;
        }

        // get the combine obs factor
        // TODO finish other weight method
        double coef1 = 0.0, coef2 = 0.0;
        switch (obstype)
        {
        case OBSCOMBIN::IONO_FREE:
            satdata.coef_ionofree(go1->band(), coef1, go2->band(), coef2);
            sigRange *= sqrt(coef1 * coef1 + coef2 * coef2);
            break;
        default:
            //std::cerr << "gspplsq: No other weighting method have been implmented!";
            return 1.0 / pow(factor * sigRange, 2);
        }

        return 1.0 / pow(factor * sigRange, 2);
    }

    void gnss_proc_spplsq::_setOut()
    {
        // std::set res file
        gnss_proc_spp::_setOut();

        // std::set output file
        std::string tmp;
        tmp = dynamic_cast<set_out *>(_set)->outputs("flt");
        if (!tmp.empty())
        {
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            _result = new base_iof;
            _result->tsys(base_time::GPS);
            _result->mask(tmp);
            _result->append(dynamic_cast<set_out *>(_set)->append());
        }
    }

    void gnss_proc_spplsq::_printout(const gnss_proc_lsqbase &lsqestimator)
    {
        std::ostringstream os;

        base_time epoch = lsqestimator.epo();
        base_allpar par = lsqestimator._x_solve;
        Symmetric Qx = lsqestimator.Qx();
        Vector dx = lsqestimator.dx();
        std::string str_dsec = base_type_conv::dbl2str(epoch.dsec());

        int idx_x, idx_y, idx_z;
        idx_x = par.getParam(_site, par_type::CRD_X, "");
        idx_y = par.getParam(_site, par_type::CRD_Y, "");
        idx_z = par.getParam(_site, par_type::CRD_Z, "");

        Triple xyz, ell, xyz_standard;
        xyz[0] = par[idx_x].value() + dx(idx_x + 1);
        xyz[1] = par[idx_y].value() + dx(idx_y + 1);
        xyz[2] = par[idx_z].value() + dx(idx_z + 1);

        // just for test by ZHJ
        xyz_standard[0] = -3.60766537057412e+06;
        xyz_standard[1] = 4.14786796276078e+06;
        xyz_standard[2] = 3.22371704084420e+06;

        xyz2ell(xyz_standard, ell, false);

        Triple dxyz, neu;
        dxyz = xyz - xyz_standard;

        xyz2neu(ell, dxyz, neu);

        os << "lsq:= " << std::fixed << std::setprecision(4)
           << epoch.str_ymdhms() << str_dsec.substr(2) << std::setprecision(4);
        //os << std::setw(20) << neu[0]
        //    << std::setw(20) << neu[1]
        //    << std::setw(20) << neu[2];
        //os << std::endl;
        for (unsigned int ipar = 0; ipar < par.parNumber(); ipar++)
        {
            os << std::setw(15) << par[ipar].value() + dx(ipar + 1);
        }
        os << std::endl;

        // Print flt results
        if (_result)
        {
            _result->write(os.str().c_str(), os.str().size());
            _result->flush();
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "have no output file!");
        }
    }

}