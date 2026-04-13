#include "hwa_set_npp.h"
#include "hwa_set_leo.h"
#include "hwa_base_log.h"
#include "hwa_gnss_model_precisebiasgpp.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_set_gen.h"
#include "hwa_gnss_all_prec.h"
#include "hwa_gnss_proc_lsqmatrix.h"

using namespace std;
using namespace hwa_set;

namespace hwa_gnss
{
    gnss_model_precise_biasgpp::gnss_model_precise_biasgpp( base_all_proc *data, hwa_set::set_base *setting) : gnss_model_precise_bias(data, setting)
    {
        auto nppmodel = dynamic_cast<set_gproc *>(setting)->npp_model();
        int base_size = dynamic_cast<set_gen *>(setting)->list_base().size();
        _ddmode = (base_size ? true : (nppmodel == NPP_MODEL::URTK ? true : false));
    }

    gnss_model_precise_biasgpp::gnss_model_precise_biasgpp( base_all_proc *data, base_log spdlog, hwa_set::set_base *setting) : gnss_model_precise_bias(data, spdlog, setting)
    {
        auto nppmodel = dynamic_cast<set_gproc *>(setting)->npp_model();
        int base_size = dynamic_cast<set_gen *>(setting)->list_base().size();
        _ddmode = (base_size ? true : (nppmodel == NPP_MODEL::URTK ? true : false));
    }

    gnss_model_precise_biasgpp::~gnss_model_precise_biasgpp()
    {
    }

    bool gnss_model_precise_biasgpp::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result)
    {
        // check obs_type valid
        double Obs_value = obsdata.getobs(gobs.gobs());
        // double snr_value = obsdata.getobs(pl2snr(gobs.gobs()));
        if (double_eq(Obs_value, 0.0))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "Obs_value is 0.0");
            return false;
        }

        tuple<string, string, base_time>* flag = &_rec_sat_before;
        if (make_tuple(obsdata.site(), obsdata.sat(), epoch) != *flag)
        {
            bool update_valid = gnss_model_precise_biasgpp::_update_obs_info_GPP(epoch, _gall_nav, _gallobj, obsdata, params);
            if (!update_valid)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "update obs information failed" + epoch.str_ymdhms("", false));
                return false;
            }

            // prepare Caculate some common bias[sat_pos,rec_pos,relative,rho]
            bool pre_valid = gnss_model_precise_biasgpp::_prepare_obs_GPP(epoch, _gall_nav, _gallobj, params);
            if (!pre_valid)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "prepare obs information failed" + epoch.str_ymdhms("", false));
                return false;
            }
            *flag = make_tuple(obsdata.site(), obsdata.sat(), epoch);
        }

        // combine equ
        gnss_proc_lsq_equationmatrix equ;
        double omc = 0.0, wgt = 0.0;
        vector<pair<int, double>> coef;

        if (!_omc_obs_all(epoch, _crt_obs, params, gobs, omc))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "omc obs failed");
            return false;
        };

        if (!_wgt_obs_all(base_data::REC, gobs, _crt_obs, 1.0, wgt))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "weight obs failed");
            return false;
        };

        if (!_prt_obs_all(epoch, _crt_obs, params, gobs, coef))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "partialrange obs failed");
            return false;
        }

        result.B.push_back(coef);
        result.P.push_back(wgt);
        result.l.push_back(omc);

        _update_obs_info(obsdata);
        return true;
    }

    double gnss_model_precise_biasgpp::cmpObs(base_time &epoch, string &sat, string &rec, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs)
    {
        if (_gallobj == nullptr)
        {
            throw logic_error("gallobj is nullptr in precisemodel cmpObs");
        }

        // Cartesian coordinates to ellipsodial coordinates
        Triple xyz, ell;

        // give crd initial value modified by glfeng
        //modified by lvhb , but no useing in there 20200917
        /*string strEst = dynamic_cast<set_gen*>(_gset)->estimator();
        bool isFLT = (strEst == "FLT");*/
        if (_is_flt)
        {
            if (param.getCrdParam(rec, xyz) < 0)
            {
                xyz = _crt_obj->crd_arp(epoch);
            }
        }
        else
        {
            if (param.getCrdParam(rec, xyz) < 0)
            {
                xyz = _crt_obj->crd(epoch);
            }
            xyz += _crt_obj->eccxyz(epoch);
        }

        xyz2ell(xyz, ell, false);

        Triple satcrd = gsatdata.satcrd();
        Triple cSat = satcrd;

        // Tropospheric wet delay correction
        double trpDelay = 0;
        trpDelay = tropoDelay(epoch, rec, param, ell, gsatdata);
        if (fabs(trpDelay) > 50)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "trpDelay > 50");
            return -1;
        }

        // idx for par correction
        int i = -1;

        // Receiver clock correction
        double clkRec = 0.0;
        i = param.getParam(rec, par_type::CLK, "");
        if (i >= 0)
        {
            clkRec = param[i].value();
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, rec + " ! warning:  Receiver Clock is not included in parameters!");
        }

        // system time offset
        double isb_offset = isbDelay(param, gsatdata, sat, rec, gobs);

        // jdhuang : DCB is correct by apply DCB all
        // Because it is only used for non-combination, so I directly deleted.
        // Inter frequency code bias FREQ_3 (receiver)
        double ifb = ifbDelay(param, gsatdata, sat, rec, gobs);

        // Used for debugging, don't delete!
#ifdef DEBUG_LEO
        cout << setw(20) << " EPOCH is :     " << epoch.str_mjdsod() << " " << setw(6) << gsatdata.sat() << setw(6) << gsatdata.site()
             << setw(20) << " Band is :      " << setw(6) << gobs.band()
             << setw(20) << " gsatdata.rho() " << fixed << left << setw(20) << setprecision(6) << gsatdata.rho()
             << setw(20) << " clkRec         " << fixed << left << setw(20) << setprecision(6) << clkRec
             << setw(20) << " gsatdata.clk() " << fixed << left << setw(20) << setprecision(6) << gsatdata.clk()
             << setw(20) << " trpDelay       " << fixed << left << setw(20) << setprecision(6) << trpDelay
             << endl;
#endif
        // Wind up correction
        double wind = 0.0;
        if (gobs.is_phase())
        {
            wind = windUp(gobs.band(), gsatdata, _trs_rec_crd);
        }

        // ion correction
        double ion = 0.0;
        auto band_1 = _band_index[gsatdata.gsys()][FREQ_1];
        ion = ionoDelay(epoch, param, gsatdata, _ion_model, band_1, gobs);

        // pcv correction
        double pcv = PCV(_corrt_sat_pcv, _corrt_rec_pcv, _realtime, epoch, _crt_sat_epo, _trs_rec_crd, gsatdata, gobs);
        //double gnss_model_precise_bias::PCV(bool corrt_sat, bool corrt_rec, bool realtime, base_time & epoch, base_time & crt_sat_epo, Triple & trs_rec_crd, gnss_data_sats & satdata, gnss_data_obs & gobs);

        // Return value
        //cout << setw(20) << " EPOCH is :     " << epoch.str_mjdsod() << " " << setw(6) << gsatdata.sat() << setw(6) << gsatdata.site()
        //    << setw(20) << " Band is :      " << setw(6) << gobs.band()
        //    << setw(20) << " gsatdata.rho() " << fixed << left << setw(20) << setprecision(6) << gsatdata.rho()
        //    << setw(20) << " clkRec         " << fixed << left << setw(20) << setprecision(6) << clkRec
        //    << setw(20) << " gsatdata.clk() " << fixed << left << setw(20) << setprecision(6) << gsatdata.clk()
        //    << setw(20) << " trpDelay       " << fixed << left << setw(20) << setprecision(6) << trpDelay
        //    << setw(20) << " isb_offset       " << fixed << left << setw(20) << setprecision(6) << isb_offset
        //    << setw(20) << " ifb       " << fixed << left << setw(20) << setprecision(6) << ifb
        //    << setw(20) << " ion       " << fixed << left << setw(20) << setprecision(6) << ion
        //    << setw(20) << " wind       " << fixed << left << setw(20) << setprecision(6) << wind
        //    << setw(20) << " pcv       " << fixed << left << setw(20) << setprecision(6) << pcv
        //    << endl;
        return gsatdata.rho() +
               clkRec -
               gsatdata.clk() +
               trpDelay +
               isb_offset +
               ifb +
               ion +
               wind +
               pcv;
    }

    bool gnss_model_precise_biasgpp::_prepare_obs_GPP(const base_time &crt_epo, gnss_all_nav *gallnav, gnss_all_obj *gallobj, base_allpar &pars)
    {
        if (!gallnav || !gallobj)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "no navgation data or atx data for epoch " + crt_epo.str_ymdhms());
            return false;
        }

        // compute reciver time
        _crt_rec_epo = crt_epo - _crt_rec_clk;
        _crt_obs.addrecTime(_crt_rec_epo);

        // get Rec crd
        bool apply_obj_valid = gnss_model_precise_bias::_apply_rec(crt_epo, _crt_rec_epo, pars);
        if (!apply_obj_valid)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "can not apply site in " + crt_epo.str_ymdhms());
            return false;
        }

        // get sat crd
        bool apply_sat_valid = _apply_sat(_crt_rec_epo, _crt_sat_epo, gallnav);
        if (!apply_sat_valid)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "can not apply sat in " + crt_epo.str_ymdhms());
            return false;
        }
        _crt_obs.addsatTime(_crt_sat_epo);

#ifdef DEBUG
        cout << "site crs" << scientific << setw(25) << setprecision(15) << _crdSiteCrs[0] << "  " << _crdSiteCrs[1] << "  " << _crdSiteCrs[2] << "  " << endl;
        cout << "sat crs" << scientific << setw(25) << setprecision(15) << _crdSatCrs[0] << "  " << _crdSatCrs[1] << "  " << _crdSatCrs[2] << "  " << endl;
#endif // DEBUG21

        // get f01 PCO
        this->_crs_rec_pco = this->_crs_rec_crd;
        this->_crs_sat_pco = this->_crs_sat_crd;

        shared_ptr<gnss_data_obj> sat_obj = this->_gallobj->obj(_crt_sat);
        shared_ptr<gnss_data_obj> rec_obj = this->_gallobj->obj(_crt_rec);

        shared_ptr<gnss_data_pcv> sat_pcv = (sat_obj != 0) ? sat_obj->pcv(crt_epo) : nullptr;
        shared_ptr<gnss_data_pcv> rec_pcv = (rec_obj != 0) ? rec_obj->pcv(crt_epo) : nullptr;

        //zzwu changes according to lvhb
        if (!_isCalSatPCO)
        {
            sat_pcv = nullptr;
        }

        if (sat_pcv&&!_realtime) //add _realtime by xiongyun
        {
            // Satellite phase center offset
            Triple pco(0, 0, 0);
            if (sat_pcv->pcoS(_crt_obs, pco, _observ, _band_index[_crt_sys][FREQ_1], _band_index[_crt_sys][FREQ_2]) > 0)
            {
                //Matrix _rot_matrix = _RotMatrix_Ant(_crt_obs, _crt_sat_epo, sat_obj, true);
                Matrix _rot_matrix = _RotMatrix_Ant(_crt_obs, _crt_epo, _crt_sat_epo, sat_obj, true);
                this->_crs_sat_pco = this->_crs_sat_pco + _rot_matrix * pco;
            }
        }

        if (rec_pcv)
        {
            // Receiver phase center offset
            Triple pco(0.0, 0.0, 0.0);
            if (rec_pcv->pcoR(_crt_obs, pco, _observ, _band_index[_crt_sys][FREQ_1], _band_index[_crt_sys][FREQ_2]) > 0)
            {
                //Matrix _rot_matrix = _RotMatrix_Ant(_crt_obs, _crt_rec_epo, rec_obj, true);
                Matrix _rot_matrix = _RotMatrix_Ant(_crt_obs, _crt_epo, _crt_rec_epo, rec_obj, true);
                this->_crs_rec_pco = this->_crs_rec_pco + _rot_matrix * pco;
            }
        }

        // compute satclk[s]
        double sat_clk = _crt_sat_clk;
        // compute reldelay[m]
        double reldelay = relDelay(this->_crs_rec_pco, this->_crs_rec_vel, this->_crs_sat_pco, this->_crs_sat_vel);
        
        _crt_obs.addclk(sat_clk * CLIGHT - reldelay); // include reldelay
        _crt_obs.addreldelay(reldelay);                  // for debug

        // addrho
        double tmp = (_crs_sat_crd - _crs_rec_crd).norm();
        _crt_obs.addrho(tmp);

        // add drate
        _crt_obs.adddrate(((_crs_sat_vel - _crs_rec_vel).dot((_crs_sat_pco - _crs_rec_pco))) / (CLIGHT * tmp)); //yjqin

        // add azim && elev
        Triple xyz_rho = _crs_sat_pco - _crs_rec_pco;
        Triple ell_r, neu_s;

        Matrix _rot_ant2crs = _RotMatrix_Ant(_crt_obs, _crt_rec_epo, _crt_rec_epo, _crt_obj, true);
        neu_s = _rot_ant2crs.transpose() * xyz_rho;
        double NE2 = neu_s[0] * neu_s[0] + neu_s[1] * neu_s[1];
        double ele = acos(sqrt(NE2) / _crt_obs.rho());
        if (sqrt(NE2) / _crt_obs.rho() > 1.0)
        {
            _crt_obs.addele(0.0);
        }
        else
        {
            _crt_obs.addele(ele);
        }
        // for off-nadir angle, yqyuan
        double offnadir = xyz_rho.dot(_crs_sat_pco) / xyz_rho.norm() / _crs_sat_pco.norm();
        offnadir = acos(offnadir);
        _crt_obs.addnadir(offnadir);
        _crt_obs.addzen_sat(offnadir);

        double azi = atan2(neu_s[1], neu_s[0]);
        if (azi < 0)
        {
            azi += 2 * hwa_pi;
        }
        _crt_obs.addazi_rec(azi);
        _crt_obs.addzen_rec((hwa_pi / 2.0 - ele));

        /// for satellite-side azimuth, added by yqyuan for satellite-side aizmuth-dependent PCV
        //Matrix _rot_matrix_scf2crs = _RotMatrix_Ant(_crt_obs, _crt_sat_epo, sat_obj, true);
        Matrix _rot_matrix_scf2crs = _RotMatrix_Ant(_crt_obs, _crt_epo, _crt_sat_epo, sat_obj, true);
        Triple xyz_s2r = _rot_matrix_scf2crs.transpose() * ((-1) * xyz_rho); // from sat. to rec. in SCF XYZ
        double azi_sat = atan2(xyz_s2r[0], xyz_s2r[1]);
        if (azi_sat < 0)
            azi_sat += 2 * hwa_pi;
        _crt_obs.addazi_sat(azi_sat);

        ///add for another elev and azi used in calculating weight matric
        Triple xyz_rh = _trs_sat_crd - _trs_rec_crd;
        Triple ell_(0, 0, 0), neu_sa(0, 0, 0), xRec(0, 0, 0), xyz_s(0, 0, 0);
        xyz2ell(_trs_rec_crd, ell_, false);
        xyz2neu(ell_, xyz_rh, neu_sa);
        double rho0 = sqrt(pow(_trs_rec_crd[0] - xyz_s[0], 2) + pow(_trs_rec_crd[1] - xyz_s[1], 2) + pow(_trs_rec_crd[2] - xyz_s[2], 2));
        double dPhi = OMEGA * rho0 / CLIGHT;
        xRec[0] = _trs_rec_crd[0] * cos(dPhi) - _trs_rec_crd[1] * sin(dPhi);
        xRec[1] = _trs_rec_crd[1] * cos(dPhi) + _trs_rec_crd[0] * sin(dPhi);
        xRec[2] = _trs_rec_crd[2];
        double NE2_ = neu_sa[0] * neu_sa[0] + neu_sa[1] * neu_sa[1];
        double ele_ = acos(sqrt(NE2_) / _crt_obs.rho());

        _crt_obs.addele_leo(ele_);
        

        // check elevation cut-off
        if (_crt_obj->id_type() == base_data::REC && _crt_obs.ele_deg() < _minElev)
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "Prepare fail! the elevation is too small");
            return false;
        }

        return true;
    }

    bool gnss_model_precise_biasgpp::_update_obs_info_GPP(const base_time &epoch, gnss_all_nav *nav, gnss_all_obj *gallobj, gnss_data_sats &obsdata, base_allpar &pars)
    {
        if (!nav || !gallobj)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "no pars");
            return false;
        }

        // Some frequently used variables are also defined here.
        _crt_epo = epoch;
        _crt_obs = obsdata;
        _crt_obs.tb12(obsdata.tb12());
        _crt_obs.tb13(obsdata.tb13()); 
        _crt_sat = obsdata.sat();
        _crt_rec = obsdata.site();
        _crt_sys = obsdata.gsys();
        _crt_obj = _gallobj->obj(_crt_rec);
        _trs_sat_crd = obsdata.satcrd(); //lvhb added in 20200912

        //zzwu change
        if(!_corrt_sat_pcv)
        {
            this->reset_SatPCO(false);
        }

        bool rec_clk_valid = _update_obj_clk_GPP("rec" + _crt_rec, epoch, nav, pars, _crt_rec_clk, _obj_clk);
        bool sat_clk_valid = _update_obj_clk_GPP("sat" + _crt_sat, epoch, nav, pars, _crt_sat_clk, _obj_clk);
        if (!rec_clk_valid || !sat_clk_valid)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "no rec or sat clk for epoch " + epoch.str_ymdhms());
            return false;
        }
        return true;
    }

    bool gnss_model_precise_biasgpp::_update_obj_clk_GPP(const string &obj, const base_time &crt_epoch, gnss_all_nav *nav, base_allpar &pars, double &clk, map<string, pair<base_time, double>> &obj_clk)
    {
        // jdhuang
        if (!nav || obj.substr(3).empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "no navigation files or obj is NONE");
            return false;
        }

        // clk[s] from clk files
        double clk_rms = 0.0;
        double dclk = 0.0;
        string type = obj.substr(0, 3);
        string name = obj.substr(3);
        auto nppmodel = dynamic_cast<set_gproc *>(_gset)->npp_model();
        int base_size = dynamic_cast<set_gen *>(_gset)->list_base().size();
        int pv_iod = 0;   // to do
        int    clk_iod = 0;

        // get clk from clk files
        //--can not get any clk informtion from file or gallpar
        int idx_clk = -1;
        if (type == "rec")
        {
            idx_clk = pars.getParam(name, par_type::CLK, "");
        }

        // update clk
        if (obj_clk[name].first != crt_epoch)
        {
            int clk_valid;
            if (_realtime && type == "sat")
            {
                
                if (dynamic_cast<gnss_all_prec*>(_gall_nav)->get_ssr_iod(name, crt_epoch, pv_iod, clk_iod) < 0)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "satellite" + name 
                            + crt_epoch.str_ymdhms(" coordinates and clock correction is not exist for ephemeris")
                            + "(PV_IOD: " + base_type_conv::int2str(pv_iod) + ", CLK_IOD: " + base_type_conv::int2str(clk_iod) + " )");
                    return false;
                }
                if (pv_iod != clk_iod)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "satellite" + name
                            + crt_epoch.str_ymdhms(" PV_IOD and CLK_IOD is not equal for ephemeris")
                            + "(PV_IOD: " + base_type_conv::int2str(pv_iod) + ", CLK_IOD: " + base_type_conv::int2str(clk_iod) + " )");
                    return false;
                }
                clk_valid = _gall_nav->clk(name, pv_iod, crt_epoch, &clk, &clk_rms, &dclk, true);
            }
            else clk_valid = _gall_nav->clk(name, crt_epoch, &clk, &clk_rms, &dclk);

            if (clk_valid < 0)
                clk = 0.0;
            if (_realtime && type == "sat" && (idx_clk < 0) && // not used for pce, jqwu
                (base_size == 0) && (nppmodel != NPP_MODEL::URTK) &&
                !_get_sat_clk_corr(crt_epoch, name, clk, dclk))
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "no sat" + name + "  clk correction for " + crt_epoch.str_ymdhms());
                return false;
            } // zhshen

            obj_clk[name].first = crt_epoch;
            obj_clk[name].second = clk;
        }
        else
        {
            clk = obj_clk[name].second;
        }

        if (double_eq(clk, 0.0) && idx_clk < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "no clk for " + name + "  " + crt_epoch.str_ymdhms());
            return false;
        }
        //-- no clk file but clk par exist
        if (double_eq(clk, 0.0) && idx_clk >= 0)
        {
            clk = pars[idx_clk].value() / CLIGHT;
        }
        //-- update clk par
        if (idx_clk >= 0)
        {
            pars[idx_clk].value(clk * CLIGHT);
        }

        return true;
    }

    bool gnss_model_precise_biasgpp::_prt_obs_all(const base_time &crt_epo, gnss_data_sats &obsdata, base_allpar &pars, gnss_data_obs &gobs, vector<pair<int, double>> &coeff)
    {
        auto par_list = pars.getPartialIndex(_crt_rec, _crt_sat);
        for (const int& ipar : par_list)
        {
            base_par par = pars.getPar(ipar);
            double coeff_value = 0.0;
            gnss_model_bias::_Partial_basic(crt_epo, _crt_obs, gobs, par, coeff_value);
            if (coeff_value != 0.0)
            {
                //note: This is bad from 1 in coeff vector
                coeff.push_back(make_pair(ipar, coeff_value));
            }
        }
        return true;
    }

    bool gnss_model_precise_biasgpp::_get_sat_clk_corr(const base_time &sat_epoch, const string &sat, double &clk, double &dclk)
    {
        if (!_gall_nav)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "there are no nav");
            return false;
        }

        bool clk_valid = true;
        int cor_valid = -1;
        double xyz_corr[3] = {0.0, 0.0, 0.0};
        // double vel[3] = {0.0, 0.0, 0.0};
        double vel_corr[3] = {0.0, 0.0, 0.0};
        // double var[3] = {0.0, 0.0, 0.0};
        double clk_corr = 0.0;
        double dclk_corr = 0.0;

        pair<int, int> iods = _gall_nav->get_iod(sat, sat_epoch);
        if (iods.first < 0 && iods.second < 0)
            return false;

        cor_valid = _gall_nav->get_pos_clk_correction(sat, sat_epoch, iods.first, xyz_corr, vel_corr, clk_corr, dclk_corr);

        if (cor_valid < 0 || double_eq(clk_corr, 0.0))
        {
            if (_gall_nav->get_pos_clk_correction(sat, sat_epoch, iods.second, xyz_corr, vel_corr, clk_corr, dclk_corr) < 0 || double_eq(clk_corr, 0.0))
            {
                clk_valid = false;
            }
        }

        _correction_clk(clk, dclk, clk_corr, dclk_corr);

        if (clk_valid)
            return clk_valid;
        else if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, sat_epoch.str_ymdhms("can not get sat clk in epoch", false));
        return false;
    }
}
