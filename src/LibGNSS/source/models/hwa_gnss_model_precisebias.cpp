#include "hwa_set_leo.h"
#include "hwa_gnss_all_prec.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_set_gen.h"
#include "hwa_set_inp.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_gnss_model_gmf.h"
#include "hwa_gnss_model_tideiers.h"

using namespace std;
using namespace hwa_set;

#ifndef OMGE_DOT
#define OMGE_DOT 7.2921151467e-5 ///< WGS 84 value of the base_earth's rotation rate [rad/sec]
#endif // !1

namespace hwa_gnss
{
    gnss_model_precise_bias::gnss_model_precise_bias( base_all_proc *data, set_base *setting) : gnss_model_bias(setting)
    {
        _gall_nav = dynamic_cast<gnss_all_nav *>((*data)[base_data::GRP_EPHEM]);
        _gallobj = dynamic_cast<gnss_all_obj *>((*data)[base_data::ALLOBJ]);
        _realtime = dynamic_cast<set_gproc *>(setting)->realtime();
        _ultrasp3 = dynamic_cast<set_gproc*>(setting)->ultrasp3(); //add by xiongyun
        _gdata_erp = dynamic_cast<gnss_data_poleut *>((*data)[base_data::ALLPOLEUT1]);
        _gdata_navde = dynamic_cast<gnss_data_navde *>((*data)[base_data::ALLDE]);
        _tide = std::shared_ptr<gnss_model_tide>(new gnss_model_tideiers(dynamic_cast<gnss_all_otl *>((*data)[base_data::ALLOTL])));
        _minElev = dynamic_cast<set_gproc *>(setting)->minimum_elev();
        _crd_est = dynamic_cast<set_gproc *>(setting)->crd_est();
        _is_flt = (dynamic_cast<set_gen *>(setting)->estimator() == "FLT");
        _trop_est = dynamic_cast<set_gproc *>(setting)->tropo();
        if (dynamic_cast<gnss_all_prec *>(_gall_nav))
        {
            _corrt_sat_pcv = !(dynamic_cast<set_inp *>(_gset)->input_size("sp3") == 0 &&
                               dynamic_cast<set_gproc *>(_gset)->ultrasp3() == false);
            if (dynamic_cast<set_inp*>(_gset)->input_size("orbit") != 0)
                _corrt_sat_pcv = true;
        }
        _gifcb = dynamic_cast<gnss_data_ifcb *>((*data)[base_data::IFCB]);
        _attitudes = dynamic_cast<set_gproc *>(setting)->attitudes();

        // yqyuan added for atmoshpheric pressure loading; 2021/11/30
        // yqyuan: if use the Spline Interpolation method, un-comment the second line below
        _atm_loading = dynamic_cast<gnss_all_atmloading *>((*data)[base_data::ALLATL]);
        _opl = dynamic_cast<gnss_all_opl *>((*data)[base_data::ALLOPL]);
        _mean_pole_model = dynamic_cast<set_gproc *>(setting)->mean_pole_model();
    }

    gnss_model_precise_bias::gnss_model_precise_bias( base_all_proc *data, base_log spdlog, set_base *setting) : gnss_model_bias(spdlog, setting)
    {
        _gall_nav = dynamic_cast<gnss_all_nav *>((*data)[base_data::GRP_EPHEM]);
        _gallobj = dynamic_cast<gnss_all_obj *>((*data)[base_data::ALLOBJ]);
        _realtime = dynamic_cast<set_gproc *>(setting)->realtime();
        _gdata_erp = dynamic_cast<gnss_data_poleut *>((*data)[base_data::ALLPOLEUT1]);
        _gdata_navde = dynamic_cast<gnss_data_navde *>((*data)[base_data::ALLDE]);
        _tide = std::shared_ptr<gnss_model_tide>(new gnss_model_tideiers(spdlog, dynamic_cast<gnss_all_otl *>((*data)[base_data::ALLOTL])));
        _minElev = dynamic_cast<set_gproc *>(setting)->minimum_elev();
        _crd_est = dynamic_cast<set_gproc *>(setting)->crd_est();
        _is_flt = (dynamic_cast<set_gen *>(setting)->estimator() == "FLT");
        _trop_est = dynamic_cast<set_gproc *>(setting)->tropo();
        if (dynamic_cast<gnss_all_prec *>(_gall_nav))
        {
            _corrt_sat_pcv = !(dynamic_cast<set_inp *>(_gset)->input_size("sp3") == 0 &&
                               dynamic_cast<set_gproc *>(_gset)->ultrasp3() == false);
            if (dynamic_cast<set_inp *>(_gset)->input_size("orbit") != 0)
                _corrt_sat_pcv = true;
        }
        _gifcb = dynamic_cast<gnss_data_ifcb *>((*data)[base_data::IFCB]);
        _attitudes = dynamic_cast<set_gproc *>(setting)->attitudes();

        // yqyuan added for atmoshpheric pressure loading; 2021/11/30
        // yqyuan: if use the Spline Interpolation method, un-comment the second line below
        _atm_loading = dynamic_cast<gnss_all_atmloading *>((*data)[base_data::ALLATL]);
        _opl = dynamic_cast<gnss_all_opl *>((*data)[base_data::ALLOPL]);
        _mean_pole_model = dynamic_cast<set_gproc *>(setting)->mean_pole_model();
    }

    gnss_model_precise_bias::~gnss_model_precise_bias()
    {
    }

    void gnss_model_precise_bias::set_multi_thread(const std::set<std::string> &sites)
    {
    }

    bool gnss_model_precise_bias::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result)
    {
        return false;
    }

    void gnss_model_precise_bias::update_obj_clk(const std::string &obj, const base_time &epo, double clk)
    {

        _obj_clk[obj].first = epo;
        _obj_clk[obj].second = clk;
        _rec_clk[obj] = clk;
    }

    double gnss_model_precise_bias::get_rec_clk(const std::string &obj)
    {
        return _rec_clk[obj];
    }

    double gnss_model_precise_bias::tropoDelay(base_time &epoch, std::string &rec, base_allpar &param, Triple site_ell, gnss_data_sats &satdata)
    {

        if (site_ell[2] > 1E4)
        {
            return 0.0;
        }
        else
        {
            if (_tropoModel == 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Tropo Model setting is not correct. Default used! Check config.");
                _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
            }

            double ele = satdata.ele();
            double azi = satdata.azi();

            double delay = 0.0;
            double zwd = 0.0;
            double zhd = 0.0;
            int i, j, k;
            Triple ell;

            xyz2ell(_trs_rec_crd, ell, false);

            if (ell[2] > 1E4)
            {
                return 100.0;
            }

            i = param.getParam(_crt_rec, par_type::TRP, "");
            j = param.getParam(_crt_rec, par_type::GRD_N, "");
            k = param.getParam(_crt_rec, par_type::GRD_E, "");

            if (i >= 0)
            {
                zwd = param[i].value();
                if (param[i].apriori() > 1E-4 && (zwd == 0.0 || epoch == param[i].beg))
                {
                    zwd = _tropoModel->getZWD(ell, epoch);
                    param[i].value(zwd);
                }
                zhd = _tropoModel->getZHD(ell, epoch);
                param[i].zhd = zhd;
            }
            else
            {
                if (_tropoModel != 0)
                {
                    zwd = _tropoModel->getZWD(ell, epoch);
                    zhd = _tropoModel->getZHD(ell, epoch);
                }
            }

            double mfh, mfw, dmfh, dmfw;
            mfh = mfw = dmfh = dmfw = 0.0;
            if (_mf_ztd == ZTDMPFUNC::GMF)
            {
                gnss_model_gmf mf;
                mf.gmf(epoch.mjd(), ell[0], ell[1], ell[2], hwa_pi / 2.0 - ele,
                       mfh, mfw, dmfh, dmfw);
            }
            else if (_mf_ztd == ZTDMPFUNC::COSZ)
            {
                mfh = mfw = 1 / sin(ele);
                dmfh = dmfw = -(cos(ele)) / (sin(ele) * sin(ele));
            }
            else
                return 0.0;

            satdata.addmfH(mfh);
            satdata.addmfW(mfw);

            delay = mfh * zhd + mfw * zwd;

            if (!_trop_est && satdata.obsWithCorr())
            {
                delay = mfh * zhd;
            }

#ifdef DEBUG
            //debug by ZHJ
            std::cout << std::fixed << std::setprecision(5)
                 << "ele" << satdata.ele_deg() << std::endl
                 << "dry delay " << zhd << std::endl
                 << "dry std::map" << mfh << std::endl
                 << "wet delay" << zwd << std::endl
                 << "wet std::map" << mfw << std::endl;
#endif // !DEBUG

            double grdN, grdE;
            grdN = grdE = 0.0;

            if (j >= 0 && k >= 0)
            {
                if (_grad_mf == GRDMPFUNC::TILTING)
                {
                    grdN = param[j].value() * dmfw * cos(azi);
                    grdE = param[k].value() * dmfw * sin(azi);
                    satdata.addmfG(dmfw);
                }
                else if (_grad_mf == GRDMPFUNC::CHEN_HERRING)
                {
                    double mfg = 1.0 / (sin(ele) * tan(ele) + 0.0032);
                    grdN = param[j].value() * 1000.0 * mfg * cos(azi);
                    grdN /= 1000.0;
                    grdE = param[k].value() * 1000.0 * mfg * sin(azi);
                    grdE /= 1000.0;
                    satdata.addmfG(mfg);
                }
                else if (_grad_mf == GRDMPFUNC::BAR_SEVER)
                {
                    double mfg = mfw * (1 / tan(ele));
                    grdN = param[j].value() * mfg * cos(azi);
                    grdE = param[k].value() * mfg * sin(azi);
                    satdata.addmfG(mfg);
                }

                delay += grdN + grdE;
            }

#ifdef DEBUG_CORR
            std::cout << epoch.str("EPOCH: %H:%M:%S") << std::endl
                 << std::fixed << std::setprecision(3);
            std::cout << satdata.sat() << " Ell:" << ell[0] << " " << ell[1] << " " << ell[2]
                 << " Hydrostatic part: " << zhd //_tropoModel->getZHD(ell, epoch)
                 << " Wet part: " << zwd
                 << " mfh: " << mfh
                 << " mfw: " << mfw
                 << " Delay: " << delay << std::endl
                 << std::endl;
#endif
            return delay;
        }
    }

    double gnss_model_precise_bias::ionoDelay(base_time &epoch, base_allpar &param, gnss_data_sats &satdata, IONMODEL &ion_model, GOBSBAND &band_1, gnss_data_obs &gobs)
    {

        if (band_1 == BAND || band_1 == BAND_A || band_1 == BAND_B || band_1 == BAND_C || band_1 == BAND_D)
        {
            return 0.0;
        }

        if (gobs.band() == BAND || gobs.band() == BAND_A || gobs.band() == BAND_B || gobs.band() == BAND_C || gobs.band() == BAND_D)
        {
            return 0.0;
        }

        if (ion_model == IONMODEL::VION)
        {
            throw std::logic_error(" not support VION model, use SION in XML");
        }

        double iono_delay = 0.0;

        // jdhuang : from G01_F
        double f1 = satdata.frequency(band_1);
        ;
        double fk = satdata.frequency(gobs.band());
        double alfa = 0.0;

        if (gobs.is_phase())
        {
            alfa = -(f1 * f1) / (fk * fk);
        }
        if (gobs.is_code())
        {
            alfa = (f1 * f1) / (fk * fk);
        }

        // ionosphere slant delay parameter
        int i = param.getParam(_crt_rec, par_type::SION, satdata.sat());
        if (i >= 0)
        {
            iono_delay = alfa * param[i].value();
        }

#ifdef DEBUG
        std::cout << "IONO delay: " << gsatdata.sat() << " " << epoch.str_ymdhms() << " iono delay = " << iono_delay << std::endl;
        //    int ooo; cin >> ooo;
#endif

        return iono_delay;
    }

    double gnss_model_precise_bias::isbDelay(base_allpar &param, gnss_data_sats &satdata, std::string &sat, std::string &rec, gnss_data_obs &gobs)
    {
        double isb_offset = 0.0;

        auto gsys = gnss_sys::sat2gsys(sat);

        switch (gsys)
        {
        case GPS:
        {
            break;
        }
        // GLONASS system time offset
        case GLO:
        {
            int idx_isb = param.getParam(rec, par_type::GLO_ISB, "");
            if (idx_isb >= 0)
            {
                isb_offset = param[idx_isb].value();
                //std::cout << "GLO Offstd::set: " << sat << " " << glonass_offset << " " << gsatdata.epoch().str_hms() << std::endl;
            }

            // add by ZHJ for GLO IFB
            int idx_ifb = param.getParam(rec, par_type::GLO_IFB, sat);
            if (idx_ifb >= 0)
            {
                isb_offset += param[idx_ifb].value();
            }

            break;
        }
        case GAL:
        {
            // Galileo system time offset
            int i = param.getParam(rec, par_type::GAL_ISB, "");
            if (i >= 0)
            {
                isb_offset = param[i].value();
                //std::cout << "GAL Offstd::set: " << sat << " " << galileo_offset << " " << gsatdata.epoch().str_hms() << std::endl;
            }
            break;
        }
        case BDS:
        {
            // BaiDou system time offset
            int i = param.getParam(rec, par_type::BDS_ISB, "");
            if (i >= 0)
            {
                isb_offset = param[i].value();
                //std::cout << "BDS ISB: " << sat << " " << param[i].value() << " " << gsatdata.epoch().str_hms() << std::endl;
            }
            break;
        }
        // QZSS system time offset
        case QZS:
        {
            int i = param.getParam(_crt_rec, par_type::QZS_ISB, "");
            if (i >= 0)
            {
                isb_offset = param[i].value();
            }
            break;
        }
        default:
            throw std::logic_error("can not support such sys ： " + gnss_sys::gsys2str(gsys));
        }

        return isb_offset;
    }
    double gnss_model_precise_bias::relDelay(Triple &crd_crt_rec, Triple &vel_crt_rec, Triple &crd_sat, Triple &vel_sat)
    {
        double reldelay = 0.0;
        reldelay = 2.0 * (crd_sat.dot(vel_sat)) / CLIGHT;
        Triple xsat = crd_sat;
        Triple xsite = crd_crt_rec;

        double r = xsite.norm() + xsat.norm();
        
        Triple xsat2site = (xsite - xsat);
        double r_crt_rec2sat = xsat2site.norm();
        

        reldelay += 2.0 * GM_CGCS / CLIGHT / CLIGHT * log((r + r_crt_rec2sat) / (r - r_crt_rec2sat));
#if DEBUG_MAT
        double reldelay_newmat = 0.0;
        reldelay_newmat = 2.0 * (dot(crd_sat, vel_sat)) / CLIGHT;
        Vector xsat_newmat = crd_sat;
        Vector xsite_newmat = crd_crt_rec;

        double r_newmat = xsite_newmat.norm() + xsat_newmat.norm();
        Vector xsat2site_newmat = (xsite_newmat - xsat_newmat);
        double r_crt_rec2sat_newmat = xsat2site_newmat.norm();

        reldelay_newmat += 2.0 * GM_CGCS / CLIGHT / CLIGHT * log((r_newmat + r_crt_rec2sat_newmat) / (r_newmat - r_crt_rec2sat_newmat));
        printf("### MAT error relDelay = %.10f\n", reldelay_newmat - reldelay);
#endif
        return reldelay;
    }

    bool gnss_model_precise_bias::_update_obs_info(gnss_data_sats &obsdata)
    {
        // smgcz
        obsdata = _crt_obs;
        return true;
    }

    double gnss_model_precise_bias::ifbDelay(base_allpar &param, gnss_data_sats &satdata, std::string &sat, std::string &rec, gnss_data_obs &gobs)
    {
        // jdhuang : DCB is correct by apply DCB all
        double ifb = 0.0;
        int i = param.getParam(_crt_rec, par_type::IFB_GPS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[satdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << std::setprecision(3) << std::setw(9) << ifb_c3 << std::endl;
        }

        i = param.getParam(_crt_rec, par_type::IFB_GAL, "");
        if (i >= 0 && gobs.is_code() && _freq_index[satdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << std::setprecision(3) << std::setw(9) << ifb_c3 << std::endl;
        }
        i = param.getParam(_crt_rec, par_type::IFB_BDS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[satdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << std::setprecision(3) << std::setw(9) << ifb_c3 << std::endl;
        }
        i = param.getParam(_crt_rec, par_type::IFB_QZS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[satdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << std::setprecision(3) << std::setw(9) << ifb_c3 << std::endl;
        }
        return ifb;
    }

    double gnss_model_precise_bias::ifcbDelay(gnss_data_sats &satdata, gnss_data_ifcb *ifcb, OBSCOMBIN obscombin)
    {
        double L = 0.0;
        double ifcb_GPS = 0.0;

        //lvhb added in 202100404 for rtk/nrtk/urtk
        if (ifcb == nullptr && _gifcb == nullptr)
        {
            return false;
        }

        if (ifcb == nullptr)
        {
            ifcb = _gifcb;
        }

        base_time epoch = satdata.epoch();
        std::string sat = satdata.sat();

        GOBSBAND b1 = gnss_sys::band_priority(satdata.gsys(), FREQ_1);
        // GOBSBAND b2 = gnss_sys::band_priority(satdata.gsys(), FREQ_2);
        GOBSBAND b3 = gnss_sys::band_priority(satdata.gsys(), FREQ_3);

        double l1 = satdata.wavelength(b1);
        // double l2 = satdata.wavelength(b2);
        double l3 = satdata.wavelength(b3);

        double c2 = l1 / l3;
        double c1 = 1.0 / (1.0 - c2 * c2);
        c2 = c2 * c2 * c1;

        if (satdata.gsys() == GSYS::GPS)
        {
            // liugege
            hwa_map_id_ifcb epoch_ifcb = ifcb->get_epo_ifcb(epoch);
            if (epoch_ifcb.find(sat) == epoch_ifcb.end())
                return false;

            if (epoch_ifcb[sat]->sigma > 0.2 ||
                epoch_ifcb[sat]->npoint <= 2)
            {
                return false;
            }
            ifcb_GPS = epoch_ifcb[sat]->value;
            if (obscombin == OBSCOMBIN::IONO_FREE)
            {
                L += ifcb_GPS;
            }
            else if (obscombin == OBSCOMBIN::RAW_ALL)
            {
                L += ifcb_GPS / (-c2);
            }
        }

        return L;
    }

    double gnss_model_precise_bias::windUp(const GOBSBAND &freq_2, gnss_data_sats &satdata, const Triple &rRec)
    {
        // GSYS gsys = satdata.gsys();
        double wavelength = 0.0;
        wavelength = satdata.wavelength(freq_2);

        if (double_eq(wavelength, 0.0))
        {
            throw std::runtime_error("wavelength is zero when getting windup");
        }

        if (fabs(satdata.wind()) > 0)
        {
            return satdata.wind() * wavelength;
        }
        else
        {
            base_time epoch = satdata.epoch();
            std::string prn = satdata.sat();
            Triple rSat = _trs_sat_crd;

            // glfeng not change time
            // double Mjd = epoch.mjd() + epoch.sod() / 86400.0;

            // First time - initialize to zero
            // -------------------------------
            if (_phase_windup.find(_crt_rec) == _phase_windup.end() ||
                _phase_windup[_crt_rec].find(prn) == _phase_windup[_crt_rec].end() ||
                _phase_windup[_crt_rec][prn].size() == 0)
            {
                _phase_windup[_crt_rec][prn][epoch] = 0.0;
            }

            // Compute the correction for new time
            // -----------------------------------
            if (_phase_windup[_crt_rec][prn].find(epoch) == _phase_windup[_crt_rec][prn].end() ||
                _phase_windup[_crt_rec][prn].size() == 1)
            {

                // the last epoch
                double dphi0 = _phase_windup[_crt_rec][prn].rbegin()->second;
                Triple rho = rRec - rSat; //glfeng
                rho /= rho.norm();

                Triple i, j, k;

                // attitude model
                std::string antype = "";
                if (_gallobj != 0)
                {
                    std::shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(satdata.sat());
                    std::shared_ptr<gnss_data_pcv> sat_pcv;
                    if (sat_obj != 0)
                        sat_pcv = sat_obj->pcv(satdata.epoch());
                    if (sat_pcv != 0)
                        antype = sat_pcv->anten();
                }

                if (_attitudes == ATTITUDES::YAW_NOMI)
                    _gattitude_model.attitude(satdata, "", i, j, k); //nominal modeling
                else if (_attitudes == ATTITUDES::YAW_RTCM)
                    _gattitude_model.attitude(satdata, satdata.yaw(), i, j, k); //value from RTCM used
                else
                    _gattitude_model.attitude(satdata, antype, i, j, k); //default

                if (antype.find("BLOCK IIR") != std::string::npos)
                {
                    i = (-1.0) * i;
                    j = (-1.0) * j;
                }
                double rlength = rho.dot(i);
                Triple dipSat = i - rlength * rho - rho.cross(j);

                // Receiver unit Vectors rx, ry
                // ----------------------------
                Triple rx;
                Triple ry;

                double recEll[3];
                xyz2ell(rRec.data(), recEll, false);
                double neu[3];

                neu[0] = 1.0;
                neu[1] = 0.0;
                neu[2] = 0.0;
                neu2xyz(recEll, neu, rx.data());

                neu[0] = 0.0;
                neu[1] = -1.0;
                neu[2] = 0.0;
                neu2xyz(recEll, neu, ry.data());

                // Effective Dipole of the Receiver Antenna
                // ----------------------------------------
                rlength = rho.dot(rx);
                Triple dipRec = rx - rlength * rho + rho.cross(ry);

                // Resulting Effect
                // ----------------
                double alpha = dipSat.dot(dipRec) / (dipSat.norm() * dipRec.norm());

                if (alpha > 1.0)
                    alpha = 1.0;
                if (alpha < -1.0)
                    alpha = -1.0;

                double dphi = acos(alpha) / 2.0 / hwa_pi; // in cycles

                if (rho.dot(dipSat.cross(dipRec)) < 0.0)
                    dphi = -dphi;

                _phase_windup[_crt_rec][prn][epoch] = floor(dphi0 - dphi + 0.5) + dphi;
            }

#ifdef DEBUG_CORR
            std::cout << "Wind up " << epoch.str_ymdhms() << " " << prn << " " << _windUpSum[prn] << " " << satdata.orb_angle() * R2D << std::endl;
            //    int ooo; cin >> ooo;
#endif

            satdata.addwind(_phase_windup[_crt_rec][prn][epoch]);
            return _phase_windup[_crt_rec][prn][epoch] * wavelength;
        }
    }

    double gnss_model_precise_bias::PCV(bool corrt_sat, bool corrt_rec, bool realtime, base_time &epoch, base_time &crt_sat_epo, Triple &trs_rec_crd, gnss_data_sats &satdata, gnss_data_obs &gobs)
    {
        // Phase center variation correction
        double pcv_R = 0.0;
        double pcv_S = 0.0;
        double pco_R = 0.0;
        double pco_S = 0.0;

        GOBSBAND band = gobs.band();

        if (_gallobj == nullptr)
        {
            throw std::runtime_error("cannot correct PCV\n");
        }

        std::shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(satdata.sat());
        std::shared_ptr<gnss_data_obj> rec_obj = _gallobj->obj(satdata.site());

        std::shared_ptr<gnss_data_pcv> sat_pcv = (sat_obj != nullptr) ? sat_obj->pcv(epoch) : nullptr;
        std::shared_ptr<gnss_data_pcv> rec_pcv = (rec_obj != nullptr) ? rec_obj->pcv(epoch) : nullptr;

        if (!_isCalSatPCO)
        { //lvhb modified in 20200918
            if (sat_pcv != nullptr)
                sat_pcv = nullptr;
        }

        if (sat_pcv != nullptr && !realtime && corrt_sat) //add _realtime by xiongyun
        {
            // Satellite phase center variation
            // -- Satellite phase center offset
            Triple pco(0, 0, 0);
            if (sat_pcv->pcoS_raw(satdata, pco, band) > 0)
            {
                std::string antenna = sat_pcv->anten();
                Triple dx(0, 0, 0);
                Triple i, j, k;
                if (_attitudes == ATTITUDES::YAW_NOMI)
                    _gattitude_model.attitude(satdata, "", i, j, k); //nominal modeling
                else if (_attitudes == ATTITUDES::YAW_RTCM)
                    _gattitude_model.attitude(satdata, satdata.yaw(), i, j, k); //value from RTCM used
                else
                    _gattitude_model.attitude(satdata, antenna, i, j, k); //default
                dx[0] = pco[0] * i(0) + pco[1] * j(0) + pco[2] * k(0);
                dx[1] = pco[0] * i(1) + pco[1] * j(1) + pco[2] * k(1);
                dx[2] = pco[0] * i(2) + pco[1] * j(2) + pco[2] * k(2);
                sat_pcv->pco_proj(pco_S, satdata, trs_rec_crd, dx);
                satdata.addpco(dx);
            }
        }

        if (rec_pcv != nullptr && corrt_rec)
        {
            // Receiver phase center offset
            Triple pco(0.0, 0.0, 0.0);
            if (rec_pcv->pcoR_raw(satdata, pco, band) > 0)
            {
                Matrix _rot_matrix = _RotMatrix_Ant(satdata, _crt_rec_epo, crt_sat_epo, rec_obj, false);
                //Matrix _rot_matrix = _RotMatrix_Ant(satdata, epoch, crt_sat_epo, rec_obj, false);
                Triple dx(_rot_matrix * (pco));
                rec_pcv->pco_proj(pco_R, satdata, trs_rec_crd, dx);
            }
            pco_R *= -1;
        }

        if (gobs.is_phase())
        {
            if (sat_pcv != 0 && !realtime && corrt_sat) //add _realtime by xiongyun
            {
                // Satellite phase center variation
                sat_pcv->pcvS_raw(pcv_S, satdata, band, trs_rec_crd);
            }
            if (rec_pcv != 0 && corrt_rec)
            {
                // Receiver phase center variation
                rec_pcv->pcvR_raw(pcv_R, satdata, band);
            }
        }

        return pcv_R + pcv_S + pco_R + pco_S;
    }

    Matrix gnss_model_precise_bias::_RotMatrix_Ant(gnss_data_sats &obsdata, const base_time &receive_epoch, const base_time &transmit_epoch, std::shared_ptr<gnss_data_obj> obj, bool isCRS)
    {
        base_data::ID_TYPE type = obj->id_type();
        Matrix rotmatrix(3, 3);
        Triple ell(0.0, 0.0, 0.0);
        double sinPhi, cosPhi, sinLam, cosLam;

        base_time t; //zzwu

        if (type == base_data::TRN)
        {
            std::shared_ptr<gnss_data_pcv> sat_pcv = obj->pcv(receive_epoch);
            std::string antenna = (sat_pcv)->anten();
            Triple dx(0, 0, 0);
            Triple i, j, k;
            if (_attitudes == ATTITUDES::YAW_NOMI)
                _gattitude_model.attitude(obsdata, "", i, j, k); //nominal modeling
            else if (_attitudes == ATTITUDES::YAW_RTCM)
                _gattitude_model.attitude(obsdata, obsdata.yaw(), i, j, k); //value from RTCM used
            else
                _gattitude_model.attitude(obsdata, antenna, i, j, k); //default
            rotmatrix.col(0) = i;
            rotmatrix.col(1) = j;
            rotmatrix.col(2) = k;
            t = transmit_epoch;
        }
        else if (type == base_data::REC)
        {
            xyz2ell(_trs_rec_crd, ell, false);
            sinPhi = sin(ell[0]);
            cosPhi = cos(ell[0]);
            sinLam = sin(ell[1]);
            cosLam = cos(ell[1]);
            rotmatrix << -sinPhi * cosLam , -sinLam , +cosPhi * cosLam
                      , -sinPhi * sinLam , +cosLam, +cosPhi * sinLam
                      , +cosPhi , 0.0 , +sinPhi;
            t = receive_epoch;
        }
        else
        {
            throw std::exception();
        }

        if (isCRS)
        {
            // change trs2crs
            //_update_rot_matrix(transmit_epoch);
            _update_rot_matrix(t); //zzwu
            rotmatrix = _trs2crs_2000->getRotMat() * rotmatrix;
        }

        return rotmatrix;
    }

    bool gnss_model_precise_bias::_prt_obs_all(const base_time &epoch, gnss_data_sats &obsdata, base_allpar &pars, gnss_data_obs &gobs, std::vector<std::pair<int, double>> &coeff)
    {
        Triple groundEll;
        xyz2ell(_trs_rec_crd, groundEll, false);
        // int parNum = pars.parNumber();
        // int idx_pcv_sat = 0, idx_pcv_rec = 0;

        auto par_list = pars.getPartialIndex(obsdata.site(), obsdata.sat());
        //for (int ipar = 0; ipar < parNum; ipar++)
        for (int ipar : par_list)
        {
            base_par par = pars.getPar(ipar);

            //double coeff_value = _Partial(epoch, _crt_obs, gobs1, gobs2, pars.getPar(ipar));
            double coeff_value = _Partial(epoch, obsdata, gobs, par);
            if (coeff_value != 0.0)
            {
                //note: This is bad from 1 in coeff std::vector
                coeff.push_back(std::make_pair(ipar + 1, coeff_value));
            }
        }
        return true;
    }

    double gnss_model_precise_bias::_Partial(const base_time &epoch, gnss_data_sats &obsdata, const gnss_data_obs &gobs, base_par &par)
    {
        Triple ell(0.0, 0.0, 0.0);

        switch (par.parType)
        {
        case par_type::CRD_X:
            if (obsdata.site() == par.site)
            {
                return (par.value() - obsdata.satcrd()[0]) / obsdata.rho();
            }
            else
                return 0.0;
        case par_type::CRD_Y:
            if (obsdata.site() == par.site)
            {
                return (par.value() - obsdata.satcrd()[1]) / obsdata.rho();
            }
            else
                return 0.0;
        case par_type::CRD_Z:
            if (obsdata.site() == par.site)
            {
                return (par.value() - obsdata.satcrd()[2]) / obsdata.rho();
            }
            else
                return 0.0;
        case par_type::CLK:
        {
            if (obsdata.site() == par.site)
                return 1.0 - obsdata.drate();
            else
                return 0.0;
        }
        case par_type::CLK_G:
        {
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::GPS)
                return 1.0 - obsdata.drate();
            else
                return 0.0;
        }
        case par_type::CLK_R:
        {
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::GLO)
                return 1.0 - obsdata.drate();
            else
                return 0.0;
        }
        case par_type::CLK_C:
        {
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::BDS)
                return 1.0 - obsdata.drate();
            else
                return 0.0;
        }
        case par_type::CLK_E:
        {
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::GAL)
                return 1.0 - obsdata.drate();
            else
                return 0.0;
        }
        case par_type::CLK_J:
        {
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::QZS)
                return 1.0 - obsdata.drate();
            else
                return 0.0;
        }
        case par_type::IFCB_F3:
        {
            if (gnss_sys::band2freq(obsdata.gsys(), gobs.band()) == FREQ_3 && par.prn == obsdata.sat())
                return -1.0;
            else
                return 0.0;
        }
        case par_type::IFCB_F4:
            if (gnss_sys::band2freq(obsdata.gsys(), gobs.band()) == FREQ_4 && par.prn == obsdata.sat())
                return -1.0;
            else
                return 0.0;
        case par_type::IFCB_F5:
            if (gnss_sys::band2freq(obsdata.gsys(), gobs.band()) == FREQ_5 && par.prn == obsdata.sat())
                return -1.0;
            else
                return 0.0;
        case par_type::IFB_C3:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.prn == obsdata.sat() && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_C4:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.prn == obsdata.sat() && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_C5:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.prn == obsdata.sat() && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::SAT_IFB_C3:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.prn == obsdata.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::SAT_IFB_C4:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.prn == obsdata.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::SAT_IFB_C5:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.prn == obsdata.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GPS_REC_IFB_C3:
            if (obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::GPS_REC_IFB_C4:
            if (obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::GPS_REC_IFB_C5:
            if (obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;

        case par_type::GAL_REC_IFB_C3:
            if (obsdata.gsys() == GAL && _observ == OBSCOMBIN::RAW_ALL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::GAL_REC_IFB_C4:
            if (obsdata.gsys() == GAL && _observ == OBSCOMBIN::RAW_ALL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::GAL_REC_IFB_C5:
            if (obsdata.gsys() == GAL && _observ == OBSCOMBIN::RAW_ALL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.site == obsdata.site())
                return 1.0;
            else
                return 0.0;
        case par_type::CLK_SAT:
            if (obsdata.sat() == par.prn)
                return -1.0;
            else
                return 0.0;
        case par_type::SION:
        {
            if (obsdata.site() == par.site)
            {
                // jdhuang : form G01_F
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                double alfa = 0.0;
                if (gobs.is_phase() && par.prn == obsdata.sat())
                {
                    alfa = -(f1 * f1) / (fk * fk);
                }
                if (gobs.is_code() && par.prn == obsdata.sat())
                {
                    alfa = (f1 * f1) / (fk * fk);
                }
                return alfa;
            }
            else
                return 0.0;
        }
        case par_type::VION:
        {
            if (obsdata.site() == par.site)
            {
                // jdhuang : change from G01_F
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                double mf = 1.0 / sqrt(1.0 - pow(R_SPHERE / (R_SPHERE + 450000.0) * sin(hwa_pi / 2.0 - obsdata.ele()), 2));
                double alfa = 0.0;
                if (gobs.is_phase() && par.prn == obsdata.sat())
                {
                    alfa = -(f1 * f1) / (fk * fk);
                }
                if (gobs.is_code() && par.prn == obsdata.sat())
                {
                    alfa = (f1 * f1) / (fk * fk);
                }
                return alfa * mf;
            }
            else
                return 0.0;
        }
        case par_type::P1P2G_REC:
        {
            if (obsdata.site() == par.site)
            {
                double f1 = G01_F;
                double fk = obsdata.frequency(gobs.band());
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (G02_F * G02_F) / (G01_F * G01_F - G02_F * G02_F);
                FREQ_SEQ freq = gnss_sys::band2freq(obsdata.gsys(), gobs.band());
                if (obsdata.gsys() == GPS && gobs.is_code() && (freq == FREQ_1 || freq == FREQ_2))
                {
                    return -alfa * beta;
                }
                else
                    return 0.0;
            }
            else
                return 0.0;
        }
        case par_type::P1P2E_REC:
        {
            if (obsdata.site() == par.site)
            {
                double f1 = E01_F;
                double fk = obsdata.frequency(gobs.band());
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (E05_F * E05_F) / (E01_F * E01_F - E05_F * E05_F);
                FREQ_SEQ freq = gnss_sys::band2freq(obsdata.gsys(), gobs.band());
                if (obsdata.gsys() == GAL && gobs.is_code() && (freq == FREQ_1 || freq == FREQ_2))
                {
                    return -alfa * beta;
                }
                else
                    return 0.0;
            }
            else
                return 0.0;
        }
        case par_type::GLO_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == GLO /*&& gobs.is_code()*/)
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_ifcb:
            if (obsdata.site() == par.site && !gobs.is_phase() && obsdata.gsys() == GLO && par.prn == obsdata.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_IFPB:
            if (obsdata.site() == par.site && gobs.is_phase() && obsdata.gsys() == GLO && par.prn == obsdata.sat())
                return 1.0;
            else
                return 0.0;
        case par_type::GLO_IFB:
            if (obsdata.site() == par.site && obsdata.gsys() == GLO && par.prn == obsdata.sat() && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::GAL_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL /*&& gobs.is_code()*/)
                return 1.0;
            else
                return 0.0;
        case par_type::BDS_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS /*&& gobs.is_code()*/)
                return 1.0;
            else
                return 0.0;
        case par_type::QZS_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS /*&& gobs.is_code()*/)
                return 1.0;
            else
                return 0.0;
        case par_type::LEO_ISB: //xjhan
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && obsdata.sat().substr(0, 1) != "G" /*&& gobs.is_code()*/)
                return 1.0; //xjhan
            else
                return 0.0;
        case par_type::IFB_QZS:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_GPS:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_GAL:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_GAL_2:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_GAL_3:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_BDS:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_BDS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::IFB_BDS_3:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::RCB_GPS_1:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::RCB_GPS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                return (f1 * f1) / (fk * fk) - 1;
            }
            else
                return 0.0;
        case par_type::RCB_BDS_1:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::RCB_BDS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                return (f1 * f1) / (fk * fk) - 1;
            }
            else
                return 0.0;
        case par_type::RCB_GAL_1:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::RCB_GAL_2:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                return (f1 * f1) / (fk * fk) - 1;
            }
            else
                return 0.0;
        case par_type::RCB_QZS_1:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS && gobs.is_code())
                return 1.0;
            else
                return 0.0;
        case par_type::RCB_QZS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                return (f1 * f1) / (fk * fk) - 1;
            }
            else
                return 0.0;
        case par_type::TRP:
        case par_type::GRD_N:
        case par_type::GRD_E:
        case par_type::CLK13_G:
        case par_type::CLK13_E:
        case par_type::CLK13_C:
        default:
            return 0.0;
        }
        return 0.0;
    }

    void gnss_model_precise_bias::_update_rot_matrix(const base_time &epoch)
    {
        //lvhb modified in 20200730
        if (_gdata_erp->isEmpty())
            return;
        if (!_gdata_erp)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "have no poleut data,cant' compute trs2crs!!");
            throw std::exception();
        }

        base_time tdt = epoch;
        tdt.tsys(base_time::TT);
        base_time utc = epoch;
        utc.tsys(base_time::UTC);
        auto find_iter = _trs2crs_list.find(tdt);
        if (find_iter == _trs2crs_list.end())
        {
            _trs2crs_2000 = std::make_shared<gnss_base_trs2crs>(false, _gdata_erp);
            _trs2crs_2000->calcRotMat(tdt, true, true, true, false, false);
            _trs2crs_list.insert(std::make_pair(tdt, _trs2crs_2000));

            auto before_iter = _trs2crs_list.lower_bound(tdt - 300.0);
            if (before_iter != _trs2crs_list.begin())
            {
                _trs2crs_list.erase(_trs2crs_list.begin(), --before_iter);
            }
        }
        else
        {
            _trs2crs_2000 = find_iter->second;
        }

#ifdef DEBUG_TRS
        // jdhuang : check for the trs2crs_2000
        std::cout << "Mark = trs2crs  : "
             << std::setw(25) << tdt.mjd()
             << std::setw(25) << tdt.sod()
             << std::setw(25) << tdt.dsec() << std::endl
             << std::setw(25) << std::fixed << std::setprecision(15)
             << trs2crs_2000->rotmat() << std::endl
             << trs2crs_2000->rotdx() << std::endl
             << trs2crs_2000->rotdy() << std::endl
             << trs2crs_2000->rotdu() << std::endl;
#endif // DEBUG
        return;
    }

    bool gnss_model_precise_bias::_apply_rec_RTK(const base_time &crt_epo, const base_time &rec_epo, base_allpar &pars, bool calculate_tides)
    {
        Triple trs_rec_xyz(0.0, 0.0, 0.0);
        if (_crd_est == CONSTRPAR::FIX)
        {
            trs_rec_xyz = _crt_obj->crd(rec_epo);
        }
        else
        {
            int ix = pars.getParam(_crt_rec, par_type::CRD_X, "");
            int iy = pars.getParam(_crt_rec, par_type::CRD_Y, "");
            int iz = pars.getParam(_crt_rec, par_type::CRD_Z, "");

            if (ix >= 0 && iy >= 0 && iz >= 0)
            {
                trs_rec_xyz = Triple(pars.getParValue(ix), pars.getParValue(iy), pars.getParValue(iz));
                if (trs_rec_xyz.isZero())
                {
                    trs_rec_xyz = _crt_obj->crd(rec_epo);
                    pars.setParValue(ix, trs_rec_xyz[0]);
                    pars.setParValue(iy, trs_rec_xyz[1]);
                    pars.setParValue(iz, trs_rec_xyz[2]);
                }
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "can't get the crd of" + _crt_rec + " in time" + rec_epo.str_ymdhms());
                return false;
            }
        }

        if (trs_rec_xyz.isZero())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "can't get the crd of" + _crt_rec + " in time" + rec_epo.str_ymdhms());
            return false;
        }

        if(calculate_tides)
        { 
            //TODO COMMENT
            bool tide_valid = _apply_rec_tides(rec_epo, trs_rec_xyz);
            if (!tide_valid)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "apply tide failed for " + _crt_rec + " in time" + rec_epo.str_ymdhms());
                return false;
            }
        }

        //ARP Correction: modified by lvhb in 20200916
        // ARP Correction
        if (!_is_flt || _crd_est == CONSTRPAR::FIX)
            trs_rec_xyz = trs_rec_xyz + _crt_obj->eccxyz(rec_epo);
        _trs_rec_crd = trs_rec_xyz;

        // TRS2CRS
        _update_rot_matrix(rec_epo);
        Matrix trs2crs = _trs2crs_2000->getRotMat();
        Vector crs_rec_xyz = trs2crs * trs_rec_xyz;
        _crs_rec_crd = Triple(crs_rec_xyz);

        // jdhuang
        // get vel
        Matrix dtrs2crs = _trs2crs_2000->getMatDu() * OMGE_DOT;
        _crs_rec_vel = dtrs2crs * trs_rec_xyz;
        return true;
    }

    bool gnss_model_precise_bias::_apply_rec(const base_time &crt_epo, const base_time &rec_epo, base_allpar &pars)
    {
        Triple trs_rec_xyz(0.0, 0.0, 0.0);
        if (_crd_est == CONSTRPAR::FIX)
        {
            trs_rec_xyz = _crt_obj->crd(rec_epo);
        }
        else
        {
            int ix = pars.getParam(_crt_rec, par_type::CRD_X, "");
            int iy = pars.getParam(_crt_rec, par_type::CRD_Y, "");
            int iz = pars.getParam(_crt_rec, par_type::CRD_Z, "");

            if (ix >= 0 && iy >= 0 && iz >= 0)
            {
                trs_rec_xyz = Triple(pars.getParValue(ix), pars.getParValue(iy), pars.getParValue(iz));
                if (trs_rec_xyz.isZero())
                {
                    trs_rec_xyz = _crt_obj->crd(rec_epo);
                    pars.setParValue(ix, trs_rec_xyz[0]);
                    pars.setParValue(iy, trs_rec_xyz[1]);
                    pars.setParValue(iz, trs_rec_xyz[2]);
                }
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "can't get the crd of" + _crt_rec + " in time" + rec_epo.str_ymdhms());
                return false;
            }
        }

        if (trs_rec_xyz.isZero())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "can't get the crd of" + _crt_rec + " in time" + rec_epo.str_ymdhms());
            return false;
        }

        //TODO COMMENT
        bool tide_valid = _apply_rec_tides(rec_epo, trs_rec_xyz);
        if (!tide_valid)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "apply tide failed for " + _crt_rec + " in time" + rec_epo.str_ymdhms());
            return false;
        }

        //ARP Correction: modified by lvhb in 20200916
        // ARP Correction
        if (!_is_flt || _crd_est == CONSTRPAR::FIX)
            trs_rec_xyz = trs_rec_xyz + _crt_obj->eccxyz(rec_epo);
        _trs_rec_crd = trs_rec_xyz;

        // TRS2CRS
        _update_rot_matrix(rec_epo);
        Matrix trs2crs = _trs2crs_2000->getRotMat();
        Vector crs_rec_xyz = trs2crs * trs_rec_xyz;
        _crs_rec_crd = Triple(crs_rec_xyz);

        // jdhuang
        // get vel
        Matrix dtrs2crs = _trs2crs_2000->getMatDu() * OMGE_DOT;
        _crs_rec_vel = dtrs2crs * trs_rec_xyz;
        return true;
    }

    int gnss_model_precise_bias::_correction_orb(double *xyz, double *vxyz, double *xyz_corr, double *vxyz_corr)
    {
        Triple pos = fromarray(xyz), vel = fromarray(vxyz), rao_pos = fromarray(xyz_corr), rao_vel = fromarray(vxyz_corr), out;
        rao2xyz(pos, vel, rao_pos, out);
        for (size_t i = 0; i < 3; i++)
        {
            xyz[i] -= out(i);
        }
        rao2xyz(pos, vel, rao_vel, out);
        for (size_t i = 0; i < 3; i++)
        {
            vxyz[i] -= out(i);
        }
        return 1;
    }

    int gnss_model_precise_bias::_correction_clk(double &clk, double &dclk, double &clk_corr, double &dclk_corr)
    {
        clk += clk_corr / CLIGHT;
        dclk += dclk_corr / CLIGHT;
        return 1;
    }

    bool gnss_model_precise_bias::_apply_sat(const base_time &rec_epo, base_time &sat_epo, gnss_all_nav *nav)
    {
        // ITERATION
        // compute sat coord(CRS)  clk(estimated) (rewrite in orb model)
        double delay = 0.0;
        Matrix rot_matrix_sat, rot_matrix_rec;

        // tyx debug for realtime PPP-RTK
        int pv_iod = 0;
        int clk_iod = 0;

        if (_realtime && !_ultrasp3)
        {
            if (dynamic_cast<gnss_all_prec*>(_gall_nav)->get_ssr_iod(_crt_sat, rec_epo, pv_iod, clk_iod) < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + _crt_sat
                        + rec_epo.str_ymdhms(" coordinates and clock correction is not exist for ephemeris")
                        + "(PV_IOD: " + base_type_conv::int2str(pv_iod) + ", CLK_IOD: " + base_type_conv::int2str(clk_iod) + " )");
                return false;
            }
            if (pv_iod != clk_iod)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + _crt_sat
                        + rec_epo.str_ymdhms(" PC_IOD and CLK_IOD is not equal for ephemeris")
                        + "(PV_IOD: " + base_type_conv::int2str(pv_iod) + ", CLK_IOD: " + base_type_conv::int2str(clk_iod) + " )");
                return false;
            }
        }
        //std::cerr << _crs_rec_crd[0]<<"  "<< _crs_rec_crd[1]<< " " << _crs_rec_crd[2] << std::endl;
        while (true)
        {
            sat_epo = rec_epo - delay;
            if (_realtime && !_ultrasp3)//add by xiongyun for realtime
            {
                //bool sat_posvel_valid = _get_crs_sat_crdvel(sat_epo, _crt_sat, _crs_sat_crd, _crs_sat_vel);    // tyx debug for realtime PPP-RTK
                bool sat_posvel_valid = _get_crs_sat_crdvel(sat_epo, _crt_sat, pv_iod, _crs_sat_crd, _crs_sat_vel);
                if (!sat_posvel_valid)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "can not get sat pos or vel for " + _crt_sat);
                    return false;
                }
                _update_rot_matrix(rec_epo);
                _trs_sat_crd = Triple(_trs2crs_2000->getRotMat().transpose() * _crs_sat_crd);
            }
            else
            {
                // Get CRS
                bool sat_pos_valid = _get_crs_sat_crd(sat_epo, _crt_sat, nav, _crs_sat_crd);
                if (!sat_pos_valid)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "can not get sat pos for " + _crt_sat);
                    return false;
                }
                if (double_eq(_crs_sat_crd[0] * _crs_sat_crd[1] * _crs_sat_crd[2], 0.0) || abs(_crs_sat_crd[0]) >= 1E18)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "can not get sat pos for " + _crt_sat);
                    return false;
                }
                // SET TRS in epoch TR [include base_earth rotation]
                _update_rot_matrix(rec_epo);
                _trs_sat_crd = Triple(_trs2crs_2000->getRotMat().transpose() * _crs_sat_crd);

                bool sat_vel_valid = _get_crs_sat_vel(sat_epo, _crt_sat, nav, _crs_sat_vel);
                if (!sat_vel_valid)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "can not get sat vel for " + _crt_sat);
                    return false;
                }
            }
            // PCO corr sat
            double pco_R = 0.0, pco_S = 0.0;
            if (_gallobj != 0)
            {
                _update_rot_matrix(rec_epo);
                _crt_obs.addcrd(_trs_sat_crd);
                Vector x_base_earth = (_crs_sat_crd.transpose() * _trs2crs_2000->getMatDu() / RAD2TSEC).transpose();
                Triple vel = Triple(_trs2crs_2000->getRotMat().transpose() * _crs_sat_vel - x_base_earth);
                _crt_obs.addvel(vel);
                std::shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(_crt_sat);
                std::shared_ptr<gnss_data_obj> rec_obj = _gallobj->obj(_crt_rec);

                std::shared_ptr<gnss_data_pcv> sat_pcv;
                std::shared_ptr<gnss_data_pcv> rec_pcv;

                if (sat_obj != 0)
                    sat_pcv = sat_obj->pcv(_crt_epo);
                if (rec_obj != 0)
                    rec_pcv = rec_obj->pcv(_crt_epo);

                GOBS_LC lc = LC_L1;
                if (sat_pcv && (!_realtime || _ultrasp3)) //add _realtime by xiongyun(do not correct pco in realtime when use bnccor)
                {
                    // Satellite phase center offset
                    Triple pco(0, 0, 0);

                    if (sat_pcv->pcoS(_crt_obs, pco, lc, _band_index[_crt_sys][FREQ_1], _band_index[_crt_sys][FREQ_2]) > 0)
                    {
                        rot_matrix_sat = _RotMatrix_Ant(_crt_obs, _crt_epo, _crt_sat_epo, sat_obj, false);
                        Triple dx(rot_matrix_sat * (pco));
                        sat_pcv->pco_proj(pco_S, _crt_obs, _trs_rec_crd, dx);
                    }
                }

                if (rec_pcv)
                {
                    // Receiver phase center offset
                    Triple pco(0.0, 0.0, 0.0);
                    if (rec_pcv->pcoR(_crt_obs, pco, lc, _band_index[_crt_sys][FREQ_1], _band_index[_crt_sys][FREQ_2]) > 0)
                    {
                        //rot_matrix_rec = _RotMatrix_Ant(_crt_obs, rec_epo, rec_obj, false);
                        rot_matrix_rec = _RotMatrix_Ant(_crt_obs, _crt_epo, _crt_sat_epo, rec_obj, false);
                        rot_matrix_rec *(pco);
                        Triple dx = rot_matrix_rec * pco;
                        rec_pcv->pco_proj(pco_R, _crt_obs, _trs_rec_crd, dx);
                    }
                    pco_R *= -1;
                }
            }

            double delay_temp;
            delay_temp = (_crs_sat_crd - _crs_rec_crd).norm();
            delay_temp += pco_R + pco_S;

            // jdhuang
            if (abs(delay_temp / CLIGHT - delay) < 1E-9)
                break;
            delay = delay_temp / CLIGHT;
        }
        if((!_realtime || _ultrasp3))
            _crt_obs.addSCF2CRS(_trs2crs_2000->getRotMat() * rot_matrix_sat, rot_matrix_sat);
        // add sat crd vel
        // Notice this sat crd should be in the TRS
        _crt_obs.addcrd(_trs_sat_crd);
        // modefied by zhshen
        Vector x_base_earth = (_crs_sat_crd.transpose() * _trs2crs_2000->getMatDu() / RAD2TSEC).transpose();
        Triple vel = _trs2crs_2000->getRotMat().transpose() * _crs_sat_vel - x_base_earth;
        _crt_obs.addvel(vel);
        // hyChang add: store _crs_sat_crd & _crs_sat_vel in _crt_obs
        _crt_obs.addcrdcrs(_crs_sat_crd);
        _crt_obs.addvel_crs(_crs_sat_vel);

        return true;
    }

    bool gnss_model_precise_bias::_get_crs_sat_vel(const base_time &sat_epoch, const std::string &sat, gnss_all_nav *nav, Triple &crs_sat_vel)
    {
        if (!nav)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "cannot get vel " + sat);
            return false;
        }

        // get vel in CRS
        Triple satcrd_next(0.0, 0.0, 0.0);
        Triple satcrd_before(0.0, 0.0, 0.0);
        
        bool crd_valid = _get_crs_sat_crd(sat_epoch + 0.0001, sat, nav, satcrd_next);
        //bool crd_valid2 = _get_crs_sat_crd(sat_epoch - 0.00001, sat, nav, satcrd_before);
        if (!crd_valid)
            return false;
        crs_sat_vel = (satcrd_next - _crs_sat_crd) * 10000;
       // crs_sat_vel = (satcrd_next - satcrd_before) / 0.00002;
        return true;
    }

    bool gnss_model_precise_bias::_get_crs_sat_crdvel(const base_time& sat_epoch, const std::string& sat, const int& iod, Triple& crs_sat_crd, Triple& crs_sat_vel)
    {
        if (!_gall_nav)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "there are no nav");
            return false;
        }

        bool pos_valid = false;
        //bool cor_valid = false;
        int cor_valid = -1;  // glfeng
        double xyz_corr[3] = { 0.0, 0.0, 0.0 };
        double vel[3] = { 0.0, 0.0, 0.0 };
        double vel_corr[3] = { 0.0, 0.0, 0.0 };
        double var[3] = { 0.0, 0.0, 0.0 };
        double clk = 0.0, dclk = 0.0;
        double clk_corr = 0.0;
        double dclk_corr = 0.0;

        double xyz_sat[3] = { 0.0, 0.0, 0.0 };

        //if (_ddmode) pos_valid = (_gall_nav->pos(sat, sat_epoch, xyz_sat, var, vel, true) >= 0) ? true : false; //by hjj
        cor_valid = (dynamic_cast<gnss_all_prec*>(_gall_nav)->get_pos_clk_correction(sat, sat_epoch, iod, xyz_corr, vel_corr, clk_corr, dclk_corr) >= 0) ? true : false;
        pos_valid = (_gall_nav->pos(sat, iod, sat_epoch, xyz_sat, var, vel, true) >= 0) ? true : false;

        std::ostringstream os;
        os << "************************* new  sat  prn " << sat << "*************************" << std::endl;
        os << "gsatdata " << sat
            << " CRD " << std::fixed << std::setprecision(3)
            << "  " << sat_epoch.str_ymdhms()
            << "  " << sat_epoch.sow() + sat_epoch.dsec()
            << " PV_IOD(CLK_IOD) " << std::setw(5) << iod
            << " X before correction " << std::setw(14) << xyz_sat[0]
            << " Y before correction " << std::setw(14) << xyz_sat[1]
            << " Z before correction " << std::setw(14) << xyz_sat[2] << std::endl;

        _correction_orb(xyz_sat, vel, xyz_corr, vel_corr);

        os << " X correction " << std::setw(14) << xyz_corr[0]
            << " Y correction " << std::setw(14) << xyz_corr[1]
            << " Z correction " << std::setw(14) << xyz_corr[2]
            << " sat clk correction " << std::setw(14) << clk_corr * CLIGHT
            << " X after correction " << std::setw(14) << xyz_sat[0]
            << " Y after correction " << std::setw(14) << xyz_sat[1]
            << " Z after correction " << std::setw(14) << xyz_sat[2] << std::endl;

        
        // TRS2CRS
        _update_rot_matrix(sat_epoch);
        crs_sat_crd = fromarray(xyz_sat);
        crs_sat_crd = _trs2crs_2000->getRotMat() * crs_sat_crd;
        crs_sat_vel = fromarray(vel);
        crs_sat_vel = _trs2crs_2000->getRotMat() * crs_sat_vel;

        if (!pos_valid)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, sat_epoch.str_ymdhms("can not get sat crd or vel in epoch", false));
        }
        if (!cor_valid)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, sat_epoch.str_ymdhms("can not get sat crd corr or vel corr in epoch", false));
        }

        return pos_valid && cor_valid;
    }

    bool gnss_model_precise_bias::_get_crs_sat_crd(const base_time &sat_epoch, const std::string &sat, gnss_all_nav *nav, Triple &crs_sat_crd)
    {
        bool pos_valid = false;
        if (nav)
        {
            double xyz_sat[3];
            pos_valid = (nav->pos(sat, sat_epoch, xyz_sat) >= 0) ? true : false;

            // TRS2CRS
            _update_rot_matrix(sat_epoch);
            crs_sat_crd = fromarray(xyz_sat);
            crs_sat_crd = _trs2crs_2000->getRotMat() * crs_sat_crd;
        }

        if (pos_valid)
            return pos_valid;
        else if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, sat_epoch.str_ymdhms("can not get sat crd in epoch", false));
        return false;
    }

    bool gnss_model_precise_bias::_apply_rec_tides(const base_time &epoch, Triple &rec)
    {
        if (!_tide)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "The tide ptr is nullptr.");
            return false;
        }

        _update_rot_matrix(epoch);
        double xpole = _trs2crs_2000->getXpole();
        double ypole = _trs2crs_2000->getYpole();
        // double gast = _trs2crs_2000->getGmst();
        Matrix rot_trs2crs = _trs2crs_2000->getRotMat();

        Triple tide(0.0, 0.0, 0.0);
        try
        {
            // solid tide
            gnss_model_tideiers *solid_ptr = dynamic_cast<gnss_model_tideiers *>(_tide.get());
            if (!solid_ptr)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "can not get the solid tide ptr.");
                return false;
            }
            Triple solid_base_earth = solid_ptr->tide_solid(epoch, rec, rot_trs2crs, _gdata_navde);

            // ocean load
            Triple load_ocean = _tide->load_ocean(epoch, _crt_rec, rec);

            //pole tide
            gnss_model_tideiers *pole_ptr = dynamic_cast<gnss_model_tideiers *>(_tide.get());
            if (!pole_ptr)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "can not get the pole tide ptr.");
                return false;
            }
            pole_ptr->set_mean_pole_model(_mean_pole_model);
            Triple tide_pole = pole_ptr->tide_pole_pod(epoch, xpole, ypole, rec);

            //atmospheric pressure loading; added by yqyuan 2021/11/30
            Triple load_atmosph{ 0, 0, 0 };
            if (_atm_loading)
            {
                gnss_model_tideiers *atmospheric_loading_ptr = dynamic_cast<gnss_model_tideiers *>(_tide.get()); // please note that in _tide. no pointer for atmloading is stored.
                atmospheric_loading_ptr->set_atm_grid(_atm_loading);
                load_atmosph = atmospheric_loading_ptr->atmospheric_loading(epoch, rec);
            }

            Triple load_ocean_pole(0.0, 0.0, 0.0);
            if (_opl)
            {
                gnss_model_tideiers *opl_ptr = dynamic_cast<gnss_model_tideiers *>(_tide.get()); // please note that in _tide. no pointer for atmloading is stored.
                opl_ptr->set_opl_grid(_opl);
                opl_ptr->set_mean_pole_model(_mean_pole_model);
                load_ocean_pole = opl_ptr->load_oceanpole(epoch, rec, xpole, ypole);
            }

            // jdhuang  for test
            tide = solid_base_earth +
                   load_ocean +
                   tide_pole +
                   load_atmosph +
                   load_ocean_pole;

#ifdef DEBUG_PRECISEMODEL
            std::cout << "Tide" + _crt_rec
                 << std::fixed << std::setprecision(15) << std::setw(20)
                 << " " << epoch.mjd()
                 << " " << epoch.sod()
                 << " " << tide[0]
                 << " " << tide[1]
                 << " " << tide[2] << std::endl;

            std::cout << "solid_base_earth" + _crt_rec
                 << std::fixed << std::setprecision(15) << std::setw(20)
                 << " " << epoch.mjd()
                 << " " << epoch.sod()
                 << " " << solid_base_earth[0]
                 << " " << solid_base_earth[1]
                 << " " << solid_base_earth[2] << std::endl;

            std::cout << "load_ocean" + _crt_rec
                 << std::fixed << std::setprecision(15) << std::setw(20)
                 << " " << epoch.mjd()
                 << " " << epoch.sod()
                 << " " << load_ocean[0]
                 << " " << load_ocean[1]
                 << " " << load_ocean[2] << std::endl;

            std::cout << "tide_pole" + _crt_rec
                 << std::fixed << std::setprecision(15) << std::setw(20)
                 << " " << epoch.mjd()
                 << " " << epoch.sod()
                 << " " << tide_pole[0]
                 << " " << tide_pole[1]
                 << " " << tide_pole[2] << std::endl;

            std::cout << "load_atmosph" + _crt_rec
                 << std::fixed << std::setprecision(15) << std::setw(20)
                 << " " << epoch.mjd()
                 << " " << epoch.sod()
                 << " " << load_atmosph[0]
                 << " " << load_atmosph[1]
                 << " " << load_atmosph[2] << std::endl;

            std::cout << "tide_freq" + _crt_rec
                 << std::fixed << std::setprecision(15) << std::setw(20)
                 << " " << epoch.mjd()
                 << " " << epoch.sod()
                 << " " << tide_freq[0]
                 << " " << tide_freq[1]
                 << " " << tide_freq[2] << std::endl;
#endif // DEBUG
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "can not get the tide ptr.");
            return false;
        }

        //unit to m
        rec = rec + tide * 1.e3;
        return true;
    }

}
