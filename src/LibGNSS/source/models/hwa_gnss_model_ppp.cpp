#include "hwa_gnss_model_ppp.h"

using namespace std;

namespace hwa_gnss
{
    gnss_model_ppp::gnss_model_ppp(std::string site, base_log spdlog, set_base *settings)
        : gnss_model_spp(spdlog, site, settings),
          _gallobj(0)
    {

        /* _site = site; */
        /* _settings = settings; */
        /* _log = glog; */

        if (_trpModStr == TROPMODEL::EXTERN)
            _tropoModel = std::make_shared<gnss_model_tropo>();

        //   if(      _trpModStr.compare("met") == 0 ){ _tropoModel = std::make_shared<gnss_model_tropo>(site); } // first option for site
        //   else if( _trpModStr.compare("nwm") == 0 ){ _tropoModel = std::make_shared<gnss_model_tropo>();     } // second option for site

        _phase = dynamic_cast<set_gproc *>(_settings)->phase();
        _grad_mf = dynamic_cast<set_gproc *>(_settings)->grad_mf();
        _trop_corr = dynamic_cast<set_gproc *>(_settings)->trop_corr();
        _attitudes = dynamic_cast<set_gproc *>(_settings)->attitudes();
    }

    gnss_model_ppp::gnss_model_ppp()
    {
    }

    // Destructor
    // --------------------------
    gnss_model_ppp::~gnss_model_ppp()
    {
    }

    // wind-up correction
    // ----------
    double gnss_model_ppp::windUp(gnss_data_sats &satdata, const Vector &rRec)
    {

        base_time epoch = satdata.epoch();
        std::string prn = satdata.sat();
        Vector rSat = satdata.satcrd();
        //GSYS sys          = satdata.gsys();

        double Mjd = epoch.mjd(true) + epoch.sod(true) / 86400.0;

        // First time - initialize to zero
        // -------------------------------
        std::map<std::string, double>::iterator it = _windUpTime.find(prn);
        if (it == _windUpTime.end())
        {
            _windUpTime[prn] = Mjd;
            _windUpSum[prn] = 0.0;

            //std::cout << "dphi:" << setw(20) << 0.0 << std::endl
            // << "dhpi0" << setw(20) << _windUpSum[prn] << std::endl;
        }

        // Compute the correction for new time
        // -----------------------------------
        else if (_windUpTime[prn] != Mjd)
        {

            _windUpTime[prn] = Mjd;

            Triple rho = (rRec - rSat).segment(0, 3);
            rho /= rho.norm();

            Vector i(3);
            Vector j(3);
            Vector k(3);

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
                attitude(satdata, "", i, j, k); //nominal modeling
            else if (_attitudes == ATTITUDES::YAW_RTCM)
                attitude(satdata, satdata.yaw(), i, j, k); //value from RTCM used
            else
                attitude(satdata, antype, i, j, k); //default
                                                    //

            // Effective Dipole of the GPS Satellite Antenna
            // ---------------------------------------------
            Triple dipSat = i - rho * rho.dot(i) - rho.cross(Triple(j));

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
            Triple dipRec = rx - rho * rho.dot(rx) + rho.cross(ry);

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

            /*       std::cout << "dphi:" << setw(20) << dphi << std::endl
           << "dhpi0" << setw(20) << _windUpSum[prn] << std::endl;
 */
            _windUpSum[prn] = floor(_windUpSum[prn] - dphi + 0.5) + dphi;
        }

#ifdef DEBUG
        std::cout << "Wind up " << epoch.str_ymdhms() << " " << prn << " " << _windUpSum[prn] << " " << satdata.orb_angle() * R2D << std::endl;
//    int ooo; cin >> ooo;
#endif

        satdata.addwind(_windUpSum[prn]);
        return _windUpSum[prn];
    }

    // set gallobj pointer
    // --------------------
    void gnss_model_ppp::setOBJ(gnss_all_obj *obj)
    {
        _gallobj = obj;
        if (obj)
        {
            _grec = obj->obj(_site);
        }
    }

    // model computed range value (phase/code)
    // ----------
    double gnss_model_ppp::cmpObs(base_time &epoch, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs, bool com)
    {

        double spp_model = gnss_model_spp::cmpObs(epoch, param, gsatdata, gobs);
        if (spp_model < 0)
            return -1;
        GSYS gs = gsatdata.gsys();
        GOBSBAND b1 = gnss_sys::band_priority(gs, FREQ_1);
        GOBSBAND b2 = gnss_sys::band_priority(gs, FREQ_2);
        // modified by glfeng consider tide correction, base_earth rotation
        // -------------------------------------------
        // Cartesian coordinates to ellipsodial coordinates
        Triple xyz_r, xyz_s, ell;
        Vector cRec(3);
        xyz_s = gsatdata.satcrd();
        if (param.getCrdParam(_site, xyz_r) < 0)
        {
            xyz_r = gsatdata.reccrd();
            if (double_eq(xyz_r[0], 0.0) || double_eq(xyz_r[1], 0.0) || double_eq(xyz_r[2], 0.0))
            {
                xyz_r = _grec->crd_arp(epoch);
            }
        }
        cRec = xyz_r;
        xyz2ell(xyz_r, ell, false);

        // Earth rotation correction
        Triple Rec_pco;
        double rho0 = sqrt(pow(xyz_r[0] - xyz_s[0], 2) + pow(xyz_r[1] - xyz_s[1], 2) + pow(xyz_r[2] - xyz_s[2], 2));
        double dPhi = OMEGA * rho0 / CLIGHT;
        Rec_pco[0] = xyz_r[0] * cos(dPhi) - xyz_r[1] * sin(dPhi);
        Rec_pco[1] = xyz_r[1] * cos(dPhi) + xyz_r[0] * sin(dPhi);
        Rec_pco[2] = xyz_r[2];

        // -------------------------------------------

        // Wind up correction
        double wind = 0.0;
        if (gobs.is_phase())
        {
            double wavelength = 0.0;
            if (_observ == OBSCOMBIN::IONO_FREE)
            {

                wavelength = gsatdata.wavelength_L3(b1, b2);
            }
            else
            {
                wavelength = gsatdata.wavelength(gobs.band());
            }
            _attitudes = ATTITUDES::DEF_YAWMODEL; // add by glfeng
            if (fabs(gsatdata.wind()) > 0)
                wind = gsatdata.wind() * wavelength;
            else
                wind = windUp(gsatdata, cRec) * wavelength;
        }
        //_attitudes = dynamic_cast<set_gproc*>(_settings)->attitudes(); // change to YAW_NOMI glfeng
        _attitudes = ATTITUDES::YAW_NOMI;

        // Phase center variation correction
        double pcv_R = 0.0;
        double pcv_S = 0.0;
        double pco_R = 0.0;
        double pco_S = 0.0;
        if (_gallobj != 0)
        {
            std::shared_ptr<gnss_data_obj> sat_obj = _gallobj->obj(gsatdata.sat());
            std::shared_ptr<gnss_data_obj> site_obj = _gallobj->obj(_site);

            std::shared_ptr<gnss_data_pcv> sat_pcv;
            std::shared_ptr<gnss_data_pcv> site_pcv;

            if (sat_obj != 0)
                sat_pcv = sat_obj->pcv(epoch);
            if (site_obj != 0)
                site_pcv = site_obj->pcv(epoch);

            GOBS_LC lc = LC_IF;
            if (_observ != OBSCOMBIN::IONO_FREE)
            {
                if (gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_1)
                    lc = LC_L1;
                if (gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_2)
                    lc = LC_L2;
                if (gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_3)
                    lc = LC_L3;
                if (gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_4)
                    lc = LC_L4;
                if (gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_5)
                    lc = LC_L5;
            }

            if (sat_pcv != 0 && com)
            {
                // Satellite phase center variation
                if (gobs.is_phase())
                    sat_pcv->pcvS(pcv_S, gsatdata, xyz_r, lc, b1, b2); // range don't consider PCV glfeng
                // Satellite phase center offset
                Triple pco(0, 0, 0);
                if (sat_pcv->pcoS(gsatdata, pco, lc, b1, b2) > 0)
                {
                    std::string antenna = sat_pcv->anten();
                    Triple dx(0, 0, 0);
                    Vector i(3);
                    Vector j(3);
                    Vector k(3);
                    if (_attitudes == ATTITUDES::YAW_NOMI)
                        attitude(gsatdata, "", i, j, k); //nominal modeling
                    else if (_attitudes == ATTITUDES::YAW_RTCM)
                        attitude(gsatdata, gsatdata.yaw(), i, j, k); //value from RTCM used
                    else
                        attitude(gsatdata, antenna, i, j, k); //default
                    dx[0] = pco[0] * i(0) + pco[1] * j(0) + pco[2] * k(0);
                    dx[1] = pco[0] * i(1) + pco[1] * j(1) + pco[2] * k(1);
                    dx[2] = pco[0] * i(2) + pco[1] * j(2) + pco[2] * k(2);
                    sat_pcv->pco_proj(pco_S, gsatdata, Rec_pco, dx); // change xyz-> Rec_pco consider base_earth rotation glfeng
                    gsatdata.addpco(dx);
                }
            }

            if (site_pcv != 0)
            {
                // Receiver phase center variation
                if (gobs.is_phase())
                    site_pcv->pcvR(pcv_R, gsatdata, lc, b1, b2); // range don't consider PCV glfeng
                // Receiver phase center offset
                Triple dx(0.0, 0.0, 0.0);
                if (site_pcv->pcoR(gsatdata, dx, xyz_r, lc) > 0)
                {
                    site_pcv->pco_proj(pco_R, gsatdata, Rec_pco, dx); // change xyz-> Rec_pco consider base_earth rotation glfeng
                }
                pco_R *= -1;
            }
        }

#ifdef DEBUG
        std::cout << setw(20) << " EPOCH is : " << epoch.str_mjdsod() << " " << setw(6) << gsatdata.sat() << setw(6) << gsatdata.site()
             << setw(20) << " Band is : " << setw(6) << gobs.band()
             << setw(20) << " spp_model  " << std::fixed << left << setw(20) << setprecision(6) << spp_model
             << setw(20) << " wind       " << std::fixed << left << setw(20) << setprecision(6) << wind
             << setw(20) << " pco_R      " << std::fixed << left << setw(20) << setprecision(6) << pco_R
             << setw(20) << " pco_S      " << std::fixed << left << setw(20) << setprecision(6) << pco_S
             << setw(20) << " pcv_R      " << std::fixed << left << setw(20) << setprecision(6) << pcv_R
             << setw(20) << " pcv_S      " << std::fixed << left << setw(20) << setprecision(6) << pcv_S
             << std::endl;

        std::cout << gsatdata.epoch().str_hms() << " "
             << gsatdata.sat() << " " << gobs2str(gobs.gobs())
             << std::fixed << setprecision(3)
             << "  SPP_model " << setw(15) << spp_model
             << "  wind: " << setw(6) << wind
             << "  Rec PCO: " << setw(6) << pco_R
             << "  Rec PCV: " << setw(6) << pcv_R
             << "  Sat PCV: " << setw(6) << pcv_S
             << "  Sat PCO: " << setw(6) << pco_S
             << std::endl;
        //         int ooo; cin >> ooo;

        std::ostringstream os;
        os << gsatdata.sat()
           << " CRD " << std::fixed << setprecision(3)
           << "  " << epoch.str_ymdhms()
           << " X " << setw(14) << satcrd[0] + gsatdata.satpco()[0]
           << " Y " << setw(14) << satcrd[1] + gsatdata.satpco()[1]
           << " Z " << setw(14) << satcrd[2] + gsatdata.satpco()[2]
           << " T " << setw(14) << gsatdata.clk();

        if (_observ == IONO_FREE)
        {
            std::cout << std::fixed << setprecision(5)
                 << "delay" << setw(20) << spp_model << std::endl
                 << "LC pco rec" << setw(20) << pco_R << std::endl
                 << "LC pco sat" << setw(20) << pco_S << std::endl
                 << "pcv_rec" << setw(20) << pcv_R << std::endl
                 << "pcv_sat" << setw(20) << pcv_S << std::endl
                 << "dphwp" << setw(20) << wind << std::endl;
        }
#endif

        // Return value
        return spp_model +
               wind +
               pco_R +
               pco_S +
               pcv_R +
               pcv_S;
    }

    // Compute troposperic delay
    // -----------
    double gnss_model_ppp::tropoDelay(base_time &epoch, base_allpar &param, Triple ell, gnss_data_sats &satdata)
    {

        if (_tropoModel == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "Tropo Model setting is not correct. Default used! Check config.");
            _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
        }

        double ele = satdata.ele();
        double azi = satdata.azi();

        double delay = 0.0;
        double zwd = 0.0;
        double zhd = 0.0;
        int i, j, k;
        i = param.getParam(_site, par_type::TRP, "");
        j = param.getParam(_site, par_type::GRD_N, "");
        k = param.getParam(_site, par_type::GRD_E, "");

        if (i >= 0)
        {
            zwd = param[i].value();
            zhd = param[i].apriori();
            //zhd = param[i].zhd;
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
        if (_tropo_mf == ZTDMPFUNC::GMF)
        {
            gnss_model_gmf mf;
            mf.gmf(epoch.mjd(), ell[0], ell[1], ell[2], hwa_pi / 2.0 - ele,
                   mfh, mfw, dmfh, dmfw);
        }
        else if (_tropo_mf == ZTDMPFUNC::COSZ)
        {
            mfh = mfw = 1 / sin(ele);
            dmfh = dmfw = -(cos(ele)) / (sin(ele) * sin(ele));
        }

        satdata.addmfH(mfh);
        satdata.addmfW(mfw);

        delay = mfh * zhd + mfw * zwd;

        // debug by ZHJ
        //if (_observ == IONO_FREE) {
        // std::cout << std::fixed << setprecision(5)
        //  << "ele" << satdata.ele_deg() << std::endl
        //  << "dry delay " << zhd << std::endl
        //  << "dry std::map" << mfh << std::endl
        //  << "wet delay" << zwd << std::endl
        //  << "wet std::map" << mfw << std::endl;
        //}

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

#ifdef DEBUG
        std::cout << epoch.str("EPOCH: %H:%M:%S") << std::endl
             << std::fixed << setprecision(3);
        std::cout << satdata.sat() << " Ell:" << ell[0] << " " << ell[1] << " " << ell[2]
             << " Hydrostatic part: " << zhd //_tropoModel->getZHD(ell, epoch)
             << " Wet part: " << zwd
             << " mfh: " << mfh
             << " mfw: " << mfw
             << " Delay: " << delay << std::endl
             << std::endl;
        //      int ooo; cin >> ooo;
#endif

        return delay;
    }

    // satellite attitude model
    // --------------------------------
    int gnss_model_ppp::attitude_old(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        if (satdata.gsys() == GAL)
        {
            _ysm(satdata, i, j, k);
        }
        else
        {
            int irc = _yaw(satdata, antype, i, j, k);
            if (irc == 0)
                return 0;
        }

        return 1;
    }

    // satellite attitude model
    // --------------------------------
    int gnss_model_ppp::attitude(gnss_data_sats &satdata, double yaw, Vector &i, Vector &j, Vector &k)
    {
        _yaw2ijk(satdata, yaw, i, j, k);
        return 1;
    }

    // satellite attitude model
    // --------------------------------
    int gnss_model_ppp::attitude(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        Vector satcrd = satdata.satcrd();
        Vector satvel = satdata.satvel();

        if (satcrd.norm() == 0 || satvel.norm() == 0)
        {
            return -1;
        }

        if (satdata.gsys() == GPS)
        {
            if (antype == "BLOCK II")
            {
                _attitude_GPSIIA(satdata, antype, i, j, k);
            }
            else if (antype == "BLOCK IIA")
            {
                _attitude_GPSIIA(satdata, antype, i, j, k);
            }
            else if (antype.find("BLOCK IIR") != std::string::npos)
            {
                _attitude_GPSIIR(satdata, antype, i, j, k);
            }
            else if (antype == "BLOCK IIF")
            {
                _attitude_GPSIIF(satdata, antype, i, j, k);
            }
            else if (antype == "BLOCK III")
            { // added by yqyuan for GPSIII (BLOCK III/IIIA/IIIF???)
                _attitude_GPSIII(satdata, antype, i, j, k);
            }
            else
                _ysm(satdata, i, j, k);
        }
        else if (satdata.gsys() == GLO)
        {
            _attitude_GLO(satdata, antype, i, j, k);
        }
        else if (satdata.gsys() == GAL)
        {
            if (antype == "GALILEO-1")
            {
                _attitude_GAL1(satdata, antype, i, j, k);
            }
            else if (antype == "GALILEO-2")
            {
                _attitude_GAL2(satdata, antype, i, j, k);
            }
            else
                _ysm(satdata, i, j, k);
        }
        else if (satdata.gsys() == BDS)
        {
            _attitude_BDS(satdata, antype, i, j, k);
        }
        else if (satdata.gsys() == QZS)
        {
            _attitude_QZS(satdata, antype, i, j, k);
        }

        if (i.norm() == 0 || j.norm() == 0 || k.norm() == 0)
            return 0;

#ifdef DEBUG
        double angl = satdata.orb_angle();
        if (angl > hwa_pi / 2)
            angl -= 2.0 * hwa_pi;
        std::cout << satdata.sat() << " " << satdata.ele_deg() << " " << angl * R2D << " " << satdata.yaw() * R2D << " " << satdata.beta() * R2D << std::endl;
#endif

        return 1;
    }

    // satellite attitude model ( with input Xsat, Vsat, Xsun) used in OI
    // --------------------------------
    // added by yqyuan
    int gnss_model_ppp::attitude(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        Vector satcrd = xsat;
        Vector satvel = vsat;

        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (satcrd.norm() == 0 || satvel.norm() == 0)
        {
            return -1;
        }

        if (prn.find("G") != std::string::npos)
        {
            if (antype == "BLOCK II")
            {
                _attitude_GPSIIA(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else if (antype == "BLOCK IIA")
            {
                _attitude_GPSIIA(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else if (antype.find("BLOCK IIR") != std::string::npos)
            {
                _attitude_GPSIIR(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else if (antype == "BLOCK IIF")
            {
                _attitude_GPSIIF(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else if (antype == "BLOCK III")
            { // added by yqyuan for GPSIII (BLOCK III/IIIA/IIIF???)
                _attitude_GPSIII(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else
                _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        else if (prn.find("R") != std::string::npos)
        {
            _attitude_GLO(antype, prn, xsat, vsat, xsun, i, j, k);
        }
        else if (prn.find("E") != std::string::npos)
        {
            if (antype == "GALILEO-1")
            {
                _attitude_GAL1(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else if (antype == "GALILEO-2")
            {
                _attitude_GAL2(antype, prn, xsat, vsat, xsun, i, j, k);
            }
            else
                _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        else if (prn.find("C") != std::string::npos)
        {
            _attitude_BDS(antype, prn, xsat, vsat, xsun, i, j, k);
        }
        else if (prn.find("J") != std::string::npos)
        {
            _attitude_QZS(antype, prn, xsat, vsat, xsun, i, j, k);
        }

        if (i.norm() == 0 || j.norm() == 0 || k.norm() == 0)
            return 0;
        return 1;
    }

    double gnss_model_ppp::_orb_angle(Vector &xsat, Vector &vsat, Vector &xsun)
    {
        double mi = 0.0;
        Triple Sun = xsun.segment(0, 3); //ICRF
        Triple Satcrd = xsat.segment(0, 3);
        Triple Satvel = vsat.segment(0, 3);

        Triple n = Satcrd.cross(Satvel);

        Triple es = Satcrd / Satcrd.norm();

        Triple eSun = Sun / Sun.norm();

        Triple p = Sun.cross(n);
        p /= p.norm();

        double E = acos(es.dot(p));
        /* mi = hwa_pi / 2.0 + (dot(es, eSun) <= 0 ? -E : E); */
        /* if (mi < -hwa_pi / 2.0) mi += 2.0 * hwa_pi; */
        /* else if (mi >= hwa_pi / 2.0) mi -= 2.0*hwa_pi; */

        double SunSat = acos(es.dot(eSun));

        if (SunSat > hwa_pi / 2)
        {
            if (E <= hwa_pi / 2)
                mi = hwa_pi / 2 - E;
            else
                mi = hwa_pi / 2 - E;
        }
        else
        {
            if (E <= hwa_pi / 2)
                mi = hwa_pi / 2 + E;
            else
                mi = E - hwa_pi - hwa_pi / 2;
        }

        return mi;
    }

    double gnss_model_ppp::_beta(Vector &xsat, Vector &vsat, Vector &xsun)
    {
        double beta = 0.0;

        Triple Sun = xsun.segment(0, 3);

        Triple Satcrd = xsat.segment(0, 3);
        Triple Satvel = vsat.segment(0, 3);

        Triple n = Satcrd.cross(Satvel);

        n /= n.norm();

        Triple nSun = Sun / Sun.norm();

        double cosa = nSun.dot(n);
        //   if(cosa < 0) cosa *= -1;

        beta = hwa_pi / 2.0 - acos(cosa);

        return beta;
    }

    // Yaw-steering mode attitude model
    // --------------------------------
    void gnss_model_ppp::_ysm(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        double MJD = satdata.epoch().dmjd();

        Triple satcrd_t = satdata.satcrd();
        Vector satcrd = satcrd_t;

        // Satelite-Earth unit std::vector
        k = -satcrd;

        // Along solar panel unit std::vector
        Vector sun = _ephplan.sunPos(MJD);
        j = Triple(k).cross(Triple(sun));

        // complete to satelite std::fixed right-hand coord system
        i = Triple(j).cross(Triple(k));

        i = i / i.norm();
        j = j / j.norm();
        k = k / k.norm();

        _last_beta[satdata.sat()] = satdata.beta();

        double yaw = atan2(-satdata.beta(), sin(satdata.orb_angle()));
        satdata.yaw(yaw);
    }

    void gnss_model_ppp::_ysm(std::string prn, double bata, double mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        Vector satcrd = xsat;

        // Satelite-Earth unit std::vector
        k = -satcrd;

        // Along solar panel unit std::vector
        Vector sun = xsun;
        j = Triple(k).cross(Triple(sun));
        // there is a little difference for Y-axis: Xsun vs Xsun-Xsat.
        // The resulting error can be ignored. yqyuan
        //Vector sat2sun = _ephplan.sunPos(MJD) - satcrd;
        //j = cross(k, sat2sun);

        // complete to satelite std::fixed right-hand coord system
        i = Triple(j).cross(Triple(k));

        i = i / i.norm();
        j = j / j.norm();
        k = k / k.norm();

        _last_beta[prn] = bata;

        // double yaw = atan2(-bata, sin(mi));
    }

    // orbit normal mode (i toward the velocity)
    // --------------------------------
    void gnss_model_ppp::_onm(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        double yaw = 0;
        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_onm(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double yaw = 0;
        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // yaw model of different satellite
    // Nominal attitude (same as GPS BLOCK II/IIA) without yaw maneuver for MEO and IGSO satellites ();
    // Yaw - std::fixed attitude mode used for GEO satellites
    // --------------------------------
    int gnss_model_ppp::_yaw(gnss_data_sats &satdata, std::string antype, Vector &xs, Vector &ys, Vector &zs)
    {
#if 0
   double exs[3], eys[3], ezs[3], r[3];
   int i;

   int opt = 2; //2 :precise yaw
   
   bool yawFix = gnss_sys::bds_geo(satdata.sat()) || (satdata.beta()*R2D <= 4 && satdata.beta()*R2D >= -4);
   if (satdata.gsys() == GSYS::BDS && yawFix){  //Only for BDS, opt=3
      opt = 3;
   }

   double rs[6];
   for (int i = 0; i < 3; i++)  rs[i] = satdata.satcrd()[i];
   for (int i = 3; i < 6; i++)  rs[i] = satdata.satvel()[i - 3];

   std::string prn = satdata.sat();
   base_time epoch = satdata.epoch();

   for (i = 0; i < 3; i++) r[i] = -rs[i];
   if (!normv3(r, ezs)) return 0;
   /* satellite yaw attitude model */
   double  ep[6];
   ep[0] = epoch.year(); ep[1] = epoch.mon(); ep[2] = epoch.day();
   ep[3] = epoch.hour(); ep[4] = epoch.mins(); ep[5] = epoch.secs();
   gtime_t time = epoch2time(ep);

   if (!normv3(r, ezs)) return 0;
   if (!sat_yaw(time, prn.c_str(), antype.c_str(), opt, rs, exs, eys)) return 0;
   for (int i = 0; i < 3; i++)
   {
      xs(i + 1) = exs[i];
      ys(i + 1) = eys[i];
      zs(i + 1) = ezs[i];
   }
#endif
        return 1;
    }

    // Attitude modelling for GPS Block IIA
    void gnss_model_ppp::_attitude_GPSIIA(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        const double R_GPSIIA[] = {
            0.1046, 0.1230, 0.1255, 0.1249, 0.1003, 0.1230, 0.1136, 0.1169, 0.1253, 0.0999,
            0.1230, 0.1230, 0.1230, 0.1230, 0.1092, 0.1230, 0.1230, 0.1230, 0.1230, 0.1230,
            0.1230, 0.1230, 0.1230, 0.0960, 0.0838, 0.1284, 0.1183, 0.1230, 0.1024, 0.1042,
            0.1230, 0.1100, 0.1230};
        int sat = base_type_conv::str2int(satdata.sat().substr(1, 2));
        double R = R_GPSIIA[sat - 1] * D2R;

        double beta0 = atan2(MUDOT_GPS, R);

        if (fabs(satdata.orb_angle()) > hwa_pi / 2 && fabs(satdata.beta()) < beta0)
        {
            // noon maneuver
            _noon_turn(satdata, R, i, j, k);
        }
        else if (fabs(satdata.orb_angle()) < hwa_pi / 2 && fabs(satdata.beta()) < beta0)
        {
            // midnight maneuver
            _midnight_turn_GPSIIA(satdata, R, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_attitude_GPSIIA(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        const double R_GPSIIA[] = {
            0.1046, 0.1230, 0.1255, 0.1249, 0.1003, 0.1230, 0.1136, 0.1169, 0.1253, 0.0999,
            0.1230, 0.1230, 0.1230, 0.1230, 0.1092, 0.1230, 0.1230, 0.1230, 0.1230, 0.1230,
            0.1230, 0.1230, 0.1230, 0.0960, 0.0838, 0.1284, 0.1183, 0.1230, 0.1024, 0.1042,
            0.1230, 0.1100, 0.1230};
        int sat = base_type_conv::str2int(prn.substr(1, 2));
        double R = R_GPSIIA[sat - 1] * D2R;

        double beta0 = atan2(MUDOT_GPS, R);
        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (fabs(orb_angle) > hwa_pi / 2 && fabs(beta) < beta0)
        {
            // noon maneuver
            _noon_turn(prn, beta, orb_angle, xsat, vsat, xsun, R, i, j, k);
        }
        else if (fabs(orb_angle) < hwa_pi / 2 && fabs(beta) < beta0)
        {
            // midnight maneuver
            _midnight_turn_GPSIIA(prn, beta, orb_angle, xsat, vsat, xsun, R, i, j, k);
        }
        else
        {
            _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
    }

    // Attitude modelling for GPS Block IIR
    void gnss_model_ppp::_attitude_GPSIIR(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        const double R = 0.2 * D2R; // maximal yaw hardware rate
        double beta0 = atan2(MUDOT_GPS, R);

        if (fabs(satdata.orb_angle()) > hwa_pi / 2 && fabs(satdata.beta()) < beta0)
        {
            // noon maneuver
            _noon_turn(satdata, R, i, j, k);
        }
        else if (fabs(satdata.orb_angle()) < hwa_pi / 2 && fabs(satdata.beta()) < beta0)
        {
            // midnight maneuver
            _midnight_turn(satdata, R, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }

        i = -1 * i; // X away from the Sum
        j = -1 * j;
        satdata.yaw(satdata.yaw() + hwa_pi);
    }

    void gnss_model_ppp::_attitude_GPSIIR(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        const double R = 0.2 * D2R; // maximal yaw hardware rate
        double beta0 = atan2(MUDOT_GPS, R);
        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (fabs(orb_angle) > hwa_pi / 2 && fabs(beta) < beta0)
        {
            // noon maneuver
            _noon_turn(prn, beta, orb_angle, xsat, vsat, xsun, R, i, j, k);
        }
        else if (fabs(orb_angle) < hwa_pi / 2 && fabs(beta) < beta0)
        {
            // midnight maneuver
            _midnight_turn_GPSIIA(prn, beta, orb_angle, xsat, vsat, xsun, R, i, j, k);
        }
        else
        {
            _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }

        i = -1 * i; // X away from the Sum
        j = -1 * j;
    }

    // noon maneuver
    // -------------------------------------------
    void gnss_model_ppp::_noon_turn(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = atan2(MUDOT_GPS, R);
        double beta = satdata.beta();
        double mi = satdata.orb_angle();

        // test of beta sign change
        auto itSAT = _last_beta.find(satdata.sat());
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta >= 0)
            R *= -1;

        double mi_s = hwa_pi - sqrt(beta0 * fabs(beta) - beta * beta);
        double yaw;
        double yaw_nom = atan2(-tan(beta), sin(mi));

        if (mi >= mi_s || mi <= 0)
        {
            if (mi < 0)
                mi += 2.0 * hwa_pi;
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GPS;
            if ((beta >= 0 && yaw < yaw_nom) || (beta < 0 && yaw > yaw_nom))
                yaw = yaw_nom;
        }
        else
            yaw = yaw_nom;

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    // midnight maneuver
    // -------------------------------------------
    void gnss_model_ppp::_midnight_turn(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = atan2(MUDOT_GPS, R);
        double beta = satdata.beta();
        double mi = satdata.orb_angle();

        // test of beta sign change
        auto itSAT = _last_beta.find(satdata.sat());
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta < 0)
            R *= -1;

        double mi_s = -sqrt(beta0 * fabs(beta) - beta * beta);
        double yaw;
        double yaw_nom = atan2(-tan(beta), sin(mi));

        if (mi >= mi_s)
        {
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GPS;
            if ((beta >= 0 && yaw > yaw_nom) || (beta < 0 && yaw < yaw_nom))
                yaw = yaw_nom;
        }
        else
            yaw = yaw_nom;

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_noon_turn(std::string _prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = atan2(MUDOT_GPS, R);
        double beta = _beta;
        double mi = _mi;

        // test of beta sign change
        auto itSAT = _last_beta.find(_prn);
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta >= 0)
            R *= -1;

        double mi_s = hwa_pi - sqrt(beta0 * fabs(beta) - beta * beta);
        double yaw;
        double yaw_nom = atan2(-tan(beta), sin(mi));

        if (mi >= mi_s || mi <= 0)
        {
            if (mi < 0)
                mi += 2.0 * hwa_pi;
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GPS;
            if ((beta >= 0 && yaw < yaw_nom) || (beta < 0 && yaw > yaw_nom))
                yaw = yaw_nom;
        }
        else
            yaw = yaw_nom;

        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    void gnss_model_ppp::_midnight_turn(Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k)
    {
    }

    // midnight maneuver
    // -------------------------------------------
    void gnss_model_ppp::_midnight_turn_GPSIIA(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = satdata.beta();
        double mi = satdata.orb_angle();
        double mi_s = -sqrt(EPS0_GPS * EPS0_GPS - beta * beta);
        double mi_e = -mi_s;

        // test of beta sign change
        auto itSAT = _last_beta.find(satdata.sat());
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta < 0)
            R *= -1;

        double yaw;

        if (mi_s <= mi && mi < mi_e)
        {
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GPS;
        }
        else if (mi_e <= mi && mi < mi_e + POST_SHADOW * MUDOT_GPS)
            satdata.setecl(true);
        else
            yaw = atan2(-tan(beta), sin(mi));

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_midnight_turn_GPSIIA(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = _beta;
        double mi = _mi;
        double mi_s = -sqrt(EPS0_GPS * EPS0_GPS - beta * beta);
        double mi_e = -mi_s;

        // test of beta sign change
        auto itSAT = _last_beta.find(prn);
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta < 0)
            R *= -1;

        double yaw;

        if (mi_s <= mi && mi < mi_e)
        {
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GPS;
        }
        else if (mi_e <= mi && mi < mi_e + POST_SHADOW * MUDOT_GPS)
            ; //satdata.setecl(true);
        else
            yaw = atan2(-tan(beta), sin(mi));

        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // midnight maneuver for GPS Block IIF
    // -------------------------------------------
    void gnss_model_ppp::_midnight_turn_GPSIIF(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = satdata.beta();
        double tan_beta = tan(beta);
        double mi = satdata.orb_angle();

        // test of beta sign change
        auto itSAT = _last_beta.find(satdata.sat());
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta < 0)
            R *= -1;

        double mi_s = -acos(cos(EPS0_GPS) / cos(beta));
        double mi_e = -mi_s;
        double sin_mi_s = sin(mi_s);
        double mi_f = MUDOT_GPS * (atan2(-tan_beta, -sin_mi_s) - atan2(-tan_beta, sin_mi_s)) / R + mi_s;

        double yaw;

        if (mi_s <= mi && mi < mi_f)
        {
            yaw = atan2(-tan_beta, sin_mi_s) + R * (mi - mi_s) / MUDOT_GPS;
            _yaw2ijk(satdata, yaw, i, j, k);
        }
        else if (mi_f <= mi && mi < mi_e)
        {
            yaw = atan2(-tan_beta, -sin_mi_s);
            _yaw2ijk(satdata, yaw, i, j, k);
        }
        else
            _ysm(satdata, i, j, k);
    }

    void gnss_model_ppp::_midnight_turn_GPSIIF(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = _beta;
        double tan_beta = tan(beta);
        double mi = _mi;

        // test of beta sign change
        auto itSAT = _last_beta.find(prn);
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                beta *= -1;
        }

        if (beta < 0)
            R *= -1;

        double mi_s = -acos(cos(EPS0_GPS) / cos(beta));
        double mi_e = -mi_s;
        double sin_mi_s = sin(mi_s);
        double mi_f = MUDOT_GPS * (atan2(-tan_beta, -sin_mi_s) - atan2(-tan_beta, sin_mi_s)) / R + mi_s;

        double yaw;

        if (mi_s <= mi && mi < mi_f)
        {
            yaw = atan2(-tan_beta, sin_mi_s) + R * (mi - mi_s) / MUDOT_GPS;
            _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
        }
        else if (mi_f <= mi && mi < mi_e)
        {
            yaw = atan2(-tan_beta, -sin_mi_s);
            _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
        }
        else
            _ysm(prn, beta, mi, xsat, vsat, xsun, i, j, k);
    }

    // midnight maneuver for GLONASS-M
    // -------------------------------------------
    void gnss_model_ppp::_midnight_turn_GLOM(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = satdata.beta();
        double tan_beta = tan(beta);
        double mi = satdata.orb_angle();

        if (beta < 0)
            R *= -1;

        double mi_s = -acos(cos(EPS0_GLO) / cos(beta));
        double mi_e = -mi_s;
        double sin_mi_s = sin(mi_s);
        double mi_f = MUDOT_GLO * (atan2(-tan_beta, -sin_mi_s) - atan2(-tan_beta, sin_mi_s)) / R + mi_s;

        // debug by ZHJ initial
        double yaw = 0.0;

        if (mi_s <= mi && mi < mi_f)
        {
            yaw = atan2(-tan_beta, sin_mi_s) + R * (mi - mi_s) / MUDOT_GLO;
        }
        else if (mi_f <= mi && mi < mi_e)
        {
            yaw = atan2(-tan_beta, -sin_mi_s);
        }

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_midnight_turn_GLOM(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = _beta;
        double tan_beta = tan(beta);
        double mi = _mi;

        if (beta < 0)
            R *= -1;

        double mi_s = -acos(cos(EPS0_GLO) / cos(beta));
        double mi_e = -mi_s;
        double sin_mi_s = sin(mi_s);
        double mi_f = MUDOT_GLO * (atan2(-tan_beta, -sin_mi_s) - atan2(-tan_beta, sin_mi_s)) / R + mi_s;

        // debug by ZHJ initial
        double yaw = 0.0;

        if (mi_s <= mi && mi < mi_f)
        {
            yaw = atan2(-tan_beta, sin_mi_s) + R * (mi - mi_s) / MUDOT_GLO;
        }
        else if (mi_f <= mi && mi < mi_e)
        {
            yaw = atan2(-tan_beta, -sin_mi_s);
        }

        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // noon maneuver
    // -------------------------------------------
    void gnss_model_ppp::_noon_turn_GLOM(gnss_data_sats &satdata, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = satdata.beta();
        double mi = satdata.orb_angle();

        if (beta >= 0)
            R = -R;

        double mi_s, mi_e, sin_mi_s, B, yaw;
        int c = 0;

        for (mi_s = 178.6 * D2R; c < 4; c++)
        {
            sin_mi_s = sin(mi_s);
            B = -beta * cos(mi_s) / (beta * beta + sin_mi_s * sin_mi_s);
            mi_s = (atan(-beta / sin_mi_s) + mi_s * B + hwa_pi * R / MUDOT_GLO - hwa_pi / 2.0) / (R / MUDOT_GLO + B);
        }
        if (beta >= 0)
            mi_s = 2.0 * hwa_pi - mi_s;
        mi_e = 2.0 * hwa_pi - mi_s;

        if (mi_s <= mi && mi < mi_e)
        {
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GLO;
        }

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_noon_turn_GLOM(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, double R, Vector &i, Vector &j, Vector &k)
    {
        double beta = _beta;
        double mi = _mi;

        if (beta >= 0)
            R = -R;

        double mi_s, mi_e, sin_mi_s, B, yaw;
        int c = 0;

        for (mi_s = 178.6 * D2R; c < 4; c++)
        {
            sin_mi_s = sin(mi_s);
            B = -beta * cos(mi_s) / (beta * beta + sin_mi_s * sin_mi_s);
            mi_s = (atan(-beta / sin_mi_s) + mi_s * B + hwa_pi * R / MUDOT_GLO - hwa_pi / 2.0) / (R / MUDOT_GLO + B);
        }
        if (beta >= 0)
            mi_s = 2.0 * hwa_pi - mi_s;
        mi_e = 2.0 * hwa_pi - mi_s;

        if (mi_s <= mi && mi < mi_e)
        {
            yaw = atan2(-tan(beta), sin(mi_s)) + R * (mi - mi_s) / MUDOT_GLO;
        }

        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // Attitude modelling for GPS Block IIR-M
    void gnss_model_ppp::_attitude_GPSIIRM(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        _attitude_GPSIIR(satdata, antype, i, j, k);
    }

    // Attitude modelling for GPS Block IIF
    void gnss_model_ppp::_attitude_GPSIIF(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        const double R_noon = 0.11 * D2R; // maximal yaw hardware rate during noon turn
        const double R_midn = 0.06 * D2R; // maximal yaw hardware rate during midnight turn
        double beta0 = atan2(MUDOT_GPS, R_noon);

        if (fabs(satdata.orb_angle()) > hwa_pi / 2 && fabs(satdata.beta()) < beta0)
        {
            // noon maneuver
            _noon_turn(satdata, R_noon, i, j, k);
        }
        else if (fabs(satdata.orb_angle()) < EPS0_GPS && fabs(satdata.beta()) < EPS0_GPS)
        {
            // midnight maneuver
            _midnight_turn_GPSIIF(satdata, R_midn, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_attitude_GPSIIF(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        const double R_noon = 0.11 * D2R; // maximal yaw hardware rate during noon turn
        const double R_midn = 0.06 * D2R; // maximal yaw hardware rate during midnight turn
        double beta0 = atan2(MUDOT_GPS, R_noon);

        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (fabs(orb_angle) > hwa_pi / 2 && fabs(beta) < beta0)
        {
            // noon maneuver
            _noon_turn(prn, beta, orb_angle, xsat, vsat, xsun, R_noon, i, j, k);
        }
        else if (fabs(orb_angle) < EPS0_GPS && fabs(beta) < EPS0_GPS)
        {
            // midnight maneuver
            _midnight_turn_GPSIIF(prn, beta, orb_angle, xsat, vsat, xsun, R_midn, i, j, k);
        }
        else
        {
            _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
    }

    // Attitude modelling for GPS III, added by yqyuan but not realized.
    void gnss_model_ppp::_attitude_GPSIII(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        // to be added;
        _ysm(satdata, i, j, k);
    }

    void gnss_model_ppp::_attitude_GPSIII(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
    }

    // Attitude modelling for GLONASS
    void gnss_model_ppp::_attitude_GLO(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        const double R = 0.25 * D2R; // maximal yaw hardware rate
        double beta0 = 2.0 * D2R;
        double mi = satdata.orb_angle();
        double mi_s = 176.8 * D2R;

        if (mi > mi_s && mi < 2.0 * hwa_pi - mi_s && fabs(satdata.beta()) < beta0)
        {
            // noon maneuver
            _noon_turn_GLOM(satdata, R, i, j, k);
        }
        else if (fabs(satdata.orb_angle()) < EPS0_GLO && fabs(satdata.beta()) < EPS0_GLO)
        {
            // midnight maneuver
            _midnight_turn_GLOM(satdata, R, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_attitude_GLO(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        const double R = 0.25 * D2R; // maximal yaw hardware rate
        double beta0 = 2.0 * D2R;

        double beta = _beta(xsat, vsat, xsun);
        double mi = _orb_angle(xsat, vsat, xsun);
        double mi_s = 176.8 * D2R;

        if (mi > mi_s && mi < 2.0 * hwa_pi - mi_s && fabs(beta) < beta0)
        {
            // noon maneuver
            _noon_turn_GLOM(prn, beta, mi, xsat, vsat, xsun, R, i, j, k);
        }
        else if (fabs(mi) < EPS0_GLO && fabs(beta) < EPS0_GLO)
        {
            // midnight maneuver
            _midnight_turn_GLOM(prn, beta, mi, xsat, vsat, xsun, R, i, j, k);
        }
        else
        {
            _ysm(prn, beta, mi, xsat, vsat, xsun, i, j, k);
        }
    }

    // noon maneuver for Galileo IOV
    // -------------------------------------------
    void gnss_model_ppp::_noon_turn_GAL1(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        double beta = satdata.beta();
        double mi = satdata.orb_angle();

        double beta_y = 2 * D2R;
        double beta_x = 15 * D2R;

        double Sx = sin(mi) * cos(beta);
        double Sy = -sin(beta);

        double sinby = sin(beta_y);
        double sinbx = sin(beta_x);
        if (Sy < 0)
            sinby *= -1;

        double Shy = (sinby + Sy) / 2 + cos(hwa_pi * fabs(Sx) / sinbx) * (sinby - Sy) / 2;

        double yaw = atan2(Shy, Sx);

        // test of beta sign change
        auto itSAT = _last_beta.find(satdata.sat());
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                yaw *= -1;
        }

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_noon_turn_GAL1(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double beta = _beta;
        double mi = _mi;

        double beta_y = 2 * D2R;
        double beta_x = 15 * D2R;

        double Sx = sin(mi) * cos(beta);
        double Sy = -sin(beta);

        double sinby = sin(beta_y);
        double sinbx = sin(beta_x);
        if (Sy < 0)
            sinby *= -1;

        double Shy = (sinby + Sy) / 2 + cos(hwa_pi * fabs(Sx) / sinbx) * (sinby - Sy) / 2;

        double yaw = atan2(Shy, Sx);

        // test of beta sign change
        auto itSAT = _last_beta.find(prn);
        if (itSAT != _last_beta.end())
        {
            if (itSAT->second * beta < 0)
                yaw *= -1;
        }

        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // Attitude modelling for Galileo IOV
    void gnss_model_ppp::_attitude_GAL1(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = 2.0 * D2R;
        double mi_n0 = (180 - 15) * D2R;
        double mi_m0 = 15 * D2R;

        if (fabs(satdata.orb_angle()) > mi_n0 && fabs(satdata.beta()) < beta0)
        {
            // noon maneuver
            _noon_turn_GAL1(satdata, i, j, k);
        }
        else if (fabs(satdata.orb_angle()) < mi_m0 && fabs(satdata.beta()) < beta0)
        {
            // midnight maneuver
            _noon_turn_GAL1(satdata, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }

        //   i = -1 * i;      // X away from the Sum
        //   j = -1 * j;
        //   satdata.yaw(satdata.yaw() + hwa_pi);
    }

    void gnss_model_ppp::_attitude_GAL1(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = 2.0 * D2R;
        double mi_n0 = (180 - 15) * D2R; // noon
        double mi_m0 = 15 * D2R;         // midnight

        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (fabs(orb_angle) > mi_n0 && fabs(beta) < beta0)
        {
            // noon maneuver
            _noon_turn_GAL1(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k); // in IGS frame
        }
        else if (fabs(orb_angle) < mi_m0 && fabs(beta) < beta0)
        {
            // midnight maneuver
            _noon_turn_GAL1(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        else
        {
            _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k); // in IGS frame
        }
    }

    // noon maneuver for Galileo FOC
    // -------------------------------------------
    void gnss_model_ppp::_noon_turn_GAL2(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        double beta = satdata.beta();
        double mi = satdata.orb_angle();

        // test of beta sign change
        auto it = _last_beta.find(satdata.sat());
        if (it != _last_beta.end())
        {
            if (it->second * beta < 0)
                beta *= -1;
        }

        double init = hwa_pi / 2;
        if (beta > 0)
            init *= -1;

        double yaw_s;
        base_time epo_s;
        base_time epo = satdata.epoch();

        auto itYAW = _last_yaw.find(satdata.sat());
        if (itYAW != _last_yaw.end())
        {
            yaw_s = itYAW->second;
        }
        else
        {
            yaw_s = atan2(-tan(beta), sin(mi));
            _last_yaw[satdata.sat()] = yaw_s;
        }

        auto itEPO = _last_epo.find(satdata.sat());
        if (itEPO != _last_epo.end())
        {
            epo_s = itEPO->second;
        }
        else
        {
            epo_s = epo;
            _last_epo[satdata.sat()] = epo_s;
        }

        double yaw = init + (yaw_s - init) * cos((2.0 * hwa_pi / 5656) * (epo - epo_s));

        _yaw2ijk(satdata, yaw, i, j, k);
    }

    void gnss_model_ppp::_noon_turn_GAL2(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        // the origional edition needs time info; updated with our model: Li et al. 2019
        double beta = _beta;
        double u = _mi;
        double yawangle = atan2(-tan(beta), sin(u)); // in IGS frame

        double ub_candidate_foc_1 = -9.1 * D2R;
        double ub_candidate_foc_2 = 170.9 * D2R;

        double yaw = 0.0;

        Vector satvel = vsat;
        double satvel_norm = satvel.norm();
        Vector satcrd = xsat;
        double satcrd_norm = satcrd.norm();
        double murate = (satvel_norm / satcrd_norm); // in radius

        double ub = 0.0;
        double du = 0.0;
        if (u >= ub_candidate_foc_1 && u <= ub_candidate_foc_2)
        {
            ub = ub_candidate_foc_1;
            du = u - ub_candidate_foc_1;
        }
        else
        {
            ub = ub_candidate_foc_2;
            if (u > 0)
            {
                du = u - ub_candidate_foc_2;
            }
            else
            {
                du = 2 * hwa_pi + u - ub_candidate_foc_2;
            }
        }

        if (fabs(beta) < 4.1 * D2R && fabs(du) < 18.2 * D2R)
        {
            double tmod = fabs(du) / murate;
            double beta0 = beta;
            double u0 = ub;
            double yangle0 = atan2(-tan(beta0), sin(u0));
            yaw = 0.5 * hwa_pi * sign(1, yangle0) + (yangle0 - 0.5 * hwa_pi * sign(1, yangle0)) * cos(fabs(2 * hwa_pi / 5656 * tmod));
        }
        else
        {
            yaw = yawangle;
        }
        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // Attitude modelling for Galileo FOC
    void gnss_model_ppp::_attitude_GAL2(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = 4.1 * D2R;
        double mi_n0 = (180 - 10) * D2R;
        double mi_m0 = 10 * D2R;

        if (fabs(satdata.orb_angle()) > mi_n0 && fabs(satdata.beta()) < beta0)
        {
            // noon maneuver
            _noon_turn_GAL2(satdata, i, j, k);
        }
        else if (fabs(satdata.orb_angle()) < mi_m0 && fabs(satdata.beta()) < beta0)
        {
            // midnight maneuver
            _noon_turn_GAL2(satdata, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
            _last_yaw[satdata.sat()] = satdata.yaw();
            _last_epo[satdata.sat()] = satdata.epoch();
        }

        //   i = -1 * i;      // X away from the Sum
        //   j = -1 * j;
        //   satdata.yaw(satdata.yaw() + hwa_pi);
    }

    void gnss_model_ppp::_attitude_GAL2(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double beta0 = 4.1 * D2R;
        double mi_n0 = (180 - 10) * D2R;
        double mi_m0 = 10 * D2R;

        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (fabs(orb_angle) > mi_n0 && fabs(beta) < beta0)
        {
            // noon maneuver
            _noon_turn_GAL2(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        else if (fabs(orb_angle) < mi_m0 && fabs(beta) < beta0)
        {
            // midnight maneuver
            _noon_turn_GAL2(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        else
        {
            _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
            _last_yaw[prn] = atan2(-beta, sin(orb_angle));
            //_last_epo[prn] = epoch;
        }
    }

    // Continuous yaw steering attitude modes of BDS CAST satellites, added by yqyuan
    void gnss_model_ppp::_cys_cast(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        double beta_threshold_cast = 2.8;
        double d_constant = 80000.0;
        if (fabs(satdata.beta()) <= beta_threshold_cast * D2R)
        {
            double sinu = sin(satdata.orb_angle());
            double f_function = 1.0 / (1.0 + d_constant * (sinu) * (sinu) * (sinu) * (sinu));
            double beta_modified = satdata.beta() + f_function * (sign(beta_threshold_cast * D2R, satdata.beta()) - satdata.beta());
            double yaw = atan2(-tan(beta_modified), sinu);
            _yaw2ijk(satdata, yaw, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_cys_cast(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double beta_threshold_cast = 2.8;
        double d_constant = 80000.0;

        double beta = _beta;
        double mi = _mi;
        if (fabs(beta) <= beta_threshold_cast * D2R)
        {
            double sinu = sin(mi);
            double f_function = 1.0 / (1.0 + d_constant * (sinu) * (sinu) * (sinu) * (sinu));
            double beta_modified = beta + f_function * (sign(beta_threshold_cast * D2R, beta) - beta);
            double yaw = atan2(-tan(beta_modified), sinu);
            _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
        }
        else
        {
            _ysm(prn, beta, mi, xsat, vsat, xsun, i, j, k);
        }
    }

    // continuous yaw steering attitude modes of BDS-3 SECM satellites, added by yqyuan
    void gnss_model_ppp::_cys_secm(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        double beta_threshold_secm = 3.0;
        if (fabs(satdata.beta()) <= beta_threshold_secm * D2R)
        {
            double sinu = sin(satdata.orb_angle());
            double Sox = -sinu * cos(satdata.beta());
            // double Soy = -sin(satdata.beta());
            double yaw = atan2(-sin(sign(beta_threshold_secm * D2R, satdata.beta())), -Sox);
            _yaw2ijk(satdata, yaw, i, j, k);
        }
        else
        {
            _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_cys_secm(std::string prn, double _beta, double _mi, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double beta_threshold_secm = 3.0;

        double beta = _beta;
        double mi = _mi;

        if (fabs(beta) <= beta_threshold_secm * D2R)
        {
            double sinu = sin(mi);
            double Sox = -sinu * cos(beta);
            // double Soy = -sin(beta);
            double yaw = atan2(-sin(sign(beta_threshold_secm * D2R, beta)), -Sox);
            _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
        }
        else
        {
            _ysm(prn, beta, mi, xsat, vsat, xsun, i, j, k);
        }
    }

    // Attitude modelling for BeiDou
    void gnss_model_ppp::_attitude_BDS(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {

        if (gnss_sys::bds_geo(satdata.sat()))
        {
            _onm(satdata, i, j, k);
        }
        // added by yqyuan for CYS mode of four BDS-2 and all BDS-3 CAST M/I satellites
        else if (gnss_sys::bds_cast(satdata.sat()))
        {
            _cys_cast(satdata, i, j, k);
        }
        // added by yqyuan for CYS mode of 10 BDS-3 SECM M satellites
        else if (gnss_sys::bds_secm(satdata.sat()))
        {
            _cys_secm(satdata, i, j, k);
        }
        // else using YS/ON switch mode
        else
        {
            if (fabs(satdata.beta()) <= 4.0 * D2R)
            { // could be updated; ref Dai X. 2015 and eclips_Dec2017.f by J. Kouba
                _onm(satdata, i, j, k);
            }
            else
                _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_attitude_BDS(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        double orb_angle = _orb_angle(xsat, vsat, xsun);
        double beta = _beta(xsat, vsat, xsun);

        if (gnss_sys::bds_geo(prn))
        {
            _onm(xsat, vsat, xsun, i, j, k);
        }
        // added by yqyuan for CYS mode of four BDS-2 and all BDS-3 CAST M/I satellites
        else if (gnss_sys::bds_cast(prn))
        {
            _cys_cast(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        // added by yqyuan for CYS mode of 10 BDS-3 SECM M satellites
        else if (gnss_sys::bds_secm(prn))
        {
            _cys_secm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
        // else using YS/ON switch mode
        else
        {
            if (fabs(beta) <= 4.0 * D2R)
            { // could be updated; ref Dai X. 2015 and eclips_Dec2017.f by J. Kouba
                _onm(xsat, vsat, xsun, i, j, k);
            }
            else
                _ysm(prn, beta, orb_angle, xsat, vsat, xsun, i, j, k);
        }
    }

    // continuous yaw steering attitude modes of QZS-2 IGSO satellites, added by yqyuan
    void gnss_model_ppp::_cys_qzs(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {
        // genernal settings and thresholds
        double beta_threshold_qzs = 5.0 * D2R; // radius
        double yrate = 0.055 * D2R;            // radius
        double us_noon_init = 173.0 * D2R;     // radius
        double ue_noon_init = 187.0 * D2R;     // radius
        double us_night_init = -7 * D2R;       // radius
        double ue_night_init = 7 * D2R;        // radius

        // loca vars. used in iters to compute us and ue
        double phi_us = 0.0;
        double phi_ue = 0.0;
        double yaw = 0.0;
        int ieclips = 0;
        double rotation_direction = 0.0;

        // sat_pos and sat_vel for murate
        Triple satvel_t = satdata.satvel();
        Triple satvel = satvel_t;
        double satvel_norm = satvel.norm();
        Triple satcrd_t = satdata.satcrd();
        Triple satcrd = satcrd_t;
        double satcrd_norm = satcrd.norm();
        double murate = (satvel_norm / satcrd_norm); // in radius
        // beta and u angle
        double beta = satdata.beta();   // in radius
        double u = satdata.orb_angle(); // in radius

        if (fabs(satdata.beta()) <= beta_threshold_qzs && murate / fabs(tan(beta)) >= yrate)
        {
            if (fabs(u <= 20 * D2R))
            { // mid-night turns
                double us_night = us_night_init;
                double ue_night = ue_night_init;
                // compute us and ue
                for (int iter_night = 1; iter_night <= 10; iter_night++)
                {
                    phi_us = atan2(tan(beta), -sin(us_night));
                    phi_ue = atan2(tan(beta), -sin(ue_night));
                    us_night = murate / yrate * (fabs(phi_us) - hwa_pi / 2);
                    ue_night = murate / yrate * (fabs(phi_ue) - hwa_pi / 2);
                }
                if ((u >= us_night) && (u <= ue_night))
                {
                    ieclips = 1;
                    phi_us = atan2(tan(beta), -sin(us_night));
                    phi_ue = atan2(tan(beta), -sin(ue_night));
                    rotation_direction = sign(1.0, phi_ue - phi_us);
                    yaw = phi_us + rotation_direction * yrate * (u - us_night) / (murate);
                }
            }
            if (fabs(u >= 160 * D2R))
            { // noon turns
                double us_noon = us_noon_init;
                double ue_noon = ue_noon_init;
                for (int iter_noon = 1; iter_noon <= 10; iter_noon++)
                {
                    phi_us = atan2(tan(beta), -sin(us_noon));
                    phi_ue = atan2(tan(beta), -sin(ue_noon));
                    us_noon = murate / yrate * (fabs(phi_us) - hwa_pi / 2) + hwa_pi;
                    ue_noon = murate / yrate * (fabs(phi_ue) - hwa_pi / 2) + hwa_pi;
                }
                if (u >= 0)
                {
                    if ((u >= us_noon) && (u <= ue_noon))
                    {
                        ieclips = 1;
                        phi_us = atan2(tan(beta), -sin(us_noon));
                        phi_ue = atan2(tan(beta), -sin(ue_noon));
                        rotation_direction = sign(1.0, phi_ue - phi_us);
                        yaw = phi_us + rotation_direction * yrate * (u - us_noon) / (murate);
                    }
                }
                else
                {
                    if ((u + 2 * hwa_pi >= us_noon) && (u + 2 * hwa_pi <= ue_noon))
                    {
                        ieclips = 1;
                        phi_us = atan2(tan(beta), -sin(us_noon));
                        phi_ue = atan2(tan(beta), -sin(ue_noon));
                        rotation_direction = sign(1.0, phi_ue - phi_us);
                        yaw = phi_us + rotation_direction * yrate * (u + 2 * hwa_pi - us_noon) / (murate);
                    }
                }
            }
            if (ieclips == 1)
            {
                _yaw2ijk(satdata, yaw, i, j, k);
            }
        }
        else
        {
            _ysm(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_cys_qzs(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        // genernal settings and thresholds
        double beta_threshold_qzs = 5.0 * D2R; // radius
        double yrate = 0.055 * D2R;            // radius
        double us_noon_init = 173.0 * D2R;     // radius
        double ue_noon_init = 187.0 * D2R;     // radius
        double us_night_init = -7 * D2R;       // radius
        double ue_night_init = 7 * D2R;        // radius

        // loca vars. used in iters to compute us and ue
        double phi_us = 0.0;
        double phi_ue = 0.0;
        double yaw = 0.0;
        // int ieclips = 0;
        double rotation_direction = 0.0;

        // sat_pos and sat_vel for murate
        Vector satvel = vsat;
        double satvel_norm = satvel.norm();
        Vector satcrd = xsat;
        double satcrd_norm = satcrd.norm();
        double murate = (satvel_norm / satcrd_norm); // in radius
                                                     // beta and u angle
        double beta = _beta(xsat, vsat, xsun);
        double u = _orb_angle(xsat, vsat, xsun);

        yaw = atan2(tan(beta), -sin(u)); // in QZS SCF

        if (fabs(beta) <= beta_threshold_qzs && murate / fabs(tan(beta)) >= yrate)
        {
            if (fabs(u <= 20 * D2R))
            { // mid-night turns
                double us_night = us_night_init;
                double ue_night = ue_night_init;
                // compute us and ue
                for (int iter_night = 1; iter_night <= 10; iter_night++)
                {
                    phi_us = atan2(tan(beta), -sin(us_night)); // in QZS SCF
                    phi_ue = atan2(tan(beta), -sin(ue_night)); // in QZS SCF
                    us_night = murate / yrate * (fabs(phi_us) - hwa_pi / 2);
                    ue_night = murate / yrate * (fabs(phi_ue) - hwa_pi / 2);
                }
                if ((u >= us_night) && (u <= ue_night))
                {
                    // ieclips = 1;
                    phi_us = atan2(tan(beta), -sin(us_night)); // in QZS SCF
                    phi_ue = atan2(tan(beta), -sin(ue_night)); // in QZS SCF
                    rotation_direction = sign(1.0, phi_ue - phi_us);
                    yaw = phi_us + rotation_direction * yrate * (u - us_night) / (murate); // in QZS SCF
                }
            }
            if (fabs(u >= 160 * D2R))
            { // noon turns
                double us_noon = us_noon_init;
                double ue_noon = ue_noon_init;
                for (int iter_noon = 1; iter_noon <= 10; iter_noon++)
                {
                    phi_us = atan2(tan(beta), -sin(us_noon)); // in QZS SCF
                    phi_ue = atan2(tan(beta), -sin(ue_noon)); // in QZS SCF
                    us_noon = murate / yrate * (fabs(phi_us) - hwa_pi / 2) + hwa_pi;
                    ue_noon = murate / yrate * (fabs(phi_ue) - hwa_pi / 2) + hwa_pi;
                }
                if (u >= 0)
                {
                    if ((u >= us_noon) && (u <= ue_noon))
                    {
                        // ieclips = 1;
                        phi_us = atan2(tan(beta), -sin(us_noon)); // in QZS SCF
                        phi_ue = atan2(tan(beta), -sin(ue_noon)); // in QZS SCF
                        rotation_direction = sign(1.0, phi_ue - phi_us);
                        yaw = phi_us + rotation_direction * yrate * (u - us_noon) / (murate); // in QZS SCF
                    }
                }
                else
                {
                    if ((u + 2 * hwa_pi >= us_noon) && (u + 2 * hwa_pi <= ue_noon))
                    {
                        // ieclips = 1;
                        phi_us = atan2(tan(beta), -sin(us_noon)); // in QZS SCF
                        phi_ue = atan2(tan(beta), -sin(ue_noon)); // in QZS SCF
                        rotation_direction = sign(1.0, phi_ue - phi_us);
                        yaw = phi_us + rotation_direction * yrate * (u + 2 * hwa_pi - us_noon) / (murate); // in QZS SCF
                    }
                }
            }
        }
        // QZS SCF to IGS SCF
        if (beta > 0)
        {
            // yaw_qzsscf > 0 and yaw_igsscf < 0
            yaw -= hwa_pi;
        }
        else
        {
            // yaw_qzsscf < 0 and yaw_igsscf > 0
            yaw += hwa_pi;
        }
        _yaw2ijk(xsat, vsat, xsun, yaw, i, j, k);
    }

    // attitude mode switch between YS and ON, only for QZS-1, added by yqyuan 20210502;
    // only works since the first half of 2017
    void gnss_model_ppp::_switch_qzs1(gnss_data_sats &satdata, Vector &i, Vector &j, Vector &k)
    {

        // ** get basic parameters: orbital beta and u **//
        double beta = satdata.beta();   // in radius
        double u = satdata.orb_angle(); // in radius
        Triple satvel_t = satdata.satvel();
        Triple satvel = satvel_t;
        double satvel_norm = satvel.norm();
        Triple satcrd_t = satdata.satcrd();
        Triple satcrd = satcrd_t;
        double satcrd_norm = satcrd.norm();
        double murate = (satvel_norm / satcrd_norm); // in radius

        double beta_first = _get_beta0();
        if (beta_first == 0)
        {
            _set_beta0(beta);
        }
        double beta_increased = beta - beta_first;

        // ** initialization **//
        double beta0_ys2on = 17.0;     // in deg
        double beta0_on2ys = 18.0;     // in deg
        double dbeta_threshold = 0.5;  // in deg; may have problem for the "0.49" situation.
        double yawrate = 0.0097 * D2R; // in rad/s
        int lys2on = 1;                // 1 for ys2on; 2 for on2ys
        double beta0 = beta0_ys2on;
        double us_switch = 0.0;
        double ue_switch = 0.0;
        //double  dmjd_2017 = 57754.0;
        //if(dmjd < dmjd_2017) {
        //    double beta0_ys2on = 20.0;       // in deg
        //    double beta0_on2ys = 20.0;       // in deg
        //    double yawrate = 0.0100*D2R;     // in rad/s  in case of processing data before 2017
        //}
        //// note: doy < 180 may only only work until for example 2025?
        //// a better way to determine YS/ON or YS/ON wouble be: beta*(beta_rate)>0 or beta*(beta_rate)<0
        //// however, the satellite may be replaced by the new one by about 2022?
        //if (doy < 180) {
        //    if (beta > 0) {
        //        beta0 = beta0_on2ys;
        //        lys2on = 2;
        //        us_switch = -110.0*D2R;
        //        ue_switch = -90.0*D2R;
        //    }
        //    else {
        //        beta0 = beta0_ys2on;
        //        lys2on = 1;
        //        us_switch = -90.0*D2R;
        //        ue_switch = -70.0*D2R;
        //    }
        //}
        //else {
        //    if (beta > 0) {
        //        beta0 = beta0_ys2on;
        //        lys2on = 1;
        //        us_switch = -90.0*D2R;
        //        ue_switch = -70.0*D2R;
        //    }
        //    else {
        //        beta0 = beta0_on2ys;
        //        lys2on = 2;
        //        us_switch = -110.0*D2R;
        //        ue_switch = -90.0*D2R;
        //    }
        //}
        if (beta * beta_increased < 0)
        {
            beta0 = beta0_ys2on;
            lys2on = 1;
            us_switch = -90.0 * D2R;
            ue_switch = -70.0 * D2R;
        }
        else
        {
            beta0 = beta0_on2ys;
            lys2on = 2;
            us_switch = -110.0 * D2R;
            ue_switch = -90.0 * D2R;
        }

        int yaw_mode = 1; // 1 for YS; 2 for ON; 3 for switches
        double phi = 0;
        // ** determining  **//
        if (abs(beta / D2R) > beta0 + dbeta_threshold || abs(beta / D2R) < beta0 - dbeta_threshold)
        {
            if (abs(beta / D2R) > beta0 + dbeta_threshold)
            {
                yaw_mode = 1;
            }
            else
            {
                yaw_mode = 2;
            }
        }
        else
        { // possible switching period
            double phi_us = 0;
            double phi_ue = 0;
            if (u > us_switch && u < ue_switch)
            {
                if (lys2on == 1)
                {
                    phi_us = atan2(tan(beta), -sin(us_switch));
                    phi_ue = 0.0;
                    ue_switch = us_switch + abs(murate * phi_us / yawrate);
                }
                else
                {
                    phi_us = 0.0;
                    phi_ue = atan2(tan(beta), -sin(ue_switch));
                    ue_switch = ue_switch - abs(murate * phi_ue / yawrate);
                }
                if (u > us_switch && u < ue_switch)
                {
                    yaw_mode = 3;
                    if (lys2on == 1)
                    {
                        phi = phi_us + sign(yawrate, (-1) * phi_us) * abs(u - us_switch) / murate;
                    }
                    else
                    {
                        phi = phi_ue - sign(yawrate, phi_ue) * abs(ue_switch - u) / murate;
                    }
                }
                else
                {
                    if ((u < us_switch && lys2on == 2) || (u > ue_switch && lys2on == 1))
                    {
                        yaw_mode = 2;
                    }
                    else
                    {
                        yaw_mode = 1;
                    }
                }
            }
            else
            {
                if (u < us_switch)
                { // not changed
                    yaw_mode = lys2on;
                }
                else
                { //  u > ue_switch : changed
                    yaw_mode = 3 - lys2on;
                }
            }
        }

        // ** update yaw angle and X Y Z (i j k) **//
        // QZS SCF to IGS SCF
        if (beta > 0)
        {
            // yaw_qzsscf > 0 and yaw_igsscf < 0
            phi -= hwa_pi;
        }
        else
        {
            // yaw_qzsscf < 0 and yaw_igsscf > 0
            phi += hwa_pi;
        }
        switch (yaw_mode)
        {
        case 1:
            _ysm(satdata, i, j, k);
            break;
        case 2:
            _onm(satdata, i, j, k);
            break;
        case 3:
            _yaw2ijk(satdata, phi, i, j, k);
            break;
        default:;
        }
    }

    // attitude mode switch between YS and ON, only for QZS-1, added by yqyuan 20210502;
    // only works since the first half of 2017
    void gnss_model_ppp::_switch_qzs1(Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {

        // ** get basic parameters: orbital beta and u **//
        Vector satvel = vsat;
        double satvel_norm = satvel.norm();
        Vector satcrd = xsat;
        double satcrd_norm = satcrd.norm();
        double murate = (satvel_norm / satcrd_norm); // in radius
                                                     // beta and u angle
        double beta = _beta(xsat, vsat, xsun);
        double u = _orb_angle(xsat, vsat, xsun);

        double beta_first = _get_beta0();
        if (beta_first == 0)
        {
            _set_beta0(beta);
        }
        double beta_increased = beta - beta_first;

        // ** initialization **//
        double beta0_ys2on = 17.0;     // in deg
        double beta0_on2ys = 18.0;     // in deg
        double dbeta_threshold = 0.5;  // in deg; may have problem for the "0.49" situation.
        double yawrate = 0.0097 * D2R; // in rad/s
        int lys2on = 1;                // 1 for ys2on; 2 for on2ys
        double beta0 = beta0_ys2on;
        double us_switch = 0.0;
        double ue_switch = 0.0;
        //double  dmjd_2017 = 57754.0;
        //if (dmjd < dmjd_2017) {
        //    double beta0_ys2on = 20.0;       // in deg
        //    double beta0_on2ys = 20.0;       // in deg
        //    double yawrate = 0.0100*D2R;     // in rad/s  in case of processing data before 2017
        //}
        // note: doy < 180 may only only work until for example 2025?
        // a better way to determine YS/ON or YS/ON wouble be: beta*(beta_rate)>0 or beta*(beta_rate)<0
        // however, the satellite may be replaced by the new one by about 2022?
        /*if (doy < 180) {
        if (beta > 0) {
            beta0 = beta0_on2ys;
            lys2on = 2;
            us_switch = -110.0*D2R;
            ue_switch = -90.0*D2R;
        }
        else {
            beta0 = beta0_ys2on;
            lys2on = 1;
            us_switch = -90.0*D2R;
            ue_switch = -70.0*D2R;
        }
    }
    else {
        if (beta > 0) {
            beta0 = beta0_ys2on;
            lys2on = 1;
            us_switch = -90.0*D2R;
            ue_switch = -70.0*D2R;
        }
        else {
            beta0 = beta0_on2ys;
            lys2on = 2;
            us_switch = -110.0*D2R;
            ue_switch = -90.0*D2R;
        }
    }*/
        if (beta * beta_increased < 0)
        {
            beta0 = beta0_ys2on;
            lys2on = 1;
            us_switch = -90.0 * D2R;
            ue_switch = -70.0 * D2R;
        }
        else
        {
            beta0 = beta0_on2ys;
            lys2on = 2;
            us_switch = -110.0 * D2R;
            ue_switch = -90.0 * D2R;
        }

        int yaw_mode = 1; // 1 for YS; 2 for ON; 3 for switches
        double phi = 0;   // actual yaw angle
        // ** determining  **//
        if (abs(beta / D2R) > beta0 + dbeta_threshold || abs(beta / D2R) < beta0 - dbeta_threshold)
        {
            if (abs(beta / D2R) > beta0 + dbeta_threshold)
            {
                yaw_mode = 1;
            }
            else
            {
                yaw_mode = 2;
            }
        }
        else
        { // possible switching period
            double phi_us = 0;
            double phi_ue = 0;
            if (u > us_switch && u < ue_switch)
            {
                if (lys2on == 1)
                {
                    phi_us = atan2(tan(beta), -sin(us_switch));
                    phi_ue = 0.0;
                    ue_switch = us_switch + abs(murate * phi_us / yawrate);
                }
                else
                {
                    phi_us = 0.0;
                    phi_ue = atan2(tan(beta), -sin(ue_switch));
                    ue_switch = ue_switch - abs(murate * phi_ue / yawrate);
                }
                if (u > us_switch && u < ue_switch)
                {
                    yaw_mode = 3;
                    if (lys2on == 1)
                    {
                        phi = phi_us + sign(yawrate, (-1) * phi_us) * abs(u - us_switch) / murate;
                    }
                    else
                    {
                        phi = phi_ue - sign(yawrate, phi_ue) * abs(ue_switch - u) / murate;
                    }
                }
                else
                {
                    if ((u < us_switch && lys2on == 2) || (u > ue_switch && lys2on == 1))
                    {
                        yaw_mode = 2;
                    }
                    else
                    {
                        yaw_mode = 1;
                    }
                }
            }
            else
            {
                if (u < us_switch)
                { // not changed
                    yaw_mode = lys2on;
                }
                else
                { //  u > ue_switch : changed
                    yaw_mode = 3 - lys2on;
                }
            }
        }

        // ** update yaw angle and X Y Z (i j k) **//
        std::string prn = "J01";
        // QZS SCF to IGS SCF
        if (beta > 0)
        {
            // yaw_qzsscf > 0 and yaw_igsscf < 0
            phi -= hwa_pi;
        }
        else
        {
            // yaw_qzsscf < 0 and yaw_igsscf > 0
            phi += hwa_pi;
        }
        switch (yaw_mode)
        {
        case 1:
            _ysm(prn, beta, u, xsat, vsat, xsun, i, j, k);
            break;
        case 2:
            _onm(xsat, vsat, xsun, i, j, k);
            break;
        case 3:
            _yaw2ijk(xsat, vsat, xsun, phi, i, j, k);
            break;
        default:;
        }
    }

    // Attitude modelling for QZSS
    void gnss_model_ppp::_attitude_QZS(gnss_data_sats &satdata, std::string antype, Vector &i, Vector &j, Vector &k)
    {
        if (satdata.sat().find("J01") != std::string::npos)
        {
            if (fabs(satdata.beta()) <= 20 * D2R)
            {                           // may be better 17 or 18 deg; to be investegated; yqyuan.
                _onm(satdata, i, j, k); // i oposite velocity direction //wrong comment. i == velocity direction
                                        // bug fixed.(yqyuan) _yaw2ijk calculate scf2crs using Xsat and Vsat (in IGS convention); independet of SCF oriention.
                                        //i = -1 * i;
                                        //j = -1 * j;
            }
            else
                _ysm(satdata, i, j, k);
        }
        // GEO satellite following ON mode, added by yqyuan
        else if (satdata.sat().find("J07") != std::string::npos)
        {
            _onm(satdata, i, j, k);
        }
        // QZS-2 IGSO satellites follows CYS mode, added by yqyuan
        else
        {
            _cys_qzs(satdata, i, j, k);
        }
    }

    void gnss_model_ppp::_attitude_QZS(std::string antype, std::string prn, Vector &xsat, Vector &vsat, Vector &xsun, Vector &i, Vector &j, Vector &k)
    {
        // double orb_angle = _orb_angle(xsat, vsat, xsun);
        // double beta = _beta(xsat, vsat, xsun);

        if (prn.find("J01") != std::string::npos)
        {
            _switch_qzs1(xsat, vsat, xsun, i, j, k);
        }
        // GEO satellite following ON mode, added by yqyuan
        else if (prn.find("J07") != std::string::npos)
        {
            _onm(xsat, vsat, xsun, i, j, k);
        }
        // QZS-2 IGSO satellites follows CYS mode, added by yqyuan
        else
        {
            _cys_qzs(xsat, vsat, xsun, i, j, k);
        }
    }

    // Calculate satellite-std::fixed std::vectors from yaw angle
    void gnss_model_ppp::_yaw2ijk(gnss_data_sats &satdata, double &yaw, Vector &i, Vector &j, Vector &k)
    {
        satdata.yaw(yaw); // store yaw angle

        Vector satcrd = satdata.satcrd();
        Vector satvel = satdata.satvel();

        if (satcrd.norm() == 0 || satvel.norm() == 0)
        {
            return;
        }

        // ITRF -> ICRF velocity
        satvel(0) -= OMEGA * satcrd(1);
        satvel(1) += OMEGA * satcrd(0);

        Triple n = Triple(satcrd).cross(Triple(satvel));

        // Satelite-Earth unit std::vector
        k = -satcrd;
        k /= k.norm();

        Triple ex = n.cross(Triple(satcrd));

        ex /= ex.norm();
        n /= n.norm();

        double cosy = cos(yaw);
        double siny = sin(yaw);
        for (int r = 0; r < 3; r++)
        {
            i(r) = -siny * n(r) + cosy * ex(r);
            j(r) = -cosy * n(r) - siny * ex(r);
        }
    }

    void gnss_model_ppp::_yaw2ijk(Vector &xsat, Vector &vsat, Vector &xsun, double &yaw, Vector &i, Vector &j, Vector &k)
    {
        //satdata.yaw(yaw);   // store yaw angle
        Triple satcrd = xsat.segment(0,3);
        Triple satvel = vsat.segment(0,3);

        if (satcrd.norm() == 0 || satvel.norm() == 0)
        {
            return;
        }

        // ITRF -> ICRF velocity
        satvel(0) -= OMEGA * satcrd(1);
        satvel(1) += OMEGA * satcrd(0);

        Triple n = satcrd.cross(satvel);

        // Satelite-Earth unit std::vector
        k = -satcrd;
        k /= k.norm();

        Triple ex = n.cross(satcrd);

        ex /= ex.norm();
        n /= n.norm();

        double cosy = cos(yaw);
        double siny = sin(yaw);
        for (int r = 0; r < 3; r++)
        {
            i(r) = -siny * n(r) + cosy * ex(r);
            j(r) = -cosy * n(r) - siny * ex(r);
        }
    }

    // sign function: return the value of a with the sign of b; added by yqyuan
    double gnss_model_ppp::sign(double a, double b)
    {
        double value = fabs(a);
        int sign_of_b = 1;
        if (b >= 0)
        {
            ;
        }
        else
        {
            sign_of_b = -1;
        }
        return value * sign_of_b;
    }

    void gnss_model_ppp::_set_beta0(double beta)
    {
        _beta0 = beta;
    }

    double gnss_model_ppp::_get_beta0()
    {
        return _beta0;
    }

} // namespace
