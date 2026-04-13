#include "hwa_gnss_model_spp.h"
#include "hwa_gnss_model_tropoBlind.h"
#include "hwa_set_gen.h"

using namespace std;

namespace hwa_gnss
{
    gnss_model_spp::gnss_model_spp()
    {
        _tropoModel = std::make_shared<gnss_model_tropo>();
        _gallbias = 0;
    }

    gnss_model_spp::gnss_model_spp(string site, set_base *settings)
        : _observ(OBSCOMBIN::IONO_FREE)
    {

        _tropoModel = std::make_shared<gnss_model_tropo>();
        _gallbias = 0;

        _settings = settings;
        _site = site;
        _phase = false;

        set<GSYS> systems = gnss_supported();
        for (set<GSYS>::iterator it = systems.begin(); it != systems.end(); it++)
        {
            _maxres_C[*it] = dynamic_cast<set_gnss *>(_settings)->maxres_C(*it);
            _maxres_L[*it] = dynamic_cast<set_gnss *>(_settings)->maxres_L(*it);
        }

        _maxres_norm = dynamic_cast<set_gproc *>(_settings)->max_res_norm();
        _tropo_mf = dynamic_cast<set_gproc *>(_settings)->tropo_mf();
        _trpModStr = dynamic_cast<set_gproc *>(_settings)->tropo_model();

        _resid_type = dynamic_cast<set_gproc *>(_settings)->residuals();
        _observ = dynamic_cast<set_gproc *>(_settings)->obs_combin();
        _cbiaschar = dynamic_cast<set_gproc *>(_settings)->cbiaschar();

        _band_index[GPS] = dynamic_cast<set_gnss *>(_settings)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(_settings)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(_settings)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(_settings)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(_settings)->band_index(QZS);

        _freq_index[GPS] = dynamic_cast<set_gnss *>(_settings)->freq_index(GPS);
        _freq_index[GAL] = dynamic_cast<set_gnss *>(_settings)->freq_index(GAL);
        _freq_index[GLO] = dynamic_cast<set_gnss *>(_settings)->freq_index(GLO);
        _freq_index[BDS] = dynamic_cast<set_gnss *>(_settings)->freq_index(BDS);
        _freq_index[QZS] = dynamic_cast<set_gnss *>(_settings)->freq_index(QZS);

        if (_trpModStr == TROPMODEL::SAASTAMOINEN)
            _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
        else if (_trpModStr == TROPMODEL::DAVIS)
            _tropoModel = std::make_shared<gnss_model_tropo_davis>();
        else if (_trpModStr == TROPMODEL::HOPFIELD)
            _tropoModel = std::make_shared<gnss_model_tropo_hopf>();
        else if (_trpModStr == TROPMODEL::MOPS)
            _tropoModel = std::make_shared<gnss_model_blindtropos>(_spdlog);
    }
    gnss_model_spp::gnss_model_spp(base_log spdlog, string site, set_base *settings)
        : _observ(OBSCOMBIN::IONO_FREE)
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
        if (nullptr == settings)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _settings = settings;
        }
        _tropoModel = std::make_shared<gnss_model_tropo>();
        _gallbias = 0;
        _site = site;
        _phase = false;
        set<GSYS> systems = gnss_supported();
        for (set<GSYS>::iterator it = systems.begin(); it != systems.end(); it++)
        {
            _maxres_C[*it] = dynamic_cast<set_gnss *>(_settings)->maxres_C(*it);
            _maxres_L[*it] = dynamic_cast<set_gnss *>(_settings)->maxres_L(*it);
        }
        _maxres_norm = dynamic_cast<set_gproc *>(_settings)->max_res_norm();
        _tropo_mf = dynamic_cast<set_gproc *>(_settings)->tropo_mf();
        _trpModStr = dynamic_cast<set_gproc *>(_settings)->tropo_model();
        _resid_type = dynamic_cast<set_gproc *>(_settings)->residuals();
        _observ = dynamic_cast<set_gproc *>(_settings)->obs_combin();
        _cbiaschar = dynamic_cast<set_gproc *>(_settings)->cbiaschar();
        _band_index[GPS] = dynamic_cast<set_gnss *>(_settings)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(_settings)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(_settings)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(_settings)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(_settings)->band_index(QZS);
        _freq_index[GPS] = dynamic_cast<set_gnss *>(_settings)->freq_index(GPS);
        _freq_index[GAL] = dynamic_cast<set_gnss *>(_settings)->freq_index(GAL);
        _freq_index[GLO] = dynamic_cast<set_gnss *>(_settings)->freq_index(GLO);
        _freq_index[BDS] = dynamic_cast<set_gnss *>(_settings)->freq_index(BDS);
        _freq_index[QZS] = dynamic_cast<set_gnss *>(_settings)->freq_index(QZS);
        if (_trpModStr == TROPMODEL::SAASTAMOINEN)
            _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
        else if (_trpModStr == TROPMODEL::DAVIS)
            _tropoModel = std::make_shared<gnss_model_tropo_davis>();
        else if (_trpModStr == TROPMODEL::HOPFIELD)
            _tropoModel = std::make_shared<gnss_model_tropo_hopf>();
        else if (_trpModStr == TROPMODEL::MOPS)
            _tropoModel = std::make_shared<gnss_model_blindtropos>(_spdlog);
        // else if (_trpModStr == GAL27)         _tropoModel = std::make_shared<t_blindgal27>();
        // else if (_trpModStr == GALTROPO27)    _tropoModel = std::make_shared<t_blindgal27>();
    }

    // Destructor
    // ---------------------
    gnss_model_spp::~gnss_model_spp()
    {
    }

    // Outliers detection based on chi2 testing of normalized residuals
    // ----------
    int gnss_model_spp::outlierDetect_chi(vector<gnss_data_sats> &data,
                                       Symmetric &Qx,
                                       const Symmetric &Qsav,
                                       const Vector &v)
    {

        vector<gnss_data_sats>::iterator it;
        vector<gnss_data_sats>::iterator itMaxV;

        double maxV = 0.0;

        int ii = 0;
        for (it = data.begin(); it != data.end(); it++)
        {
            ++ii;

            if (maxV == 0.0 || v(ii) * v(ii) > maxV)
            {

                maxV = v(ii) * v(ii);
                itMaxV = it;
            }

            if (_phase)
            {
                ++ii;
                if (maxV == 0.0 || v(ii) * v(ii) > maxV)
                {

                    maxV = v(ii) * v(ii);
                    itMaxV = it;
                }
            }
        }

        if (maxV > 5.024)
        { // chi2(100) = 2.706; chi2(050) = 3.841; chi2(025) = 5.024; chi2(010) = 6.635; chi2(005) = 7.879;

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _site + " outlier " + itMaxV->sat() + " size:" + base_type_conv::int2str(data.size()) + " v_norm: " + base_type_conv::dbl2str(maxV) + " " + itMaxV->epoch().str_hms());
            data.erase(itMaxV);
            Qx = Qsav;
            return 1;
        }

        return 0;
    }

    // Outliers detection
    // ----------
    int gnss_model_spp::outlierDetect(vector<gnss_data_sats> &data,
                                   Symmetric &Qx,
                                   const Symmetric &Qsav,
                                   const Vector &v)
    {

        vector<gnss_data_sats>::iterator itMaxVcodeGPS;
        vector<gnss_data_sats>::iterator itMaxVcodeGLO;
        vector<gnss_data_sats>::iterator itMaxVcodeGAL;
        vector<gnss_data_sats>::iterator itMaxVcodeBDS;
        vector<gnss_data_sats>::iterator itMaxVcodeQZS;

        vector<gnss_data_sats>::iterator itMaxVphaseGPS;
        vector<gnss_data_sats>::iterator itMaxVphaseGLO;
        vector<gnss_data_sats>::iterator itMaxVphaseGAL;
        vector<gnss_data_sats>::iterator itMaxVphaseBDS;
        vector<gnss_data_sats>::iterator itMaxVphaseQZS;

        double maxVcodeGPS, maxVcodeGLO, maxVcodeGAL, maxVcodeBDS, maxVcodeQZS;
        maxVcodeGPS = maxVcodeGLO = maxVcodeGAL = maxVcodeBDS = maxVcodeQZS = 0.0;

        double maxVphaseGPS, maxVphaseGLO, maxVphaseGAL, maxVphaseBDS, maxVphaseQZS;
        maxVphaseGPS = maxVphaseGLO = maxVphaseGAL = maxVphaseBDS = maxVphaseQZS = 0.0;

        // deviding multi-freq residual vector into single-freq vectors
        vector<Vector> v_frqs = _devide_res(v);

        // find maximal code/phase residuals for individual GNSS
        for (unsigned int i = 0; i < v_frqs.size(); i++)
        {
            double maxres = _maxres(v_frqs[i], GPS, false, data, itMaxVcodeGPS);
            if (maxres > maxVcodeGPS)
                maxVcodeGPS = maxres;
            maxres = _maxres(v_frqs[i], GLO, false, data, itMaxVcodeGLO);
            if (maxres > maxVcodeGLO)
                maxVcodeGLO = maxres;
            maxres = _maxres(v_frqs[i], GAL, false, data, itMaxVcodeGAL);
            if (maxres > maxVcodeGAL)
                maxVcodeGAL = maxres;
            maxres = _maxres(v_frqs[i], BDS, false, data, itMaxVcodeBDS);
            if (maxres > maxVcodeBDS)
                maxVcodeBDS = maxres;
            maxres = _maxres(v_frqs[i], QZS, false, data, itMaxVcodeQZS);
            if (maxres > maxVcodeQZS)
                maxVcodeQZS = maxres;

            maxres = _maxres(v_frqs[i], GPS, true, data, itMaxVphaseGPS);
            if (maxres > maxVphaseGPS)
                maxVphaseGPS = maxres;
            maxres = _maxres(v_frqs[i], GLO, true, data, itMaxVphaseGLO);
            if (maxres > maxVphaseGLO)
                maxVphaseGLO = maxres;
            maxres = _maxres(v_frqs[i], GAL, true, data, itMaxVphaseGAL);
            if (maxres > maxVphaseGAL)
                maxVphaseGAL = maxres;
            maxres = _maxres(v_frqs[i], BDS, true, data, itMaxVphaseBDS);
            if (maxres > maxVphaseBDS)
                maxVphaseBDS = maxres;
            maxres = _maxres(v_frqs[i], QZS, true, data, itMaxVphaseQZS);
            if (maxres > maxVphaseQZS)
                maxVphaseQZS = maxres;
        }

#ifdef DEBUG
        std::cout << "Max res range: " << maxVcodeGPS << " " << itMaxVcodeGPS->sat() << std::endl;
        std::cout << "Max res phase: " << maxVphaseGPS << " " << itMaxVphaseGPS->sat() << std::endl;
#endif
        //Only detect the outliers for the constellations with maximum outliers
        //The GLONASS is set as the basic reference
        double maxvc = maxVcodeGLO, maxvp = maxVphaseGLO;
        GSYS maxsys = GLO;
        if (maxVcodeGPS > maxvc || maxVphaseGPS > maxvp)
        {
            maxvc = maxVcodeGPS;
            maxvp = maxVphaseGPS;
            maxsys = GPS;
        }
        if (maxVcodeGAL > maxvc || maxVcodeGAL > maxvp)
        {
            maxvc = maxVcodeGAL;
            maxvp = maxVcodeGAL;
            maxsys = GAL;
        }
        if (maxVcodeBDS > maxvc || maxVphaseBDS > maxvp)
        {
            maxvc = maxVcodeBDS;
            maxvp = maxVphaseBDS;
            maxsys = BDS;
        }
        if (maxVcodeQZS > maxvc || maxVphaseQZS > maxvp)
        {
            maxvc = maxVcodeQZS;
            maxvp = maxVphaseQZS;
            maxsys = QZS;
        }

        if (maxsys == GLO && _check_outl(false, maxVcodeGLO, itMaxVcodeGLO, data))
        {
            data.erase(itMaxVcodeGLO);
            Qx = Qsav;
            return 1;
        }
        if (maxsys == GLO && _check_outl(true, maxVphaseGLO, itMaxVphaseGLO, data))
        {
            data.erase(itMaxVphaseGLO);
            Qx = Qsav;
            return 1;
        }

        if (maxsys == GPS && _check_outl(false, maxVcodeGPS, itMaxVcodeGPS, data))
        {
            data.erase(itMaxVcodeGPS);
            Qx = Qsav;
            return 1;
        }
        if (maxsys == GPS && _check_outl(true, maxVphaseGPS, itMaxVphaseGPS, data))
        {
            data.erase(itMaxVphaseGPS);
            Qx = Qsav;
            return 1;
        }

        if (maxsys == GAL && _check_outl(false, maxVcodeGAL, itMaxVcodeGAL, data))
        {
            data.erase(itMaxVcodeGAL);
            Qx = Qsav;
            return 1;
        }
        if (maxsys == GAL && _check_outl(true, maxVphaseGAL, itMaxVphaseGAL, data))
        {
            data.erase(itMaxVphaseGAL);
            Qx = Qsav;
            return 1;
        }

        if (maxsys == BDS && _check_outl(false, maxVcodeBDS, itMaxVcodeBDS, data))
        {
            data.erase(itMaxVcodeBDS);
            Qx = Qsav;
            return 1;
        }
        if (maxsys == BDS && _check_outl(true, maxVphaseBDS, itMaxVphaseBDS, data))
        {
            data.erase(itMaxVphaseBDS);
            Qx = Qsav;
            return 1;
        }

        if (maxsys == QZS && _check_outl(false, maxVcodeQZS, itMaxVcodeQZS, data))
        {
            data.erase(itMaxVcodeQZS);
            Qx = Qsav;
            return 1;
        }
        if (maxsys == QZS && _check_outl(true, maxVphaseQZS, itMaxVphaseQZS, data))
        {
            data.erase(itMaxVphaseQZS);
            Qx = Qsav;
            return 1;
        }

        return 0;
    }

    // Outliers detection
    // ----------
    int gnss_model_spp::outlierDetect(vector<gnss_data_sats> &data,
                                   Symmetric &Qx,
                                   const Symmetric &Qsav)
    {

        vector<gnss_data_sats>::iterator itMaxVcodeNORM = data.end();
        vector<gnss_data_sats>::iterator itMaxVphaseNORM = data.end();

        vector<gnss_data_sats>::iterator itMaxVcodeORIG = data.end();
        vector<gnss_data_sats>::iterator itMaxVphaseORIG = data.end();

        vector<gnss_data_sats>::iterator itDataErase = data.end();

        double maxVcodeNORM = 0.0;
        double maxVphaseNORM = 0.0;

        double maxVcodeORIG = 0.0;
        double maxVphaseORIG = 0.0;

        // find maximal code/phase residuals
        maxVcodeNORM = _maxres(false, data, itMaxVcodeNORM, RESIDTYPE::RES_NORM);
        maxVphaseNORM = _maxres(true, data, itMaxVphaseNORM, RESIDTYPE::RES_NORM);

        maxVcodeORIG = _maxres(false, data, itMaxVcodeORIG, RESIDTYPE::RES_ORIG);
        maxVphaseORIG = _maxres(true, data, itMaxVphaseORIG, RESIDTYPE::RES_ORIG);

#ifdef DEBUG
        if (itMaxVcodeNORM != data.end())
            std::cout << "Max res range norm: " << std::fixed << setprecision(3) << maxVcodeNORM << " " << itMaxVcodeNORM->sat() << std::endl;
        if (itMaxVphaseNORM != data.end())
            std::cout << "Max res phase norm: " << std::fixed << setprecision(3) << maxVphaseNORM << " " << itMaxVphaseNORM->sat() << std::endl;
        if (itMaxVcodeORIG != data.end())
            std::cout << "Max res range orig: " << std::fixed << setprecision(3) << maxVcodeORIG << " " << itMaxVcodeORIG->sat() << std::endl;
        if (itMaxVphaseORIG != data.end())
            std::cout << "Max res phase orig: " << std::fixed << setprecision(3) << maxVphaseORIG << " " << itMaxVphaseORIG->sat() << std::endl;
        int ooo;
        cin >> ooo;
#endif

        if (_check_outl(true, maxVphaseNORM, itMaxVphaseNORM, maxVphaseORIG, itMaxVphaseORIG, itDataErase, data))
        {
            //added by xiongyun to get erase sat
            auto it = find(_outlier_sat.begin(), _outlier_sat.end(), itDataErase->sat());
            if (it != _outlier_sat.end())
            {
                _outlier_sat.push_back(itDataErase->sat());
            }
            data.erase(itDataErase);
            Qx = Qsav;
            return 1;
        }
        if (_check_outl(false, maxVcodeNORM, itMaxVcodeNORM, maxVcodeORIG, itMaxVcodeORIG, itDataErase, data))
        {
            //added by xiongyun
            auto it = find(_outlier_sat.begin(), _outlier_sat.end(), itDataErase->sat());
            if (it == _outlier_sat.end())
            {
                _outlier_sat.push_back(itDataErase->sat());
            }
            data.erase(itDataErase);
            Qx = Qsav;
            return 1;
        }

        return 0;
    }

    int gnss_model_spp::outlierDetect(vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &Qsav, vector<gnss_data_sats>::iterator &exception_sat)
    {

        vector<gnss_data_sats>::iterator itMaxVcodeNORM = data.end();
        vector<gnss_data_sats>::iterator itMaxVphaseNORM = data.end();

        vector<gnss_data_sats>::iterator itMaxVcodeORIG = data.end();
        vector<gnss_data_sats>::iterator itMaxVphaseORIG = data.end();

        vector<gnss_data_sats>::iterator itDataErase = data.end();

        double maxVcodeNORM = 0.0;
        double maxVphaseNORM = 0.0;

        double maxVcodeORIG = 0.0;
        double maxVphaseORIG = 0.0;

        // find maximal code/phase residuals
        maxVcodeNORM = _maxres(false, data, itMaxVcodeNORM, RESIDTYPE::RES_NORM);
        maxVphaseNORM = _maxres(true, data, itMaxVphaseNORM, RESIDTYPE::RES_NORM);

        maxVcodeORIG = _maxres(false, data, itMaxVcodeORIG, RESIDTYPE::RES_ORIG);
        maxVphaseORIG = _maxres(true, data, itMaxVphaseORIG, RESIDTYPE::RES_ORIG);

        if (_check_outl(true, maxVphaseNORM, itMaxVphaseNORM, maxVphaseORIG, itMaxVphaseORIG, itDataErase, data))
        {
            exception_sat = itDataErase;
            Qx = Qsav;
            return 1;
        }
        if (_check_outl(false, maxVcodeNORM, itMaxVcodeNORM, maxVcodeORIG, itMaxVcodeORIG, itDataErase, data))
        {
            exception_sat = itDataErase;
            Qx = Qsav;
            return 1;
        }
        return 0;
    }

    // model for computed range value
    // ----------
    double gnss_model_spp::cmpObs(base_time &epoch, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs, bool com)
    {

        // Cartesian coordinates to ellipsodial coordinates
        Triple xyz, ell;

        // give crd initial value modified by glfeng
        //modified by lvhb , but no useing in there 20200917
        string strEst = dynamic_cast<set_gen *>(_settings)->estimator();
        bool isFLT = (strEst == "FLT");
        if (isFLT)
        {
            if (param.getCrdParam(_site, xyz) < 0)
            {
                xyz = _grec->crd_arp(epoch);
            }
        }
        else
        {
            if (param.getCrdParam(_site, xyz) < 0)
            {
                xyz = _grec->crd(epoch);
            }
            xyz += _grec->eccxyz(epoch);
        }

        xyz2ell(xyz, ell, false);

        Triple satcrd = gsatdata.satcrd();
        Vector cSat = satcrd;

        string sat = gsatdata.sat();
        string rec = gsatdata.site();
        base_time epo = gsatdata.epoch();

        // Tropospheric wet delay correction
        double trpDelay = 0;
        trpDelay = tropoDelay(epoch, param, ell, gsatdata);
        if (fabs(trpDelay) > 50)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "cmpObs", "trpDelay > 50");
            return -1;
        }

        // idx for par correction
        int i = -1;

        // Receiver clock correction
        double clkRec = 0.0;
        i = param.getParam(_site, par_type::CLK, "");
        if (i >= 0)
        {
            clkRec = param[i].value();
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, _site + " ! warning:  Receiver Clock is not included in parameters!");
        }

        // system time offset
        double isb_offset = isbCorrection(param, sat, rec, gobs);

        double ifb = 0.0;
        i = param.getParam(_site, par_type::IFB_GPS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }

        i = param.getParam(_site, par_type::IFB_GAL, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }
        i = param.getParam(_site, par_type::IFB_BDS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }
        i = param.getParam(_site, par_type::IFB_QZS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }

#ifdef DEBUG_LEO
        std::cout << setw(20) << " EPOCH is :     " << epoch.str_mjdsod() << " " << setw(6) << gsatdata.sat() << setw(6) << gsatdata.site()
             << setw(20) << " Band is :      " << setw(6) << gobs.band()
             << setw(20) << " gsatdata.rho() " << std::fixed << left << setw(20) << setprecision(6) << gsatdata.rho()
             << setw(20) << " clkRec         " << std::fixed << left << setw(20) << setprecision(6) << clkRec
             << setw(20) << " gsatdata.clk() " << std::fixed << left << setw(20) << setprecision(6) << gsatdata.clk()
             << setw(20) << " trpDelay       " << std::fixed << left << setw(20) << setprecision(6) << trpDelay
             << std::endl;
#endif
        // Return value
        return gsatdata.rho() +
               clkRec -
               gsatdata.clk() +
               trpDelay +
               isb_offset +
               ifb;
    }

    double gnss_model_spp::cmpObsIRC(base_time &epoch, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs, bool com)
    {

        // Cartesian coordinates to ellipsodial coordinates
        Triple xyz, ell;
        //Vector cRec(3);

        // give crd initial value modified by glfeng
        if (param.getCrdParam(_site, xyz) < 0)
        {
            xyz = _grec->crd(epoch);
        }

        xyz2ell(xyz, ell, false);

        Triple satcrd = gsatdata.satcrd();
        Triple cSat = satcrd;

        string sat = gsatdata.sat();
        base_time epo = gsatdata.epoch();

        // Tropospheric wet delay correction
        double trpDelay = 0;
        trpDelay = tropoDelay(epoch, param, ell, gsatdata);
        if (fabs(trpDelay) > 50)
            return -1;

        // Receiver clock correction
        double clkRec = 0.0;
        int i;
        par_type clktype;
        clktype = par_type::CLK;
        GSYS crt_sys = gnss_sys::str2gsys(sat.substr(0, 1));
        switch (crt_sys)
        {
        case GSYS::GPS:
            clktype = par_type::CLK_G;
            break;
        case GSYS::BDS:
            clktype = par_type::CLK_C;
            break;
        case GSYS::GAL:
            clktype = par_type::CLK_E;
            break;
        case GSYS::GLO:
            clktype = par_type::CLK_R;
            break;
        case GSYS::QZS:
            clktype = par_type::CLK_J;
            break;
        default:
            break;
        }
        i = param.getParam(_site, clktype, "");
        if (i >= 0)
            clkRec = param[i].value();

        // Inter frequency clocks bias
        double ifcb = 0.0;
        i = param.getParam(_site, par_type::IFCB_F3, gsatdata.sat());
        if (i >= 0 && gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_3)
        {
            ifcb = -param[i].value();
        }
        i = param.getParam(_site, par_type::IFCB_F4, gsatdata.sat());
        if (i >= 0 && gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_4)
        {
            ifcb = -param[i].value();
        }
        i = param.getParam(_site, par_type::IFCB_F5, gsatdata.sat());
        if (i >= 0 && gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_5)
        {
            ifcb = -param[i].value();
        }

        //LX changed the IFB expression
        //        // Inter frequency code bias FREQ_3
        //        double ifb = 0.0;
        //        i = param.getParam(_site, par_type::IFB_C3, "");
        //        if (i >= 0 && gobs.is_code() && gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_3) {
        //            ifb = param[i].value();
        //            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        //        }
        //
        //        // Inter frequency code bias FREQ_4
        //        i = param.getParam(_site, par_type::IFB_C4, "");
        //        if (i >= 0 && gobs.is_code() && gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_4) {
        //            ifb = param[i].value();
        //            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c4 << std::endl;
        //        }
        //
        //        // Inter frequency code bias FREQ_5
        //        i = param.getParam(_site, par_type::IFB_C5, "");
        //        if (i >= 0 && gobs.is_code() && gnss_sys::band2freq(gsatdata.gsys(), gobs.band()) == FREQ_5) {
        //            ifb = param[i].value();
        //            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c5 << std::endl;
        //        }

        double ifb = 0.0;
        i = param.getParam(_site, par_type::IFB_GPS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }

        i = param.getParam(_site, par_type::IFB_GAL, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }
        i = param.getParam(_site, par_type::IFB_BDS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }
        i = param.getParam(_site, par_type::IFB_QZS, "");
        if (i >= 0 && gobs.is_code() && _freq_index[gsatdata.gsys()][gobs.band()] == FREQ_3)
        {
            ifb = param[i].value();
            //std::cout << epoch.str_ymdhms() << " " << gsatdata.sat() << " " << std::fixed << setprecision(3) << setw(9) << ifb_c3 << std::endl;
        }

        // GLONASS system time offset
        double glonass_offset = 0;
        if (gsatdata.gsys() == GLO)
        {
            int i;
            i = param.getParam(_site, par_type::GLO_ISB, "");
            if (i >= 0)
            {
                glonass_offset = param[i].value();
                //std::cout << "GLO Offset: " << sat << " " << glonass_offset << " " << gsatdata.epoch().str_hms() << std::endl;
            }

            // add by ZHJ for GLO IFB
            i = param.getParam(_site, par_type::GLO_IFB, gsatdata.sat());
            if (i >= 0)
            {
                glonass_offset += param[i].value();
            }
        }

        // Galileo system time offset
        double galileo_offset = 0;
        if (gsatdata.gsys() == GAL)
        {
            int i;
            i = param.getParam(_site, par_type::GAL_ISB, "");
            if (i >= 0)
            {
                galileo_offset = param[i].value();
                //std::cout << "GAL Offset: " << sat << " " << galileo_offset << " " << gsatdata.epoch().str_hms() << std::endl;
            }
        }

        // BaiDou system time offset
        double beidou_offset = 0;
        if (gsatdata.gsys() == BDS)
        {
            int i;
            i = param.getParam(_site, par_type::BDS_ISB, "");
            if (i >= 0)
            {
                beidou_offset = param[i].value();
                //std::cout << "BDS ISB: " << sat << " " << param[i].value() << " " << gsatdata.epoch().str_hms() << std::endl;
            }
        }

        // QZSS system time offset
        double qzss_offset = 0;
        if (gsatdata.gsys() == QZS)
        {
            int i;
            i = param.getParam(_site, par_type::QZS_ISB, "");
            if (i >= 0)
                qzss_offset = param[i].value();
        }

        // ionosphere delay
        double ionDelay = 0.0;
        if (_observ != OBSCOMBIN::IONO_FREE)
        {
            ionDelay = ionoDelay(epoch, param, ell, gsatdata, gobs);
        }

        // Code bias
        double cbias = 0.0;
        GSYS sys = gsatdata.gsys();
        double fk = gsatdata.frequency(gobs.band());
        FREQ_SEQ freq = gnss_sys::band2freq(gsatdata.gsys(), gobs.band());

        GOBS g;
        if (gobs.attr() == ATTR)
            g = gsatdata.id_range(gobs.band()); // automatic GOBS selection
        else
            g = gobs.gobs(); // specific GOBS

        //std::cout << _site << " DCB: " << sat << " " << epo.str_ymdhms() << " C1W-C2W " << corrDCB/CLIGHT*1e9 << std::endl;
        //std::cout << "DCB: " << sat << " " << epo.str_ymdhms() << " C1C-C5Q " << corrDCB_15 << std::endl;
        if (gobs.is_code() && _observ != OBSCOMBIN::IONO_FREE)
        {
            // find DCB
            double corrDCB = 0.0;
            //    double corrDCB_15 = 0.0;
            if (_gallbias)
            {
                if (sys == GPS)
                {
                    if (g < 1000)
                    { // 3char signals used
                        if (_cbiaschar == CBIASCHAR::CHAR2)
                            corrDCB = _gallbias->get(epo, sat, P1, P2);
                        else
                            corrDCB = _gallbias->get(epo, sat, C1W, C2W);
                        //corrDCB_15 = _gallbias->get(epo, sat, C1W, C5Q);
                    }
                    else
                    { // 2char signals used
                        if (_cbiaschar == CBIASCHAR::CHAR3)
                            corrDCB = _gallbias->get(epo, sat, C1W, C2W);
                        else
                            corrDCB = _gallbias->get(epo, sat, P1, P2);
                    }
                }
                else if (sys == GAL)
                {
                    if (g < 1000)
                        corrDCB = _gallbias->get(epo, sat, C1X, C5X);
                    else
                        corrDCB = _gallbias->get(epo, sat, C1, C2);
                }
            }

            // Satellite GPS code bias P1-P2
            if (sys == GPS && (freq == FREQ_1 || freq == FREQ_2))
            {
                double f1 = G01_F;
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (G02_F * G02_F) / (G01_F * G01_F - G02_F * G02_F);
                cbias = -alfa * beta * corrDCB; // P1-P2
            }

            // // Satellite GPS code bias P1-P5
            // if(sys == GPS && freq == FREQ_3) {
            //   double beta = (G01_F*G01_F)/(G01_F*G01_F - G05_F*G05_F);
            //   cbias = - beta * corrDCB_15; // P1-P5
            // }

            // Satellite GAL code bias E1-E5a
            if (sys == GAL && (freq == FREQ_1 || freq == FREQ_2))
            {
                double f1 = E01_F;
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (E05_F * E05_F) / (E01_F * E01_F - E05_F * E05_F);
                cbias = -alfa * beta * corrDCB; // E1-E5
            }

            // Receiver GPS code bias P1-P2
            int idcb;
            idcb = param.getParam(_site, par_type::P1P2G_REC, "");
            if (idcb >= 0 && sys == GPS && (freq == FREQ_1 || freq == FREQ_2))
            {
                double f1 = G01_F;
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (G02_F * G02_F) / (G01_F * G01_F - G02_F * G02_F);
                cbias -= alfa * beta * param[idcb].value();
            }

            // Receiver GAL code bias E1-E5a
            idcb = param.getParam(_site, par_type::P1P2E_REC, "");
            if (idcb >= 0 && sys == GAL && (freq == FREQ_1 || freq == FREQ_2))
            {
                double f1 = E01_F;
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (E05_F * E05_F) / (E01_F * E01_F - E05_F * E05_F);
                cbias -= alfa * beta * param[idcb].value();
            }
        }

        // Return value
        return gsatdata.rho() +
               clkRec -
               gsatdata.clk() +
               trpDelay +
               ifcb +
               ifb +
               glonass_offset +
               galileo_offset +
               beidou_offset +
               qzss_offset +
               ionDelay +
               cbias;
    }

    double gnss_model_spp::cmpObsD(base_time &epoch, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs)
    {

        // Cartesian coordinates to ellipsodial coordinates
        Triple xyz, ell;
        Triple cRec, vRec;

        if (param.getCrdParam(_site, xyz) > 0)
        {
            cRec = xyz;
        }
        else
        {
            xyz = _grec->crd_arp(epoch);
            cRec = xyz;
        }
        xyz2ell(xyz, ell, false);

        int i = param.getParam(_site, par_type::VEL_X, "");
        int j = param.getParam(_site, par_type::VEL_Y, "");
        int k = param.getParam(_site, par_type::VEL_Z, "");
        int l = param.getParam(_site, par_type::CLK_RAT, "");
        vRec(0) = param[i].value();
        vRec(1) = param[j].value();
        vRec(2) = param[k].value();
        double dclk_Rec = param[l].value();
        double dclk = gsatdata.dclk();
        Triple satcrd = gsatdata.satcrd();
        Triple satvel = gsatdata.satvel();
        Vector cSat = satcrd;
        Vector vSat = satvel;
        Vector e = (cSat - cRec) / gsatdata.rho();

        //double res = dotproduct(e, vSat - vRec);
        //std::cout << std::fixed << setprecision(10) << res << std::endl;
        double res = e.dot(vSat - vRec) +
                     OMEGA / CLIGHT * (vSat(1) * cRec(0) + cSat(1) * vRec(0) - vSat(0) * cRec(1) - cSat(0) * vRec(1));
        //std::cout << std::fixed << setprecision(10) << res << std::endl;

        return res +
               dclk_Rec -
               dclk;
    }

    // Compute troposperic delay
    // -----------
    double gnss_model_spp::tropoDelay(base_time &epoch, base_allpar &param, Triple ell, gnss_data_sats &satdata)
    {

        if (_tropoModel == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Tropo Model setting is not correct. Default used! Check config.");
            _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
        }

        double ele = satdata.ele();

        double delay = 0.0;
        double zwd = 0.0;
        double zhd = 0.0;

        if (abs(ell[2]) > 1E4)
        {
            return 0.0;
        }

        int i;
        i = param.getParam(_site, par_type::TRP, "");
        if (i >= 0)
        {
            zwd = param[i].value();
            zhd = param[i].apriori();
        }
        else
        {
            if (_tropoModel != 0)
            {
                zwd = _tropoModel->getZWD(ell, epoch);
                zhd = _tropoModel->getZHD(ell, epoch);
            }
        }

        if (_tropo_mf == ZTDMPFUNC::GMF)
        {
            double gmfh, gmfw, dgmfh, dgmfw;
            gnss_model_gmf mf;
            mf.gmf(epoch.mjd(), ell[0], ell[1], ell[2], hwa_pi / 2.0 - ele,
                   gmfh, gmfw, dgmfh, dgmfw);
            //      delay = gmfh * _tropoModel->getZHD(ell, epoch) + gmfw * zwd;
            delay = gmfh * zhd + gmfw * zwd;

#ifdef DEBUG
            std::cout << epoch.str("EPOCH: %H:%M:%S") << std::endl
                 << std::fixed << setprecision(3);
            std::cout << "Ell:" << ell[0] << " " << ell[1] << " " << ell[2]
                 << " Hydrostatic part: " << zhd // _tropoModel->getZHD(ell, epoch)
                 << " Wet part: " << zwd
                 << " gmfh: " << gmfh
                 << " gmfw: " << gmfw
                 << " Delay: " << delay << std::endl
                 << std::endl;
            int ooo;
            cin >> ooo;
#endif
        }
        else if (_tropo_mf == ZTDMPFUNC::COSZ)
        {
            double mf = 1 / sin(ele);
            //      delay = 1/sin(ele) * _tropoModel->getZHD(ell, epoch) +1/sin(ele) * zwd;
            delay = mf * zhd + mf * zwd;
        }

        return delay;
    }

    // Compute ionospheic delay
    // -----------
    double gnss_model_spp::ionoDelay(base_time &epoch, base_allpar &param, Triple site_ell, gnss_data_sats &gsatdata, gnss_data_obs &gobs)
    {

        double iono_param = 0.0;
        double iono_apriori = 0.0;
        //double iono_model   = 0.0;
        double iono_delay = 0.0;

        double mf = 1 / sqrt(1.0 - pow(R_SPHERE / (R_SPHERE + 450000.0) * sin(hwa_pi / 2.0 - gsatdata.ele()), 2));

        // jdhuang : from G01_F
        double f1 = gsatdata.frequency(gnss_sys::band_priority(gsatdata.gsys(), FREQ_1));
        double fk = gsatdata.frequency(gobs.band());
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
        int i = param.getParam(_site, par_type::SION, gsatdata.sat());
        if (i >= 0)
        {
            iono_delay = alfa * param[i].value();
        }

        // ionosphere vertical delay paremeter
        i = param.getParam(_site, par_type::VION, gsatdata.sat());
        if (i >= 0)
        {
            iono_apriori = alfa * mf * param[i].apriori();
            iono_param = alfa * mf * param[i].value();
            iono_delay = iono_apriori + iono_param;
        }

#ifdef DEBUG
        std::cout << "IONO delay: " << gsatdata.sat() << " " << epoch.str_ymdhms() << " iono delay = " << iono_delay << std::endl;
        //    int ooo; cin >> ooo;
#endif

        return iono_delay;
    }

    double gnss_model_spp::isbCorrection(base_allpar &param, string &sat, string &rec, gnss_data_obs &gobs)
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
                //std::cout << "GLO Offset: " << sat << " " << glonass_offset << " " << gsatdata.epoch().str_hms() << std::endl;
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
                //std::cout << "GAL Offset: " << sat << " " << galileo_offset << " " << gsatdata.epoch().str_hms() << std::endl;
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
            int i = param.getParam(_site, par_type::QZS_ISB, "");
            if (i >= 0)
            {
                isb_offset = param[i].value();
            }
            break;
        }
        default:
            throw std::logic_error("can not support such sys �� " + gnss_sys::gsys2str(gsys));
        }

        return isb_offset;
    }

    void gnss_model_spp::reset_observ(OBSCOMBIN observ)
    {
        _observ = observ;
    }

    void gnss_model_spp::setrec(shared_ptr<gnss_data_obj> rec)
    {
        _grec = rec;
    }

    // Find maximal residual
    double gnss_model_spp::_maxres(const Vector &v, GSYS gs, bool phase, vector<gnss_data_sats> &data, vector<gnss_data_sats>::iterator &itDATA)
    {
        unsigned int inc = 2;

        if (v.rows() == data.size())
        { // code data only
            if (phase)
                return 0.0;
            inc = 1;
        }

        vector<gnss_data_sats>::iterator it;
        int ii = 1;
        if (phase)
            ii = 2;
        double maxres = 0.0;

        for (it = data.begin(); it != data.end(); it++)
        {
            if (it->gsys() != gs)
            {
                ii += inc;
                continue;
            }

            if (maxres == 0.0 || abs(v(ii)) > maxres)
            {
                maxres = abs(v(ii));
                itDATA = it;
            }

            ii += inc;
        }

        return maxres;
    }

    // Find maximal residual
    double gnss_model_spp::_maxres(bool phase, vector<gnss_data_sats> &data, vector<gnss_data_sats>::iterator &itDATA, RESIDTYPE res_type, GSYS gs)
    {

        vector<gnss_data_sats>::iterator it;

        double maxres = 0.0;

        for (it = data.begin(); it != data.end(); it++)
        {
            if (it->gsys() != gs && gs != GNS)
                continue;

            vector<double> res;
            if (phase)
                res = it->residuals(res_type, TYPE_L);
            else
                res = it->residuals(res_type, TYPE_C);

            for (auto itRES = res.begin(); itRES != res.end(); itRES++)
            {
                if (maxres == 0.0 || fabs(*itRES) > maxres)
                {
                    maxres = fabs(*itRES);
                    itDATA = it;
                }
            }
        }

        return maxres;
    }

    // check maximal residual
    bool gnss_model_spp::_check_outl(bool phase, double &maxres, vector<gnss_data_sats>::iterator &itData, vector<gnss_data_sats> &data)
    {
        map<GSYS, double> map_res;
        if (phase)
            map_res = _maxres_L;
        else
            map_res = _maxres_C;

        GSYS gs;
        if (itData != data.end())
            gs = itData->gsys();
        else
            return false;

        if ((maxres > map_res[gs] && _resid_type == RESIDTYPE::RES_ORIG) ||
            (maxres > _maxres_norm && _resid_type == RESIDTYPE::RES_NORM))
        {

            if (phase)
                _logOutl(true, itData->sat(), data.size(), maxres, itData->ele_deg(), itData->epoch(), _resid_type);
            else
                _logOutl(true, itData->sat(), data.size(), maxres, itData->ele_deg(), itData->epoch(), _resid_type);

            return true;
        }
        return false;
    }

    // check maximal residual
    bool gnss_model_spp::_check_outl(bool phase, double &maxresNORM, vector<gnss_data_sats>::iterator &itDataNORM,
                                  double &maxresORIG, vector<gnss_data_sats>::iterator &itDataORIG,
                                  vector<gnss_data_sats>::iterator &itDataErase, vector<gnss_data_sats> &data)
    {
        map<GSYS, double> map_res;
        if (phase)
            map_res = _maxres_L;
        else
            map_res = _maxres_C;

        GSYS gs;
        if (itDataORIG != data.end())
            gs = itDataORIG->gsys();
        else
            return false;

        if (_resid_type == RESIDTYPE::RES_ORIG)
        {
            if (maxresORIG > map_res[gs])
            {
                itDataErase = itDataORIG;
                if (phase)
                    _logOutl(true, itDataORIG->sat(), data.size(), maxresORIG, itDataORIG->ele_deg(), itDataORIG->epoch(), _resid_type);
                else
                    _logOutl(false, itDataORIG->sat(), data.size(), maxresORIG, itDataORIG->ele_deg(), itDataORIG->epoch(), _resid_type);
                return true;
            }
        }
        else if (_resid_type == RESIDTYPE::RES_NORM)
        {
            if (maxresNORM > _maxres_norm)
            {
                itDataErase = itDataNORM;
                if (phase)
                    _logOutl(true, itDataNORM->sat(), data.size(), maxresNORM, itDataNORM->ele_deg(), itDataNORM->epoch(), _resid_type);
                else
                    _logOutl(false, itDataNORM->sat(), data.size(), maxresNORM, itDataNORM->ele_deg(), itDataNORM->epoch(), _resid_type);
                return true;
            }
        }
        else if (_resid_type == RESIDTYPE::RES_ALL)
        {
            if (maxresORIG > map_res[gs] || maxresNORM > _maxres_norm)
            {
                if (itDataORIG == itDataNORM)
                {
                    itDataErase = itDataORIG;
                    if (phase)
                        _logOutl(true, itDataORIG->sat(), data.size(), maxresORIG, itDataORIG->ele_deg(), itDataORIG->epoch(), RESIDTYPE::RES_ORIG);
                    else
                        _logOutl(false, itDataORIG->sat(), data.size(), maxresORIG, itDataORIG->ele_deg(), itDataORIG->epoch(), RESIDTYPE::RES_ORIG);
                    return true;
                }
                else
                {
                    double ratioORIG = maxresORIG / map_res[gs];
                    double ratioNORM = maxresNORM / _maxres_norm;
                    if (ratioNORM >= ratioORIG)
                    {
                        itDataErase = itDataNORM;
                        if (phase)
                            _logOutl(true, itDataNORM->sat(), data.size(), maxresNORM, itDataNORM->ele_deg(), itDataNORM->epoch(), RESIDTYPE::RES_NORM);
                        else
                            _logOutl(false, itDataNORM->sat(), data.size(), maxresNORM, itDataNORM->ele_deg(), itDataNORM->epoch(), RESIDTYPE::RES_NORM);
                        return true;
                    }
                    else
                    {
                        itDataErase = itDataORIG;
                        if (phase)
                            _logOutl(true, itDataORIG->sat(), data.size(), maxresORIG, itDataORIG->ele_deg(), itDataORIG->epoch(), RESIDTYPE::RES_ORIG);
                        else
                            _logOutl(false, itDataORIG->sat(), data.size(), maxresORIG, itDataORIG->ele_deg(), itDataORIG->epoch(), RESIDTYPE::RES_ORIG);
                        return true;
                    }
                }
            }
        }

        return false;
    }

    // logging outlier
    void gnss_model_spp::_logOutl(bool phase, string prn, int data_size, double maxres, double ele, base_time epo, RESIDTYPE resid_type)
    {
        string obsType = "";
        string resType = "";
        if (phase)
            obsType = "phase";
        else
            obsType = "range";

        if (resid_type == RESIDTYPE::RES_NORM)
            resType = "Norm residual";
        if (resid_type == RESIDTYPE::RES_ORIG)
            resType = "Orig residual";
        if (resid_type == RESIDTYPE::RES_ALL)
            resType = "All residual";

        std::ostringstream os;
        os << _site << " outlier (" << resType << ": " << obsType << ") " << prn
           << " size:" << std::fixed << setw(2) << data_size
           << " v: " << std::fixed << setw(16) << right << setprecision(3) << maxres
           << " ele: " << std::fixed << setw(6) << setprecision(2) << ele
           << " " << epo.str_hms();
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, os.str());
    }

    // devide multi-freq residuals vector into single-freq
    vector<Vector> gnss_model_spp::_devide_res(const Vector &v_orig)
    {
        vector<Vector> vec;

        unsigned int k = 1;
        if (_observ == OBSCOMBIN::RAW_DOUBLE)
            k = 2;

        if (k == 1)
        {
            vec.push_back(v_orig);
            return vec;
        }

        Vector v_L1(v_orig.rows() / k);
        Vector v_L2(v_orig.rows() / k);

        int i = 0;
        int j = 0;
        while (i < v_orig.rows() - 1)
        {
            v_L1(j) = v_orig(i);
            v_L2(j) = v_orig(i + 1);
            j += 1;
            i += k;
        }

        vec.push_back(v_L1);
        vec.push_back(v_L2);

        return vec;
    }

} // namespace
