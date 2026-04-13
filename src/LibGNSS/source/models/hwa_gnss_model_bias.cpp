#include "hwa_gnss_model_Bias.h"
#include "hwa_base_string.h"
#include "hwa_gnss_model_gmf.h"
#include "hwa_gnss_model_tropoblind.h"
#include "hwa_set_gbase.h"

using namespace hwa_set;
using namespace std;

namespace hwa_gnss
{
    gnss_model_bias::gnss_model_bias(set_base *setting)
    {
        // std::set the setting pointer
        if (nullptr == setting)
        {
            spdlog::critical("your std::set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = setting;
        }

        _frequency = dynamic_cast<set_gproc *>(setting)->frequency();
        _crd_est = dynamic_cast<set_gproc *>(setting)->crd_est();
        _mf_ztd = dynamic_cast<set_gproc *>(setting)->tropo_mf();
        _mf_grd = dynamic_cast<set_gproc *>(setting)->grad_mf();
        _trpModStr = dynamic_cast<set_gproc *>(setting)->tropo_model();
        _ion_model = dynamic_cast<set_gproc *>(setting)->ion_model();
        _weight = dynamic_cast<set_gproc *>(setting)->weighting();
        _observ = dynamic_cast<set_gproc *>(setting)->obs_combin();

        if (_trpModStr == TROPMODEL::SAASTAMOINEN)
            _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
        else if (_trpModStr == TROPMODEL::DAVIS)
            _tropoModel = std::make_shared<gnss_model_tropo_davis>();
        else if (_trpModStr == TROPMODEL::HOPFIELD)
            _tropoModel = std::make_shared<gnss_model_tropo_hopf>();
        else if (_trpModStr == TROPMODEL::MOPS)
            _tropoModel = std::make_shared<gnss_model_blindtropos>(_spdlog);

        _sigCodeGPS = dynamic_cast<set_gnss *>(setting)->sigma_C(GPS);
        _sigCodeGLO = dynamic_cast<set_gnss *>(setting)->sigma_C(GLO);
        _sigCodeGAL = dynamic_cast<set_gnss *>(setting)->sigma_C(GAL);
        _sigCodeBDS = dynamic_cast<set_gnss *>(setting)->sigma_C(BDS);
        _sigCodeQZS = dynamic_cast<set_gnss *>(setting)->sigma_C(QZS);
        _sigPhaseGPS = dynamic_cast<set_gnss *>(setting)->sigma_L(GPS);
        _sigPhaseGLO = dynamic_cast<set_gnss *>(setting)->sigma_L(GLO);
        _sigPhaseGAL = dynamic_cast<set_gnss *>(setting)->sigma_L(GAL);
        _sigPhaseBDS = dynamic_cast<set_gnss *>(setting)->sigma_L(BDS);
        _sigPhaseQZS = dynamic_cast<set_gnss *>(setting)->sigma_L(QZS);

        //for LEO processing by zhangwei
        _sigCodeGPSLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(GPS);
        _sigCodeGLOLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(GLO);
        _sigCodeGALLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(GAL);
        _sigCodeBDSLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(BDS);
        _sigCodeQZSLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(QZS);
        _sigPhaseGPSLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(GPS);
        _sigPhaseGLOLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(GLO);
        _sigPhaseGALLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(GAL);
        _sigPhaseBDSLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(BDS);
        _sigPhaseQZSLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(QZS);

        _band_index[GPS] = dynamic_cast<set_gnss *>(setting)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(setting)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(setting)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(setting)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(setting)->band_index(QZS);

        _freq_index[GPS] = dynamic_cast<set_gnss *>(setting)->freq_index(GPS);
        _freq_index[GAL] = dynamic_cast<set_gnss *>(setting)->freq_index(GAL);
        _freq_index[GLO] = dynamic_cast<set_gnss *>(setting)->freq_index(GLO);
        _freq_index[BDS] = dynamic_cast<set_gnss *>(setting)->freq_index(BDS);
        _freq_index[QZS] = dynamic_cast<set_gnss *>(setting)->freq_index(QZS);

        _meanpole_model = dynamic_cast<set_gproc *>(setting)->mean_pole_model();
    }

    gnss_model_bias::gnss_model_bias(base_log spdlog, set_base *setting)
    {
        // std::set spdlog
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }

        // std::set the setting pointer
        if (nullptr == setting)
        {
            spdlog::critical("your std::set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = setting;
        }

        _frequency = dynamic_cast<set_gproc *>(setting)->frequency();
        _crd_est = dynamic_cast<set_gproc *>(setting)->crd_est();
        _mf_ztd = dynamic_cast<set_gproc *>(setting)->tropo_mf();
        _mf_grd = dynamic_cast<set_gproc *>(setting)->grad_mf();
        _trpModStr = dynamic_cast<set_gproc *>(setting)->tropo_model();
        _ion_model = dynamic_cast<set_gproc *>(setting)->ion_model();
        _weight = dynamic_cast<set_gproc *>(setting)->weighting();
        _observ = dynamic_cast<set_gproc *>(setting)->obs_combin();

        if (_trpModStr == TROPMODEL::SAASTAMOINEN)
            _tropoModel = std::make_shared<gnss_model_tropo_SAAST>();
        else if (_trpModStr == TROPMODEL::DAVIS)
            _tropoModel = std::make_shared<gnss_model_tropo_davis>();
        else if (_trpModStr == TROPMODEL::HOPFIELD)
            _tropoModel = std::make_shared<gnss_model_tropo_hopf>();
        else if (_trpModStr == TROPMODEL::MOPS)
            _tropoModel = std::make_shared<gnss_model_blindtropos>(_spdlog);

        _sigCodeGPS = dynamic_cast<set_gnss *>(setting)->sigma_C(GPS);
        _sigCodeGLO = dynamic_cast<set_gnss *>(setting)->sigma_C(GLO);
        _sigCodeGAL = dynamic_cast<set_gnss *>(setting)->sigma_C(GAL);
        _sigCodeBDS = dynamic_cast<set_gnss *>(setting)->sigma_C(BDS);
        _sigCodeQZS = dynamic_cast<set_gnss *>(setting)->sigma_C(QZS);
        _sigPhaseGPS = dynamic_cast<set_gnss *>(setting)->sigma_L(GPS);
        _sigPhaseGLO = dynamic_cast<set_gnss *>(setting)->sigma_L(GLO);
        _sigPhaseGAL = dynamic_cast<set_gnss *>(setting)->sigma_L(GAL);
        _sigPhaseBDS = dynamic_cast<set_gnss *>(setting)->sigma_L(BDS);
        _sigPhaseQZS = dynamic_cast<set_gnss *>(setting)->sigma_L(QZS);

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGPS ", format("%16.8f", _sigCodeGPS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGLO ", format("%16.8f", _sigCodeGLO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGAL ", format("%16.8f", _sigCodeGAL));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeBDS ", format("%16.8f", _sigCodeBDS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeQZS ", format("%16.8f", _sigCodeQZS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGPS", format("%16.8f", _sigPhaseGPS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGLO", format("%16.8f", _sigPhaseGLO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGAL", format("%16.8f", _sigPhaseGAL));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseBDS", format("%16.8f", _sigPhaseBDS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseQZS", format("%16.8f", _sigPhaseQZS));

        //for LEO processing by zhangwei
        _sigCodeGPSLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(GPS);
        _sigCodeGLOLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(GLO);
        _sigCodeGALLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(GAL);
        _sigCodeBDSLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(BDS);
        _sigCodeQZSLEO = dynamic_cast<set_gnss *>(setting)->sigma_C_LEO(QZS);
        _sigPhaseGPSLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(GPS);
        _sigPhaseGLOLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(GLO);
        _sigPhaseGALLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(GAL);
        _sigPhaseBDSLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(BDS);
        _sigPhaseQZSLEO = dynamic_cast<set_gnss *>(setting)->sigma_L_LEO(QZS);

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGPSLEO  ", format("%16.8f", _sigCodeGPSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGLOLEO  ", format("%16.8f", _sigCodeGLOLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGALLEO  ", format("%16.8f", _sigCodeGALLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeBDSLEO  ", format("%16.8f", _sigCodeBDSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeQZSLEO  ", format("%16.8f", _sigCodeQZSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGPSLEO ", format("%16.8f", _sigPhaseGPSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGLOLEO ", format("%16.8f", _sigPhaseGLOLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGALLEO ", format("%16.8f", _sigPhaseGALLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseBDSLEO ", format("%16.8f", _sigPhaseBDSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseQZSLEO ", format("%16.8f", _sigPhaseQZSLEO));

        _band_index[GPS] = dynamic_cast<set_gnss *>(setting)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(setting)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(setting)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(setting)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(setting)->band_index(QZS);

        _freq_index[GPS] = dynamic_cast<set_gnss *>(setting)->freq_index(GPS);
        _freq_index[GAL] = dynamic_cast<set_gnss *>(setting)->freq_index(GAL);
        _freq_index[GLO] = dynamic_cast<set_gnss *>(setting)->freq_index(GLO);
        _freq_index[BDS] = dynamic_cast<set_gnss *>(setting)->freq_index(BDS);
        _freq_index[QZS] = dynamic_cast<set_gnss *>(setting)->freq_index(QZS);

        _meanpole_model = dynamic_cast<set_gproc *>(setting)->mean_pole_model();
    }

    gnss_model_bias::gnss_model_bias()
    {
    }

    gnss_model_bias::~gnss_model_bias()
    {
    }

	bool gnss_model_bias::get_omc_obs_all(const base_time& crt_epo, gnss_data_sats& obsdata, base_allpar& pars, gnss_data_obs& gobs, double& omc)
	{
		return _omc_obs_all(crt_epo, obsdata, pars, gobs, omc);
	}

    bool gnss_model_bias::_wgt_obs_all(const base_data::ID_TYPE &obj_type, gnss_data_obs &gobs1, gnss_data_sats &obsdata, const double &factorP, double &wgt)
    {
        GSYS gsys = obsdata.gsys();
        std::string gsat = obsdata.sat();

        GOBSTYPE type = gobs1.type();
        if (type != TYPE_C &&
            type != TYPE_P &&
            type != TYPE_L)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " type should be TYPE_C/TYPE_P/TYPE_L");
            return false;
        }

        double factor = factorP;
        double sigRange = 0.0;
        double sigPhase = 0.0;

        // jdhuang : fix the bug
        if (gsys == GSYS::BDS)
            factor = 2.0;
        if (gsys == GSYS::BDS && gnss_sys::bds_geo(gsat))
            factor = 5.0;

        // get the sys sigRange
        if (type == TYPE_C || type == TYPE_P)
        {
            if (obj_type == base_data::REC) //add if by zhangwei
            {
                switch (gsys)
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
                    return false;
                }
            }
            else if (obj_type == base_data::REC_LEO)
            {
                switch (gsys)
                {
                case GPS:
                    sigRange = _sigCodeGPSLEO;
                    break;
                case GLO:
                    sigRange = _sigCodeGLOLEO;
                    break;
                case GAL:
                    sigRange = _sigCodeGALLEO;
                    break;
                case BDS:
                    sigRange = _sigCodeBDSLEO;
                    break;
                case QZS:
                    sigRange = _sigCodeQZSLEO;
                    break;
                default:
                    sigRange = 0.0;
                    return false;
                }
            }
            else
            {
                sigRange = 0.0;
                return false;
            }
        }
        else if (type == TYPE_L)
        {
            if (obj_type == base_data::REC) //add if by zhangwei
            {
                switch (gsys)
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
                    return false;
                }
            }
            else if (obj_type == base_data::REC_LEO)
                switch (gsys)
                {
                case GPS:
                    sigPhase = _sigPhaseGPSLEO;
                    break;
                case GLO:
                    sigPhase = _sigPhaseGLOLEO;
                    break;
                case GAL:
                    sigPhase = _sigPhaseGALLEO;
                    break;
                case BDS:
                    sigPhase = _sigPhaseBDSLEO;
                    break;
                case QZS:
                    sigPhase = _sigPhaseQZSLEO;
                    break;
                default:
                    sigPhase = 0.0;
                    return false;
                }
            else
            {
                sigPhase = 0.0;
                return false;
            }
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "no obs type");
            return false;
        }

        if (obj_type == base_data::REC)
        {
            double leo_ele = obsdata.ele_leo();
            double leo_ele_deg = obsdata.ele_leo_deg();
            double sin_leo_ele = sin(leo_ele);
            double snr = obsdata.getobs(pl2snr(gobs1.gobs()));          // if snr==0, there are some bugs.
            double SSI = floor(snr / 5) <= 9 ? floor(snr / 5) : 9;
            SSI = SSI >= 0 ? SSI : 9;           // when the snr is null, then std::set SSI=9.
            // get the _weight factor
            switch (_weight)
            {
            case OBSWEIGHT::DEF_OBS_WEIGHT:
                std::cerr << "gspplsq: WeightObs (default) should not happened!\n";
                break;
            case OBSWEIGHT::EQUAL:
                factor *= 1;
                break;
            case OBSWEIGHT::SINEL:
                factor *= 1.0 / 2.0 / sin(leo_ele);
                break;
            case OBSWEIGHT::SINEL2:
                factor *= 1.0 / 2.0 / pow(sin_leo_ele, 2);
                break;
            case OBSWEIGHT::SINEL4:
                factor *= 1.0 / 2.0 / pow(sin_leo_ele, 4);
                break;
            case OBSWEIGHT::PARTELE:
                factor = (leo_ele_deg <= 30.0) ? factor * (1.0 / 2.0 / sin_leo_ele) : factor;
                break;
            case OBSWEIGHT::SNR:
                factor *= sqrt(134.02 * pow(10, -(snr / 17.91)));
                break;
            case OBSWEIGHT::SNRELE:
                factor = (leo_ele_deg <= 30.0) ? factor * (1.0 / 2.0 / sin_leo_ele) : factor;
                factor *= SSI / 9;
                break;
            default:
                std::cerr << "gspplsq: we can't deal with this WeightObs method!";
                return false;
            }
        }


        // get the combine obs factor
        // TODO finish other weight method
        auto b1 = gobs1.band();
        if (type == TYPE_C || type == TYPE_P)
        {
            sigRange = sigRange /** 3*/;
        }
        else if (type == TYPE_L)
        {
            sigPhase = sigPhase * obsdata.wavelength(b1) /** 3*/;
        }
        else
        {
            sigRange = 0.0;
            sigPhase = 0.0;
        }

        if (sigPhase == 0.0 && sigRange == 0.0)
            return false;

        int obsLevel = obsdata.getobsLevelFlag(gobs1.gobs());

        switch (type)
        {
        case TYPE_L:
            if (sigPhase == 0.0)
                return false;
            wgt = 1.0 / pow(factor * sigPhase, 2);
            if (obsLevel == 0)
                wgt *= 1e-4;
            if (obsLevel == 1)
                wgt *= 1e-3;
            if (obsLevel == 2)
                wgt *= 1e-2;
#ifdef DEBUG_GLO
            std::cout << std::fixed << std::setprecision(5) << std::setw(20) << wgt << std::endl;
#endif
            break;
        case TYPE_C:
        case TYPE_P:
            if (sigRange == 0.0)
                return false;
            wgt = 1.0 / pow(factor * sigRange, 2);
            if (obsdata.getoutliers(gobs1.gobs()) >= 1)
                wgt *= 0.2;
            if (obsLevel == 0)
                wgt *= 1e-4;
            if (obsLevel == 1)
                wgt *= 1e-3;
            if (obsLevel == 2)
                wgt *= 1e-2;
#ifdef DEBUG_GLO
            std::cout << std::fixed << std::setprecision(5) << std::setw(20) << wgt << std::endl;
#endif
            break;
        default:
            return false;
        }

        if (obsdata.site() == "COMA"/*|| obsdata.site() == "COMC"*/)
            wgt *= 0.16;
        return true;
    }

    bool gnss_model_bias::_omc_obs_all(const base_time &crt_epo, gnss_data_sats &obsdata, base_allpar &pars, gnss_data_obs &gobs, double &omc)
    {

        if (!gobs.is_code() && !gobs.is_phase())
        {
            throw std::logic_error("only support code and phase observation.");
        }

        double obs = gobs.is_code() ? obsdata.obs_C(gobs) : obsdata.obs_L(gobs);

        // jdhuang : if obs = 0.0, return false
        if (double_eq(obs, 0.0))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "obs type is empty, the obs is zero!");
            return false;
        }

        base_time epo = crt_epo;
        std::string sat_name = obsdata.sat();
        std::string site_name = obsdata.site();
        double ModelObs = cmpObs(epo, sat_name, site_name, pars, obsdata, gobs);
        if (ModelObs < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "Compute ModelObs error!");
            return false; //glfeng
        }

        double Amb = 0.0;
        omc = (obs - ModelObs - Amb);

        return true;
    }

    bool gnss_model_bias::_Partial_basic(const base_time &epoch, gnss_data_sats &obsdata, const gnss_data_obs &gobs, const base_par &par, double &partial)
    {
        partial = 0.0;

        switch (par.parType)
        {
        case par_type::CRD_X:
            if (obsdata.site() == par.site)
            {
                partial = (par.value() - obsdata.satcrd()[0]) / obsdata.rho();
            }
            break;
        case par_type::CRD_Y:
            if (obsdata.site() == par.site)
            {
                partial = (par.value() - obsdata.satcrd()[1]) / obsdata.rho();
            }
            break;
        case par_type::CRD_Z:
            if (obsdata.site() == par.site)
            {
                partial = (par.value() - obsdata.satcrd()[2]) / obsdata.rho();
            }
            break;
        case par_type::CLK:
            if (obsdata.site() == par.site)
            {
                partial = 1.0 - obsdata.drate();
            }
            break;
        case par_type::CLK_G:
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::GPS)
            {
                partial = 1.0 - obsdata.drate();
            }
            break;
        case par_type::CLK_R:
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::GLO)
            {
                partial = 1.0 - obsdata.drate();
            }
            break;
        case par_type::CLK_C:
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::BDS)
            {
                partial = 1.0 - obsdata.drate();
            }
            break;
        case par_type::CLK_E:
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::GAL)
            {
                partial = 1.0 - obsdata.drate();
            }
            break;
        case par_type::CLK_J:
            if (obsdata.site() == par.site && obsdata.gsys() == GSYS::QZS)
            {
                partial = 1.0 - obsdata.drate();
            }
            break;
        case par_type::IFCB_F3:
            if (gnss_sys::band2freq(obsdata.gsys(), gobs.band()) == FREQ_3 && par.prn == obsdata.sat())
            {
                partial = -1.0;
            }
            break;
        case par_type::IFCB_F4:
            if (gnss_sys::band2freq(obsdata.gsys(), gobs.band()) == FREQ_4 && par.prn == obsdata.sat())
            {
                partial = -1.0;
            }
            break;
        case par_type::IFCB_F5:
            if (gnss_sys::band2freq(obsdata.gsys(), gobs.band()) == FREQ_5 && par.prn == obsdata.sat())
            {
                partial = -1.0;
            }
            break;
        case par_type::IFB_C3:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.prn == obsdata.sat() && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_C4:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.prn == obsdata.sat() && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_C5:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.prn == obsdata.sat() && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::SAT_IFB_C3:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.prn == obsdata.sat())
            {
                partial = 1.0;
            }
            break;
        case par_type::SAT_IFB_C4:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.prn == obsdata.sat())
            {
                partial = 1.0;
            }
            break;
        case par_type::SAT_IFB_C5:
            if (_freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.prn == obsdata.sat())
            {
                partial = 1.0;
            }
            break;
        case par_type::GPS_REC_IFB_C3:
            if (obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::GPS_REC_IFB_C4:
            if (obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::GPS_REC_IFB_C5:
            if (obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::GAL_REC_IFB_C3:
            if (obsdata.gsys() == GAL && _observ == OBSCOMBIN::RAW_ALL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::GAL_REC_IFB_C4:
            if (obsdata.gsys() == GAL && _observ == OBSCOMBIN::RAW_ALL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::GAL_REC_IFB_C5:
            if (obsdata.gsys() == GAL && _observ == OBSCOMBIN::RAW_ALL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && par.site == obsdata.site())
            {
                partial = 1.0;
            }
            break;
        case par_type::CLK_SAT:
            if (obsdata.sat() == par.prn)
            {
                partial = -1.0;
            }
            break;
        case par_type::TRP:
            if (obsdata.site() == par.site)
            {
                double mfw, dmfw, mfh, dmfh;
                mfw = dmfw = mfh = dmfh = 0.0;
                Triple ell(0.0, 0.0, 0.0);
                xyz2ell(_trs_rec_crd, ell, false);
                _getmf(par, obsdata, ell, epoch, mfw, mfh, dmfw, dmfh);
                partial = mfw;
            }
            break;
        case par_type::GRD_N:
            if (obsdata.site() == par.site)
            {
                double mfw, dmfw, mfh, dmfh;
                mfw = dmfw = mfh = dmfh = 0.0;
                Triple ell(0.0, 0.0, 0.0);
                xyz2ell(_trs_rec_crd, ell, false);
                _getmf(par, obsdata, ell, epoch, mfw, mfh, dmfw, dmfh);
                if (_mf_grd == GRDMPFUNC::CHEN_HERRING)
                {
                    double sinel = sin(obsdata.ele());
                    double tanel = tan(obsdata.ele());
                    double cosaz = cos(obsdata.azi());
                    partial = (1.0 / (sinel * tanel + 0.0032)) * cosaz;
                }
                else if (_mf_grd == GRDMPFUNC::TILTING)
                {
                    double cosaz = cos(obsdata.azi());
                    partial = dmfw * cosaz;
                }
                else if (_mf_grd == GRDMPFUNC::BAR_SEVER)
                {
                    double tanel = tan(obsdata.ele());
                    double cosaz = cos(obsdata.azi());
                    partial = mfw * (1.0 / tanel) * cosaz;
                }
            }
            break;
        case par_type::GRD_E:
            if (obsdata.site() == par.site)
            {
                double mfw, dmfw, mfh, dmfh;
                mfw = dmfw = mfh = dmfh = 0.0;
                Triple ell(0.0, 0.0, 0.0);
                xyz2ell(_trs_rec_crd, ell, false);
                _getmf(par, obsdata, ell, epoch, mfw, mfh, dmfw, dmfh);
                if (_mf_grd == GRDMPFUNC::CHEN_HERRING)
                {
                    double sinel = sin(obsdata.ele());
                    double tanel = tan(obsdata.ele());
                    double sinaz = sin(obsdata.azi());
                    partial = (1.0 / (sinel * tanel + 0.0032)) * sinaz;
                }
                else if (_mf_grd == GRDMPFUNC::TILTING)
                {
                    double sinaz = sin(obsdata.azi());
                    partial = dmfw * sinaz;
                }
                else if (_mf_grd == GRDMPFUNC::BAR_SEVER)
                {
                    double tanel = tan(obsdata.ele());
                    double sinaz = sin(obsdata.azi());
                    partial = mfw * (1 / tanel) * sinaz;
                }
            }
            break;
        case par_type::SION:
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
                partial = alfa;
            }
            break;
        case par_type::VION:
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
                partial = alfa * mf;
            }
            break;
        case par_type::P1P2G_REC:
            if (obsdata.site() == par.site)
            {
                double f1 = G01_F;
                double fk = obsdata.frequency(gobs.band());
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (G02_F * G02_F) / (G01_F * G01_F - G02_F * G02_F);
                FREQ_SEQ freq = gnss_sys::band2freq(obsdata.gsys(), gobs.band());
                if (obsdata.gsys() == GPS && gobs.is_code() && (freq == FREQ_1 || freq == FREQ_2))
                {
                    partial = -alfa * beta;
                }
            }
            break;
        case par_type::P1P2E_REC:
            if (obsdata.site() == par.site)
            {
                double f1 = E01_F;
                double fk = obsdata.frequency(gobs.band());
                double alfa = (f1 * f1) / (fk * fk);
                double beta = (E05_F * E05_F) / (E01_F * E01_F - E05_F * E05_F);
                FREQ_SEQ freq = gnss_sys::band2freq(obsdata.gsys(), gobs.band());
                if (obsdata.gsys() == GAL && gobs.is_code() && (freq == FREQ_1 || freq == FREQ_2))
                {
                    partial = -alfa * beta;
                }
            }
            break;
        case par_type::GLO_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == GLO)
            {
                partial = 1.0;
            }
            break;
        case par_type::GLO_ifcb:
            if (obsdata.site() == par.site && !gobs.is_phase() && obsdata.gsys() == GLO && par.prn == obsdata.sat())
            {
                partial = 1.0;
            }
            break;
        case par_type::GLO_IFPB:
            if (obsdata.site() == par.site && gobs.is_phase() && obsdata.gsys() == GLO && par.prn == obsdata.sat())
            {
                partial = 1.0;
            }
            break;
        case par_type::GLO_IFB:
            if (obsdata.site() == par.site && obsdata.gsys() == GLO && par.prn == obsdata.sat() && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::GAL_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL)
            {
                partial = 1.0;
            }
            break;
        case par_type::BDS_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS)
            {
                partial = 1.0;
            }
            break;
        case par_type::QZS_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS)
            {
                partial = 1.0;
            }
            break;
        case par_type::LEO_ISB:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && obsdata.sat().substr(0, 1) != "G")
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_QZS:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_GPS:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_GAL:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_GAL_2:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_GAL_3:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_BDS:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_3 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_BDS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_4 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::IFB_BDS_3:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && _freq_index[obsdata.gsys()][gobs.band()] == FREQ_5 && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::RCB_GPS_1:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::RCB_GPS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == GPS && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                partial = (f1 * f1) / (fk * fk) - 1;
            }
            break;
        case par_type::RCB_BDS_1:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::RCB_BDS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == BDS && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                partial = (f1 * f1) / (fk * fk) - 1;
            }
            break;
        case par_type::RCB_GAL_1:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::RCB_GAL_2:
            if (obsdata.site() == par.site && obsdata.gsys() == GAL && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                partial = (f1 * f1) / (fk * fk) - 1;
            }
            break;
        case par_type::RCB_QZS_1:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS && gobs.is_code())
            {
                partial = 1.0;
            }
            break;
        case par_type::RCB_QZS_2:
            if (obsdata.site() == par.site && obsdata.gsys() == QZS && gobs.is_code())
            {
                auto gsys = obsdata.gsys();
                double f1 = obsdata.frequency(_band_index[gsys][FREQ_1]);
                double fk = obsdata.frequency(gobs.band());
                partial = (f1 * f1) / (fk * fk) - 1;
            }
            break;
        default:
            return false;
        }
        return true;
    }

    void gnss_model_bias::_getmf(const base_par &par, gnss_data_sats &satData, const Triple &crd, const base_time &epoch, double &mfw, double &mfh, double &dmfw, double &dmfh)
    {
        if (par.parType != par_type::TRP && par.parType != par_type::GRD_N && par.parType != par_type::GRD_E)
            return;

        double ele = satData.ele();

        if (_mf_ztd == ZTDMPFUNC::COSZ)
        {
            mfw = mfh = 1.0 / sin(ele);
        }
        else if (_mf_ztd == ZTDMPFUNC::GMF)
        {
            gnss_model_gmf mf;
            mf.gmf(epoch.mjd(), crd[0], crd[1], crd[2], hwa_pi / 2.0 - ele, mfh, mfw, dmfh, dmfw);
        }
        else
            std::cerr << "ZTD mapPing function is not std::set up correctly!!!" << std::endl;
    }
}