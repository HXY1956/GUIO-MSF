#include "hwa_base_eigendef.h"
#include "hwa_base_timesync.h"
#include "hwa_gnss_proc_pvtflt.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_gnss_coder_bnccorr.h"
#include "hwa_gnss_coder_bncobs.h"
#include "hwa_gnss_proc_Spplsq.h"
#include "hwa_gnss_model_precisebiasgpp.h"
#include "hwa_gnss_proc_qualitycontrol.h"
#include "hwa_gnss_data_interpol.h"
#include "hwa_base_log.h"

using namespace std;
using namespace hwa_base;

hwa_gnss::gnss_proc_pvtflt::gnss_proc_pvtflt(std::string mark, std::string mark_base, set_base *gset,  base_all_proc *allproc)
    : gnss_proc_spp(mark, gset),
      gnss_proc_pppflt(mark, gset),
      _fix_mode(FIX_MODE::NO),
      _isFirstFix(true),
      _amb_state(false),
      _site_base(mark_base),
      _gModel_base(nullptr),
      _pos_constrain(false),
      _allproc(allproc),
      _gquality_control(gset, nullptr)
{

    _vel = Triple(0, 0, 0);
    _Qx_vel.resize(4);
    _Qx_vel.setZero();
    _dclkStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_dclk());
    _velStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_vel());
    _fix_mode = dynamic_cast<set_amb *>(_set)->fix_mode();
    _upd_mode = dynamic_cast<set_amb *>(_set)->upd_mode();
    _max_res_norm = dynamic_cast<set_gproc *>(_set)->max_res_norm();
    _sd_sat = dynamic_cast<set_gproc *>(_set)->sd_sat();
    _minsat = dynamic_cast<set_gproc *>(_set)->minsat();
    _realtime = dynamic_cast<set_gproc *>(_set)->realtime();
    _isBase = false;
    if (!_site_base.empty())
        _isBase = true;
    if (_isBase)
        _rtkinit();

    if (!_pos_kin)
    {
        xyz_standard = dynamic_cast<set_rec *>(gset)->get_crd_xyz(_site);
        neu_sum_nepo = std::make_pair(0, Triple(0.0, 0.0, 0.0));
    }
    //freq_index and band_index
    _band_index[GPS] = dynamic_cast<set_gnss *>(gset)->band_index(GPS);
    _band_index[GAL] = dynamic_cast<set_gnss *>(gset)->band_index(GAL);
    _band_index[GLO] = dynamic_cast<set_gnss *>(gset)->band_index(GLO);
    _band_index[BDS] = dynamic_cast<set_gnss *>(gset)->band_index(BDS);
    _band_index[QZS] = dynamic_cast<set_gnss *>(gset)->band_index(QZS);

    _freq_index[GPS] = dynamic_cast<set_gnss *>(gset)->freq_index(GPS);
    _freq_index[GAL] = dynamic_cast<set_gnss *>(gset)->freq_index(GAL);
    _freq_index[GLO] = dynamic_cast<set_gnss *>(gset)->freq_index(GLO);
    _freq_index[BDS] = dynamic_cast<set_gnss *>(gset)->freq_index(BDS);
    _freq_index[QZS] = dynamic_cast<set_gnss *>(gset)->freq_index(QZS);

    _gobs = dynamic_cast<gnss_all_obs *>((*_allproc)[base_data::ALLOBS]);
    _gnav = dynamic_cast<gnss_all_prec *>((*_allproc)[base_data::GRP_EPHEM]);
    _gionex_GIM = dynamic_cast<gnss_data_ionex *>((*_allproc)[base_data::IONEX]);
    _gupd = dynamic_cast<gnss_data_upd *>((*_allproc)[base_data::UPD]);
    //_allaug = dynamic_cast<gnss_data_aug*>((*_allproc)[base_data::AUG]);

    this->setOTL(dynamic_cast<gnss_all_otl *>((*_allproc)[base_data::ALLOTL]));
    this->setOBJ(dynamic_cast<gnss_all_obj *>((*_allproc)[base_data::ALLOBJ]));
    this->setDCB(dynamic_cast<gnss_all_bias *>((*_allproc)[base_data::ALLBIAS]));
    this->setFCB(dynamic_cast<gnss_all_bias *>((*_allproc)[base_data::ALLBIAS]));

    // init AMB update mode
    _frequency = dynamic_cast<set_gproc *>(gset)->frequency();
    _slip_model = dynamic_cast<set_gproc *>(gset)->slip_model();
    _turbo_liteMode = dynamic_cast<set_turboedit *>(gset)->liteMode();
    if (_slip_model == SLIPMODEL::TURBO_EDIT)
    {
        _gturboedit = std::make_shared<gnss_proc_exeturboedit>(gset, _spdlog, _allproc);
        if (!_turbo_liteMode)
        {
            _gturboedit->ProcessBatch(_site, _gobs->beg_obs(_site), _gobs->end_obs(_site), _sampling);
            switch (_frequency)
            {
            case 1:
            case 2:
                _slip12 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr));
                break;
            case 3:
                _slip12 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr));
                _slip13 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr, 3));
                break;
            case 4:
            case 5:
                _slip12 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr));
                _slip13 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr, 3));
                _slip14 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr, 4));
                _slip15 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, nullptr, 5));
                break;
            default:
                throw std::logic_error("you should std::set the frequency in xml as 1/2/3/4/5.");
                break;
            }
        }
    }
    else // DEFAULT
    {
        _gpre = std::make_shared<gnss_proc_preproc>(_gobs, gset);
        _gpre->spdlog(_spdlog);
        _gpre->setNav(_gnav);
    }

    if (_observ == OBSCOMBIN::RAW_SINGLE) // extern ionosphere constraint for single-frequency PPP
    {
        if (_gionex_GIM)
            _iono_constraint = true;
        else
            _iono_constraint = false;

        _epoch_cout = 0;
    }

    // amb fix
    if (_ambfix)
    {
        delete _ambfix;
        _ambfix = nullptr;
    }
    if (_fix_mode != FIX_MODE::NO)
    {
        _ambfix = new gnss_ambiguity(_site, _set);
    }

    std::shared_ptr<gnss_model_bias> precise_bias(new gnss_model_precise_biasgpp(_allproc, gset));

    if (!_isBase)
    {
        switch (_observ)
        {
        case OBSCOMBIN::RAW_ALL:
        case OBSCOMBIN::RAW_DOUBLE:
        case OBSCOMBIN::RAW_MIX:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_all(gset, precise_bias, _allproc));
            if (!_turbo_liteMode)
            {
                _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_all(_slip12, _slip13, _slip14, _slip15, _band_index));
            }
            break;
        case OBSCOMBIN::IF_P1:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_mix(gset, precise_bias, _allproc));
            break;
        case OBSCOMBIN::IONO_FREE:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_if(gset, precise_bias, _allproc));
            if (!_turbo_liteMode)
            {
                if (_frequency == 2)
                    _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_if(_slip12, _band_index));
                if (_frequency == 3)
                    _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_if_1X(_slip12, _slip13, _band_index));
            }
            break;
        default:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_all(gset, precise_bias, _allproc));
            if (!_turbo_liteMode)
            {
                _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_all(_slip12, _slip13, _slip14, _slip15, _band_index));
            }
            break;
        }
    }
    else
    {
        _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_dd(gset, precise_bias, _allproc));
        dynamic_cast<gnss_model_comb_dd *>(&(*_base_model))->set_observ(_observ);
        dynamic_cast<gnss_model_comb_dd *>(&(*_base_model))->set_site(_site, _site_base);
    }
    _isClient = dynamic_cast<set_gen *>(gset)->isClient();
    if (_isClient)
        _gInterpol = std::make_shared<gnss_data_interpol>(gset, allproc, _spdlog); //lvhb added in 20201105
    _wl_Upd_time = base_time(WL_IDENTIFY);
    _ewl_Upd_time = base_time(EWL_IDENTIFY);
    _nppmodel = dynamic_cast<set_gproc *>(_set)->npp_model();
    _isCompAug = dynamic_cast<set_npp *>(_set)->comp_aug();
    _isCorObs = dynamic_cast<set_npp*>(_set)->cor_obs();
    _reset_amb_ppprtk = dynamic_cast<set_npp *>(_set)->reset_amb_ppprtk();
    _receiverType = dynamic_cast<set_gproc *>(_set)->get_receiverType();
    _obs_level = dynamic_cast<set_npp *>(_set)->obs_level();
}
hwa_gnss::gnss_proc_pvtflt::gnss_proc_pvtflt(std::string mark, std::string mark_base, set_base *gset, base_log spdlog,  base_all_proc *allproc)
    : gnss_proc_spp(mark, gset, spdlog),
      gnss_proc_pppflt(mark, gset, spdlog),
      _fix_mode(FIX_MODE::NO),
      _isFirstFix(true),
      _amb_state(false),
      _site_base(mark_base),
      _gModel_base(nullptr),
      _pos_constrain(false),
      _allproc(allproc),
      _gquality_control(gset, nullptr)
{
    _gquality_control.spdlog(spdlog);
    _vel = Triple(0, 0, 0);
    _Qx_vel.resize(4);
    _Qx_vel.setZero();
    _dclkStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_dclk());
    _velStoModel = new gnss_model_white_noise(dynamic_cast<set_flt *>(_set)->noise_vel());
    _fix_mode = dynamic_cast<set_amb *>(_set)->fix_mode();
    _upd_mode = dynamic_cast<set_amb *>(_set)->upd_mode();
    _max_res_norm = dynamic_cast<set_gproc *>(_set)->max_res_norm();
    _sd_sat = dynamic_cast<set_gproc *>(_set)->sd_sat();
    _minsat = dynamic_cast<set_gproc *>(_set)->minsat();
    _realtime = dynamic_cast<set_gproc *>(_set)->realtime();
    _isBase = false;
    if (!_site_base.empty())
        _isBase = true;
    if (_isBase)
        _rtkinit();
    if (!_pos_kin)
    {
        xyz_standard = dynamic_cast<set_rec *>(gset)->get_crd_xyz(_site);
        neu_sum_nepo = std::make_pair(0, Triple(0.0, 0.0, 0.0));
    }
    _band_index[GPS] = dynamic_cast<set_gnss *>(gset)->band_index(GPS);
    _band_index[GAL] = dynamic_cast<set_gnss *>(gset)->band_index(GAL);
    _band_index[GLO] = dynamic_cast<set_gnss *>(gset)->band_index(GLO);
    _band_index[BDS] = dynamic_cast<set_gnss *>(gset)->band_index(BDS);
    _band_index[QZS] = dynamic_cast<set_gnss *>(gset)->band_index(QZS);
    _freq_index[GPS] = dynamic_cast<set_gnss *>(gset)->freq_index(GPS);
    _freq_index[GAL] = dynamic_cast<set_gnss *>(gset)->freq_index(GAL);
    _freq_index[GLO] = dynamic_cast<set_gnss *>(gset)->freq_index(GLO);
    _freq_index[BDS] = dynamic_cast<set_gnss *>(gset)->freq_index(BDS);
    _freq_index[QZS] = dynamic_cast<set_gnss *>(gset)->freq_index(QZS);
    _gobs = dynamic_cast<gnss_all_obs *>((*_allproc)[base_data::ALLOBS]);
    _gnav = dynamic_cast<gnss_all_prec *>((*_allproc)[base_data::GRP_EPHEM]);
    _gionex_GIM = dynamic_cast<gnss_data_ionex *>((*_allproc)[base_data::IONEX]);
    _gupd = dynamic_cast<gnss_data_upd *>((*_allproc)[base_data::UPD]);
    this->setOTL(dynamic_cast<gnss_all_otl *>((*_allproc)[base_data::ALLOTL]));
    this->setOBJ(dynamic_cast<gnss_all_obj *>((*_allproc)[base_data::ALLOBJ]));
    this->setDCB(dynamic_cast<gnss_all_bias *>((*_allproc)[base_data::ALLBIAS]));
    this->setFCB(dynamic_cast<gnss_all_bias *>((*_allproc)[base_data::ALLBIAS]));
    _frequency = dynamic_cast<set_gproc *>(gset)->frequency();
    _slip_model = dynamic_cast<set_gproc *>(gset)->slip_model();
    _turbo_liteMode = dynamic_cast<set_turboedit *>(gset)->liteMode();
    if (_slip_model == SLIPMODEL::TURBO_EDIT)
    {
        _gturboedit = std::make_shared<gnss_proc_exeturboedit>(gset, _spdlog, _allproc, mark);
        if (!_turbo_liteMode)
        {
            _gturboedit->ProcessBatch(_site, _gobs->beg_obs(_site), _gobs->end_obs(_site), _sampling);
            switch (_frequency)
            {
            case 1:
            case 2:
                _slip12 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog));
                break;
            case 3:
                _slip12 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog));
                _slip13 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog, 3));
                break;
            case 4:
            case 5:
                _slip12 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog));
                _slip13 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog, 3));
                _slip14 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog, 4));
                _slip15 = std::shared_ptr<gnss_data_cycleslip>(new gnss_data_turboedit(gset, _spdlog, 5));
                break;
            default:
                throw std::logic_error("you should std::set the frequency in xml as 1/2/3/4/5.");
                break;
            }
        }
    }
    else // DEFAULT
    {
        _gpre = std::make_shared<gnss_proc_preproc>(_gobs, gset);
        _gpre->spdlog(_spdlog);
        _gpre->setNav(_gnav);
    }
    if (_observ == OBSCOMBIN::RAW_SINGLE) // extern ionosphere constraint for single-frequency PPP
    {
        if (_gionex_GIM)
            _iono_constraint = true;
        else
            _iono_constraint = false;
        _epoch_cout = 0;
    }
    if (_ambfix)
    {
        delete _ambfix;
        _ambfix = nullptr;
    }
    if (_fix_mode != FIX_MODE::NO)
    {
        _ambfix = new gnss_ambiguity(_site, _set);
    }
    std::shared_ptr<gnss_model_bias> precise_bias(new gnss_model_precise_biasgpp(_allproc, _spdlog, gset));
    if (!_isBase)
    {
        switch (_observ)
        {
        case OBSCOMBIN::RAW_ALL:
        case OBSCOMBIN::RAW_DOUBLE:
        case OBSCOMBIN::RAW_MIX:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_all(gset, _spdlog, precise_bias, _allproc));
            if (!_turbo_liteMode)
            {
                _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_all(_slip12, _slip13, _slip14, _slip15, _band_index));
            }
            break;
        case OBSCOMBIN::IF_P1:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_mix(gset, _spdlog, precise_bias, _allproc));
            break;
        case OBSCOMBIN::IONO_FREE:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_if(gset, _spdlog, precise_bias, _allproc));
            if (!_turbo_liteMode)
            {
                if (_frequency == 2)
                    _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_if(_slip12, _band_index));
                if (_frequency == 3)
                    _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_if_1X(_slip12, _slip13, _band_index));
            }
            break;
        default:
            _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_all(gset, _spdlog, precise_bias, _allproc));
            if (!_turbo_liteMode)
            {
                _update_AMB = std::shared_ptr<gnss_proc_update_par>(new gnss_proc_update_par_all(_slip12, _slip13, _slip14, _slip15, _band_index));
            }
            break;
        }
    }
    else
    {
        _base_model = std::shared_ptr<gnss_model_base>(new gnss_model_comb_dd(gset, _spdlog, precise_bias, _allproc));
        dynamic_cast<gnss_model_comb_dd *>(&(*_base_model))->set_observ(_observ);
        dynamic_cast<gnss_model_comb_dd *>(&(*_base_model))->set_site(_site, _site_base);
    }

    // use to estimate bias parameter(ISB/IFB/..) as white noise
    _isClient = dynamic_cast<set_npp *>(gset)->isClient();
    if (_isClient)
        _gInterpol = std::make_shared<gnss_data_interpol>(gset, allproc, _site, _spdlog); //lvhb added in 20201105

    ///< real time
    _wl_Upd_time = base_time(WL_IDENTIFY);
    _ewl_Upd_time = base_time(EWL_IDENTIFY);
    _nppmodel = dynamic_cast<set_npp *>(_set)->npp_model();
    _isCompAug = dynamic_cast<set_npp *>(_set)->comp_aug();
    _isCorObs = dynamic_cast<set_npp*>(_set)->cor_obs();
    _reset_amb_ppprtk = dynamic_cast<set_npp *>(_set)->reset_amb_ppprtk();
    _receiverType = dynamic_cast<set_gproc *>(_set)->get_receiverType();
    _obs_level = dynamic_cast<set_npp *>(_set)->obs_level();
    _motion_model = dynamic_cast<set_gproc *>(_set)->motion_model();
}

hwa_gnss::gnss_proc_pvtflt::~gnss_proc_pvtflt()
{
    if (_ambfix)
    {
        delete _ambfix;
        _ambfix = nullptr;
    }
    if (_gModel_base)
    {
        delete _gModel_base;
        _gModel_base = nullptr;
    }
}

int hwa_gnss::gnss_proc_pvtflt::_addObsD(gnss_data_sats &satdata, unsigned int &iobs, base_allpar &param, Triple &XYZ, Matrix &A, Vector &l, Diag &P)
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

    _getgobs(satdata.sat(), TYPE_D, b1, gobs1);
    _getgobs(satdata.sat(), TYPE_D, b2, gobs2);
    _getgobs(satdata.sat(), TYPE_D, b3, gobs3);
    _getgobs(satdata.sat(), TYPE_D, b4, gobs4);
    _getgobs(satdata.sat(), TYPE_D, b5, gobs5);

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

    double weight_coef = _weightObs(satdata, gobs1);
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

    double sigDoppler = 0.0;
    switch (gs)
    {
    case GPS:
        sigDoppler = _sigDopplerGPS;
        break;
    case GLO:
        sigDoppler = _sigDopplerGLO;
        break;
    case GAL:
        sigDoppler = _sigDopplerGAL;
        break;
    case BDS:
        sigDoppler = _sigDopplerBDS;
        break;
    case QZS:
        sigDoppler = _sigDopplerQZS;
        break;
    default:
        sigDoppler = 0.0;
    }

    //Create reduced measurements (prefit residuals)
    double modObsD = 0.0;
    // int n = obsnum;
    for (int i = 0; i < obsnum; i++)
    {
        double Di = 0.0;
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
        if (double_eq(modObsD, 0.0) || double_eq(Di, 0.0))
            continue;
        l(iobs + i) = -Di - modObsD;
    }

    // Create weight matrix
    for (int i = 0; i < obsnum; i++)
    {
        P(iobs + i) = weight_coef * (1 / (sigDoppler * sigDoppler));
    }
    // Create first design matrix
    double A_conf;
    int i = param.getParam(_site, par_type::VEL_X, "");
    int j = param.getParam(_site, par_type::VEL_Y, "");
    int k = param.getParam(_site, par_type::VEL_Z, "");
    Triple groundVel(param[i].value(), param[j].value(), param[k].value());

    for (int i = 0; i < obsnum; i++)
    {
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
            if (double_eq(A_conf, 0.0) || double_eq(l(iobs + i), 0))
                continue;
            A(iobs + i, ipar) = A_conf;
        }
    }

    iobs = iobs + obsnum;

    return 1;
}

bool hwa_gnss::gnss_proc_pvtflt::getexeturboedit(std::shared_ptr<gnss_proc_exeturboedit>& tb)
{
    if (_gturboedit)
    {
        tb = _gturboedit;
        return true;
    }
    else
    {
        tb = nullptr;
        return false;
    }
        
}

int hwa_gnss::gnss_proc_pvtflt::_rtkinit()
{

    /* setting initiating */
    _param->delAllParam();
    _Qx.setZero();
    int ipar = 0;

    // Add coordinates parameters
    if (_crd_est != CONSTRPAR::FIX)
    {
        _param->addParam(base_par(_site, par_type::CRD_X, ++ipar, ""));
        _param->addParam(base_par(_site, par_type::CRD_Y, ++ipar, ""));
        _param->addParam(base_par(_site, par_type::CRD_Z, ++ipar, ""));
    }

    // Add tropospheric wet delay parameter
    if (_tropo_est)
    {
        //rover
        base_par trp_rover(_site, par_type::TRP, ++ipar, "");
        trp_rover.setMF(_ztd_mf);
        _param->addParam(trp_rover);
        //base
        base_par trp_base(_site_base, par_type::TRP, ++ipar, "");
        trp_base.setMF(_ztd_mf);
        _param->addParam(trp_base);
    }

    // Filling init parameter covariance matrix
    _Qx.resize(_param->parNumber());
    _Qx.setZero();
    double crdInit = _sig_init_crd;
    double ztdInit = _sig_init_ztd;
    for (unsigned int i = 0; i < _param->parNumber(); i++)
    {
        if (_param->operator[](i).parType == par_type::CRD_X)
            _Qx.matrixW()(i, i) = crdInit * crdInit;
        else if (_param->operator[](i).parType == par_type::CRD_Y)
            _Qx.matrixW()(i, i) = crdInit * crdInit;
        else if (_param->operator[](i).parType == par_type::CRD_Z)
            _Qx.matrixW()(i, i) = crdInit * crdInit;
        else if (_param->operator[](i).parType == par_type::TRP)
            _Qx.matrixW()(i, i) = ztdInit * ztdInit;
    }

    return 1;
}

int hwa_gnss::gnss_proc_pvtflt::_setCrd()
{
    //save base coordinate to grec
    std::shared_ptr<gnss_data_obj> grec_base = _gallobj->obj(_site_base);
    /* rover/reference std::fixed position */
    if (grec_base == nullptr)
        return -1;

    _gModel_base = new gnss_model_ppp(_site_base, _spdlog, _set);
    _gModel_base->reset_observ(_observ);
    _gModel_base->setOBJ(_gallobj);

    //get base coordinate
    base_pos basepos = dynamic_cast<set_gproc *>(_set)->basepos();
    base_time beg = dynamic_cast<set_gen *>(_set)->beg();
    base_time end = dynamic_cast<set_gen *>(_set)->end();

    if (basepos == base_pos::SPP)
    { // from spp
        std::shared_ptr<gnss_proc_spplsq> spplsq = std::make_shared<gnss_proc_spplsq>(_site_base, _set, _gallobj, _spdlog);

        spplsq->setDAT(_gobs, _gnav);
        if (spplsq->processBatch(beg, end) <= 0)
        {
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, _site_base + "spplsq failed");
            }
            return -1;
        }
        Triple crd = spplsq->getAvergeCrd();
        grec_base->crd(crd, Triple(1, 1, 1), beg, end, true);
    }
    else if (basepos == base_pos::CFILE)
    {
        // std::set the value [read from xml/snx/rnxo]
        auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(_site_base);
        grec_base->crd(crd, Triple(0.01, 0.01, 0.01), beg, end, true);
    }

    //rover
    if (_crd_est == CONSTRPAR::FIX && !_pos_kin)
    {
        // std::set the value [read from xml/snx/rnxo]
        auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(_site);
        _grec->crd(crd, Triple(0.01, 0.01, 0.01), beg, end, true);
    }

    return 1;
}

int hwa_gnss::gnss_proc_pvtflt::_setCrd(const base_time & t)
{
    //save base coordinate to grec
    std::shared_ptr<gnss_data_obj> grec_base = _gallobj->obj(_site_base);
    /* rover/reference std::fixed position */
    if (grec_base == nullptr)
        return -1;

    _gModel_base = new gnss_model_ppp(_site_base, _spdlog, _set);
    _gModel_base->reset_observ(_observ);
    _gModel_base->setOBJ(_gallobj);

    //get base coordinate
    base_pos basepos = dynamic_cast<set_gproc *>(_set)->basepos();

    if (basepos == base_pos::SPP)
    { // from spp
        std::shared_ptr<gnss_proc_spplsq> spplsq = std::make_shared<gnss_proc_spplsq>(_site_base, _set, _gallobj, _spdlog);

        spplsq->setDAT(_gobs, _gnav);
        if (spplsq->processBatch(t, t) <= 0)
        {
            if (_spdlog)
            {
                SPDLOG_LOGGER_INFO(_spdlog, _site_base + "spplsq failed");
            }
            return -1;
        }
        Triple crd = spplsq->getAvergeCrd();
        grec_base->crd(crd, Triple(1, 1, 1), t, LAST_TIME, true);
    }
    else if (basepos == base_pos::CFILE)
    {
        // std::set the value [read from xml/snx/rnxo]
        auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(_site_base);
        grec_base->crd(crd, Triple(0.01, 0.01, 0.01), t, t, true);
    }

    //rover
    if (_crd_est == CONSTRPAR::FIX && !_pos_kin)
    {
        // std::set the value [read from xml/snx/rnxo]
        auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(_site);
        _grec->crd(crd, Triple(0.01, 0.01, 0.01), t, t, true);
    }

    return 1;
}

void hwa_gnss::gnss_proc_pvtflt::grec_clear()
{
    _gallobj->obj(_site)->clear_crd();
    _gallobj->obj(_site_base)->clear_crd();
}

/* detect valid zero-difference residuals*/
bool hwa_gnss::gnss_proc_pvtflt::_valid_residual(bool phase_process, std::string sat_name, enum FREQ_SEQ &f, std::map<std::string, std::map<FREQ_SEQ, std::pair<int, int>>> &index_l)
{
    try
    {
        /* if no phase observable, psudorange is also unusable */
        if (phase_process)
        {
            index_l.at(sat_name).at(_observ == OBSCOMBIN::IONO_FREE ? FREQ_1 : f).second;
        }
        else
        {
            index_l.at(sat_name).at(_observ == OBSCOMBIN::IONO_FREE ? FREQ_1 : f).second;
            index_l.at(sat_name).at(_observ == OBSCOMBIN::IONO_FREE ? FREQ_1 : f).first;
        }
    }
    catch (std::exception e)
    {
        return false;
    }
    return true;
}

int hwa_gnss::gnss_proc_pvtflt::_combineDD(Matrix &A, Symmetric &P, Vector &l)
{
    try
    {
        std::map<std::string, std::map<FREQ_SEQ, std::pair<int, int>>> index_l;
        for (int i = 0; i < _obs_index.size(); i++)
        {
            std::string sat = _obs_index[i].first;
            FREQ_SEQ f = _obs_index[i].second.first;
            GOBSTYPE obstype = _obs_index[i].second.second;
            if (index_l.find(sat) == index_l.end())
            {
                index_l.insert(std::make_pair(sat, std::map<FREQ_SEQ, std::pair<int, int>>()));
            }
            if (index_l[sat].find(f) == index_l[sat].end())
            {
                index_l[sat].insert(std::make_pair(f, std::make_pair(-1, -1)));
            }
            if (obstype == TYPE_C)
                index_l[sat][f].first = i;
            else if (obstype == GOBSTYPE::TYPE_L)
                index_l[sat][f].second = i;
        }

        _obs_index.clear();
        int iobs = 0;

        std::set<std::string> sysall = dynamic_cast<set_gen *>(_set)->sys();

        int nrows = A.rows();
        Matrix DD;
        DD.resize(nrows, nrows);
        DD.setZero();
        _sat_ref.clear();
        bool isSetRefSat = dynamic_cast<set_amb *>(_set)->isSetRefSat();

        bool isPhaseProcess = true;
        for (int obslevel = _obs_level; obslevel <= 3; obslevel++)
        {
            for (auto sys_iter = sysall.begin(); sys_iter != sysall.end(); sys_iter++)
            {
                enum GSYS sys = gnss_sys::str2gsys(*sys_iter);
                std::vector<GOBSBAND> band = dynamic_cast<set_gnss *>(_set)->band(sys);
                int nf = 5;
                if (band.size())
                    nf = band.size();
                if (_observ == OBSCOMBIN::IONO_FREE)
                    nf = 1;
                FREQ_SEQ f;
                std::string sat_ref;
                for (FREQ_SEQ freq = FREQ_1; freq <= 2 * nf; freq = (FREQ_SEQ)(freq + 1))
                {
                    if (freq <= nf)
                    { //phase equations
                        isPhaseProcess = true;
                        f = freq;
                    }
                    else
                    { //code equations
                        isPhaseProcess = false;
                        f = (FREQ_SEQ)(freq - nf);
                    }
                    if (f > _frequency)
                        continue; // modified by lvhb in 20201211
                    std::string sat;
                    gnss_data_sats obs_sat_ref;
                    enum GSYS gs;
                    if (!isSetRefSat || (_observ == OBSCOMBIN::RAW_MIX && !isPhaseProcess))
                        sat_ref.clear();
                    if (sat_ref.empty())
                    {
                        for (auto it = _data.begin(); it != _data.end(); it++)
                        {
                            std::string sat = it->sat();

                            gs = it->gsys();
                            if (gs == QZS)
                                gs = GPS;
                            if (gs != sys)
                                continue;
                            if ((gs == BDS) && gnss_sys::bds_geo(sat))
                                continue;
                            if (/*(_observ == RAW_ALL||_observ == IONO_FREE||_observ == RAW_DOUBLE)
                                 &&*/
                                !_reset_amb && !_reset_par && it->islip())
                                continue;

                            if (_isClient)
                            {
                                GOBSBAND b = _band_index[gs][f];
                                GOBS gobs = it->select_phase(b, true);
                                if (!isPhaseProcess)
                                    gobs = it->select_range(b, true);
                                if (it->getobsLevelFlag(gobs) != obslevel)
                                    continue;
                            }
                            /* select reference satellite,lvhb modified in 20200826 */
                            if (isSetRefSat && (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX))
                            {
                                FREQ_SEQ f1 = FREQ_1;
                                FREQ_SEQ f2 = FREQ_2;
                                FREQ_SEQ f3 = FREQ_3;
                                FREQ_SEQ f4 = FREQ_4;
                                FREQ_SEQ f5 = FREQ_5;
                                if (_valid_residual(isPhaseProcess, sat, f1, index_l) == false || (_frequency >= 2 && nf >= 2 && _valid_residual(isPhaseProcess, sat, f2, index_l) == false) || (_frequency >= 3 && nf >= 3 && _valid_residual(isPhaseProcess, sat, f3, index_l) == false) || (_frequency >= 4 && nf >= 4 && _valid_residual(isPhaseProcess, sat, f4, index_l) == false) || (_frequency >= 5 && nf >= 5 && _valid_residual(isPhaseProcess, sat, f5, index_l) == false))
                                    continue;
                            }
                            else if (_valid_residual(isPhaseProcess, sat, f, index_l) == false)
                                continue;

                            if (sat_ref.empty())
                            {
                                sat_ref = sat;
                                obs_sat_ref = *it;
                                continue;
                            }

                            if (it->ele_deg() >= obs_sat_ref.ele_deg())
                            {
                                sat_ref = sat;
                                obs_sat_ref = *it;
                            }
                        } //end select sat_ref
                    }
                    if (sat_ref.empty())
                        continue;
                    if (_ipSatRep[sys] != "" && sat_ref != _ipSatRep[sys])
                    {
                        std::cerr << "refsat bug" << std::endl;
                        continue;
                    }
                    if (freq == FREQ_1)
                        _sat_ref.insert(sat_ref);
                    //if (isPhaseProcess) _sat_refs[sys][f] = sat_ref;

                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + " " + sat_ref + ": is reference satellite!");

                    int index_ref;
                    if (isPhaseProcess)
                        index_ref = index_l[sat_ref][f].second;
                    else
                        index_ref = index_l[sat_ref][f].first;
                    /* make double difference */
                    for (auto it = _data.begin(); it != _data.end(); it++)
                    {
                        sat = it->sat();
                        if (sat == sat_ref)
                            continue;

                        gs = it->gsys();
                        if (gs == QZS)
                            gs = GPS;
                        if (gs != sys)
                            continue;

                        if (_isClient)
                        {
                            GOBSBAND b = _band_index[gs][f];
                            GOBS gobs = it->select_phase(b, true);
                            if (!isPhaseProcess)
                                gobs = it->select_range(b, true);
                            if (it->getobsLevelFlag(gobs) != obslevel)
                                continue;
                        }

                        if (_valid_residual(isPhaseProcess, sat, f, index_l) == false)
                            continue;
                        int index_sat;
                        if (isPhaseProcess)
                            index_sat = index_l[sat][f].second;
                        else
                            index_sat = index_l[sat][f].first;

                        DD(iobs, index_ref) = -1;
                        DD(iobs, index_sat) = 1;
                        GOBSTYPE obstype = TYPE_C;
                        if (isPhaseProcess)
                            obstype = GOBSTYPE::TYPE_L;
                        _obs_index.push_back(std::make_pair(it->sat(), std::make_pair(f, obstype)));
                        iobs++;
                    } //end sat
                }      //end f
            }          //end sys
        }

        if (iobs <= 0)
        {
            SPDLOG_LOGGER_INFO(_spdlog, _site + " and "+_site_base+" have no common satellite in one system!");
            _delPar(par_type::AMB_IF);
            _delPar(par_type::AMB_L1);
            _delPar(par_type::AMB_L2);
            _delPar(par_type::AMB_L3);
            _delPar(par_type::AMB_L4);
            _delPar(par_type::AMB_L5);
            _delPar(par_type::SION);
            _delPar(par_type::VION);
            return -1;
        }

        DD = DD.block(0, 0, iobs, DD.cols()).eval();

        A = DD * A;
        l = DD * l;

        P.matrixW() = (DD * P.matrixR().inverse() * DD.transpose()).inverse().eval();

        if (_sd_sat) //lvhb added in 20210517
        {
            int iclk = _param->getParam(_site, par_type::CLK, "");
            if (iclk >= 0)
            {
                for (int i = 0; i < iobs; i++)
                    A(i, iclk) = 0.0;
            }
        }

        return 1;
    }
    catch (...)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt_newmodelmodel", "generate double-differented equation failed!");
        return -1;
    }
}

int hwa_gnss::gnss_proc_pvtflt::_postRes(const Matrix &A, const Symmetric &P, const Vector &l, Matrix &pA, Symmetric &pP, Vector &pl)
{
    if (_sat_ref.empty())
        return -1;
    if (!_amb_state)
    {
        _post_A = A;
        _post_P = P;
        _post_l = l;
        return 0;
    }

    // index_l todo : sat,freq,std::pair<P,L>
    std::map<std::string, std::map<FREQ_SEQ, std::pair<int, int>>> index_l;
    for (int i = 0; i < _obs_index.size(); i++)
    {
        std::string sat = _obs_index[i].first;
        FREQ_SEQ f = _obs_index[i].second.first;
        GOBSTYPE obstype = _obs_index[i].second.second;
        if (index_l.find(sat) == index_l.end())
        {
            index_l.insert(std::make_pair(sat, std::map<FREQ_SEQ, std::pair<int, int>>()));
        }
        if (index_l[sat].find(f) == index_l[sat].end())
        {
            index_l[sat].insert(std::make_pair(f, std::make_pair(-1, -1)));
        }
        if (obstype == TYPE_C)
            index_l[sat][f].first = i;
        else if (obstype == GOBSTYPE::TYPE_L)
            index_l[sat][f].second = i;
    }

    int nobs = P.rows();
    int ncols = A.cols();
    pA.resize(nobs, A.cols());
    pA = A;
    pP = P;
    pl.resize(nobs);
    pl = l;
    Diag delP;
    delP.resize(nobs);
    delP.setZero();

    hwa_vector_amb_dd DD = _ambfix->getDD();
    int iobs = 0;
    std::vector<int> ind;
    for (auto it = 0; it < nobs; it++)
        ind.push_back(it);

    for (auto itref = _sat_ref.begin(); itref != _sat_ref.end(); itref++)
    {
        std::string satref = *itref;
        for (auto itdd = DD.begin(); itdd != DD.end(); itdd++)
        {
            if (itdd->isWlFixed && itdd->isNlFixed)
            {
                std::string sat1 = std::get<0>(itdd->ddSats[0]);
                std::string sat2 = std::get<0>(itdd->ddSats[1]);

                std::string crt_sat = sat1;
                if (crt_sat.substr(0, 1) != satref.substr(0, 1))
                    continue;
                if (sat1 != satref && sat2 != satref)
                    continue;
                if (sat1 == satref && sat2 != satref)
                {
                    crt_sat = sat2;
                    itdd->rlc *= -1.0;
                    itdd->rwl *= -1.0;
                    itdd->inl *= -1.0;
                }
                FREQ_SEQ f = FREQ_X;
                double dif = 0.0;
                if (_observ == OBSCOMBIN::IONO_FREE)
                {
                    gnss_data_obs_manager gnss(_spdlog, satref);
                    auto gs = gnss.gsys();
                    double lambda_1 = gnss.wavelength(_band_index[gs][FREQ_1]);
                    double lambda_2 = gnss.wavelength(_band_index[gs][FREQ_2]);

                    double wl = round(itdd->rwl);
                    double mw_coef = lambda_2 * SQR(lambda_1) / (SQR(lambda_2) - SQR(lambda_1));
                    dif = itdd->rlc - (itdd->inl * itdd->factor + wl * mw_coef);
                    f = FREQ_1;
                }
                else if (_observ == OBSCOMBIN::RAW_ALL)
                {
                    if (itdd->ambtype == "AMB_L1")
                        f = FREQ_1;
                    else if (itdd->ambtype == "AMB_L2")
                        f = FREQ_2;
                    else if (itdd->ambtype == "AMB_L3")
                        f = FREQ_3;
                    else if (itdd->ambtype == "AMB_L4")
                        f = FREQ_4;
                    else if (itdd->ambtype == "AMB_L5")
                        f = FREQ_5;
                    else
                        continue;

                    dif = itdd->rlc - itdd->inl * itdd->factor;
                }
                if (f != FREQ_X)
                {
                    auto index = index_l[crt_sat][f].second;
                    pl(index) -= dif;
                    // std::vector<int>::const_iterator it = find(ind.begin(), ind.end(), index);
                    //if (it != ind.end())ind.erase(it);
                }
            }
        } //end dd
    }      //end satref

    std::vector<int> ind1 = ind, ind2 = ind;
    int pobs = nobs - ind.size();
    pP.Matrix_rem(ind);
    Matrix_remR(pA, ind1);
    remR(pl, ind2);
    pA.block(0, 3, pobs, ncols - 3).setZero();  
    for (int i = 0; i < pobs; i++)
        pP.matrixW()(i, i) *= 10;

    return iobs;
}

int hwa_gnss::gnss_proc_pvtflt::_prepareData()
{

    int flag = 1;
    if (_isBase)
    {
        if (_preprocess(_site_base, _data_base) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + std::string("gpvtflt:  ") +  "Preparing data failed!");
            flag = -1;
        }
        _vBanc_base = _vBanc;
    }
    if (_preprocess(_site, _data) < 0)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _site + std::string("gpvtflt:  ") +  "Preparing data failed!");
        flag = -1;
    }
    if (_cntrep == 0)
    {
        Triple xyz = _vBanc.segment(0, 3);
        _gquality_control.processOneEpoch(_epoch, _site, xyz, _data);
        if (_isBase)
        {
            xyz = _vBanc_base.segment(0, 3);
            _gquality_control.processOneEpoch(_epoch, _site_base, xyz, _data_base);
        }
    }

    if (_isBase)
    {
        if ((_nSat = _selcomsat(_data_base, _data)) < _minsat)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "no common satellite");
            flag = -1;
        }
    }
    if (flag == -1)
    {
        _nSat = 0;
    }

    if (_data.size() < _minsat)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + base_type_conv::int2str(_data.size()) + " skipped (data.size < minsat)");
        return -1;
    }

    return flag;
}

int hwa_gnss::gnss_proc_pvtflt::_preprocess(const std::string &ssite, std::vector<gnss_data_sats> &sdata)
{
    Matrix BB;
    BB.resize(sdata.size(), 4);
    BB.setZero();
    int iobs = 0;
    std::vector<gnss_data_sats>::iterator iter = sdata.begin();

    _nSat = sdata.size(); // configured GNSS is considered (see erase in above swich)
    std::set<std::string> sat_rm = dynamic_cast<set_gen *>(_set)->sat_rm();
    while (iter != sdata.end())
    {
        // except sat from config file
        std::string satname = iter->sat();
        if (sat_rm.find(satname) != sat_rm.end())
        {
            iter = sdata.erase(iter);
            continue;
        }

        //check each satellite obs and crd, zzwu
        if (!_check_sat(ssite, &*iter, BB, iobs))
        {
            iter = sdata.erase(iter);
            continue;
        }
        else
        {
            ++iter;
        }
    } //end sdata

    if (sdata.size() < _minsat)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _epoch.str_ymdhms(ssite + " epoch ") + " skipped (Bancroft not calculated: " + base_type_conv::int2str(sdata.size()) + " < _minsat: " + base_type_conv::int2str(_minsat) + ")");
        //      std::cout << gnss_sys::gsys2str(_gnss) << " " << _epoch.str_ymdhms( _site + " epoch ") << " " << base_type_conv::int2str(_data.size()) <<  " skipped (Bancroft not calculated: _data.size < _minsat)" << std::endl;
        return -1;
    }


    BB = BB.block(0 ,0, iobs, BB.cols()).eval(); // delete zero rows

    if (BB.rows() < static_cast<int>(_minsat))
    {
        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _epoch.str_ymdhms(_site + " epoch ") + " skipped (Bancroft not calculated: BB.rows < _minsat)");
        //      std::cout << _epoch.str_ymdhms( _site + " epoch ") << base_type_conv::int2str(_data.size()) <<  " skipped (Bancroft not calculated: _data.size < _minsat)" << std::endl;
        return -1;
    }

    ////std::cout << " PVT crd: " << _epoch.str_hms() << " " << std::fixed << std::fixed << _vBanc(1) << " " << _vBanc(2) << " " << _vBanc(3) << std::endl;

    if (!_cmp_rec_crd(ssite, BB))
    {
        return -1;
    }

    //Compute sat elevation and rho
    iter = sdata.begin();
    while (iter != sdata.end())
    {
        if (!_cmp_sat_info(ssite, &*iter))
        {
            iter = sdata.erase(iter); // !!!! zveda iterator !!!!
            if (sdata.size() < _minsat)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, ssite + _epoch.str_ymdhms(" epoch ") + base_type_conv::int2str(sdata.size()) + " skipped (data.size < _rtk_set->minsat)");
                return -1;
            }
        }
        else
        {
            ++iter;
        }

    } //end sdata

    return 0;
}

void hwa_gnss::gnss_proc_pvtflt::_predict(const base_time& runEpoch)
{

    // Predict coordinates, clock and troposphere
    // state std::vector is the same, covariance matrix is predicted with noise
    double bl = 0.0;

    _cntrep++;

    if (!_isBase)
        _syncSys();

    if (!_initialized)
        _delPar(par_type::SION);

    if (_nSat == 0)
    {
        _delPar(par_type::AMB_IF);
        _delPar(par_type::AMB_L1);
        _delPar(par_type::AMB_L2);
        _delPar(par_type::AMB_L3);
        _delPar(par_type::AMB_L4);
        _delPar(par_type::AMB_L5);
        _delPar(par_type::SION);
        _delPar(par_type::VION);
    }

    // add/remove ionosphere delay
    //Actually,we only estimate sion/sdsion by using white noise here.    lvhb 202007
    _syncIono();

    if (_isBase && 1) //lvhb 20200822
    {
        if (_iono_est)
        {
            auto crd = _gallobj->obj(_site_base)->crd_arp(_epoch);
            Triple tmpcrd = _vBanc.segment(0, 3);
            Triple tmpell, tmpdxyz, tmpneu;
            xyz2ell(crd, tmpell, false);
            tmpdxyz = tmpcrd - crd;
            xyz2neu(tmpell, tmpdxyz, tmpneu);
            bl = tmpneu.norm();

            if (bl < 10000 && _pos_kin)
            {
                _delPar(par_type::VION);
                _delPar(par_type::SION);
            }
        }
    }
    // add/remove inter-frequency biases
    // LX changed IFB estimation
    //if (!_isBase) _syncIFB();
    if (!_isBase && _frequency >= 3)
        _syncIFB();

    _Noise.resize(_Qx.rows());
    _Noise.setZero();

    // Predict ambiguity
    // Add or remove ambiguity parameter and appropriate rows/columns covar. matrix
    if (_phase)
    {
        _syncAmb();
        //std::cout << runEpoch.sow() + runEpoch.dsec() << ": ";
        //for (unsigned int i = 0; i < _param->parNumber(); i++)
        //{
        //    if (_param->operator[](i).parType == par_type::AMB_IF ||
        //        _param->operator[](i).parType == par_type::AMB_L1 ||
        //        _param->operator[](i).parType == par_type::AMB_L2 ||
        //        _param->operator[](i).parType == par_type::AMB_L3 ||
        //        _param->operator[](i).parType == par_type::AMB_L4 ||
        //        _param->operator[](i).parType == par_type::AMB_L5)
        //        std::cout << _param->operator[](i).str_type() << " " << _param->operator[](i).value() << "; ";
        //}
        //std::cout << "\n";
    }
    _predictCrd();
    _predictClk();
    _predictBias();
    _predictIono(bl, runEpoch);
    _predictTropo();
    _predictAmb();


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
}

void hwa_gnss::gnss_proc_pvtflt::_delPar(const par_type par)
{
    // Remove params and appropriate rows/columns covar. matrix
    for (unsigned int i = 0; i <= _param->parNumber() - 1; i++)
    {
        if (_param->operator[](i).parType == par)
        {
            _Qx.Matrix_remRC(_param->operator[](i).index, _param->operator[](i).index);
            _param->delParam(i);
            _param->reIndex();
            i--;
        }
    }

    return;
}

int hwa_gnss::gnss_proc_pvtflt::_combineMW(gnss_data_sats &satdata)
{

    std::string sat = satdata.sat();
    double obs_intv = _sampling;
    base_time crt_time = _epoch;
    if (crt_time == _beg_time)
    {
        _MW[crt_time - obs_intv][sat][1] = 0.0;
        _MW[crt_time - obs_intv][sat][2] = 0.0;
        _MW[crt_time - obs_intv][sat][3] = 0.0;
        _MW[crt_time - obs_intv][sat][4] = 0.0;
        _MW[crt_time - obs_intv][sat][5] = 0.0; // ele
    }

    GOBSBAND b1, b2;
    if (_auto_band)
    { // automatic dual band selection -> for Anubis purpose
        std::set<GOBSBAND> bands = satdata.band_avail();
        auto itBAND = bands.begin();
        b1 = *itBAND;
        itBAND++;
        b2 = *itBAND;
    }
    else
    { // select fix defined band according the table
        std::vector<GOBSBAND> band = dynamic_cast<set_gnss *>(_set)->band(satdata.gsys());
        if (band.size())
        {
            b1 = band[0];
            b2 = band[1];
        }
        else
        {
            b1 = gnss_sys::band_priority(satdata.gsys(), FREQ_1);
            b2 = gnss_sys::band_priority(satdata.gsys(), FREQ_2);
        }
    }
    gnss_data_obs gobsP1 = gnss_data_obs(satdata.select_range(b1));
    gnss_data_obs gobsP2 = gnss_data_obs(satdata.select_range(b2));
    gnss_data_obs gobsL1 = gnss_data_obs(satdata.select_phase(b1));
    gnss_data_obs gobsL2 = gnss_data_obs(satdata.select_phase(b2));
    double mw_obs = satdata.MW_cycle(gobsL1, gobsL2, gobsP1, gobsP2);
    bool islip = (satdata.getlli(gobsL1.gobs()) >= 1 || satdata.getlli(gobsL2.gobs()) >= 1);

    //dcb correct  已经在观测之上修改了dcb
    if (_isBase)
    {
        gnss_data_sats satdata_base;
        for (int i = 0; i < _data_base.size(); i++)
        {
            if (_data_base[i].sat() == satdata.sat())
                satdata_base = _data_base[i];
        }
        if (satdata_base.site() == "")
            mw_obs = 0.0;
        else
        {
            gobsP1 = gnss_data_obs(satdata_base.select_range(b1));
            gobsP2 = gnss_data_obs(satdata_base.select_range(b2));
            gobsL1 = gnss_data_obs(satdata_base.select_phase(b1));
            gobsL2 = gnss_data_obs(satdata_base.select_phase(b2));
            double mw_obs_base = satdata_base.MW_cycle(gobsL1, gobsL2, gobsP1, gobsP2);

            if (double_eq(mw_obs, 0.0) || double_eq(mw_obs_base, 0.0))
                mw_obs = 0.0;
            else
                mw_obs -= mw_obs_base;

            if (!islip)
                islip = (satdata_base.getlli(gobsL1.gobs()) >= 1 || satdata_base.getlli(gobsL2.gobs()) >= 1);
        }
    }

    if (!double_eq(mw_obs, 0.0))
    {
        // calculate smooth MW
        base_time pre_time = crt_time - obs_intv;
        if (_MW.find(pre_time) != _MW.end() && _MW[pre_time].find(sat) != _MW[pre_time].end() && !double_eq(_MW[pre_time][sat][4], 0.0) && !islip)
        {
            _MW[crt_time][sat][1] = _MW[pre_time][sat][1] + 1;
            _MW[crt_time][sat][4] = mw_obs;
            if (!double_eq(_MW[crt_time][sat][1], 1.0) && _MW[crt_time][sat][1] > 1.0)
            {
                _MW[crt_time][sat][3] = _MW[pre_time][sat][3] * (_MW[crt_time][sat][1] - 2) /
                                            (_MW[crt_time][sat][1] - 1) +
                                        pow(mw_obs - _MW[pre_time][sat][2], 2) / _MW[crt_time][sat][1];
            }
            else
            {
                _MW[crt_time][sat][3] = _MW[pre_time][sat][3];
            }
            _MW[crt_time][sat][2] = _MW[pre_time][sat][2] + (mw_obs - _MW[pre_time][sat][2]) / _MW[crt_time][sat][1];
            _MW[crt_time][sat][5] = satdata.ele_deg();
        }
        // exclude mw begining ,it is different from last all
        else
        {
            _MW[crt_time][sat][1] = 1;
            _MW[crt_time][sat][4] = mw_obs;
            _MW[crt_time][sat][3] = 0.0;
            _MW[crt_time][sat][2] = mw_obs / _MW[crt_time][sat][1];
            _MW[crt_time][sat][5] = satdata.ele_deg();
        }
    }
    // exclude zero wl
    else
    {
        _MW[crt_time][sat][1] = 0.0;
        _MW[crt_time][sat][2] = 0.0;
        _MW[crt_time][sat][3] = 0.0;
        _MW[crt_time][sat][4] = 0.0;
        _MW[crt_time][sat][5] = 0.0;
        return -1;
    }

    return 1;
}

bool hwa_gnss::gnss_proc_pvtflt::_external_pos(const Triple &xyz_r, const Triple &rms)
{
    _extn_pos = xyz_r;
    _extn_rms = rms;
    _pos_constrain = true;
    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_pos_virtual_obs(const Triple &xyz_r, const Triple &rms, Matrix &A, Vector &l, Symmetric &P)
{
    try
    {
        int obs_num = A.rows(), par_num = _param->parNumber();
        int virtual_obs_num = 3;
        ResizeKeep(A, obs_num + virtual_obs_num, par_num);
        ResizeKeep(P.matrixW(), obs_num + virtual_obs_num, obs_num + virtual_obs_num);
        ResizeKeep(l ,obs_num + virtual_obs_num);
        int icrdx = _param->getParam(_site, par_type::CRD_X, "");
        int icrdy = _param->getParam(_site, par_type::CRD_Y, "");
        int icrdz = _param->getParam(_site, par_type::CRD_Z, "");
        A(obs_num, icrdx) = 1;
        P.matrixW()(obs_num, obs_num) = SQR(rms[0]);
        l(obs_num) = 0;
        A(obs_num + 1, icrdy) = 1;
        P.matrixW()(obs_num + 1, obs_num + 1) = SQR(rms[1]);
        l(obs_num + 1) = 0;
        A(obs_num + 2, icrdz) = 1;
        P.matrixW()(obs_num + 2, obs_num + 2) = SQR(rms[2]);
        l(obs_num + 2) = 0;
        return true;
    }
    catch (...)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt", "Add position virtual obs failed!");
        return false;
    }
}

int hwa_gnss::gnss_proc_pvtflt::_processEpochVel()
{

    double threshold = 0.0;

    Matrix A;
    Diag P;
    Vector l, dx;
    std::vector<gnss_data_sats>::iterator it;
    base_allpar param_vel;

    int ipar = 0;
    param_vel.addParam(base_par(_site, par_type::VEL_X, ++ipar, ""));
    param_vel.addParam(base_par(_site, par_type::VEL_Y, ++ipar, ""));
    param_vel.addParam(base_par(_site, par_type::VEL_Z, ++ipar, ""));
    param_vel.addParam(base_par(_site, par_type::CLK_RAT, ++ipar, ""));

    if (!_initialized || _Qx_vel(0, 0) == 0)
    {
        for (int i = 0; i < 3; i++)
        {
            _Qx_vel.matrixW()(i, i) = _sig_init_vel * _sig_init_vel;
        }
        _Qx_vel.matrixW()(3, 3) = _dclkStoModel->getQ() * _dclkStoModel->getQ();
    }
    else
    {
        for (int i = 0; i < param_vel.parNumber(); i++)
        {
            if (param_vel[i].parType == par_type::VEL_X)
            {
                param_vel[i].value(_vel[0]);
                _Qx_vel.matrixW()(i, i) += _velStoModel->getQ() * _velStoModel->getQ();
            }
            else if (param_vel[i].parType == par_type::VEL_Y)
            {
                param_vel[i].value(_vel[1]);
                _Qx_vel.matrixW()(i, i) += _velStoModel->getQ() * _velStoModel->getQ();
            }
            else if (param_vel[i].parType == par_type::VEL_Z)
            {
                param_vel[i].value(_vel[2]);
                _Qx_vel.matrixW()(i, i) += _velStoModel->getQ() * _velStoModel->getQ();
            }
            else if (param_vel[i].parType == par_type::CLK_RAT)
            {
                param_vel[i].value(0);
                _Qx_vel.matrixW()(i, i) = _dclkStoModel->getQ() * _dclkStoModel->getQ();
            }
        }
    }

    do
    {
        // define a number of measurements
        unsigned int nObs = _data.size();
        if (_doppler)
        {
            nObs *= 5;
        }

        unsigned int nPar = param_vel.parNumber();
        A.resize(nObs, nPar);
        A.setZero();
        l.resize(nObs);
        l.setZero();
        P.resize(nObs);
        P.setZero();
        dx.resize(nPar);
        dx.setZero();
        unsigned int iobs = 0;

        Triple groundXYZ;
        _param->getCrdParam(_site, groundXYZ);

        // Create matrices and std::vectors for estimation
        // ----------------------------------------------------------------------------------
        // loop over all measurement in current epoch

        for (it = _data.begin(); it != _data.end();)
        {
            if (_addObsD(*it, iobs, param_vel, groundXYZ, A, l, P) > 0)
            {
            }
            else
            {
                it = _data.erase(it);
                continue;
            } // just return next iterator

            ++it; // increase iterator if not erased
        }

        A = A.block(0, 0, iobs, A.cols()).eval();
        P.matrixW() = P.matrixR().block(0, 0, iobs, iobs).eval();
        l = l.segment(0, iobs).eval();

        _filter->update(A, P, l, dx, _Qx_vel);

        int freedom = A.rows() - A.cols();
        if (freedom < 1)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt", "No redundant observations!");
            //    _restore(QsavBP, XsavBP);
            //    return -1;
            freedom = 1;
        }

        int index = param_vel.getParam(_site, par_type::VEL_X, "");

        for (unsigned int iPar = 0; iPar < param_vel.parNumber(); iPar++)
        {
            param_vel[iPar].value(param_vel[iPar].value() + dx(param_vel[iPar].index));
        }
        threshold = sqrt(dx(index) * dx(index) + dx(index + 1) * dx(index + 1) + dx(index + 2) * dx(index + 2));
    } while (threshold > 1e-2);

    if (param_vel.getVelParam(_site, _vel) < 0)
        return -1;

    return 1;
}

int hwa_gnss::gnss_proc_pvtflt::_processEpoch(const base_time &runEpoch)
{

#ifdef DEBUG
    std::cout << "gpvtflt: Epoch - full data size: " << _data.size() << " "
         << runEpoch.str_hms() << std::endl;
#endif
    if (_grec == nullptr)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "No receiver settings available!!!");
        return -1;
    }

    _epoch = runEpoch;
    _amb_state = false;

    if (!_crd_xml_valid())
        _sig_init_crd = 100.0;

    Matrix A;
    Symmetric P;
    Vector l, dx;
    /*Vector v_orig, v_norm, v_test;*/
    Vector v_norm;
    Symmetric Qsav, QsavBP;
    /*std::vector<gnss_data_sats>::iterator it;*/
    base_allpar XsavBP;
    double vtpv;
    int nobs_total, npar_number;
    std::string outlier = "";
    _cntrep = 0; // number of iterations caused by outliers
    _crt_SNR.clear();
    _crt_ObsLevel.clear();

    // select obs for tb log (rover)
    if (_slip_model == SLIPMODEL::TURBO_EDIT && !_turbo_liteMode)
    {
        if (!this->_post_turbo_select_obs(runEpoch, _data))
            return -1;
    }

    do
    {
        _remove_sat(outlier);

        if (_prepareData() < 0)
        {
            if (_initialized)
            {
                _predict(runEpoch); //todo:add time gap
                    //            _reset_param();
                    //            _reset_ambig();b
            }
            return -1;
        }

        QsavBP = _Qx;
        XsavBP = *_param;
        _predict(runEpoch);
        //_prt_info(_epoch);

        _initialized = true;

        if (_data.size() < _minsat)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Not enough visible satellites!");
            _restore(QsavBP, XsavBP);
            return -1;
        }

        // define a number of measurements
        unsigned int nObs = _data.size();
        unsigned int mult = 1;
        if (_observ == OBSCOMBIN::RAW_DOUBLE)
        {
            mult = 2;
            nObs *= 2;
        }
        if (_observ == OBSCOMBIN::RAW_ALL /*|| _observ == OBSCOMBIN::RAW_MIX*/)
        {
            mult = 2;
            nObs *= 5;
        } // reservation for 5 freq - not used raws will be removed
        if (_phase)
        {
            mult *= 2;
            nObs *= 2;
        } // code + phase
        if (_observ == OBSCOMBIN::IF_P1)
        {
            mult = 2;
            nObs *= 3;
        } // IF + P1(不增加参数，多一倍观测值）
        // if (_doppler) { mult; nObs *= 2; }
        // nObs *= 2;

        /*G01 P1 P2 P3... L1 L2L3*/
        unsigned int nPar = _param->parNumber();
        unsigned int iobs = 1;

        _frqNum.clear();
        _obs_index.clear();
        if (_isBase)
        {
            dynamic_cast<gnss_model_comb_dd *>(&(*_base_model))->set_base_data(&_data_base);
            dynamic_cast<gnss_model_comb_dd *>(&(*_base_model))->set_rec_info(_gallobj->obj(_site_base)->crd_arp(_epoch), _vBanc(3), _vBanc_base(3));
            //dynamic_cast<gnss_model_comb_dd*>(&(*_base_model))->set_rec_info(_gallobj->obj(_site_base)->crd_arp(_epoch), 0, 0);
        }
        // use combmodel
        gnss_proc_lsq_equationmatrix equ;
        iobs = _cmp_equ(equ);

        equ.chageNewMat(A, P, l, nPar);
        dx.resize(nPar);
        dx.setZero();
        
        _obs_index.clear();
        _generateObsIndex(equ);

        if (iobs < _minsat * mult)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Not enough processable observations!");
            _restore(QsavBP, XsavBP);
            return -1;
        }

        if (_isBase || _sd_sat)
        {
            if (_combineDD(A, P, l) < 0)
                return -1;
        }

        Qsav = _Qx;

        try
        {
            if (_nppmodel == NPP_MODEL::PPP_RTK && !_isCompAug)
            {
                if(!_isCorObs)
                    _addconstraint(A, P, l);
            }
            _filter->update(A, P, l, dx, _Qx);
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt", " filter update failed!");
            _Qx = Qsav;
            return -1;
        }

        // increasing variance after update in case of introducing new ambiguity
        if (_cntrep == 1 && !_reset_amb && !_reset_par && !_pos_kin)
        {
            for (size_t iPar = 0; iPar < _param->parNumber(); iPar++)
            {
                //if (_param->operator[](iPar).parType == base_par::AMB_IF) {
                if (_param->operator[](iPar).str_type().find("AMB") != std::string::npos)
                {
                    std::string sat = _param->operator[](iPar).prn;
                    if (_newAMB.find(sat) != _newAMB.end() /*&& _cntrep == 1*/)
                    {
                        if (_newAMB[sat] == 1)
                            _Qx.matrixW()(iPar, iPar) += 10;
                        if (_newAMB[sat] == 2 && _Qx(iPar, iPar) > 0.01)
                            _Qx.matrixW()(iPar, iPar) += 1;
                        //_newAMB[sat]++;
                    }
                }
            }
            auto amb_prn = _param->amb_prns();
            for (const auto &sat : amb_prn)
            {
                if (_newAMB.find(sat) != _newAMB.end())
                    _newAMB[sat]++;
            }
        }
        _posterioriTest(A, P, l, dx, _Qx, v_norm, vtpv);

        nobs_total = A.rows();
        npar_number = A.cols();
        _realnobs = l.size();

#ifdef DEBUG
        //    Symmetric Corr = cov2corr(_Qx);
        std::cout << "Used satellites: ";
        for (unsigned int i = 0; i < _data.size(); i++)
            std::cout << _data[i].sat() << " ";
        std::cout << std::endl;
        std::cout << "Used parameters: ";
        for (unsigned int i = 0; i < _param->parNumber(); i++)
            std::cout << _param->operator[](i).str_type() << "_" << _param->operator[](i).prn << " ";
        std::cout << std::endl;
        std::cout << std::fixed << std::setprecision(3)
             << _epoch.str_ymdhms() << std::endl
             //         << " m0: " << std::setw(7) << _sig_unit << std::endl
             << " dx: " << std::setw(7) << dx
             //         << " l.rows(): " << l.rows() << std::endl
             //         << std::setw(7) << l
             //         << " v_orig.rows(): " << v_orig.rows()
             //         << " v_orig: "  << std::setw(7) << v_orig
             //         << " v_norm: "  << std::setw(7) << v_norm
             //         << " A(" << A.rows() << "," << A.cols() << "):" << std::setw(7) << std::endl << A
             //         << " P: "  << std::setw(7) << P
             //         << " Q: "  << std::setw(7) << _Qx
             //         << " C: "  << std::setw(7) << Corr
             << " Param " << std::setw(7) << _param
             << std::endl;
        std::cout.flush();

        // for(int i = 1; i <= _param->parNumber(); i++) std::cout << _param[_param->getParam(i)].str_type() << " "; std::cout << std::endl;
        // for(int i = 1; i <= _param->parNumber(); i++) std::cout << _param[_param->getParam(i)].str_type() << " " << Corr.row(i) << " "; std::cout << std::endl;

        //int ooo; cin >> ooo;
#endif
        //} while (_outlierDetect_lhb(v_orig,v_norm, Qsav, outlier) != 0);
    } while (_outlierDetect(v_norm, Qsav, outlier) != 0);
    _nSat_excl = _nSat - _data.size(); // number of excluded satellites due to different reasons
    //_print_ele();
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
            SPDLOG_LOGGER_INFO(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + " skipped: " + base_type_conv::int2str(_data.size()) + " < _minsat)");
        //    std::cout << "Epoch " << _epoch.str_hms() << " not calculated due to _minsat." << std::endl;
        _restore(QsavBP, XsavBP);
        return -1;
    }
    // the velocity is not estimated by the real-time model (suggested by zhshen !!!)
    if (_doppler)
    {
        int irc = _processEpochVel();
        if (irc < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt", "estimate velocity by doppler failed!");
            _vel = Triple(0, 0, 0);
            _Qx_vel.setZero();
        }
    }

    _filter->add_data(*_param, dx, _Qx, _sig_unit, Qsav);
    _filter->add_data(A, P, l);
    _filter->add_data(vtpv, nobs_total, npar_number);

    base_allpar param_after = *_param;

    _amb_resolution();
    for (unsigned int iPar = 0; iPar < _param->parNumber(); iPar++)
    {
        _param->operator[](iPar).value(_param->operator[](iPar).value() + dx(_param->operator[](iPar).index));
        if (_reset_amb_ppprtk && _isClient && !_isBase && _nppmodel == NPP_MODEL::PPP_RTK)
            _param->operator[](iPar).amb_ini = !_amb_state;
        //std::cout << _param->operator[](iPar).str_type() << " " << _param->operator[](iPar).value() << std::endl;
    }

    return _amb_state ? 1 : 0;
}

// RTK 模糊度固定主函数：_amb_resolution()
int hwa_gnss::gnss_proc_pvtflt::_amb_resolution()
{
    // 获取当前滤波器估计的增量向量和协方差矩阵
    Vector dx_tmp = _filter->dx();               // 参数改正量（float解）
    Symmetric Qx_tmp = _filter->Qx();             // 参数协方差矩阵
    _param_fixed = _filter->param();                    // 初始浮点解参数备份
    _amb_state = false;                                 // 当前模糊度固定状态，初始为false

#if DEBUG
    // 输出当前历元时间戳和参数信息（调试用）
    std::cout << _epoch.str_ymdhms() << std::endl;
    for (int i = 0; i < _param_fixed.parNumber(); i++) {
        std::cout << std::fixed << std::setw(20) << " Float EPO  "
            << std::setw(20) << _filter->param()[i].str_type() + "  "
            << std::setw(20) << std::setprecision(5) << _filter->param()[i].value()
            << std::setw(15) << std::setprecision(5) << _filter->dx()(i + 1)
            << std::setw(20) << std::setprecision(5) << _filter->param()[i].value() + _filter->dx()(i + 1)
            << std::setw(20) << std::setprecision(5) << _filter->stdx()(i + 1) << std::endl;
}
#endif

    // 判断是否启用固定模式
    if (_fix_mode != FIX_MODE::NO)
    {
        // 实时或使用UPD（未校准相位延迟）解时，更新时间标签
        if (_realtime || (_gupd && _gupd->wl_epo_mode())) {
            _ewl_Upd_time = _epoch;
            _wl_Upd_time = _epoch;
        }

        // 初始化模糊度固定模块的一些设置（首次调用）
        if (_isFirstFix) {
            _glofrq_num = _gobs->glo_freq_num(); // 获取GLONASS频率编号
            if (_glofrq_num.empty() && _gnav) {
                _glofrq_num = _gnav->glo_freq_num();
            }
            _ambfix->setLOG(_spdlog);           // 设置日志记录器
            _ambfix->setWaveLength(_glofrq_num);// 设置波长信息（GLONASS）
            _ambfix->setUPD(_gupd);             // 设置UPD改正
            _isFirstFix = false;
        }

        _ambfix->setObsType(_observ);                   // 设置观测类型（IF组合、RAW等）
        _ambfix->setActiveAmb(_filter->npar_number());  // 设置当前活动的模糊度数

        // 设置观测特征值（MW、SNR、高度角等）
        if (_observ == OBSCOMBIN::IONO_FREE)
            _ambfix->setMW(_MW[_epoch]);
        else {
            _ambfix->setELE(_crt_ele);
            _ambfix->setSNR(_crt_SNR);
        }

        if (_isClient) {
            _ambfix->setObsLevel(_crt_ObsLevel);
        }

        // 设置参考卫星（如果启用了明确设置）
        bool isSetRefSat = dynamic_cast<set_amb*>(_set)->isSetRefSat();
        if (isSetRefSat && !_isBase) {
            bool ref_valid = _getSatRef();
            if (!ref_valid) return 0;  // 无有效参考卫星
        }
        if (!isSetRefSat) _sat_ref.clear();
        _ambfix->setSatRef(_sat_ref);

        // 按顺序进行三类模糊度固定尝试：EWL -> WL -> NL
        if ((_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX) && _frequency >= 3) {
            _ambfix->processBatch(_epoch, _filter, nullptr, "EWL");
        }
        if (_observ == OBSCOMBIN::RAW_ALL || (_observ == OBSCOMBIN::RAW_MIX && _frequency >= 2)) {
            _ambfix->processBatch(_epoch, _filter, nullptr, "WL");
        }

        // 最后尝试固定NL（最关键的整数固定）
        int nlfix_valid = _ambfix->processBatch(_epoch, _filter, nullptr, "NL");
        if (nlfix_valid < 0) {
            _amb_state = false;
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + _epoch.str_ymdhms(" epoch ") + ": fix ambiguity failed !");
        }
        else {
            _amb_state = _ambfix->amb_fixed();
        }
    }

#ifdef DEBUG
    // 输出固定后参数结果（调试）
    std::cout << _epoch.str_ymdhms() << std::endl;
    for (int i = 0; i < _param_fixed.parNumber(); i++) {
        std::cout << std::fixed << std::setw(20) << "EPO  "
            << std::setw(20) << _filter->param()[i].str_type() + "  "
            << std::setw(20) << std::setprecision(5) << _filter->param()[i].value()
            << std::setw(15) << std::setprecision(5) << _filter->dx()(i + 1)
            << std::setw(20) << std::setprecision(5) << _filter->param()[i].value() + _filter->dx()(i + 1) << std::endl;
    }
#endif

    // 输出解的最终结果（固定或浮点）
    std::ostringstream os;
    if (_amb_state) {
        _param_fixed = _ambfix->getFinalParams();  // 获取固定后参数解
        _prtOut(_epoch, _param_fixed, _filter->Qx(), _data, os, line, true);
        _prt_port(_epoch, _param_fixed, _filter->Qx(), _data);
    }
    else {
        for (unsigned int iPar = 0; iPar < _param_fixed.parNumber(); iPar++) {
            _param_fixed[iPar].value(_param_fixed[iPar].value() + dx_tmp(_param_fixed[iPar].index));
        }
        _prtOut(_epoch, _param_fixed, Qx_tmp, _data, os, line, true);
        _prt_port(_epoch, _param_fixed, Qx_tmp, _data);
    }

    // 写入日志文件（若配置）
    if (_flt) {
        _flt->write(os.str().c_str(), os.str().size());
        _flt->flush();
    }

    return 1;
}


bool gnss_proc_pvtflt::_getSatRef()
{
    _sat_ref.clear();
    //one_epoch_upd ewl_upd, wl_upd, nl_upd;
    //if (_frequency >= 3) ewl_upd = _gupd->get_epo_upd(EWL_IDENTIFY);
    //wl_upd = _gupd->get_epo_upd(WL_IDENTIFY);
    //nl_upd = _gupd->get_epo_upd(_epoch);
    one_epoch_upd wl_upd, nl_upd, ewl_upd;
    if (_gupd)
    {
        wl_upd = _gupd->get_epo_upd(UPDTYPE::WL, _wl_Upd_time);
        ewl_upd = _gupd->get_epo_upd(UPDTYPE::EWL, _ewl_Upd_time);
        if (_upd_mode == UPD_MODE::UPD)
            nl_upd = _gupd->get_epo_upd(UPDTYPE::NL, _epoch);
    }

    base_allpar params_all = _filter->param();
    std::set<std::string> sysall = dynamic_cast<set_gen *>(_set)->sys();
    for (auto sys_iter = sysall.begin(); sys_iter != sysall.end(); sys_iter++)
    {
        GSYS sys = gnss_sys::str2gsys(*sys_iter);
        double max_mw = 0.0;
        std::string ref;

        for (int i = 0; i < params_all.parNumber(); i++)
        {
            if (params_all[i].str_type().find("AMB") == std::string::npos)
                continue;

            if (gnss_sys::sat2gsys(params_all[i].prn) != sys)
                continue;

            if (sys == GSYS::BDS && stoi(params_all[i].prn.substr(1, 3)) <= 5)
                continue; // exclude BDS C01~C05

            if (_nppmodel == NPP_MODEL::PPP_RTK && !_isCompAug && _isClient)
            {
                if (!_sagnss_coder_aug_map[_epoch][params_all[i].prn])
                    continue;
            }

            if (_observ == OBSCOMBIN::RAW_MIX && _receiverType != RECEIVERTYPE::And)
            {
                if (_sat_freqs[params_all[i].prn] == "1")
                    continue;
            }

            // reference satellite must have WL/NL UPD
            //LX 注释窄巷因为整数钟时没有窄巷upd
            // elev >= 15 angle && continuous observation epoch > 600s
            //if (wl_upd.find(params_all[i].prn) == wl_upd.end() || _frequency >= 3 && ewl_upd.find(params_all[i].prn) == ewl_upd.end()) continue;
            //if (_frequency >= 3 && ewl_upd[params_all[i].prn]->npoint <= 2) continue;
            //if (!((wl_upd[params_all[i].prn]->npoint > 2 && _epoch - params_all[i].beg) >= 0 && _gupd->get_epo_upd(_epoch).find(params_all[i].prn) != _gupd->get_epo_upd(_epoch).end()\
            //    && _crt_ele[params_all[i].prn] > 15))continue;

            //if (_upd_mode == UPD_MODE::UPD)
            //{
            //    if (_gupd && !(wl_upd.find(params_all[i].prn) != wl_upd.end() && nl_upd.find(params_all[i].prn) != nl_upd.end())) continue;
            //}
            //else if (_upd_mode == UPD_MODE::IRC)
            //{
            //    if (_gupd && !(wl_upd.find(params_all[i].prn) != wl_upd.end()))continue;
            //}
            //if (_crt_ele[params_all[i].prn] < 15 || !_isClient && (_epoch - params_all[i].beg < 300)) continue;

            // modified 0->300 for Server !!!!!
            if (_upd_mode == UPD_MODE::UPD)
            {
                if (_gupd && _frequency >= 3 && (ewl_upd.find(params_all[i].prn) == ewl_upd.end() || ewl_upd[params_all[i].prn]->npoint <= 2))
                    continue;
                if (_gupd && !(wl_upd.find(params_all[i].prn) != wl_upd.end() && nl_upd.find(params_all[i].prn) != nl_upd.end()))
                    continue;
                if (_gupd && !(wl_upd[params_all[i].prn]->npoint > 2))
                    continue;
            }
            else if (_upd_mode == UPD_MODE::IRC)
            {
                if (_gupd && !(wl_upd.find(params_all[i].prn) != wl_upd.end()))
                    continue;
            }

            if (_crt_ele[params_all[i].prn] < 15 || (!_isClient && (_epoch - params_all[i].beg <= 0)))
                continue;

            double temp = (_epoch - params_all[i].beg + 1) * _crt_ele[params_all[i].prn];
            // max  elev * continuous observation epoch
            if (temp > max_mw)
            {
                max_mw = temp;
                ref = params_all[i].prn;
            }
        }
        //    std::cerr << "ref_sat    " << (_gupd->get_epo_upd(EWL_IDENTIFY).find(ref) != _gupd->get_epo_upd(EWL_IDENTIFY).end()) << std::endl;
        //    std::cerr << "ref_sat    " << ref << std::endl;
        if (!ref.empty())
        {
            _sat_ref.insert(ref);
        }
    }
    if (_sat_ref.empty())
        return false;
    else
        return true;
}
int hwa_gnss::gnss_proc_pvtflt::_outlierDetect_lhb(const Vector &v_post, const Vector &v_norm, const Symmetric &Qsav, std::string &sat)
{

    int nobs = v_post.rows();
    double max = 0.0;
    int idx = -1;
    if (_nppmodel == NPP_MODEL::PPP_RTK && !_isCompAug)
    {
        nobs = _realnobs;
    }
    for (int i = 0; i < nobs; i++)
    {
        sat = _obs_index[i].first;

        gnss_data_sats tmpdata;
        for (int j = 0; j < _data.size(); i++)
        {
            if (sat == (_data)[j].sat())
                tmpdata = (_data)[j];
        }
        if (tmpdata.sat() == "")
            continue;

        if (fabs(v_post(i)) > _max_res_norm / sin(tmpdata.ele()) && fabs(v_norm(i)) > _max_res_norm)
        {
            if (fabs(v_post(i)) > max)
            {
                max = fabs(v_post(i));
                idx = i;
            }
        }
    }
    if (idx >= 0)
    {
        _Qx = Qsav;
        sat = _obs_index[idx].first;
        std::string obsType = gobstype2str(_obs_index[idx].second.second);
        std::ostringstream os;
        os << _site << " outlier (" << obsType << _obs_index[idx].second.first << ") " << sat
           << " v: " << std::fixed << std::setw(16) << std::right << std::setprecision(3) << max;
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _epoch.str_ymdhms(" epoch ") + os.str());
    }
    else
        sat = "";

    return idx + 1;
}
int hwa_gnss::gnss_proc_pvtflt::_outlierDetect(const Vector &v, const Symmetric &Qsav, std::string &sat)
{
    int nobs = v.rows();
    double max = 0.0;
    int idx = -1;
    if (_nppmodel == NPP_MODEL::PPP_RTK && !_isCompAug)
    {
        nobs = _realnobs;
    }
    for (int i = 0; i < nobs; i++)
    {
        if (fabs(v(i)) > max && fabs(v(i)) > _max_res_norm)
        {
            max = fabs(v(i));
            idx = i;
        }
    }
    if (idx >= 0)
    {
        _Qx = Qsav;
        sat = _obs_index[idx].first;
        std::string obsType = gobstype2str(_obs_index[idx].second.second);
        std::ostringstream os;
        os << _site << " outlier (" << obsType << _obs_index[idx].second.first << ") " << sat
           << " v: " << std::fixed << std::setw(16) << std::right << std::setprecision(3) << max <<" "<< _epoch.sow() + _epoch.dsec();
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _epoch.str_ymdhms(" epoch ") + os.str());
    }
    else
        sat = "";

    return idx + 1;
}

std::string hwa_gnss::gnss_proc_pvtflt::_getsat_from_obs(int idx)
{
    return std::string();
}

void hwa_gnss::gnss_proc_pvtflt::_print_ele()
{

    std::ofstream file("ele.txt", std::ios::app);

    std::streambuf *stream_buffer_cout = std::cout.rdbuf();
    std::streambuf *stream_buffer_file = file.rdbuf();
    std::cout.rdbuf(stream_buffer_file);

    std::cout << _epoch.sow() + _epoch.dsec() << "  " << _data.size() << "  ";
    for (std::vector<gnss_data_sats>::iterator it = _data.begin(); it != _data.end(); ++it)
    {
        std::cout << it->sat() << ":" << it->ele_deg() << "  ";
    }
    std::cout << std::endl;
    std::cout.rdbuf(stream_buffer_cout);
}

int hwa_gnss::gnss_proc_pvtflt::_prt_info(const base_time &runEpoch)
{
    std::cout << runEpoch.sow() + runEpoch.dsec() << "   " << _param->parNumber() << "  ";
    // Edit and save parematres
    for (unsigned int iPar = 0; iPar < _param->parNumber(); iPar++)
    {
        std::cout << "  " + _param->operator[](iPar).str_type() << " ";
        std::cout << std::fixed << std::setw(18) << std::setprecision(6) << _param->operator[](iPar).value() << "  " << _Qx(iPar, iPar) << "  \n";
    }
    std::cout << std::endl;
    return 0;
}

std::string hwa_gnss::gnss_proc_pvtflt::_gen_kml_description(const base_time &epoch, const base_posdata::data_pos &posdata)
{
    char s[1000];
    std::string grade = _quality_grade(posdata);
    sprintf(s, "<B>Epoch:</B><BR><BR>\
        <TABLE border = \"1\"  width=\"100%%\"  Align = \"center\">\
        <TR ALIGN = RIGHT>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>Time : </TD><TD>%s</TD><TD>%s</TD><TR>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>Week : </TD><TD> %s </TD><TD>Sow : </TD><TD>%.4f</TD><TR>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>Pos(m): </TD><TD>%.4f</TD><TD>%.4f</TD><TD>%.4f</TD><TR>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>Quality : </TD><TD>Q%s </TD><TD>Amb : </TD><TD>%s</TD><TR>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>nSats : </TD><TD>%d </TD><TD>PDOP : </TD><TD>%.2f </TD><TR>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>Sig(m) : </TD><TD>%.3f</TD><TD>%.3f</TD><TD>%.3f</TD><TR>\
        <TR ALIGN = RIGHT><TD ALIGN = LEFT>Vel(m/s) : </TD><TD>%.3f</TD><TD>%.3f</TD><TD>%.3f</TD><TR>\
        </TABLE>",
        epoch.str_ymd().c_str(), epoch.str_hms().c_str(),
        epoch.str_gwk().c_str(), posdata.t,
        posdata.pos[0], posdata.pos[1], posdata.pos[2],
        grade.c_str(), posdata.amb_state ? "FIXED" : "FLOAT",
        posdata.nSat, posdata.PDOP,
        sqrt(posdata.Rpos[0]), sqrt(posdata.Rpos[1]), sqrt(posdata.Rpos[2]),
        posdata.vn[0], posdata.vn[1], posdata.vn[2]
    );

    return std::string(s);
}

std::string hwa_gnss::gnss_proc_pvtflt::_quality_grade(const base_posdata::data_pos & pos)
{
    if (sqrt(pos.Rpos[0]) < 0.1 &&
        sqrt(pos.Rpos[1]) < 0.1&&
        sqrt(pos.Rpos[2]) < 0.1)
    {
        if (pos.amb_state)
            return "1";
        else
            return "2"; // for float PPP
    }
    else if (
        sqrt(pos.Rpos[0]) < 0.2 &&
        sqrt(pos.Rpos[1]) < 0.2&&
        sqrt(pos.Rpos[2]) < 0.2)
    {
        if (pos.amb_state)
            return "2";
        else
            return "3";
    }
    else if (
        sqrt(pos.Rpos[0]) < 0.5 &&
        sqrt(pos.Rpos[1]) < 0.5 &&
        sqrt(pos.Rpos[2]) < 0.5)
        if (pos.amb_state)
            return "3";
        else
            return "4";
    else if (
        sqrt(pos.Rpos[0]) < 1 &&
        sqrt(pos.Rpos[1]) < 1 &&
        sqrt(pos.Rpos[2]) < 1)
        return "5";
    else
        return "6";
}

int hwa_gnss::gnss_proc_pvtflt::processBatch(const base_time &beg_r, const base_time &end_r, bool prtOut)
{
    if (_grec == nullptr)
    {
        std::ostringstream os;
        os << "ERROR: No object found (" << _site << "). Processing terminated!!! " << beg_r.str_ymdhms() << " -> " << end_r.str_ymdhms() << std::endl;
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, os.str());
        return -1;
    }

    int sign = 1;
    double subint = 0.1;

    InitProc(beg_r, end_r, &subint);
    _gobs->setepoches(_site);

    int smt_delay = dynamic_cast<set_flt *>(_set)->smt_delay();
    base_time now(_beg_time);

    if (_spdlog)
        SPDLOG_LOGGER_INFO(_spdlog, _site + ": Start GNSS Processing filtering: " + now.str_ymdhms() + " " + _end_time.str_ymdhms());
    bool time_loop = true;
    while (now <= _end_time)
    {
        // synchronization
        if (now != _end_time)
        {
            //判断当前历元 epo 是否符合设定的采样率 smp
            if (!time_sync(now, _sampling, _scale, _spdlog))
            {                                       // now.sod()%(int)_sampling != 0 ){
                now.add_dsec(sign * subint / 100); // add_dsec used for synchronization!

                continue;
            }
            if (_sampling > 1)
                now.reset_dsec();
        }
    
        _available_data_realtime(now);

        if (_isClient)
        {
            if (!_gInterpol->interpolAug(now, _data, _data_base))
            {
                if (_sampling > 1)
                    now.add_secs(int(sign * _sampling)); // =<1Hz data
                else
                    now.add_dsec(sign * _sampling); //  >1Hz data
                continue;
            }
        }

        _slip_detect(now);
        int irc_epo = ProcessOneEpoch(now);
        if (irc_epo < 0)
        {
            if (_sampling > 1)
                now.add_secs(int(sign * _sampling)); // =<1Hz data
            else
                now.add_dsec(sign * _sampling); //  >1Hz data
            continue;
        }
        else
            _success = true;

        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + now.str_ymdhms(" processing epoch: "));

        double percent;
        if (_end_time == _beg_time) percent = 100;
        else percent = now.diff(_beg_time) / _end_time.diff(_beg_time) * 100.0;

        if(_realtime)
            std::cerr << "\r" << _site << "   " << now.str_ymdhms() << std::setw(5) << " Q = " << (_amb_state ? 1 : 2) << std::fixed << std::setprecision(1) << std::setw(6) << percent << "%";
        else
        {
            if (double_eq(now.sow() % 1, 0.0))
                std::cerr << "\r" << _site << "   " <<  now.str_ymdhms() << std::setw(5) << " Q = " << (_amb_state ? 1 : 2) << std::fixed << std::setprecision(1) << std::setw(6) << percent << "%";
        }
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
    }

    _running = false;
    if (_smooth && smt_delay == 0)
    {
        _backflt(prtOut);
    }

    if (beg_r != end_r)
    { // do not print in real-time
        double npd_perc = 0;
        npd_perc = (double(_n_NPD_flt) / double(_n_all_flt)) * 100;
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, _site + ": Forward filter -     Not positive definite covariance matrices: " + base_type_conv::dbl2str(npd_perc, 0) + " %   (" + base_type_conv::int2str(_n_NPD_flt) + ", " + base_type_conv::int2str(_n_all_flt) + ")");

        if (_smooth)
        {
            npd_perc = (double(_n_NPD_smt) / double(_n_all_smt)) * 100;
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + ": Backward smoothing - Not positive definite covariance matrices: " + base_type_conv::dbl2str(npd_perc, 0) + " %   (" + base_type_conv::int2str(_n_NPD_smt) + ", " + base_type_conv::int2str(_n_all_smt) + ")");
        }
    }

    //output 3drms for all epochs,fenggl 202010
    if (_crd_est != CONSTRPAR::KIN && !_pos_kin && _enufile)
    {
        neu_sum_nepo.second /= neu_sum_nepo.first;
        std::ostringstream enu_info;
        enu_info << " RMS = " << std::fixed << std::setw(12) << std::setprecision(4) << sqrt(neu_sum_nepo.second[1])
                 << std::setw(12) << std::setprecision(4) << sqrt(neu_sum_nepo.second[0])
                 << std::setw(12) << std::setprecision(4) << sqrt(neu_sum_nepo.second[2]) << std::endl;
        _enufile->write(enu_info.str().c_str(), enu_info.str().size());
        _enufile->flush();
    }

    return 1;
}

bool hwa_gnss::gnss_proc_pvtflt::InitProc(const base_time &begT, const base_time &endT, double *subint)
{
    _beg_time = begT;
    _end_time = endT;

    if (_isBase)
    {
        if (_setCrd() < 0)
            return -1;
    }

    _gquality_control.setNav(_gnav);
    _dop.set_data(_gnav, _gobs, _site);
    _running = true;

    if (!_isBase)
    {
        for (int i = 0; i < _param->parNumber(); i++)
            if (_param->operator[](i).site == "")
                _param->setSite(_site);
    }

    //double subint = 0.1;
    if (subint)
    {
        if (_scale > 0)
            *subint = 1.0 / _scale;
        if (_sampling > 1)
            *subint = pow(10, floor(log10(_sampling)));
    }

    _prtOutHeader();

    return true;
}

int hwa_gnss::gnss_proc_pvtflt::ProcessOneEpoch(const base_time &now, std::vector<gnss_data_sats> *data_rover, std::vector<gnss_data_sats> *data_base)
{

    if (/*_data.size()*/ _getData(now, data_rover, false) == 0)
    {
        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + now.str_ymdhms(" no observation found at epoch: "));
        }
        return -1;
    }

    // apply dcb
    if (_gallbias)
    {
        for (auto &itdata : _data)
        {
            itdata.apply_bias(_gallbias);
        }
    }

    std::vector<gnss_data_sats>::iterator it = _data.begin();
    std::string double_freq = "";
    std::string single_freq = "";

    std::ostringstream obsqualityInfo; obsqualityInfo.str("");
    obsqualityInfo << "> " << std::setw(6) << now.sod() << std::endl;
    _sat_freqs.clear();
    while (it != _data.end())
    {
        GOBSBAND b1 = _band_index[it->gsys()][FREQ_1];
        GOBSBAND b2 = _band_index[it->gsys()][FREQ_2];

        auto obsL1 = it->select_phase(b1);
        auto obsL2 = it->select_phase(b2);
        auto obsP1 = it->select_range(b1);
        auto obsP2 = it->select_range(b2);
        auto snrL1 = it->getobs(pl2snr(obsP1)) > it->getobs(pl2snr(obsL1)) ? it->getobs(pl2snr(obsP1)) : it->getobs(pl2snr(obsL1));
        auto snrL2 = it->getobs(pl2snr(obsP2)) > it->getobs(pl2snr(obsL2)) ? it->getobs(pl2snr(obsP2)) : it->getobs(pl2snr(obsL2));
        auto ele = it->ele_deg();
        //auto snrL1 = it->obs_S(b1);
        //std::cerr << ele << std::setw(6) << std::endl;

        if ((obsL1 == GOBS::X && obsL2 != GOBS::X) || (obsL1 != GOBS::X && obsL2 == GOBS::X))
        {
            single_freq += "  " + it->sat();
            _sat_freqs[it->sat()] = "1";
        }

        if (obsL1 != GOBS::X && obsL2 != GOBS::X)
        {
            double_freq += "  " + it->sat();
            _sat_freqs[it->sat()] = "2";
        }
        obsqualityInfo << it->sat();
        if (obsP1 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
        else obsqualityInfo << std::setw(4) << "1";
        if (obsL1 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
        else obsqualityInfo << std::setw(4) << "1";
        if (obsP2 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
        else obsqualityInfo << std::setw(4) << "1";
        if (obsL2 == GOBS::X) obsqualityInfo << std::setw(4) << "0";
        else obsqualityInfo << std::setw(4) << "1";
        obsqualityInfo << std::setw(10) << std::setprecision(4) << snrL1
           << std::setw(10) << std::setprecision(4) << snrL2 << std::endl;
        ++it;
    }
    if (_obsqualityfile)
    {
        _obsqualityfile->write(obsqualityInfo.str().c_str(), obsqualityInfo.str().size());
        _obsqualityfile->flush();
    }

    if (_isBase)
    {
        if (/*_data_base.size()*/_getData(now, data_base, true) == 0)
        {
            if (SPDLOG_LEVEL_TRACE == _spdlog->level())
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, std::string("gpvtflt:  ") +  _site_base + now.str_ymdhms(" no observation found at epoch: "));
            }
            return -1;
        }
        // apply dcb
        if (_gallbias)
        {
            for (auto &itdata_base : _data_base)
            {
                itdata_base.apply_bias(_gallbias);
            }
        }
    }

    base_time obsEpo = _data.begin()->epoch();

    _timeUpdate(obsEpo);

    // reseting params according to conf
    //zzwu comment according to zhshen
    /*if (_reset_amb > 0)
    {
        if (now.sod() % _reset_amb == 0)
        {
            _reset_ambig();
        }
    }*/
    if (_reset_par > 0)
    {
        if (now.sod() % _reset_par == 0)
        {
            _reset_param();
        }
    }

    if (_realtime)
    {
        _gobs->erase(_site, obsEpo - 1800);
        if (_isBase)
            _gobs->erase(_site_base, obsEpo);
    }

    // save apriory coordinates
    if (_crd_est != CONSTRPAR::FIX)
        _saveApr(obsEpo, *_param, _Qx);

    int irc_epo = gnss_proc_pvtflt::_processEpoch(obsEpo);

    // Constraint finish at this epoch
    _pos_constrain = false;

    if (irc_epo < 0)
    {
        _success = false;
        _removeApr(obsEpo);
        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, _site + now.str_ymdhms(" epoch ") + " was not calculated");
    }
    else
        _success = true;

    return irc_epo;
}

base_allpar &hwa_gnss::gnss_proc_pvtflt::getUCparam(std::set<std::string> *satref)
{
    base_allpar &param = _ambfix->updateUCAmb(satref);
    for (auto i = 0; i < (_isBase ? 2 : 1); i++)
    {
        std::string site = _site;
        double clk = _vBanc(3);
        if (i)
        {
            site = _site_base;
            clk = _vBanc_base(3);
        }
        int index = param.getParam(site, par_type::CLK, "");
        if (index < 0)
        {
            int ipar = param.parNumber();
            base_par par(site, par_type::CLK, ++ipar, "");
            par.value(clk);
            param.addParam(par);
        }
    }
    return param;
}

int hwa_gnss::gnss_proc_pvtflt::processBatchFB(const base_time &beg_r, const base_time &end_r)
{

    //this->processBatch(beg_r, end_r, false);

    _beg_end = false;

    //_reset_param();

    this->processBatch(beg_r, end_r, true);

    _beg_end = true;

    return 1;
}

void hwa_gnss::gnss_proc_pvtflt::Add_UPD(gnss_data_upd *gupd)
{
    _gupd = gupd;
}

//基站和流动站观测值以卫星prn号对齐
int hwa_gnss::gnss_proc_pvtflt::_selcomsat(std::vector<gnss_data_sats> &data_base, std::vector<gnss_data_sats> &data_rover)
{
    int nSat = 0;
    //com_sat.clear(); //std::mapobsdata.clear();
    std::vector<gnss_data_sats> bdata, rdata;
    //first to recycle on base-site observations
    for (auto iter_base = data_base.begin(); iter_base != data_base.end(); iter_base++)
    {
        std::string comsat = iter_base->sat();
        //second to recycle on rover-site obserations
        for (auto iter_rover = data_rover.begin(); iter_rover != data_rover.end(); iter_rover++)
        {
            if (iter_rover->sat() == comsat /*&& _set->exsats.find(comsat) == _set->exsats.end()*/)
            {
                //std::mapobsdata[_site_base][comsat] = *iter_base;
                //std::mapobsdata[_site][comsat] = *iter_rover;
                bdata.push_back(*iter_base);
                rdata.push_back(*iter_rover);
                //com_sat.insert(comsat);
                nSat++;
                break;
            }
        }
    }
    if (nSat)
    {
        data_base = bdata;
        data_rover = rdata;
    }
    return nSat;
}

void hwa_gnss::gnss_proc_pvtflt::_udsdAmb()
{

    for (std::map<std::string, int>::iterator it = _newAMB.begin(); it != _newAMB.end();)
    {
        if (it->second > 2)
            _newAMB.erase(it++);
        else
            ++it;
    }
    if (_nSat == 0)
    {
        //zzwu add
        for (unsigned int i = 0; i < _param->parNumber(); i++)
        {
            if (_param->operator[](i).parType == par_type::AMB_IF ||
                _param->operator[](i).parType == par_type::AMB_L1 ||
                _param->operator[](i).parType == par_type::AMB_L2 ||
                _param->operator[](i).parType == par_type::AMB_L3 ||
                _param->operator[](i).parType == par_type::AMB_L4 ||
                _param->operator[](i).parType == par_type::AMB_L5)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "AMB will be removed! For Sat PRN " + _param->operator[](i).prn + " Epoch: " + _epoch.str_ymdhms());

#ifdef DEBUG
                std::cout << "AMB will be removed! For Sat PRN " << _param->operator[](i).prn
                    << " Epoch: " << _epoch.str_ymdhms() << std::endl;
#endif

                _amb_obs.erase(std::make_pair(_param->operator[](i).prn, _param->operator[](i).parType));

                _newAMB.erase(_param->operator[](i).prn);

                _Qx.Matrix_remRC(_param->operator[](i).index, _param->operator[](i).index);
                _param->delParam(i);
                _param->reIndex();
                i--;
            }
        }
        //
        return;
    }
    // Add ambiguity parameter and appropriate rows/columns covar. matrix
    std::set<std::string> mapPRN;
    for (int i = 0; i < _data.size(); i++)
    {
        auto rsatdata = _data[i];
        auto bsatdata = _data_base[i];

        GSYS gs = rsatdata.gsys();
        std::string sat = rsatdata.sat();
        mapPRN.insert(rsatdata.sat());

        // bool com = true;
        // if (_gnav)
        //     com = _gnav->com();
        std::vector<GOBSBAND> band = dynamic_cast<set_gnss *>(_set)->band(gs);

        std::tuple<GOBS, GOBS, GOBS, GOBS> amb_obs_identifier = std::make_tuple(X, X, X, X);

        if (_observ == OBSCOMBIN::IONO_FREE)
        {
            GOBSBAND b1, b2;
            if (_auto_band)
            { // automatic dual band selection -> for Anubis purpose
                std::set<GOBSBAND> bands = rsatdata.band_avail();
                auto itBAND = bands.begin();
                if (bands.size() < 2)
                    continue;
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
            }
            else
            { // select fix defined band according the table
                if (!band.empty())
                {
                    b1 = band[0];
                    b2 = band[1];
                }
                else
                {
                    b1 = gnss_sys::band_priority(gs, FREQ_1);
                    b2 = gnss_sys::band_priority(gs, FREQ_2);
                }
            }
            bool update_amb = false;
            double amb[2] = {0};
            
            for (int j = 0; j < 2; j++)
            {
                gnss_data_sats tmpsatdata = rsatdata;
                if (j == 1)
                    tmpsatdata = bsatdata;
                gnss_data_obs gobs1(tmpsatdata.select_phase(b1));
                gnss_data_obs gobs1_P(tmpsatdata.select_range(b1));
                gnss_data_obs gobs2(tmpsatdata.select_phase(b2));
                gnss_data_obs gobs2_P(tmpsatdata.select_range(b2));
                if (j == 0)
                {
                    std::get<0>(amb_obs_identifier) = gobs1.gobs();
                    std::get<1>(amb_obs_identifier) = gobs2.gobs();
                }
                else if (j == 1)
                {
                    std::get<2>(amb_obs_identifier) = gobs1.gobs();
                    std::get<3>(amb_obs_identifier) = gobs2.gobs();
                }

                if (gobs1.gobs() == X || gobs2.gobs() == X || gobs1_P.gobs() == X || gobs2_P.gobs() == X)
                {
                    update_amb = false;
                    break;
                }
                amb[j] = tmpsatdata.L3(gobs1, gobs2);
                if (double_eq(amb[j], 0.0))
                {
                    update_amb = false;
                    break;
                }
                amb[j] -= tmpsatdata.P3(gobs1_P, gobs2_P);
                if (_param->getParam(_site, par_type::AMB_IF, sat) < 0)
                {
                    update_amb = true;
                }
                if (tmpsatdata.getlli(gobs1.gobs()) >= 1 || tmpsatdata.getlli(gobs2.gobs()) >= 1)
                {
                    update_amb = true;
                    //tmpsatdata.addslip(true);
                    rsatdata.addslip(true); //lvhb save slip information in rover's data
                }
            }
            if (update_amb == false)
            {
                if (_amb_obs.find(std::make_pair(sat, par_type::AMB_IF)) == _amb_obs.end())
                {
                    //std::cerr << "Error: amb_obs not correct!" << sat << " " << _epoch.str_hms() << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, "amb_obs not correct!" + sat + " " + _epoch.str_hms());
                }
                else if (_amb_obs[std::make_pair(sat, par_type::AMB_IF)] != amb_obs_identifier)
                {
                    //std::cerr << "Warning: amb_obs switched silently!" << sat << " " << _epoch.str_hms() << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, "Warning: amb_obs switched silently!" + sat + " " + _epoch.str_hms());
                    _amb_obs[std::make_pair(sat, par_type::AMB_IF)] = amb_obs_identifier;
                }
                continue;
            }
            double sdamb = amb[0] - amb[1];
            int idx = _param->getParam(_site, par_type::AMB_IF, sat);
            if (idx < 0)
            {
                base_par newPar(_site, par_type::AMB_IF, _param->parNumber() + 1, sat);
                newPar.value(sdamb); // first ambiguity value
                _param->addParam(newPar);
                _newAMB[sat] = 1;
                _Qx.Matrix_addRC(_param->parNumber() - 1, _param->parNumber() - 1);
                _Qx.matrixW()(_param->parNumber() - 1, _param->parNumber() - 1) = _sigAmbig * _sigAmbig;
            }
            else
            {
                _param->operator[](idx).value(sdamb);
                _Qx.matrixW()(idx, idx) = _sigAmbig * _sigAmbig;
                _newAMB[sat] = 1;
            }

            _amb_obs[std::make_pair(sat, par_type::AMB_IF)] = amb_obs_identifier;

            //std::cout << "ADD AMB IF " << it->sat() << " " << it->epoch().str_ymdhms() << std::endl;
            //if (_log) _log->comment(3, "gppp", "AMB_IF was added! For Sat PRN " + sat + " Epoch: " + _epoch.str_ymdhms());
        }
        else if (_observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
        {
            int newAmb = 0;
            int nf = 5;
            if (_auto_band)
            { // automatic dual band selection -> for Anubis purpose
                GOBSBAND b1, b2;
                std::set<GOBSBAND> bands = rsatdata.band_avail();
                auto itBAND = bands.begin();
                if (bands.size() < 2)
                    continue;
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
                band.clear();
                band.push_back(b1);
                band.push_back(b2);
            }
            if (band.size())
                nf = band.size();
            for (FREQ_SEQ f = FREQ_1; f <= nf; f = (FREQ_SEQ)(f + 1))
            {
                GOBSBAND b;
                par_type amb_type;
                switch (f)
                {
                /*case FREQ_X:    amb = base_par::AMB_IF; break;*/
                case FREQ_1:
                    amb_type = par_type::AMB_L1;
                    break;
                case FREQ_2:
                    amb_type = par_type::AMB_L2;
                    break;
                case FREQ_3:
                    amb_type = par_type::AMB_L3;
                    break;
                case FREQ_4:
                    amb_type = par_type::AMB_L4;
                    break;
                case FREQ_5:
                    amb_type = par_type::AMB_L5;
                    break;
                default:
                    break;
                }
                if (f > _frequency)
                    continue; // modified by lvhb in 20201211
                // '>' -> '>=' . Modified by YuxuanZhou. 2020/05/31
                if (band.size() >= f)
                { // automatic dual band selection -> for Anubis purpose
                    b = band[f - 1];
                }
                else
                    b = gnss_sys::band_priority(gs, f);

                bool update_amb = false;
                bool skip = false;
                double amb[2] = {0};
                amb[0] = amb[1] = 0.0;
                for (int j = 0; j < 2; j++)
                {
                    gnss_data_sats tmpsatdata = rsatdata;
                    if (j == 1)
                        tmpsatdata = bsatdata;
                    //_getgobs(sat, TYPE_L, b, gobs);
                    gnss_data_obs gobs(tmpsatdata.select_phase(b));
                    if (gobs.gobs() == X)
                    {
                        update_amb = false;
                        skip = true;
                        break;
                    }
                    amb[j] = tmpsatdata.obs_L(gobs);
                    if (double_eq(amb[j], 0.0))
                    {
                        update_amb = false;
                        skip = true;
                        break;
                    }
                    if (_observ == OBSCOMBIN::RAW_SINGLE && f > FREQ_1)
                    {
                        update_amb = false;
                        skip = true;
                        break;
                    }
                    if (_observ == OBSCOMBIN::RAW_DOUBLE && f > FREQ_2)
                    {
                        update_amb = false;
                        skip = true;
                        break;
                    }
                    gnss_data_obs gobsP(tmpsatdata.select_range(b));
                    if (gobsP.gobs() == X)
                    {
                        update_amb = false;
                        skip = true;
                        break;
                    }
                    amb[j] -= tmpsatdata.obs_C(gobsP);

                    if (j == 0)
                    {
                        std::get<0>(amb_obs_identifier) = gobs.gobs();
                    }
                    else if (j == 1)
                    {
                        std::get<2>(amb_obs_identifier) = gobs.gobs();
                    }
                    // add L1 amb - everytime when RAW observations
                    if (_param->getParam(_site, amb_type, sat) < 0)
                    {
                        update_amb = true;
                    }
                    if (tmpsatdata.getlli(gobs.gobs()) >= 1)
                    {
                        update_amb = true;
                        //tmpsatdata.addslip(true);
                        rsatdata.addslip(true); // save slip information in rover's data
                    }

                } //end base+rove
                if (skip)
                    continue;
                if (update_amb == false)
                {
                    if (_amb_obs.find(std::make_pair(sat, amb_type)) == _amb_obs.end())
                    {
                        //std::cerr << "Error: amb_obs not correct!" << sat << " " << _epoch.str_hms() << std::endl;
                        if (_spdlog)
                            SPDLOG_LOGGER_INFO(_spdlog, "amb_obs not correct!" + sat + " " + _epoch.str_hms());
                    }
                    else if (_amb_obs[std::make_pair(sat, amb_type)] != amb_obs_identifier)
                    {
                        //std::cerr << "Warning: amb_obs switched silently!" << sat << " " << _epoch.str_hms() << std::endl;
                        if (_spdlog)
                            SPDLOG_LOGGER_INFO(_spdlog, "Warning: amb_obs switched silently!" + sat + " " + _epoch.str_hms());
                        _amb_obs[std::make_pair(sat, amb_type)] = amb_obs_identifier;
                    }
                    continue;
                }
                double sdamb = amb[0] - amb[1];
                int idx = _param->getParam(_site, amb_type, sat);
                if (idx < 0)
                {
                    base_par newPar(_site, amb_type, _param->parNumber() + 1, sat);
                    newPar.value(sdamb);
                    _param->addParam(newPar);
                    _Qx.Matrix_addRC(_param->parNumber() - 1, _param->parNumber() - 1);
                    _Qx.matrixW()(_param->parNumber() - 1, _param->parNumber() - 1) = _sigAmbig * _sigAmbig;
                    //         std::cout << "ADD AMB L1 " << it->sat() << " " << it->epoch().str_ymdhms()  << " " << L1 - cmpObs << std::endl;
                    //if (_log) _log->comment(3, "gppp", "RAW AMB_L1 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                }
                else
                {
                    _param->operator[](idx).value(sdamb);
                    _Qx.matrixW()(idx, idx) = _sigAmbig * _sigAmbig;
                    newAmb = 1;
                }

                _amb_obs[std::make_pair(sat, amb_type)] = amb_obs_identifier;

            } //end f
            if (newAmb)
                _newAMB[sat] = 1;
        } //end uc

    } //end data

    // Remove ambiguity parameter and appropriate rows/columns covar. matrix

    for (unsigned int i = 0; i < _param->parNumber(); i++)
    {
        if (_param->operator[](i).parType == par_type::AMB_IF ||
            _param->operator[](i).parType == par_type::AMB_L1 ||
            _param->operator[](i).parType == par_type::AMB_L2 ||
            _param->operator[](i).parType == par_type::AMB_L3 ||
            _param->operator[](i).parType == par_type::AMB_L4 ||
            _param->operator[](i).parType == par_type::AMB_L5)
        {

            std::set<std::string>::iterator prnITER = mapPRN.find(_param->operator[](i).prn);
            if (prnITER == mapPRN.end())
            {

                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "AMB will be removed! For Sat PRN " + _param->operator[](i).prn + " Epoch: " + _epoch.str_ymdhms());

#ifdef DEBUG
                std::cout << "AMB will be removed! For Sat PRN " << _param->operator[](i).prn
                     << " Epoch: " << _epoch.str_ymdhms() << std::endl;
#endif

                _amb_obs.erase(std::make_pair(_param->operator[](i).prn, _param->operator[](i).parType));

                _newAMB.erase(_param->operator[](i).prn);

                _Qx.Matrix_remRC(_param->operator[](i).index, _param->operator[](i).index);
                _param->delParam(i);
                _param->reIndex();
                i--;
            }
        }
    }

    return;
}
void hwa_gnss::gnss_proc_pvtflt::_udAmb()
{

    for (std::map<std::string, int>::iterator it = _newAMB.begin(); it != _newAMB.end();)
    {
        if (it->second > 2)
            _newAMB.erase(it++);
        else
            ++it;
    }

    if (_nSat == 0)
    {
        //zzwu add
        for (unsigned int i = 0; i < _param->parNumber(); i++)
        {
            if (_param->operator[](i).parType == par_type::AMB_IF ||
                _param->operator[](i).parType == par_type::AMB_L1 ||
                _param->operator[](i).parType == par_type::AMB_L2 ||
                _param->operator[](i).parType == par_type::AMB_L3 ||
                _param->operator[](i).parType == par_type::AMB_L4 ||
                _param->operator[](i).parType == par_type::AMB_L5)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "AMB will be removed! For Sat PRN " + _param->operator[](i).prn + " Epoch: " + _epoch.str_ymdhms());

#ifdef DEBUG
                std::cout << "AMB will be removed! For Sat PRN " << _param->operator[](i).prn
                    << " Epoch: " << _epoch.str_ymdhms() << std::endl;
#endif

                _amb_obs.erase(std::make_pair(_param->operator[](i).prn, _param->operator[](i).parType));

                _newAMB.erase(_param->operator[](i).prn);

                _Qx.Matrix_remRC(_param->operator[](i).index, _param->operator[](i).index);
                _param->delParam(i);
                _param->reIndex();
                i--;
            }
        }
        //
        return;
    }

    // Add ambiguity parameter and appropriate rows/columns covar. matrix
    std::set<std::string> mapPRN;

    std::vector<gnss_data_sats>::iterator it;
    for (it = _data.begin(); it != _data.end(); ++it)
    { // loop over all satellites
        mapPRN.insert(it->sat());

        std::tuple<GOBS, GOBS, GOBS, GOBS> amb_obs_identifier = std::make_tuple(X, X, X, X);

        GSYS gs = it->gsys();

        GOBSBAND b1 = _band_index[gs][FREQ_1];
        GOBSBAND b2 = _band_index[gs][FREQ_2];
        GOBSBAND b3 = _band_index[gs][FREQ_3];
        GOBSBAND b4 = _band_index[gs][FREQ_4];
        GOBSBAND b5 = _band_index[gs][FREQ_5];

        // GOBS tmp;
        //gnss_data_obs gobs1, gobs2, gobs3, gobs4, gobs5;
        //gnss_data_obs gobs1_P, gobs2_P, gobs3_P, gobs4_P, gobs5_P;

        gnss_data_obs gobs1(it->select_phase(b1));
        gnss_data_obs gobs2(it->select_phase(b2));
        gnss_data_obs gobs3(it->select_phase(b3));
        gnss_data_obs gobs4(it->select_phase(b4));
        gnss_data_obs gobs5(it->select_phase(b5));
        gnss_data_obs gobs1_P(it->select_range(b1));
        gnss_data_obs gobs2_P(it->select_range(b2));
        gnss_data_obs gobs3_P(it->select_range(b3));
        gnss_data_obs gobs4_P(it->select_range(b4));
        gnss_data_obs gobs5_P(it->select_range(b5));
        //std::cout << gobs1.attr() << " " << gobs1_P.attr() << " " << "gobs:" << (gobs1.attr() == gobs1_P.attr()) << std::endl;
        //std::cout << gobs2.attr() << " " << gobs2_P.attr() << " " << "gobs:" << (gobs2.attr() == gobs2_P.attr()) << std::endl;

        double LIF, L1, L2, L3, L4, L5;
        double PIF, P1, P2, P3, P4, P5;
        LIF = L1 = L2 = L3 = L4 = L5 = 0.0;
        PIF = P1 = P2 = P3 = P4 = P5 = 0.0;

        if (_observ == OBSCOMBIN::IONO_FREE || _observ == OBSCOMBIN::IF_P1)
        {
            LIF = it->L3(gobs1, gobs2);
            PIF = it->P3(gobs1_P, gobs2_P);
        }
        L1 = it->obs_L(gobs1);
        P1 = it->obs_C(gobs1_P);
        if ((_observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::IF_P1 || _observ == OBSCOMBIN::RAW_MIX) && !double_eq(L1, 0.0))
            it->tb12(true);
        L2 = it->obs_L(gobs2);
        P2 = it->obs_C(gobs2_P);
        if (!double_eq(L2, 0.0))
            it->tb12(true);
        L3 = it->obs_L(gobs3);
        P3 = it->obs_C(gobs3);
        if (!double_eq(L3, 0.0))
            it->tb13(true);
        L4 = it->obs_L(gobs4);
        P4 = it->obs_C(gobs4);
        if (!double_eq(L4, 0.0))
            it->tb14(true);
        L5 = it->obs_L(gobs5);
        P5 = it->obs_C(gobs5);
        if (!double_eq(L5, 0.0))
            it->tb15(true);

        if (!_isCompAug && _sagnss_coder_aug_map.size() > 20 && _cntrep == 1)
        {
            int count = 0;
            for (int i = 1; i <= 10; i++)
            {
                base_time t = _epoch - _sampling * i;
                //if (_sagnss_coder_aug_map[t].find(it->sat()) == _sagnss_coder_aug_map[t].end()) continue;
                if (!_sagnss_coder_aug_map[t][it->sat()])
                    count++;
            }
            if (count == 10)
            {
                it->addobs(gobs1.gobs(), 1);
                it->addobs(gobs2.gobs(), 1);
                std::cout << " AAA " << _epoch.str_hms() << "  " << it->sat() << std::endl;
            }
        }

        // bool com = true;
        // if (_gnav)
        //     com = _gnav->com();

        if (_observ == OBSCOMBIN::IONO_FREE || _observ == OBSCOMBIN::IF_P1)
        {
            int idx = _param->getParam(_site, par_type::AMB_IF, it->sat());
            if (idx < 0)
            {
                if (double_eq(LIF, 0.0) || double_eq(PIF, 0.0))
                    continue;

                base_par newPar(it->site(), par_type::AMB_IF, _param->parNumber() + 1, it->sat());

                newPar.value(LIF - PIF);           // first ambiguity value
                newPar.setTime(_epoch, LAST_TIME); // beg -> end
                _param->addParam(newPar);
                _newAMB[it->sat()] = 1;

                _Qx.Matrix_addRC(_param->parNumber() - 1, _param->parNumber() - 1);
                _Qx.matrixW()(_param->parNumber() - 1, _param->parNumber() - 1) = _sigAmbig * _sigAmbig;
                //std::cout << "ADD AMB IF " << it->sat() << " " << it->epoch().str_ymdhms() << std::endl;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "AMB_IF was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
            }
            else if (it->getlli(gobs1.gobs()) >= 1 || it->getlli(gobs2.gobs()) >= 1 || (_isClient && _param->operator[](idx).amb_ini == true && _param->operator[](idx).beg - _sampling < _epoch))
            {
                if (double_eq(LIF, 0.0) || double_eq(PIF, 0.0))
                    continue;
                it->addslip(true);
                _param->operator[](idx).value(LIF - PIF);
                _param->operator[](idx).setTime(_epoch, LAST_TIME); // beg -> end  (because of cycle slip)
                _Qx.matrixW()(idx, idx) = _sigAmbig * _sigAmbig;
                _newAMB[it->sat()] = 1;
            }

            // check amb_obs
            amb_obs_identifier = std::make_tuple(gobs1.gobs(), gobs2.gobs(), X, X);
            if (idx < 0 || it->getlli(gobs1.gobs()) >= 1 || it->getlli(gobs2.gobs()) >= 1)
                _amb_obs[std::make_pair(it->sat(), par_type::AMB_IF)] = amb_obs_identifier;
            else if (_amb_obs.find(std::make_pair(it->sat(), par_type::AMB_IF)) == _amb_obs.end())
            {
                //std::cerr << "Error: amb_obs not correct!" << it->sat() << " " << _epoch.str_hms() << std::endl;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "amb_obs not correct!" + it->sat() + " " + _epoch.str_hms());
            }
            else if (_amb_obs[std::make_pair(it->sat(), par_type::AMB_IF)] != amb_obs_identifier)
            {
                //std::cerr << "Warning: amb_obs switched silently!" << it->sat() << " " << _epoch.str_hms() << std::endl;
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "Warning: amb_obs switched silently!" + it->sat() + " " + _epoch.str_hms());
                _amb_obs[std::make_pair(it->sat(), par_type::AMB_IF)] = amb_obs_identifier;
            }
        }
        else if (_observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_DOUBLE || _observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
        {
            int newAmb = 0;
            // int nf = 5;
            // if (_observ == OBSCOMBIN::RAW_SINGLE)
            //     nf = 1;
            // else if (_observ == OBSCOMBIN::RAW_DOUBLE)
            //     nf = 2;
            // else if (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_MIX)
            //     nf = 5;
            //nf = band.size();
            for (const auto &ib : _band_index[gs])
            {
                //    base_par::t_type amb_type = base_par::AMB_IF;
                if (ib.first > _frequency)
                    continue;
                gnss_data_obs gobsi;
                double Li;
                double Pi;
                par_type amb_type;

                if (ib.first == FREQ_1)
                {
                    amb_type = par_type::AMB_L1;
                    Li = L1;
                    Pi = P1;
                    gobsi = gobs1;
                }
                else if (ib.first == FREQ_2)
                {
                    amb_type = par_type::AMB_L2;
                    Li = L2;
                    Pi = P2;
                    gobsi = gobs2;
                }
                else if (ib.first == FREQ_3)
                {
                    amb_type = par_type::AMB_L3;
                    Li = L3;
                    Pi = P3;
                    gobsi = gobs3;
                }
                else if (ib.first == FREQ_4)
                {
                    amb_type = par_type::AMB_L4;
                    Li = L4;
                    Pi = P4;
                    gobsi = gobs4;
                }
                else if (ib.first == FREQ_5)
                {
                    amb_type = par_type::AMB_L5;
                    Li = L5;
                    Pi = P5;
                    gobsi = gobs5;
                }
                else
                {
                    throw std::logic_error("Unknown band!!");
                };

                if (double_eq(Li, 0.0) || double_eq(Pi, 0.0))
                    continue;
                int idx = -1;
                // add L1 amb - everytime when RAW observations
                idx = _param->getParam(_site, amb_type, it->sat());
                if (idx < 0)
                {
                    base_par newPar(it->site(), amb_type, _param->parNumber() + 1, it->sat());
                    newPar.value(Li - Pi);
                    newPar.setTime(_epoch, LAST_TIME); // beg -> end
                    _param->addParam(newPar);

                    _Qx.Matrix_addRC(_param->parNumber() - 1, _param->parNumber() - 1);
                    _Qx.matrixW()(_param->parNumber() - 1, _param->parNumber() - 1) = _sigAmbig * _sigAmbig;
                    //         std::cout << "ADD AMB L1 " << it->sat() << " " << it->epoch().str_ymdhms()  << " " << L1 - cmpObs << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, "RAW AMB_L1 was added! For Sat PRN " + it->sat() + " Epoch: " + _epoch.str_ymdhms());
                    newAmb = 1;
                    //std::cout << "AMB will be Added! For Sat PRN " << it->sat()
                    //    << " Epoch: " << _epoch.str_ymdhms() << std::endl;
                }
                else if (it->getlli(gobsi.gobs()) >= 1 || (_isClient && _param->operator[](idx).amb_ini == true && _param->operator[](idx).beg - _sampling < _epoch)) // check cycle slip
                {
                    it->addslip(true);
                    _param->operator[](idx).value(Li - Pi);
                    _param->operator[](idx).setTime(_epoch, LAST_TIME); // beg -> end  (because of cycle slip)
                    _Qx.matrixW()(idx, idx) = _sigAmbig * _sigAmbig;
                    _newAMB[it->sat()] = 1;
                    //std::cout << "AMB will be reset! For Sat PRN " << it->sat()
                    //    << " Epoch: " << _epoch.str_ymdhms() << std::endl;
                    if (_param->operator[](idx).amb_ini == true)
                        std::cout << "reset AMB  " << _epoch.str_hms() << std::endl;
                }

                // check amb_obs
                amb_obs_identifier = std::make_tuple(gobsi.gobs(), X, X, X);
                if (idx < 0 || it->getlli(gobsi.gobs()) >= 1)
                    _amb_obs[std::make_pair(it->sat(), amb_type)] = amb_obs_identifier;
                else if (_amb_obs.find(std::make_pair(it->sat(), amb_type)) == _amb_obs.end())
                {
                    //std::cerr << "Error: amb_obs not correct!" << it->sat() << " " << _epoch.str_hms() << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, "amb_obs not correct!" + it->sat() + " " + _epoch.str_hms());
                }
                else if (_amb_obs[std::make_pair(it->sat(), amb_type)] != amb_obs_identifier)
                {
                    //std::cerr << "Warning: amb_obs switched silently!" << it->sat() << " " << _epoch.str_hms() << std::endl;
                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, "Warning: amb_obs switched silently!" + it->sat() + " " + _epoch.str_hms());
                    _amb_obs[std::make_pair(it->sat(), amb_type)] = amb_obs_identifier;
                }
            }

            if (newAmb)
                _newAMB[it->sat()] = 1;
        }

    } // end loop over all observations

    // Remove ambiguity parameter and appropriate rows/columns covar. matrix

    for (unsigned int i = 0; i < _param->parNumber(); i++)
    {
        if (_param->operator[](i).parType == par_type::AMB_IF ||
            _param->operator[](i).parType == par_type::AMB_L1 ||
            _param->operator[](i).parType == par_type::AMB_L2 ||
            _param->operator[](i).parType == par_type::AMB_L3 ||
            _param->operator[](i).parType == par_type::AMB_L4 ||
            _param->operator[](i).parType == par_type::AMB_L5)
        {

            std::set<std::string>::iterator prnITER = mapPRN.find(_param->operator[](i).prn);
            if (prnITER == mapPRN.end())
            {

                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "AMB will be removed! For Sat PRN " + _param->operator[](i).prn + " Epoch: " + _epoch.str_ymdhms());

#ifdef DEBUG
                std::cout << "AMB will be removed! For Sat PRN " << _param->operator[](i).prn
                     << " Epoch: " << _epoch.str_ymdhms() << std::endl;
#endif
                //std::cout << "AMB will be removed! For Sat PRN " << _param->operator[](i).prn
                //    << " Epoch: " << _epoch.str_ymdhms() << std::endl;
                _amb_obs.erase(std::make_pair(_param->operator[](i).prn, _param->operator[](i).parType));

                _newAMB.erase(_param->operator[](i).prn);

                _Qx.Matrix_remRC(_param->operator[](i).index, _param->operator[](i).index);
                _param->delParam(i);
                _param->reIndex();
                i--;
            }
        }
    }

    return;
}

void hwa_gnss::gnss_proc_pvtflt::_post_turbo_syncAmb()
{

    for (std::map<std::string, int>::iterator it = _newAMB.begin(); it != _newAMB.end();)
    {
        if (it->second > 2)
            _newAMB.erase(it++);
        else
            ++it;
    }

    gnss_proc_update_par_info update_info;
    _update_AMB->update_amb_pars(_epoch, *_param, _data, update_info);

    std::vector<int> remove_id;
    std::vector<base_par> newparlist;
    update_info.get(remove_id);
    update_info.get(newparlist);

    std::set<std::string> addSats;
    // First. // Add ambiguity parameter and appropriate rows/columns covar. matrix
    for (auto par : newparlist)
    {

        par.index = _param->parNumber() + 1;
        _param->addParam(par);
        _newAMB[par.prn] = 1;
        addSats.insert(par.prn);

        _Qx.Matrix_addRC(_param->parNumber() - 1, _param->parNumber() - 1);
        _Qx.matrixW()(_param->parNumber() - 1, _param->parNumber() - 1) = _sigAmbig * _sigAmbig;

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "AMB_IF was added! For Sat PRN " + par.prn + " Epoch: " + _epoch.str_ymdhms());
    }

    // Second. Remove old pars  // Remove ambiguity parameter and appropriate rows/columns covar. matrix
    sort(remove_id.begin(), remove_id.end());
    for (int i = 0; i < remove_id.size(); i++)
    {
        std::string prn = _param->operator[](remove_id[i] - i - 1).prn;
        int index = _param->operator[](remove_id[i] - i - 1).index;
        if (addSats.find(prn) == addSats.end())
        {
            _newAMB.erase(prn);
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "AMB will be removed! For Sat PRN " + prn + " Epoch: " + _epoch.str_ymdhms());
        }

        _Qx.Matrix_remRC(index, index);
        _param->delParam(remove_id[i] - i - 1);
        _param->reIndex();
    }
    return;
}

int hwa_gnss::gnss_proc_pvtflt::_get_spplsq_result(std::string site, base_time current, Triple &crd, double &clk)
{
    gnss_proc_spplsq *_spplsq = new gnss_proc_spplsq(site, _set, _gallobj, _spdlog, false);
    _spplsq->setDAT(_gobs, _gnav);
    _spplsq->spdlog(_spdlog);
    if (_spplsq->processBatch(current, current) <= 0)
    {

        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, site + current.str_ymdhms() + "calculate initial crd failed using spplsq");
        }
        delete _spplsq;
        _spplsq = nullptr;
        return -1;
    };
    crd = _spplsq->getCrd(current);
    clk = _spplsq->getRecClk(current);
    delete _spplsq;
    _spplsq = nullptr;
    return 1;
}

void hwa_gnss::gnss_proc_pvtflt::Add_rho_azel(/*const base_time & runEpoch,*/ const std::string &site_name, Triple &xyz_s, const Triple &xyz_r, gnss_data_sats &obs_sat)
{
    Triple xyz_rho = xyz_s - xyz_r;
    Triple ell_r, neu_s;
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
    if (_isBase && site_name != _site)
    {
        std::string tmp = _site;
        _site = site_name;
        _apply_tides(_epoch, xRec);
        _site = tmp;
    }
    else
        _apply_tides(_epoch, xRec);

    double tmp = (xyz_s - xRec).norm();

    obs_sat.addrho(tmp);
    //std::cout << iter->sat() << " " << iter->epoch().str_ymdhms() << std::fixed << std::setprecision(3) << " " << neu_s[0] << " " << neu_s[1] << " " << neu_s[2] << std::endl;
    double NE2 = neu_s[0] * neu_s[0] + neu_s[1] * neu_s[1];
    double ele = acos(sqrt(NE2) / tmp);
    if (neu_s[2] < 0.0)
    {
        ele *= -1.0;
    }

    if (sqrt(NE2) / tmp > 1.0)
        obs_sat.addele(0.0);
    else
        obs_sat.addele(ele);

    double azi = atan2(neu_s[1], neu_s[0]);
    if (azi < 0)
        azi += 2 * hwa_pi;
    obs_sat.addazi_rec(azi);
    //if (!_use_ecl) obs_sat.addecl(_lastEcl);

    return;
}

void hwa_gnss::gnss_proc_pvtflt::_prtOut(base_time &epoch, base_allpar &X, const Symmetric &Q, std::vector<gnss_data_sats> &data, std::ostringstream &os, xml_node &node, bool saveProd)
{

    // get CRD params
    Triple xyz, ell;
    X.getCrdParam(_site, xyz);
    if (_crd_est == CONSTRPAR::FIX)
    {
        std::shared_ptr<gnss_data_obj> grec = _gallobj->obj(_site);
        xyz = grec->crd_arp(epoch);
    }
    xyz2ell(xyz, ell, false);

    // CRD using eccentricities
    Triple xyz_ecc = xyz - _grec->eccxyz(epoch); // MARKER + ECC = ARP

    // get CRD rms  (XYZ)
    double Xrms = 0.0, Yrms = 0.0, Zrms = 0.0,
           Vxrms = 0.0, Vyrms = 0.0, Vzrms = 0.0;
    double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;
    int icrdx = _param->getParam(_site, par_type::CRD_X, "");
    int icrdy = _param->getParam(_site, par_type::CRD_Y, "");
    int icrdz = _param->getParam(_site, par_type::CRD_Z, "");
    int icrdclk = _param->getParam(_site, par_type::CLK, "");
    double clk = 0.0;
    if (icrdclk >= 0)
    {
        clk = X[icrdclk].value();
    }
    if (icrdx >= 0 && icrdy >= 0 && icrdz >= 0)
    {
        if (Q(icrdx + 1, icrdx + 1) < 0)
            Xrms = -1;
        else
            Xrms = sqrt(Q(icrdx + 1, icrdx + 1));
        if (Q(icrdy + 1, icrdy + 1) < 0)
            Yrms = -1;
        else
            Yrms = sqrt(Q(icrdy + 1, icrdy + 1));
        if (Q(icrdz + 1, icrdz + 1) < 0)
            Zrms = -1;
        else
            Zrms = sqrt(Q(icrdz + 1, icrdz + 1));
        cov_xy = Q(icrdy + 1, icrdx + 1);
        cov_xz = Q(icrdz + 1, icrdx + 1);
        cov_yz = Q(icrdz + 1, icrdy + 1);
    }

    Triple crd_rms(Xrms, Yrms, Zrms);
    Triple vRec(0, 0, 0);

    if (_doppler)
    {
        vRec = _vel;
        if (_Qx_vel(1, 1) < 0)
            Vxrms = -1;
        else
            Vxrms = sqrt(_Qx_vel(1, 1));
        if (_Qx_vel(2, 2) < 0)
            Vyrms = -1;
        else
            Vyrms = sqrt(_Qx_vel(2, 2));
        if (_Qx_vel(3, 3) < 0)
            Vzrms = -1;
        else
            Vzrms = sqrt(_Qx_vel(3, 3));
    }
    else
    {
        vRec = Triple(0.0, 0.0, 0.0);
    }

    // get CLK param
    // double clk = 0.0;
    // int iclk = X.getParam(_site, par_type::CLK, "");
    // if (iclk >= 0)
    //     clk = X[iclk].value();

    // get ZTD params
    // double ZHD = 0.0, ZWD = 0.0;
    // int iztd = X.getParam(_site, par_type::TRP, "");
    // if (iztd >= 0)
    // {
        // ZHD = X[iztd].zhd; //_gModel->tropoModel()->getZHD(ell, epoch);
        // ZWD = X[iztd].value();
        // // ZTD = ZHD + ZWD;
        // if (Q(iztd + 1, iztd + 1) < 0)
        //     ZWDrms = -1;
        // else
        //     ZWDrms = sqrt(Q(iztd + 1, iztd + 1)); // _param is indexed from 0, but Q from 1 !
    // }
    // else
    // {
    //     // test purpose only (if ZTD not estimated)
    //     if (_gModel->tropoModel() != 0)
    //     {
    //         ZWD = _gModel->tropoModel()->getZWD(ell, epoch);
    //         ZHD = _gModel->tropoModel()->getZHD(ell, epoch);
            // ZTD = ZHD + ZWD;
    //     }
    // }

    // // get Tropo gradient
    // int ign = X.getParam(_site, par_type::GRD_N, "");
    // int ige = X.getParam(_site, par_type::GRD_E, "");
    // // double grdN = 0.0, grdE = 0.0;
    // //  double rmsN = 0.0, rmsE = 0.0;
    // if (ign >= 0 && ige >= 0)
    // {
    //     // grdN = X[ign].value();
    //     // grdE = X[ige].value();
    //     //    rmsN = sqrt(Q(ign+1,ign+1));                  // _param is indexed from 0, but Q from 1 !
    //     //    rmsE = sqrt(Q(ige+1,ige+1));                  // _param is indexed from 0, but Q from 1 !
    // }

    std::set<std::string> sat_list;
    for (auto it = _data.begin(); it != _data.end(); it++)
        sat_list.insert(it->sat());
    _dop.set_sats(sat_list);
    double pdop = -1, hdop = -1;
    if (_dop.calculate(epoch, xyz) >= 0)
    {
        // gdop = _dop.gdop();
        pdop = _dop.pdop();
        hdop = _dop.hdop();
    }

    std::set<std::string> ambs = X.amb_prns();
    int nsat = ambs.size();

    // get amb status
    std::string amb = "Float";
    if (_amb_state)
        amb = "Fixed";

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
        if (_param->getCrdParam(_site, XYZ) > 0)
        {
        }
        else if (_valid_crd_xml)
        {
            XYZ = _grec->crd_arp(_epoch);
        }
        xyz2ell(XYZ, Ell, false);
        int itrp = _param->getParam(_site, par_type::TRP, "");
        if (itrp >= 0)
            _param->operator[](itrp).apriori(_gModel->tropoModel()->getZHD(Ell, _epoch));
    }
    Triple blh; 
    xyz2ell(xyz_ecc, blh, true);
    double crt = epoch.sow() + epoch.dsec();
    Triple Qpos(Xrms*Xrms, Yrms*Yrms, Zrms*Zrms), Qvel(Vxrms*Vxrms, Vyrms*Vyrms, Vzrms*Vzrms);
    Triple position(xyz_ecc[0], xyz_ecc[1], xyz_ecc[2]), velocity(vRec[0], vRec[1], vRec[2]);
    base_posdata::data_pos posdata = base_posdata::data_pos{ crt, position, velocity, Qpos, Qvel, pdop, nsat, _amb_state };
    bool ins = dynamic_cast<set_inp *>(_set)->input_size("imu") > 0 ? true : false;
    // write kml
    if (_kml && !ins)
    {
        std::ostringstream out;
        // zzwu for >180 lon
        Triple ell1(ell);
        if (ell1[1] > hwa_pi)
            ell1[1] = ell1[1] - 2 * hwa_pi;
        out << std::fixed << std::setprecision(11) << " " << std::setw(0) << ell1[1] * R2D << ',' << ell1[0] * R2D;
        // zzwu
        // out << std::fixed << std::setprecision(11) << " " << std::setw(0) << ell[1] * R2D << ',' << ell[0] * R2D;
        std::string val = out.str();

        xml_node root = _doc;
        node = this->_default_node(root, _root.c_str());
        xml_node document = node.child("Document");
        xml_node last_child = document.last_child();
        xml_node placemark = document.insert_child_after("Placemark", last_child);
        std::string q = "#P" + _quality_grade(posdata);
        this->_default_node(placemark, "styleUrl", q.c_str());
        this->_default_node(placemark, "time", base_type_conv::int2str(_epoch.sow()).c_str());
        xml_node point = this->_default_node(placemark, "Point");
        this->_default_node(point, "coordinates", val.c_str()); // for point
        xml_node description = placemark.append_child("description");
        description.append_child(pugi::node_cdata).set_value(_gen_kml_description(epoch, posdata).c_str());
        xml_node TimeStamp = placemark.append_child("TimeStamp");
        std::string time = base_type_conv::trim(epoch.str_ymd()) + "T" + base_type_conv::trim(epoch.str_hms()) + "Z";
        this->_default_node(TimeStamp, "when", time.c_str());


        xml_node Placemark = document.child("Placemark");
        xml_node LineString = Placemark.child("LineString");
        this->_default_node(LineString, "coordinates", val.c_str(), false); // for line
    }

    std::string str_dsec = base_type_conv::dbl2str(epoch.dsec());

    double bl = 0;
    if (_isBase)
    {
        auto crd_base = _gallobj->obj(_site_base)->crd_arp(epoch);
        Triple tmpell, tmpdxyz, tmpneu;
        xyz2ell(crd_base, tmpell, false);
        tmpdxyz = xyz - crd_base;
        xyz2neu(tmpell, tmpdxyz, tmpneu);
        bl = tmpneu.norm();
    }

    os << std::fixed << std::setprecision(4) << " "
       // << epoch.str_ymdhms() << str_dsec.substr(2) << std::setprecision(4)
       << " " << epoch.sow() + epoch.dsec();
    if (_crd_est != CONSTRPAR::FIX)
    {
        os << std::fixed << std::setprecision(4)
           << " " << std::setw(15) << xyz_ecc[0] // [m]
           << " " << std::setw(15) << xyz_ecc[1] // [m]
           << " " << std::setw(15) << xyz_ecc[2] // [m]
           << " " << std::setw(10) << vRec[0]    // [m/s]
           << " " << std::setw(10) << vRec[1]    // [m/s]
           << " " << std::setw(10) << vRec[2]    // [m/s]
           //<< " " << std::setw(12) << dclk_Rec
           //<< " " << std::setw(9) << ZTD             // [m]
           //<< " " << std::setw(9) << grdN * 1000       // [mm]
           //<< " " << std::setw(9) << grdE * 1000       // [mm]
           //<< " " << std::setw(17) << clk //<< (clk/CLIGHT)*1e6             // [m]
           << std::fixed << std::setprecision(4)
           << " " << std::setw(9) << Xrms  // [m]
           << " " << std::setw(9) << Yrms  // [m]
           << " " << std::setw(9) << Zrms  // [m]
           << " " << std::setw(9) << Vxrms // [m/s]
           << " " << std::setw(9) << Vyrms // [m/s]
           << " " << std::setw(9) << Vzrms // [m/s]
            //<< " " << std::setw(7) << ZWDrms          // [m]
            ;
    }
    os << std::fixed << std::setprecision(0)
       << " " << std::setw(5) << nsat // nsat
       << std::fixed << std::setprecision(1)
       //<< " " << std::setw(3) << gdop            // gdop
       << std::fixed << std::setprecision(2)
       << " " << std::setw(5) << pdop // pdop
       << std::fixed << std::setprecision(2)
       << " " << std::setw(8) << _sig_unit // m0
       << " " << std::setw(8) << amb;
    if (_fix_mode != FIX_MODE::NO)
        os << std::setprecision(2) << " " << std::fixed << std::setw(10) << _ambfix->get_ratio();
    if (_isBase)
        os << " " << std::fixed << std::setprecision(3) << std::setw(10) << bl; //lvhb
    os << " " << std::setw(8) << _quality_grade(posdata);
    os << " " << std::setw(8)<< std::setprecision(4) << clk;
    if (_observ == OBSCOMBIN::RAW_MIX)
    {
        std::map<std::string, int> satsnum = _param->freq_sats_num(2);
        os << std::setw(8) << satsnum["Single"] << std::setw(8) << satsnum["Double"];
    }
    os << std::endl;
    // write enu
    Triple xyz_ecc1 = xyz_ecc;
    if (_enufile)
    {
        Triple dxyz, ell, neu;

        std::ostringstream enu_info;
        //double crt_t = epoch.sow() + epoch.dsec();
        double crt_t = epoch.sod() + epoch.dsec();

        if (_crd_est == CONSTRPAR::KIN || _pos_kin)
        {
            // just for now
            enu_info << _site << " " << std::fixed << std::setw(8) << epoch.mjd() << std::fixed << std::setw(10) << std::setprecision(1) << crt_t;
            //enu_info << std::fixed << std::setw(10) << std::setprecision(1) << crt_t;
            enu_info << std::setw(16) << std::setprecision(4) << xyz_ecc1[0]
                     << std::setw(16) << std::setprecision(4) << xyz_ecc1[1]
                     << std::setw(16) << std::setprecision(4) << xyz_ecc1[2] << "  " << nsat << std::endl;
        }
        else
        {
            enu_info << std::fixed << std::setw(10) << std::setprecision(1) << crt_t;
            xyz2ell(xyz_standard, ell, false);
            dxyz = xyz_ecc1 - xyz_standard;
            xyz2neu(ell, dxyz, neu);

            // RMS
            neu_sum_nepo.first++;                                                                          // nepoch
            neu_sum_nepo.second += Triple(pow(neu[0], 2), pow(neu[1], 2), pow(neu[2], 2)); // sum

            enu_info << std::fixed << std::setw(16) << std::setprecision(4) << neu[1]
                     << std::setw(16) << std::setprecision(4) << neu[0]
                     << std::setw(16) << std::setprecision(4) << neu[2]
                     << " " << std::setw(8) << amb << std::endl;
        }
        _enufile->write(enu_info.str().c_str(), enu_info.str().size());
        _enufile->flush();
    }

    //write gpgga by lvhb in 20210312
    if (_gpggafile)
    {
        std::ostringstream gpgga_info;
        base_time utcT = epoch;
        utcT.tsys(base_time::base_timesys::UTC);
        int hour = utcT.hour();
        int min = utcT.mins();
        double sec = utcT.secs() + utcT.dsec();
        std::string lat = ",N,", lon = ",E,";
        if (blh[0] < 0)
            lat = ",S,";
        if (blh[1] < 0)
            lon = ",W,";
        int Q = 5;
        if (_amb_state)
            Q = 4;
        //for six cent
        gpgga_info << "$GPGGA," << std::setw(2) << std::setfill('0') << hour
                   << std::setw(2) << std::setfill('0') << min
                   //<< std::setw(6) << setfill('0') << std::setiosflags(ios::fixed) << std::setprecision(3) << sec << ","
                   << std::setw(5) << std::setfill('0') << std::setiosflags(std::ios::fixed) << std::setprecision(2) << sec << ","
                   << std::setw(2) << std::setfill('0') << int(blh[0])
                   //<< std::setw(7) << setfill('0') << std::setprecision(4) << fmod(blh[0] * 60, 60) << lat
                   << std::setw(10) << std::setfill('0') << std::setprecision(7) << fmod(blh[0] * 60, 60) << lat
                   << std::setw(3) << std::setfill('0') << int(blh[1])
                   //<< std::setw(7) << std::setprecision(4) << setfill('0') << fmod(60 * blh[1], 60) << lon
                   << std::setw(10) << std::setprecision(7) << std::setfill('0') << fmod(60 * blh[1], 60) << lon
                   << std::setw(1) << Q << "," << std::setw(2) << std::setfill('0') << nsat << ','
                   << std::setw(3) << std::setprecision(1) << hdop << ','
                   << std::setprecision(3) << blh[2] << ",M,0.0,M,,";

        unsigned int CRC = 0;
        std::string strtmp = gpgga_info.str();
        for (int i = 0; i < strtmp.length(); i++)
        {
            char ctmp = strtmp.at(i);
            CRC = CRC ^ (unsigned int)(ctmp);
        }
        gpgga_info << "*" << std::setiosflags(std::ios::uppercase /*|ios::hex*/) << std::hex << CRC;
        gpgga_info << "\r";

        _gpggafile->write(gpgga_info.str().c_str(), gpgga_info.str().size());
        _gpggafile->flush();
    }

    return;
}

void hwa_gnss::gnss_proc_pvtflt::_prt_port(base_time &epoch, base_allpar &X, const Symmetric &Q, std::vector<gnss_data_sats> &data)
{

    std::ostringstream os;
    // get CRD params
    Triple xyz, ell;
    X.getCrdParam(_site, xyz);
    if (_crd_est == CONSTRPAR::FIX)
    {
        std::shared_ptr<gnss_data_obj> grec = _gallobj->obj(_site);
        xyz = grec->crd_arp(epoch);
    }
    xyz2ell(xyz, ell, false);

    // CRD using eccentricities
    Triple xyz_ecc = xyz - _grec->eccxyz(epoch); // MARKER + ECC = ARP

    double Xrms = 0.0, Yrms = 0.0, Zrms = 0.0;
    int icrdx = _param->getParam(_site, par_type::CRD_X, "");
    int icrdy = _param->getParam(_site, par_type::CRD_Y, "");
    int icrdz = _param->getParam(_site, par_type::CRD_Z, "");
    if (icrdx >= 0 && icrdy >= 0 && icrdz >= 0)
    {
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
    }

    Triple crd_rms(Xrms, Yrms, Zrms);
    Triple vRec(0, 0, 0);
    if (_doppler)
    {
        vRec = _vel;
        // if (_Qx_vel(1, 1) < 0)
        //     Vxrms = -1;
        // else
        //     Vxrms = sqrt(_Qx_vel(1, 1));
        // if (_Qx_vel(2, 2) < 0)
        //     Vyrms = -1;
        // else
        //     Vyrms = sqrt(_Qx_vel(2, 2));
        // if (_Qx_vel(3, 3) < 0)
        //     Vzrms = -1;
        // else
        //     Vzrms = sqrt(_Qx_vel(3, 3));
    }
    else
    {
        vRec = Triple(0.0, 0.0, 0.0);
    }

    std::set<std::string> sat_list;
    for (auto it = _data.begin(); it != _data.end(); it++)
        sat_list.insert(it->sat());
    _dop.set_sats(sat_list);
    double pdop = -1;
    if (_dop.calculate(epoch, xyz) >= 0)
        pdop = _dop.pdop();

    std::set<std::string> ambs = X.amb_prns();
    int nsat = ambs.size();

    // get amb status
    std::string amb = "Float";
    if (_amb_state)
        amb = "Fixed";

    Triple blh;
    xyz2ell(xyz_ecc, blh, true);

    std::string str_dsec = base_type_conv::dbl2str(epoch.dsec());

    // double bl = 0;
    if (_isBase)
    {
        auto crd_base = _gallobj->obj(_site_base)->crd_arp(epoch);
        Triple tmpell, tmpdxyz, tmpneu;
        xyz2ell(crd_base, tmpell, false);
        tmpdxyz = xyz - crd_base;
        xyz2neu(tmpell, tmpdxyz, tmpneu);
        // bl = tmpneu.norm();
    }

    os << std::fixed << std::setprecision(4) << " "
       // << epoch.str_ymdhms() << str_dsec.substr(2) << std::setprecision(4)
       << " " << epoch.sow() + epoch.dsec()
       << std::fixed << std::setprecision(4)
       << " " << std::setw(15) << xyz_ecc[0] // [m]
       << " " << std::setw(15) << xyz_ecc[1] // [m]
       << " " << std::setw(15) << xyz_ecc[2] // [m]
       << " " << std::setw(10) << vRec[0]    // [m/s]
       << " " << std::setw(10) << vRec[1]    // [m/s]
       << " " << std::setw(10) << vRec[2]    // [m/s]
       << std::fixed << std::setprecision(0)
       << " " << std::setw(5) << nsat // nsat
       << std::fixed << std::setprecision(2)
       << " " << std::setw(5) << pdop // pdop
       << std::fixed << std::setprecision(2)
       << " " << std::setw(8) << amb
       << std::setw(10) << (_amb_state ? _ambfix->get_ratio() : 0.0)
       << std::endl;
    bool ins = dynamic_cast<set_inp *>(_set)->input_size("imu") > 0 ? true : false;
    if (!ins && _maptcp.find(FLT_OUT) != _maptcp.end())
        _maptcp[FLT_OUT]->run_send(os.str());
}

void hwa_gnss::gnss_proc_pvtflt::_get_result(base_time &epoch, base_posdata::data_pos &pos)
{

    double crt = epoch.sow() + epoch.dsec();
    base_allpar X = _param_fixed;
    Symmetric Qx;
    if (_amb_state) Qx = _filter->Qx();
    else Qx = _Qx;
    // get CRD params
    Triple xyz, xyz_ecc;
    // get CRD rms  (XYZ)
    double Xrms = 0.0, Yrms = 0.0, Zrms = 0.0,
           Vxrms = 0.0, Vyrms = 0.0, Vzrms = 0.0;

    if (X.getCrdParam(_site, xyz) > 0)
    {
        // CRD using eccentricities
        xyz_ecc = xyz - _grec->eccxyz(epoch); // MARKER + ECC = ARP

        int icrdx = _param->getParam(_site, par_type::CRD_X, "");
        int icrdy = _param->getParam(_site, par_type::CRD_Y, "");
        int icrdz = _param->getParam(_site, par_type::CRD_Z, "");
        if (Qx(icrdx, icrdx) < 0)
            Xrms = -1;
        else
            Xrms = sqrt(Qx(icrdx, icrdx));
        if (Qx(icrdy, icrdy) < 0)
            Yrms = -1;
        else
            Yrms = sqrt(Qx(icrdy, icrdy));
        if (Qx(icrdz, icrdz) < 0)
            Zrms = -1;
        else
            Zrms = sqrt(Qx(icrdz, icrdz));
    }
    else
    {
        xyz_ecc = _grec->crd_arp(epoch);
        Xrms = Yrms = Zrms = 30;
    }
    // get VEL rms
    Triple vRec(0, 0, 0);
    if (_doppler)
    {
        vRec = _vel;
        if (_Qx_vel(0, 0) < 0)
            Vxrms = -1;
        else
            Vxrms = sqrt(_Qx_vel(0, 0));
        if (_Qx_vel(1, 1) < 0)
            Vyrms = -1;
        else
            Vyrms = sqrt(_Qx_vel(1, 1));
        if (_Qx_vel(2, 2) < 0)
            Vzrms = -1;
        else
            Vzrms = sqrt(_Qx_vel(2, 2));
    }
    else
    {
        vRec = Triple(0.0, 0.0, 0.0);
        Vxrms = Vyrms = Vzrms = 0;
    }

    std::set<std::string> sat_list;
    for (auto it = _data.begin(); it != _data.end(); it++)
        sat_list.insert(it->sat());
    _dop.set_sats(sat_list);
    double pdop = -1;
    if (_dop.calculate(epoch, xyz) >= 0)
        pdop = _dop.pdop();

    std::set<std::string> ambs = X.amb_prns();
    int nsat = ambs.size();

    Triple Qpos(Xrms*Xrms, Yrms*Yrms, Zrms*Zrms), Qvel(Vxrms*Vxrms, Vyrms*Vyrms, Vzrms*Vzrms);
    Triple position(xyz_ecc[0], xyz_ecc[1], xyz_ecc[2]), velocity(vRec[0], vRec[1], vRec[2]);

    pos = base_posdata::data_pos{crt, position, velocity, Qpos, Qvel, pdop, nsat, _amb_state, _sig_unit};
}

void hwa_gnss::gnss_proc_pvtflt::_get_result(base_time & epo, base_posdata::rtk_pos & pos)
{
    //get base coordinate
    std::shared_ptr<gnss_data_obj> grec_base = _gallobj->obj(_site_base);
    //gtrace("gnss_proc_pvtflt::_get_result");
    Triple X_base = grec_base->crd(epo);
    Triple x_base;
    x_base(0) = X_base[0];
    x_base(1) = X_base[1];
    x_base(2) = X_base[2];

    double crt = epo.sow() + epo.dsec();
    base_allpar X = _param_fixed;
    Symmetric Qx;
    if (_amb_state) Qx = _filter->Qx();
    else Qx = _Qx;
    // get CRD params
    Triple xyz, xyz_ecc;
    // get CRD rms  (XYZ)
    double Xrms = 0.0, Yrms = 0.0, Zrms = 0.0;

    if (X.getCrdParam(_site, xyz) > 0)
    {
        // CRD using eccentricities
        xyz_ecc = xyz - _grec->eccxyz(epo); // MARKER + ECC = ARP

        int icrdx = _param->getParam(_site, par_type::CRD_X, "");
        int icrdy = _param->getParam(_site, par_type::CRD_Y, "");
        int icrdz = _param->getParam(_site, par_type::CRD_Z, "");
        if (Qx(icrdx, icrdx) < 0)
            Xrms = -1;
        else
            Xrms = sqrt(Qx(icrdx, icrdx));
        if (Qx(icrdy, icrdy) < 0)
            Yrms = -1;
        else
            Yrms = sqrt(Qx(icrdy, icrdy));
        if (Qx(icrdz, icrdz) < 0)
            Zrms = -1;
        else
            Zrms = sqrt(Qx(icrdz, icrdz));
    }
    else
    {
        xyz_ecc = _grec->crd_arp(epo);
        Xrms = Yrms = Zrms = 30;
    }
    // get VEL rms
    Triple vRec(0, 0, 0);
    if (_doppler)
    {
        vRec = _vel;
        // if (_Qx_vel(1, 1) < 0)
        //     Vxrms = -1;
        // else
        //     Vxrms = sqrt(_Qx_vel(1, 1));
        // if (_Qx_vel(2, 2) < 0)
        //     Vyrms = -1;
        // else
        //     Vyrms = sqrt(_Qx_vel(2, 2));
        // if (_Qx_vel(3, 3) < 0)
        //     Vzrms = -1;
        // else
        //     Vzrms = sqrt(_Qx_vel(3, 3));
    }
    else
    {
        vRec = Triple(0.0, 0.0, 0.0);
    }

    std::set<std::string> sat_list;
    for (auto it = _data.begin(); it != _data.end(); it++)
        sat_list.insert(it->sat());
    _dop.set_sats(sat_list);
    double pdop = -1;
    if (_dop.calculate(epo, xyz) >= 0)
        pdop = _dop.pdop();

    std::set<std::string> ambs = X.amb_prns();
    int nsat = ambs.size();

    //Triple Qpos(Xrms*Xrms, Yrms*Yrms, Zrms*Zrms), Qvel(Vxrms*Vxrms, Vyrms*Vyrms, Vzrms*Vzrms);
    //Triple position(xyz_ecc[0], xyz_ecc[1], xyz_ecc[2]), velocity(vRec[0], vRec[1], vRec[2]);
    Triple dx = xyz_ecc - X_base;
    Triple baseline(dx[0], dx[1], dx[2]);
    Triple Rbaseline(Xrms*Xrms, Yrms*Yrms, Zrms*Zrms);

    pos = base_posdata::rtk_pos{ crt, baseline, Rbaseline, x_base, dx.norm(), pdop, nsat, _amb_state, _sig_unit };
}

void hwa_gnss::gnss_proc_pvtflt::_get_result(base_time & epo, Vector & pos)
{
    pos = _vBanc;
}

// added by zhshen
void hwa_gnss::gnss_proc_pvtflt::_prtOutHeader()
{

    std::ostringstream os;

    //lvhb added in 20200818
    if (_isBase)
    {
        auto beg = dynamic_cast<set_gen *>(_set)->beg();
        auto crd_base = _gallobj->obj(_site_base)->crd(beg);
        os << "# base -" << _site_base << " Pos(XYZ): " << std::fixed << std::setprecision(4) << crd_base[0] << std::setw(15) << crd_base[1] << std::setw(15) << crd_base[2] << std::endl;
        if (_crd_est == CONSTRPAR::FIX)
        {
            auto crd = _gallobj->obj(_site)->crd(beg);
            os << "# rover-" << _site << " Pos(XYZ): " << std::fixed << std::setprecision(4) << crd[0] << std::setw(15) << crd[1] << std::setw(15) << crd[2] << std::endl;
        }
    }

    // first line
    os << "#" << std::setw(15) << "Seconds of Week";
    if (_crd_est != CONSTRPAR::FIX)
    {
        os << " " << std::setw(12) << "X-ECEF " << // [m]
            " " << std::setw(15) << "Y-ECEF" <<      // [m]
            " " << std::setw(15) << "Z-ECEF" <<      // [m]
            " " << std::setw(10) << "Vx-ECEF" <<      // [m/s]
            " " << std::setw(10) << "Vy-ECEF" <<      // [m/s]
            " " << std::setw(10) << "Vz-ECEF" <<      // [m/s]
                                              //<< " " << std::setw(12) << dclk_Rec
                                              //<< " " << std::setw(9) << ZTD             // [m]
                                              //<< " " << std::setw(9) << grdN * 1000       // [mm]
                                              //<< " " << std::setw(9) << grdE * 1000       // [mm]
                                              //<< " " << std::setw(17) << clk //<< (clk/CLIGHT)*1e6             // [m]
            " " << std::setw(9) << "X-RMS"
           << " " << std::setw(9) << "Y-RMS"
           << " " << std::setw(9) << "Z-RMS"
           << " " << std::setw(9) << "Vx-RMS"
           << " " << std::setw(9) << "Vy-RMS"
           << " " << std::setw(9) << "Vz-RMS";
    }
    os << " " << std::setw(5) << "NSat"
       << " " << std::setw(5) << "PDOP"
       << " " << std::setw(8) << "sigma0"
       << " " << std::setw(10) << "AmbStatus";
    if (_fix_mode != FIX_MODE::NO)
        os << " " << std::setw(10) << "Ratio"; //lvhb added
    if (_isBase)
        os << " " << std::setw(10) << "BL"; //lvhb added
    os << " " << std::setw(8) << "Quality"; //zhshen added
    os << std::endl;

    // second line
    os << "#" << std::setw(15) << "(s)";
    if (_crd_est != CONSTRPAR::FIX)
    {
        os << " " << std::setw(12) << "(m)" << // [m]
            " " << std::setw(15) << "(m)" <<      // [m]
            " " << std::setw(15) << "(m)" <<      // [m]
            " " << std::setw(10) << "(m/s)" << // [m/s]
            " " << std::setw(10) << "(m/s)" << // [m/s]
            " " << std::setw(10) << "(m/s)" << // [m/s]
                                          //<< " " << std::setw(12) << dclk_Rec
                                          //<< " " << std::setw(9) << ZTD             // [m]
                                          //<< " " << std::setw(9) << grdN * 1000       // [mm]
                                          //<< " " << std::setw(9) << grdE * 1000       // [mm]
                                          //<< " " << std::setw(17) << clk //<< (clk/CLIGHT)*1e6             // [m]
            " " << std::setw(9) << "(m)" <<      // [m]
            " " << std::setw(9) << "(m)" <<      // [m]
            " " << std::setw(9) << "(m)" <<      // [m]
            " " << std::setw(9) << "(m/s)" <<  // [m/s]
            " " << std::setw(9) << "(m/s)" <<  // [m/s]
            " " << std::setw(9) << "(m/s)"      // [m/s]
            ;
    }
    os << " " << std::setw(5) << "(#)"
       << " " << std::setw(5) << "(#)"
       << " " << std::setw(8) << "(m)"
       << " " << std::setw(10) << " ";
    if (_fix_mode != FIX_MODE::NO)
        os << std::setw(10) << " "; //lvhb added
    if (_isBase)
        os << " " << std::setw(10) << "(m)"; //lvhb added
    os << std::setw(8) << " ";
    os << std::endl;

    // Print flt results
    if (_flt)
    {
        _flt->write(os.str().c_str(), os.str().size());
        _flt->flush();
    }
}

void hwa_gnss::gnss_proc_pvtflt::_combineEWL(gnss_data_sats &satdata)
{
}

void hwa_gnss::gnss_proc_pvtflt::_generateObsIndex(gnss_proc_lsq_equationmatrix &equ)
{
    //_obs_index.clear();
    auto band_index = dynamic_cast<gnss_model_comb *>(&(*_base_model))->get_band_index();
    auto freq_index = dynamic_cast<gnss_model_comb *>(&(*_base_model))->get_freq_index();
    for (int i = 0; i < equ.num_equ(); i++)
    {
        char sys_char = equ.get_satname(i)[0];
        GSYS sys;
        bool sys_valid = true;
        switch (sys_char)
        {
        case 'G':
            sys = GSYS::GPS;
            break;
        case 'R':
            sys = GSYS::GLO;
            break;
        case 'E':
            sys = GSYS::GAL;
            break;
        case 'C':
            sys = GSYS::BDS;
            break;
        case 'J':
            sys = GSYS::QZS;
            break;
        default:
            sys_valid = false;
            break;
        }
        if (!sys_valid)
            throw std::exception();
        GOBSBAND b = equ.get_obscombtype(i).getBand_1();
        FREQ_SEQ f = freq_index[sys][b];
        GOBSTYPE obstype = equ.get_obscombtype(i).is_code() ? TYPE_C : GOBSTYPE::TYPE_L;
        _obs_index.push_back(std::make_pair(equ.get_satname(i), std::make_pair(f, obstype)));
    }
}

void hwa_gnss::gnss_proc_pvtflt::_equationToMatrix(gnss_proc_lsq_equationmatrix &equ, Matrix &A, Symmetric &P,
                                         Vector &l, unsigned int nPar)
{
    unsigned nObs = equ.num_equ();
    A.resize(nObs, nPar);
    A.setZero();
    P.resize(nObs);
    P.setZero();
    l.resize(nObs);
    l.setZero();
    for (int i = 0; i < equ.B.size(); i++)
    {
        for (int j = 0; j < equ.B[i].size(); j++)
        {
            if (equ.B[i][j].first >= nPar)
                continue;
            A(i, equ.B[i][j].first) = equ.B[i][j].second;
        }
        l(i) = equ.l[i];
        P.matrixW()(i, i) = equ.P[i];
        //std::cout << equ.get_satname(i) << " " << equ.get_obscombtype(i).getBand() << " " << std::fixed << equ.l[i] << std::endl;
    }
}

void hwa_gnss::gnss_proc_pvtflt::_extern_ion_constraint(const base_time &runEpoch)
{
    _epoch_cout++;
    hwa_gnss_model_iono_tecgrid iono_tec;
    std::vector<gnss_data_sats>::iterator it = _data.begin();
    while (it != _data.end())
    {
        int ipar_sion = _param->getParam(_site, par_type::SION, it->sat());
        if (ipar_sion >= 0)
        {
            double ion_L1, ion_rms;
            Triple site_pos = _vBanc.segment(0, 3);
            if (iono_tec.getIonoDelay(_gionex_GIM, *it, runEpoch, site_pos, ion_L1, ion_rms))
            {
                _param->operator[](ipar_sion).value(ion_L1);
                _Qx.matrixW()(ipar_sion, ipar_sion) = 1.0 + 0.1 * _epoch_cout;
            }
        }
        ++it;
    }
}

bool hwa_gnss::gnss_proc_pvtflt::_calculate_timediff_baseline(const std::string &site, const base_time &epoch0, const base_time &epoch1, Triple &baseline)
{
    std::vector<gnss_data_sats> data_t0 = _gobs->obs(site, epoch0);
    std::vector<gnss_data_sats> data_t1 = _gobs->obs(site, epoch1);

    if (data_t0.size() < 5 || data_t1.size() < 5)
        return false;

    Triple crd_t0, crd_t1;
    base_allpar param_t0;
    base_allpar param_t1;

    // solve bancroft to get approximate coordinates
    Vector spp_t0, spp_t1;
    _preprocess(site, data_t0);
    spp_t0 = _vBanc;
    _preprocess(site, data_t1);
    spp_t1 = _vBanc;

    int max_obs = 1000;
    Matrix B = Matrix::Zero(max_obs, 4);
    Vector l = Vector::Zero(max_obs);
    Vector x;
    int iobs = 0;

    Triple rr_t0, rr_t1;
    rr_t0 = Triple(spp_t0[0], spp_t0[1], spp_t0[2]);
    rr_t1 = Triple(spp_t1[0], spp_t1[1], spp_t1[2]);

    // use only L1 carrier phase data
    // construct a least-square problem
    for (auto &satdata_t0 : data_t0)
    {
        std::string sat = satdata_t0.sat();
        for (auto &satdata_t1 : data_t1)
        {
            if (sat == satdata_t1.sat())
            {
                std::vector<GOBSBAND> band = dynamic_cast<set_gnss *>(_set)->band(satdata_t0.gsys());
                GOBSBAND b1 = band[0];
                gnss_data_obs gobsL1 = gnss_data_obs(satdata_t0.select_phase(b1));
                double obs_L1_t0 = satdata_t0.obs_L(gobsL1);
                double obs_L1_t1 = satdata_t1.obs_L(gobsL1);
                if (double_eq(obs_L1_t0, 0.0) || double_eq(obs_L1_t1, 0.0) || satdata_t0.lli().at(gobsL1.gobs()) > 0 || satdata_t1.lli().at(gobsL1.gobs()) > 0)
                    continue;

                Triple rs_t0, rs_t1;
                rs_t0 = Triple(satdata_t0.satcrd()[0], satdata_t0.satcrd()[1], satdata_t0.satcrd()[2]);
                rs_t1 = Triple(satdata_t1.satcrd()[0], satdata_t1.satcrd()[1], satdata_t1.satcrd()[2]);

                double model = (rs_t1.dot(rs_t1) - rs_t0.dot(rs_t0) - 2 * rr_t0.dot(rs_t1 - rs_t0)) / ((rs_t0 - rr_t0).norm() + (rs_t1 - rr_t1).norm())
                               //+
                               //(-2 * rs_t1 + rr_t1 + rr_t0).dot(rr_t1 - rr_t0)
                               /// ((rs_t0 - rr_t0).norm() + (rs_t1 - rr_t1).norm())
                               + satdata_t0.clk() - satdata_t1.clk();

                double omc = (obs_L1_t1 - obs_L1_t0) - model;

                B.block(iobs, 0, 1, 3) = ((-2 * rs_t1 + rr_t1 + rr_t0) / ((rs_t0 - rr_t0).norm() + (rs_t1 - rr_t1).norm())).transpose();
                B(iobs, 3) = 1;
                l(iobs) = omc;
                iobs++;
            }
        }
    }

    B.conservativeResize(iobs, 4);
    l.conservativeResize(iobs);

    x = (B.transpose() * B).inverse() * B.transpose() * l;

    baseline[0] = x(0);
    baseline[1] = x(1);
    baseline[2] = x(2);
    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_post_turbo_select_obs(const base_time &epoch, std::vector<gnss_data_sats> &data)
{
    auto iter = data.begin();
    while (iter != data.end())
    {
        const std::string &obs_rec = iter->site();
        const std::string &obs_sat = iter->sat();

        bool obs_valid = _slip12->use_of_obs(obs_rec, obs_sat, epoch);
        if (obs_valid)
        {
            iter->tb12(obs_valid);
            if (_frequency >= 3)
            {
                bool obs_13 = _slip13->use_of_obs(obs_rec, obs_sat, epoch);
                iter->tb13(obs_13);
            }
            if (_frequency >= 5)
            {
                bool obs_14 = _slip14->use_of_obs(obs_rec, obs_sat, epoch);
                bool obs_15 = _slip15->use_of_obs(obs_rec, obs_sat, epoch);
                iter->tb14(obs_14);
                iter->tb15(obs_15);
            }
            iter++;
        }
        else
        {
            iter = data.erase(iter);
        }
    }

    if (data.empty())
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "This epoch have no useful data : " + epoch.str_ymdhms());
        return false;
    }

    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_addconstraint(Matrix &A, Symmetric &P, Vector &l)
{
    if (!_exist)
        return false;
    double dl;
    double p0_trp = 1E3;
    double p0_ion = 1E6;

    gnss_data_obscombtype type;
    gnss_proc_lsq_equationmatrix virtual_equ;

    int idx_trp = _param->getParam(_site, par_type::TRP, "");
    for (auto iter = _data.begin(); iter != _data.end(); iter++)
    {
        auto sat = iter->sat();
        auto gs = iter->gsys();

        if (_sagnss_coder_aug.find(sat) == _sagnss_coder_aug.end())
            continue;

        if (_sagnss_coder_aug[sat].find(std::make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1)) == _sagnss_coder_aug[sat].end())
            continue;

        double trp = _sagnss_coder_aug[sat][std::make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1)];

        GOBSBAND b = _band_index[gs][FREQ_1];
        if (_sagnss_coder_aug[sat].find(std::make_pair(AUGTYPE::TYPE_ION, b)) == _sagnss_coder_aug[sat].end())
            continue;
        double ion1 = _sagnss_coder_aug[sat][std::make_pair(AUGTYPE::TYPE_ION, b)];

        if (double_eq(trp, 0.0) || double_eq(ion1, 0.0))
            continue;

        // trp
        double mfW = iter->mfW();
        dl = trp * mfW - _param->operator[](idx_trp).value() * mfW;
        std::vector<std::pair<int, double>> B_trp;
        B_trp.push_back(std::make_pair(idx_trp, 1 * mfW));

        virtual_equ.add_equ(B_trp, p0_trp, dl, _site, sat, type, false);

        //ion
        int idx = _param->getParam(_site, par_type::SION, sat);
        dl = ion1 - _param->operator[](idx).value();
        std::vector<std::pair<int, double>> B_ion;
        B_ion.push_back(std::make_pair(idx, 1));
        virtual_equ.add_equ(B_ion, p0_ion, dl, _site, sat, type, false);
    }
    int nobs = A.rows();
    int npar = A.cols();

    Matrix B_mat;
    Symmetric P_mat;
    Vector l_mat;
    virtual_equ.chageNewMat(B_mat, P_mat, l_mat, npar);

    int virtual_obs_num = l_mat.size();
    ResizeKeep(A ,nobs + virtual_obs_num, npar);
    ResizeKeep(P.matrixW(), nobs + virtual_obs_num, nobs + virtual_obs_num);
    ResizeKeep(l, nobs + virtual_obs_num);

    // Float + Fixed
    A.block(nobs, 0, virtual_obs_num, npar) = B_mat;
    P.matrixW().block(nobs, nobs, virtual_obs_num, virtual_obs_num) = P_mat.matrixR();
    l.block(nobs, 0, virtual_obs_num, 1) = l_mat;
    _generateObsIndex(virtual_equ);
    nobs += virtual_obs_num;

    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_addconstraint(gnss_proc_flt *filter)
{
    if (!_exist)
        return false;
    double dl;
    double p0_trp = 1E2;
    double p0_ion = 1E6;

    gnss_data_obscombtype type;
    gnss_proc_lsq_equationmatrix virtual_equ;

    int idx_trp = _param->getParam(_site, par_type::TRP, "");
    for (auto iter = _data.begin(); iter != _data.end(); iter++)
    {
        auto sat = iter->sat();
        auto gs = iter->gsys();

        if (_sagnss_coder_aug.find(sat) == _sagnss_coder_aug.end())
            continue;

        if (_sagnss_coder_aug[sat].find(std::make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1)) == _sagnss_coder_aug[sat].end())
            continue;

        double trp = _sagnss_coder_aug[sat][std::make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1)];

        GOBSBAND b = _band_index[gs][FREQ_1];
        if (_sagnss_coder_aug[sat].find(std::make_pair(AUGTYPE::TYPE_ION, b)) == _sagnss_coder_aug[sat].end())
            continue;
        double ion1 = _sagnss_coder_aug[sat][std::make_pair(AUGTYPE::TYPE_ION, b)];

        if (double_eq(trp, 0.0) || double_eq(ion1, 0.0))
            continue;

        // trp
        double mfW = iter->mfW();
        dl = trp - _param->operator[](idx_trp).value() * mfW;
        std::vector<std::pair<int, double>> B_trp;
        B_trp.push_back(std::make_pair(idx_trp, 1 * mfW));
        virtual_equ.add_equ(B_trp, p0_trp, dl, _site, "", type, false);

        //ion
        int idx = _param->getParam(_site, par_type::SION, sat);
        dl = ion1 - _param->operator[](idx).value();
        std::vector<std::pair<int, double>> B_ion;
        B_ion.push_back(std::make_pair(idx, 1));
        virtual_equ.add_equ(B_ion, p0_ion, dl, _site, "", type, false);
    }

    Matrix B_mat;
    Symmetric P_mat;
    Vector l_mat;
    virtual_equ.chageNewMat(B_mat, P_mat, l_mat, filter->npar_number());

    try
    {
        filter->update();
    }
    catch (std::exception e)
    {
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, "Ambiguity Constrain Failed!");
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, e.what());
        return false;
    }

    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_available_data_realtime(const base_time& now)
{
    // For real time procesing, added by zhShen
    // glfeng  modified time settings  sleep 10 to 1, > to >=
    if (_realtime) //modify by zzwu
    {
        while (now >= _gobs->end_obs(_site))
            base_time::gmsleep(1);
        if (_isBase)
        {
            while (now >= _gobs->end_obs(_site_base))
                base_time::gmsleep(1);
        }
        else
        {
            while (!dynamic_cast<gnss_all_prec*>(_gnav)->corr_avali(now))
                base_time::gmsleep(1);
            while (!dynamic_cast<gnss_all_bias*>(_gallbias)->bias_avail(now))
                base_time::gmsleep(1);

            if (_upd_mode == UPD_MODE::UPD && _fix_mode != FIX_MODE::NO)
            {
                while (!_gupd->upd_avail(now))
                    base_time::gmsleep(1);
            }
        }
    }
    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_slip_detect(const base_time& now)
{
    if (_slip_model == SLIPMODEL::TURBO_EDIT)
    {
        if (_isClient)
            _gturboedit->setSingleEpoData(&_data);
        _gturboedit->ProcessBatch(_site, now, now, _sampling);
        if (_isBase && !_isClient)
        {
            _gturboedit->ProcessBatch(_site_base, now, now, _sampling);
        }
    }
    else
    {
        if (_isClient)
            _gpre->setSingleEpoData(&_data);
        _gpre->ProcessBatch(_site, now, now, _sampling, false);
        if (_isBase && !_isClient)
        {
            _gpre->ProcessBatch(_site_base, now, now, _sampling, false);
        }
    }
    return true;
}

int hwa_gnss::gnss_proc_pvtflt::_getData(const base_time& now, std::vector <gnss_data_sats>* data, bool isBase)
{
    // clean/collect/filter epoch data
    if (_isClient || data != NULL)
    {
        if (data != NULL)
        {
            if (!isBase)
            {
                _data.erase(_data.begin(), _data.end());
                _data = *data;
            }
            else
            {
                _data_base.erase(_data_base.begin(), _data_base.end());
                _data_base = *data;
            }

        }
    }
    else
    {
        if (!isBase)
        {
            _data.erase(_data.begin(), _data.end());
            _data = _gobs->obs(_site, now);
        }
        else
        {
            _data_base.erase(_data_base.begin(), _data_base.end());
            _data_base = _gobs->obs(_site_base, now);
        }
        
    }

    return (isBase ? static_cast<int>(_data_base.size()) : static_cast<int>(_data.size()));
}

bool hwa_gnss::gnss_proc_pvtflt::_crd_xml_valid()
{
    Triple crdapr = _grec->crd_arp(_epoch);

    if (double_eq(crdapr[0], 0.0) && double_eq(crdapr[1], 0.0) &&
        double_eq(crdapr[2], 0.0))
    {
        _valid_crd_xml = false;
    }
    else
    {
        _valid_crd_xml = true;
    }

    return _valid_crd_xml;
}

void hwa_gnss::gnss_proc_pvtflt::_remove_sat(const std::string& satid)
{
    std::vector<gnss_data_sats>::iterator it;
    // erase sat satellite because of outliers
    it = _data.begin();
    if (!satid.empty())
    { //lvhb modified
        while (it != _data.end())
        {
            if (it->sat() == satid)
            {
                it = _data.erase(it);
                continue;
            }
            ++it;
        }
    }

    return;
}

bool hwa_gnss::gnss_proc_pvtflt::_check_sat(const std::string& ssite, gnss_data_sats* const iter, Matrix& BB, int& iobs)
{
    GSYS gs = iter->gsys();

    //GOBSBAND b1, b2;
    GOBSBAND b1 = _band_index[gs][FREQ_1];
    GOBSBAND b2 = _band_index[gs][FREQ_2];

    iter->spdlog(_spdlog);

    auto l1 = iter->select_phase(b1);
    auto l2 = iter->select_phase(b2);

    auto p1 = iter->select_range(b1);
    auto p2 = iter->select_range(b2);

    // check data availability
    if (p1 == X || l1 == X)
    {
        return false;
    }

    std::string strGOBS = gobs2str(p1);
    strGOBS.replace(0, 1, "S");
    double obs_snr1 = iter->getobs(str2gobs(strGOBS));
    double obs_snr2 = iter->getobs(pha2snr(l1));
    _crt_SNR[iter->sat()][FREQ_1] = double_eq(obs_snr2, 0.0) ? obs_snr1 : obs_snr2;

    strGOBS = gobs2str(p2);
    strGOBS.replace(0, 1, "S");
    obs_snr1 = iter->getobs(str2gobs(strGOBS));
    obs_snr2 = iter->getobs(pha2snr(l2));
    _crt_SNR[iter->sat()][FREQ_2] = double_eq(obs_snr2, 0.0) ? obs_snr1 : obs_snr2;

    double P3, L3;
    if (_observ == OBSCOMBIN::RAW_SINGLE)
    {
        P3 = iter->obs_C(p1);
        L3 = iter->obs_L(l1);
    }
    else if (_observ == OBSCOMBIN::IF_P1 || _observ == OBSCOMBIN::RAW_MIX)
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
        if (p2 == X || l2 == X)
        {
            return false;
        }
        P3 = iter->P3(p1, p2);
        L3 = iter->L3(l1, l2);
    }

    if (double_eq(L3, 0.0) && _phase)
    {
        std::ostringstream str;
        str << "prepareData: erasing data due to no phase double bands observation, "
            << "epo: " << _epoch.str_hms() << ", "
            << "prn: " << iter->sat() << " (" << iter->channel() << ")";
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, str.str());
        return false;
    }

    if (double_eq(P3, 0.0))
    {
        std::ostringstream str;
        str << "prepareData: erasing data due to no code double bands observation, "
            << "epo: " << _epoch.str_hms() << ", "
            << "prn: " << iter->sat();
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, str.str());
        return false;
    }

    if (_satPos(_epoch, *iter) < 0)
    {
       std::ostringstream str;
       str << "prepareData: erasing data since _satPos failed, "
           << "epo: " << _epoch.str_hms() << ", "
           << "prn: " << iter->sat();
       if (_spdlog)
           SPDLOG_LOGGER_INFO(_spdlog, str.str());
#ifdef DEBUG
        std::cerr << "prepareData: sat pos not calculated "
            << _epoch.str_ymdhms(ssite + " " + iter->sat() + " " + base_type_conv::int2str(sdata.size()) + " ")
            << std::endl;
        std::cerr.flush();
#endif
        return false;
    }
    else
    {
        if (ssite == _site && _isClient)
        {
            _crt_SNR[iter->sat()][FREQ_1] = iter->getobs(pha2snr(l1));
            _crt_SNR[iter->sat()][FREQ_2] = iter->getobs(pha2snr(l2));
            _crt_ObsLevel[iter->sat()][FREQ_1] = iter->getobsLevelFlag(l1);
            _crt_ObsLevel[iter->sat()][FREQ_2] = iter->getobsLevelFlag(l2);
        }

        BB(iobs, 0) = iter->satcrd()[0];
        BB(iobs, 1) = iter->satcrd()[1];
        BB(iobs, 2) = iter->satcrd()[2];
        BB(iobs, 3) = P3 + iter->clk();
        iobs++;

        if (_nppmodel == NPP_MODEL::PPP_RTK && !_isCompAug && _isClient)
            _sagnss_coder_aug_map[_epoch][iter->sat()] = iter->exisgnss_coder_aug();
    }
    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_cmp_rec_crd(const std::string& ssite, Matrix& BB)
{
    _vBanc.setZero();

    if (BB.rows() >= 4)
        gbancroft(BB, _vBanc);
    //std::cout << "sppflt 2  :" << fixed << fixed << _vBanc(0) << "  " << _vBanc(1) << "  " << _vBanc(2) << "  " << _vBanc(3) << "  " << ssite << endl;
    std::shared_ptr<gnss_data_obj> grec = _gallobj->obj(ssite);
    if (grec == nullptr)
        return false;

    if (!_initialized && (_vBanc.segment(0, 3) - grec->crd_arp(_epoch)).norm() > 1000)
    {
        if (ssite == _site)
            _valid_crd_xml = false;
        else
        {
            _vBanc(0) = grec->crd_arp(_epoch)[0];
            _vBanc(1) = grec->crd_arp(_epoch)[1];
            _vBanc(2) = grec->crd_arp(_epoch)[2];
        }
    }

    if (_valid_crd_xml && !_initialized)
    {
        _vBanc(0) = grec->crd_arp(_epoch)[0];
        _vBanc(1) = grec->crd_arp(_epoch)[1];
        _vBanc(2) = grec->crd_arp(_epoch)[2];
        //if (_isClient)    std::cout << "sppflt 2.1  :" << std::fixed << std::fixed << _vBanc(1) << "  " << _vBanc(2) << "  " << _vBanc(3) << "  " << ssite << std::endl;
    }


    if (ssite == _site && _crd_est != CONSTRPAR::FIX)
    {
        /********* spplsq init **********/
        //Triple crd(0, 0, 0); double clk = 0;
        //if (_get_spplsq_result(_site, _epoch, crd, clk) > 0)
        //{
        //    _vBanc(0) = crd[0];
        //    _vBanc(1) = crd[1];
        //    _vBanc(2) = crd[2];
        //    _vBanc(3) = clk;
        //}
        if (_pos_constrain)
        {
            _vBanc(0) = _extn_pos[0];
            _vBanc(1) = _extn_pos[1];
            _vBanc(2) = _extn_pos[2];
            int iclk = _param->getParam(_site, par_type::CLK, "");
            if (BB.rows() < 4)
                _vBanc(3) = _param->operator[](iclk).value();
        }
    }
    else if (ssite == _site_base && dynamic_cast<set_gproc*>(_set)->basepos() == base_pos::CFILE)
    {
        _vBanc(0) = grec->crd_arp(_epoch)[0];
        _vBanc(1) = grec->crd_arp(_epoch)[1];
        _vBanc(2) = grec->crd_arp(_epoch)[2];
    }
    //std::cout << " PVT crd: " << _epoch.str_hms() << " " << std::fixed << std::fixed << _vBanc(0) << " " << _vBanc(1) << " " << _vBanc(2) << std::endl;
    
    //lvhb: just to judge spp/bancroft coordinates
    Triple test_xyz = _vBanc.segment(0, 3);
    Triple test_ell;
    xyz2ell(test_xyz, test_ell, true);
    double radius = test_xyz.norm();
    if (radius < B_WGS - 500)
    {
        std::string warning = "WARNING: Unexpected site (" + ssite + ") coordinates from Bancroft. Orbits/clocks or code observations should be checked.";
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, warning + _epoch.str_ymdhms(" Epoch "));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, warning + _epoch.str_ymdhms(" Epoch "));
        if (!_phase)
        {
            return false;
            //return -1;
        }
        //return -1;
        return false;
    }

    return true;
}

bool hwa_gnss::gnss_proc_pvtflt::_cmp_sat_info(const std::string& ssite, gnss_data_sats* const iter)
{
    Triple xyz_r, xyz_s;
    xyz_s = iter->satcrd();

    std::shared_ptr<gnss_data_obj> grec = _gallobj->obj(ssite);

    if (_isBase && ssite == _site_base)
    { //lvhb added in 20200523
        xyz_r = grec->crd_arp(_epoch);
        if (xyz_r.norm() == 0)
        {
            std::cerr << std::endl;
            std::cerr << xyz_r[0] << " " << xyz_r[1] << " " << xyz_r[2] << std::endl;
        }
        _vBanc(0) = xyz_r[0];
        _vBanc(1) = xyz_r[1];
        _vBanc(2) = xyz_r[2];
    }
    else
    {
        if (_crd_est == CONSTRPAR::FIX && _valid_crd_xml)
        {
            xyz_r = grec->crd_arp(_epoch);
            //lvhb added in 202008
            _vBanc(0) = xyz_r[0];
            _vBanc(1) = xyz_r[1];
            _vBanc(2) = xyz_r[2];
        }
        else if (_crd_est == CONSTRPAR::EST || _crd_est == CONSTRPAR::KIN || _crd_est == CONSTRPAR::SIMU_KIN)
        { //用于适配
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
                    _param->getCrdParam(ssite, xyz_r);
            }
            //modified by lvhb,20200925
            if (_pos_kin)
            {
                Triple crd_r = xyz_r - grec->eccxyz(_epoch);
                grec->crd(crd_r, Triple(1, 1, 1), _epoch, LAST_TIME, true);
            }
        }
        //else
        //{
        //    std::cerr << "Undefined CRD constraining" << std::endl;
        //    return -1;
        //}
    }
    Add_rho_azel(/*runEpoch,*/ ssite, xyz_s, xyz_r, *iter); //lvhb added in 20200523

    // check elevation cut-off
    if (iter->ele_deg() < _minElev)
    {
        std::ostringstream os;
        os << "Erasing " << iter->sat() << " data due to low elevation angle (ele = " << std::fixed << std::setprecision(1) << iter->ele_deg()
            << ") " << iter->epoch().str_ymdhms();
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, os.str());
        //iter = sdata.erase(iter); // !!!! zveda iterator !!!!
        //if (sdata.size() < _minsat)
        //{
        //    if (_log)
        //        _log->comment(2, "", ssite + _epoch.str_ymdhms(" epoch ") + base_type_conv::int2str(sdata.size()) + " skipped (data.size < _rtk_set->minsat)");
        //    return -1;
        //}
        //continue;
        return false;
    }

    // Printing beta and orbit angles for deep verbosity
    //std::ostringstream os;
    //os << iter->sat() << " " << iter->epoch().str_ymdhms() << " " << std::fixed << std::setprecision(1) << iter->beta() << " " << iter->orb_angle();
    //if (_spdlog)
    //    SPDLOG_LOGGER_INFO(_spdlog, std::string("gpvtflt:  ") +  os.str());

    // Changed by LX: add for PPP-RTK
    _crt_ele[iter->sat()] = iter->ele_deg();

    // check satellite eclipsing
    if (iter->ecl())
    {
        std::ostringstream os;
        os << "Erasing " << iter->sat() << " data due to satellite eclipsing (beta = " << std::fixed << std::setprecision(1) << iter->beta()
            << " ,orbit angle = " << iter->orb_angle() << ") " << iter->epoch().str_ymdhms();
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, std::string("gpvtflt:  ") +  os.str());
        //iter = sdata.erase(iter); // !!!! zveda iterator !!!!
        //if (sdata.size() < _minsat)
        //{
        //    if (_log)
        //        _log->comment(2, "", ssite + _epoch.str_ymdhms(" epoch ") + base_type_conv::int2str(sdata.size()) + " skipped (data.size < _rtk_set->minsat)");
        //    return -1;
        //}
        return false;
    }
    else
        return true;
        //iter++;
}

void hwa_gnss::gnss_proc_pvtflt::_syncAmb()
{
    if (_isBase)
        _udsdAmb();
    else
    {
        if (!_turbo_liteMode && _slip_model == SLIPMODEL::TURBO_EDIT)
            _post_turbo_syncAmb();
        else
            _udAmb();
    }
    return;
}

void hwa_gnss::gnss_proc_pvtflt::_predictCrd()
{
    double crdInit = _sig_init_crd;

    int i = 0;

    i = _param->getParam(_site, par_type::CRD_X, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0)
        {
            _param->operator[](i).value(_vBanc(0));
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
                    _param->operator[](i).value(_vBanc(0));
                if (_cntrep == 1 && _success)
                    _Qx.matrixW()(i, i) += _crdStoModel->getQ() * _crdStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
            }
        }
    }

    i = _param->getParam(_site, par_type::CRD_Y, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0)
        {
            _param->operator[](i).value(_vBanc(1));
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
                    _param->operator[](i).value(_vBanc(1));
                if (_cntrep == 1 && _success)
                    _Qx.matrixW()(i, i) += _crdStoModel->getQ() * _crdStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
            }
        }
    }

    i = _param->getParam(_site, par_type::CRD_Z, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0)
        {
            _param->operator[](i).value(_vBanc(2));
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
                    _param->operator[](i).value(_vBanc(2));
                if (_cntrep == 1 && _success)
                    _Qx.matrixW()(i, i) += _crdStoModel->getQ() * _crdStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
            }
        }
    }

    if (_motion_model)
    {
        i = _param->getParam(_site, par_type::CRD_X, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx(i, i) == 0.0)
            {
                _param->operator[](i).value(_vBanc(0));
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
                    if (_cntrep == 1 && _success && _pos_kin && _epoch > _beg_time)
                    {
                        _param->operator[](i).value(_param->operator[](i).value() + _vel[0] * _sampling);
                        //_Qx(i, i) -= _crdStoModel->getQ() * _crdStoModel->getQ();
                        _Qx.matrixW()(i, i) += _Qx_vel(0, 0) * _sampling * _sampling;
                    }
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
                }
            }
        }

        i = _param->getParam(_site, par_type::CRD_Y, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx(i, i) == 0.0)
            {
                _param->operator[](i).value(_vBanc(1));
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
                    if (_cntrep == 1 && _success && _pos_kin && _epoch > _beg_time)
                    {
                        _param->operator[](i).value(_param->operator[](i).value() + _vel[1] * _sampling);
                        //_Qx(i, i) -= _crdStoModel->getQ() * _crdStoModel->getQ();
                        _Qx.matrixW()(i, i) += _Qx_vel(1, 1) * _sampling * _sampling;
                    }
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
                }
            }
        }

        i = _param->getParam(_site, par_type::CRD_Z, "");
        if (i >= 0)
        {
            if (!_initialized || _Qx(i, i) == 0.0)
            {
                _param->operator[](i).value(_vBanc(2));
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
                    if (_cntrep == 1 && _success && _pos_kin && _epoch > _beg_time)
                    {
                        _param->operator[](i).value(_param->operator[](i).value() + _vel[2] * _sampling);
                        //_Qx(i, i) -= _crdStoModel->getQ() * _crdStoModel->getQ();
                        _Qx.matrixW()(i, i) += _Qx_vel(2, 2) * _sampling * _sampling;
                    }
                    if (_smooth)
                        _Noise.matrixW()(i, i) = _crdStoModel->getQ() * _crdStoModel->getQ();
                }
            }
        }
    }

    if (_pos_constrain)
    {
        int icrdx = _param->getParam(_site, par_type::CRD_X, "");
        for (int j = 0; j < 3; j++)
        {
            _Qx.matrixW()(_param->operator[](icrdx).index + j, _param->operator[](icrdx).index + j) = SQR(_extn_rms[j]);
        }
    }
    return;
}

void hwa_gnss::gnss_proc_pvtflt::_predictClk()
{
    int i = 0;

    i = _param->getParam(_site, par_type::CLK, "");
    if (i >= 0)
    {
        _param->operator[](i).value(_vBanc(3));
        for (unsigned int jj = 0; jj < _param->parNumber(); jj++)
            _Qx.matrixW()(i, jj) = 0.0;
        _Qx.matrixW()(i, i) = _clkStoModel->getQ() * _clkStoModel->getQ();
        if (_smooth)
            _Noise.matrixW()(i, i) = _clkStoModel->getQ() * _clkStoModel->getQ();
    }
    //std::cout << "Pridavam CLK " << " " << _param->operator[](i).value() << " " << _Qx(i, i) << std::endl;
    return;
}

void hwa_gnss::gnss_proc_pvtflt::_predictBias()
{
    int i = 0;

    i = _param->getParam(_site, par_type::GLO_ISB, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
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

    i = _param->getParam(_site, par_type::GLO_ifcb, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
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

    i = _param->getParam(_site, par_type::GAL_ISB, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
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

    i = _param->getParam(_site, par_type::BDS_ISB, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
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

    i = _param->getParam(_site, par_type::QZS_ISB, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
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

    i = _param->getParam(_site, par_type::IFB_GPS, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
            _Qx.matrixW()(i, i) = 3000 * 3000;
        }
        else
        {
            if (_cntrep == 1)
                _Qx.matrixW()(i, i) += _gpsStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _gpsStoModel->getQ();
        }
    }
    i = _param->getParam(_site, par_type::IFB_GAL, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
            _Qx.matrixW()(i, i) = 3000 * 3000;
        }
        else
        {
            if (_cntrep == 1)
                _Qx.matrixW()(i, i) += _galStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _galStoModel->getQ();
        }
    }
    i = _param->getParam(_site, par_type::IFB_BDS, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _param->operator[](i).value(0.0);
            _Qx.matrixW()(i, i) = 3000 * 3000;
        }
        else
        {
            if (_cntrep == 1)
                _Qx.matrixW()(i, i) += _bdsStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _bdsStoModel->getQ();
        }
    }

    i = _param->getParam(_site, par_type::IFB_QZS, "");
    if (i >= 0)
    {
        if (!_initialized || _Qx(i, i) == 0.0 || _isClient)
        {
            _Qx.matrixW()(i, i) = 3000 * 3000;
        }
        else
        {
            if (_cntrep == 1)
                _Qx.matrixW()(i, i) += _qzsStoModel->getQ();
            if (_smooth)
                _Noise.matrixW()(i, i) = _qzsStoModel->getQ();
        }
    }
    return;
}

void hwa_gnss::gnss_proc_pvtflt::_predictIono(const double& bl, const base_time& runEpoch)
{
    int i = 0;

    //double bl = 0;
    // predict ionosphere delay
    if (_iono_est)
    {
        std::vector<gnss_data_sats>::iterator it;
        for (it = _data.begin(); it != _data.end(); ++it)
        {
            i = _param->getParam(_site, par_type::VION, it->sat());
            if (i >= 0)
            {
                if (_gion)
                { // Introduce new apriory value from IONEX if available
                    Triple site_xyz(0.0, 0.0, 0.0);
                    Triple site_ell(0.0, 0.0, 0.0);
                    Triple ipp_ell(0.0, 0.0, 0.0);

                    _param->getCrdParam(_site, site_xyz);
                    if (site_xyz.isZero())
                        site_xyz = _vBanc;
                    xyz2ell(site_xyz, site_ell, false);
                    ell2ipp(&*it, site_ell, ipp_ell);

                    // use iono-free combination instead ! //
                    //      double ionomodel = _gion->iono(ipp_ell[0] * R2D, ipp_ell[1] * R2D, it->epoch());
                    double ionomodel = 1.0; // ( ionomodel * 40.28 * 1e16 ) / (G01_F*G01_F);

                    _param->operator[](i).apriori(ionomodel);
                }

                if (_cntrep == 1 &&
                    !double_eq(_Qx(i, i), _sig_init_vion * _sig_init_vion))
                {
                    _Qx.matrixW()(i, i) += _ionStoModel->getQ(); // *_ionStoModel->getQ();
                }

                // if (!_initialized || _Qx(i, i) == 0.0) {
                //   std::cout << "DEBUG " << it->sat() << " " << it->epoch().str_ymdhms() << std::endl;
                //   _Qx(i, i) =  _sig_init_vion * _sig_init_vion;
                // }
                // else {
                //   if (_cntrep == 1) _Qx(i, i) += _ionStoModel->getQ() * _ionStoModel->getQ();
                //   if (_smooth) _Noise(i, i) = _ionStoModel->getQ() * _ionStoModel->getQ();
                // }
            }

            i = _param->getParam(_site, par_type::SION, it->sat());
            if (i >= 0)
            {
                // for (unsigned int jj = 1; jj <= _param->parNumber(); jj++) _Qx(i, jj) = 0.0;
                double var = _sig_init_vion * _sig_init_vion;
                if (_isBase && double_eq(_Qx(i, i), var))
                {
                    var *= SQR(bl / (1e4));
                    _Qx.matrixW()(i, i) = var;
                    _param->operator[](i).value(1e-6);
                }
                else if ((_isClient && _isCompAug && !_isBase) || _isCorObs)
                {
                    _param->operator[](i).value(0.0);
                    _Qx.matrixW()(i, i) = var;
                }
                if (_cntrep == 1 && !double_eq(_Qx(i, i), var))
                {

                    if (_isBase)
                    {

                        _Qx.matrixW()(i, i) += SQR(bl / (1e4) * cos(it->ele())) * _ionStoModel->getQ();
                    }
                    else
                    {
                        _Qx.matrixW()(i, i) += _ionStoModel->getQ();
                    }
                }
            }
        }
    }

    if (_observ == OBSCOMBIN::RAW_SINGLE && _iono_constraint && _cntrep == 1)
    {
        _extern_ion_constraint(runEpoch);
    }
    return;
}

void hwa_gnss::gnss_proc_pvtflt::_predictTropo()
{
    int i = 0;
    i = _param->getParam(_site, par_type::GRD_N, "");
    if (i >= 0)
    {
        if (_cntrep == 1 && _initialized)
            _Qx.matrixW()(i, i) += _grdStoModel->getQ();
        if (_smooth)
            _Noise.matrixW()(i, i) = _grdStoModel->getQ();
    }

    i = _param->getParam(_site, par_type::GRD_E, "");
    if (i >= 0)
    {
        if (_cntrep == 1 && _initialized)
            _Qx.matrixW()(i, i) += _grdStoModel->getQ();
        if (_smooth)
            _Noise.matrixW()(i, i) = _grdStoModel->getQ();
    }

    if (_tropo_est)
    {
        //rover: sppflt::predict()
        //base:
        double ztdInit = _sig_init_ztd;
        for (int j = 0; j < 1 + (_isBase ? 1 : 0); j++)
        {
            auto tmpsite = _site;
            auto tmpgrec = _grec;
            auto tmpgmodel = _gModel;

            if (j)
            {
                tmpsite = _site_base;
                tmpgrec = _gallobj->obj(_site_base);
                tmpgmodel = _gModel_base;
            }
            i = _param->getParam(tmpsite, par_type::TRP, "");
            if (i >= 0)
            {
                Triple Ell, XYZ;

                if (_param->getCrdParam(tmpsite, XYZ) > 0)
                {
                }
                else
                {
                    XYZ = tmpgrec->crd_arp(_epoch);
                }
                xyz2ell(XYZ, Ell, false);
                if (tmpgmodel->tropoModel() != 0)
                {
                    _param->operator[](i).apriori(_sig_init_ztd);
                }

                if (!_initialized || _Qx(i, i) == 0.0 || (_isClient && _isCompAug))
                {
                    if (_valid_ztd_xml)
                    {
                        _param->operator[](i).value(_aprox_ztd_xml - tmpgmodel->tropoModel()->getZHD(Ell, _epoch));
                    }
                    else
                    {
                        if (tmpgmodel->tropoModel() != 0)
                        {
                            _param->operator[](i).value(tmpgmodel->tropoModel()->getZWD(Ell, _epoch));
                        }
                    }
                    if (_isClient && _isCompAug)
                        _param->operator[](i).value(0.0);
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
        }
    }
    return;
}

void hwa_gnss::gnss_proc_pvtflt::_predictAmb()
{
    // int i = 0;

    // ambiguity randomwalk
    if (_slip_model != SLIPMODEL::TURBO_EDIT || (_slip_model == SLIPMODEL::TURBO_EDIT && _turbo_liteMode))
    {
        for (unsigned int i = 0; i < _param->parNumber(); i++)
        {
            if (_param->operator[](i).parType == par_type::AMB_IF ||
                _param->operator[](i).parType == par_type::AMB_L1 ||
                _param->operator[](i).parType == par_type::AMB_L2 ||
                _param->operator[](i).parType == par_type::AMB_L3 ||
                _param->operator[](i).parType == par_type::AMB_L4 ||
                _param->operator[](i).parType == par_type::AMB_L5)
            {

                if (_cntrep == 1)
                    _Qx.matrixW()(i, i) += _ambStoModel->getQ();
                if (_smooth)
                    _Noise.matrixW()(i, i) = _ambStoModel->getQ();
                _param->operator[](i).stime = _param->operator[](i).end = _epoch;
            }
        }
    }
    return;
}

unsigned int hwa_gnss::gnss_proc_pvtflt::_cmp_equ(gnss_proc_lsq_equationmatrix &equ)
{
    std::vector<gnss_data_sats>::iterator it = _data.begin();
    for (it = _data.begin(); it != _data.end();)
    {
        if (!_base_model->cmb_equ(_epoch, *_param, *it, equ))
        {
            it = _data.erase(it);
            continue;
        }
        if (_observ == OBSCOMBIN::IONO_FREE)
            _combineMW(*it);
        if (_nppmodel == NPP_MODEL::PPP_RTK && _observ == OBSCOMBIN::RAW_MIX && it->band_avail(true).size() == 1 && _receiverType != And)
        {
            int nequ = equ.num_equ();
            equ.P[nequ - 2] *= pow(0.8, 1);
            equ.P[nequ - 1] *= pow(0.8, 1);
        }

        if (_nppmodel == NPP_MODEL::PPP_RTK && !_isCompAug && _isClient)
        {
            if (!it->exisgnss_coder_aug())
            {
                int nequ = equ.num_equ();
                equ.P[nequ - 4] *= 0.1;
                equ.P[nequ - 3] *= 0.1;
                equ.P[nequ - 2] *= 0.1;
                equ.P[nequ - 1] *= 0.1;
            }
        }
        ++it;
    }

    return equ.num_equ();
}

void hwa_gnss::gnss_proc_pvtflt::_posterioriTest(const Matrix& A, const Symmetric& P, const Vector& l,
    const Vector& dx, const Symmetric& Q, Vector& v_norm, double& vtpv)
{
    Vector v_orig, v_test;
    v_orig = l - A * dx;

    // normalized post-fit residuals
    Matrix Qv = A * Q.matrixR() * A.transpose() + P.matrixR().inverse();
    v_norm.resize(v_orig.rows());
    for (int i = 0; i < v_norm.rows(); i++)
    {
        v_norm(i) = sqrt(1 / Qv(i, i)) * v_orig(i);
    }

    int freedom = A.rows() - A.cols();
    if (freedom < 1)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt", "No redundant observations!");
        freedom = 1;
    }

    Vector vtPv = v_orig.transpose() * P.matrixR() * v_orig;
    _sig_unit = vtPv(0) / freedom;
    vtpv = vtPv(0);
    return;
}
void hwa_gnss::gnss_proc_pvtflt::_posterioriTest(const Matrix& A, const Matrix& R, const Vector& l,
    const Vector& dx, const Symmetric& Q, Vector& v_norm, double& vtpv)
{
    Vector v_orig, v_test;
    v_orig = l - A * dx;

    // normalized post-fit residuals
    Matrix Qv = A * Q.matrixR() * A.transpose() + R;
    v_norm.resize(v_orig.rows());
    for (int i = 0; i < v_norm.rows(); i++)
    {
        v_norm(i) = sqrt(1 / Qv(i, i)) * v_orig(i);
    }

    int freedom = A.rows() - A.cols();
    if (freedom < 1)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "gnss_proc_pvtflt", "No redundant observations!");
        freedom = 1;
    }

    Vector vtPv = v_orig.transpose() * R.inverse() * v_orig;
    _sig_unit = vtPv(0) / freedom;
    vtpv = vtPv(0);
    return;
}
