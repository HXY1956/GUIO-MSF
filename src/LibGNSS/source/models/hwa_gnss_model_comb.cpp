#include "hwa_gnss_model_comb.h"
#include "hwa_base_string.h"
#include "hwa_set_turboedit.h"
#include "hwa_gnss_proc_lsqmatrix.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_set_gbase.h"

namespace hwa_gnss
{
    gnss_model_comb::gnss_model_comb(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : _bias_model(std::move(bias_model)),
                                                                                                             _spdlog(nullptr)
    {
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

        // =======================================================================================
        // sigma
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

        _frequency = dynamic_cast<set_gproc *>(setting)->frequency();
        _ion_model = dynamic_cast<set_gproc *>(setting)->ion_model();
        _ifb_model = dynamic_cast<set_gproc *>(setting)->ifb_model();
        _ifcb_model = dynamic_cast<set_gproc *>(setting)->ifcb_model();
        _observ = dynamic_cast<set_gproc *>(setting)->obs_combin();

        _gallbias = dynamic_cast<gnss_all_bias *>((*data)[base_data::ALLBIAS]);
        _update_amb_lite = (dynamic_cast<set_turboedit *>(setting)->liteMode() || dynamic_cast<set_gproc *>(setting)->realtime());
    }

    gnss_model_comb::gnss_model_comb(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : _bias_model(std::move(bias_model)),
                                                                                                                              _spdlog(spdlog)
    {
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

        // =======================================================================================
        // sigma
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
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGPS " + format("%16.8f", _sigCodeGPS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGLO " + format("%16.8f", _sigCodeGLO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGAL " + format("%16.8f", _sigCodeGAL));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeBDS " + format("%16.8f", _sigCodeBDS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeQZS " + format("%16.8f", _sigCodeQZS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGPS" + format("%16.8f", _sigPhaseGPS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGLO" + format("%16.8f", _sigPhaseGLO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGAL" + format("%16.8f", _sigPhaseGAL));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseBDS" + format("%16.8f", _sigPhaseBDS));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseQZS" + format("%16.8f", _sigPhaseQZS));

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
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGPSLEO  " + format("%16.8f", _sigCodeGPSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGLOLEO  " + format("%16.8f", _sigCodeGLOLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeGALLEO  " + format("%16.8f", _sigCodeGALLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeBDSLEO  " + format("%16.8f", _sigCodeBDSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigCodeQZSLEO  " + format("%16.8f", _sigCodeQZSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGPSLEO " + format("%16.8f", _sigPhaseGPSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGLOLEO " + format("%16.8f", _sigPhaseGLOLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseGALLEO " + format("%16.8f", _sigPhaseGALLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseBDSLEO " + format("%16.8f", _sigPhaseBDSLEO));
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "sigPhaseQZSLEO " + format("%16.8f", _sigPhaseQZSLEO));

        _frequency = dynamic_cast<set_gproc *>(setting)->frequency();
        _ion_model = dynamic_cast<set_gproc *>(setting)->ion_model();
        _ifb_model = dynamic_cast<set_gproc *>(setting)->ifb_model();
        _ifcb_model = dynamic_cast<set_gproc *>(setting)->ifcb_model();
        _observ = dynamic_cast<set_gproc *>(setting)->obs_combin();

        _gallbias = dynamic_cast<gnss_all_bias *>((*data)[base_data::ALLBIAS]);
        _update_amb_lite = (dynamic_cast<set_turboedit *>(setting)->liteMode() || dynamic_cast<set_gproc *>(setting)->realtime());
    }

    gnss_model_comb::~gnss_model_comb() = default;

    std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> gnss_model_comb::get_band_index()
    {
        return _band_index;
    }

    std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> gnss_model_comb::get_freq_index()
    {
        return _freq_index;
    }

    bool gnss_model_comb::_wgt_raw_obs(const gnss_data_obs &gobs, const gnss_data_sats &satdata, const double &factorP, const OBSWEIGHT &wgt_type, double &wgt)
    {
        auto obs_type = gobs.type();
        if (obs_type != TYPE_C && obs_type != TYPE_P && obs_type != TYPE_L)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " type should be TYPE_C/TYPE_P/TYPE_L");
            return false;
        }

        double factor = factorP;

        // jdhuang : fix the bug
        auto gsat = satdata.sat();
        auto gsys = satdata.gsys();
        if (gsys == GSYS::BDS)
            factor = 2.0;
        if (gsys == GSYS::BDS && gnss_sys::bds_geo(gsat))
            factor = 5.0;

        // case of Range
        auto obj_type = satdata.id_type();
        double sigRange = 0.0;
        if (obs_type == TYPE_C || obs_type == TYPE_P)
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
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, " sys should be GPS/GAL/GLO/BDS/QZS");
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
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, " sys should be GPS/GAL/GLO/BDS/QZS");
                    return false;
                }
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " obj_type should be REC/REC_LEO");
                return false;
            }
        }

        // case of Phase
        double sigPhase = 0.0;
        if (obs_type == TYPE_L)
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
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, " sys should be GPS/GAL/GLO/BDS/QZS");
                    return false;
                }
            }
            else if (obj_type == base_data::REC_LEO)
            {
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
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, " sys should be GPS/GAL/GLO/BDS/QZS");
                    return false;
                }
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " obj_type should be REC/REC_LEO");
                return false;
            }
        }
        else
        {
            return false;
        }

        if (obj_type == base_data::REC)
        {
            double leo_ele = satdata.ele_leo();
            double leo_ele_deg = satdata.ele_leo_deg();
            double sin_leo_ele = sin(leo_ele);
            // get the _weight factor
            switch (wgt_type)
            {
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
            case OBSWEIGHT::DEF_OBS_WEIGHT:
            default:
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " WeightObs (default) should not happened!\n");
                return false;
            }
        }

        // get the combine obs factor
        // TODO finish other weight method
        switch (obs_type)
        {
        case TYPE_L:
            sigPhase = sigPhase * satdata.wavelength(gobs.band());
            if (sigPhase == 0.0)
                return false;
            wgt = 1.0 / pow(factor * sigPhase, 2);
            break;
        case TYPE_C:
        case TYPE_P:
            sigRange = sigRange;
            if (sigRange == 0.0)
                return false;
            wgt = 1.0 / pow(factor * sigRange, 2);
            break;
        default:
            return false;
        }
        return true;
    }

    gnss_model_comb_if::gnss_model_comb_if(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : gnss_model_comb(setting, std::move(bias_model), data)
    {
        _clk_type_index[std::make_pair(FREQ_3, GPS)] = par_type::CLK13_G;
        _clk_type_index[std::make_pair(FREQ_4, GPS)] = par_type::CLK14_G;
        _clk_type_index[std::make_pair(FREQ_5, GPS)] = par_type::CLK15_G;

        _clk_type_index[std::make_pair(FREQ_3, GAL)] = par_type::CLK13_E;
        _clk_type_index[std::make_pair(FREQ_4, GAL)] = par_type::CLK14_E;
        _clk_type_index[std::make_pair(FREQ_5, GAL)] = par_type::CLK15_E;

        _clk_type_index[std::make_pair(FREQ_3, BDS)] = par_type::CLK13_C;
        _clk_type_index[std::make_pair(FREQ_4, BDS)] = par_type::CLK14_C;
        _clk_type_index[std::make_pair(FREQ_5, BDS)] = par_type::CLK15_C;

        _clk_type_index[std::make_pair(FREQ_3, QZS)] = par_type::CLK13_J;
        _clk_type_index[std::make_pair(FREQ_4, QZS)] = par_type::CLK14_J;
        _clk_type_index[std::make_pair(FREQ_5, QZS)] = par_type::CLK15_J;
    }

    gnss_model_comb_if::gnss_model_comb_if(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : gnss_model_comb(setting, spdlog, std::move(bias_model), data)
    {
        _clk_type_index[std::make_pair(FREQ_3, GPS)] = par_type::CLK13_G;
        _clk_type_index[std::make_pair(FREQ_4, GPS)] = par_type::CLK14_G;
        _clk_type_index[std::make_pair(FREQ_5, GPS)] = par_type::CLK15_G;

        _clk_type_index[std::make_pair(FREQ_3, GAL)] = par_type::CLK13_E;
        _clk_type_index[std::make_pair(FREQ_4, GAL)] = par_type::CLK14_E;
        _clk_type_index[std::make_pair(FREQ_5, GAL)] = par_type::CLK15_E;

        _clk_type_index[std::make_pair(FREQ_3, BDS)] = par_type::CLK13_C;
        _clk_type_index[std::make_pair(FREQ_4, BDS)] = par_type::CLK14_C;
        _clk_type_index[std::make_pair(FREQ_5, BDS)] = par_type::CLK15_C;

        _clk_type_index[std::make_pair(FREQ_3, QZS)] = par_type::CLK13_J;
        _clk_type_index[std::make_pair(FREQ_4, QZS)] = par_type::CLK14_J;
        _clk_type_index[std::make_pair(FREQ_5, QZS)] = par_type::CLK15_J;
    }

    gnss_model_comb_if::~gnss_model_comb_if() = default;

    bool gnss_model_comb_if::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        // ========================================================================================================================================
        // check Obs type
        const GOBSBAND &b1 = _band_index[obsdata.gsys()][FREQ_1];
        const GOBSBAND &b2 = _band_index[obsdata.gsys()][FREQ_2];

        // ========================================================================================================================================
        obsdata.apply_bias(_gallbias);

        return this->cmb_equ_IF(epoch, params, obsdata, b1, b2, result);
    }

    bool gnss_model_comb_if::cmb_equ_IF(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, GOBSBAND b1, GOBSBAND b2, gnss_model_base_equation &result)
    {
        // jdhuang
        // ========================================================================================================
        if (b1 == BAND || b2 == BAND)
            return false;

        // ========================================================================================================
        gnss_data_obs gobsP1(obsdata.select_range(b1));
        gnss_data_obs gobsP2(obsdata.select_range(b2));
        gnss_data_obs gobsL1(obsdata.select_phase(b1));
        gnss_data_obs gobsL2(obsdata.select_phase(b2));

        std::vector<std::pair<gnss_data_obs, gnss_data_obs>> type_list;
        type_list.emplace_back(gobsP1, gobsP2);
        type_list.emplace_back(gobsL1, gobsL2);

        auto gsys = obsdata.gsys();
        // ========================================================================================================
        // IF coef
        double coef1, coef2;
        obsdata.coef_ionofree(b1, coef1, b2, coef2);

        // ========================================================================================================
        gnss_proc_lsq_equationmatrix equ_IF;
        for (const auto &item : type_list)
        {
            gnss_data_obs gobs1 = item.first;
            gnss_data_obs gobs2 = item.second;

            //combine f1 and f2
            gnss_model_base_equation temp_equ;
            if (!_bias_model->cmb_equ(epoch, params, obsdata, gobs1, temp_equ))
                return false;
            if (!_bias_model->cmb_equ(epoch, params, obsdata, gobs2, temp_equ))
                return false;

            // ========================================================================================================
            if (temp_equ.B[0].size() != temp_equ.B[1].size())
                throw std::logic_error("coeff size is not equal in f1 and f2");

            std::vector<std::pair<int, double>> coef_IF;
            double P_IF = 0.0, l_IF = 0.0;
            for (int i = 0; i < temp_equ.B[0].size(); i++)
            {
                if (temp_equ.B[0][i].first != temp_equ.B[1][i].first)
                {
                    throw std::logic_error("coeff par is not the same in f1 and f2");
                }
                // combine coeff
                coef_IF.emplace_back(temp_equ.B[0][i].first, coef1 * temp_equ.B[0][i].second + coef2 * temp_equ.B[1][i].second);
            }
            // combine P and l
            P_IF = 1.0 / (pow(coef1, 2) / temp_equ.P[0] + pow(coef2, 2) / temp_equ.P[1]);
            l_IF = coef1 * temp_equ.l[0] + coef2 * temp_equ.l[1];

            //jdhuang :
            if (_ifcb_model == IFCB_MODEL::COR && _frequency == 3 && gobs1.is_phase() && _freq_index[gsys][b2] == FREQ_3 && obsdata.gsys() == GPS)
            {
                l_IF += dynamic_cast<gnss_model_precise_bias *>(_bias_model.get())->ifcbDelay(obsdata, nullptr, OBSCOMBIN::IONO_FREE);
            }

            // ========================================================================================================
            // CORRECT AMB
            if (gobs1.is_phase() && (!obsdata.is_carrier_range(b1) || !obsdata.is_carrier_range(b2)))
            {
                par_type amb_type = par_type::NO_DEF;
                if (_freq_index[gsys][b2] == FREQ_2)
                    amb_type = par_type::AMB_IF;
                if (_freq_index[gsys][b2] == FREQ_3)
                    amb_type = par_type::AMB13_IF;
                if (_freq_index[gsys][b2] == FREQ_4)
                    amb_type = par_type::AMB14_IF;
                if (_freq_index[gsys][b2] == FREQ_5)
                    amb_type = par_type::AMB15_IF;

                int idx = params.getParam(obsdata.site(), amb_type, obsdata.sat());
                if (idx < 0)
                    return false;

                // if amb is new  then init value with Obs - Modelobs
                if (double_eq(params[idx].value(), 0.0) || params[idx].beg == epoch)
                {
                    double obs_P3 = obsdata.P3(gobsP1, gobsP2);
                    double obs_L3 = obsdata.L3(gobsL1, gobsL2);

                    params[idx].value(obs_L3 - obs_P3);
                    //params[idx].apriori(1e-9); //xjhan test
                }
                // update B l
                coef_IF.emplace_back(idx, 1.0);
                l_IF -= params[idx].value();
            }

            // ========================================================================================================
            // add 13 CLK
            if (!_add_IF_multi_rec_clk(_freq_index[gsys][b2], obsdata, params, coef_IF))
                return false;

#ifdef DEBUG_LEO
            // jdhuang : for debug
            std::string tmp, name;
            std::string time(epoch.str_ymdhms());
            char tmp_wgt[18];
            sprintf(tmp_wgt, "%16.8f", P_IF);
            std::string wgt(tmp_wgt);
            char tmp_rsi[18];
            sprintf(tmp_rsi, "%16.8f", l_IF);
            std::string rsi(tmp_rsi);
            if (gobs1.is_code())
                name = "obsp  ";
            else
                name = "obsl  ";
            tmp = name + time +
                  "range rec : " + obsdata.site() +
                  " satp : " + obsdata.sat() +
                  " wgtp : " + wgt +
                  " rsip : " + rsi; //+

            //jdhuang
             for
                 debug, do not remove
                                std::cout
                            << epoch.str_ymdhms()
                            << std::setw(10) << " l_IF "
                            << std::setw(10) << gfreqseq2str(_freq_index[gsys][b1]) + "_" + gfreqseq2str(_freq_index[gsys][b2])
                            << std::setw(10) << obsdata.site()
                            << std::setw(10) << obsdata.sat()
                            << std::setw(10) << l_IF
                            << std::endl;
#endif

             gnss_data_obscombtype type(gobs1, b1, b2, _freq_index[gsys][b1], _freq_index[gsys][b2], OBSCOMBIN::IONO_FREE);
             equ_IF.add_equ(coef_IF, P_IF, l_IF, obsdata.site(), obsdata.sat(), type, false);
        }

        if (dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result))
        {
            auto *lsq_result = dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result);
            lsq_result->add_equ(equ_IF);
        }
        else
        {
            result = result + equ_IF;
        }

        return true;
    }

    bool gnss_model_comb_if::_add_IF_multi_rec_clk(const FREQ_SEQ &freq, gnss_data_sats &obsdata, base_allpar &params, std::vector<std::pair<int, double>> &coef_IF)
    {
        auto gsys = obsdata.gsys();
        auto site = obsdata.site();
        if (freq <= FREQ_2)
            return true;

        // ???12???????
        for (auto iter = coef_IF.begin(); iter != coef_IF.end();)
        {
            par_type type = params[iter->first].parType;
            if (type == par_type::CLK ||
                type == par_type::GAL_ISB ||
                type == par_type::BDS_ISB ||
                type == par_type::QZS_ISB)
            {
                iter = coef_IF.erase(iter);
                continue;
            }
            iter++;
        }

        par_type clk_type = par_type::NO_DEF;
        if (_clk_type_index.find(std::make_pair(freq, gsys)) != _clk_type_index.end())
        {
            clk_type = _clk_type_index[std::make_pair(freq, gsys)];
        }

        int idx = params.getParam(site, clk_type, "");
        if (idx < 0)
        {
            throw std::runtime_error(" Can not find clk par in gnss_model_comb_if::_add_IF_multi_rec_clk");
            return false;
        }

        //update_value
        int idx_clk12 = params.getParam(site, par_type::CLK, "");
        params[idx].value(params[idx_clk12].value());
        coef_IF.emplace_back(idx, 1.0 - obsdata.drate());
        return true;
    }

    gnss_model_comb_if1x::gnss_model_comb_if1x(set_base *setting, base_log spdlog, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data) : gnss_model_comb(setting, spdlog, bias_model, data),
                                                                                                                                     _cmb_IF(setting, spdlog, bias_model, data)
    {
    }

    gnss_model_comb_if1x::~gnss_model_comb_if1x() = default;

    bool gnss_model_comb_if1x::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        obsdata.apply_bias(_gallbias);

        if (!obsdata.tb12())
            return false;

        auto gsys = obsdata.gsys();
        // auto band_size = _band_index[gsys].size();

        // jdhuang : for multi-freq
        if (_frequency <= 2)
        {
            return false;
        }

        bool valid = false;
        if (_frequency > 2)
        {
            // check Obs type
            GOBSBAND b1 = (_band_index[gsys].count(FREQ_1) != 0) ? _band_index[gsys][FREQ_1] : BAND;
            GOBSBAND b2 = (_band_index[gsys].count(FREQ_2) != 0) ? _band_index[gsys][FREQ_2] : BAND;
            GOBSBAND b3 = (_band_index[gsys].count(FREQ_3) != 0) ? _band_index[gsys][FREQ_3] : BAND;

            // 1+2 ???? 1+3 ????
            if (obsdata.tb12())
                valid = _cmb_IF.cmb_equ_IF(epoch, params, obsdata, b1, b2, result) || valid;
            if (obsdata.tb13())
                valid = _cmb_IF.cmb_equ_IF(epoch, params, obsdata, b1, b3, result) || valid;
        }

        if (_frequency > 3)
        {
            // check Obs type
            GOBSBAND b1 = (_band_index[gsys].count(FREQ_1) != 0) ? _band_index[gsys][FREQ_1] : BAND;
            GOBSBAND b4 = (_band_index[gsys].count(FREQ_4) != 0) ? _band_index[gsys][FREQ_4] : BAND;
            if (obsdata.tb14())
                valid = _cmb_IF.cmb_equ_IF(epoch, params, obsdata, b1, b4, result) || valid;
        }

        if (_frequency > 4)
        {
            // check Obs type
            GOBSBAND b1 = (_band_index[gsys].count(FREQ_1) != 0) ? _band_index[gsys][FREQ_1] : BAND;
            GOBSBAND b5 = (_band_index[gsys].count(FREQ_5) != 0) ? _band_index[gsys][FREQ_5] : BAND;
            if (obsdata.tb15())
                valid = _cmb_IF.cmb_equ_IF(epoch, params, obsdata, b1, b5, result) || valid;
        }

        return valid;
    }

    gnss_model_comb_all::gnss_model_comb_all(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : gnss_model_comb(setting, std::move(bias_model), data)
    {
        ambtype_list[FREQ_1] = par_type::AMB_L1;
        ambtype_list[FREQ_2] = par_type::AMB_L2;
        ambtype_list[FREQ_3] = par_type::AMB_L3;
        ambtype_list[FREQ_4] = par_type::AMB_L4;
        ambtype_list[FREQ_5] = par_type::AMB_L5;
    }

    gnss_model_comb_all::gnss_model_comb_all(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : gnss_model_comb(setting, spdlog, std::move(bias_model), data)
    {
        ambtype_list[FREQ_1] = par_type::AMB_L1;
        ambtype_list[FREQ_2] = par_type::AMB_L2;
        ambtype_list[FREQ_3] = par_type::AMB_L3;
        ambtype_list[FREQ_4] = par_type::AMB_L4;
        ambtype_list[FREQ_5] = par_type::AMB_L5;
    }

    gnss_model_comb_all::~gnss_model_comb_all() = default;

    bool gnss_model_comb_all::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        // ========================================================================================================================================
        if (_gallbias!=nullptr)
            obsdata.apply_bias(_gallbias);

        // check band_list
        std::map<FREQ_SEQ, GOBSBAND> &crt_bands = _band_index[obsdata.gsys()];

        // ========================================================================================================================================
        std::string grec = obsdata.site();
        std::string gsat = obsdata.sat();
        GSYS gsys = obsdata.gsys();

        if (crt_bands.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "crt bands is empty for ==> " + gnss_sys::gsys2str(gsys));
            return false;
        }

        // ========================================================================================================================================
        for (const auto &iter : crt_bands)
        {
            gnss_proc_lsq_equationmatrix equ_all;
            std::vector<std::pair<int, double>> coefP;
            std::vector<std::pair<int, double>> coefL;

            // double omcP = 0.0;
            // double omcL = 0.0;
            // double wgtP = 0.0;
            // double wgtL = 0.0;
            GOBSBAND band = iter.second;
            gnss_data_obs obsP(obsdata.select_range(band, true));
            gnss_data_obs obsL(obsdata.select_phase(band, true));

            // ========================================================================================================================================
            // check Obs Valid
            auto freq = iter.first;
            if (freq > _frequency)
                continue; //modified by lvhb in 20201211

            // check Obs Valid
            if (obsP.type() != TYPE_C && obsP.type() != TYPE_P)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "check your obs file, we have no range obs for " + gobsband2str(band));
                if (freq == FREQ_1 && _observ != OBSCOMBIN::RAW_MIX)
                    return false; // skip sat without freq_1 observation
                else
                    continue;
            }

            if (obsL.type() != TYPE_L)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "check your obs file, we have no phase obs for " + gobsband2str(band));
                if (freq == FREQ_1 && _observ != OBSCOMBIN::RAW_MIX)
                    return false;
                else
                    continue;
            }

            // jdhuang add for freq3 check
            if (_frequency == 1 && freq >= FREQ_2)
                continue;
            if (_frequency == 2 && freq >= FREQ_3)
                continue;
            if (_frequency == 3 && freq >= FREQ_4)
                continue;
            if (_frequency == 4 && freq >= FREQ_5)
                continue;
            if (freq >= FREQ_6)
                continue;

            // jdhuang
            // spdlog tb check
            if (
                (freq == FREQ_1 && !obsdata.tb12() && !obsdata.tb13() && !obsdata.tb14() && !obsdata.tb15()) ||
                (!_update_amb_lite && freq == FREQ_2 && !obsdata.tb12()) ||
                (!_update_amb_lite && freq == FREQ_3 && !obsdata.tb13()) ||
                (!_update_amb_lite && freq == FREQ_4 && !obsdata.tb14()) ||
                (!_update_amb_lite && freq == FREQ_5 && !obsdata.tb15()))
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "bad arc in spdlog tb for freq " + gfreqseq2str(freq) + " " + grec + " + " + gsat);
                continue;
            }

            // ========================================================================================================================================
            gnss_model_base_equation tempP;
            if (!_bias_model->cmb_equ(epoch, params, obsdata, obsP, tempP))
                continue;
            gnss_model_base_equation tempL;
            if (!_bias_model->cmb_equ(epoch, params, obsdata, obsL, tempL))
                continue;

            // add AMB
            int idx = params.getParam(grec, ambtype_list[_freq_index[gsys][band]], gsat);
            if (idx >= 0)
            {
                //lvhb modified for npp testing,20200917
                // if amb is new  then init value with Obs - Modelobs
                if (double_eq(params[idx].value(), 0.0) || params[idx].beg == epoch)
                {
                    // jdhuang change for ambfix
                    double obs_P = obsdata.obs_C(obsP);
                    double obs_L = obsdata.obs_L(obsL);
                    params[idx].value(obs_L - obs_P);
                }

                tempL.B[0].push_back(std::make_pair(idx, 1.0));
                tempL.l[0] -= params[idx].value();
                //ifcb correction
                if (_ifcb_model == IFCB_MODEL::COR && _frequency >= 3 && freq == FREQ_3)
                {
                    tempL.l[0] += dynamic_cast<gnss_model_precise_bias *>(_bias_model.get())->ifcbDelay(obsdata, nullptr, OBSCOMBIN::RAW_ALL);
                }
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "check your spdlog file, the amb idx < 0 for :" + grec + "_" + gsat);
                continue;
            }

            if (idx < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "check your spdlog file, the amb idx < 0 for :" + grec + "_" + gsat);
                return false;
            }

            // ========================================================================================================================================
            gnss_data_obscombtype typeP(obsP, band, freq, OBSCOMBIN::RAW_ALL);
            equ_all.add_equ(tempP.B[0], tempP.P[0], tempP.l[0], obsdata.site(), obsdata.sat(), typeP, false);
            gnss_data_obscombtype typeL(obsL, band, freq, OBSCOMBIN::RAW_ALL);
            equ_all.add_equ(tempL.B[0], tempL.P[0], tempL.l[0], obsdata.site(), obsdata.sat(), typeL, false);

            // add result
            if (dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result))
            {
                auto *lsq_reuslt = dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result);
                lsq_reuslt->add_equ(equ_all);
            }
            else
            {
                result = result + equ_all;
            }
            // ========================================================================================================================================
        }
        if (!result.l.size())
            return false; // refer LvHB 20201221
        return true;
    }

    gnss_model_comb_wl::gnss_model_comb_wl(set_base *setting, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : gnss_model_comb(setting, std::move(bias_model), data)
    {
    }

    gnss_model_comb_wl::gnss_model_comb_wl(set_base *setting, base_log spdlog, std::shared_ptr<gnss_model_bias> bias_model,  base_all_proc *data) : gnss_model_comb(setting, spdlog, std::move(bias_model), data)
    {
    }

    gnss_model_comb_wl::~gnss_model_comb_wl() = default;

    bool gnss_model_comb_wl::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        obsdata.apply_bias(_gallbias);
        // check Obs type
        GOBSBAND b1 = _band_index[obsdata.gsys()][FREQ_1];
        GOBSBAND b2 = _band_index[obsdata.gsys()][FREQ_2];

        return this->cmb_equ_WL(epoch, params, obsdata, b1, b2, result);
    }

    bool gnss_model_comb_wl::cmb_equ_WL(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, GOBSBAND b1, GOBSBAND b2, gnss_model_base_equation &result)
    {
        // only for freq1 and freq2
        gnss_data_obs gobsP1(obsdata.select_range(b1));
        gnss_data_obs gobsP2(obsdata.select_range(b2));
        gnss_data_obs gobsL1(obsdata.select_phase(b1));
        gnss_data_obs gobsL2(obsdata.select_phase(b2));

        std::vector<std::pair<gnss_data_obs, gnss_data_obs>> type_list;
        type_list.emplace_back(gobsP1, gobsP2);
        type_list.emplace_back(gobsL1, gobsL2);

        // WL coef
        double coef1, coef2;

        gnss_proc_lsq_equationmatrix equ_WL;

        // cycle [Code and Phase]
        for (const auto &item : type_list)
        {
            gnss_data_obs gobs1 = item.first;
            gnss_data_obs gobs2 = item.second;

            if (gobs1.is_phase())
            {
                obsdata.coef_widelane(b1, coef1, b2, coef2);
            }
            else
            {
                obsdata.coef_narrlane(b1, coef1, b2, coef2);
            }

            //combine P1 and P2
            gnss_model_base_equation temp_equ;
            if (!_bias_model->cmb_equ(epoch, params, obsdata, gobs1, temp_equ))
            {
                return false;
            }
            if (!_bias_model->cmb_equ(epoch, params, obsdata, gobs2, temp_equ))
            {
                return false;
            }

            std::vector<std::pair<int, double>> coef_WL;
            double P_WL = 0.0, l_WL = 0.0;
            if (temp_equ.B[0].size() != temp_equ.B[1].size())
            {
                throw std::logic_error("coeff size is not equal in f1 and f2");
            }
            for (int i = 0; i < temp_equ.B[0].size(); i++)
            {
                if (temp_equ.B[0][i].first != temp_equ.B[1][i].first)
                {
                    throw std::logic_error("coeff par is not the same in f1 and f2");
                }
                // combine coeff
                coef_WL.emplace_back(temp_equ.B[0][i].first, coef1 * temp_equ.B[0][i].second + coef2 * temp_equ.B[1][i].second);
            }

            // combine P
            P_WL = sqrt(pow(coef1, 2) * pow(temp_equ.P[0], 2) + pow(coef2, 2) * pow(temp_equ.P[1], 2));
            // combine l
            l_WL = coef1 * temp_equ.l[0] + coef2 * temp_equ.l[1];

            // CORRECT AMB
            if (gobs1.is_phase())
            {
                par_type amb_type = par_type::AMB_WL;

                int idx = params.getParam(obsdata.site(), amb_type, obsdata.sat());

                if (idx < 0)
                {
                    return false;
                }

                // if amb is new  then init value with Obs - Modelobs
                if (double_eq(params[idx].value(), 0.0) || params[idx].beg == epoch)
                {
                    double MW_value = -obsdata.MW(gobs1, gobs2);
                    //correct_DCB("CAS", _gallbias, obsdata, MW_value, OBSCOMBIN::MW_COMBIN, &gobs1, &gobs2);  // glfeng for Normalize
                    params[idx].value(MW_value);
                }

                // update B l
                coef_WL.emplace_back(idx, 1.0);
                l_WL -= params[idx].value();
            }

            // jdhuang : for debug
            if (!gobs1.is_phase())
            {
                std::string tmp;
                std::string time(epoch.str_ymdhms());
                char tmp_wgt[18];
                sprintf(tmp_wgt, "%16.8f", P_WL);
                std::string wgt(tmp_wgt);
                char tmp_rsi[18];
                sprintf(tmp_rsi, "%16.8f", l_WL);
                std::string rsi(tmp_rsi);
                tmp = time +
                      "Range rec : " +
                      " satP : " + obsdata.sat() +
                      " wgtP : " + wgt +
                      " rsiP : " + rsi; //+
                //std::cout << "OBSP  " + tmp << std::endl;
            }
            else
            {
                std::string tmp;
                std::string time(epoch.str_ymdhms());
                char tmp_wgt[18];
                sprintf(tmp_wgt, "%16.8f", P_WL);
                std::string wgt(tmp_wgt);
                char tmp_rsi[18];
                sprintf(tmp_rsi, "%16.8f", l_WL);
                std::string rsi(tmp_rsi);
                tmp = time +
                      "Range rec : " +
                      " satL : " + obsdata.sat() +
                      " wgtL : " + wgt +
                      " rsiL : " + rsi; //+
                                        // std::cout << "OBSL  " + tmp << std::endl;
            }
            gnss_data_obscombtype type(gobs1, OBSCOMBIN::WL_COMBIN);
            equ_WL.add_equ(coef_WL, P_WL, l_WL, obsdata.site(), obsdata.sat(), type, false);
        }

        if (dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result))
        {
            auto *lsq_result = dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result);
            lsq_result->add_equ(equ_WL);
        }
        else
        {
            result = result + equ_WL;
        }
        return true;
    }

    gnss_model_comb_dd::gnss_model_comb_dd(set_base *setting, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data) : gnss_model_comb(setting, bias_model, data),
                                                                                                              gnss_model_comb_all(setting, bias_model, data),
                                                                                                              gnss_model_comb_if(setting, bias_model, data)
    {
        _data_base = nullptr;
        _site = "";
        _site_base = "";
    }

    gnss_model_comb_dd::gnss_model_comb_dd(set_base *setting, base_log spdlog, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data) : gnss_model_comb(setting, spdlog, bias_model, data),
                                                                                                                               gnss_model_comb_all(setting, spdlog, bias_model, data),
                                                                                                                               gnss_model_comb_if(setting, spdlog, bias_model, data)
    {
        _data_base = nullptr;
        _site = "";
        _site_base = "";
    }

    gnss_model_comb_dd::~gnss_model_comb_dd() = default;

    void gnss_model_comb_dd::set_observ(OBSCOMBIN observ)
    {
        _observ = observ;
    }

    void gnss_model_comb_dd::set_base_data(std::vector<gnss_data_sats> *data_base)
    {
        _data_base = data_base;
    }

    void gnss_model_comb_dd::set_site(const std::string &site, const std::string &site_base)
    {
        _site = site;
        _site_base = site_base;
    }

    void gnss_model_comb_dd::set_rec_info(const Triple &xyz_base, double clk_rover, double clk_base)
    {
        _crd_base = xyz_base;
        _clk_rover = clk_rover;
        _clk_base = clk_base;
    }

    bool gnss_model_comb_dd::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        gnss_data_sats obsdata_other(_spdlog);
        if (_data_base == nullptr)
            return false;
        for (auto &i : *_data_base)
        {
            if (obsdata.sat() == i.sat())
                obsdata_other = i;
        }
        if (obsdata_other.sat().empty())
            return false;
        //obsdata.apply_dcb(*_gallbias);
        //obsdata_other.apply_dcb(*_gallbias);
        unsigned npar_orig = params.parNumber();
        base_allpar params_temp;
        _temp_params(params, params_temp);
        if (_observ == OBSCOMBIN::IONO_FREE)
        {
            // check Obs type
            GOBSBAND b1 = _band_index[obsdata.gsys()][FREQ_1];
            GOBSBAND b2 = _band_index[obsdata.gsys()][FREQ_2];

            gnss_data_obs gobsP1(obsdata.select_range(b1));
            gnss_data_obs gobsP2(obsdata.select_range(b2));
            gnss_data_obs gobsL1(obsdata.select_phase(b1));
            gnss_data_obs gobsL2(obsdata.select_phase(b2));

            gnss_data_obs gobsP1_other(obsdata_other.select_range(b1));
            gnss_data_obs gobsP2_other(obsdata_other.select_range(b2));
            gnss_data_obs gobsL1_other(obsdata_other.select_phase(b1));
            gnss_data_obs gobsL2_other(obsdata_other.select_phase(b2));

            std::vector<std::pair<gnss_data_obs, gnss_data_obs>> type_list;
            type_list.emplace_back(gobsP1, gobsP2);
            type_list.emplace_back(gobsL1, gobsL2);

            std::vector<std::pair<gnss_data_obs, gnss_data_obs>> type_list_other;
            type_list_other.emplace_back(gobsP1_other, gobsP2_other);
            type_list_other.emplace_back(gobsL1_other, gobsL2_other);

            // IF coef
            double coef1, coef2;
            obsdata.coef_ionofree(b1, coef1, b2, coef2);

            gnss_proc_lsq_equationmatrix equ_IF;

            // meter [Code and Phase]
            for (int i = 0; i < type_list.size(); i++)
            {
                gnss_data_obs gobs1 = type_list[i].first;
                gnss_data_obs gobs2 = type_list[i].second;
                gnss_data_obs gobs1_other = type_list_other[i].first;
                gnss_data_obs gobs2_other = type_list_other[i].second;

                //combine P1 and P2
                gnss_model_base_equation temp_equ;
                base_time crt = obsdata.epoch();
                if (!_bias_model->cmb_equ(crt, params_temp, obsdata, gobs1, temp_equ))
                {
                    /*return false;*/
                    continue; //Lvhb changed in 202012
                }
                if (!_bias_model->cmb_equ(crt, params_temp, obsdata, gobs2, temp_equ))
                {
                    //return false;
                    continue; //Lvhb changed in 202012
                }
                gnss_model_base_equation temp_equ_other;
                base_time crt_other = obsdata_other.epoch();
                if (!_bias_model->cmb_equ(crt_other, params_temp, obsdata_other, gobs1_other, temp_equ_other))
                {
                    //return false;
                    continue; //Lvhb changed in 202012
                }
                if (!_bias_model->cmb_equ(crt_other, params_temp, obsdata_other, gobs2_other, temp_equ_other))
                {
                    //return false;
                    continue; //Lvhb changed in 202012
                }
                std::vector<std::pair<int, double>> coef_IF;
                double P_IF = 0.0, l_IF = 0.0;

                if (temp_equ.B[0].size() != temp_equ.B[1].size())
                {
                    throw std::logic_error("coeff size is not equal in f1 and f2");
                }
                for (int j = 0; j < temp_equ.B[0].size(); j++)
                {
                    if (temp_equ.B[0][j].first != temp_equ.B[1][j].first)
                    {
                        throw std::logic_error("coeff par is not the same in f1 and f2");
                    }
                    // combine coeff
                    if (temp_equ.B[0][j].first >= npar_orig)
                        continue;
                    coef_IF.emplace_back(temp_equ.B[0][j].first, coef1 * temp_equ.B[0][j].second + coef2 * temp_equ.B[1][j].second);
                }

                if (temp_equ_other.B[0].size() != temp_equ_other.B[1].size())
                {
                    throw std::logic_error("coeff size is not equal in f1 and f2");
                }
                for (int j = 0; j < temp_equ_other.B[0].size(); j++)
                {
                    if (temp_equ_other.B[0][j].first != temp_equ_other.B[1][j].first)
                    {
                        throw std::logic_error("coeff par is not the same in f1 and f2");
                    }
                    // combine coeff
                    if (temp_equ_other.B[0][j].first >= npar_orig)
                        continue;
                    coef_IF.emplace_back(temp_equ_other.B[0][j].first, -(coef1 * temp_equ_other.B[0][j].second + coef2 * temp_equ_other.B[1][j].second));
                }

                // combine P
                P_IF = 1.0 / (pow(coef1, 2) / temp_equ.P[0] + pow(coef2, 2) / temp_equ.P[1]);
                // combine l
                l_IF = coef1 * temp_equ.l[0] + coef2 * temp_equ.l[1] - (coef1 * temp_equ_other.l[0] + coef2 * temp_equ_other.l[1]);

                // CORRECT AMB
                if (gobs1.is_phase())
                {
                    par_type amb_type = par_type::AMB_IF;
                    if (_freq_index[obsdata.gsys()][b2] == FREQ_3)
                    {
                        amb_type = par_type::AMB13_IF;
                    }
                    if (_freq_index[obsdata.gsys()][b2] == FREQ_4)
                    {
                        amb_type = par_type::AMB14_IF;
                    }
                    if (_freq_index[obsdata.gsys()][b2] == FREQ_5)
                    {
                        amb_type = par_type::AMB15_IF;
                    }

                    int idx = params_temp.getParam(obsdata.site(), amb_type, obsdata.sat());

                    if (idx < 0)
                    {
                        return false;
                    }

                    // if amb is new  then init value with Obs - Modelobs
                    if (double_eq(params_temp[idx].value(), 0.0) || params_temp[idx].beg == epoch)
                    {
                        double obs_P3 = obsdata.P3(gobsP1, gobsP2);
                        double obs_L3 = obsdata.L3(gobsL1, gobsL2);
                        params_temp[idx].value(obs_L3 - obs_P3);
                    }

                    // update B l
                    coef_IF.emplace_back(idx, 1.0);
                    l_IF -= params_temp[idx].value();
                }

                // CORRECT CLK
                //remove 12 clk and isb
                if (_freq_index[obsdata.gsys()][b2] >= FREQ_3)
                {
                    for (auto iter = coef_IF.begin(); iter != coef_IF.end();)
                    {
                        par_type type = params_temp[iter->first].parType;
                        if (type == par_type::CLK ||
                            type == par_type::GAL_ISB ||
                            type == par_type::BDS_ISB ||
                            type == par_type::QZS_ISB)
                        {
                            iter = coef_IF.erase(iter);
                            continue;
                        }
                        iter++;
                    }
                }

                // add 13 CLK
                if (_freq_index[obsdata.gsys()][b2] == FREQ_3)
                {
                    par_type clk_type;
                    if (obsdata.gsys() == GPS)
                        clk_type = par_type::CLK13_G;
                    else if (obsdata.gsys() == GAL)
                        clk_type = par_type::CLK13_E;
                    else if (obsdata.gsys() == BDS)
                        clk_type = par_type::CLK13_C;
                    else if (obsdata.gsys() == QZS)
                        clk_type = par_type::CLK13_J;
                    else
                        return false;

                    int idx = params_temp.getParam(obsdata.site(), clk_type, "");
                    if (idx < 0)
                    {
                        return false;
                    }
                    //update_value
                    int idx_clk12 = params_temp.getParam(obsdata.site(), par_type::CLK, obsdata.sat());
                    params_temp[idx] = params_temp[idx_clk12];

                    coef_IF.emplace_back(idx, 1.0 - obsdata.drate());
                }

                // add 14 15 ... clk
                // ....
                gnss_data_obscombtype type(gobs1, OBSCOMBIN::IONO_FREE);
                equ_IF.add_equ(coef_IF, P_IF, l_IF, obsdata.site(), obsdata.sat(), type, false);
            }

            if (dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result))
            {
                auto *lsq_result = dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result);
                lsq_result->add_equ(equ_IF);
            }
            else
            {
                result = result + equ_IF;
            }
        }
        else if (_observ == OBSCOMBIN::RAW_ALL || _observ == OBSCOMBIN::RAW_SINGLE || _observ == OBSCOMBIN::RAW_MIX)
        {
            // check band_list
            std::map<FREQ_SEQ, GOBSBAND> crt_bands = _band_index[obsdata.gsys()];
            // added RAW_MIX wh
            if ((_observ == OBSCOMBIN::RAW_SINGLE /*|| _observ == OBSCOMBIN::RAW_MIX*/) && crt_bands.size() > 1)
            {
                auto first_band = (*crt_bands.begin());
                crt_bands.clear();
                crt_bands.insert(first_band);
            }
            std::map<FREQ_SEQ, par_type> ambtype_list = {
                {FREQ_1, par_type::AMB_L1},
                {FREQ_2, par_type::AMB_L2},
                {FREQ_3, par_type::AMB_L3},
                {FREQ_4, par_type::AMB_L4},
                {FREQ_5, par_type::AMB_L5}};

            if (crt_bands.empty())
            {
                return false;
            }
            gnss_proc_lsq_equationmatrix equ_all;
            gnss_model_base_equation tempP, tempL;
            std::vector<int> index(10, 0); //added by lvhb in 20201211,10=5freq*2site;

            for (int isite = 0; isite < 2; isite++)
            {
                gnss_data_sats *satdata_ptr;
                if (isite == 0)
                    satdata_ptr = &obsdata_other;
                else
                    satdata_ptr = &obsdata;

                for (const auto &iter : crt_bands)
                {
                    if (iter.first > _frequency)
                        continue; //modified by lvhb in 20201211

                    GOBSBAND band = iter.second;
                    gnss_data_obs obsP(satdata_ptr->select_range(band));
                    gnss_data_obs obsL(satdata_ptr->select_phase(band));

                    // check Obs Valid
                    // for P code modified by zhshen
                    if (obsP.type() != TYPE_C && obsP.type() != TYPE_P)
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_INFO(_spdlog, "check your obs file, we have no range obs for " + gobsband2str(band));
                        continue; //Lvhb changed in 202012
                    }

                    if (obsL.type() != TYPE_L)
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_INFO(_spdlog, "check your obs file, we have no phase obs for " + gobsband2str(band));
                        continue; //Lvhb changed in 202012
                    }

                    // jdhuang add for freq3 check
                    auto freq = iter.first;
                    base_time crt = satdata_ptr->epoch();
                    if (!_bias_model->cmb_equ(crt, params_temp, *satdata_ptr, obsP, tempP))
                    {
                        continue; //Lvhb changed in 202012
                    }

                    if (!_bias_model->cmb_equ(crt, params_temp, *satdata_ptr, obsL, tempL))
                    {
                        continue; //Lvhb changed in 202012
                    }
                    index[isite * crt_bands.size() + freq - 1] = tempL.l.size();
                    // add AMB for ROVER
                    if (satdata_ptr->site() == _site)
                    {
                        int idx = params_temp.getParam(satdata_ptr->site(), ambtype_list[_freq_index[satdata_ptr->gsys()][band]], satdata_ptr->sat());

                        if (idx < 0)
                        {
                            return false;
                        }
                        // if amb is new  then init value with Obs - Modelobs
                        if (double_eq(params_temp[idx].value(), 0.0) || params_temp[idx].beg == epoch)
                        {
                            //pars[idx].value(obs - ModelObs);
                            double obs_P = satdata_ptr->obs_C(obsP);
                            double obs_L = satdata_ptr->obs_L(obsL);
                            //apply_DCB(_gallbias, obsdata, obs_P, OBSCOMBIN::RAW_ALL, &obsP);

                            params_temp[idx].value(obs_L - obs_P);
                        }
                        tempL.B.back().push_back(std::make_pair(idx, 1.0));
                        tempL.l.back() -= params_temp[idx].value();
                    }
                }
            }

            // make differenced equations

            int freq_count = 0;
            for (const auto &iter : crt_bands)
            {
                if (iter.first > _frequency)
                    continue; //modified by lvhb in 20201211

                freq_count++;

                GOBSBAND band = iter.second;
                gnss_data_obs obsP(obsdata.select_range(band));
                gnss_data_obs obsL(obsdata.select_phase(band));

                std::vector<std::pair<int, double>> B_P, B_L;
                double P_P, P_L, l_P, l_L;
                //int ibase = freq_count - 1;
                //int irover = crt_bands.size()  + freq_count - 1;
                //if(!index[ibase]||!index[irover]) continue;
                int ibase = index[freq_count - 1];
                int irover = index[crt_bands.size() + freq_count - 1];
                if (!ibase || !irover)
                    continue;
                ibase -= 1;
                irover -= 1;

                for (const auto &b : tempP.B[irover])
                {
                    if (b.first >= npar_orig)
                        continue;
                    B_P.push_back(b);
                }
                for (const auto &b : tempP.B[ibase])
                {
                    if (b.first >= npar_orig)
                        continue;
                    B_P.emplace_back(b.first, -b.second);
                }
                for (const auto &b : tempL.B[irover])
                {
                    if (b.first >= npar_orig)
                        continue;
                    B_L.push_back(b);
                }
                for (const auto &b : tempL.B[ibase])
                {
                    if (b.first >= npar_orig)
                        continue;
                    B_L.emplace_back(b.first, -b.second);
                }

                P_P = 1 / (1 / tempP.P[irover] + 1 / tempP.P[ibase]);
                l_P = tempP.l[irover] - tempP.l[ibase];
                P_L = 1 / (1 / tempL.P[irover] + 1 / tempL.P[ibase]);
                l_L = tempL.l[irover] - tempL.l[ibase];

                gnss_data_obscombtype typeP(obsP, OBSCOMBIN::RAW_ALL);
                gnss_data_obscombtype typeL(obsL, OBSCOMBIN::RAW_ALL);
                equ_all.add_equ(B_P, P_P, l_P, obsdata.site(), obsdata.sat(), typeP, false);
                equ_all.add_equ(B_L, P_L, l_L, obsdata.site(), obsdata.sat(), typeL, false);
            }

            // add result
            if (dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result))
            {
                auto *lsq_reuslt = dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result);
                lsq_reuslt->add_equ(equ_all);
            }
            else
            {
                result = result + equ_all;
            }
        }
        for (int ipar = 0; ipar < params.parNumber(); ipar++)
            params[ipar] = params_temp[ipar];
        return true;
    }

    bool gnss_model_comb_dd::_temp_params(base_allpar &params, base_allpar &params_temp)
    {
        params_temp = params;
        base_par par_x_base;
        par_x_base.site = _site_base;
        par_x_base.parType = par_type::CRD_X;
        par_x_base.value(_crd_base[0]);
        base_par par_y_base;
        par_y_base.site = _site_base;
        par_y_base.parType = par_type::CRD_Y;
        par_y_base.value(_crd_base[1]);
        base_par par_z_base;
        par_z_base.site = _site_base;
        par_z_base.parType = par_type::CRD_Z;
        par_z_base.value(_crd_base[2]);
        base_par par_clk_rover;
        par_clk_rover.site = _site;
        par_clk_rover.parType = par_type::CLK;
        par_clk_rover.value(_clk_rover);
        base_par par_clk_base;
        par_clk_base.site = _site_base;
        par_clk_base.parType = par_type::CLK;
        par_clk_base.value(_clk_base);
        params_temp.addParam(par_x_base);
        params_temp.addParam(par_y_base);
        params_temp.addParam(par_z_base);
        params_temp.addParam(par_clk_rover);
        params_temp.addParam(par_clk_base);
        params_temp.reIndex();
        return true;
    }

    gnss_model_comb_mix::gnss_model_comb_mix(set_base *setting, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data) : gnss_model_comb(setting, bias_model, data),
                                                                                                                gnss_model_comb_all(setting, bias_model, data),
                                                                                                                gnss_model_comb_if(setting, bias_model, data)
    {
        _gdata_ionex = dynamic_cast<gnss_data_ionex *>((*data)[base_data::IONEX]); //< IONEX Grid data, glfeng
    }

    gnss_model_comb_mix::gnss_model_comb_mix(set_base *setting, base_log spdlog, const std::shared_ptr<gnss_model_bias> &bias_model,  base_all_proc *data) : gnss_model_comb(setting, spdlog, bias_model, data),
                                                                                                                                 gnss_model_comb_all(setting, spdlog, bias_model, data),
                                                                                                                                 gnss_model_comb_if(setting, spdlog, bias_model, data)
    {

        _gdata_ionex = dynamic_cast<gnss_data_ionex *>((*data)[base_data::IONEX]); //< IONEX Grid data, glfeng
    }

    gnss_model_comb_mix::~gnss_model_comb_mix() = default;

    bool gnss_model_comb_mix::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        obsdata.apply_bias(_gallbias);
        if (_observ == OBSCOMBIN::IF_P1)
        {
            return this->_cmb_equ_IF_P1(epoch, params, obsdata, result);
        }
        return false;
    }

    bool gnss_model_comb_mix::_cmb_equ_IF_P1(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_model_base_equation &result)
    {
        GSYS sys = obsdata.gsys();
        std::set<GOBSBAND> bands = obsdata.band_avail();
        GOBSBAND b1 = _band_index[sys][FREQ_1];
        GOBSBAND b2 = _band_index[sys][FREQ_2];

        // P1 FREQ1 range
        bool isIF = false, isP1 = false;

        if (bands.find(b1) != bands.end())
        {
            gnss_proc_lsq_equationmatrix equ_all;
            gnss_data_obs obsP(obsdata.select_range(b1, true));

            gnss_model_base_equation tempP;
            if (_bias_model->cmb_equ(epoch, params, obsdata, obsP, tempP))
                isP1 = true;

            if (isP1)
            {
                // ion_delay
                if (_gdata_ionex)
                {
                    hwa_gnss_model_iono_tecgrid iono_tec;
                    double sion_L1, sion_rms;
                    Triple trs_rec_crd = _bias_model->_trs_rec_crd;
                    if (iono_tec.getIonoDelay(_gdata_ionex, obsdata, epoch, trs_rec_crd, sion_L1, sion_rms))
                    {
                        double f1 = obsdata.frequency(gnss_sys::band_priority(obsdata.gsys(), FREQ_1));
                        double fk = obsdata.frequency(b1);
                        double alfa = (f1 * f1) / (fk * fk);
                        double iono_delay = alfa * sion_L1;
                        tempP.l.back() -= iono_delay;
                    }
                }

                gnss_data_obscombtype type(obsP, OBSCOMBIN::IF_P1);
                equ_all.add_equ(tempP.B[0], tempP.P[0], tempP.l[0], obsdata.site(), obsdata.sat(), type, false);

                // add result
                if (dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result))
                {
                    auto *lsq_reuslt = dynamic_cast<gnss_proc_lsq_equationmatrix *>(&result);
                    lsq_reuslt->add_equ(equ_all);
                }
                else
                {
                    result = result + equ_all;
                }
            }
        }

        // IF type
        if (bands.find(b1) != bands.end() && bands.find(b2) != bands.end())
        {
            isIF = this->cmb_equ_IF(epoch, params, obsdata, b1, b2, result);
        }

        if (isP1 || isIF)
            return true;
        else
            return false;
    }
}