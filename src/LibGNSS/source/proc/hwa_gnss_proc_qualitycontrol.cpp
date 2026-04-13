#include <iomanip>
#include <memory>

#include "hwa_gnss_proc_qualitycontrol.h"
#include "hwa_gnss_model_Bancroft.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{

    gnss_proc_smooth::gnss_proc_smooth(set_base *settings)
    {
        std::string smoothModel = dynamic_cast<set_gproc *>(settings)->range_smooth_mode(&_smoothWindow);
        if (smoothModel == "DOPPLER")
            _smoothModel = SMOOTH_model::SMT_DOPPLER;
        else if (smoothModel == "PHASE")
            _smoothModel = SMOOTH_model::SMT_PHASE;
        else
            _smoothModel = SMOOTH_model::SMT_NONE;

        _smoothFactor = 1.0 / _smoothWindow;
        _sampling = dynamic_cast<set_gen *>(settings)->sampling();

        _band_index[GPS] = dynamic_cast<set_gnss *>(settings)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(settings)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(settings)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(settings)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(settings)->band_index(QZS);
    }

    void gnss_proc_smooth::smooth_range_obs(std::vector<gnss_data_sats> &obsdata, const base_time &now)
    {
        if (_smoothModel == SMOOTH_model::SMT_NONE)
            return;

        // doppler
        if (_smoothModel == SMOOTH_model::SMT_DOPPLER)
        {
            //_doppler_smt_range(obsdata, now);
            _doppler_smt_range_new(obsdata, now);
        }
        // phase
        else
        {
            _phase_smt_range(obsdata, now);
        }
    }

    void gnss_proc_smooth::_doppler_smt_range(std::vector<gnss_data_sats> &obsdata, const base_time &now)
    {
        for (auto &oneObs : obsdata)
        {
            if (oneObs.getrangestate("smooth_range"))
                continue;
            std::string site = oneObs.site();
            // GSYS gsys = oneObs.gsys();
            std::string gsat = oneObs.sat();

            if (_smt_beg_time.find(site) == _smt_beg_time.end() || _smt_beg_time[site].find(gsat) == _smt_beg_time[site].end())
            {
                _smt_beg_time[site][gsat] = now;
            }
            else if (now.diff(_smt_beg_time[site][gsat]) > _smoothWindow * _sampling)
            {
                _smt_beg_time[site][gsat] = now;
            }

            std::vector<GOBS> vec_obs = oneObs.obs();
            for (GOBS obs_type : vec_obs)
            {
                // skip not code obs
                if (!gnss_data_obs(obs_type).is_code())
                {
                    continue;
                }

                gnss_data_obs gobs(obs_type);
                GOBSBAND band = gobs.band();

                GOBS GOBSP = obs_type;
                std::string strGOBS = gobs2str(GOBSP);
                strGOBS.replace(0, 1, "D");
                GOBS GOBSD = str2gobs(strGOBS);

                // get range/doppler obs.
                double obsP = oneObs.obs_C(GOBSP); // m
                double obsD = oneObs.obs_D(GOBSD); // m/s

                if (_smt_beg_time[site][gsat] == now || GOBSD != _pre_orig_val[site][gsat][band].first || oneObs.getoutliers(GOBSD) > 0)
                {
                    _pre_orig_val[site][gsat][band] = std::make_pair(GOBSD, obsD);
                    _pre_pre_orig_val[site][gsat][band] = _pre_orig_val[site][gsat][band]; //
                    _pre_smt_range[site][gsat][GOBSP] = obsP;
                    _smt_beg_time[site][gsat] == now;
                    continue;
                }
                if ((fabs(_pre_pre_orig_val[site][gsat][band].second + obsD - _pre_orig_val[site][gsat][band].second * 2.0) < 0.171))
                {
                    double smt = _smoothFactor * obsP + (1.0 - _smoothFactor) * (_pre_smt_range[site][gsat][GOBSP] -
                                                                                 (_pre_orig_val[site][gsat][band].second + obsD) / 2.0); //-OBSD
                    _pre_pre_orig_val[site][gsat][band].first = _pre_orig_val[site][gsat][band].first;                                   //
                    _pre_pre_orig_val[site][gsat][band].second = _pre_orig_val[site][gsat][band].second;                                 //
                    _pre_orig_val[site][gsat][band].first = GOBSD;
                    _pre_orig_val[site][gsat][band].second = obsD;
                    _pre_smt_range[site][gsat][GOBSP] = smt;
                    oneObs.resetobs(GOBSP, smt);
                    oneObs.setrangestate("smooth_range", true);
                }
                else //
                {
                    _pre_pre_orig_val[site][gsat][band].first = _pre_orig_val[site][gsat][band].first;   //
                    _pre_pre_orig_val[site][gsat][band].second = _pre_orig_val[site][gsat][band].second; //
                    _pre_orig_val[site][gsat][band].first = GOBSD;                                       //
                    _pre_orig_val[site][gsat][band].second = obsD;
                    _smt_beg_time[site][gsat] == now; //
                }
            }
        }
    }

    void gnss_proc_smooth::_doppler_smt_range_new(std::vector<gnss_data_sats> &obsdata, const base_time &now)
    {
        for (auto &oneObs : obsdata)
        {
            if (oneObs.getrangestate("smooth_range"))
                continue;
            std::string site = oneObs.site();
            GSYS gsys = oneObs.gsys();
            std::string gsat = oneObs.sat();

            for (auto itband : _band_index[gsys])
            {
                GOBSBAND b = itband.second;

                if (_smt_last_time.find(site) == _smt_last_time.end() || _smt_last_time[site].find(gsat) == _smt_last_time[site].end() || _smt_last_time[site][gsat].find(b) == _smt_last_time[site][gsat].end())
                {
                    _smt_last_time[site][gsat][b] = now;
                }

                GOBS GOBSP = oneObs.select_range(b, true);
                GOBS GOBSL = oneObs.select_phase(b, true);
                GOBS GOBSD;
                if (_receiverType == And)
                {
                    std::string strGOBS = gobs2str(GOBSP);
                    strGOBS.replace(0, 1, "D");
                    GOBSD = str2gobs(strGOBS);
                }
                else
                {
                    std::string strGOBS = gobs2str(GOBSL);
                    strGOBS.replace(0, 1, "D");
                    GOBSD = str2gobs(strGOBS);
                }

                // get range/doppler obs.
                double obsP = oneObs.obs_C(GOBSP); // m
                double obsD = oneObs.obs_D(GOBSD); // m/s

                if (double_eq(obsP, 0.0) || double_eq(obsD, 0.0))
                {
                    continue;
                }
                else
                {
                    if (GOBSD != _pre_orig_val[site][gsat][b].first || oneObs.getoutliers(GOBSD) > 0 || now.diff(_smt_last_time[site][gsat][b]) > 1.1) // >1s
                    {
                        _pre_orig_val[site][gsat][b] = std::make_pair(GOBSD, obsD);
                        _pre_pre_orig_val[site][gsat][b] = _pre_orig_val[site][gsat][b]; //
                        _pre_smt_range[site][gsat][GOBSP] = obsP;
                    }
                    else
                    {
                        double delta_DD = fabs(_pre_pre_orig_val[site][gsat][b].second + obsD - _pre_orig_val[site][gsat][b].second * 2.0);
                        double thres = 0.9 * oneObs.wavelength(b);
                        if (_pre_pre_orig_val[site][gsat][b].first == _pre_orig_val[site][gsat][b].first && delta_DD < thres)
                        {
                            double smt = _smoothFactor * obsP + (1.0 - _smoothFactor) * (_pre_smt_range[site][gsat][GOBSP] -
                                                                                         (_pre_orig_val[site][gsat][b].second + obsD) / 2.0); //-OBSD
                            _pre_pre_orig_val[site][gsat][b].first = _pre_orig_val[site][gsat][b].first;                                      //
                            _pre_pre_orig_val[site][gsat][b].second = _pre_orig_val[site][gsat][b].second;                                    //
                            _pre_orig_val[site][gsat][b].first = GOBSD;
                            _pre_orig_val[site][gsat][b].second = obsD;
                            _pre_smt_range[site][gsat][GOBSP] = smt;
                            _smt_last_time[site][gsat][b] = now;
                            oneObs.resetobs(GOBSP, smt);
                            oneObs.setrangestate("smooth_range", true);
                            std::cout << " smt doppler " << now.str_hms() << "  " << now.sow() << "  " << now.sod() << "  " + gsat
                                 << "  " << gobs2str(GOBSP) << std::setw(14) << std::setprecision(3) << obsP
                                 << std::setw(14) << std::setprecision(3) << smt << std::endl;
                        }
                        else //
                        {
                            std::cout << "reset doppler " << now.str_hms() << "  " << gsat
                                 << "  " << gobs2str(_pre_pre_orig_val[site][gsat][b].first)
                                 << "  " << gobs2str(_pre_orig_val[site][gsat][b].first)
                                 << "  " << delta_DD << std::endl;
                            _pre_pre_orig_val[site][gsat][b].first = _pre_orig_val[site][gsat][b].first;   //
                            _pre_pre_orig_val[site][gsat][b].second = _pre_orig_val[site][gsat][b].second; //
                            _pre_orig_val[site][gsat][b].first = GOBSD;                                    //
                            _pre_orig_val[site][gsat][b].second = obsD;
                            _pre_smt_range[site][gsat][GOBSP] = obsP;
                        }
                    }
                }
            }
        }
    }

    void gnss_proc_smooth::_phase_smt_range(std::vector<gnss_data_sats> &obsdata, const base_time &now)
    {
        for (auto &oneObs : obsdata)
        {
            if (oneObs.getrangestate("smooth_range"))
                continue;
            std::string site = oneObs.site();
            // GSYS gsys = oneObs.gsys();
            std::string gsat = oneObs.sat();

            if (_smt_beg_time.find(site) == _smt_beg_time.end() || _smt_beg_time[site].find(gsat) == _smt_beg_time[site].end())
            {
                _smt_beg_time[site][gsat] = now;
            }
            else if (now.diff(_smt_beg_time[site][gsat]) > _smoothWindow * _sampling)
            {
                _smt_beg_time[site][gsat] = now;
            }

            std::vector<GOBS> vec_obs = oneObs.obs();
            for (GOBS obs_type : vec_obs)
            {
                // skip not code obs
                if (!gnss_data_obs(obs_type).is_code())
                {
                    continue;
                }

                gnss_data_obs gobs(obs_type);
                GOBSBAND band = gobs.band();

                GOBS GOBSP = obs_type;
                GOBS GOBSL = oneObs.select_phase(band, true);

                double obsP = oneObs.obs_C(GOBSP);
                double obsL = oneObs.obs_L(GOBSL);

                if (_smt_beg_time[site][gsat] == now || GOBSL != _pre_orig_val[site][gsat][band].first) //|| oneObs.getlli(GOBSL) > 0
                {
                    _pre_orig_val[site][gsat][band] = std::make_pair(GOBSL, obsL);
                    _pre_smt_range[site][gsat][GOBSP] = obsP;
                    _smt_beg_time[site][gsat] == now;
                }
                else if (oneObs.getlli(GOBSL) > 0)
                {
                    _pre_orig_val[site][gsat][band] = std::make_pair(GOBSL, obsL);
                    _pre_smt_range[site][gsat][GOBSP] = obsP;
                    _smt_beg_time[site][gsat] == now;
                }
                else
                {
                    double smt = _smoothFactor * obsP + (1.0 - _smoothFactor) *
                                                            (_pre_smt_range[site][gsat][GOBSP] + obsL - _pre_orig_val[site][gsat][band].second);
                    _pre_orig_val[site][gsat][band].first = GOBSL;
                    _pre_orig_val[site][gsat][band].second = obsL;
                    _pre_smt_range[site][gsat][GOBSP] = smt;
                    oneObs.resetobs(GOBSP, smt);
                    oneObs.setrangestate("smooth_range", true);
                }
            }
        }
    }

    gnss_proc_bds_codebias_corr::gnss_proc_bds_codebias_corr(set_base *settings)
    {
        // std::set the setting pointer
        if (nullptr == settings)
        {
            spdlog::critical("your std::set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _set = settings;
        }

        _correct_bds_code_bias = dynamic_cast<set_gproc *>(settings)->bds_code_bias_correction();
        std::set<std::string> sys_list = dynamic_cast<set_gen *>(settings)->sys();
        if (sys_list.find("BDS") == sys_list.end())
        {
            _correct_bds_code_bias = false;
        }

        _band_index[BDS] = dynamic_cast<set_gnss *>(settings)->band_index(BDS);
        _band_index[GPS] = dynamic_cast<set_gnss *>(settings)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(settings)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(settings)->band_index(GLO);
        _band_index[QZS] = dynamic_cast<set_gnss *>(settings)->band_index(QZS);

        // std::set Wanninger & Beer
        _IGSO_MEO_Corr[BAND_2]["IGSO"] = {{0, -0.55}, {1, -0.40}, {2, -0.34}, {3, -0.23}, {4, -0.15}, {5, -0.04}, {6, 0.09}, {7, 0.19}, {8, 0.27}, {9, 0.35}}; // B1
        _IGSO_MEO_Corr[BAND_7]["IGSO"] = {{0, -0.71}, {1, -0.36}, {2, -0.33}, {3, -0.19}, {4, -0.14}, {5, -0.03}, {6, 0.08}, {7, 0.17}, {8, 0.24}, {9, 0.33}}; // B2
        _IGSO_MEO_Corr[BAND_6]["IGSO"] = {{0, -0.27}, {1, -0.23}, {2, -0.21}, {3, -0.15}, {4, -0.11}, {5, -0.04}, {6, 0.05}, {7, 0.14}, {8, 0.19}, {9, 0.32}}; // B3

        _IGSO_MEO_Corr[BAND_2]["MEO"] = {{0, -0.47}, {1, -0.38}, {2, -0.32}, {3, -0.23}, {4, -0.11}, {5, 0.06}, {6, 0.34}, {7, 0.69}, {8, 0.97}, {9, 1.05}}; //B1
        _IGSO_MEO_Corr[BAND_7]["MEO"] = {{0, -0.40}, {1, -0.31}, {2, -0.26}, {3, -0.18}, {4, -0.06}, {5, 0.09}, {6, 0.28}, {7, 0.48}, {8, 0.64}, {9, 0.69}}; //B2
        _IGSO_MEO_Corr[BAND_6]["MEO"] = {{0, -0.22}, {1, -0.15}, {2, -0.13}, {3, -0.10}, {4, -0.04}, {5, 0.05}, {6, 0.14}, {7, 0.27}, {8, 0.36}, {9, 0.47}}; //B3
    }

    void gnss_proc_bds_codebias_corr::apply_IGSO_MEO(const std::string &rec, Triple &rec_crd, gnss_all_nav *gnav, std::vector<gnss_data_sats> &obsdata)
    {
        if (!this->_correct_bds_code_bias || obsdata.size() == 0)
            return;

        if (rec_crd.isZero())
        {
            if (!this->_recAprCoordinate(rec, rec_crd, gnav, obsdata))
                return;
        }

        // Calculate elevation and Correct Obs
        std::map<GOBSBAND, double> Band_cor;
        std::string sat_type;

        std::vector<gnss_data_sats>::iterator data;
        for (data = obsdata.begin(); data != obsdata.end();)
        {
            if (data->getrangestate("bds_code_bias")) // ???bds_code_bias
            {
                data++;
                continue;
            }
            if (data->site() != rec || data->gsys() != BDS) // ???BDS
            {
                data++;
                continue;
            }
            std::string sat = data->sat();
            if (sat <= "C05" || sat > "C16") // BDS2 - IGSO/MEO
            {
                data++;
                continue;
            }
            if (sat == "C11" || sat == "C12" || sat == "C14")
                sat_type = "MEO";
            else
                sat_type = "IGSO";

            if (data->satcrd().isZero())
            {
                if (dynamic_cast<set_gproc *>(_set)->realtime())
                {
                    if (data->addprd_realtime(gnav) < 0)
                    {
                        data = obsdata.erase(data);
                        continue;
                    }
                }
                else
                {
                    if (data->addprd(gnav) < 0)
                    {
                        data = obsdata.erase(data);
                        continue;
                    }
                }
            }
            Triple sat_crd = data->satcrd();
            Triple rec_sat_vector = sat_crd - rec_crd;
            double distance = rec_sat_vector.norm();
            double elev = (rec_crd[0] * rec_sat_vector[0] + rec_crd[1] * rec_sat_vector[1] + rec_crd[2] * rec_sat_vector[2]) / rec_crd.norm() / distance;
            elev = 90.0 - acos(elev) * 180.0 / hwa_pi;

            // get correction
            double elev0 = elev / 10.0;
            int elev0_int = floor(elev0);
            if (elev0_int < 0)
            {
                Band_cor[BAND_2] = _IGSO_MEO_Corr[BAND_2][sat_type][0];
                Band_cor[BAND_7] = _IGSO_MEO_Corr[BAND_7][sat_type][0];
                Band_cor[BAND_6] = _IGSO_MEO_Corr[BAND_6][sat_type][0];
            }
            else if (elev0_int >= 9)
            {
                Band_cor[BAND_2] = _IGSO_MEO_Corr[BAND_2][sat_type][9];
                Band_cor[BAND_7] = _IGSO_MEO_Corr[BAND_7][sat_type][9];
                Band_cor[BAND_6] = _IGSO_MEO_Corr[BAND_6][sat_type][9];
            }
            else
            {
                Band_cor[BAND_2] = _IGSO_MEO_Corr[BAND_2][sat_type][elev0_int] * (1.0 - elev0 + elev0_int) + _IGSO_MEO_Corr[BAND_2][sat_type][elev0_int + 1] * (elev0 - elev0_int);
                Band_cor[BAND_7] = _IGSO_MEO_Corr[BAND_7][sat_type][elev0_int] * (1.0 - elev0 + elev0_int) + _IGSO_MEO_Corr[BAND_7][sat_type][elev0_int + 1] * (elev0 - elev0_int);
                Band_cor[BAND_6] = _IGSO_MEO_Corr[BAND_6][sat_type][elev0_int] * (1.0 - elev0 + elev0_int) + _IGSO_MEO_Corr[BAND_6][sat_type][elev0_int + 1] * (elev0 - elev0_int);
            }

            std::vector<GOBS> obs_vec = data->obs();
            for (auto obs_type : obs_vec)
            {
                // skip not code obs
                if (!gnss_data_obs(obs_type).is_code())
                {
                    continue;
                }

                GOBSBAND b = gnss_data_obs(obs_type).band();
                double obs_P = data->obs_C(obs_type);
#ifdef DEBUG
                std::cout << data->epoch().str_ymdhms() << "   " << sat << "   " << gobs2str(obs_type) << std::fixed << std::setw(16) << std::setprecision(3) << obs_P
                     << std::setw(16) << std::setprecision(3) << Band_cor[b] << std::setw(16) << std::setprecision(3) << obs_P + Band_cor[b] << std::endl;
#endif // !DEBUG

                obs_P += Band_cor[b];
                data->resetobs(obs_type, obs_P);
                data->setrangestate("bds_code_bias", true);
            }

            data++;
        }
        return;
    }

    bool gnss_proc_bds_codebias_corr::_recAprCoordinate(const std::string &rec, Triple &rec_crd, gnss_all_nav *gnav, std::vector<gnss_data_sats> &obsdata)
    {
        Matrix BB;

        BB.resize(obsdata.size(), 4);
        BB.setZero();
        int iobs = 0;

        std::vector<gnss_data_sats>::iterator iter;
        for (iter = obsdata.begin(); iter != obsdata.end();)
        {
            if (iter->site() != rec) // ??
            {
                iter++;
                continue;
            }

            GSYS gs = iter->gsys();

            GOBSBAND b1 = _band_index[gs][FREQ_1];
            GOBSBAND b2 = _band_index[gs][FREQ_2];

            GOBS l1 = iter->select_phase(b1);
            GOBS l2 = iter->select_phase(b2);
            GOBS p1 = iter->select_range(b1);
            GOBS p2 = iter->select_range(b2);

            if (p1 == X || l1 == X || p2 == X || l2 == X)
            {
                iter++;
                continue;
            }

            double P3 = iter->P3(p1, p2);
            double L3 = iter->L3(l1, l2);

            if (double_eq(L3, 0.0) || double_eq(P3, 0.0))
            {
                iter++;
                continue;
            }

            if (dynamic_cast<set_gproc *>(_set)->realtime())
            {
                if (iter->addprd_realtime(gnav) < 0)
                {
                    iter = obsdata.erase(iter);
                    continue;
                }
            }
            else
            {
                if (iter->addprd(gnav) < 0)
                {
                    iter = obsdata.erase(iter);
                    continue;
                }
            }

            BB(iobs, 0) = iter->satcrd()[0];
            BB(iobs, 1) = iter->satcrd()[1];
            BB(iobs, 2) = iter->satcrd()[2];
            BB(iobs, 3) = P3 + iter->clk();
            iobs++;

            iter++;
        }

        if (iobs < 4)
            return false;

        BB = BB.block(0, 0, iobs, BB.cols()); // delete zero rows

        Vector vBanc;

        gbancroft(BB, vBanc);

        rec_crd[0] = vBanc(0);
        rec_crd[1] = vBanc(1);
        rec_crd[2] = vBanc(2);

        return true;
    }

    gnss_proc_OUTLIER::gnss_proc_OUTLIER(set_base *settings)
    {
        if (1)
        {
            //lvhb shut down the outputting in 202011
            if (_debug_outliers)
            {
                if (_debug_outliers->is_open())
                    _debug_outliers->close();
                delete _debug_outliers;
                _debug_outliers = nullptr;
            }
        }
        else
        {
            _debug_outliers = new base_iof;
            _debug_outliers->tsys(base_time::GPS);
            _debug_outliers->mask("debug_outliers.log");
            _debug_outliers->append(false);
        }
        _observ = dynamic_cast<set_gproc *>(settings)->obs_combin();
        _frequency = dynamic_cast<set_gproc *>(settings)->frequency();

        _freq_index[GPS] = dynamic_cast<set_gnss *>(settings)->freq_index(GPS);
        _freq_index[GAL] = dynamic_cast<set_gnss *>(settings)->freq_index(GAL);
        _freq_index[GLO] = dynamic_cast<set_gnss *>(settings)->freq_index(GLO);
        _freq_index[BDS] = dynamic_cast<set_gnss *>(settings)->freq_index(BDS);
        _freq_index[QZS] = dynamic_cast<set_gnss *>(settings)->freq_index(QZS);

        _single_mix = {OBSCOMBIN::RAW_SINGLE, OBSCOMBIN::RAW_MIX, OBSCOMBIN::IF_P1};
    }

    gnss_proc_OUTLIER::gnss_proc_OUTLIER(set_base *settings, base_log spdlog) : _spdlog(spdlog)
    {
        if (1)
        {
            //lvhb shut down the outputting in 202011
            if (_debug_outliers)
            {
                if (_debug_outliers->is_open())
                    _debug_outliers->close();
                delete _debug_outliers;
                _debug_outliers = nullptr;
            }
        }
        else
        {
            _debug_outliers = new base_iof;
            _debug_outliers->tsys(base_time::GPS);
            _debug_outliers->mask("debug_outliers.log");
            _debug_outliers->append(false);
        }
        _observ = dynamic_cast<set_gproc *>(settings)->obs_combin();
        _frequency = dynamic_cast<set_gproc *>(settings)->frequency();

        _freq_index[GPS] = dynamic_cast<set_gnss *>(settings)->freq_index(GPS);
        _freq_index[GAL] = dynamic_cast<set_gnss *>(settings)->freq_index(GAL);
        _freq_index[GLO] = dynamic_cast<set_gnss *>(settings)->freq_index(GLO);
        _freq_index[BDS] = dynamic_cast<set_gnss *>(settings)->freq_index(BDS);
        _freq_index[QZS] = dynamic_cast<set_gnss *>(settings)->freq_index(QZS);

        _single_mix = {OBSCOMBIN::RAW_SINGLE, OBSCOMBIN::RAW_MIX, OBSCOMBIN::IF_P1};
    }

    gnss_proc_OUTLIER::~gnss_proc_OUTLIER()
    {
        if (_debug_outliers)
        {
            if (_debug_outliers->is_open())
                _debug_outliers->close();
            delete _debug_outliers;
            _debug_outliers = nullptr;
        }
    }

    void gnss_proc_OUTLIER::flagRangeOutliers(std::shared_ptr<gnss_data_obs_manager> ObsPre, std::shared_ptr<gnss_data_obs_manager> Obs, double sampling)
    {
        std::ostringstream os;
        os.str("");
        base_time crt_time = Obs->epoch();
        std::string crt_site = Obs->site();

        std::vector<GOBS> obs_vec = Obs->obs();

        // Gao Y et al. Modeling and estimation of C1-P1 bias in GPS receivers[J]. Journal of Geodesy, 2001.
        GSYS gsys = Obs->gsys();
        GOBSBAND b1 = GNSS_BAND_PRIORITY.at(gsys)[1];
        GOBS ref_obsP = Obs->select_range(b1);
        double ref_P = Obs->obs_C(ref_obsP); // m

        for (auto obs_type : obs_vec)
        {
            // skip not code obs
            if (!gnss_data_obs(obs_type).is_code())
                continue;
            // skip ref obs(add case: ref_obsP == X)
            if (obs_type == ref_obsP || ref_obsP == X)
                continue;

            gnss_data_obs gobs(obs_type);
            GOBSBAND bX = gobs.band();

            double obs_P = Obs->obs_C(obs_type);
            double diff_P = abs(obs_P - ref_P);

            double value = (b1 == bX) ? 10 : 30; // meters

            os << std::fixed << std::setw(10) << " PP[m] " << Obs->sat() << "  " << std::setw(6) << ObsPre->epoch().sod() << std::setw(6) << crt_time.sod() << "  " << gobs2str(obs_type) << "  " << gobs2str(ref_obsP) << std::setw(15) << std::setprecision(4) << diff_P << std::endl;

            if (diff_P > value)
            {
                Obs->addoutliers(obs_type, 1);
                Obs->addoutliers(ref_obsP, 1);
            }
        }

        // Assume GNSS observations are OK

        if (crt_time.diff(ObsPre->epoch()) > sampling)
            return;

        for (auto obs_doppler : obs_vec)
        {
            // skip not doppler obs
            if (!gnss_data_obs(obs_doppler).is_doppler())
                continue;

            double crt_doppler = Obs->getobs(obs_doppler);    // cycle / s
            double pre_doppler = ObsPre->getobs(obs_doppler); // cycle / s

            // check Obstype Gap
            if (double_eq(pre_doppler, NULL_GOBS) || double_eq(crt_doppler, NULL_GOBS))
                continue;

            // ???????????????, cycle/s
            double delta_doppler = crt_doppler - pre_doppler;

            // Codeless (???α?????)
            if (obs_doppler == D1N)
            {
                if (abs(delta_doppler) > 10)
                    Obs->addoutliers(obs_doppler, 1);
                continue;
            }

            std::string strGOBS = gobs2str(obs_doppler);
            strGOBS.replace(0, 1, "C");
            GOBS obs_range = str2gobs(strGOBS);

            if (Obs->getoutliers(obs_range) > 0)
                continue;

            GOBSBAND b = gnss_data_obs(obs_doppler).band();
            crt_doppler *= Obs->wavelength(b);    // m/s
            pre_doppler *= ObsPre->wavelength(b); // m/s

            double crt_range = Obs->obs_C(obs_range);    // m
            double pre_range = ObsPre->obs_C(obs_range); // m

            if (double_eq(crt_range, NULL_GOBS) || double_eq(pre_range, NULL_GOBS))
                continue;

            double delta_PD = crt_range - pre_range + 0.5 * (crt_doppler + pre_doppler) * sampling; // m
            os << std::fixed << std::setw(10) << " D[c/s] " << Obs->sat() << "  " << std::setw(6) << ObsPre->epoch().sod() << std::setw(6) << crt_time.sod() << "  " << gobs2str(obs_doppler) << std::setw(15) << std::setprecision(4) << delta_doppler << std::endl;
            os << std::fixed << std::setw(10) << " PD[m] " << Obs->sat() << "  " << std::setw(6) << ObsPre->epoch().sod() << std::setw(6) << crt_time.sod() << "  " << gobs2str(obs_range) << "  " << gobs2str(obs_doppler) << std::setw(15) << std::setprecision(4) << delta_PD << std::endl;

            if (abs(delta_doppler) > 10 && abs(delta_PD) > 2) // cycle/s, m
            {
                Obs->addoutliers(obs_doppler, 1); // doppler
                Obs->addoutliers(obs_range, 1);   // range
            }
            else if (abs(delta_PD) > 10) // m
            {
                Obs->addoutliers(obs_range, 1); // range
            }
        }

        // Print flt results
        if (_debug_outliers)
        {
            _debug_outliers->write(os.str().c_str(), os.str().size());
            _debug_outliers->flush();
        }
    }

    void gnss_proc_OUTLIER::excludeBadObs(std::vector<gnss_data_sats> &obsdata)
    {
        std::vector<gnss_data_sats>::iterator it;
        bool valid_SNR, valid_Code, valid_Phase, Freq_Lack;
        std::set<FREQ_SEQ> nFreq;

        for (it = obsdata.begin(); it != obsdata.end();)
        {
            valid_SNR = true;
            valid_Code = true;
            valid_Phase = true;
            Freq_Lack = false;
            nFreq.clear();
            double P1 = 0.0;

            GSYS gsys = it->gsys();
            std::set<GOBSBAND> band_avail;

            band_avail = it->band_avail(true);

            if (band_avail.size() == 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "{} Erase {} for [no freq]", it->epoch().str_hms(), it->sat());
                it = obsdata.erase(it);
                continue;
            }
            else
            {
                for (auto band : band_avail)
                {
                    // skip on used band
                    if (_freq_index[gsys].find(band) == _freq_index[gsys].end())
                        continue;

                    // skip exceed frequency setting
                    if (_freq_index[gsys][band] > _frequency)
                        continue;

                    // freq num count
                    if (nFreq.find(_freq_index[gsys][band]) == nFreq.end())
                        nFreq.insert(_freq_index[gsys][band]);

                    GOBS obsP = it->select_range(band);
                    GOBS obsL = it->select_phase(band);
                    GOBS obsS;

                    obsS = pha2snr(obsL);

                    double obs_snr = it->getobs(obsS);
                    if (obs_snr <= 10.0 && !double_eq(obs_snr, 0.0))
                        valid_SNR = false;

                    double obs_code = it->getobs(obsP);
                    //if (obs_code <= 1.9e7) valid_Code = false;

                    double obs_phase = it->obs_L(obsL); // meter
                    //if (obs_phase <= 1.9e7) valid_Phase = false;
                    if (_observ != OBSCOMBIN::RAW_MIX && double_eq(obs_phase, 0.0))
                        valid_Phase = false;

                    if (_single_mix.find(_observ) == _single_mix.end())
                    {
                        if (_freq_index[gsys][band] == FREQ_1)
                        {
                            P1 = obs_code;
                        }
                        else if (!double_eq(P1, 0.0))
                        {
                            if (abs(obs_code - P1) >= 200)
                                valid_Code = false;
                        }
                    }
                }

                if (nFreq.size() == 0)
                {
                    Freq_Lack = true;
                }
                else if (nFreq.size() == 1)
                {
                    if (_single_mix.find(_observ) == _single_mix.end())
                    {
                        Freq_Lack = true;
                        if (_spdlog)
                            SPDLOG_LOGGER_WARN(_spdlog, "{} frequency missing {}", it->epoch().str_hms(), it->sat());
                    }
                }
                else if (nFreq.size() < _frequency)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_WARN(_spdlog, "{} frequency missing {}", it->epoch().str_hms(), it->sat());
                }

                if (!valid_SNR || !valid_Phase || !valid_Code || Freq_Lack)
                {
#ifdef DEBUG_pppRTK
                    std::cout << " Erase data:  " << it->epoch().str_hms() << "   " << it->sat() << "  SNR= " << (int)valid_SNR << "  Phase= " << (int)valid_Phase
                         << "  Code= " << (int)valid_Code << " BandNum= " << it->band_avail(true).size() << std::endl;
#endif
                    std::vector<GOBS> obss = it->obs();
#ifdef DEBUG_pppRTK
                    std::cout << it->sat();
                    for (auto iii : obss)
                        std::cout << "   " + gobs2str(iii) << std::fixed << std::setw(18) << std::setprecision(3) << it->getobs(iii);
                    std::cout << std::endl;
#endif
                    it = obsdata.erase(it);
                    continue;
                }

                it++;
            }
        }
    }

    // Constructor
    // ----------
    gnss_proc_quality_control::gnss_proc_quality_control(base_log spdlog, set_base *settings, gnss_all_nav *gnav) : _gnav(gnav),
                                                                                                   _spdlog(spdlog),
                                                                                                   _bds_codebias_cor(settings),
                                                                                                   _smooth_range(settings),
                                                                                                   _outliers_proc(settings, nullptr)
    {
        // std::set spdlog
        if (nullptr == spdlog)
        {
            throw std::logic_error("your spdlog in gnss_proc_quality_control::gnss_proc_quality_control i nullptr");
        }
        _outliers_proc.setLog(spdlog);
    }

    // Constructor
    // ----------
    gnss_proc_quality_control::gnss_proc_quality_control(set_base *settings, gnss_all_nav *gnav) : _gnav(gnav),
                                                                                  _bds_codebias_cor(settings),
                                                                                  _smooth_range(settings),
                                                                                  _outliers_proc(settings, nullptr)
    {
    }

    // Destructor
    // ----------
    gnss_proc_quality_control::~gnss_proc_quality_control()
    {
    }

    int gnss_proc_quality_control::processOneEpoch(const base_time &now, const std::string &rec, Triple &rec_crd, std::vector<gnss_data_sats> &obsdata)
    {
        this->_outliers_proc.excludeBadObs(obsdata);
        this->_bds_codebias_cor.apply_IGSO_MEO(rec, rec_crd, _gnav, obsdata);
        this->_smooth_range.smooth_range_obs(obsdata, now);

        return 1;
    }

} // namespace
