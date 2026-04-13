#include "hwa_set_amb.h"
#include "hwa_set_inp.h"
#include "hwa_base_string.h"
#include "hwa_base_file.h"
#include "hwa_base_time.h"
#include "hwa_set_gbase.h"
#include "hwa_gnss_amb_ow.h"
#include "hwa_gnss_all_amb.h"
#include "hwa_gnss_all_bias.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_amb_common.h"
#include "hwa_gnss_coder_rinexo.h"
#include <thread>

namespace hwa_gnss
{
    gnss_all_amb::gnss_all_amb()
    {
    }

    gnss_all_amb::gnss_all_amb(set_base *gset, base_log spdlog)
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
        add_set(gset);
    }

    gnss_all_amb::~gnss_all_amb()
    {
    }

    void gnss_all_amb::add_set(set_base *gset)
    {
        _gset = gset;
        _frequency = dynamic_cast<set_gproc *>(gset)->frequency();
        _obs_comb = dynamic_cast<set_gproc *>(gset)->obs_combin();
        _gsys = dynamic_cast<set_gen *>(gset)->sys();
        _required_ids = {AMB_ID::WL, AMB_ID::RAW1};
        if (_frequency > 2)
            _required_ids.insert(AMB_ID::EWL);
        if (_frequency > 3)
            _required_ids.insert(AMB_ID::EWL24);
        if (_frequency > 4)
            _required_ids.insert(AMB_ID::EWL25);

        _dd_mode = dynamic_cast<set_amb *>(gset)->dd_mode();
        _upd_mode = dynamic_cast<set_amb *>(gset)->upd_mode();
        _min_common_time = dynamic_cast<set_amb *>(gset)->min_common_time();

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
    }

    void gnss_all_amb::add_log(base_log spdlog)
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
    }

    void gnss_all_amb::add_data(base_allpar *pars, gnss_all_obs *obs, gnss_all_bias *allbias, gnss_all_obj *obj, gnss_data_upd *upd)
    {
        std::string func_id = "gnss_all_amb::add_data";
        this->_gobs = obs;
        this->_gbias = allbias;
        this->_gpars = pars;
        this->_gobj = obj;
        this->_gupd = upd;
        // ADD DATA
         base_all_proc *data = new  base_all_proc(_spdlog);
        data->Add_Data("DCB data", this->_gbias);

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "_get_amb_oneway_from_pars");
        gnss_all_amb::_get_amb_oneway_from_pars();

        if (_obs_comb == OBSCOMBIN::RAW_ALL)
            gnss_all_amb::_get_wl_from_pars();
        else
            gnss_all_amb::_get_wl_from_gobs();

        _gpars->delAllParam();
    }

    void gnss_all_amb::add_bdeci(gnss_amb_bdeci *EWL, gnss_amb_bdeci *WL, gnss_amb_bdeci *NL)
    {
        _evaluator_EWL = EWL;
        _evaluator_WL = WL;
        _evaluator_NL = NL;
    }

    bool gnss_all_amb::cmb_sd_by_sat()
    {
        std::string func_id = "_cmb_sd_ambs";
        // purpose: combin SD ambiguities for each receiver
        if (_sat_list.empty())
            return false;
        std::vector<std::string> vsat(_sat_list.begin(), _sat_list.end());

        std::set<AMB_ID> required_ids;
        if (_obs_comb == OBSCOMBIN::RAW_ALL)
            required_ids = {AMB_ID::RAW1, AMB_ID::RAW2, AMB_ID::WL};
        else
            required_ids = {AMB_ID::IF12, AMB_ID::WL};
        for (const auto &rec : _rec_list)
        {
            if (_all_ow_ambs.find(rec) == _all_ow_ambs.end())
                continue;
            const auto &ow_rec = _all_ow_ambs.at(rec);
            for (size_t i = 0; i < vsat.size(); ++i)
            {
                if (ow_rec.find(vsat[i]) == ow_rec.end())
                    continue;
                for (size_t j = i + 1; j < vsat.size(); ++j)
                {
                    if (ow_rec.find(vsat[j]) == ow_rec.end())
                        continue;
                    // check satellite system
                    if (vsat[i].substr(0, 1) != vsat[j].substr(0, 1))
                        continue;

                    const auto &ows1 = ow_rec.at(vsat[i]);
                    const auto &ows2 = ow_rec.at(vsat[j]);
                    for (const auto &ow1 : ows1)
                    {
                        if (!ow1->check_amb(required_ids))
                            continue;
                        for (const auto &ow2 : ows2)
                        {
                            if (!ow2->check_amb(required_ids))
                                continue;
                            // check common time
                            if (!check_sd_amb(*ow1, *ow2, _min_common_time))
                                continue;
                            _all_sd_ambs[rec].push_back(std::make_shared<gnss_amb_SD>(*ow1, *ow2));
                        }
                    }
                } // loop for second satellite
            }     // loop for first satellite
        }         // loop for receivers
        return true;
    }

    double gnss_all_amb::_get_sd_upd(const UPDTYPE &tp, const std::string &sat1, const std::string &sat2, const base_time &t)
    {
        if (!_gupd)
            return 0.0;

        one_epoch_upd epoch_upd = _gupd->get_epo_upd(tp, t);
        if (epoch_upd.find(sat1) == epoch_upd.end() || epoch_upd.find(sat2) == epoch_upd.end())
            return 0.0;
        double upd1 = epoch_upd[sat1]->value;
        double upd2 = epoch_upd[sat2]->value;
        double sig1 = epoch_upd[sat1]->sigma;
        double sig2 = epoch_upd[sat2]->sigma;
        int npoint1 = epoch_upd[sat1]->npoint;
        int npoint2 = epoch_upd[sat2]->npoint;
        bool isvalid = (tp == UPDTYPE::NL) ? (sig1 < 0.1 && sig2 < 0.1 && npoint1 > 3 && npoint2 > 3) : (sig1 < 0.2 && sig2 < 0.2 && npoint1 > 2 && npoint2 > 2);

        return isvalid ? (-upd1 + upd2) : 0.0;
    }

    bool gnss_all_amb::cmb_one_dd(const std::pair<std::string, std::string> &bl)
    {
        std::string func_id = "_cmb_one_dd";

        if (_all_sd_ambs.find(bl.first) == _all_sd_ambs.end() || _all_sd_ambs.find(bl.second) == _all_sd_ambs.end())
            return false;

        const auto &vsd1 = _all_sd_ambs[bl.first];
        const auto &vsd2 = _all_sd_ambs[bl.second];
        if (vsd1.empty() || vsd2.empty())
            return false;

        std::set<AMB_ID> required_ids;
        if (_obs_comb == OBSCOMBIN::RAW_ALL)
            required_ids = {AMB_ID::RAW1, AMB_ID::RAW2, AMB_ID::WL};
        else
            required_ids = {AMB_ID::IF12, AMB_ID::WL};
        for (const auto &sd1 : vsd1)
        {
            if (!sd1->check_amb(required_ids))
                continue;
            for (const auto &sd2 : vsd2)
            {
                if (!sd2->check_amb(required_ids))
                    continue;
                if (!check_dd_amb(*sd1, *sd2, _min_common_time))
                    continue;
                _all_dd_ambs[std::make_pair(bl.first, bl.second)].push_back(std::make_shared<gnss_amb_DD>(*sd1, *sd2));
            }
        }
        return true;
    }

    void gnss_all_amb::_cmb_one_dd(const std::pair<std::string, std::string> &bl, std::vector<std::shared_ptr<gnss_amb_DD>> &vdd) const
    {
        if (_all_sd_ambs.find(bl.first) == _all_sd_ambs.end() || _all_sd_ambs.find(bl.second) == _all_sd_ambs.end())
            return;

        const auto &vsd1 = _all_sd_ambs.at(bl.first);
        const auto &vsd2 = _all_sd_ambs.at(bl.second);
        if (vsd1.empty() || vsd2.empty())
            return;

        std::set<AMB_ID> required_ids;
        if (_obs_comb == OBSCOMBIN::RAW_ALL)
            required_ids = {AMB_ID::RAW1, AMB_ID::RAW2, AMB_ID::WL};
        else
            required_ids = {AMB_ID::IF12, AMB_ID::WL};
        for (const auto &sd1 : vsd1)
        {
            if (!sd1->check_amb(required_ids))
                continue;
            for (const auto &sd2 : vsd2)
            {
                if (!sd2->check_amb(required_ids))
                    continue;
                if (!check_dd_amb(*sd1, *sd2, _min_common_time))
                    continue;
                vdd.push_back(std::make_shared<gnss_amb_DD>(*sd1, *sd2));
            }
        }
    }

    void gnss_all_amb::_fix_ambiguity(const std::shared_ptr<gnss_amb> &amb)
    {
        int freq = amb->frequency();
        bool is_single = amb->amb_type() == AMB_TYPE::SD;
        // fix EWL25 ambiguity
        if (freq > 4 && amb->check_amb({AMB_ID::EWL25}))
        {
            double val = amb->value(AMB_ID::EWL25);
            if (is_single && _upd_mode == UPD_MODE::UPD)
            {
                double cor = _get_sd_upd(UPDTYPE::EWL25, amb->sats().first, amb->sats().second, base_time(EWL25_IDENTIFY));
                amb->set_upd_cor(AMB_ID::EWL25, cor);
                val += cor;
            }
            amb->set_value(AMB_ID::EWL25, val);
            amb->set_fixed(AMB_ID::EWL25, _evaluator_EWL->bdeci(val, amb->sigma(AMB_ID::EWL25), 1));
        }
        // fix EWL24 ambiguity
        if (freq > 3 && amb->check_amb({AMB_ID::EWL24}))
        {
            double val = amb->value(AMB_ID::EWL24);
            if (is_single && _upd_mode == UPD_MODE::UPD)
            {
                double cor = _get_sd_upd(UPDTYPE::EWL25, amb->sats().first, amb->sats().second, base_time(EWL24_IDENTIFY));
                amb->set_upd_cor(AMB_ID::EWL24, cor);
                val += cor;
            }
            amb->set_value(AMB_ID::EWL24, val);
            amb->set_fixed(AMB_ID::EWL24, _evaluator_EWL->bdeci(val, amb->sigma(AMB_ID::EWL24), 1));
        }
        // fix EWL ambiguity
        if (freq > 2 && amb->check_amb({AMB_ID::EWL}))
        {
            double val = amb->value(AMB_ID::EWL);
            if (is_single && _upd_mode == UPD_MODE::UPD)
            {
                double cor = _get_sd_upd(UPDTYPE::EWL25, amb->sats().first, amb->sats().second, base_time(EWL_IDENTIFY));
                amb->set_upd_cor(AMB_ID::EWL, cor);
                val += cor;
            }
            amb->set_value(AMB_ID::EWL, val);
            amb->set_fixed(AMB_ID::EWL, _evaluator_EWL->bdeci(val, amb->sigma(AMB_ID::EWL), 1));
        }
        // fix WL ambiguity
        double val = amb->value(AMB_ID::WL);
        if (is_single && _upd_mode != UPD_MODE::OSB)
        {
            double cor = _get_sd_upd(UPDTYPE::WL, amb->sats().first, amb->sats().second, base_time(WL_IDENTIFY));
            amb->set_upd_cor(AMB_ID::WL, cor);
            val += cor;
        }
        amb->set_value(AMB_ID::WL, val);
        amb->set_fixed(AMB_ID::WL, _evaluator_WL->bdeci(val, amb->sigma(AMB_ID::WL), 1));
        if (!amb->fixed(AMB_ID::WL))
            return;

        // fix L1 ambiguity
        std::string sat = amb->sats().first;
        gnss_data_obs_manager gnss(_spdlog);
        gnss.sat(sat);
        GSYS gsys = gnss_sys::sat2gsys(sat);
        double f1 = gnss.frequency(_band_index[gsys][FREQ_1]);
        double f2 = gnss.frequency(_band_index[gsys][FREQ_2]);
        double n_nl = 0;
        if (_obs_comb == OBSCOMBIN::RAW_ALL)
        {
            /*
            * Note: there are 3 methods to calculate uncombined N1 ambiguity, 
            * (1) by calculating IF ambiguity
            * (2) = (N1 + N2 + round(WL)) / 2
            * (3) = N1
            * the first method shows the highest fixing rate
            */
            double n_if = CLIGHT / (f1 * f1 - f2 * f2) * (f1 * amb->value(AMB_ID::RAW1) - f2 * amb->value(AMB_ID::RAW2)); // in meter
            n_nl = n_if * (f1 + f2) / CLIGHT - f2 / (f1 - f2) * round(amb->value(AMB_ID::WL));                            // in cycle
                                                                                                                          //n_nl = amb.value(AMB_ID::RAW1);
                                                                                                                          //n_nl = (amb.value(AMB_ID::RAW1) + amb.value(AMB_ID::RAW2) + round(amb.value(AMB_ID::WL))) / 2;
        }
        else
        {
            n_nl = amb->value(AMB_ID::IF12) * (f1 + f2) / CLIGHT - f2 / (f1 - f2) * round(amb->value(AMB_ID::WL));
        }
        if (double_eq(n_nl, 0))
            return;
        // Note: the sign of NL UPD is negative, float(NL) = int(NL) - upd_nl, while float(WL) = int(WL) + upd_wl
        if (is_single && _upd_mode == UPD_MODE::UPD)
        {
            double cor = _get_sd_upd(UPDTYPE::NL, amb->sats().first, amb->sats().second, amb->t_end());
            amb->set_upd_cor(AMB_ID::RAW1, cor);
            n_nl -= cor;
        }
        amb->set_value(AMB_ID::RAW1, n_nl);
        amb->set_sigma(AMB_ID::RAW1, 0.05);
        amb->set_fixed(AMB_ID::RAW1, _evaluator_NL->bdeci(n_nl, 0.05, 1));
    }

    void gnss_all_amb::_fix_one_ow(const std::shared_ptr<gnss_amb_SD> &sd)
    {
        int idx1 = sd->idx()[0] - 1, idx2 = sd->idx()[1] - 1;
        if (idx1 < 0 || idx2 < 0)
            return;
        const auto &ow1 = _vow[idx1];
        const auto &ow2 = _vow[idx2];
        // all std::fixed or all not std::fixed
        if (ow1->is_used == ow2->is_used)
            return;

        const auto &amb_id = sd->amb_ids();
        std::set<AMB_ID> proc_id = {AMB_ID::RAW1, AMB_ID::WL, AMB_ID::EWL, AMB_ID::EWL24, AMB_ID::EWL25};
        for (const auto &id : proc_id)
        {
            if (!sd->fixed(id))
                continue;
            if (ow1->is_used && ow1->fixed(id) && !ow2->fixed(id))
            {
                ow2->add_amb(id, round(ow1->value(id)) - round(sd->value(id)), 0.05, 0, true);
            }
            else if (ow2->is_used && !ow1->fixed(id) && ow2->fixed(id))
            {
                ow1->add_amb(id, round(ow2->value(id)) + round(sd->value(id)), 0.05, 0, true);
            }
        }

        std::cout << "fix OW ambiguity:" << std::endl;
        if (ow2->is_used)
        {
            std::cout << std::setw(8) << idx1 << "  " << std::setw(5) << ow1->rec() << std::setw(5) << ow1->sat();
            for (const auto &id : proc_id)
            {
                if (!ow1->fixed(id))
                    continue;
                std::cout << std::setw(8) << gnss_amb::ambId2str(id) << std::setw(8) << std::setprecision(3) << ow1->value(id);
            }
        }
        else
        {
            std::cout << std::setw(8) << idx2 << "  " << std::setw(5) << ow2->rec() << std::setw(5) << ow2->sat();
            for (const auto &id : proc_id)
            {
                if (!ow2->fixed(id))
                    continue;
                std::cout << std::setw(8) << gnss_amb::ambId2str(id) << std::setw(8) << std::setprecision(3) << ow2->value(id);
            }
        }
        std::cout << std::endl;

        sd->is_used = true;
        ow1->is_used = true;
        ow2->is_used = true;
        _all_ow_ambs[ow1->rec()][ow1->sat()].push_back(ow1);
        _all_ow_ambs[ow2->rec()][ow2->sat()].push_back(ow2);
    }

    void gnss_all_amb::process_dd_ambs(const std::vector<std::pair<std::string, std::string>> &baselines)
    {
        for (const auto &id : _required_ids)
        {
            for (const auto &sys : _gsys)
            {
                num_all[id][gnss_sys::str2gsys(sys)] = 0;
                num_fixed[id][gnss_sys::str2gsys(sys)] = 0;
            }
        }
        base_mutex add_mtx;
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < baselines.size(); ++i)
        {
            std::pair<std::string, std::string> bl = baselines.at(i);
            std::vector<std::shared_ptr<gnss_amb_DD>> dd_ambs;
            _cmb_one_dd(bl, dd_ambs);
            if (dd_ambs.empty())
                continue;
            for (auto &amb : dd_ambs)
            {
                _fix_ambiguity(amb);
            }
            // sort all dd by arc length
            sort(dd_ambs.begin(), dd_ambs.end(),
                 [](const std::shared_ptr<gnss_amb_DD> &dd1, const std::shared_ptr<gnss_amb_DD> &dd2)
                 { return dd1->seslen() > dd2->seslen(); });
            int norder = 4;
            /*
            * select and sort all independent DD ambs by following orders:
            * 1. all NL, WL, EWL are std::fixed (> 2-freq)
            * 2. all NL, WL are std::fixed (2-freq)
            * 3. NL not std::fixed, but WL or EWL is std::fixed
            * 4. no amb is std::fixed
            */
            // bool isfirst = true;
            int ndef = 0;
            int order0 = _frequency > 2 ? 1 : 2;

            std::vector<std::shared_ptr<gnss_amb_DD>> dd_used;
            std::map<AMB_ID, std::map<GSYS, int>> num_fixed_bl;
            std::map<AMB_ID, std::map<GSYS, int>> num_all_bl;
            double *pdE = new double[_num_ows * (8000 + 100)];
            double *pdC = new double[8000 + 100];

            for (int iorder = order0; iorder < norder; ++iorder)
            {
                for (auto iter = dd_ambs.begin(); iter != dd_ambs.end(); ++iter)
                {
                    if ((*iter)->is_used || (*iter)->is_dependent)
                        continue;
                    if (iorder == 1 && !((*iter)->fixed(AMB_ID::RAW1) && (*iter)->freq_fixed() > 2))
                        continue;
                    if (iorder == 2 && !((*iter)->fixed(AMB_ID::RAW1) && (*iter)->fixed(AMB_ID::WL)))
                        continue;
                    if (iorder == 3 && !((*iter)->freq_fixed() > 0))
                        continue;

                    if (_check_amb_depend(pdE, pdC, _num_ows, ndef, 4, (*iter)->idx(), _num_ows, 8000))
                    {
                        //if (_check_amb_depend(isfirst, _num_ows, &ndef, 4, (*iter)->idx(), _num_ows, 8000)) {
                        (*iter)->is_dependent = true;
                        continue;
                    }
                    // isfirst = false;
                    (*iter)->is_used = true;
                    dd_used.push_back(*iter);
                    // summary
                    _summary_amb(**iter, num_fixed_bl, num_all_bl);
                }
            }

            delete[] pdE;
            delete[] pdC;
            add_mtx.lock();
            // _all_dd_ambs[bl] = dd_ambs;
            _vdd.insert(_vdd.end(), dd_used.begin(), dd_used.end());
            _update_amb_sum(num_fixed_bl, num_all_bl);
            add_mtx.unlock();
        }
    }

    void gnss_all_amb::process_sd_ambs()
    {
        for (const auto &id : _required_ids)
        {
            for (const auto &sys : _gsys)
            {
                num_all[id][gnss_sys::str2gsys(sys)] = 0;
                num_fixed[id][gnss_sys::str2gsys(sys)] = 0;
            }
        }

        std::vector<std::string> vrec(_rec_list.begin(), _rec_list.end());

        base_mutex add_mtx;
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < vrec.size(); ++i)
        {
            const std::string &rec = vrec.at(i);
            if (_all_sd_ambs.find(rec) == _all_sd_ambs.end())
                continue;
            std::vector<std::shared_ptr<gnss_amb_SD>> sd_ambs = _all_sd_ambs[rec];
            if (sd_ambs.empty())
                continue;

            for (auto &amb : sd_ambs)
            {
                _fix_ambiguity(amb);
            }
            // sort all sd by arc length
            sort(sd_ambs.begin(), sd_ambs.end(),
                 [](const std::shared_ptr<gnss_amb_SD> &sd1, const std::shared_ptr<gnss_amb_SD> &sd2)
                 { return sd1->seslen() > sd2->seslen(); });

            int norder = 4;
            /*
            * select and sort all independent DD ambs by following orders:
            * 1. all NL, WL, EWL are std::fixed (> 2-freq)
            * 2. all NL, WL are std::fixed (2-freq)
            * 3. NL not std::fixed, but WL or EWL is std::fixed
            * 4. no amb is std::fixed
            */
            // bool isfirst = true;
            int ndef = 0;
            int order0 = _frequency > 2 ? 1 : 2;

            std::vector<std::shared_ptr<gnss_amb_SD>> sd_used;
            std::map<AMB_ID, std::map<GSYS, int>> num_fixed_bl;
            std::map<AMB_ID, std::map<GSYS, int>> num_all_bl;
            double *pdE = new double[_num_ows * (8000 + 100)];
            double *pdC = new double[8000 + 100];

            for (int iorder = order0; iorder < norder; ++iorder)
            {
                for (auto iter = sd_ambs.begin(); iter != sd_ambs.end(); ++iter)
                {
                    if ((*iter)->is_used || (*iter)->is_dependent)
                        continue;
                    if (iorder == 1 && !((*iter)->fixed(AMB_ID::RAW1) && (*iter)->freq_fixed() > 2))
                        continue;
                    if (iorder == 2 && !((*iter)->fixed(AMB_ID::RAW1) && (*iter)->fixed(AMB_ID::WL)))
                        continue;
                    if (iorder == 3 && !((*iter)->freq_fixed() > 0))
                        continue;
                    // if (_check_amb_depend(isfirst, _num_ows, &ndef, 2, (*iter)->idx(), _num_ows, 8000)) {
                    if (_check_amb_depend(pdE, pdC, _num_ows, ndef, 2, (*iter)->idx(), _num_ows, 8000))
                    {
                        (*iter)->is_dependent = true;
                        continue;
                    }
                    // isfirst = false;
                    (*iter)->is_used = true;
                    sd_used.push_back(*iter);
                    // summary
                    _summary_amb(**iter, num_fixed_bl, num_all_bl);
                }
            }

            delete[] pdE;
            delete[] pdC;
            add_mtx.lock();
            _vsd.insert(_vsd.end(), sd_used.begin(), sd_used.end());
            _update_amb_sum(num_fixed_bl, num_all_bl);
            add_mtx.unlock();
        } // loop for receivers
    }

    void gnss_all_amb::fix_ow_by_sd()
    {
        std::set<AMB_ID> proc_id = {AMB_ID::RAW1, AMB_ID::WL, AMB_ID::EWL, AMB_ID::EWL24, AMB_ID::EWL25};
        for (const auto &rec : _rec_list)
        {
            if (_all_sd_ambs.find(rec) == _all_sd_ambs.end())
                continue;
            const auto &sd_ambs = _all_sd_ambs[rec];
            if (sd_ambs.empty())
                continue;

            for (auto &sd : sd_ambs)
                sd->is_used = false;

            // find reference ambiguity
            bool find_ref = false;
            for (const auto &sd : sd_ambs)
            {
                if (sd->is_dependent)
                    continue;
                if (sd->freq_fixed() == _frequency)
                {
                    // all frequencies are std::fixed
                    int idx1 = sd->idx()[0] - 1, idx2 = sd->idx()[1] - 1;
                    if (idx1 < 0 || idx2 < 0)
                        continue;
                    const auto &ow1 = _vow[idx1];
                    const auto &ow2 = _vow[idx2];

                    const auto &amb_id = sd->amb_ids();
                    for (const auto &id : proc_id)
                    {
                        if (!sd->fixed(id))
                            continue;
                        ow1->add_amb(id, round(sd->value(id)), 0.05, 0, true);
                        ow2->add_amb(id, 0, 0.05, 0, true);
                    }
                    find_ref = true;
                    ow1->is_used = true;
                    ow2->is_used = true;
                    sd->is_used = true;

                    std::cout << "reference OW ambiguity:" << std::endl;
                    std::cout << std::setw(8) << idx1 << "  " << std::setw(5) << ow1->rec() << std::setw(5) << ow1->sat();
                    for (const auto &id : proc_id)
                    {
                        if (!ow1->fixed(id))
                            continue;
                        std::cout << std::setw(8) << gnss_amb::ambId2str(id) << std::setw(8) << std::setprecision(3) << ow1->value(id);
                    }
                    std::cout << std::endl;
                    std::cout << std::setw(8) << idx2 << "  " << std::setw(5) << ow2->rec() << std::setw(5) << ow2->sat();
                    for (const auto &id : proc_id)
                    {
                        if (!ow2->fixed(id))
                            continue;
                        std::cout << std::setw(8) << gnss_amb::ambId2str(id) << std::setw(8) << std::setprecision(3) << ow2->value(id);
                    }
                    std::cout << std::endl;
                    break;
                }
            }
            if (!find_ref)
                continue;

            int num = 0, num0 = 0;
            for (int i = 0; i < 20; ++i)
            {
                if (num != 0 && num0 == num)
                    break;
                num0 = num;
                for (const auto &sd : sd_ambs)
                {
                    if (sd->is_used || !sd->fixed(AMB_ID::RAW1))
                        continue;
                    _fix_one_ow(sd);
                    if (sd->is_used)
                        num++;
                }
                std::cout << "iter " << std::setprecision(4) << i << ", all std::fixed amb: " << num << std::endl;
            }
        } // receiver loop
    }

    void gnss_all_amb::_get_amb_oneway_from_pars()
    {
        if (!_gpars)
            return;

        for (const auto &par : _gpars->getAllPar())
        {
            AMB_ID amb_id = gnss_amb::par2amb_id(par.parType);
            if (amb_id == AMB_ID::UNDEF)
                continue;
            std::string sys = gnss_sys::gsys2str(gnss_sys::sat2gsys(par.prn));
            if (_gsys.find(sys) == _gsys.end())
                continue;
            // frequence filter
            if (_obs_comb == OBSCOMBIN::IONO_FREE)
            {
                if (gnss_amb::ambId2str(amb_id).substr(0, 2) != "IF")
                    continue;
                if (_frequency < 5 && amb_id > AMB_ID::IF14)
                    continue;
                if (_frequency < 4 && amb_id > AMB_ID::IF13)
                    continue;
                if (_frequency < 3 && amb_id > AMB_ID::IF12)
                    continue;
            }
            else
            {
                if (gnss_amb::ambId2str(amb_id).substr(0, 3) != "RAW")
                    continue;
                if (_frequency < 5 && amb_id > AMB_ID::RAW4)
                    continue;
                if (_frequency < 4 && amb_id > AMB_ID::RAW3)
                    continue;
                if (_frequency < 3 && amb_id > AMB_ID::RAW2)
                    continue;
            }
            // BDS GEO satellites are ignored
            if (gnss_sys::bds_geo(par.prn))
                continue;
            // check ambiguity period
            if (par.end - par.beg < _min_common_time)
                continue;

            if (_rec_list.find(par.site) == _rec_list.end())
                _rec_list.insert(par.site);
            if (_sat_list.find(par.prn) == _sat_list.end())
                _sat_list.insert(par.prn);

            // whether is New OW ambiguity or not
            int ipos = _get_oneway_pos(par.site, par.prn, par.beg, par.end);
            if (ipos < 0)
            {
                gnss_amb_ow ow(par.prn, par.site, par.beg, par.end);
                double val = _amb_in_cycle(par.prn, amb_id, par.value());
                ow.add_amb(amb_id, val, 0.05, 0, false);
                _all_ow_ambs[par.site][par.prn].push_back(std::make_shared<gnss_amb_ow>(ow));
            }
            else
            {
                const auto &ids = _all_ow_ambs[par.site][par.prn][ipos]->amb_ids();
                if (ids.find(amb_id) != ids.end())
                    continue;
                double val = _amb_in_cycle(par.prn, amb_id, par.value());
                _all_ow_ambs[par.site][par.prn][ipos]->add_amb(amb_id, par.beg, par.end, val, 0.05);
            }
        } // loop for pars

        // update receiver list
        for (auto iter = _rec_list.begin(); iter != _rec_list.end(); ++iter)
        {
            if (_all_ow_ambs.find(*iter) == _all_ow_ambs.end())
                _rec_list.erase(*iter);
        }

        // get site coordinates
        for (const auto &rec : _rec_list)
        {
            Triple crd;
            if (_gpars->getCrdParam(rec, crd) > 0)
                _rec_crd.emplace(rec, crd);
        }

        // check ow ambiguities
        std::set<AMB_ID> required_ids;
        if (_obs_comb == OBSCOMBIN::RAW_ALL)
            required_ids = {AMB_ID::RAW1, AMB_ID::RAW2};
        else
            required_ids = {AMB_ID::IF12};

        _num_ows = 0;
        for (const auto &rec : _rec_list)
        {
            t_ow_obj &mow = _all_ow_ambs[rec];
            for (auto ivow = mow.begin(); ivow != mow.end();)
            {
                auto &vow = ivow->second;
                for (auto iow = vow.begin(); iow != vow.end();)
                {
                    // ensure the existence of required AMB IDs
                    if ((*iow)->t_end() - (*iow)->t_beg() < _min_common_time || !(*iow)->check_amb(required_ids))
                    {
                        iow = vow.erase(iow);
                    }
                    else
                    {
                        _sats_of_rec[rec].insert((*iow)->sat());
                        ++_num_ows;
                        (*iow)->set_idx(_num_ows);
                        _vow.push_back(*iow);
                        ++iow;
                    }
                }
                if (vow.empty())
                    ivow = mow.erase(ivow);
                else
                    ++ivow;
            }
        }
    }

    void hwa_gnss::gnss_all_amb::_get_obs_onesite(const std::string &site)
    {
        if (!_gset)
            return;
        _gobs->clear_obj();
        base_data *gdata = nullptr;
        std::vector<std::string> inp = dynamic_cast<set_inp *>(_gset)->inputs(IFMT::RINEXO_INP);
        auto itINP = inp.begin();
        for (size_t i = 0; i < inp.size() && itINP != inp.end(); ++i, ++itINP)
        {
            // Get the file format/path, which will be used in decoder
            std::string path(*itINP);
            std::string id("ID" + base_type_conv::int2str(i));

            // check the obs file valid
            auto pos = path.find_last_of("o");
            std::string name = path.substr(pos - 11, 4);
            transform(name.begin(), name.end(), name.begin(), ::toupper);
            if (name != site)
                continue;

            // Check the file path
            if (path.substr(0, 7) != "file://")
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "path is not file (skipped)!");
                continue;
            }

            base_io *gio = nullptr;
            base_coder *gcoder = nullptr;

            gdata = _gobs;
            gcoder = new gnss_coder_rinexo(_gset, "", 5000);

            // runepoch for the time costed each epoch (i guess)
            base_time runepoch(base_time::GPS);
            base_time lstepoch(base_time::GPS);
            // READ DATA FROM FILE
            if (gcoder)
            {
                // prepare gio for the file
                gio = new base_file(_spdlog);
                gio->spdlog(_spdlog);
                gio->path(path);

                // Put the file into gcoder
                gcoder->clear();
                gcoder->path(path);
                gcoder->spdlog(_spdlog);
                // Put the data container into gcoder
                gcoder->add_data(id, gdata);
                gcoder->add_data("OBJ", _gobj);
                gio->coder(gcoder);

                // Read the data from file here
                gio->run_read();
                // Write the information of reading process to spdlog file
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "READ : " + path + " time: " + base_type_conv::dbl2str(lstepoch.diff(runepoch)) + " sec");
                delete gio;
                gio = nullptr;
                delete gcoder;
                gcoder = nullptr;
            }
        }
    }

    std::pair<double, double> gnss_all_amb::_get_ow_wl(const std::vector<hwa_spt_obsmanager> &vobs)
    {
        if (vobs.empty())
            return {0.0, 999.0};
        std::list<std::pair<double, double>> mwobs;
        for (const auto &obs : vobs)
        {
            obs->apply_bias(_gbias);
            GOBSBAND b1 = _band_index[obs->gsys()][FREQ_1];
            GOBSBAND b2 = _band_index[obs->gsys()][FREQ_2];

            gnss_data_obs gobsP1 = gnss_data_obs(obs->select_range(b1, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsL1 = gnss_data_obs(obs->select_phase(b1, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsP2 = gnss_data_obs(obs->select_range(b2, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsL2 = gnss_data_obs(obs->select_phase(b2, _obs_comb == OBSCOMBIN::RAW_ALL));
            // ignore invalid obs
            if (gobsP1.type() == GOBSTYPE::TYPE || gobsL1.type() == GOBSTYPE::TYPE || gobsP2.type() == GOBSTYPE::TYPE || gobsL2.type() == GOBSTYPE::TYPE)
                continue;

            double mw = obs->MW_cycle(gobsL1, gobsL2, gobsP1, gobsP2);
            if (std::isnan(mw) || double_eq(mw, 0.0))
                continue;
            mwobs.push_back({mw, 1.0});
        }
        if (mwobs.empty())
            return {0.0, 999.0};

        double mean_mw = 0.0, _sigma = 0.0, mean_sigma = 0.0;
        getMeanWgt(true, mwobs, mean_mw, _sigma, mean_sigma);
        if (double_eq(mean_sigma, 0.0))
            mean_sigma = 999.0;

        return {mean_mw, mean_sigma};
    }

    std::pair<double, double> gnss_all_amb::_get_ow_ewl(const std::vector<hwa_spt_obsmanager> &vobs, const AMB_ID &mode)
    {
        if (vobs.empty())
            return {0.0, 999.0};
        if (mode != AMB_ID::EWL && mode != AMB_ID::EWL24 && mode != AMB_ID::EWL25)
            return {0.0, 999.0};
        GOBSBAND b1 = _band_index[vobs[0]->gsys()][FREQ_1];
        GOBSBAND b2 = _band_index[vobs[0]->gsys()][FREQ_2];
        GOBSBAND b3 = BAND;
        switch (mode)
        {
        case AMB_ID::EWL25:
            b3 = _band_index[vobs[0]->gsys()][FREQ_5];
            break;
        case AMB_ID::EWL24:
            b3 = _band_index[vobs[0]->gsys()][FREQ_4];
            break;
        default:
            b3 = _band_index[vobs[0]->gsys()][FREQ_3];
            break;
        }
        std::list<std::pair<double, double>> mwobs;
        for (const auto &obs : vobs)
        {
            obs->apply_bias(_gbias);

            gnss_data_obs gobsP1 = gnss_data_obs(obs->select_range(b1, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsL1 = gnss_data_obs(obs->select_phase(b1, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsP2 = gnss_data_obs(obs->select_range(b2, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsL2 = gnss_data_obs(obs->select_phase(b2, _obs_comb == OBSCOMBIN::RAW_ALL));
            gnss_data_obs gobsL3 = gnss_data_obs(obs->select_phase(b3, _obs_comb == OBSCOMBIN::RAW_ALL));

            // ignore invalid obs
            if (gobsP1.type() == GOBSTYPE::TYPE || gobsL1.type() == GOBSTYPE::TYPE || gobsP2.type() == GOBSTYPE::TYPE || gobsL2.type() == GOBSTYPE::TYPE || gobsL3.type() == GOBSTYPE::TYPE)
                continue;

            double mw = obs->EWL_cycle(gobsL1, gobsL2, gobsL3, gobsP1, gobsP2);
            if (std::isnan(mw) || double_eq(mw, 0.0))
                continue;
            mwobs.push_back({mw, 1.0});
        }
        if (mwobs.empty())
            return {0.0, 0.0};

        double mean_mw = 0.0, _sigma = 0.0, mean_sigma = 0.0;
        getMeanWgt(true, mwobs, mean_mw, _sigma, mean_sigma);
        if (double_eq(mean_sigma, 0.0))
            mean_sigma = 999.0;

        return {mean_mw, mean_sigma};
    }

    void gnss_all_amb::_summary_amb(const gnss_amb &amb)
    {
        std::set<AMB_ID> ids = amb.amb_ids();
        GSYS sys = gnss_sys::sat2gsys(amb.sats().first);
        for (const auto &id : _required_ids)
        {
            if (ids.find(id) != ids.end())
            {
                num_all[id][sys]++;
                if (amb.fixed(id))
                    num_fixed[id][sys]++;
            }
        }
    }

    void gnss_all_amb::_summary_amb(const gnss_amb &amb, t_amb_sum &nfixed, t_amb_sum &nall) const
    {
        std::set<AMB_ID> ids = amb.amb_ids();
        GSYS sys = gnss_sys::sat2gsys(amb.sats().first);
        for (const auto &id : _required_ids)
        {
            if (ids.find(id) != ids.end())
            {
                nall[id][sys]++;
                if (amb.fixed(id))
                    nfixed[id][sys]++;
            }
        }
    }

    void gnss_all_amb::_update_amb_sum(const t_amb_sum &nfixed, const t_amb_sum &nall)
    {
        for (const auto &sum_id : nall)
        {
            const AMB_ID &amb_id = sum_id.first;
            for (const auto &sum_sys : sum_id.second)
            {
                num_all[amb_id][sum_sys.first] += sum_sys.second;
                if (nfixed.find(amb_id) != nfixed.end() && nfixed.at(amb_id).find(sum_sys.first) != nfixed.at(amb_id).end())
                {
                    num_fixed[amb_id][sum_sys.first] += nfixed.at(amb_id).at(sum_sys.first);
                }
            }
        }
    }

    void gnss_all_amb::_get_wl_from_gobs()
    {
        if (!_gbias || !_gobs || !_gset)
            return;

        base_time beg = dynamic_cast<set_gen *>(_gset)->beg();
        base_time end = dynamic_cast<set_gen *>(_gset)->end();
        double intv = dynamic_cast<set_amb *>(_gset)->wl_interval();
        if (double_eq(intv, 0))
            intv = 30;

        std::set<std::string> rec_list = dynamic_cast<set_gen *>(_gset)->recs();
        OFSTREAM_MODE rom = dynamic_cast<set_gproc *>(_gset)->read_ofile_mode();
        if (rec_list.empty())
            return;

        for (const std::string &rec : rec_list)
        {
            if (_rec_list.find(rec) == _rec_list.end() || _all_ow_ambs.find(rec) == _all_ow_ambs.end())
                continue;

            if (rom == OFSTREAM_MODE::REAL_TIME)
                _get_obs_onesite(rec);
            for (auto &ow_sat : _all_ow_ambs[rec])
            {
                std::string sat(ow_sat.first);
                for (auto &ow : ow_sat.second)
                {
                    int freq = ow->frequency();
                    const auto &vobs = _gobs->obs_prn_pt(rec, sat, ow->t_beg(), ow->t_end());
                    if (freq > 4)
                    {
                        auto ewl25 = _get_ow_ewl(vobs, AMB_ID::EWL25);
                        if (!double_eq(ewl25.first, 0) || ewl25.second < 10)
                        {
                            ow->add_amb(AMB_ID::EWL25, ewl25.first, ewl25.second, 0.0, false);
                        }
                    }
                    if (freq > 3)
                    {
                        auto ewl24 = _get_ow_ewl(vobs, AMB_ID::EWL24);
                        if (!double_eq(ewl24.first, 0) || ewl24.second < 10)
                        {
                            ow->add_amb(AMB_ID::EWL24, ewl24.first, ewl24.second, 0.0, false);
                        }
                    }
                    if (freq > 2)
                    {
                        auto ewl = _get_ow_ewl(vobs, AMB_ID::EWL);
                        if (!double_eq(ewl.first, 0) || ewl.second < 10)
                        {
                            ow->add_amb(AMB_ID::EWL, ewl.first, ewl.second, 0.0, false);
                        }
                    }
                    auto wl = _get_ow_wl(vobs);
                    if (!double_eq(wl.first, 0) || wl.second < 10)
                    {
                        ow->add_amb(AMB_ID::WL, wl.first, wl.second, 0.0, false);
                    }
                } // loop for ow ambiguities
            }     // loop for sattellites
        }         // loop for receivers
        _gobs->clear_obj();
    }

    void gnss_all_amb::_get_wl_from_pars()
    {
        if (!_gset)
            return;

        std::set<std::string> rec_list = dynamic_cast<set_gen *>(_gset)->recs();
        if (rec_list.empty())
            return;
        double sig = 0.01;

        for (const std::string &rec : rec_list)
        {
            if (_rec_list.find(rec) == _rec_list.end() || _all_ow_ambs.find(rec) == _all_ow_ambs.end())
                continue;

            for (auto &ow_sat : _all_ow_ambs[rec])
            {
                std::string sat(ow_sat.first);
                for (auto &ow : ow_sat.second)
                {
                    int freq = ow->frequency();
                    std::set<AMB_ID> amb_id = ow->amb_ids();
                    if (freq > 4 && ow->check_amb({AMB_ID::RAW2, AMB_ID::RAW5}))
                    {
                        double val = ow->value(AMB_ID::RAW2) - ow->value(AMB_ID::RAW5);
                        //double sig = sqrt(ow.sigma(AMB_ID::RAW2) * ow.sigma(AMB_ID::RAW2) + ow.sigma(AMB_ID::RAW5) * ow.sigma(AMB_ID::RAW5));
                        ow->add_amb(AMB_ID::EWL25, val, sig, 0.0, false);
                    }
                    if (freq > 3 && ow->check_amb({AMB_ID::RAW2, AMB_ID::RAW4}))
                    {
                        double val = ow->value(AMB_ID::RAW2) - ow->value(AMB_ID::RAW4);
                        //double sig = sqrt(ow.sigma(AMB_ID::RAW2) * ow.sigma(AMB_ID::RAW2) + ow.sigma(AMB_ID::RAW4) * ow.sigma(AMB_ID::RAW4));
                        ow->add_amb(AMB_ID::EWL24, val, sig, 0.0, false);
                    }
                    if (freq > 2 && ow->check_amb({AMB_ID::RAW2, AMB_ID::RAW3}))
                    {
                        double val = ow->value(AMB_ID::RAW2) - ow->value(AMB_ID::RAW3);
                        //double sig = sqrt(ow.sigma(AMB_ID::RAW2) * ow.sigma(AMB_ID::RAW2) + ow.sigma(AMB_ID::RAW3) * ow.sigma(AMB_ID::RAW3));
                        ow->add_amb(AMB_ID::EWL, val, sig, 0.0, false);
                    }
                    double val = ow->value(AMB_ID::RAW1) - ow->value(AMB_ID::RAW2);
                    //double sig = sqrt(ow.sigma(AMB_ID::RAW1) * ow.sigma(AMB_ID::RAW1) + ow.sigma(AMB_ID::RAW2) * ow.sigma(AMB_ID::RAW2));
                    ow->add_amb(AMB_ID::WL, val, sig, 0.0, false);
                } // loop for ow ambiguities
            }     // loop for sattellites
        }         // loop for receivers
    }

    double gnss_all_amb::_amb_in_cycle(const std::string &sat, const AMB_ID &amb_id, const double &val)
    {
        double wavelength = 0.0;
        gnss_data_obs_manager gnss(_spdlog);
        gnss.sat(sat);
        auto gsys = gnss_sys::sat2gsys(sat);
        switch (amb_id)
        {
        case AMB_ID::RAW1:
            wavelength = CLIGHT / gnss.frequency(_band_index[gsys][FREQ_1]);
            break;
        case AMB_ID::RAW2:
            wavelength = CLIGHT / gnss.frequency(_band_index[gsys][FREQ_2]);
            break;
        case AMB_ID::RAW3:
            wavelength = CLIGHT / gnss.frequency(_band_index[gsys][FREQ_3]);
            break;
        case AMB_ID::RAW4:
            wavelength = CLIGHT / gnss.frequency(_band_index[gsys][FREQ_4]);
            break;
        case AMB_ID::RAW5:
            wavelength = CLIGHT / gnss.frequency(_band_index[gsys][FREQ_5]);
            break;
        case AMB_ID::EWL25:
            wavelength = CLIGHT / (gnss.frequency(_band_index[gsys][FREQ_2]) - gnss.frequency(_band_index[gsys][FREQ_5]));
            break;
        case AMB_ID::EWL24:
            wavelength = CLIGHT / (gnss.frequency(_band_index[gsys][FREQ_2]) - gnss.frequency(_band_index[gsys][FREQ_4]));
            break;
        case AMB_ID::EWL:
            wavelength = CLIGHT / (gnss.frequency(_band_index[gsys][FREQ_2]) - gnss.frequency(_band_index[gsys][FREQ_3]));
            break;
        case AMB_ID::WL:
            wavelength = CLIGHT / (gnss.frequency(_band_index[gsys][FREQ_1]) - gnss.frequency(_band_index[gsys][FREQ_2]));
            break;
        // IF ambiguities are processed in meters (in this class)
        case AMB_ID::IF12:
        case AMB_ID::IF13:
        case AMB_ID::IF14:
        case AMB_ID::IF15:
            wavelength = 1;
            break;
        default:
            throw(std::logic_error("wrong AMB ID"));
        }
        return val / wavelength;
    }
    bool gnss_all_amb::_check_amb_depend(double pdE[], double pdC[], int iNamb, int &iNdef, int iN_oneway,
                                       std::vector<int> arriIpt2ow, int iNdim_ow, int iNdim_for_check) const
    {
        if (iNdef >= iNdim_for_check)
            return true;
        double dC_dot = 0.0;
        const double dOper[4] = {1.0, -1.0, -1.0, 1.0};
        double dEPS = 1e-12;
        for (int i = 0; i < iNdef; i++)
        {
            pdC[i] = 0.0;
            for (int j = 0; j < iN_oneway; j++)
            {
                if (arriIpt2ow[j] > iNdim_ow)
                    return true;
                pdC[i] = pdC[i] + pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + i] * dOper[j];
            }
            dC_dot = dC_dot + pdC[i] * pdC[i];
        }
        // bool isLdepend = false;
        if (abs(dC_dot - iN_oneway) <= dEPS)
            return true;

        for (int j = 0; j < iNamb; j++)
            pdE[j * (iNdim_for_check + 1) + iNdef] = 0.0;
        for (int j = 0; j < iN_oneway; j++)
            pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + iNdef] = dOper[j];

        if (dC_dot > dEPS)
        {
            for (int i = 0; i < iNdef; i++)
            {
                if (abs(pdC[i]) < dEPS)
                    continue;
                for (int j = 0; j < iNamb; j++)
                    pdE[j * (iNdim_for_check + 1) + iNdef] = pdE[j * (iNdim_for_check + 1) + iNdef] - pdC[i] * pdE[j * (iNdim_for_check + 1) + i];
            }

            dC_dot = 0.0;
            for (int j = 0; j < iNamb; j++)
                dC_dot = dC_dot + pdE[j * (iNdim_for_check + 1) + iNdef] * pdE[j * (iNdim_for_check + 1) + iNdef];
            dC_dot = sqrt(dC_dot);
            for (int j = 0; j < iNamb; j++)
                pdE[j * (iNdim_for_check + 1) + iNdef] = pdE[j * (iNdim_for_check + 1) + iNdef] / dC_dot;
        }
        else
        {
            dC_dot = sqrt(iN_oneway * 1.0);
            for (int j = 0; j < iN_oneway; j++)
                pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + iNdef] = pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + iNdef] / dC_dot;
        }

        pdC[iNdef] = dC_dot;
        iNdef = iNdef + 1;
        return false;
    }

    bool gnss_all_amb::_check_amb_depend(bool isFirst, int iNamb, int *iNdef, int iN_oneway, std::vector<int> arriIpt2ow, int iMaxamb_ow, int iMaxamb_for_check)
    {
        const std::string strCprogName = "check_amb_depend";
        const double dEPS = 1e-12;
        static int iNdim_ow = 0;
        static int iNdim_for_check = 0;
        int i, j;
        //static double *pdE = 0;
        //static double *pdC = 0;
        const double dOper[4] = {1.0, -1.0, -1.0, 1.0};
        double dC_dot;
        bool isLdepend;

        try
        {
            if (isFirst)
            {
                if (_pdE)
                {
                    delete _pdE;
                    _pdE = nullptr;
                }
                if (_pdC)
                {
                    delete _pdC;
                    _pdC = nullptr;
                }
                //lvhb test in 202010
                _pdE = new double[iMaxamb_ow * (iMaxamb_for_check + 100)];
                _pdC = new double[iMaxamb_for_check + 100];
                if (iMaxamb_ow == 0 || iMaxamb_for_check == 0)
                {
                    std::string err = "***ERROR: memory allocatation for e&c ";
                    throw err;
                }
                iNdim_ow = iMaxamb_ow;
                iNdim_for_check = iMaxamb_for_check;
                return false;
            }

            if (*iNdef >= iNdim_for_check)
            {
                std::string err2 = "***ERROR: independent ones already reaches the allocated memory ";
                throw err2;
            }

            dC_dot = 0.0;
            for (i = 0; i < *iNdef; i++)
            {
                _pdC[i] = 0.0;
                for (j = 0; j < iN_oneway; j++)
                {
                    if (arriIpt2ow[j] > iNdim_ow)
                    {
                        std::string err3 = "***ERROR: base element beyond the allocated memory ";
                        throw err3;
                    }
                    _pdC[i] = _pdC[i] + _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + i] * dOper[j];
                }
                dC_dot = dC_dot + _pdC[i] * _pdC[i];
            }

            isLdepend = false;
            if (abs(dC_dot - iN_oneway) <= dEPS)
            {
                isLdepend = true;
                return isLdepend;
            }

            for (j = 0; j < iNamb; j++)
            {
                _pdE[j * (iNdim_for_check + 1) + *iNdef] = 0.0;
            }
            for (j = 0; j < iN_oneway; j++)
            {
                _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + *iNdef] = dOper[j];
            }

            if (dC_dot > dEPS)
            {
                for (i = 0; i < *iNdef; i++)
                {
                    if (abs(_pdC[i]) < dEPS)
                    {
                        continue;
                    }
                    for (j = 0; j < iNamb; j++)
                    {
                        _pdE[j * (iNdim_for_check + 1) + *iNdef] = _pdE[j * (iNdim_for_check + 1) + *iNdef] - _pdC[i] * _pdE[j * (iNdim_for_check + 1) + i];
                    }
                }

                dC_dot = 0.0;
                for (j = 0; j < iNamb; j++)
                {
                    dC_dot = dC_dot + _pdE[j * (iNdim_for_check + 1) + *iNdef] * _pdE[j * (iNdim_for_check + 1) + *iNdef];
                }
                dC_dot = sqrt(dC_dot);
                for (j = 0; j < iNamb; j++)
                {
                    _pdE[j * (iNdim_for_check + 1) + *iNdef] = _pdE[j * (iNdim_for_check + 1) + *iNdef] / dC_dot;
                }
            }
            else
            {
                dC_dot = sqrt(iN_oneway * 1.0);
                for (j = 0; j < iN_oneway; j++)
                {
                    _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + *iNdef] = _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + *iNdef] / dC_dot;
                }
            }

            _pdC[*iNdef] = dC_dot;
            *iNdef = *iNdef + 1;
            return false;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ERROR[gnss_ambiguity::_check_amb_depend] : throw exception");
            return false;
        }
    }

    bool gnss_all_amb::_check_amb_depend_SVD(std::vector<int> idx, int num_ow, Matrix &vector_space)
    {
        if (idx.size() < 4 || idx[0] < 0 || idx[1] < 0 || idx[2] < 0 || idx[3] < 0)
            return true;
        if (vector_space.isZero())
        {
            vector_space.resize(1, num_ow);
            vector_space.setConstant(0.0);
            vector_space(1, idx[0]) = 1;
            vector_space(1, idx[1]) = -1;
            vector_space(1, idx[2]) = -1;
            vector_space(1, idx[3]) = 1;
            return false;
        }

        Eigen::RowVectorXd new_vector(num_ow);
        new_vector.setZero();
        new_vector(idx[0]) = 1;
        new_vector(idx[1]) = -1;
        new_vector(idx[2]) = -1;
        new_vector(idx[3]) = 1;

        // 数学原理：奇异矩阵经过SVD分解后，D矩阵对角线的非零元素个数等于A矩阵的矩阵秩
        Matrix A = Vstack(vector_space, new_vector);
        Diag D;
        if (A.rows() < A.cols())
        {
            SVD(A.transpose(), D.matrixW());
        }
        else
        {
            SVD(A, D.matrixW());
        }

        return (fabs(D(D.rows(), D.cols())) > 0.01);
    }

    int gnss_all_amb::_get_oneway_pos(const std::string &site, const std::string &sat, const base_time &beg, const base_time &end)
    {
        if (_all_ow_ambs.find(site) == _all_ow_ambs.end() || _all_ow_ambs[site].find(sat) == _all_ow_ambs[site].end() ||
            _all_ow_ambs[site][sat].empty())
            return -1;

        for (size_t i = 0; i < _all_ow_ambs[site][sat].size(); ++i)
        {
            if (_all_ow_ambs[site][sat][i]->check_ow_time(beg, end, _min_common_time))
                return i;
        }
        return -1;
    }
}
