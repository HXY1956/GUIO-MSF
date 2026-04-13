#include "hwa_set_gen.h"
#include "hwa_set_gbase.h"
#include "hwa_gnss_amb_ambiguity.h"
#include "hwa_gnss_amb_common.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_base_par.h"
#include "hwa_base_allpar.h"

using namespace std;

namespace hwa_base {

    map<string, int> base_allpar::freq_sats_num(const int& freq)
    {

        set<string> prns1, prns2, prnsif, prns13if, prns14if, prns15if;
        map<string, int> Nsats;
        vector<base_par>::const_iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
        {
            if (iter->parType == par_type::AMB_L1)
            {
                prns1.insert(iter->prn);
            }
            else if (iter->parType == par_type::AMB_L2)
            {
                prns2.insert(iter->prn);
            }
            else if (iter->parType == par_type::AMB_IF)
            {
                prnsif.insert(iter->prn);
            }
            else if (iter->parType == par_type::AMB13_IF)
            {
                prns13if.insert(iter->prn);
            }
            else if (iter->parType == par_type::AMB14_IF)
            {
                prns14if.insert(iter->prn);
            }
            else if (iter->parType == par_type::AMB15_IF)
            {
                prns15if.insert(iter->prn);
            }
        }

        if (prnsif.size() != 0 || prns13if.size() != 0)
        {
            set<string> tmp;
            set_intersection(prnsif.begin(), prnsif.end(), prns13if.begin(), prns13if.end(), inserter(tmp, tmp.begin()));
            Nsats["Triple"] = tmp.size();
            Nsats["Double"] = prnsif.size() + prns13if.size() - 2 * tmp.size();
        }
        else if (prns1.size() != 0 || prns2.size() != 0)
        {
            set<string> tmp;
            set_intersection(prns1.begin(), prns1.end(), prns2.begin(), prns2.end(), inserter(tmp, tmp.begin()));
            Nsats["Double"] = tmp.size();
            Nsats["Single"] = prns1.size() + prns2.size() - 2 * tmp.size();
        }

        return Nsats;
    }

    set<string> base_allpar::amb_prns()
    {

        set<string> prns;
        vector<base_par>::const_iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
        {
            if (iter->str_type().find("AMB") != string::npos)
            {
                prns.insert(iter->prn);
            }
        }
        return prns;
    }

    set<string> base_allpar::amb_prns(const par_type& type)
    {

        set<string> prns;
        vector<base_par>::const_iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
        {
            // changed by wangbo
            if (iter->parType == type)
            {
                prns.insert(iter->prn);
            }
        }
        return prns;
    }

    const base_par& base_allpar::getAmbParam(const int& idx) const
    {
        return _vParam[idx];
    }

    int base_allpar::getAmbParam(const string& site, const string& prn,
        const par_type& type, const base_time& beg, const base_time& end) const
    {

        if (this->_index_par.count(base_par_head(type, site, prn)) == 0)
        {
            return -1;
        }
        else
        {
            auto all = this->_index_par.find(base_par_head(type, site, prn))->second;
            base_time_arc dst_timearc(beg, end);
            //auto iter = lower_bound(all.begin(),all.end(), dst_timearc);
            for (auto iter = all.begin(); iter != all.end(); ++iter)
            {
                if (dst_timearc.inside(iter->first))
                {
                    auto ans = lower_bound(_point_par.begin(), _point_par.end(), iter->second);
                    assert(ans != _point_par.end() && *ans == iter->second);
                    return ans - _point_par.begin();
                }
            }
            return -1;
        }
        return -1;
    }

    std::vector<int> base_allpar::delAmb()
    {
        vector<int> ind;
        vector<base_par>::iterator iter;
        iter = _vParam.begin();
        while (iter != _vParam.end())
        {
            if (iter->parType == par_type::AMB_IF || iter->parType == par_type::AMB_L1 || iter->parType == par_type::AMB_L2 || iter->parType == par_type::AMB_L3 || iter->parType == par_type::AMB_L4 || iter->parType == par_type::AMB_L5 || iter->parType == par_type::AMB_WL)
            {
                ind.push_back(iter->index);
                int i = iter - _vParam.begin();

                auto& all = this->_index_par[_vParam[i].get_head()];
                for (auto iter = all.begin(); iter != all.end(); ++iter)
                {
                    if (iter->second == _point_par[i])
                    {
                        all.erase(iter);
                        break;
                    }
                }
                if (this->_index_par[_vParam[i].get_head()].size() == 0)
                {
                    this->_index_par.erase(_vParam[i].get_head());
                }
                _point_par.erase(_point_par.begin() + i);

                iter = _vParam.erase(iter);
            }
            else
                ++iter;
        }
        return ind;
    }
}

namespace hwa_gnss
{
    gnss_ambiguity::gnss_ambiguity()
    {
        _pdE = nullptr;
        _pdC = nullptr;
        _ratiofile = nullptr;
        _bootfile = nullptr;
        _ewl_Upd_time = base_time(EWL_IDENTIFY);
        _ewl24_Upd_time = base_time(EWL24_IDENTIFY);
        _ewl25_Upd_time = base_time(EWL25_IDENTIFY);
        _wl_Upd_time = base_time(WL_IDENTIFY);
    }

    gnss_ambiguity::gnss_ambiguity(std::string site, set_base *gset)
        : _part_fix(false),
          _is_first(true),
          _is_first_wl(true),
          _is_first_nl(true),
          _is_first_ewl(true),
          _is_first_ewl24(true),
          _is_first_ewl25(true),
          //_apply_irc(false),
          _amb_fixed(false),
          _simulation(false), //xjhan
          _obstype(OBSCOMBIN::IONO_FREE),
          _upd_mode(UPD_MODE::UPD),
          _fix_mode(FIX_MODE::SEARCH)
    {
        _site = site;
        _gset = gset;
        _beg = dynamic_cast<set_gen *>(_gset)->beg();
        _end = dynamic_cast<set_gen *>(_gset)->end();
        _sys = dynamic_cast<set_gen *>(_gset)->sys();
        _sat_rm = dynamic_cast<set_gen *>(_gset)->sat_rm();
        _interval = dynamic_cast<set_gen *>(_gset)->sampling();

        _band_index[GPS] = dynamic_cast<set_gnss *>(_gset)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(_gset)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(_gset)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(_gset)->band_index(BDS);
        _band_index[QZS] = dynamic_cast<set_gnss *>(_gset)->band_index(QZS);

        _fix_mode = dynamic_cast<set_amb *>(_gset)->fix_mode();
        _upd_mode = dynamic_cast<set_amb *>(_gset)->upd_mode();

        _part_fix = dynamic_cast<set_amb *>(_gset)->part_ambfix();
        _part_fix_num = dynamic_cast<set_amb *>(_gset)->part_ambfix_num();
        _min_common_time = dynamic_cast<set_amb *>(_gset)->min_common_time();
        _map_EWL_decision = dynamic_cast<set_amb *>(_gset)->get_amb_decision("EWL");
        _map_WL_decision = dynamic_cast<set_amb *>(_gset)->get_amb_decision("WL");
        _map_NL_decision = dynamic_cast<set_amb *>(_gset)->get_amb_decision("NL");
        _ratio = dynamic_cast<set_amb *>(_gset)->lambda_ratio();
        _boot = dynamic_cast<set_amb *>(_gset)->bootstrapping();
        _frequency = dynamic_cast<set_gproc *>(_gset)->frequency();
        _FloatFixSep = dynamic_cast<set_amb *>(_gset)->FloatFixSep();
        _FixFixSep = dynamic_cast<set_amb *>(_gset)->FixFixSep();
        _full_fix_num = dynamic_cast<set_amb *>(_gset)->full_fix_num();
        dynamic_cast<set_amb *>(_gset)->refixsettings(_ctrl_last_fixepo_gap, _ctrl_min_fixed_num);

        //init ratio and boot file
        if (_fix_mode == FIX_MODE::SEARCH)
            _initRatiofile(); //xjhan
        if (_boot > 0.0)
            _initBootfile();

        _realtime = dynamic_cast<set_gproc *>(_gset)->realtime();
        _ewl_Upd_time = base_time(EWL_IDENTIFY);
        _ewl24_Upd_time = base_time(EWL24_IDENTIFY);
        _ewl25_Upd_time = base_time(EWL25_IDENTIFY);
        _wl_Upd_time = base_time(WL_IDENTIFY);
    }

    gnss_ambiguity::~gnss_ambiguity()
    {
        if (_ratiofile)
        {
            if (_ratiofile->is_open())
                _ratiofile->close();
            delete _ratiofile;
            _ratiofile = NULL;
        }
        if (_bootfile)
        {
            if (_bootfile->is_open())
                _bootfile->close();
            delete _bootfile;
            _bootfile = nullptr;
        }
        // glfeng
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
    }

    bool gnss_ambiguity::amb_fixed()
    {
        return _amb_fixed;
    }

    int gnss_ambiguity::processBatch(const base_time& t, gnss_proc_flt* gflt, gnss_data_cycleslip* cycle_slip, std::string mode)
    {
        // 实时模式或处于UPD解算模式时，更新时间戳
        if (_realtime == true || (_gupd && _gupd->wl_epo_mode())) {
            _ewl_Upd_time = t;
            _wl_Upd_time = t;
            _ewl24_Upd_time = t;
            _ewl25_Upd_time = t;
        }

        // 清除当前模式下过期的模糊度固定次数计数
        for (auto it : _fix_epo_num[mode]) {
            if (abs(t.diff(_last_fix_time[mode][it.first])) > _interval + 0.01)
                it.second = 0;
        }

        // 初始化状态变量
        _sats_index.clear();
        _lock_epo_num.clear();
        std::vector<int> fixed_amb;
        _amb_freqs.clear();
        if (_fix_mode == FIX_MODE::HOLD)
            _addFixConstraint_before(gflt);  // 在HOLD模式下添加之前固定的约束条件

        _DD.clear();
        _total_amb_num = _fixed_amb_num = 0;
        _outRatio = 0.0;

        // 初始化模糊度共性对象
        gnss_amb_cmn amb_cmn(t, gflt);
        amb_cmn.set_mode(mode);
        _crt_time = amb_cmn.now();
        int parnum = amb_cmn.param().parNumber();

        // 根据模式设置是否为第一次处理该模糊度类型
        int koder = 0;
        if (mode == "EWL") { _is_first = _is_first_ewl; koder = 3; }
        else if (mode == "WL") { _is_first = _is_first_wl; koder = 2; }
        else if (mode == "NL") { _is_first = _is_first_nl; koder = 1; }

        // 如果是首次处理，检查是否存在模糊度依赖关系
        if (_is_first) {
            int maxamb = cycle_slip ? cycle_slip->get_active_amb(_site) + parnum
                : _max_active_amb_one_epo + parnum;
            _is_first = _checkAmbDepend(_is_first, 0, 0, 0, 0, maxamb, maxamb);
            _is_first = false;
    }

        // 保存首次标志位
        if (mode == "EWL") _is_first_ewl = _is_first;
        else if (mode == "WL") _is_first_wl = _is_first;
        else if (mode == "NL") _is_first_nl = _is_first;

        // 定义双差模糊度
        int namb = _defineDDAmb(&amb_cmn);
        if (namb < 0) return -1;
        _total_amb_num = namb;

        // 计算不同模式下的宽巷模糊度
        if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_MIX) {
            if (mode != "NL" && !_calDDAmbWLALL(&amb_cmn, mode)) return -1;
        }
        else if (_obstype == OBSCOMBIN::IONO_FREE) {
            if (!_calDDAmbWL()) return -1;
        }
        else if (_obstype == OBSCOMBIN::WL_COMBIN) {
            if (!_calDDAmbWLWL()) return -1;
        }

        // 查找参考卫星
        if (!_sat_refs.empty() && !_findRefSD()) return -1;

        // 应用UPD改正并固定模糊度
        if (mode == "NL") {
            if (!_applyUpd(_crt_time)) return -1;
            if (_obstype == OBSCOMBIN::IONO_FREE) {
                if (!_fixAmbIF()) return -1;
            }
            else {
                if (!_fixAmbUDUC()) return -1;
            }
        }
        else {
            if (!_applyWLUpd(_crt_time, mode)) return -1;
            if (!_fixAmbWL(mode)) return -1;
        }

        _DD_save = _DD;

        // 筛选可用模糊度
        int ndef = _selectAmb(koder, namb);
        if (ndef < 0) return -1;

        // 若为宽巷类，要求双频
        if (mode == "WL" || mode == "EWL") {
            for (auto itdd = _DD.begin(); itdd != _DD.end();) {
                if (itdd->ddSats.size() < 4) {
                    itdd = _DD.erase(itdd);
                    continue;
                }
                itdd++;
            }
        }

        // 若开启固定功能且为NL模式，执行LAMBDA搜索
        if (_fix_mode != FIX_MODE::NO && mode == "NL") {
            if (!_ambSolve(&amb_cmn, fixed_amb, mode)) return -1;
        }

        // 判断是否满足固定条件
        if (_last_fix_time[mode].find("sum") != _last_fix_time[mode].end()) {
            if (t.diff(_last_fix_time[mode]["sum"]) >= _ctrl_last_fixepo_gap * _interval && fixed_amb.size() < _ctrl_min_fixed_num)
                return -1;
        }

        // 更新当前模糊度解
        for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++) {
            if (mode == "NL") {
                it_dd->iwl = _IWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site];
                it_dd->iewl = _IEWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site];
            }
            if (mode == "WL")
                _IWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site] = it_dd->iwl;
            if (mode == "EWL")
                _IEWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site] = it_dd->iewl;
        }

        gnss_proc_flt fltpreAMB(*gflt);
        if (mode == "NL") {
            if (!_addFixConstraint(gflt)) return -1;
        }
        else {
            if (!_addFixConstraintWL(gflt, mode)) return -1;
        }

        // 最终判断是否固定成功，并更新历史信息
        if (amb_cmn.get_ratio() > _ratio && amb_cmn.get_boot() > _boot) {
            _amb_fixed = true;
            _fixed_amb_num = fixed_amb.size();

            if ((_FloatFixSep > 0.0 && _float_fix_check(gflt, fltpreAMB, t) < 0) ||
                (_FixFixSep > 0.0 && _fix_fix_check(gflt, fltpreAMB, t, mode) < 0)) {
                _amb_fixed = false;
                _fixed_amb_num = 0;
            }
            else {
                _DD_previous[mode] = _DD;
                for (auto it_dd : _DD) {
                    _fix_epo_num[mode][get<0>(it_dd.ddSats[0])]++;
                    _last_fix_time[mode][get<0>(it_dd.ddSats[0])] = t;
                    _fix_epo_num[mode][get<0>(it_dd.ddSats[1])]++;
                    _last_fix_time[mode][get<0>(it_dd.ddSats[1])] = t;
        }
                _last_fix_time[mode]["sum"] = t;
        }
    }
        else {
            _amb_fixed = false;
            _fixed_amb_num = 0;
        }

        _outRatio = amb_cmn.get_ratio();
        auto tmp_param = gflt->param();
        auto tmp_dx = gflt->dx();

        updateFixParam(tmp_param, tmp_dx);

        // 如果为HOLD模式，保存当前固定信息以便下次使用
        if (_fix_mode == FIX_MODE::HOLD) {
            for (auto itdd : _DD) {
                for (auto itdd1 = _DD_save.begin(); itdd1 != _DD_save.end();) {
                    if ((get<0>(itdd.ddSats[0]) == get<0>(itdd1->ddSats[0]) && get<0>(itdd.ddSats[1]) == get<0>(itdd1->ddSats[1]) &&
                        itdd.beg_epo == itdd1->beg_epo && itdd.inl == itdd1->inl) &&
                        amb_cmn.get_ratio() > 10) {
                        _DD_save.erase(itdd1);
                        itdd.fix_epoch = itdd1->fix_epoch + 1;
                        continue;
                    }
                    else {
                        itdd1++;
                    }
                }
                _DD_save.push_back(itdd);
            }
        }

        return 1;
        }


    int gnss_ambiguity::processBatch(const base_time &t, gnss_proc_lsqbase *glsq, gnss_data_cycleslip *cycle_slip, std::string mode)
    {
        if (_realtime == true || (_gupd && _gupd->wl_epo_mode()))
        {
            _ewl_Upd_time = t;
            _ewl24_Upd_time = t;
            _ewl25_Upd_time = t;
            _wl_Upd_time = t;
            if (_realtime == true)
            {
                _lock_epo_num.clear();
                for (auto it : _fix_epo_num[mode])
                {
                    if (abs(t.diff(_last_fix_time[mode][it.first])) > _interval + 0.01)
                        it.second = 0;
                }
            }
        }

        _DD.clear();
        _amb_freqs.clear();
        gnss_amb_cmn amb_cmn(t, glsq);
        
        amb_cmn.set_mode(mode);
        _crt_time = amb_cmn.now();
        std::vector<int> fixed_amb;
        int parnum = amb_cmn.param().parNumber();

        int koder = 0;
        if (mode == "EWL")
        {
            _is_first = _is_first_ewl;
            koder = 3;
        }
        else if (mode == "EWL24")
        {
            _is_first = _is_first_ewl24;
            koder = 4;
        }
        else if (mode == "EWL25")
        {
            _is_first = _is_first_ewl25;
            koder = 5;
        }
        else if (mode == "WL")
        {
            _is_first = _is_first_wl;
            koder = 2;
        }
        else if (mode == "NL")
        {
            _is_first = _is_first_nl;
            koder = 1;
        }
        // check if a new ambiguity is dependant of the already selected
        if (_is_first) //glfeng  glsq->get_active_amb(_site)
        {
            int maxamb;
            if (cycle_slip)
                maxamb = cycle_slip->get_active_amb(_site) + parnum;
            else
                maxamb = _max_active_amb_one_epo + parnum;
            _is_first = _checkAmbDepend(_is_first, 0, 0, 0, 0, maxamb, maxamb);
            _is_first = false;
        }

        if (mode == "EWL")
            _is_first_ewl = _is_first;
        else if (mode == "EWL24")
            _is_first_ewl24 = _is_first;
        else if (mode == "EWL25")
            _is_first_ewl25 = _is_first;
        else if (mode == "WL")
            _is_first_wl = _is_first;
        else if (mode == "NL")
            _is_first_nl = _is_first;

        // define double difference ambiguities over one baseline (PPP sd amb.)
        int namb = _defineDDAmb(&amb_cmn);
        if (namb < 0)
            return -1;
        _total_amb_num = namb;
        // calulate widelane upd for iono_free
        if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_MIX)
        {
            if (!_calDDAmbWLALL(&amb_cmn, mode))
                return -1;
        }
        else if (_obstype == OBSCOMBIN::IONO_FREE)
        {
            if (!_calDDAmbWL())
                return -1;
        }
        else if (_obstype == OBSCOMBIN::WL_COMBIN)
        {
            if (!_calDDAmbWLWL())
                return -1;
        }

        // find DD that contains reference satellite
        if (!_sat_refs.empty() && !_findRefSD())
            return -1;

        // apply UPD correction
        // fix widelane and narrowlane ambiguities
        if (mode == "NL")
        {
            if (!_applyUpd(_crt_time))
                return -1;

            if (_obstype == OBSCOMBIN::IONO_FREE)
            {
                if (!_fixAmbIF())
                    return -1;
            }
            else if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_SINGLE || _obstype == OBSCOMBIN::RAW_MIX)
            {
                if (!_fixAmbUDUC())
                    return -1;
            }
            else if (_obstype == OBSCOMBIN::WL_COMBIN)
            {
                if (!_fixAmbWL())
                    return -1;
            }
        }
        else
        {
            if (!_applyWLUpd(_crt_time, mode))
                return -1;
            if (!_fixAmbWL(mode))
                return -1;
        }

#ifdef DEBUG_pppRTK
        std::cout << "Processing Epoch  " << _crt_time.str_ymdhms() << std::endl;
        std::cout << mode + "  Candidate DD " << _DD.size() << std::endl;
        for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
        {
            std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1]);
            std::cout << std::setw(17) << std::setprecision(4) << it_dd->rwl << std::setw(17) << std::setprecision(4) << it_dd->iwl
                 << std::setw(12) << std::setprecision(4) << it_dd->srwl << std::setw(7) << boolalpha << it_dd->isWlFixed;
            std::cout << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                 << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
        }
#endif
        // select usable amb.
        int ndef = _selectAmb(koder, namb);
        //LX added for PPP-RTK
        if (ndef < 4)
            return -1; //refer LX code

#ifdef DEBUG
        std::cout << "Processing Epoch  " << _crt_time.str_ymdhms() << std::endl;
        std::cout << mode << "  ALL DD  " << _DD.size() << std::endl;
        if (mode == "WL" || mode == "NL")
        {
            for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
            {
                std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1])
                     << std::setw(17) << std::setprecision(4) << it_dd->rwl << std::setw(17) << std::setprecision(4) << it_dd->iwl
                     << std::setw(12) << std::setprecision(4) << it_dd->srwl << std::setw(7) << boolalpha << it_dd->isWlFixed
                     << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                     << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
            }
        }
        else if (mode == "EWL")
        {
            for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
            {
                std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1]);
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rewl << std::setw(17) << std::setprecision(4) << it_dd->iewl
                     << std::setw(12) << std::setprecision(4) << it_dd->srewl << std::setw(7) << boolalpha << it_dd->isEwlFixed;
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                     << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
            }
        }
        else if (mode == "EWL24")
        {
            for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
            {
                std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1]);
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rewl24 << std::setw(17) << std::setprecision(4) << it_dd->iewl24
                     << std::setw(12) << std::setprecision(4) << it_dd->srewl24 << std::setw(7) << boolalpha << it_dd->isEwl24Fixed;
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                     << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
            }
        }
        else if (mode == "EWL25")
        {
            for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
            {
                std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1]);
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rewl25 << std::setw(17) << std::setprecision(4) << it_dd->iewl25
                     << std::setw(12) << std::setprecision(4) << it_dd->srewl25 << std::setw(7) << boolalpha << it_dd->isEwl25Fixed;
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                     << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
            }
        }
#endif
        // lambda search
        if (_fix_mode == FIX_MODE::SEARCH && mode == "NL")
        {
            if (!_ambSolve(&amb_cmn, fixed_amb, mode))
                return -1;
        }
        // int std::fixed = fixed_amb.size();
        //    if (std::fixed < 5)return -1;
        //    if (std::fixed < _total_amb_num / 2)return -1;
        if (mode == "NL")
            std::cout << "FINAL DD Ambiguity " << _DD.size() << std::endl;
        for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
        {
            if (mode == "NL")
            {
                it_dd->iwl = _IWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site];
                it_dd->iewl = _IEWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site];
                it_dd->iewl24 = _IEWL24[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])];
                it_dd->iewl25 = _IEWL25[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])];
#ifdef DEBUG
                std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1]);
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rwl << std::setw(17) << std::setprecision(4) << it_dd->iwl
                     << std::setw(12) << std::setprecision(4) << it_dd->srwl << std::setw(7) << boolalpha << it_dd->isWlFixed;
                std::cout << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                     << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
#endif
            }
            if (mode == "WL")
                _IWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site] = it_dd->iwl;
            if (mode == "EWL")
                _IEWL[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])][it_dd->site] = it_dd->iewl;
            if (mode == "EWL24")
                _IEWL24[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])] = it_dd->iewl24;
            if (mode == "EWL25")
                _IEWL25[get<0>(it_dd->ddSats[0])][get<0>(it_dd->ddSats[1])] = it_dd->iewl25;
        }
#ifdef DEBUG
        // print information to AR spdlog file
        std::cout << "Processing Epoch  " << _crt_time.str_ymdhms() << std::endl;
        std::cout << "ALL DD  " << _DD.size() << std::endl;
        for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
        {
            std::cout << std::setw(5) << get<0>(it_dd->ddSats[0]) << std::setw(5) << get<0>(it_dd->ddSats[1]);
            std::cout << std::setw(17) << std::setprecision(4) << it_dd->rwl << std::setw(17) << std::setprecision(4) << it_dd->iwl
                 << std::setw(12) << std::setprecision(4) << it_dd->srwl << std::setw(7) << boolalpha << it_dd->isWlFixed;
            std::cout << std::setw(17) << std::setprecision(4) << it_dd->rnl << std::setw(17) << std::setprecision(4) << (double)(it_dd->inl)
                 << std::setw(12) << std::setprecision(4) << it_dd->srnl << std::setw(7) << boolalpha << it_dd->isNlFixed << std::endl;
        }
#endif

#ifdef DEBUG
        std::cout << "Before add Constraint : Covariance Matrix " << std::endl;
        for (int i = 1; i <= parnum; i++)
        {
            for (int j = 1; j <= parnum; j++)
            {
                std::cout << std::setw(16) << std::setprecision(12) << glsq->Qx()(i, j);
            }
            std::cout << std::endl;
        }
#endif

        // add constraint
        if (mode == "NL")
        {
            if (!_addFixConstraint(glsq))
                return -1;
        }
        else
        {
            if (!_addFixConstraintWL(glsq, mode))
                return -1;
            gnss_amb_cmn amb_cmn(_crt_time, glsq);
        }
        // amb state should be determined after addFixConstraint.(zhshen)
        if (amb_cmn.get_ratio() > _ratio && amb_cmn.get_boot() > _boot)
        {
            _amb_fixed = true;
            _fixed_amb_num = fixed_amb.size();
            if (_realtime == true)
            {
                for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
                {
                    _fix_epo_num[mode][get<0>(it_dd->ddSats[0])]++;
                    _last_fix_time[mode][get<0>(it_dd->ddSats[0])] = t;
                    _fix_epo_num[mode][get<0>(it_dd->ddSats[1])]++;
                    _last_fix_time[mode][get<0>(it_dd->ddSats[1])] = t;
                }
            }
        }
        else
        {
            _amb_fixed = false;
            _fixed_amb_num = 0;
        }
        auto tmp_dx = glsq->dx();
        auto tmp_stdx = glsq->stdx();
        updateFixParam(glsq->_x_solve, tmp_dx, &tmp_stdx); //lvhb 202007

#ifdef DEBUG
        std::cout << " After " << mode << " Ambiguity Fix: " << _crt_time.str_ymdhms() << std::endl;
        for (int i = 0; i < glsq->_x_solve.parNumber(); i++)
        {
            std::cout << std::setw(20) << "EPO  " << std::setw(20) << glsq->_x_solve[i].str_type() + "  " << std::setw(20) << std::setprecision(5) << glsq->_x_solve[i].value() << std::setw(15) << std::setprecision(5) << glsq->dx()(i + 1) << std::setw(20) << std::setprecision(5) << glsq->_x_solve[i].value() + glsq->dx()(i + 1) << std::setw(12) << std::setprecision(5) << glsq->stdx(i + 1) << std::endl;
        }
#endif

#ifdef DEBUG
        std::cout << "After add Constraint : Covariance Matrix " << std::endl;
        for (int i = 1; i <= parnum; i++)
        {
            for (int j = 1; j <= parnum; j++)
            {
                std::cout << std::setw(16) << std::setprecision(12) << glsq->Qx()(i, j);
            }
            std::cout << std::endl;
        }
#endif
        return 1;
    }

    base_allpar &gnss_ambiguity::getFinalParams()
    {
        // TODO: ???????? return ???
        return _param;
    }

    base_allpar &gnss_ambiguity::updateUCAmb(std::set<std::string> *satref)
    {
        // select the ref _sat1
        std::map<std::string, std::map<FREQ_SEQ, double>> singleND;
        if (satref)
            *satref = _sat_refs;
        for (auto itref = _sat_refs.begin(); itref != _sat_refs.end(); itref++)
        {
            _NDRecovery(*itref, _param, singleND);
        }
        if (singleND.empty())
        {
            _param.delAllParam();
            return _param;
        }
        // delete all amb, lsq_del no use
        std::vector<int> lsq_del = _param.delAmb();
        int ipar = _param.parNumber();

        // add new amb for aug out
        int minfreqnum = 2;
        if (_obstype == OBSCOMBIN::RAW_MIX || _obstype == OBSCOMBIN::RAW_SINGLE)
            minfreqnum = 1;
        for (const auto &itsat : singleND)
        {
            if (itsat.second.size() < minfreqnum)
                continue;
            for (const auto &itfre : itsat.second)
            {
                base_par new_amb;
                new_amb.value(itfre.second);
                new_amb.prn = itsat.first;
                new_amb.site = _site;
                if (itfre.first == FREQ_SEQ::FREQ_1)
                {
                    new_amb.parType = par_type::AMB_L1;
                }
                else if (itfre.first == FREQ_SEQ::FREQ_2)
                {
                    new_amb.parType = par_type::AMB_L2;
                }
                else if (itfre.first == FREQ_SEQ::FREQ_3)
                {
                    new_amb.parType = par_type::AMB_L3;
                }
                else if (itfre.first == FREQ_SEQ::FREQ_4)
                {
                    new_amb.parType = par_type::AMB_L4;
                }
                else if (itfre.first == FREQ_SEQ::FREQ_5)
                {
                    new_amb.parType = par_type::AMB_L5;
                }
                else
                    continue;
                new_amb.index = ++ipar;
                _param.addParam(new_amb);
            }
        }
        _param.reIndex();

        return _param;
    }
    void gnss_ambiguity::setSatRef(std::set<std::string> &satRef)
    {
        _sat_refs = satRef;

        _EWL_flag.clear();
        _IEWL.clear();
        _EWL24_flag.clear();
        _IEWL24.clear();
        _WL_flag.clear();
        _IWL.clear();
    }

    void gnss_ambiguity::updateFixParam(base_allpar &param, Vector &dx, Vector *stdx)
    {
        _param.delAllParam();
        _param = param;

        for (unsigned int i = 0; i < param.parNumber(); i++)
        {
            _param[i].value(param[i].value() + dx(i + 1));
            if (stdx)
                _param[i].apriori((*stdx)(i + 1));
        }
    }

    double gnss_ambiguity::lambdaSolve(double &ratio, const Matrix &anor, const Vector &fltpar, Vector &ibias, bool parlamb)
    //double gnss_ambiguity::lambdaSolve(double & ratio, const Matrix & anor, const std::vector<double>& fltpar, std::vector<int>& ibias, bool parlamb)
    {
        double boot_temp = 0.0; //by qzy
        _ratio = ratio;
        if (parlamb)
            _part_fix = true;
        else
            _part_fix = false;

        int namb = fltpar.rows();
        std::vector<double> fltpar_;
        std::vector<int> ibias_;

        for (int i = 1; i <= namb; i++)
        {
            fltpar_.push_back(fltpar(i));
        }
        double ratio_post = _lambdaSearch(anor, fltpar_, ibias_, &boot_temp);

        if (ratio_post > 0)
        {
            if (ibias_.size() == namb)
            {
                for (int i = 1; i <= namb; i++)
                {
                    ibias(i) = ibias_[i - 1];
                }
            }
        }
        else
            ratio_post = 0.0;

        return ratio_post;
    }

    void gnss_ambiguity::setWaveLength(std::map<std::string, int> &glofrq_num)
    {
        std::set<std::string> waveLenSats;

        // get system/satellite need to calculate wavelength
        for (auto strsys : _sys)
        {
            GSYS gsys = gnss_sys::str2gsys(strsys);
            switch (gsys)
            {
            case GPS:
                waveLenSats.insert("G");
                break;
            case GAL:
                waveLenSats.insert("E");
                break;
            case BDS:
                waveLenSats.insert("C");
                break;
            case GLO: // FDMA
                for (auto it_R : glofrq_num)
                {
                    waveLenSats.insert(it_R.first);
                }
                break;
            case QZS:
                waveLenSats.insert("J");
                break;
            case LEO:
                waveLenSats.insert("2");
                waveLenSats.insert("3");
                waveLenSats.insert("4");
                waveLenSats.insert("5");
                break;
            default:
                break;
            }
        }

        gnss_data_obs_manager *gnss = new gnss_data_obs_manager(_spdlog);
        // Loop waveLenSats
        for (auto iter : waveLenSats)
        {
            if (iter == "2" || iter == "3" || iter == "4" || iter == "5") //xjhan
                gnss->sat("G");
            else
                gnss->sat(iter);

            GSYS gsys = gnss->gsys();
            if (iter.substr(0, 1) == "R")
                gnss->channel(glofrq_num.at(iter));

            _sys_wavelen[iter]["L1"] = gnss->wavelength(_band_index[gsys][FREQ_1]);
            _sys_wavelen[iter]["L2"] = gnss->wavelength(_band_index[gsys][FREQ_2]);
            _sys_wavelen[iter]["WL"] = gnss->wavelength_WL(_band_index[gsys][FREQ_1], _band_index[gsys][FREQ_2]);
            _sys_wavelen[iter]["NL"] = gnss->wavelength_NL(_band_index[gsys][FREQ_1], _band_index[gsys][FREQ_2]);
            if (_frequency >= 3)
            {
                _sys_wavelen[iter]["L3"] = gnss->wavelength(_band_index[gsys][FREQ_3]);
                _sys_wavelen[iter]["EWL"] = gnss->wavelength_WL(_band_index[gsys][FREQ_2], _band_index[gsys][FREQ_3]);
            }
            if (_frequency >= 4)
            {
                _sys_wavelen[iter]["L4"] = gnss->wavelength(_band_index[gsys][FREQ_4]);
                _sys_wavelen[iter]["EWL24"] = gnss->wavelength_WL(_band_index[gsys][FREQ_2], _band_index[gsys][FREQ_4]);
            }
            if (_frequency >= 5)
            {
                _sys_wavelen[iter]["L5"] = gnss->wavelength(_band_index[gsys][FREQ_5]);
                _sys_wavelen[iter]["EWL25"] = gnss->wavelength_WL(_band_index[gsys][FREQ_2], _band_index[gsys][FREQ_5]);
            }
        }

        delete gnss;
        gnss = nullptr;

        return;
    }

    // _checkAmbDepend(): 检查模糊度线性相关性（依赖性），防止选择冗余的双差组合
 // 输入参数：
 // - isFirst: 是否为第一次初始化调用，用于分配内存
 // - iNamb: 当前模糊度数（E的维度）
 // - iNdef: 当前独立模糊度数目（C的维度）
 // - iN_oneway: 当前双差的组成个数（如两个卫星差为2）
 // - arriIpt2ow: 存储当前差分的索引（卫星编号对应的索引）
 // - iMaxamb_ow: 最大支持的模糊度数
 // - iMaxamb_for_check: 最大检查维度数（通常为 iMaxamb_ow）
    bool gnss_ambiguity::_checkAmbDepend(bool isFirst, int iNamb, int* iNdef, int iN_oneway, int* arriIpt2ow, int iMaxamb_ow, int iMaxamb_for_check)
    {
        const std::string strCprogName = "check_amb_depend";
        const double dEPS = 1e-12;
        static int iNdim_ow = 0;              // 模糊度维度
        static int iNdim_for_check = 0;       // 检查用维度
        int i, j;
        const double dOper[4] = { 1.0, -1.0, -1.0, 1.0 }; // 用于双差构造的系数
        double dC_dot;                        // 内积结果
        bool isLdepend;                       // 是否线性相关

        try {
            // 如果是首次调用，申请或重置内存
            if (isFirst) {
                if (_pdE) { delete _pdE; _pdE = nullptr; }
                if (_pdC) { delete _pdC; _pdC = nullptr; }

                // 分配新内存，留冗余空间防越界
                _pdE = new double[iMaxamb_ow * (iMaxamb_for_check + 100)];
                _pdC = new double[iMaxamb_for_check + 100];
                if (iMaxamb_ow == 0 || iMaxamb_for_check == 0)
                    throw std::string("***ERROR: memory allocatation for e&c ");

                iNdim_ow = iMaxamb_ow;
                iNdim_for_check = iMaxamb_for_check;
                return false; // 初始化调用，返回false
            }

            if (*iNdef >= iNdim_for_check)
                throw std::string("***ERROR: independent ones already reaches the allocated memory ");

            // Step 1: 构造当前列向量（C），表示本次模糊度组合
            dC_dot = 0.0;
            for (i = 0; i < *iNdef; i++) {
                _pdC[i] = 0.0;
                for (j = 0; j < iN_oneway; j++) {
                    if (arriIpt2ow[j] > iNdim_ow)
                        throw std::string("***ERROR: base element beyond the allocated memory ");
                    _pdC[i] += _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + i] * dOper[j];
                }
                dC_dot += _pdC[i] * _pdC[i];
            }

            // Step 2: 判断是否线性相关（即该向量为已有列的线性组合）
            isLdepend = false;
            if (abs(dC_dot - iN_oneway) <= dEPS) {
                isLdepend = true; // 存在线性依赖
                return isLdepend;
            }

            // Step 3: 构造新的正交列（施密特正交化）
            for (j = 0; j < iNamb; j++) {
                _pdE[j * (iNdim_for_check + 1) + *iNdef] = 0.0;
            }
            for (j = 0; j < iN_oneway; j++) {
                _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + *iNdef] = dOper[j];
            }

            // Step 4: Gram-Schmidt 正交化处理
            if (dC_dot > dEPS) {
                for (i = 0; i < *iNdef; i++) {
                    if (abs(_pdC[i]) < dEPS) continue;
                    for (j = 0; j < iNamb; j++) {
                        _pdE[j * (iNdim_for_check + 1) + *iNdef] -= _pdC[i] * _pdE[j * (iNdim_for_check + 1) + i];
                    }
                }

                dC_dot = 0.0;
                for (j = 0; j < iNamb; j++) {
                    dC_dot += _pdE[j * (iNdim_for_check + 1) + *iNdef] * _pdE[j * (iNdim_for_check + 1) + *iNdef];
                }
                dC_dot = sqrt(dC_dot);
                for (j = 0; j < iNamb; j++) {
                    _pdE[j * (iNdim_for_check + 1) + *iNdef] /= dC_dot;
                }
            }
            else {
                dC_dot = sqrt(iN_oneway * 1.0);
                for (j = 0; j < iN_oneway; j++) {
                    _pdE[(arriIpt2ow[j] - 1) * (iNdim_for_check + 1) + *iNdef] /= dC_dot;
                }
            }

            // Step 5: 标记新列并更新独立数目
            _pdC[*iNdef] = dC_dot;
            (*iNdef)++;
            return false;
        }
        catch (...) {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_check_amb_depend] : throw exception");
            return false;
        }
    }


    // _defineDDAmb(): 构造双差（Double-Difference）模糊度对
// 参数：amb_cmn - 共性处理对象，包含参数、状态等
// 返回值：成功返回模糊度参数数目，失败返回 -1
    int gnss_ambiguity::_defineDDAmb(gnss_amb_cmn* amb_cmn)
    {
        std::vector<base_par> params_all = amb_cmn->param().getAllPar();
        std::vector<base_par> params;
        int amb_idx1, amb_idx2;

        // 遍历所有参数，筛选出可用的模糊度参数
        for (auto it_par = params_all.begin(); it_par != params_all.end(); it_par++)
        {
            it_par->index = distance(params_all.begin(), it_par) + 1;
            it_par->pred = it_par->value() + amb_cmn->dx()(it_par->index); // 浮点模糊度预测值

            // 筛选条件：必须为模糊度参数，排除AMB13、已删除卫星、值为0的参数
            if (it_par->str_type().find("AMB") == std::string::npos ||
                it_par->str_type().find("AMB13") != std::string::npos ||
                _sat_rm.find(it_par->prn) != _sat_rm.end() ||
                double_eq(it_par->value(), 0.0))
            {
                continue;
            }

            // 根据频率类型进一步筛选可用频点
            if (it_par->str_type().find("AMB_L1") != std::string::npos)
            {
                if (!_ObsLevel.empty() && _ObsLevel[it_par->prn][FREQ_1] >= 0 && _ObsLevel[it_par->prn][FREQ_1] < 3)
                    continue;
                _amb_freqs[it_par->prn][it_par->site].push_back(FREQ_1);
            }
            else if (it_par->str_type().find("AMB_L2") != std::string::npos)
            {
                if (!_ObsLevel.empty() && _ObsLevel[it_par->prn][FREQ_2] >= 0 && _ObsLevel[it_par->prn][FREQ_2] < 3)
                    continue;
                _amb_freqs[it_par->prn][it_par->site].push_back(FREQ_2);
            }
            else if (it_par->str_type().find("AMB_L3") != std::string::npos)
            {
                if (!_ObsLevel.empty() && _ObsLevel[it_par->prn][FREQ_3] >= 0 && _ObsLevel[it_par->prn][FREQ_3] < 3)
                    continue;
            }

            // 保留该参数并记录其频率索引
            params.push_back(*it_par);
            _sats_index[it_par->prn][it_par->str_type().substr(0, 6)] = it_par->index;

            // 初始化每个卫星的锁定历元数、固定次数
            _lock_epo_num[it_par->prn] = round((it_par->end - it_par->beg) / _interval) + 1;
            if (_last_fix_time[amb_cmn->get_mode()].find(it_par->prn) == _last_fix_time[amb_cmn->get_mode()].end())
                _last_fix_time[amb_cmn->get_mode()][it_par->prn] = FIRST_TIME;
            if (_fix_epo_num[amb_cmn->get_mode()].find(it_par->prn) == _fix_epo_num[amb_cmn->get_mode()].end())
                _fix_epo_num[amb_cmn->get_mode()][it_par->prn] = 0;
        }

        // 若参数不足，无法构造DD
        if (params.size() < 2) {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : Ambguity's number is less than 2");
            return -1;
        }

        // 构建双差组合（遍历所有模糊度对）
        for (auto itsat1 = params.begin(); itsat1 != params.end() - 1; itsat1++) {
            amb_idx1 = distance(params.begin(), itsat1) + 1;
            for (auto itsat2 = itsat1 + 1; itsat2 != params.end(); itsat2++) {

                // 判断是否属于同一系统/同一站点
                if (!_simulation) {
                    if (gnss_sys::sat2gsys(itsat1->prn) != gnss_sys::sat2gsys(itsat2->prn) || itsat1->site != itsat2->site)
                        continue;
                }
                else {
                    GSYS tmp1 = gnss_sys::sat2gsys(itsat1->prn), tmp2 = gnss_sys::sat2gsys(itsat2->prn);
                    if (itsat1->prn.substr(0, 1) >= "2" && itsat1->prn.substr(0, 1) <= "5") tmp1 = GSYS::LEO;
                    if (itsat2->prn.substr(0, 1) >= "2" && itsat2->prn.substr(0, 1) <= "5") tmp2 = GSYS::LEO;
                    if (tmp1 != tmp2) continue;
                }

                if (itsat1->prn == itsat2->prn) continue;              // 相同卫星
                if (itsat1->parType != itsat2->parType) continue;      // 不同频率类型

                // 构造DD项
                gnss_amb_dd_base dd;
                dd.beg_epo = max(itsat1->beg, itsat2->beg);
                dd.end_epo = min(min(itsat1->end, itsat2->end), _crt_time);
                if ((dd.end_epo - dd.beg_epo) < _min_common_time) continue; // 共存时间不足

                dd.ambtype = itsat1->str_type().substr(0, 6);
                dd.isEwlFixed = dd.isEwl24Fixed = dd.isEwl25Fixed = dd.isWlFixed = dd.isNlFixed = false;
                amb_idx2 = distance(params.begin(), itsat2) + 1;
                dd.ddSats.push_back(std::make_tuple(itsat1->prn, itsat1->index, amb_idx1));
                dd.ddSats.push_back(std::make_tuple(itsat2->prn, itsat2->index, amb_idx2));
                dd.site = itsat1->site;

                // 计算双差值（meter 或 cycle）
                dd.rlc = itsat1->pred - itsat2->pred;
                if (itsat1->prn.substr(0, 1) == "R") {
                    if (_sys_wavelen[itsat1->prn]["NL"] == 0) continue;
                    dd.rlc = itsat1->pred / _sys_wavelen[itsat1->prn]["NL"] - itsat2->pred / _sys_wavelen[itsat2->prn]["NL"];
                    if (_obstype == OBSCOMBIN::RAW_ALL)
                        dd.rwl_R1 = itsat2->pred / _sys_wavelen[itsat2->prn]["NL"];
                    if (_obstype == OBSCOMBIN::WL_COMBIN)
                        dd.rlc = itsat1->pred / _sys_wavelen[itsat1->prn]["WL"] - itsat2->pred / _sys_wavelen[itsat2->prn]["WL"];
                }

                // 计算协方差
                if (_fix_mode != FIX_MODE::NO) {
                    double sigma = amb_cmn->Qx()(itsat1->index, itsat1->index)
                        - 2 * amb_cmn->Qx()(itsat1->index, itsat2->index)
                        + amb_cmn->Qx()(itsat2->index, itsat2->index);
                    if (double_eq(sigma, 0.0) || amb_cmn->sigma0() < 0) continue;
                    dd.srlc = amb_cmn->sigma0() * sqrt(abs(sigma)) * sigma / abs(sigma);
                }

                // 添加至DD列表
                _DD.push_back(dd);
            }
        }

        if (_DD.empty()) {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_defineDDAmb] : double-Difference ambiguity is empty");
            return -1;
        }
        else
            return params.size();
    }

    bool gnss_ambiguity::_calDDAmbWL()
    {
        std::string sat1, sat2;
        std::string sat11, sat12, sat21, sat22;
        //widelane ambiguities from MW-combination
        for (auto itdd = _DD.begin(); itdd != _DD.end();)
        {
            sat1 = get<0>(itdd->ddSats[0]);
            sat2 = get<0>(itdd->ddSats[1]);
            if (itdd->ddSats.size() != 2 || _MW.find(sat1) == _MW.end() || _MW.find(sat2) == _MW.end() || _MW[sat1][1] <= 2.0 || _MW[sat2][1] <= 2.0)
            {
                itdd = _DD.erase(itdd);
                continue;
            }
            else
            {
                itdd->rwl = _MW[sat1][2] - _MW[sat2][2];
                itdd->srwl = sqrt(abs(_MW[sat1][3] / _MW[sat1][1]) + abs(_MW[sat2][3] / _MW[sat2][1]));

                if (sat1.substr(0, 1) == "R")
                {
                    itdd->rwl_R1 = _MW[sat1][2];
                    itdd->srwl_R1 = sqrt(abs(_MW[sat1][3] / _MW[sat1][1]));
                }
                if (sat2.substr(0, 1) == "R")
                {
                    itdd->rwl_R2 = _MW[sat2][2];
                    itdd->srwl_R2 = sqrt(abs(_MW[sat2][3] / _MW[sat2][1]));
                }
                itdd++;
            }
        }
        if (_DD.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_calDDAmbWL] : Double-Difference ambiguity is empty");
            return false;
        }
        else
            return true;
    }
    bool gnss_ambiguity::_calDDAmbWLWL()
    {
        std::string sat1, sat2;
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            sat1 = get<0>(itdd->ddSats[0]);
            sat2 = get<0>(itdd->ddSats[1]);
            //xjhan
            if (sat1.substr(0, 1) != "R")
            {
                itdd->factor = _sys_wavelen[sat2.substr(0, 1)]["WL"];
                itdd->rlc = itdd->rlc / itdd->factor;
            }
            else
            {
                itdd->factor = sqrt(pow(_sys_wavelen[sat1]["WL"], 2) + pow(_sys_wavelen[sat2]["WL"], 2));
            }
            itdd->srlc = itdd->srlc / itdd->factor;

            itdd->rwl = itdd->rnl = itdd->rlc;
            itdd->srwl = itdd->srnl = itdd->srlc;
        }
        return true;
    }
    // _calDDAmbWLALL(): 计算所有双差模糊度的宽巷模糊度（Widelane）组合
 // 输入参数：
 // - amb_cmn: 模糊度共性处理对象，包含参数状态和协方差等
 // - mode: 当前处理模式（如 "WL", "EWL", "EWL24", "EWL25"）
 // 返回值：成功返回 true，失败返回 false
    bool gnss_ambiguity::_calDDAmbWLALL(gnss_amb_cmn* amb_cmn, std::string mode)
    {
        std::string sat11, sat12, sat21, sat22, ambtype1, ambtype2;
        double qq, lambda_1 = 0, lambda_2 = 0, lambda_21 = 0, lambda_22 = 0;

        // 遍历所有双差模糊度项
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            sat11 = get<0>(itdd->ddSats[0]);
            sat12 = get<0>(itdd->ddSats[1]);

            // 根据模式设定对应频率波长和模糊度类型
            if (mode == "WL")
            {
                if (sat11.substr(0, 1) != "R") // 非GLONASS卫星，取系统波长
                {
                    lambda_1 = _sys_wavelen[sat11.substr(0, 1)]["L1"];
                    lambda_2 = _sys_wavelen[sat11.substr(0, 1)]["L2"];
                }
                else // GLONASS特殊处理，波长按卫星编号区分
                {
                    lambda_1 = _sys_wavelen[sat11]["L1"];
                    lambda_2 = _sys_wavelen[sat11]["L2"];
                    lambda_21 = _sys_wavelen[sat12]["L1"];
                    lambda_22 = _sys_wavelen[sat12]["L2"];
                }
                ambtype1 = "AMB_L1";
                ambtype2 = "AMB_L2";
            }
            else if (mode == "EWL") // 扩展宽巷组合L2-L3
            {
                lambda_1 = _sys_wavelen[sat11.substr(0, 1)]["L2"];
                lambda_2 = _sys_wavelen[sat11.substr(0, 1)]["L3"];
                ambtype1 = "AMB_L2";
                ambtype2 = "AMB_L3";
            }
            else if (mode == "EWL24") // L2-L4组合
            {
                lambda_1 = _sys_wavelen[sat11.substr(0, 1)]["L2"];
                lambda_2 = _sys_wavelen[sat11.substr(0, 1)]["L4"];
                ambtype1 = "AMB_L2";
                ambtype2 = "AMB_L4";
            }
            else if (mode == "EWL25") // L2-L5组合
            {
                lambda_1 = _sys_wavelen[sat11.substr(0, 1)]["L2"];
                lambda_2 = _sys_wavelen[sat11.substr(0, 1)]["L5"];
                ambtype1 = "AMB_L2";
                ambtype2 = "AMB_L5";
            }

            // 如果当前模糊度类型不匹配跳过
            if (itdd->ambtype != ambtype1)
                continue;

            // 内层循环匹配另一类模糊度，计算宽巷双差
            for (auto itdd1 = _DD.begin(); itdd1 != _DD.end(); itdd1++)
            {
                sat21 = get<0>(itdd1->ddSats[0]);
                sat22 = get<0>(itdd1->ddSats[1]);

                if (itdd1->ambtype != ambtype2) // 模糊度类型不同跳过
                    continue;

                if (itdd1->site != itdd->site) // 不同接收机站点跳过
                    continue;

                if ((sat11 == sat21) && (sat12 == sat22)) // 卫星匹配，进行计算
                {
                    if (sat11.substr(0, 1) != "R")
                    {
                        // 非GLONASS，宽巷值计算，单位cycle
                        itdd->rwl = itdd->rlc / lambda_1 - itdd1->rlc / lambda_2;
                    }
                    else
                    {
                        // GLONASS需要考虑多频波长差异和校正
                        itdd->rwl = (itdd->rlc + itdd->rwl_R1) * _sys_wavelen[sat11]["NL"] / lambda_1 - itdd->rwl_R1 * _sys_wavelen[sat12]["NL"] / lambda_21;
                        itdd->rwl -= (itdd1->rlc + itdd1->rwl_R1) * _sys_wavelen[sat11]["NL"] / lambda_2 - itdd1->rwl_R1 * _sys_wavelen[sat12]["NL"] / lambda_22;
                    }

                    // 计算宽巷协方差
                    qq = amb_cmn->Qx()(get<1>(itdd->ddSats[0]), get<1>(itdd1->ddSats[0])) / lambda_1 * lambda_1
                        - amb_cmn->Qx()(get<1>(itdd->ddSats[0]), get<1>(itdd1->ddSats[1])) / lambda_1 * lambda_2
                        - amb_cmn->Qx()(get<1>(itdd->ddSats[1]), get<1>(itdd1->ddSats[0])) / lambda_1 * lambda_2
                        + amb_cmn->Qx()(get<1>(itdd->ddSats[1]), get<1>(itdd1->ddSats[1])) / lambda_2 * lambda_2;

                    if (double_eq(qq, 0.0)) // 避免除零错误
                        continue;

                    if (amb_cmn->sigma0() < 0) // 浮点标准差无效
                        continue;

                    // 计算宽巷模糊度标准差srwl
                    itdd->srwl = itdd->srlc / lambda_1 * lambda_1 + itdd1->srlc / lambda_2 * lambda_2
                        - 2 * (amb_cmn->sigma0() * sqrt(abs(qq)) * qq / abs(qq));

                    // 保存宽巷双差对应的两个模糊度项
                    itdd->ddSats.push_back(std::make_tuple(sat21, get<1>(itdd1->ddSats[0]), get<2>(itdd1->ddSats[0])));
                    itdd->ddSats.push_back(std::make_tuple(sat22, get<1>(itdd1->ddSats[1]), get<2>(itdd1->ddSats[1])));

                    break; // 找到匹配后跳出内循环
                }
            }
        }

        if (_DD.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_calDDAmbWL] : Double-Difference ambiguity is empty");
            return false;
        }
        else
            return true;
    }


    void gnss_ambiguity::_DDsitecheck(std::map<std::string, int>& DDsitelist)
    {
        std::map<std::string, int> DDsite_tmp;
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            DDsite_tmp.insert(std::make_pair(itdd->site, 0));
        }
        for (auto sitename = DDsite_tmp.begin(); sitename != DDsite_tmp.end(); sitename++)
        {
            for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
            {
                if (itdd->site == sitename->first)
                    sitename->second += 1;
            }
            DDsitelist.insert(std::make_pair(sitename->first,sitename->second));
        }
    }

    void gnss_ambiguity::_DDsiteerase(std::map<std::string, int>& DDsitelist)
    {
        int _max = 0, _min = 1000;
        std::string maxsite="default", minsite="default";
        bool loop = false;
        if (_max == 0 && _min == 1000)
           loop = true;
        while (loop) {
            _max = 0; _min = 1000;
            for (auto it_site = DDsitelist.begin(); it_site != DDsitelist.end(); it_site++)
            {
                int max_sav = _max;
                int min_sav = _min;
                _max = std::max(_max, it_site->second);
                _min = std::min(_min, it_site->second);
                if (_max != max_sav)
                    maxsite = it_site->first;
                if (_min != min_sav)
                    minsite = it_site->first;
            }
            if (_min !=1000 && _max != 0 && _min < 0.5 * _max)
            {
                for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
                {
                    std::cerr << std::endl<<"minsite: "<<minsite << std::endl;
                    std::cerr << std::endl << "maxsite: " << maxsite << std::endl;
                    std::cerr << std::endl << "ddsite: " << itdd->site << std::endl;
                    if (itdd->site == minsite)
                    {
                        itdd = _DD.erase(itdd);
                        itdd--;
                    }
                }
                for (auto it_site = DDsitelist.begin(); it_site != DDsitelist.end(); it_site++)
                {
                    if (it_site->first == minsite)
                        DDsitelist.erase(it_site);
                }
            }
            else
                loop = false;
        }
    }

    bool gnss_ambiguity::_getSingleUpd(std::string mode, base_time t, std::string sat, double &value, double &sigma)
    {
        UPDTYPE upd_type = str2updmode(base_type_conv::trim(mode));
        one_epoch_upd epoch_upd = _gupd->get_epo_upd(upd_type, t);
        value = 0.0;
        if (epoch_upd.find(sat) == epoch_upd.end())
            return false;
        if (mode == "EWL" || mode == "EWL24" || mode == "EWL25")
        {
            if (epoch_upd[sat]->sigma > 0.2 || epoch_upd[sat]->npoint <= 2)
                return false;
            value = epoch_upd[sat]->value;
        }
        else if (mode == "EWL_EPOCH")
        {
            //one_epoch_upd_epoch epoch_upd_epoch = _gupd_epoch->get_epo_upd_epoch(t);
            if (epoch_upd[sat]->sigma > 0.2 || epoch_upd[sat]->npoint <= 2)
                return false;
            value = epoch_upd[sat]->value;
        }
        //LX changed add the else
        else if (mode == "WL")
        {
            if (epoch_upd[sat]->sigma > 0.2 || epoch_upd[sat]->npoint <= 2)
                return false;
            value = epoch_upd[sat]->value;
        }
        else if (mode == "NL")
        {
            if (epoch_upd[sat]->sigma > 0.1 || epoch_upd[sat]->npoint <= 3)
                return false;
            value = epoch_upd[sat]->value;
            sigma += pow(epoch_upd[sat]->sigma, 2);
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applySingleUpd] : Undefined UPD Mode (WL/NL) : " + mode);
            return false;
        }
        return true;
    }

    // _applyUpd(): 应用UPD（User Phase Delay）修正，更新双差模糊度的窄巷和宽巷值
   // 参数：
   //   t - 当前时间
   // 返回值：总是返回true（过程正常完成）
   // 功能说明：从UPD模块获取当前时间各卫星对应的UPD值，
   // 结合宽巷和窄巷模糊度计算修正，更新DD模糊度的相关值。
    bool gnss_ambiguity::_applyUpd(base_time t)
    {
        std::string sat1, sat2;
        double upd_wl1, upd_wl2;       // 宽巷UPD值
        double upd_ewl1, upd_ewl2;     // 扩展宽巷UPD值
        double upd_nl1, upd_nl2;       // 窄巷UPD值
        double sig, rec_upd;           // 卫星接收机UPD
        double upd_ewl24_1, upd_ewl24_2, upd_ewl25_1, upd_ewl25_2; // 其他扩展宽巷UPD值

        upd_wl1 = upd_wl2 = rec_upd = 0.0;

        // 如果有UPD模块且系统包含GLONASS，则获取GLONASS接收机UPD
        if (_gupd)
        {
            if (_sys.find("GLO") != _sys.end())
            {
                rec_upd = _glonassRecUpd(_site);
            }
        }

        // 遍历所有双差模糊度项，准备应用UPD修正
        for (auto itdd = _DD.begin(); itdd != _DD.end();)
        {
            sat1 = get<0>(itdd->ddSats[0]); // 双差第1颗卫星编号
            sat2 = get<0>(itdd->ddSats[1]); // 双差第2颗卫星编号

            // 初始化UPD修正值为0
            upd_nl1 = upd_nl2 = sig = 0.0;
            upd_wl1 = upd_wl2 = 0.0;
            upd_ewl1 = upd_ewl2 = 0.0;
            upd_ewl24_1 = upd_ewl24_2 = 0.0;
            upd_ewl25_1 = upd_ewl25_2 = 0.0;

            if (_gupd)
            {
                // 多频情况下获取扩展宽巷UPD
                if (_frequency >= 3 && _obstype == OBSCOMBIN::RAW_ALL)
                {
                    if (!_getSingleUpd("EWL", _ewl_Upd_time, sat1, upd_ewl1, sig) ||
                        !_getSingleUpd("EWL", _ewl_Upd_time, sat2, upd_ewl2, sig))
                    {
                        if (itdd->ambtype == "AMB_L3")
                        {
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : EWL, Sat: " + sat1 + " " + sat2);
                            itdd = _DD.erase(itdd);
                            continue;
                        }
                        else
                        {
                            upd_ewl1 = upd_ewl2 = 0.0;
                        }
                    }
                }

                // 类似处理扩展宽巷24、25频点UPD
                if (_frequency >= 4 && _obstype == OBSCOMBIN::RAW_ALL)
                {
                    if (!_getSingleUpd("EWL24", _ewl24_Upd_time, sat1, upd_ewl24_1, sig) ||
                        !_getSingleUpd("EWL24", _ewl24_Upd_time, sat2, upd_ewl24_2, sig))
                    {
                        if (itdd->ambtype == "AMB_L4")
                        {
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : EWL24, Sat: " + sat1 + " " + sat2);
                            itdd = _DD.erase(itdd);
                            continue;
                        }
                        else
                        {
                            upd_ewl24_1 = upd_ewl24_2 = 0.0;
                        }
                    }
                }
                if (_frequency == 5 && _obstype == OBSCOMBIN::RAW_ALL)
                {
                    if (!_getSingleUpd("EWL25", _ewl25_Upd_time, sat1, upd_ewl25_1, sig) ||
                        !_getSingleUpd("EWL25", _ewl25_Upd_time, sat2, upd_ewl25_2, sig))
                    {
                        if (itdd->ambtype == "AMB_L5")
                        {
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : EWL25, Sat: " + sat1 + " " + sat2);
                            itdd = _DD.erase(itdd);
                            continue;
                        }
                        else
                        {
                            upd_ewl25_1 = upd_ewl25_2 = 0.0;
                        }
                    }
                }

                // 获取宽巷UPD，排除OSB模式
                if (_upd_mode != UPD_MODE::OSB)
                {
                    if (!_getSingleUpd("WL", _wl_Upd_time, sat1, upd_wl1, sig) ||
                        !_getSingleUpd("WL", _wl_Upd_time, sat2, upd_wl2, sig))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : WL, Sat: " + sat1 + " " + sat2);
                        itdd = _DD.erase(itdd);
                        continue;
                    }
                }

                // 获取窄巷UPD，仅UPD模式生效
                if (_upd_mode == UPD_MODE::UPD)
                {
                    if (!_getSingleUpd("NL", t, sat1, upd_nl1, sig) ||
                        !_getSingleUpd("NL", t, sat2, upd_nl2, sig))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : NL, Sat: " + sat1 + " " + sat2);
                        itdd = _DD.erase(itdd);
                        continue;
                    }
                }
            }

            // 计算窄巷UPD修正差值
            itdd->sd_rnl_cor = upd_nl1 - upd_nl2;
            itdd->sigcor = sqrt(sig);

            // 针对不同观测组合类型，计算窄巷和宽巷模糊度的最终值
            if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_SINGLE || _obstype == OBSCOMBIN::RAW_MIX)
            {
                if (sat1.substr(0, 1) != "R") // 非GLONASS卫星
                {
                    // 获取卫星频点波长
                    double lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L1"];
                    double lambda_2 = _sys_wavelen[sat2.substr(0, 1)]["L2"];
                    double lambda_3 = _sys_wavelen[sat2.substr(0, 1)]["L3"];
                    double lambda_4 = _sys_wavelen[sat2.substr(0, 1)]["L4"];
                    double lambda_5 = _sys_wavelen[sat2.substr(0, 1)]["L5"];

                    // 计算并修正L1-L5各频点的模糊度
                    itdd->sd_r1_cor = itdd->sd_rnl_cor - lambda_1 / (lambda_2 - lambda_1) * (-upd_wl1 + upd_wl2);
                    itdd->sd_r2_cor = itdd->sd_rnl_cor - lambda_2 / (lambda_2 - lambda_1) * (-upd_wl1 + upd_wl2);
                    if (upd_ewl1 != 0.0 && upd_ewl2 != 0.0)
                        itdd->sd_r3_cor = itdd->sd_r2_cor - (-upd_ewl1 + upd_ewl2);
                    if (upd_ewl24_1 != 0.0 && upd_ewl24_2 != 0.0)
                        itdd->sd_r4_cor = itdd->sd_r2_cor - (-upd_ewl24_1 + upd_ewl24_2);
                    if (upd_ewl25_1 != 0.0 && upd_ewl25_2 != 0.0)
                        itdd->sd_r5_cor = itdd->sd_r2_cor - (-upd_ewl25_1 + upd_ewl25_2);

                    // 根据模糊度类型赋值窄巷模糊度rnl及相关系数factor
                    if (itdd->ambtype == "AMB_L1")
                    {
                        itdd->rnl = itdd->rlc / lambda_1 + itdd->sd_r1_cor;
                        itdd->srnl = itdd->srlc / lambda_1;
                        itdd->factor = lambda_1;
                    }
                    else if (itdd->ambtype == "AMB_L2")
                    {
                        itdd->rnl = itdd->rlc / lambda_2 + itdd->sd_r2_cor;
                        itdd->srnl = itdd->srlc / lambda_2;
                        itdd->factor = lambda_2;
                    }
                    else if (itdd->ambtype == "AMB_L3")
                    {
                        itdd->rnl = itdd->rlc / lambda_3 + itdd->sd_r3_cor;
                        itdd->srnl = itdd->srlc / lambda_3;
                        itdd->factor = lambda_3;
                    }
                    else if (itdd->ambtype == "AMB_L4")
                    {
                        itdd->rnl = itdd->rlc / lambda_4;
                        itdd->rnl += itdd->sd_r4_cor;
                        itdd->srnl = itdd->srlc / (lambda_4);
                        itdd->factor = lambda_4;
                    }
                    else if (itdd->ambtype == "AMB_L5")
                    {
                        itdd->rnl = itdd->rlc / lambda_5;
                        itdd->rnl += itdd->sd_r5_cor;
                        itdd->srnl = itdd->srlc / (lambda_5);
                        itdd->factor = lambda_5;
                    }
                }
                else // GLONASS卫星特殊计算
                { // added by hlgou 2022.1.10
                    double lambda_11, lambda_12, lambda_21, lambda_22;
                    lambda_11 = _sys_wavelen[sat1]["L1"];
                    lambda_12 = _sys_wavelen[sat1]["L2"];
                    lambda_21 = _sys_wavelen[sat2]["L1"];
                    lambda_22 = _sys_wavelen[sat2]["L2"];

                    itdd->sd_r1_cor = upd_nl1 + lambda_11 / (lambda_12 - lambda_11) * upd_wl1;
                    itdd->sd_r1_cor -= (upd_nl2 + lambda_21 / (lambda_22 - lambda_21) * upd_wl2);
                    itdd->sd_r2_cor = upd_nl1 + lambda_12 / (lambda_12 - lambda_11) * upd_wl1;
                    itdd->sd_r2_cor -= (upd_nl2 + lambda_22 / (lambda_22 - lambda_21) * upd_wl2);

                    if (itdd->ambtype == "AMB_L1")
                    {
                        itdd->rnl = (itdd->rlc + itdd->rwl_R1) * _sys_wavelen[sat1]["NL"] / lambda_11;
                        itdd->rnl -= ((itdd->rwl_R1) * _sys_wavelen[sat2]["NL"] / lambda_21);
                        itdd->rnl += itdd->sd_r1_cor;
                        itdd->srnl = itdd->srlc / (lambda_11);
                        itdd->factor = lambda_11;
                    }
                    else if (itdd->ambtype == "AMB_L2")
                    {
                        itdd->rnl = (itdd->rlc + itdd->rwl_R1) * _sys_wavelen[sat1]["NL"] / lambda_12;
                        itdd->rnl -= ((itdd->rwl_R1) * _sys_wavelen[sat2]["NL"] / lambda_22);
                        itdd->rnl += itdd->sd_r2_cor;
                        itdd->srnl = itdd->srlc / (lambda_21);
                        itdd->factor = lambda_21;
                    }
                }
            }

            // 针对电离层自由组合情况
            if (_obstype == OBSCOMBIN::IONO_FREE)
            {
                if (sat1.substr(0, 1) != "R")
                {
                    // 非GLONASS卫星，应用宽巷UPD修正（非OSB模式）
                    if (_upd_mode != UPD_MODE::OSB)
                        itdd->rwl += (-upd_wl1 + upd_wl2);

                    // 计算窄巷模糊度rnl，单位cycle
                    itdd->rnl = itdd->rlc / _sys_wavelen[sat1.substr(0, 1)]["NL"]
                        - round(itdd->rwl) * _sys_wavelen[sat1.substr(0, 1)]["WL"] / _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    itdd->srnl = itdd->srlc / _sys_wavelen[sat1.substr(0, 1)]["NL"];
                    itdd->factor = _sys_wavelen[sat1.substr(0, 1)]["NL"];

                    // 应用窄巷UPD修正
                    if (_upd_mode == UPD_MODE::UPD)
                        itdd->rnl += itdd->sd_rnl_cor;
                }
                else
                {
                    // GLONASS卫星特殊处理，应用宽巷UPD修正并计算窄巷模糊度
                    if (_upd_mode != UPD_MODE::OSB)
                        itdd->rwl += (-upd_wl1 + upd_wl2);

                    itdd->rnl = itdd->rlc - round(itdd->rwl) * 3.5; // LX修改的固定宽巷波长
                    itdd->srnl = itdd->srlc / _sys_wavelen[sat1]["NL"];
                    itdd->factor = _sys_wavelen[sat1]["NL"];

                    if (_upd_mode == UPD_MODE::UPD)
                        itdd->rnl += itdd->sd_rnl_cor;
                }
            }


#ifdef DEBUG
            std::cout << "correct NLUPD " << std::fixed << std::setw(20) << std::setprecision(3) << itdd->rwl << std::setw(20) << std::setprecision(3) << itdd->rlc
                 << "  " << sat1 << "  " << sat2 << std::setw(20) << std::setprecision(3) << itdd->sd_r1_cor << std::setw(20) << std::setprecision(3) << itdd->sd_r2_cor
                 << std::setw(20) << std::setprecision(3) << itdd->rnl
                 << std::setw(20) << std::setprecision(3) << itdd->srlc << std::setw(20) << std::setprecision(3) << itdd->srnl << std::endl;
#endif
            itdd++;
        }

        return true;
    }

    // _applyWLUpd: 应用宽巷类模糊度的UPD（更新）修正
 // 参数:
 //   t    - 当前时间戳，用于查找对应时间的UPD值
 //   mode - 宽巷模糊度类型字符串，支持 "WL", "EWL", "EWL24", "EWL25"
 // 返回值:
 //   始终返回true
 // 功能说明:
 //   遍历所有双差模糊度项，依据传入的mode类型从UPD数据结构中提取对应的UPD值。
 //   包括宽巷(WL)和扩展宽巷(EWL、EWL24、EWL25)的UPD修正。
 //   对于每个双差项，根据卫星系统不同（非GLONASS与GLONASS区分）及mode，更新模糊度估计值及相关变量。
 //   若UPD数据缺失或错误，剔除对应双差项。
 //   更新模糊度的UPD偏差值和波长因子，为后续模糊度固定做准备。
    bool gnss_ambiguity::_applyWLUpd(base_time t, std::string mode)
    {
        std::string sat1, sat2;          // 双差涉及的两个卫星编号
        double upd_wl1, upd_wl2;    // 宽巷UPD修正值
        double upd_ewl1, upd_ewl2;  // 扩展宽巷UPD修正值
        double sig;                 // UPD对应的协方差或标准差
        upd_wl1 = upd_wl2 = upd_ewl1 = upd_ewl2 = 0.0;

        for (auto itdd = _DD.begin(); itdd != _DD.end();)
        {
            sat1 = get<0>(itdd->ddSats[0]); // 获取双差第一个卫星
            sat2 = get<0>(itdd->ddSats[1]); // 获取双差第二个卫星

            sig = 0.0;

            if (_gupd) // 如果UPD数据存在
            {
                // 根据频率和mode选择对应UPD修正，若查找失败，则剔除该双差项
                if (_frequency >= 3 && mode == "EWL")
                {
                    if ((!_getSingleUpd("EWL", _ewl_Upd_time, sat1, upd_ewl1, sig)) ||
                        (!_getSingleUpd("EWL", _ewl_Upd_time, sat2, upd_ewl2, sig) && mode == "EWL"))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : EWL, Sat: " + sat1 + " " + sat2);
                        itdd = _DD.erase(itdd);
                        continue;
                    }
                }
                else if (_frequency >= 4 && mode == "EWL24")
                {
                    if (!_getSingleUpd("EWL24", _ewl24_Upd_time, sat1, upd_ewl1, sig) ||
                        (!_getSingleUpd("EWL24", _ewl24_Upd_time, sat2, upd_ewl2, sig) && mode == "EWL24"))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : EWL24, Sat: " + sat1 + " " + sat2);
                        itdd = _DD.erase(itdd);
                        continue;
                    }
                }
                else if (_frequency == 5 && mode == "EWL25")
                {
                    if (!_getSingleUpd("EWL25", _ewl25_Upd_time, sat1, upd_ewl1, sig) ||
                        (!_getSingleUpd("EWL25", _ewl25_Upd_time, sat2, upd_ewl2, sig) && mode == "EWL25"))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : EWL25, Sat: " + sat1 + " " + sat2);
                        itdd = _DD.erase(itdd);
                        continue;
                    }
                }

                // 获取宽巷(WL)的UPD修正
                if (!_getSingleUpd("WL", _wl_Upd_time, sat1, upd_wl1, sig) ||
                    !_getSingleUpd("WL", _wl_Upd_time, sat2, upd_wl2, sig))
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_applyUpd] : _getSingleUpd Wrong : WL, Sat: " + sat1 + " " + sat2);
                    itdd = _DD.erase(itdd);
                    continue;
                }
            }

            // 对非GLONASS卫星处理（sat1编号首字母非'R'）
            if (sat1.substr(0, 1) != "R")
            {
                // 根据模糊度类型设置对应的波长因子
                if (itdd->ambtype == "AMB_L2")
                {
                    itdd->factor = _sys_wavelen[sat2.substr(0, 1)]["L2"];
                }

                if (mode == "WL" && itdd->rwl != 0)
                {
                    // 宽巷模糊度加上UPD修正差值（卫星2 - 卫星1）
                    itdd->rwl += (-upd_wl1 + upd_wl2);
                    itdd->sd_rwl_cor = (-upd_wl1 + upd_wl2);

                    // 记录波长因子
                    itdd->factor = _sys_wavelen[sat1.substr(0, 1)]["WL"];

                    itdd++; // 处理下一个双差项
                    continue;
                }
                else if (mode == "EWL" && itdd->rwl != 0)
                {
                    itdd->rewl = itdd->rwl + (-upd_ewl1 + upd_ewl2);
                    itdd->sd_rewl_cor = (-upd_ewl1 + upd_ewl2);
                    itdd->factor = _sys_wavelen[sat1.substr(0, 1)]["EWL"];

                    itdd++;
                    continue;
                }
                else if (mode == "EWL24" && itdd->rwl != 0)
                {
                    itdd->rewl24 = itdd->rwl + (-upd_ewl1 + upd_ewl2);
                    itdd->sd_rewl24_cor = (-upd_ewl1 + upd_ewl2);
                    itdd->factor = _sys_wavelen[sat1.substr(0, 1)]["EWL24"];

                    itdd++;
                    continue;
                }
                else if (mode == "EWL25" && itdd->rwl != 0)
                {
                    itdd->rewl25 = itdd->rwl + (-upd_ewl1 + upd_ewl2);
                    itdd->sd_rewl25_cor = (-upd_ewl1 + upd_ewl2);
                    itdd->factor = _sys_wavelen[sat1.substr(0, 1)]["EWL25"];

                    itdd++;
                    continue;
                }
                else
                {
                    itdd++;
                    continue;
                }
            }
            else
            {
                // GLONASS卫星特殊处理（编号首字母为'R'）
                if (mode == "WL" && itdd->rwl != 0)
                {
                    itdd->rwl += (-upd_wl1 + upd_wl2);
                    itdd->sd_rwl_cor = (-upd_wl1 + upd_wl2);
                    itdd->factor = _sys_wavelen[sat1]["WL"];

                    itdd++;
                    continue;
                }
                else
                {
                    itdd++;
                    continue;
                }
            }
        }
        return true;
    }


    bool gnss_ambiguity::_fixAmbIF()
    {
        std::string sat1, sat2;
        gnss_amb_bdeci bdeci;
        double prob, alpha, alpha1, alpha2;

        for (auto itdd = _DD.begin(); itdd != _DD.end();)
        {
            sat1 = get<0>(itdd->ddSats[0]);
            sat2 = get<0>(itdd->ddSats[1]);

            // Judge Widelane Ambiguity whether can be std::fixed
            if (sat1.substr(0, 1) != "R") //
            {
                bdeci.bdeci(itdd->rwl, itdd->srwl, 1, _map_WL_decision["maxdev"],
                            _map_WL_decision["maxsig"], prob, alpha);
                if (alpha > _map_WL_decision["alpha"])
                    itdd->isWlFixed = true;
                if (_part_fix && abs(itdd->rwl - round(itdd->rwl)) < 0.2 && itdd->srwl < 0.5)
                {
                    itdd->isWlFixed = true;
                }
            }
            else
            {
                bdeci.bdeci(itdd->rwl_R1, itdd->srwl_R1, 1, _map_WL_decision["maxdev"],
                            _map_WL_decision["maxsig"], prob, alpha1);
                bdeci.bdeci(itdd->rwl_R2, itdd->srwl_R2, 1, _map_WL_decision["maxdev"],
                            _map_WL_decision["maxsig"], prob, alpha2);
                if (alpha1 > _map_WL_decision["alpha"] && alpha2 > _map_WL_decision["alpha"])
                {
                    itdd->isWlFixed = true;
                }
                if (_part_fix && abs(itdd->rwl_R1 - round(itdd->rwl_R1)) < 0.25 && abs(itdd->rwl_R2 - round(itdd->rwl_R2)) < 0.25)
                {
                    itdd->isWlFixed = true;
                }
            }

            // Judge Narrowlane Ambiguity whether can be std::fixed
            if (double_eq(itdd->srnl, 0.0))
                itdd->srnl = 0.05;
            bdeci.bdeci(itdd->rnl, itdd->srnl, 1, _map_NL_decision["maxdev"],
                        _map_NL_decision["maxsig"], prob, alpha);
            if (alpha > _map_NL_decision["alpha"])
                itdd->isNlFixed = true;
            // Tips: MW[_sat1][5] means elevation
            if (sat1.substr(0, 1) != "R")
            {
                if (_part_fix && abs(itdd->rnl - round(itdd->rnl)) < 0.2 && _MW[sat1][5] > 25.0 && _MW[sat2][5] > 25.0)
                {
                    itdd->isNlFixed = true;
                }
            }
            else
            {
                if (_part_fix && abs(itdd->rnl - round(itdd->rnl)) < 0.25 && _MW[sat1][5] > 20.0 && _MW[sat2][5] > 20.0)
                {
                    itdd->isNlFixed = true;
                }
            }

            //std::cout << "  " << sat1 << "  " << sat2 << std::fixed << std::setw(17) << std::setprecision(4) << itdd->rwl << std::setw(17) << std::setprecision(4) << itdd->srwl << std::setw(6) << itdd->isWlFixed << std::setw(17) << std::setprecision(4) << itdd->rnl << std::setw(17) << std::setprecision(4) << itdd->srnl << std::setw(6)  << itdd->isNlFixed
            //    << std::setw(17) << std::setprecision(4) << _MW[sat1][5] << std::setw(17) << std::setprecision(4) << _MW[sat2][5] << std::endl;
            //std::cout << std::fixed << sat1 << "  " << sat2 << "   " << std::setw(9) << std::setprecision(4) << _MW[sat1][5]
            //     << std::setw(9) << std::setprecision(4) << _MW[sat2][5] << std::setw(17) << std::setprecision(4) << itdd->rwl
            //     << std::setw(17) << std::setprecision(4) << itdd->srwl << std::setw(6) << (int)itdd->isWlFixed
            //     << std::setw(17) << std::setprecision(4) << itdd->rnl << std::setw(17) << std::setprecision(4) << itdd->srnl
            //     << std::setw(6) << (int)itdd->isNlFixed << std::endl;
            itdd++;
        }

        return true;
    }

    // _fixAmbUDUC(): 判断并标记窄巷和宽巷模糊度是否被成功固定
 // 返回值：成功返回true，失败返回false
 // 功能说明：针对当前所有双差模糊度项，依据模糊度估计值、信号高度角和观测类型判断是否满足固定条件。
 // 特别处理单频情况及不同观测组合，设置模糊度的固定标志。
    bool gnss_ambiguity::_fixAmbUDUC()
    {
        std::string sat1, sat2;

        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            sat1 = get<0>(itdd->ddSats[0]); // 卫星1编号
            sat2 = get<0>(itdd->ddSats[1]); // 卫星2编号

            // 设置扩展宽巷和宽巷的固定标志（从预先计算的标志映射中获取）
            itdd->isEwlFixed = _EWL_flag[sat1][sat2][itdd->site];
            itdd->isEwl24Fixed = _EWL24_flag[sat1][sat2];
            itdd->isEwl25Fixed = _EWL25_flag[sat1][sat2];
            itdd->isWlFixed = _WL_flag[sat1][sat2][itdd->site];

            // 判断是否为单频模糊度（频点数量为1）
            if (_amb_freqs[sat1][itdd->site].size() == 1 || _amb_freqs[sat2][itdd->site].size() == 1)
                itdd->isSngleFreq = true;

            // 针对RAW_MIX观测组合且单频的模糊度，不处理某些频点的模糊度固定
            if (_obstype == OBSCOMBIN::RAW_MIX && itdd->isSngleFreq == true)
            {
                if (itdd->ambtype == "AMB_L5" || itdd->ambtype == "AMB_L4" || itdd->ambtype == "AMB_L3")
                    continue;
            }
            else
            {
                if (itdd->ambtype == "AMB_L5" || itdd->ambtype == "AMB_L4" || itdd->ambtype == "AMB_L3" || itdd->ambtype == "AMB_L2")
                    continue;
            }

            // 防止标准差为0，赋一个最小正值
            if (double_eq(itdd->srnl, 0.0))
                itdd->srnl = 0.05;

            // 根据窄巷模糊度偏差与阈值判断是否可固定（默认最大偏差通过_map_NL_decision["maxdev"]设定）
            if (abs(itdd->rnl - round(itdd->rnl)) < _map_NL_decision["maxdev"])
            {
                itdd->isNlFixed = true;
            }

            // 若两个卫星连续固定次数及锁定次数较多，放宽固定条件
            if (_fix_epo_num["NL"][sat1] > 20 && _fix_epo_num["NL"][sat2] > 20 &&
                _lock_epo_num[sat1] > 200 && _lock_epo_num[sat2] > 200)
            {
                if (abs(itdd->rnl - round(itdd->rnl)) < _map_NL_decision["maxdev"] * 1.1)
                {
                    itdd->isNlFixed = true;
                }
            }

#ifdef DEBUG
            // 调试输出当前模糊度信息，包括站点、类型、卫星、高度角及固定状态
            std::cout << _site << "  " << itdd->ambtype << "  " << sat1 << "  " << sat2
                << std::fixed << std::setw(17) << std::setprecision(4) << itdd->rnl
                << std::setw(17) << std::setprecision(4) << itdd->srnl
                << std::setw(17) << std::setprecision(4) << _ELE[sat1]
                << std::setw(17) << std::setprecision(4) << _ELE[sat2]
                << std::setw(6) << boolalpha << itdd->isNlFixed << std::endl;
#endif
        }

        if (_DD.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_fixAmbUDUC] : Double-Difference ambiguity is empty");
            return false;
        }
        else
            return true;
        }


    // _fixAmbWL(): 判断并标记宽巷模糊度是否被成功固定
    // 返回值：总是返回true
    // 功能说明：利用概率判决算法，根据宽巷模糊度估计及其标准差计算固定概率，判断是否固定宽巷模糊度。
    // 额外支持部分固定条件。
    bool gnss_ambiguity::_fixAmbWL()
    {
        std::string sat1, sat2;
        gnss_amb_bdeci bdeci;  // 模糊度概率判决工具类实例
        double prob, alpha;

        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            sat1 = get<0>(itdd->ddSats[0]);
            sat2 = get<0>(itdd->ddSats[1]);

            // 调用概率判决函数，计算宽巷模糊度的固定概率和置信度alpha
            bdeci.bdeci(itdd->rwl, itdd->srwl, 1, _map_WL_decision["maxdev"],
                _map_WL_decision["maxsig"], prob, alpha);

            // 若置信度alpha超过阈值，则标记宽巷和窄巷模糊度为固定
            if (alpha > _map_WL_decision["alpha"])
            {
                itdd->isWlFixed = true;
                itdd->isNlFixed = true;
            }

            // 支持部分固定策略：当宽巷模糊度接近整数时也判为固定
            if (_part_fix && abs(itdd->rwl - round(itdd->rwl)) < 0.25)
            {
                itdd->isWlFixed = true;
                itdd->isNlFixed = true;
            }
        }
        return true;
    }

    // _fixAmbWL(std::string mode): 判断并标记宽巷类模糊度是否被成功固定
 // 参数:
 //   mode - 模糊度类型字符串，支持 "WL", "EWL", "EWL24", "EWL25"
 // 返回值:
 //   始终返回true
 // 功能说明:
 //   遍历所有双差模糊度项，基于模糊度的浮点值与最接近整数的距离以及锁定次数等条件，判断模糊度是否满足固定条件。
 //   根据不同mode，分别对宽巷(WL)、扩展宽巷(EWL)、以及两种扩展宽巷24/25频点(EWL24, EWL25)进行判断。
 //   并设置对应的固定标志及整数化模糊度值。
    bool gnss_ambiguity::_fixAmbWL(std::string mode)
    {
        std::string sat1, sat2;

        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            sat1 = get<0>(itdd->ddSats[0]);
            sat2 = get<0>(itdd->ddSats[1]);

            if (mode == "WL")
            {
                // 过滤掉模糊度值为0的项
                if (itdd->rwl == 0)
                    continue;
                _WL_flag[sat1][sat2][itdd->site] = false;

                // 防止标准差为0，赋默认最小值
                if (double_eq(itdd->srwl, 0.0))
                    itdd->srwl = 0.05;

                // 若模糊度浮点值与整数距离小于阈值，认为该宽巷模糊度已固定
                if (abs(itdd->rwl - round(itdd->rwl)) < _map_WL_decision["maxdev"])
                {
                    itdd->isWlFixed = true;
                    _WL_flag[sat1][sat2][itdd->site] = true;
                    itdd->iwl = round(itdd->rwl);
                }

                // 如果该双差两颗卫星锁定次数和固定次数较多，放宽固定条件
                if (_fix_epo_num[mode][sat1] > 20 && _fix_epo_num[mode][sat2] > 20 &&
                    _lock_epo_num[sat1] > 200 && _lock_epo_num[sat2] > 200)
                {
                    if (abs(itdd->rwl - round(itdd->rwl)) < _map_WL_decision["maxdev"] * 1.1)
                    {
                        itdd->isWlFixed = true;
                        _WL_flag[sat1][sat2][itdd->site] = true;
                        itdd->iwl = round(itdd->rwl);
                    }
                }
            }
            else if (mode == "EWL")
            {
                if (itdd->rewl == 0)
                    continue;
                _EWL_flag[sat1][sat2][itdd->site] = false;

                if (double_eq(itdd->srewl, 0.0))
                    itdd->srewl = 0.05;

                if (abs(itdd->rewl - round(itdd->rewl)) < _map_EWL_decision["maxdev"])
                {
                    itdd->isEwlFixed = true;
                    _EWL_flag[sat1][sat2][itdd->site] = true;
                    itdd->iewl = round(itdd->rewl);
                }

                if (_fix_epo_num[mode][sat1] > 20 && _fix_epo_num[mode][sat2] > 20 &&
                    _lock_epo_num[sat1] > 200 && _lock_epo_num[sat2] > 200)
                {
                    if (abs(itdd->rewl - round(itdd->rewl)) < _map_EWL_decision["maxdev"] * 1.1)
                    {
                        itdd->isEwlFixed = true;
                        _EWL_flag[sat1][sat2][itdd->site] = true;
                        itdd->iewl = round(itdd->rewl);
                    }
                }
            }
            else if (mode == "EWL24")
            {
                if (itdd->rewl24 == 0)
                    continue;
                _EWL24_flag[sat1][sat2] = false;

                if (double_eq(itdd->srewl24, 0.0))
                    itdd->srewl24 = 0.05;

                if (abs(itdd->rewl24 - round(itdd->rewl24)) < _map_EWL_decision["maxdev"])
                {
                    itdd->isEwl24Fixed = true;
                    _EWL24_flag[sat1][sat2] = true;
                    itdd->iewl24 = round(itdd->rewl24);
                }
            }
            else if (mode == "EWL25")
            {
                if (itdd->rewl25 == 0)
                    continue;
                _EWL25_flag[sat1][sat2] = false;

                if (double_eq(itdd->srewl25, 0.0))
                    itdd->srewl25 = 0.05;

                if (abs(itdd->rewl25 - round(itdd->rewl25)) < _map_EWL_decision["maxdev"])
                {
                    itdd->isEwl25Fixed = true;
                    _EWL25_flag[sat1][sat2] = true;
                    itdd->iewl25 = round(itdd->rewl25);
                }
            }
        }
        return true;
    }


    // _selectAmb: 选择独立的双差模糊度组合（DD Ambiguities）
 // 参数:
 //   korder - 模糊度类型顺序，1: 窄巷(NL)，2: 宽巷(WL)，3: 扩展宽巷(EWL)，4: EWL24，5: EWL25
 //   namb - 当前模糊度数量
 // 返回值:
 //   返回独立的模糊度数量（ndef），失败返回-1
 // 功能说明:
 //   先按持续时间对所有双差模糊度项降序排序，优先选择持续时间长的模糊度。
 //   根据korder过滤出对应类型已固定的模糊度，然后调用_checkAmbDepend检查模糊度之间的线性独立性。
 //   若模糊度线性相关则剔除，否则保留。
 //   并在剔除时更新对应的固定标志（如_WL_flag等）。
    int gnss_ambiguity::_selectAmb(int korder, int namb)
    {
        int ndef = 0;          // 独立模糊度计数
        bool is_depend;        // 线性依赖标志

        // 按双差模糊度持续时间从大到小排序，持续时间长的优先
        sort(_DD.begin(), _DD.end(), _ddCompare);

        // 遍历所有双差模糊度
        for (auto itdd = _DD.begin(); itdd != _DD.end();)
        {
            // 根据模糊度类型过滤未固定项
            if (korder == 1 && !itdd->isNlFixed)
            {
                itdd = _DD.erase(itdd);
                continue;
            }
            if (korder == 2 && !itdd->isWlFixed)
            {
                itdd = _DD.erase(itdd);
                continue;
            }
            if (korder == 3 && !itdd->isEwlFixed)
            {
                itdd = _DD.erase(itdd);
                continue;
            }
            if (korder == 4 && !itdd->isEwl24Fixed)
            {
                itdd = _DD.erase(itdd);
                continue;
            }
            if (korder == 5 && !itdd->isEwl25Fixed)
            {
                itdd = _DD.erase(itdd);
                continue;
            }

            // 获取该双差模糊度涉及的两个卫星的索引
            int ipt2ow[2];
            ipt2ow[0] = get<2>(itdd->ddSats[0]);
            ipt2ow[1] = get<2>(itdd->ddSats[1]);

            // 调用_checkAmbDepend检测当前模糊度是否与已选模糊度线性相关
            is_depend = _checkAmbDepend(_is_first, namb, &ndef, 2, ipt2ow, 0, 0);

            if (is_depend)
            {
                // 若相关则剔除，并清除对应固定标志
                auto sat1 = get<0>(itdd->ddSats[0]);
                auto sat2 = get<0>(itdd->ddSats[1]);
                switch (korder)
                {
                case 2:
                    _WL_flag[sat1][sat2][itdd->site] = false;
                    break;
                case 3:
                    _EWL_flag[sat1][sat2][itdd->site] = false;
                    break;
                case 4:
                    _EWL24_flag[sat1][sat2] = false;
                    break;
                case 5:
                    _EWL25_flag[sat1][sat2] = false;
                    break;
                default:
                    break;
                }
                itdd = _DD.erase(itdd);
                continue;
            }
            else
            {
                // 不相关则保留，继续检查下一个
                itdd++;
            }
        }

        // 返回选中的独立模糊度数目，异常时返回-1
        if (ndef > 0 && ndef <= 999999)
            return ndef;
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_selectAmb] : _selectAmb Wrong");
            return -1;
        }
    }


    bool gnss_ambiguity::_prepareCovariance(gnss_amb_cmn *amb_cmn, Symmetric &covariance, std::vector<double> &value)
    {
        int row = 0, col = 0;
        // std::set covariance-matrix
        covariance.resize(_DD.size());
        covariance.setZero();
        for (auto itdd1 = _DD.begin(); itdd1 != _DD.end(); itdd1++)
        {
            // Row of covariance-matrix

            value.push_back(itdd1->rnl);
            row = distance(_DD.begin(), itdd1);

            for (auto itdd2 = itdd1; itdd2 != _DD.end(); itdd2++)
            {

                // Column of covariance-matrix
                col = distance(_DD.begin(), itdd2);

                Matrix Q(2, 2);
                // Covariance of ambiguity between four satellites
                Q(0, 0) = amb_cmn->Qx()(get<1>(itdd1->ddSats[0]), get<1>(itdd2->ddSats[0]));
                Q(0, 1) = amb_cmn->Qx()(get<1>(itdd1->ddSats[0]), get<1>(itdd2->ddSats[1]));
                Q(1, 0) = amb_cmn->Qx()(get<1>(itdd1->ddSats[1]), get<1>(itdd2->ddSats[0]));
                Q(1, 1) = amb_cmn->Qx()(get<1>(itdd1->ddSats[1]), get<1>(itdd2->ddSats[1]));

                // Combinatorial transformation
                auto tmp = (Q(0, 0) - Q(1, 0) - Q(0, 1) + Q(1, 1)) / itdd1->factor / itdd2->factor;
                covariance.set(tmp, row, col); // unit [cycle]
                                                                                                                //covariance(col, row) = covariance(row, col);// ???
            }
        }
        // Unit weight

        if (value.size() == 0 || covariance.size() == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_prepareCovariance] : prepare Double-Difference covariance is Wrong");
            return false;
        }
        else
            return true;
    }

    bool gnss_ambiguity::_prepareCovarianceWL(gnss_amb_cmn *amb_cmn, Symmetric &covariance, std::vector<double> &value, std::string mode)
    {
        int row = 0, col = 0;
        double lambda_1 = 0.0, lambda_2 = 0.0;
        Vector op_dd(4);
        // std::set covariance-matrix
        covariance.resize(_DD.size());
        covariance.setZero();

        for (auto itdd1 = _DD.begin(); itdd1 != _DD.end(); itdd1++)
        {
            std::string sat = get<0>(itdd1->ddSats[0]);
            if (mode == "WL")
            {
                if (itdd1->rwl == 0)
                    continue;
                if (sat.substr(0, 1) != "R")
                {
                    lambda_1 = _sys_wavelen[sat.substr(0, 1)]["L1"];
                    lambda_2 = _sys_wavelen[sat.substr(0, 1)]["L2"];
                }
                else
                {
                    lambda_1 = _sys_wavelen[sat]["L1"];
                    lambda_2 = _sys_wavelen[sat]["L2"];
                }

                value.push_back(itdd1->rwl);
            }
            else if (mode == "EWL")
            {
                if (itdd1->rewl == 0)
                    continue;
                if (sat.substr(0, 1) != "R")
                {
                    lambda_1 = _sys_wavelen[sat.substr(0, 1)]["L2"];
                    lambda_2 = _sys_wavelen[sat.substr(0, 1)]["L3"];
                }
                else
                {
                    lambda_1 = _sys_wavelen[sat]["L2"];
                    lambda_2 = _sys_wavelen[sat]["L3"];
                }

                value.push_back(itdd1->rewl);
            }

            // Row of covariance-matrix

            row = distance(_DD.begin(), itdd1);

            for (auto itdd2 = itdd1; itdd2 != _DD.end(); itdd2++)
            {
                if (get<0>(itdd1->ddSats[0]).substr(1) != get<0>(itdd2->ddSats[0]).substr(1))
                    continue;
                // Column of covariance-matrix
                col = distance(_DD.begin(), itdd2);
                op_dd << 1 / (lambda_1 * lambda_1), -1 / (lambda_1 * lambda_2), -1 / (lambda_1 * lambda_2), 1 / (lambda_2 * lambda_2);
                Matrix Q(2, 2);

                // Covariance of ambiguity between four satellites
                Q(0, 0) = amb_cmn->Qx()(get<1>(itdd1->ddSats[0]), get<1>(itdd2->ddSats[0])) * op_dd(1) + amb_cmn->Qx()(get<1>(itdd1->ddSats[0]), get<1>(itdd2->ddSats[2])) * op_dd(2) + amb_cmn->Qx()(get<1>(itdd1->ddSats[2]), get<1>(itdd2->ddSats[0])) * op_dd(3) + amb_cmn->Qx()(get<1>(itdd1->ddSats[2]), get<1>(itdd2->ddSats[2])) * op_dd(4);
                Q(0, 1) = amb_cmn->Qx()(get<1>(itdd1->ddSats[0]), get<1>(itdd2->ddSats[1])) * op_dd(1) + amb_cmn->Qx()(get<1>(itdd1->ddSats[0]), get<1>(itdd2->ddSats[3])) * op_dd(2) + amb_cmn->Qx()(get<1>(itdd1->ddSats[2]), get<1>(itdd2->ddSats[1])) * op_dd(3) + amb_cmn->Qx()(get<1>(itdd1->ddSats[2]), get<1>(itdd2->ddSats[3])) * op_dd(4);
                Q(1, 0) = amb_cmn->Qx()(get<1>(itdd1->ddSats[1]), get<1>(itdd2->ddSats[0])) * op_dd(1) + amb_cmn->Qx()(get<1>(itdd1->ddSats[1]), get<1>(itdd2->ddSats[2])) * op_dd(2) + amb_cmn->Qx()(get<1>(itdd1->ddSats[3]), get<1>(itdd2->ddSats[0])) * op_dd(3) + amb_cmn->Qx()(get<1>(itdd1->ddSats[3]), get<1>(itdd2->ddSats[2])) * op_dd(4);
                Q(1, 1) = amb_cmn->Qx()(get<1>(itdd1->ddSats[1]), get<1>(itdd2->ddSats[1])) * op_dd(1) + amb_cmn->Qx()(get<1>(itdd1->ddSats[1]), get<1>(itdd2->ddSats[3])) * op_dd(2) + amb_cmn->Qx()(get<1>(itdd1->ddSats[3]), get<1>(itdd2->ddSats[1])) * op_dd(3) + amb_cmn->Qx()(get<1>(itdd1->ddSats[3]), get<1>(itdd2->ddSats[3])) * op_dd(4);

                // Combinatorial transformation
                auto tmp = Q(0, 0) - Q(1, 0) - Q(0, 1) + Q(1, 1);
                covariance.set(tmp, row, col);  // unit [cycle]
            }
        }
        // Unit weight
        covariance.matrixW() = covariance.matrixR() * pow(amb_cmn->sigma0(), 2);

        if (value.size() == 0 || covariance.size() == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR[gnss_ambiguity::_prepareCovariance] : prepare Double-Difference covariance is Wrong");
            return false;
        }
        else
            return true;
    }

    double gnss_ambiguity::_lambdaSearch(const Matrix& anor, const std::vector<double> &fltpar, std::vector<int> &ibias, double *boot)
    {
        const int maxcan = 2;
        int namb = fltpar.size();
        int ncan = 0, ipos = 0;
        double ratio = 0.0;
        int i, j;
        double disall[maxcan] = {0};
        double *fbias = new double[namb]{0};
        double *Q = new double[namb * namb]{0};
        double *cands = new double[namb * maxcan]{0};
        gnss_amb_lambda *lambda = new gnss_amb_lambda();

        try
        {
            //row
            for (i = 0; i < namb; i++)
            {
                ibias.push_back(round(fltpar[i]));
                fbias[i] = fltpar[i] - ibias[i];

                //col
                for (j = 0; j < namb; j++)
                {
                    Q[i * namb + j] = anor(i, j);
                    if (i < j)
                        Q[i * namb + j] = 0.0;
                }
            }

            for (i = 0; i < maxcan; i++)
            {
                disall[i] = 0.0;
            }
            lambda->LAMBDA4(maxcan, namb, Q, fbias, &ncan, &ipos, cands, disall, boot);

            if (double_eq(disall[1], 0.0))
            {
                ratio = disall[0];
            }
            else
            {
                ratio = disall[1] / disall[0];
            }

            if (!double_eq(ratio, 0.0) && _part_fix && lambda->pDia != NULL)
            {
                _mDia.resize(namb);
                _mDia.setZero();
                for (i = 0; i < namb; i++)
                {
                    _mDia(i + 1) = lambda->pDia[i];
                }
            }

#ifdef DEBUG
            std::cout << " #integer combination and RATIO: " << std::setw(5) << ncan << std::fixed << std::setw(10) << std::setprecision(1) << ratio << std::endl;
            for (i = 0; i < maxcan; i++)
            {
                std::cout << std::fixed << std::setw(15) << std::setprecision(2) << disall[i];
            }
            std::cout << std::endl;
#endif
            if (ratio < _ratio || *boot < _boot)
                ibias.clear();
            else
            {
                for (i = 0; i < namb; i++)
                {
#ifdef DEBUG
                    std::cout << std::fixed << std::setw(15) << std::setprecision(2) << fltpar[i]
                         << std::setw(15) << std::setprecision(2) << ibias[i] * 1.0;
                    for (j = 0; j < maxcan; j++)
                    {
                        std::cout << std::fixed << std::setw(15) << std::setprecision(2) << cands[i * maxcan + j];
                    }
                    std::cout << std::endl;
#endif
                    ibias[i] += round(cands[i * maxcan + 0]);
                }
            }
            delete[] fbias;
            fbias = NULL;
            delete[] Q;
            Q = NULL;
            delete[] cands;
            cands = NULL;
            delete lambda;
            lambda = NULL;

            return ratio;
        }
        catch (...)
        {

            if (fbias != NULL)
            {
                delete[] fbias;
                fbias = NULL;
            }
            if (Q != NULL)
            {
                delete[] Q;
                Q = NULL;
            }
            if (cands != NULL)
            {
                delete[] cands;
                cands = NULL;
            }
            if (lambda != NULL)
            {
                delete lambda;
                lambda = NULL;
            }
            //LX changed: if no clear, EWL/WL will influence the NL AR
            ibias.clear();
            return 0.0;
        }
    }

    bool gnss_ambiguity::_ambSolve(gnss_amb_cmn* amb_cmn, std::vector<int>& fixed_amb, std::string mode)
    {
        Symmetric covariance; // 定义协方差矩阵，用于存储模糊度的协方差信息
        std::vector<double> value;       // 定义浮点模糊度值向量，存储初始的浮点解
        double ratio = 0.0;         // 定义Ratio值，用于检验整数解的可靠性
        double boot = 0.0;          // 定义Bootstrapping值，辅助评估解的可靠性
        int index;                  // 定义索引变量，用于跟踪当前处理的模糊度位置

        // 检查候选模糊度数量是否足够进行LAMBDA搜索
        // _full_fix_num通常设置为2或3，表示进行可靠搜索所需的最小模糊度数量
        if (_DD.size() < _full_fix_num)
        {
            // 遍历所有双差模糊度候选
            for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
            {
                std::string sat1 = get<0>(itdd->ddSats[0]); // 获取第一颗卫星标识
                std::string sat2 = get<0>(itdd->ddSats[1]); // 获取第二颗卫星标识
                // 根据不同模式更新相应的宽巷或扩展宽巷标志和值
                if (mode == "WL" && _WL_flag[sat1][sat2][itdd->site])
                    _IWL[sat1][sat2][itdd->site] = itdd->iwl;
                if (mode == "EWL" && _EWL_flag[sat1][sat2][itdd->site])
                    _IEWL[sat1][sat2][itdd->site] = itdd->iewl;
                if (mode == "EWL24" && _EWL24_flag[sat1][sat2])
                    _IEWL24[sat1][sat2] = itdd->iewl24;
                continue;
            }
            // 输出警告信息，提示候选模糊度数量不足
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_ambSolve] : too few candidate ambiguities for LAMBDA Search");
            return false; // 返回失败
        }

        // 根据模式准备协方差矩阵和浮点模糊度值
        if (mode == "NL")
        {
            // 调用函数准备NL模式下的协方差矩阵和浮点模糊度值
            if (!_prepareCovariance(amb_cmn, covariance, value))
                return false;
        }
        else
        {
            // 调用函数准备WL或EWL模式下的协方差矩阵和浮点模糊度值
            if (!_prepareCovarianceWL(amb_cmn, covariance, value, mode))
                return false;
        }

        // 使用LAMBDA方法解决整数模糊度
        // _lambdaSearch函数执行LAMBDA算法，尝试将浮点解转换为整数解
        ratio = _lambdaSearch(covariance.matrixR(), value, fixed_amb, &boot);
        // 对Ratio值设置上限，防止过大值影响判断
        if (ratio > 99.0)
            ratio = 99.0;
        // 记录NL模式下的Ratio和Bootstrapping值，用于后续分析
        if (mode == "NL")
        {
            _os_ratio << "RATIO" << std::fixed << std::setw(10) << std::setprecision(2) << ratio;
            _os_boot << "BOOTSTRAPPING" << std::fixed << std::setw(10) << std::setprecision(2) << boot;
        }

        // 部分模糊度固定循环
        // 当Ratio值或Bootstrapping值不满足条件，且允许部分固定时，进入循环
        int newamb = 0;
        while ((ratio < _ratio || (boot < _boot && mode == "NL")) && _part_fix)
        {
            double max_diag = 0.0; // 用于记录最大方差
            index = 1;             // 初始化索引

            // 寻找方差最大的模糊度候选进行排除
            if (newamb < 2)
            {
                for (unsigned int i = 0; i < value.size(); i++)
                {
                    auto itdd = _DD.begin() + i; // 获取当前模糊度候选迭代器
                    // 检查当前模糊度候选是否满足固定条件
                    if (_fix_epo_num[mode][get<0>(itdd->ddSats[0])] == 0 ||
                        _fix_epo_num[mode][get<0>(itdd->ddSats[1])] == 0 ||
                        _lock_epo_num[get<0>(itdd->ddSats[0])] < 5 ||
                        _lock_epo_num[get<0>(itdd->ddSats[1])] < 5)
                    {
                        // 更新最大方差和对应索引
                        if (covariance(i, i) > max_diag)
                        {
                            max_diag = covariance(i, i);
                            index = i;
                        }
                    }
                }
                newamb++;
            }

            // 如果未找到满足条件的候选，重新遍历所有候选寻找最大方差
            if (max_diag == 0.0)
            {
                for (unsigned int i = 0; i < value.size(); i++)
                {
                    if (covariance(i, i) > max_diag)
                    {
                        max_diag = covariance(i, i);
                        index = i;
                    }
                }
            }

            // 获取当前要排除的模糊度候选
            auto itdd_tmp = _DD.begin() + index;

            // 调试信息输出，显示被排除的模糊度候选详情
#ifdef DEBUG_pppRTK
            std::cout << std::fixed << "Erase DD  " << get<0>(itdd_tmp->ddSats[0]) << "  " << get<0>(itdd_tmp->ddSats[1]) << "   "
                << std::setw(17) << std::setprecision(4) << itdd_tmp->rwl << std::setw(17) << std::setprecision(4) << itdd_tmp->srwl << std::setw(6) << (int)itdd_tmp->isWlFixed
                << std::setw(17) << std::setprecision(4) << itdd_tmp->rnl << std::setw(17) << std::setprecision(4) << itdd_tmp->srnl << std::setw(6) << (int)itdd_tmp->isNlFixed << std::endl;
#endif

            // 根据模式更新相应的宽巷或扩展宽巷标志
            if (mode == "WL")
            {
                _WL_flag[get<0>(itdd_tmp->ddSats[0])][get<0>(itdd_tmp->ddSats[1])][itdd_tmp->site] = false;
            }
            else if (mode == "EWL")
            {
                _EWL_flag[get<0>(itdd_tmp->ddSats[0])][get<0>(itdd_tmp->ddSats[1])][itdd_tmp->site] = false;
            }

            // 从协方差矩阵中移除对应行和列
            covariance.Matrix_remRC(index, index);
            // 从浮点模糊度值向量中移除对应元素
            value.erase(value.begin() + index);
            // 从双差模糊度候选集中移除对应元素
            _DD.erase(_DD.begin() + index);

            // 清空当前固定模糊度向量，准备下一轮搜索
            fixed_amb.clear();
            // 检查剩余模糊度数量是否满足部分固定条件
            if (value.size() <= _part_fix_num)
                return false;

            // 重新进行LAMBDA搜索，尝试得到新的整数解
            ratio = _lambdaSearch(covariance.matrixR(), value, fixed_amb, &boot);

            // 再次对Ratio值设置上限
            if (ratio > 99.0)
                ratio = 99.0;
            // 更新NL模式下的Ratio和Bootstrapping记录
            if (mode == "NL")
            {
                _os_ratio << std::setw(10) << std::setprecision(2) << ratio;
                _os_boot << std::setw(10) << std::setprecision(2) << boot;
            }
            }

        // 当NL模式下的解满足条件时，记录Ratio和Bootstrapping值
        if (mode == "NL" && ratio > _ratio && boot > _boot)
        {
            _os_ratio << std::endl; // 换行记录Ratio值
            _os_boot << std::endl;  // 换行记录Bootstrapping值
            _writeRatio(ratio); // 写入Ratio值到日志或文件
            if (_boot > 0.0)
                _writeBoot(boot); // 如果Bootstrapping值有效，写入到日志或文件
        }

        // 将当前Ratio和Bootstrapping值传递给公共数据对象
        amb_cmn->set_ratio(ratio);
        amb_cmn->set_boot(boot);

        // 根据不同模式更新固定模糊度值
        if (mode == "WL")
        {
            // 检查是否成功得到固定模糊度解
            if (fixed_amb.size() != 0)
            {
                // 遍历所有双差模糊度候选，更新宽巷模糊度固定值
                for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
                {
                    if (it_dd->rwl == 0) // 跳过已固定的宽巷模糊度
                        continue;
                    int ipos = distance(_DD.begin(), it_dd); // 获取当前候选的索引
                    it_dd->iwl = fixed_amb[ipos]; // 更新宽巷模糊度固定值
                }
                return true; // 返回成功
            }
            else
            {
                // 输出警告信息，提示未能得到候选固定模糊度解
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_ambSolve] : LAMBDA Search can't get candidate std::fixed ambiguities");
                return false; // 返回失败
            }
        }
        else if (mode == "EWL")
        {
            if (fixed_amb.size() != 0)
            {
                for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
                {
                    if (it_dd->rewl == 0)
                        continue;
                    int ipos = distance(_DD.begin(), it_dd);
                    it_dd->iewl = fixed_amb[ipos];
                }
                return true;
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_ambSolve] : LAMBDA Search can't get candidate std::fixed ambiguities");
                return false;
            }
        }
        else if (mode == "NL")
        {
            if (fixed_amb.size() != 0)
            {
                for (auto it_dd = _DD.begin(); it_dd != _DD.end(); it_dd++)
                {
                    int ipos = distance(_DD.begin(), it_dd);
                    it_dd->inl = fixed_amb[ipos];
                }
                return true;
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_ambiguity::_ambSolve] : LAMBDA Search can't get candidate std::fixed ambiguities");
                return false;
            }
        }
        else
        {
            // 输出错误信息，提示未知的工作模式
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "Error[gnss_ambiguity::_ambSolve] : Unknown Combination[NL/WL/EWL] : " + mode);
            return false;
        }
    }

    bool gnss_ambiguity::_addFixConstraint(gnss_proc_flt* gflt)
    {
        //////========================= Virtual observation equation ===========================================
        double dl = 0, flt = 0, integer = 0, Ba = 1, Bb = -1;
        double p0 = 1E9; // 设置虚拟观测方程的权重，权重较大表示对约束的信任度高
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            // 判断宽巷模糊度是否固定，根据不同的观测组合类型决定是否跳过当前迭代
            bool wl_fix = ((_obstype == OBSCOMBIN::RAW_MIX && itdd->isSngleFreq != true) || (_obstype != OBSCOMBIN::RAW_MIX && _obstype != OBSCOMBIN::RAW_SINGLE));
            if ((!itdd->isWlFixed && wl_fix) || !itdd->isNlFixed)
                continue;

            int index_sat1 = get<1>(itdd->ddSats[0]); // 获取第一颗卫星的索引
            int index_sat2 = get<1>(itdd->ddSats[1]); // 获取第二颗卫星的索引

            // 计算双差浮点模糊度解
            flt = gflt->param()[index_sat1 - 1].value() - gflt->param()[index_sat2 - 1].value();

            // 根据不同的观测组合类型计算虚拟观测方程的整数部分和系数
            if (_obstype == OBSCOMBIN::IONO_FREE)
            {
                std::string sys = get<0>(itdd->ddSats[0]).substr(0, 1); // 获取卫星系统标识（如 GPS、GLONASS 等）
                if (sys != "R") // 如果不是 GLONASS 系统
                {
                    integer = (itdd->inl - itdd->sd_rnl_cor + round(itdd->rwl) * _sys_wavelen[sys]["WL"] / _sys_wavelen[sys]["L2"]) * itdd->factor;
                }
                else // 处理 GLONASS 系统的特殊情形
                {
                    std::string sat1 = get<0>(itdd->ddSats[0]);
                    std::string sat2 = get<0>(itdd->ddSats[1]);
                    integer = (itdd->inl - itdd->sd_rnl_cor + 3.5 * round(itdd->rwl)); // 特殊的整数模糊度计算方式
                    Ba = 1 / _sys_wavelen[sat1]["NL"];                                 // 设置系数 Ba
                    Bb = -1 / _sys_wavelen[sat2]["NL"];                                // 设置系数 Bb
                    flt = gflt->param()[index_sat1 - 1].value() * Ba + gflt->param()[index_sat2 - 1].value() * Bb; // 重新计算浮点解
                }
            }
            else if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_SINGLE || _obstype == OBSCOMBIN::RAW_MIX)
            {
                double Lx_cor = 0.0; // 用于存储 L1、L2 或 L3 的修正值
                std::string sys = get<0>(itdd->ddSats[0]).substr(0, 1);
                if (sys != "R")
                {
                    if (itdd->ambtype == "AMB_L1")
                        Lx_cor = itdd->sd_r1_cor;
                    else if (itdd->ambtype == "AMB_L2")
                        Lx_cor = itdd->sd_r2_cor;
                    else if (itdd->ambtype == "AMB_L3")
                        Lx_cor = itdd->sd_r3_cor;
                    integer = (itdd->inl - Lx_cor) * itdd->factor; // 计算整数部分
                }
                else
                {
                    std::string sat1 = get<0>(itdd->ddSats[0]);
                    std::string sat2 = get<0>(itdd->ddSats[1]);
                    if (itdd->ambtype == "AMB_L1")
                    {
                        Ba = 1 / _sys_wavelen[sat1]["L1"]; // 设置 L1 的系数 Ba
                        Bb = -1 / _sys_wavelen[sat2]["L1"]; // 设置 L1 的系数 Bb
                        Lx_cor = itdd->sd_r1_cor;
                    }
                    else if (itdd->ambtype == "AMB_L2")
                    {
                        Ba = 1 / _sys_wavelen[sat1]["L2"]; // 设置 L2 的系数 Ba
                        Bb = -1 / _sys_wavelen[sat2]["L2"]; // 设置 L2 的系数 Bb
                        Lx_cor = itdd->sd_r2_cor;
                    }
                    flt = gflt->param()[index_sat1 - 1].value() * Ba + gflt->param()[index_sat2 - 1].value() * Bb; // 重新计算浮点解
                    integer = (itdd->inl - Lx_cor); // 计算整数部分
                }
            }

            dl = integer - flt; // 计算虚拟观测方程的残差

            std::vector<std::pair<int, double>> B; // 定义系数矩阵 B
            B.push_back(std::make_pair(index_sat1, Ba)); // 添加第一个卫星的系数
            B.push_back(std::make_pair(index_sat2, Bb)); // 添加第二个卫星的系数

            Matrix B_mat;               // 定义 B 矩阵
            Symmetric P_mat;      // 定义权矩阵 P
            Vector l_mat;         // 定义观测向量 l
            gnss_proc_lsq_equationmatrix virtual_equ; // 定义虚拟观测方程对象

            // 添加虚拟观测方程
            virtual_equ.add_equ(B, p0, dl, _site, get<0>(itdd->ddSats[0]) + "_" + get<0>(itdd->ddSats[1]), gnss_data_obscombtype(), false);
            virtual_equ.chageNewMat(B_mat, P_mat, l_mat, gflt->npar_number()); // 转换为新的矩阵形式

            gflt->resetQ(); // 重置滤波器的 Q 矩阵
            gflt->add_virtual_obs(B_mat, P_mat, l_mat); // 将虚拟观测方程添加到滤波器中
        }

        try
        {
            gflt->update(); // 尝试更新滤波器状态
        }
        catch (exception e)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, e.what(), "Ambiguity Constrain Failed!"); // 捕获异常并记录错误信息
            return false; // 返回失败
        }

        return true; // 返回成功
        //////========================= Virtual observation equation ===========================================
    }

    // _addFixConstraint_before(): 为HOLD固定模式添加历史固定约束
// gflt: 当前滤波器对象，添加约束到其虚拟观测方程组中
    bool gnss_ambiguity::_addFixConstraint_before(gnss_proc_flt* gflt)
    {
        double dl, flt, integer;
        double p0 = 1E9; // 观测权重（虚拟观测给较大值）
        int index_sat1, index_sat2;

        // 获取当前估计器所有参数（浮点解）
        std::vector<base_par> params_all = gflt->param().getAllPar();

        for (auto itdd = _DD_save.begin(); itdd != _DD_save.end(); itdd++)
        {
            // 判断是否满足加入约束的条件（如模糊度已成功固定、非单频）
            bool wl_fix = ((_obstype == OBSCOMBIN::RAW_MIX && itdd->isSngleFreq != true) || (_obstype != OBSCOMBIN::RAW_MIX && _obstype != OBSCOMBIN::RAW_SINGLE));
            if ((!itdd->isWlFixed && wl_fix) || !itdd->isNlFixed || itdd->fix_epoch <= 10)
                continue; // 跳过不满足条件的历史记录

            // 初始化参数索引
            index_sat1 = 0, index_sat2 = 0;

            // 在参数列表中找到两个对应卫星的模糊度参数索引
            for (auto it_par = params_all.begin(); it_par != params_all.end(); it_par++)
            {
                it_par->index = distance(params_all.begin(), it_par) + 1;
                if (index_sat1 == 0 &&
                    it_par->str_type().find("AMB") != std::string::npos &&
                    it_par->prn == get<0>(itdd->ddSats[0]) &&
                    !double_eq(it_par->value(), 0.0) &&
                    (itdd->beg_epo >= it_par->beg && itdd->end_epo <= it_par->end))
                {
                    index_sat1 = distance(params_all.begin(), it_par) + 1;
                }

                if (index_sat2 == 0 &&
                    it_par->str_type().find("AMB") != std::string::npos &&
                    it_par->prn == get<0>(itdd->ddSats[1]) &&
                    !double_eq(it_par->value(), 0.0) &&
                    (itdd->beg_epo >= it_par->beg && itdd->end_epo <= it_par->end))
                {
                    index_sat2 = distance(params_all.begin(), it_par) + 1;
                }
            }

            if (index_sat1 == 0 || index_sat2 == 0)
                continue; // 如果有一个未找到参数索引，跳过

            // 获取当前估计的模糊度差值
            flt = gflt->param()[index_sat1 - 1].value() - gflt->param()[index_sat2 - 1].value();

            // 计算整数模糊度值（历史固定值）
            if (_obstype == OBSCOMBIN::IONO_FREE)
            {
                std::string sys = get<0>(itdd->ddSats[0]).substr(0, 1);
                integer = (itdd->inl - itdd->sd_rnl_cor + round(itdd->rwl) * _sys_wavelen[sys]["WL"] / _sys_wavelen[sys]["L2"]) * itdd->factor;
            }
            else if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_SINGLE || _obstype == OBSCOMBIN::RAW_MIX)
            {
                double Lx_cor = 0.0;
                if (itdd->ambtype == "AMB_L1") Lx_cor = itdd->sd_r1_cor;
                else if (itdd->ambtype == "AMB_L2") Lx_cor = itdd->sd_r2_cor;
                else if (itdd->ambtype == "AMB_L3") Lx_cor = itdd->sd_r3_cor;
                integer = (itdd->inl - Lx_cor) * itdd->factor;
            }

            // 构造等式观测值差值 dl = 整数值 - 浮点估计值
            dl = integer - flt;

            // 构造虚拟观测方程 B*x = l
            std::vector<std::pair<int, double>> B;
            B.push_back(std::make_pair(index_sat1, 1));
            B.push_back(std::make_pair(index_sat2, -1));

            Matrix B_mat;
            Symmetric P_mat;
            Vector l_mat;
            gnss_proc_lsq_equationmatrix virtual_equ;
            gnss_data_obscombtype type;

            // 将虚拟约束加入矩阵
            virtual_equ.add_equ(B, p0, dl, _site, get<0>(itdd->ddSats[0]) + "_" + get<0>(itdd->ddSats[1]), type, false);
            virtual_equ.chageNewMat(B_mat, P_mat, l_mat, gflt->npar_number());

            gflt->resetQ();
            gflt->add_virtual_obs(B_mat, P_mat, l_mat);
        }

        try {
            gflt->resetQ();
            gflt->update(); // 更新状态解，加入虚拟观测约束
        }
        catch (exception e) {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, e.what(), "Ambiguity Constrain Failed!");
            return false;
        }

        return true; // 所有约束成功添加
    }

    bool gnss_ambiguity::_addFixConstraint(gnss_proc_lsqbase *lsqestimator)
    {
        //////========================= Virtual observation equation ===========================================
        double dl, flt, integer, Ba = 1, Bb = -1;
        double p0 = 1E9;
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            bool wl_fix = ((_obstype == OBSCOMBIN::RAW_MIX && itdd->isSngleFreq != true) || (_obstype != OBSCOMBIN::RAW_MIX && _obstype != OBSCOMBIN::RAW_SINGLE));
            if ((!itdd->isWlFixed && wl_fix) || !itdd->isNlFixed)
                continue;
            int index_sat1 = get<1>(itdd->ddSats[0]);
            int index_sat2 = get<1>(itdd->ddSats[1]);

            flt = lsqestimator->_x_solve[index_sat1 - 1].value() - lsqestimator->_x_solve[index_sat2 - 1].value(); // Unit: meter

            if (_obstype == OBSCOMBIN::IONO_FREE)
            {
                std::string sys = get<0>(itdd->ddSats[0]).substr(0, 1);
                //integer = (itdd->inl + round(itdd->rwl) * _sys_wavelen[sys]["WL"] / _sys_wavelen[sys]["L2"]) * itdd->factor; //wrong
                //xjhan
                if (sys != "R")
                {
                    integer = (itdd->inl - itdd->sd_rnl_cor + round(itdd->rwl) * _sys_wavelen[sys]["WL"] / _sys_wavelen[sys]["L2"]) * itdd->factor;
                }
                else
                {
                    std::string sat1 = get<0>(itdd->ddSats[0]);
                    std::string sat2 = get<0>(itdd->ddSats[1]);
                    integer = (itdd->inl - itdd->sd_rnl_cor + 3.5 * round(itdd->rwl)); // hlgou changed according to LX
                    Ba = 1 / _sys_wavelen[sat1]["NL"];                                 // Coefficient of B matrix
                    Bb = -1 / _sys_wavelen[sat2]["NL"];
                    flt = lsqestimator->_x_solve[index_sat1 - 1].value() * Ba + lsqestimator->_x_solve[index_sat2 - 1].value() * Bb;
                }
            }
            else if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_SINGLE || _obstype == OBSCOMBIN::RAW_MIX)
            {
                //dl = itdd->inl * itdd->factor - flt;  //wrong
                double Lx_cor = 0.0;
                std::string sys = get<0>(itdd->ddSats[0]).substr(0, 1);
                if (sys != "R")
                {
                    if (itdd->ambtype == "AMB_L1")
                        Lx_cor = itdd->sd_r1_cor;
                    else if (itdd->ambtype == "AMB_L2")
                        Lx_cor = itdd->sd_r2_cor;
                    else if (itdd->ambtype == "AMB_L3")
                        Lx_cor = itdd->sd_r3_cor;
                    else if (itdd->ambtype == "AMB_L4")
                        Lx_cor = itdd->sd_r4_cor;
                    else if (itdd->ambtype == "AMB_L5")
                        Lx_cor = itdd->sd_r5_cor;
                    integer = (itdd->inl - Lx_cor) * itdd->factor;
                }
                else
                {
                    std::string sat1 = get<0>(itdd->ddSats[0]);
                    std::string sat2 = get<0>(itdd->ddSats[1]);
                    if (itdd->ambtype == "AMB_L1")
                    {
                        Ba = 1 / _sys_wavelen[sat1]["L1"]; // Coefficient of B matrix
                        Bb = -1 / _sys_wavelen[sat2]["L1"];
                        Lx_cor = itdd->sd_r1_cor;
                    }
                    else if (itdd->ambtype == "AMB_L2")
                    {
                        Ba = 1 / _sys_wavelen[sat1]["L2"]; // Coefficient of B matrix
                        Bb = -1 / _sys_wavelen[sat2]["L2"];
                        Lx_cor = itdd->sd_r2_cor;
                    }
                    flt = lsqestimator->_x_solve[index_sat1 - 1].value() * Ba + lsqestimator->_x_solve[index_sat2 - 1].value() * Bb;
                    integer = (itdd->inl - Lx_cor);
                }
            }
            else if (_obstype == OBSCOMBIN::WL_COMBIN)
            {
                integer = itdd->inl * itdd->factor;
            }

            dl = integer - flt;

            // jqwu for test
            //std::cout << "NLFixConstraint:  " <<  itdd->ambtype << "  " << std::setw(5) << get<0>(itdd->ddSats[0]) << std::setw(5) << get<0>(itdd->ddSats[1])
            //     << right << std::setw(17) << std::setprecision(4) << integer << std::setw(17) << std::setprecision(4) << flt << std::setw(17) << std::setprecision(4) << dl <<  std::endl;

            std::vector<std::pair<int, double>> B;
            B.push_back(std::make_pair(index_sat1, Ba));
            B.push_back(std::make_pair(index_sat2, Bb));

            gnss_proc_lsq_equationmatrix virtual_equ;
            gnss_data_obscombtype type;
            virtual_equ.add_equ(B, p0, dl, _site, get<0>(itdd->ddSats[0]) + "_" + get<0>(itdd->ddSats[1]), type, false);
            lsqestimator->add_equation(virtual_equ);
        }

        try
        {
            for (int i = 0; i < lsqestimator->NEQ().rows(); i++)
            {
                lsqestimator->_x_solve[i].apriori(0.0);
            }
            lsqestimator->solve_NEQ();
        }
        catch (exception e)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, e.what(), "Solve Ebase_quation Fail!");
            return false;
        }

        return true;
        //////========================= Virtual observation equation ===========================================
    }

    bool gnss_ambiguity::_addFixConstraintWL(gnss_proc_flt* gflt, std::string mode)
    {
        //////========================= Virtual observation equation ===========================================
        double dl = 0.0; // 虚拟观测方程的残差
        double integer = 0.0; // 模糊度的整数解
        double flt = 0.0; // 模糊度的浮点解
        double lambda_1 = 0.0; // 第一个频率的波长
        double lambda_2 = 0.0; // 第二个频率的波长
        double p0 = 1E6; // 虚拟观测方程的权重
        double lambda_11 = 0.0, lambda_21 = 0.0; // GLONASS卫星的第一个频率波长
        double lambda_12 = 0.0, lambda_22 = 0.0; // GLONASS卫星的第二个频率波长
        double Ba = 0.0, Bb = 0.0, Bc = 0.0, Bd = 0.0; // 虚拟观测方程的系数

        Symmetric _Qx_tmp; // 临时存储滤波器的协方差矩阵
        gnss_proc_flt flttmp(*gflt); // 滤波器的临时副本
        _Qx_tmp = gflt->Qx(); // 获取当前协方差矩阵

        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            int index_sat1 = get<1>(itdd->ddSats[0]); // 获取第一颗卫星的索引
            int index_sat2 = get<1>(itdd->ddSats[1]); // 获取第二颗卫星的索引
            int index_sat3 = get<1>(itdd->ddSats[2]); // 获取第三颗卫星的索引
            int index_sat4 = get<1>(itdd->ddSats[3]); // 获取第四颗卫星的索引

            std::string sat1 = get<0>(itdd->ddSats[0]); // 获取第一颗卫星的标识
            std::string sat2 = get<0>(itdd->ddSats[1]); // 获取第二颗卫星的标识
            std::string sat3 = get<0>(itdd->ddSats[2]); // 获取第三颗卫星的标识
            std::string sat4 = get<0>(itdd->ddSats[3]); // 获取第四颗卫星的标识

            if (sat1.substr(0, 1) != "R") // 检查是否为GLONASS系统
            {
                if (mode == "WL")
                {
                    if (!itdd->isWlFixed || itdd->rwl == 0) // 检查宽巷模糊度是否固定
                        continue;
                    // 获取L1和L2的波长
                    lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L1"];
                    lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    // 计算整数模糊度解
                    integer = (itdd->iwl - itdd->sd_rwl_cor);
                }
                else if (mode == "EWL")
                {
                    if (!itdd->isEwlFixed || itdd->rewl == 0) // 检查扩展宽巷模糊度是否固定
                        continue;
                    // 获取L2和L3的波长
                    lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L3"];
                    // 计算整数模糊度解
                    integer = (itdd->iewl - itdd->sd_rewl_cor);
                }
                // 计算系数
                Ba = 1 / lambda_1;
                Bb = -1 / lambda_2;
                Bc = -1 / lambda_1;
                Bd = 1 / lambda_2;
            }
            else // 处理GLONASS系统
            {
                if (mode == "WL")
                {
                    if (!itdd->isWlFixed || itdd->rwl == 0)
                        continue;
                    // 获取GLONASS卫星的波长
                    lambda_11 = _sys_wavelen[sat1]["L1"];
                    lambda_12 = _sys_wavelen[sat3]["L2"];
                    lambda_21 = _sys_wavelen[sat2]["L1"];
                    lambda_22 = _sys_wavelen[sat4]["L2"];
                    // 计算整数模糊度解
                    integer = (itdd->iwl - itdd->sd_rwl_cor);
                }
                // 计算系数
                Ba = 1 / lambda_11;
                Bb = -1 / lambda_12;
                Bc = -1 / lambda_21;
                Bd = 1 / lambda_22;
            }

            // 计算浮点解
            flt = gflt->param()[index_sat1 - 1].value() * Ba +
                gflt->param()[index_sat3 - 1].value() * Bb +
                gflt->param()[index_sat2 - 1].value() * Bc +
                gflt->param()[index_sat4 - 1].value() * Bd;

            // 计算残差
            dl = integer - flt;

            // 构建系数矩阵 B
            std::vector<std::pair<int, double>> B;
            B.push_back(std::make_pair(index_sat1, Ba));
            B.push_back(std::make_pair(index_sat3, Bb));
            B.push_back(std::make_pair(index_sat2, Bc));
            B.push_back(std::make_pair(index_sat4, Bd));

            // 构建虚拟观测方程
            Matrix B_mat;
            Symmetric P_mat;
            Vector l_mat;
            gnss_proc_lsq_equationmatrix virtual_equ;
            gnss_data_obscombtype type;

            // 添加虚拟观测方程
            virtual_equ.add_equ(B, p0, dl, _site,
                get<0>(itdd->ddSats[0]) + "_" + get<0>(itdd->ddSats[2]) + "_" +
                get<0>(itdd->ddSats[1]) + "_" + get<0>(itdd->ddSats[3]), type, false);
            virtual_equ.chageNewMat(B_mat, P_mat, l_mat, gflt->npar_number());

            gflt->resetQ(); // 重置滤波器的 Q 矩阵
            gflt->add_virtual_obs(B_mat, P_mat, l_mat); // 将虚拟观测方程添加到滤波器中
        }

        try
        {
            gflt->update(); // 更新滤波器状态
        }
        catch (exception e)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, e.what(), "Solve Ebase_quation Fail!"); // 记录异常信息
            return false; // 返回失败
        }

        return true; // 返回成功
        //////========================= Virtual observation equation ===========================================
    }

    bool gnss_ambiguity::_addFixConstraintWL(gnss_proc_lsqbase *lsqestimator, std::string mode)
    {
        //////========================= Virtual observation equation ===========================================
        gnss_proc_lsqbase lsqtmp(*lsqestimator);
        double dl= 0, integer= 0, flt= 0, lambda_1= 0, lambda_2= 0;
        double p0 = 1E6;
        double lambda_11= 0, lambda_21= 0, lambda_12= 0, lambda_22= 0; // the wave_length for glonass
        double Ba= 0, Bb= 0, Bc= 0, Bd= 0;                             // the Coefficient of B matrix
        if (mode == "EWL" || mode == "EWL24" || mode == "EWL25")
            p0 = 1E4;
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            int index_sat1 = get<1>(itdd->ddSats[0]);
            int index_sat2 = get<1>(itdd->ddSats[1]);
            int index_sat3 = get<1>(itdd->ddSats[2]);
            int index_sat4 = get<1>(itdd->ddSats[3]);
            std::string sat1 = get<0>(itdd->ddSats[0]);
            std::string sat2 = get<0>(itdd->ddSats[1]);
            std::string sat3 = get<0>(itdd->ddSats[2]);
            std::string sat4 = get<0>(itdd->ddSats[3]);

            if (sat1.substr(0, 1) != "R")
            {
                if (mode == "WL")
                {
                    if (!itdd->isWlFixed || itdd->rwl == 0)
                        continue;
                    //dl = (itdd->iwl - itdd->rwl);
                    lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L1"];
                    lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    //if(_apply_irc) integer = (itdd->iwl + itdd->sd_rwl_cor);
                    //else integer = (itdd->iwl - itdd->sd_rwl_cor);
                    integer = (itdd->iwl - itdd->sd_rwl_cor);
                }
                else if (mode == "EWL")
                {
                    if (!itdd->isEwlFixed || itdd->rewl == 0)
                        continue;
                    //dl = (itdd->iewl - itdd->rewl);
                    lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L3"];
                    integer = (itdd->iewl - itdd->sd_rewl_cor);
                }
                else if (mode == "EWL24")
                {
                    if (!itdd->isEwl24Fixed || itdd->rewl24 == 0)
                        continue;
                    //dl = (itdd->iewl - itdd->rewl);
                    lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L4"];
                    integer = (itdd->iewl24 - itdd->sd_rewl24_cor);
                }
                else if (mode == "EWL25")
                {
                    if (!itdd->isEwl25Fixed || itdd->rewl25 == 0)
                        continue;
                    //dl = (itdd->iewl - itdd->rewl);
                    lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                    lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L5"];
                    integer = (itdd->iewl25 - itdd->sd_rewl25_cor);
                }
                Ba = 1 / lambda_1;
                Bb = -1 / lambda_2;
                Bc = -1 / lambda_1;
                Bd = 1 / lambda_2;
            }
            else
            { // added by hlgou 2022.1.11
                if (mode == "WL")
                {
                    if (!itdd->isWlFixed || itdd->rwl == 0)
                        continue;
                    lambda_11 = _sys_wavelen[sat1]["L1"];
                    lambda_12 = _sys_wavelen[sat3]["L2"];
                    lambda_21 = _sys_wavelen[sat2]["L1"];
                    lambda_22 = _sys_wavelen[sat4]["L2"];

                    integer = (itdd->iwl - itdd->sd_rwl_cor);
                }
                Ba = 1 / lambda_11;
                Bb = -1 / lambda_12;
                Bc = -1 / lambda_21;
                Bd = 1 / lambda_22;
            }
            //        std::cerr << sat1<<lsqestimator->_x_solve[index_sat1 - 1].value() << "        " << sat3<< lsqestimator->_x_solve[index_sat3 - 1].value()<<std::endl;

            flt = lsqestimator->_x_solve[index_sat1 - 1].value() * Ba + lsqestimator->_x_solve[index_sat3 - 1].value() * Bb + lsqestimator->_x_solve[index_sat2 - 1].value() * Bc + lsqestimator->_x_solve[index_sat4 - 1].value() * Bd;
            dl = integer - flt;

            // jqwu for test
            //std::cout << "WLFixConstraint:  " << mode << "  " << std::setw(5) << get<0>(itdd->ddSats[0]) << std::setw(5) << get<0>(itdd->ddSats[1])
            //    << right << std::setw(17) << std::setprecision(4) << integer << std::setw(17) << std::setprecision(4) << flt << std::setw(17) << std::setprecision(4) << dl << std::endl;

            std::vector<std::pair<int, double>> B;
            B.push_back(std::make_pair(index_sat1, Ba));
            B.push_back(std::make_pair(index_sat3, Bb));
            B.push_back(std::make_pair(index_sat2, Bc));
            B.push_back(std::make_pair(index_sat4, Bd));

            gnss_proc_lsq_equationmatrix virtual_equ;
            gnss_data_obscombtype type;
            virtual_equ.add_equ(B, p0, dl, _site, get<0>(itdd->ddSats[0]) + "_" + get<0>(itdd->ddSats[2]) + "_" + get<0>(itdd->ddSats[1]) + "_" + get<0>(itdd->ddSats[3]), type, false);
            lsqestimator->add_equation(virtual_equ);
            //lsqtmp.add_equation(virtual_equ);
        }
        try
        {
            for (int i = 0; i < lsqestimator->NEQ().rows(); i++)
            {
                lsqestimator->_x_solve[i].apriori(0.0);
            }
            lsqestimator->solve_NEQ();

            // jqwu for test
            //if (mode == "WL") std::cout << "After  WL fix:" << std::endl;
            //else              std::cout << "After EWL fix:" << std::endl;
            //for (int i = 0; i < lsqestimator->NEQ().rows(); i++) {
            //    std::cout << std::fixed << std::setw(20) << "EPO  " << std::setw(20) << lsqestimator->_x_solve[i].str_type() + "  " <<
            //        std::setw(20) << std::setprecision(5) << lsqestimator->_x_solve[i]._value() <<
            //        std::setw(15) << std::setprecision(5) << lsqestimator->dx()(i + 1) <<
            //        std::setw(20) << std::setprecision(5) << lsqestimator->_x_solve[i]._value() + lsqestimator->dx()(i + 1) <<
            //        std::setw(12) << std::setprecision(5) << lsqestimator->stdx(i + 1) << std::endl;
            //}
        }
        catch (exception e)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, e.what(), "Solve Ebase_quation Fail!");
            return false;
        }

        return true;
        //////========================= Virtual observation equation ===========================================
    }

    void gnss_ambiguity::_initRatiofile()
    {
        //output ratio   file
        std::string ratio_path = dynamic_cast<set_out *>(_gset)->outputs("ratio"); //xjhan
        if (ratio_path.empty())
            ratio_path = "ratio-" + _site;

        base_type_conv::substitute(ratio_path, "$(rec)", _site, false);

        _ratiofile = new base_iof;
        _ratiofile->tsys(base_time::GPS);
        _ratiofile->mask(ratio_path);
        _ratiofile->append(dynamic_cast<set_out *>(_gset)->append());
        std::ostringstream os;
        os << " Epoch        Ratio" << std::endl;
        if (_ratiofile)
        {
            _ratiofile->write(os.str().c_str(), os.str().size());
            _ratiofile->flush();
        }
    }

    void gnss_ambiguity::_initBootfile()
    {
        //output boot   file
        std::string bootfile = "boot-" + _site;

        _bootfile = new base_iof;
        _bootfile->tsys(base_time::GPS);
        _bootfile->mask(bootfile);
        _bootfile->append(dynamic_cast<set_out *>(_gset)->append());
        std::ostringstream os;
        os << " Epoch        Boot" << std::endl;
        if (_bootfile)
        {
            _bootfile->write(os.str().c_str(), os.str().size());
            _bootfile->flush();
        }
    }

    void gnss_ambiguity::_writeRatio(double ratio)
    {
        std::ostringstream os;
        int nepoch = (int)(((_crt_time.dmjd() - _beg.dmjd()) * 86400.0 + _interval * 1E-3) / _interval) + 1;

        // write ratio data
        os << " " << std::setw(4) << nepoch << "     " << std::fixed << std::setw(5) << std::setprecision(2) << ratio << std::endl;

        // Print ratio results
        if (_ratiofile)
        {
            _ratiofile->write(os.str().c_str(), os.str().size());
            _ratiofile->flush();
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "have no output file!");
        }

        return;
    }

    void gnss_ambiguity::_writeBoot(double BootStrapping)
    {
        std::ostringstream os;
        int nepoch = (int)(((_crt_time.dmjd() - _beg.dmjd()) * 86400.0 + _interval * 1E-3) / _interval) + 1;

        // write bootstrapping data
        os << " " << std::setw(4) << nepoch << "     " << std::fixed << std::setw(5) << std::setprecision(2) << BootStrapping * 100.0 << "%" << std::endl;

        // Print bootstrapping results
        if (_bootfile)
        {
            _bootfile->write(os.str().c_str(), os.str().size());
            _bootfile->flush();
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "have no output file!");
        }

        return;
    }

    double gnss_ambiguity::_glonassRecUpd(std::string site)
    {
        double upd_rec = 0.0;
        std::set<std::string> mapSat;
        list<std::pair<double, double>> wl_sig;
        double sigx = 0.0, mean = 0.0, sig = 0.0, upd = 0.0;
        int npnt = 0;
        std::string sat1, sat2;
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            sat1 = get<0>(itdd->ddSats[0]);
            sat2 = get<0>(itdd->ddSats[1]);
            if (site != _site)
                continue;
            if (sat1.substr(0, 1) != "R" || sat2.substr(0, 1) != "R")
                continue;
            if (mapSat.find(sat1) == mapSat.end())
            {
                mapSat.insert(sat1);
                if (_getSingleUpd("WL", _wl_Upd_time, sat1, upd, sig))
                {
                    wl_sig.push_back(std::make_pair(_MW[sat1][2] - upd, 1.0));
                    npnt++;
                }
            }
            if (mapSat.find(sat2) == mapSat.end())
            {
                mapSat.insert(sat2);
                if (_getSingleUpd("WL", _wl_Upd_time, sat2, upd, sig))
                {
                    wl_sig.push_back(std::make_pair(_MW[sat2][2] - upd, 1.0));
                    npnt++;
                }
            }
        }
        if (npnt > 2)
        {
            getMeanFract(wl_sig, mean, sig, sigx);
            // According Panda software, Comment it
            // mean = getFraction(mean, -0.5);
            upd_rec = sigx <= 0.10 ? mean : 0.0;
        }
        return upd_rec;
    }

    // _findRefSD(): 查找并保留包含参考卫星的双差模糊度组合
// 返回值：如果参考卫星集合为空，返回false；否则处理并返回true
    bool gnss_ambiguity::_findRefSD()
    {
        // 如果参考卫星集合为空，记录日志并返回false
        if (_sat_refs.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "_sat_refs is empty");
            return false;
        }

        // 遍历所有双差模糊度项，剔除不包含任一参考卫星的双差组合
        for (auto it_dd = _DD.begin(); it_dd != _DD.end();)
        {
            std::string sat1 = get<0>(it_dd->ddSats[0]); // 双差第一颗卫星
            std::string sat2 = get<0>(it_dd->ddSats[1]); // 双差第二颗卫星

            // 如果两个卫星均不在参考卫星集合中，则删除该双差项
            if (_sat_refs.find(sat1) == _sat_refs.end() &&
                _sat_refs.find(sat2) == _sat_refs.end())
            {
                it_dd = _DD.erase(it_dd); // 删除并更新迭代器
                continue;
            }
            else
                it_dd++; // 否则继续检查下一个
        }

        // 经过筛选后，保留了包含参考卫星的双差组合，返回true
        return true;
    }


    bool gnss_ambiguity::_NDRecovery(std::string satref, base_allpar &params, std::map<std::string, std::map<FREQ_SEQ, double>> &singleND)
    {
        std::map<FREQ_SEQ, double> UPD_ref;
        // choose frequency
        double lambda_1 = _sys_wavelen[satref.substr(0, 1)]["L1"];
        double lambda_2 = _sys_wavelen[satref.substr(0, 1)]["L2"];
        double lambda_3 = _sys_wavelen[satref.substr(0, 1)]["L3"];
        double lambda_4 = _sys_wavelen[satref.substr(0, 1)]["L4"];
        double lambda_5 = _sys_wavelen[satref.substr(0, 1)]["L5"];
        if (satref.substr(0, 1) == "R") //xjhan
        {
            lambda_1 = _sys_wavelen[satref]["L1"];
            lambda_2 = _sys_wavelen[satref]["L2"];
            lambda_3 = _sys_wavelen[satref]["L3"];
        }
        double upd_ewl25, upd_ewl24, upd_ewl, upd_wl, upd_nl, sig, ref_wl, ref_nl;
        upd_ewl24 = upd_ewl25 = upd_ewl = upd_wl = upd_nl = sig = ref_wl = ref_nl = 0;
        int index, index1, index2, index3, index4, index5;

        std::string crt_sat;
        // change by lvhb for RTK which upd is not necessary
        if (_gupd)
        {
            if (_upd_mode == UPD_MODE::UPD)
            {
                if (!_getSingleUpd("WL", _wl_Upd_time, satref, upd_wl, sig) || !_getSingleUpd("NL", _crt_time, satref, upd_nl, sig))
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_amb_fixServer::_NDRecovery] : _getSingleUpd Wrong : NL, Sat: " + satref);
                    return false;
                }
            }
            else if (_upd_mode == UPD_MODE::IRC)
            {
                if (!_getSingleUpd("WL", _wl_Upd_time, satref, upd_wl, sig))
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_amb_fixServer::_NDRecovery] : _getSingleUpd Wrong : NL, Sat: " + satref);
                    return false;
                }
                upd_nl = 0;
            }
            else
            {
                upd_wl = 0;
                upd_nl = 0;
            }

            if (_frequency >= 3 && !_getSingleUpd("EWL", _ewl_Upd_time, satref, upd_ewl, sig))
            {
                upd_ewl = 0;
            }
            if (_frequency >= 4 && !_getSingleUpd("EWL24", _ewl24_Upd_time, satref, upd_ewl24, sig))
            {
                upd_ewl24 = 0;
            }
            if (_frequency >= 5 && !_getSingleUpd("EWL25", _ewl24_Upd_time, satref, upd_ewl25, sig))
            {
                upd_ewl25 = 0;
            }
        }

        if (_obstype == OBSCOMBIN::IONO_FREE)
        {
            index = params.getParam(_site, par_type::AMB_IF, satref);
            if (index >= 0)
            {
                //double ref_lc = params[index]._value() + glsq->dx()(index + 1);
                double ref_lc = params[index].value();
                /*
                * Wide-lane non-difference integer ambiguity, based on the original 
                * unmodified wide-lane upd wide-lane ambiguity after rounding up
                */
                ref_wl = round(_MW[satref][2] - upd_wl);

                ref_nl = (lambda_1 + lambda_2) / (lambda_1 * lambda_2) * ref_lc -
                         lambda_1 / (lambda_2 - lambda_1) * (_MW[satref][2] - upd_wl);
                ref_nl = round(ref_nl - upd_nl);

                if (ref_nl == 0 || (ref_nl - ref_wl) == 0)
                    ref_nl += 1e-9; //xjhan

                /*
                ref_wl = round(_MW[satref][2] );
                // ????L1??????????UPD??
                ref_nl = (lambda_1 + lambda_2) / (lambda_1 * lambda_2) * ref_lc -
                    lambda_1 / (lambda_2 - lambda_1) *ref_wl;
                ref_nl = round(ref_nl);
                */
                singleND[satref][FREQ_1] = ref_nl * lambda_1;            // N1
                singleND[satref][FREQ_2] = (ref_nl - ref_wl) * lambda_2; // N2

                //std::cout << "MW" << std::setw(12)<< _MW[satref][2] << std::setw(12) << upd_wl <<"ref_wl"<< std::setw(12) << ref_wl << "ref_nl" << std::setw(12) << ref_nl << std::endl;
            }
            else
                return false;
        }
        else if (_obstype == OBSCOMBIN::RAW_ALL)
        {
            // get L1 L2 upd
            double upd_n1 = -upd_nl - lambda_1 / (lambda_2 - lambda_1) * upd_wl;
            double upd_n2 = -upd_nl - lambda_2 / (lambda_2 - lambda_1) * upd_wl;

            index1 = _frequency < 2 ? -1 : params.getParam(_site, par_type::AMB_L1, satref);
            index2 = _frequency < 2 ? -1 : params.getParam(_site, par_type::AMB_L2, satref);
            index3 = _frequency < 3 ? -1 : params.getParam(_site, par_type::AMB_L3, satref);
            index4 = _frequency < 3 ? -1 : params.getParam(_site, par_type::AMB_L4, satref);
            index5 = _frequency < 3 ? -1 : params.getParam(_site, par_type::AMB_L5, satref);

            if ((_frequency >= 2 && index1 >= 0 && index2 >= 0) ||
                (_frequency >= 3 && index1 >= 0 && index2 >= 0 && index3 >= 0))
            {
                singleND[satref][FREQ_1] = params[index1].value() - upd_n1 * lambda_1;
                singleND[satref][FREQ_2] = params[index2].value() - upd_n2 * lambda_2;
                /* singleND[satref][FREQ_1] = round(params[index1].value() / lambda_1 - upd_n1) * lambda_1;
                singleND[satref][FREQ_2] = round(params[index2].value() / lambda_2 - upd_n2) * lambda_2;*/

                upd_n1 = upd_nl - lambda_1 / (lambda_2 - lambda_1) * (-upd_wl);
                upd_n2 = upd_nl - lambda_2 / (lambda_2 - lambda_1) * (-upd_wl);

                UPD_ref[FREQ_1] = upd_n1 * lambda_1;
                UPD_ref[FREQ_2] = upd_n2 * lambda_2;

                if (_frequency == 3 /*&& upd_ewl != 0*/ && index3 >= 0)
                {
                    double upd_n3 = (upd_n2 - upd_ewl);
                    //singleND[satref][FREQ_3] = params[index3].value() - upd_n3 * lambda_3;
                    singleND[satref][FREQ_3] = round(params[index3].value() / lambda_3 - upd_n3) * lambda_3;
                    //std::cout << "ref_1  amb " << satref<<" "<< singleND[satref][FREQ_1] / lambda_1 << std::endl;
                }
                if (_frequency >= 4 && index4 >= 0)
                {
                    double upd_n4 = (upd_n2 - upd_ewl24);
                    singleND[satref][FREQ_4] = round(params[index4].value() / lambda_4 - upd_n4) * lambda_4;
                    std::cout << "ref_4  amb " << satref << " " << singleND[satref][FREQ_4] / lambda_4 << std::endl;
                }
                if (_frequency >= 5 && index5 >= 0)
                {
                    double upd_n5 = (upd_n2 - upd_ewl25);
                    singleND[satref][FREQ_5] = round(params[index5].value() / lambda_5 - upd_n5) * lambda_5;
                    std::cout << "ref_5  amb " << satref << " " << singleND[satref][FREQ_5] / lambda_5 << std::endl;
                }

                upd_n1 = upd_nl - lambda_1 / (lambda_2 - lambda_1) * (-upd_wl);
                upd_n2 = upd_nl - lambda_2 / (lambda_2 - lambda_1) * (-upd_wl);

                UPD_ref[FREQ_1] = upd_n1 * lambda_1;
                UPD_ref[FREQ_2] = upd_n2 * lambda_2;
            }
        }
        else if (_obstype == OBSCOMBIN::RAW_MIX || _obstype == OBSCOMBIN::RAW_SINGLE) //Lvhb added in 202105
        {
            // get L1 L2 upd
            double upd_n1 = upd_nl - lambda_1 / (lambda_2 - lambda_1) * (-upd_wl);
            double upd_n2 = upd_nl - lambda_2 / (lambda_2 - lambda_1) * (-upd_wl);

            index1 = _frequency < 1 ? -1 : params.getParam(_site, par_type::AMB_L1, satref);
            index2 = _frequency < 2 ? -1 : params.getParam(_site, par_type::AMB_L2, satref);
            index3 = _frequency < 3 ? -1 : params.getParam(_site, par_type::AMB_L3, satref);

            if (_frequency >= 1)
            {
                if (index1 >= 0)
                {
                    singleND[satref][FREQ_1] = round(params[index1].value() / lambda_1 - upd_n1) * lambda_1;
                    UPD_ref[FREQ_1] = upd_n1 * lambda_1;
                }
                if (index2 >= 0)
                {
                    singleND[satref][FREQ_2] = round(params[index2].value() / lambda_2 - upd_n2) * lambda_2;
                    UPD_ref[FREQ_2] = upd_n2 * lambda_2;
                }
                if (index3 >= 0)
                {
                    double upd_n3 = (upd_n2 - upd_ewl);
                    singleND[satref][FREQ_3] = round(params[index3].value() / lambda_3 - upd_n3) * lambda_3;
                }
            }
        }

        // Other satellites N1 and N2
        bool bUseSys = false;                      //lvhb
        bool bUseFreq1 = false, bUseFreq2 = false; //lvhb added in 202105
        bool bUseFreq3 = false;                    //lvhb added in 202201214
        bool bUseFreq4 = false;
        bool bUseFreq5 = false;
        for (auto itdd = _DD.begin(); itdd != _DD.end(); itdd++)
        {
            if (_obstype == OBSCOMBIN::RAW_MIX || _obstype == OBSCOMBIN::RAW_SINGLE)
            { //lvhb added in 202105
                if (itdd->isNlFixed)
                {
                    std::string sat1 = get<0>(itdd->ddSats[0]);
                    std::string sat2 = get<0>(itdd->ddSats[1]);

                    crt_sat = sat1;
                    if (!_simulation && (crt_sat.substr(0, 1) != satref.substr(0, 1)))
                        continue;
                    if (sat1 != satref && sat2 != satref)
                        continue;
                    if (sat1 == satref && sat2 != satref)
                    {
                        crt_sat = sat2;
                        itdd->rwl *= -1.0;
                        itdd->inl *= -1.0;
                        itdd->iwl *= -1.0;
                        itdd->iewl *= -1.0;
                    }
                    bUseSys = true;

                    if (itdd->ambtype == "AMB_L1")
                    {
                        singleND[crt_sat][FREQ_1] = singleND[satref][FREQ_1] + itdd->inl * lambda_1;
                        bUseFreq1 = true;
                        if (itdd->isWlFixed && index2 >= 0)
                        {
                            singleND[crt_sat][FREQ_2] = singleND[satref][FREQ_2] + (itdd->inl - itdd->iwl) * lambda_2;
                            bUseFreq2 = true;
                        }

                        if (itdd->isWlFixed && index2 >= 0 && index3 >= 0 && itdd->isEwlFixed)
                        {
                            singleND[crt_sat][FREQ_3] = singleND[satref][FREQ_3] + (itdd->inl - itdd->iwl - itdd->iewl) * lambda_3;
                            ;
                            bUseFreq3 = true;
                        }
                    }
                    else if (itdd->ambtype == "AMB_L2")
                    {
                        singleND[crt_sat][FREQ_2] = singleND[satref][FREQ_2] + itdd->inl * itdd->factor;
                        bUseFreq2 = true;
                        if (index3 >= 0 && itdd->isEwlFixed)
                        {
                            singleND[crt_sat][FREQ_3] = singleND[satref][FREQ_3] + (itdd->inl - itdd->iewl) * lambda_3;
                            bUseFreq3 = true;
                        }
                    }
                    else if (itdd->ambtype == "AMB_L3")
                    {
                        singleND[crt_sat][FREQ_3] = singleND[satref][FREQ_3] + itdd->inl * itdd->factor;
                        bUseFreq3 = true;
                    }
                }
            }
            else
            {
                if (itdd->isWlFixed && itdd->isNlFixed)
                {
                    bUseFreq1 = bUseFreq2 = true;
                    std::string sat1 = get<0>(itdd->ddSats[0]);
                    std::string sat2 = get<0>(itdd->ddSats[1]);
                    //LX: delete the Triple-frequency sat EWL cannot be fixed.
                    // int index = params.getParam(_site, par_type::AMB_L3, sat1);
                    //if (_frequency == 3 && index >= 0 && !itdd->isEwlFixed) continue;

                    crt_sat = sat1;
                    if (!_simulation && (crt_sat.substr(0, 1) != satref.substr(0, 1)))
                        continue; //xjhan
                    if (sat1 != satref && sat2 != satref)
                        continue;
                    if (sat1 == satref && sat2 != satref)
                    {
                        crt_sat = sat2;
                        itdd->rwl *= -1.0;
                        itdd->inl *= -1.0;
                        itdd->iwl *= -1.0;
                        itdd->iewl *= -1.0;
                        itdd->iewl24 *= -1.0;
                        itdd->iewl25 *= -1.0;
                    }
                    bUseSys = true;

                    if (_obstype == OBSCOMBIN::IONO_FREE)
                    {
                        int index = params.getParam(_site, par_type::AMB_IF, crt_sat);
                        if (index >= 0)
                        {
                            double wl = round(itdd->rwl) + ref_wl;
                            double nl = itdd->inl + ref_nl;
                            if (nl == 0 || (nl - wl) == 0)
                                nl += 1e-9;

                            //xjhan
                            if (crt_sat.substr(0, 1) != "R")
                            {
                                singleND[crt_sat][FREQ_1] = nl * lambda_1;
                                singleND[crt_sat][FREQ_2] = (nl - wl) * lambda_2;
                            }
                            else
                            {
                                singleND[crt_sat][FREQ_1] = nl * _sys_wavelen[crt_sat]["L1"];
                                singleND[crt_sat][FREQ_2] = (nl - wl) * _sys_wavelen[crt_sat]["L2"];
                            }
                            //std::cout << "N1 ambiguity" << std::setw(12) << crt_sat << std::setw(12) << nl <<std::setw(12) << singleND[crt_sat][FREQ_1] << std::endl;
                            //std::cout << "N2 ambiguity" << std::setw(12) << crt_sat << std::setw(12) << nl - wl << std::setw(12) << singleND[crt_sat][FREQ_2] << std::endl;
                        }
                    }
                    else if (_obstype == OBSCOMBIN::RAW_ALL)
                    {
                        if (itdd->ambtype == "AMB_L1")
                        {
                            singleND[crt_sat][FREQ_1] = singleND[satref][FREQ_1] + itdd->inl * lambda_1;
                            singleND[crt_sat][FREQ_2] = singleND[satref][FREQ_2] + (itdd->inl - itdd->iwl) * lambda_2;

                            if (!dynamic_cast<set_npp *>(_gset)->comp_aug())
                            {
                                upd_ewl = upd_wl = upd_nl = sig = ref_wl = ref_nl = 0;
                                if (_gupd)
                                {
                                    if (_upd_mode == UPD_MODE::UPD)
                                    {
                                        if (!_getSingleUpd("WL", _wl_Upd_time, crt_sat, upd_wl, sig) || !_getSingleUpd("NL", _crt_time, crt_sat, upd_nl, sig))
                                        {
                                            if (_spdlog)
                                                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_amb_fixServer::_NDRecovery] : _getSingleUpd Wrong : NL, Sat: " + crt_sat);
                                            return false;
                                        }
                                    }
                                    else if (_upd_mode == UPD_MODE::IRC)
                                    {
                                        if (!_getSingleUpd("WL", _wl_Upd_time, crt_sat, upd_wl, sig))
                                        {
                                            if (_spdlog)
                                                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning[gnss_amb_fixServer::_NDRecovery] : _getSingleUpd Wrong : NL, Sat: " + crt_sat);
                                            return false;
                                        }
                                        upd_nl = 0;
                                    }
                                    else
                                    {
                                        upd_wl = 0;
                                        upd_nl = 0;
                                    }
                                }

                                double upd_n1 = upd_nl - lambda_1 / (lambda_2 - lambda_1) * (-upd_wl);
                                double upd_n2 = upd_nl - lambda_2 / (lambda_2 - lambda_1) * (-upd_wl);

                                singleND[crt_sat][FREQ_1] -= upd_n1 * lambda_1;
                                singleND[crt_sat][FREQ_2] -= upd_n2 * lambda_2;
                            }

                            if (_frequency >= 3  && index3 >= 0 && itdd->isEwlFixed)
                            {
                                singleND[crt_sat][FREQ_3] = singleND[satref][FREQ_3] + (itdd->inl - itdd->iwl - itdd->iewl) * lambda_3;
                                bUseFreq3 = true;
                            }
                            if (_frequency >= 4  && index4 >= 0 && itdd->isEwl24Fixed)
                            {
                                singleND[crt_sat][FREQ_4] = singleND[satref][FREQ_4] + (itdd->inl - itdd->iwl - itdd->iewl24) * lambda_4;
                                bUseFreq4 = true;
                            }
                            if (_frequency >= 5  && index5 >= 0 && itdd->isEwl25Fixed)
                            {
                                singleND[crt_sat][FREQ_5] = singleND[satref][FREQ_5] + (itdd->inl - itdd->iwl - itdd->iewl25) * lambda_5;
                                bUseFreq5 = true;
                            }
                            //std::cout << sat1 << sat2 <<std::setw(12) << singleND[satref][FREQ_1]/ lambda_1<< std::setw(12) << singleND[crt_sat][FREQ_1]/ lambda_1 <<std::setw(12)<<itdd->inl<<std::endl;
                        }
                        //    else if (itdd->ambtype == "AMB_L2")
                        //    {
                        //        singleND[crt_sat][FREQ_2] = singleND[satref][FREQ_2] + itdd->inl * itdd->factor;
                        //    }
                        //    else if (itdd->ambtype == "AMB_L3" && upd_ewl != 0)
                        //    {
                        //        singleND[crt_sat][FREQ_3] = singleND[satref][FREQ_3] + itdd->inl * itdd->factor;
                        //    }
                        //    else if (itdd->ambtype == "AMB_L4")
                        //    {
                        //        singleND[crt_sat][FREQ_4] = singleND[satref][FREQ_4] + itdd->inl * itdd->factor;
                        //    }
                        //    else if (itdd->ambtype == "AMB_L5")
                        //    {
                        //        singleND[crt_sat][FREQ_5] = singleND[satref][FREQ_5] + itdd->inl * itdd->factor;
                        //    }
                    }
                }
            }
        }

        if (!dynamic_cast<set_npp *>(_gset)->comp_aug())
        {
            singleND[satref][FREQ_1] -= UPD_ref[FREQ_1];
            singleND[satref][FREQ_2] -= UPD_ref[FREQ_2];
        }

#ifdef DEBUG_pppRTK
        std::cout << "after the ND" << std::endl;
        for (auto itsat = singleND.begin(); itsat != singleND.end(); itsat++)

        {
            if (itsat->first.substr(0, 1) != satref.substr(0, 1))
                continue;
            std::cout << std::setw(5) << itsat->first << std::fixed
                 << std::setw(15) << std::setprecision(4) << itsat->second[FREQ_1] / lambda_1
                 << std::setw(15) << std::setprecision(4) << itsat->second[FREQ_2] / lambda_2
                 << std::setw(15) << std::setprecision(4) << itsat->second[FREQ_3] / lambda_3
                 << std::setw(15) << std::setprecision(4) << itsat->second[FREQ_4] / lambda_4
                 << std::setw(15) << std::setprecision(4) << itsat->second[FREQ_5] / lambda_5
                 << std::endl;
        }
#endif // DEBUG \
    // delete satellite whose ambiguity number less than 2
        int minfrenum = 2;
        for (auto itsat = singleND.begin(); itsat != singleND.end();)
        {
            //lvhb: delete satref
            if (!bUseSys && itsat->first == satref)
            {
                itsat = singleND.erase(itsat);
                continue;
            }
            //lvhb: delete nouseful frequency in satref
            if (itsat->second.size() >= 3 && !bUseFreq3 && itsat->first == satref)
            {
                std::map<FREQ_SEQ, double>::iterator key = singleND[satref].find(FREQ_3);
                if (key != singleND[satref].end())
                    singleND[satref].erase(key);
            }

            if (itsat->second.size() >= 4 && !bUseFreq4 && itsat->first == satref)
            {
                std::map<FREQ_SEQ, double>::iterator key = singleND[satref].find(FREQ_4);
                if (key != singleND[satref].end())
                    singleND[satref].erase(key);
            }

            if (itsat->second.size() >= 5 && !bUseFreq5 && itsat->first == satref)
            {
                std::map<FREQ_SEQ, double>::iterator key = singleND[satref].find(FREQ_5);
                if (key != singleND[satref].end())
                    singleND[satref].erase(key);
            }

            if (_obstype == OBSCOMBIN::RAW_MIX || _obstype == OBSCOMBIN::RAW_SINGLE)
            { //lvhb added in 202105
                if (itsat->second.size() >= 1 && !bUseFreq1 && itsat->first == satref)
                {
                    std::map<FREQ_SEQ, double>::iterator key = singleND[satref].find(FREQ_1);
                    if (key != singleND[satref].end())
                        singleND[satref].erase(key);
                }
                if (itsat->second.size() >= 2 && !bUseFreq2 && itsat->first == satref)
                {
                    std::map<FREQ_SEQ, double>::iterator key = singleND[satref].find(FREQ_2);
                    if (key != singleND[satref].end())
                        singleND[satref].erase(key);
                }
                minfrenum = 1;
            }

            if (itsat->second.size() < minfrenum)
            {
                itsat = singleND.erase(itsat);
            }
            else
            {
                itsat++;
            }
        }
        if (singleND.size() < 1)
            return false;
        else
            return true;
    }

    int gnss_ambiguity::_float_fix_check(gnss_proc_flt *gfltnow, gnss_proc_flt &gfltpre, base_time now)
    {
        base_allpar param = gfltnow->param();
        Vector dx_crt = gfltnow->dx();
        Vector dx_pre = gfltpre.dx();
        int ix = param.getParam(_site, par_type::CRD_X, "");
        int iy = param.getParam(_site, par_type::CRD_Y, "");
        int iz = param.getParam(_site, par_type::CRD_Z, "");

        double dx = fabs(dx_crt(ix) - dx_pre(ix));
        double dy = fabs(dx_crt(iy) - dx_pre(iy));
        double dz = fabs(dx_crt(iz) - dx_pre(iz));

        if (dx >= _FloatFixSep || dy >= _FloatFixSep || dz >= _FloatFixSep)
        {
            //std::cout << now.str_hms() << "  FloatFixSep Exceed "  << _FloatFixSep << "  "
            //    << std::setw(8) << std::setprecision(3) << dx
            //    << std::setw(8) << std::setprecision(3) << dy
            //    << std::setw(8) << std::setprecision(3) << dz << std::endl;
            return -1;
        }
        return 1;
    }

    int gnss_ambiguity::_fix_fix_check(gnss_proc_flt *gfltnow, gnss_proc_flt &gfltpre, base_time now, std::string mode)
    {
        if (_DD_previous[mode].empty())
            return 0;

        if (now.diff(_last_fix_time[mode]["sum"]) >= 5)
            return 0;

        if (mode != "NL")
            return 0;

        double dl, p0, integer = 0.0, flt = 0.0, lambda_1 = 0.0, lambda_2 = 0.0;

        if (mode == "NL")
            p0 = 1E9;
        else
            p0 = 1E6;

        gnss_proc_lsq_equationmatrix virtual_equ;
        gnss_data_obscombtype type;
        int index_sat1, index_sat2, index_sat3, index_sat4;

        hwa_vector_amb_dd used_DD;
        if (mode == "NL")
        {
            std::set<std::string> exist_pair;
            hwa_vector_amb_dd Redundant_DD = _DD_previous[mode];
            auto DD_iter = Redundant_DD.begin();
            while (DD_iter != Redundant_DD.end())
            {
                std::string sat1 = get<0>(DD_iter->ddSats[0]);
                std::string sat2 = get<0>(DD_iter->ddSats[1]);
                index_sat1 = _sats_index[sat1][DD_iter->ambtype];
                index_sat2 = _sats_index[sat2][DD_iter->ambtype];

                if (index_sat1 >= 1 && index_sat2 >= 1)
                {
                    exist_pair.insert(sat1 < sat2 ? sat1 + "-" + sat2 : sat2 + "-" + sat1);
                    used_DD.push_back(*DD_iter);
                    DD_iter = Redundant_DD.erase(DD_iter);
                }
                else if (index_sat1 < 1 && index_sat2 < 1)
                {
                    DD_iter = Redundant_DD.erase(DD_iter);
                }
                else
                {
                    DD_iter++;
                }
            }

            if (Redundant_DD.size() >= 2)
            {
                // lossSat      laftSat    dd_idx   +/-
                std::map<std::string, std::map<std::string, std::pair<int, int>>> cand_pairs;
                for (int i = 0; i < Redundant_DD.size(); i++)
                {
                    std::string sat1 = get<0>(Redundant_DD[i].ddSats[0]);
                    std::string sat2 = get<0>(Redundant_DD[i].ddSats[1]);
                    index_sat1 = _sats_index[sat1][Redundant_DD[i].ambtype];
                    index_sat2 = _sats_index[sat2][Redundant_DD[i].ambtype];
                    std::string loss, exist;
                    int sign;
                    if (index_sat1 >= 1)
                    {
                        loss = sat2;
                        exist = sat1;
                        sign = 1;
                    }
                    if (index_sat2 >= 1)
                    {
                        loss = sat1;
                        exist = sat2;
                        sign = -1;
                    }

                    if (cand_pairs[loss].size() == 0)
                        cand_pairs[loss][exist] = std::make_pair(i, sign);
                    else
                    {
                        std::string exist0 = cand_pairs[loss].begin()->first;
                        int i0 = cand_pairs[loss].begin()->second.first;
                        int sign0 = cand_pairs[loss].begin()->second.second;
                        std::string new_pair = exist0 < exist ? exist0 + "-" + exist : exist + "-" + exist0;
                        if (exist0 != exist && exist_pair.find(new_pair) == exist_pair.end())
                        {
                            exist_pair.insert(new_pair);
                            gnss_amb_dd_base dd;

                            dd.ambtype = Redundant_DD[i].ambtype;
                            dd.isWlFixed = true;
                            dd.isNlFixed = true;

                            dd.ddSats.push_back(std::make_tuple(exist0, 99, 99));
                            dd.ddSats.push_back(std::make_tuple(exist, 99, 99));

                            //add
                            if (sign0 * sign > 0)
                            {
                                dd.inl = sign0 * (Redundant_DD[i0].inl - Redundant_DD[i].inl);
                                dd.rwl = sign0 * (Redundant_DD[i0].rwl - Redundant_DD[i].rwl);
                                dd.sd_rnl_cor = sign0 * (Redundant_DD[i0].sd_rnl_cor - Redundant_DD[i].sd_rnl_cor);
                                dd.sd_r1_cor = sign0 * (Redundant_DD[i0].sd_r1_cor - Redundant_DD[i].sd_r1_cor);
                                dd.sd_r2_cor = sign0 * (Redundant_DD[i0].sd_r2_cor - Redundant_DD[i].sd_r2_cor);
                                dd.sd_r3_cor = sign0 * (Redundant_DD[i0].sd_r3_cor - Redundant_DD[i].sd_r3_cor);
                            }
                            //minus
                            else
                            {
                                dd.inl = sign0 * (Redundant_DD[i0].inl + Redundant_DD[i].inl);
                                dd.rwl = sign0 * (Redundant_DD[i0].rwl + Redundant_DD[i].rwl);
                                dd.sd_rnl_cor = sign0 * (Redundant_DD[i0].sd_rnl_cor + Redundant_DD[i].sd_rnl_cor);
                                dd.sd_r1_cor = sign0 * (Redundant_DD[i0].sd_r1_cor + Redundant_DD[i].sd_r1_cor);
                                dd.sd_r2_cor = sign0 * (Redundant_DD[i0].sd_r2_cor + Redundant_DD[i].sd_r2_cor);
                                dd.sd_r3_cor = sign0 * (Redundant_DD[i0].sd_r3_cor + Redundant_DD[i].sd_r3_cor);
                            }
                            used_DD.push_back(dd);
                        }
                    }
                }
            }
        }
        else
        {
            used_DD = _DD_previous[mode];
        }

        for (auto itdd = used_DD.begin(); itdd != used_DD.end(); itdd++)
        {
            std::string sat1 = get<0>(itdd->ddSats[0]);
            std::string sat2 = get<0>(itdd->ddSats[1]);

            if (mode == "NL")
            {
                bool wl_fix = ((_obstype == OBSCOMBIN::RAW_MIX && itdd->isSngleFreq != true) || (_obstype != OBSCOMBIN::RAW_MIX && _obstype != OBSCOMBIN::RAW_SINGLE));
                if ((!itdd->isWlFixed && wl_fix) || !itdd->isNlFixed)
                    continue;

                index_sat1 = _sats_index[sat1][itdd->ambtype];
                index_sat2 = _sats_index[sat2][itdd->ambtype];
                if (index_sat1 < 1 || index_sat2 < 1)
                    continue;

                flt = gfltpre.param()[index_sat1 - 1].value() - gfltpre.param()[index_sat2 - 1].value();

                if (_obstype == OBSCOMBIN::IONO_FREE)
                {
                    std::string sys = get<0>(itdd->ddSats[0]).substr(0, 1);
                    integer = (itdd->inl - itdd->sd_rnl_cor + round(itdd->rwl) * _sys_wavelen[sys]["WL"] / _sys_wavelen[sys]["L2"]) * itdd->factor;
                }
                else if (_obstype == OBSCOMBIN::RAW_ALL || _obstype == OBSCOMBIN::RAW_SINGLE || _obstype == OBSCOMBIN::RAW_MIX)
                {
                    double Lx_cor = 0.0;
                    if (itdd->ambtype == "AMB_L1")
                        Lx_cor = itdd->sd_r1_cor;
                    else if (itdd->ambtype == "AMB_L2")
                        Lx_cor = itdd->sd_r2_cor;
                    else if (itdd->ambtype == "AMB_L3")
                        Lx_cor = itdd->sd_r3_cor;
                    integer = (itdd->inl - Lx_cor) * itdd->factor;
                }
            }
            else if (mode == "WL")
            {
                std::string sat3 = get<0>(itdd->ddSats[2]);
                std::string sat4 = get<0>(itdd->ddSats[3]);
                index_sat1 = _sats_index[sat1]["AMB_L1"];
                index_sat2 = _sats_index[sat2]["AMB_L1"];
                index_sat3 = _sats_index[sat3]["AMB_L2"];
                index_sat4 = _sats_index[sat4]["AMB_L2"];
                if (index_sat1 < 1 || index_sat2 < 1 || index_sat3 < 1 || index_sat4 < 1)
                    continue;
                if (!itdd->isWlFixed || itdd->rwl == 0)
                    continue;
                lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L1"];
                lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                integer = (itdd->iwl - itdd->sd_rwl_cor);
                flt = gfltpre.param()[index_sat1 - 1].value() / lambda_1 - gfltpre.param()[index_sat3 - 1].value() / lambda_2 - gfltpre.param()[index_sat2 - 1].value() / lambda_1 + gfltpre.param()[index_sat4 - 1].value() / lambda_2;
            }
            else if (mode == "EWL")
            {
                std::string sat3 = get<0>(itdd->ddSats[2]);
                std::string sat4 = get<0>(itdd->ddSats[3]);
                index_sat1 = _sats_index[sat1]["AMB_L2"];
                index_sat2 = _sats_index[sat2]["AMB_L2"];
                index_sat3 = _sats_index[sat3]["AMB_L3"];
                index_sat4 = _sats_index[sat4]["AMB_L3"];
                if (index_sat1 < 1 || index_sat2 < 1 || index_sat3 < 1 || index_sat4 < 1)
                    continue;
                if (!itdd->isEwlFixed || itdd->rewl == 0)
                    continue;
                lambda_1 = _sys_wavelen[sat1.substr(0, 1)]["L2"];
                lambda_2 = _sys_wavelen[sat1.substr(0, 1)]["L3"];
                integer = (itdd->iewl - itdd->sd_rewl_cor);
                flt = gfltpre.param()[index_sat1 - 1].value() / lambda_1 - gfltpre.param()[index_sat3 - 1].value() / lambda_2 - gfltpre.param()[index_sat2 - 1].value() / lambda_1 + gfltpre.param()[index_sat4 - 1].value() / lambda_2;
            }

            dl = integer - flt;

            std::vector<std::pair<int, double>> B;
            if (mode == "NL")
            {
                B.push_back(std::make_pair(index_sat1, 1));
                B.push_back(std::make_pair(index_sat2, -1));
            }
            else
            {
                B.push_back(std::make_pair(index_sat1, 1 / lambda_1));
                B.push_back(std::make_pair(index_sat3, -1 / lambda_2));
                B.push_back(std::make_pair(index_sat2, -1 / lambda_1));
                B.push_back(std::make_pair(index_sat4, 1 / lambda_2));
            }

            virtual_equ.add_equ(B, p0, dl, _site, get<0>(itdd->ddSats[0]) + "_" + get<0>(itdd->ddSats[1]), type, false);
        }

        int valid = 1;

        if (virtual_equ.num_equ() >= 4)
        {
            Matrix B_mat;
            Symmetric P_mat;
            Vector l_mat;
            virtual_equ.chageNewMat(B_mat, P_mat, l_mat, gfltpre.npar_number());
            gfltpre.resetQ();
            gfltpre.add_virtual_obs(B_mat, P_mat, l_mat);

            try
            {
                gfltpre.update();
            }
            catch (exception e)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, e.what(), "Solve Ebase_quation Fail!");
                return 0;
            }

            base_allpar param = gfltnow->param();
            Vector dx_crt = gfltnow->dx();
            Vector dx_pre = gfltpre.dx();
            int ix = param.getParam(_site, par_type::CRD_X, "");
            int iy = param.getParam(_site, par_type::CRD_Y, "");
            int iz = param.getParam(_site, par_type::CRD_Z, "");

            double dx = fabs(dx_crt(ix) - dx_pre(ix));
            double dy = fabs(dx_crt(iy) - dx_pre(iy));
            double dz = fabs(dx_crt(iz) - dx_pre(iz));
            //std::cout << now.str_hms() << "  FixFixSep  "  << std::setw(8) << std::setprecision(3) << dx
            //    << std::setw(8) << std::setprecision(3) << dy
            //    << std::setw(8) << std::setprecision(3) << dz  << std::endl;

            if (dx >= _FixFixSep || dy >= _FixFixSep || dz >= _FixFixSep)
            {
                //std::cout << now.str_hms() << "   FixFixSep Exceed   " << _FixFixSep << std::endl;
                valid = -1;
            }
        }

        return valid;
    }

    bool _ddCompare(const gnss_amb_dd_base &dd1, const gnss_amb_dd_base &dd2)
    {
        return (dd1.end_epo - dd1.beg_epo) > (dd2.end_epo - dd2.beg_epo);
    }

    gnss_amb_cmn::gnss_amb_cmn()
    {
    }

    gnss_amb_cmn::gnss_amb_cmn(const base_time &t, gnss_proc_lsqbase *lsq)
    {

        _now = t;
        _param = lsq->_x_solve;
        _sigma0 = lsq->sigma0();
        _dx = lsq->dx();
        _Qx = lsq->Qx();
        _stdx = lsq->stdx();
        _amb_fixed = false;
        _ratio = 0.0;
    }

    gnss_amb_cmn::gnss_amb_cmn(const base_time &t, gnss_proc_flt *flt)
    {

        _now = t;
        _param = flt->param();
        _sigma0 = flt->sigma0();
        _vtpv = flt->vtpv();
        _Qx = flt->Qx();
        _dx = flt->dx();
        _stdx = flt->stdx();
        _nobs_total = flt->nobs_total();
        _npar_number = flt->npar_number();
        _amb_fixed = false;
        _active_amb.insert(std::make_pair("", 1));
        _ratio = 0.0;
    }

    gnss_amb_cmn::~gnss_amb_cmn()
    {
    }

    void gnss_amb_cmn::active_amb(std::map<std::string, int> active_amb)
    {
        _active_amb = active_amb;
    }

    int gnss_amb_cmn::active_amb(std::string site)
    {
        return _active_amb[site];
    }
    void gnss_amb_cmn::amb_fixed(bool b)
    {
        _amb_fixed = b;
    }

    bool gnss_amb_cmn::amb_fixed()
    {
        return _amb_fixed;
    }

    void gnss_amb_cmn::set_ratio(double r)
    {
        _ratio = r;
    }

    double gnss_amb_cmn::get_ratio()
    {
        return _ratio;
    }

    void gnss_amb_cmn::set_boot(double b)
    {
        _boot = b;
    }

    double gnss_amb_cmn::get_boot()
    {
        return _boot;
    }

    void gnss_amb_cmn::set_mode(std::string mode)
    {
        _mode = mode;
    }

    std::string gnss_amb_cmn::get_mode()
    {
        return _mode;
    }

    base_time gnss_amb_cmn::now() const
    {
        return _now;
    }

    double gnss_amb_cmn::sigma0() const
    {
        return _sigma0;
    }

    base_allpar gnss_amb_cmn::param() const
    {
        return _param;
    }

    Vector gnss_amb_cmn::dx() const
    {
        return _dx;
    }

    Vector gnss_amb_cmn::stdx() const
    {
        return _stdx;
    }

    Symmetric gnss_amb_cmn::Qx() const
    {
        return _Qx;
    }

}
