#include "hwa_gnss_amb_fixdd.h"
#include "hwa_gnss_all_amb.h"
#include "hwa_gnss_coder_ambcon.h"
#include "hwa_set_out.h"
#include "hwa_base_file.h"
#include "hwa_base_log.h"

namespace hwa_gnss
{
    gnss_amb_fixdd::gnss_amb_fixdd()
    {
    }

    gnss_amb_fixdd::gnss_amb_fixdd(base_log spdlog, set_base *set) : gnss_amb_fix(spdlog, set)
    {
        // add data to gall ambs
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

        _gambs.add_set(_gset);
        _gambs.add_log(_spdlog);
    }

    gnss_amb_fixdd::~gnss_amb_fixdd()
    {
    }

    bool gnss_amb_fixdd::ProcessBatch()
    {
        std::string func_id = "ProcessBatch";
        if (!_gpar || !_gset || !_gbias)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "check the gpar || gset || gbias");
            return false;
        }

        // add data to gambs
        _gambs.add_data(_gpar, _gobs, _gbias, _gobj, _gupd);
        _gambs.add_bdeci(_evaluator_EWL, _evaluator_WL, _evaluator_NL);

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "combin SD ambiguities for each site");
        _gambs.cmb_sd_by_sat();
        _gambs.clear(AMB_TYPE::UD);

        if (_mode == AMB_TYPE::DD)
        {
            if (!_select_baseline())
                return false;

            _gambs.process_dd_ambs(_gRecBl.baselines());
            _gambs.clear(AMB_TYPE::SD);
        }
        else if (_mode == AMB_TYPE::SD)
        {
            _gambs.process_sd_ambs();
        }
        else if (_mode == AMB_TYPE::UD)
        {
            _gambs.process_sd_ambs();
            _gambs.fix_ow_by_sd();
        }

        // write ambcon
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "write ambiguity constraint file");
        this->_write_ambcon();

        return true;
    }

    bool gnss_amb_fixdd::_select_baseline()
    {
        if (!_gpar || !_gset || !_gobj)
        {
            throw_logical_error(_spdlog, "check your gparr || gset || gobj ptr!");
        }

        _rec_list = _gambs.rec_list();
        _sat_list = _gambs.sat_list();

        if (_rec_list.empty() || _sat_list.empty())
            return false;

        for (const auto &sat : _sat_list)
            _gSatBl.add_sat(sat);

        for (const auto &rec : _rec_list)
            for (const auto &sat : _gambs.sats_of_rec(rec))
                _gRecBl.add_rec(rec, sat, _gambs.get_crd(rec));

        _gRecBl.add_cut(_beg, _max_baseline_length);
        _gRecBl.add_gobj(_gobj);
        _gRecBl.add_gset(_gset);
        _gRecBl.add_glog(_spdlog);

        return true;
    }

    void gnss_amb_fixdd::_write_ambcon()
    {
        gnss_data_ambcon _gambcon(_spdlog);
        _gambcon.segnss_sys(_gambs.gsys());
        _gambcon.set_amb_num(_gambs.num_fixed, _gambs.num_all);
        _gambcon.set_sat_rec(_gambs.sat_list(), _gambs.rec_list());
        _gambcon.set_time(_beg, _end);

        if (_mode == AMB_TYPE::DD)
            _gambcon.add_amb(_gambs.get_amb_dds());
        else if (_mode == AMB_TYPE::SD)
            _gambcon.add_amb(_gambs.get_sd_ambs());

        _gambcon.set_type(_mode);

        base_file gout(_spdlog);
        gnss_coder_ambcon gamb_coder(_gset, "", 4096);
        std::string path = (dynamic_cast<set_out *>(_gset)->outputs("ambiguity"));
        //if (path.empty())
        //    path = (dynamic_cast<set_out *>(_gset)->outputs("con"));
        if (path.empty())
            path = "grt$(date).con";
        base_type_conv::substitute(path, "$(date)", _beg.str_yyyydoy(), false);

        gout.path(path);
        gout.spdlog(this->_spdlog);
        gamb_coder.clear();
        gamb_coder.spdlog(this->_spdlog);
        gamb_coder.path(path);
        gamb_coder.add_data("ID1", &_gambcon);
        gamb_coder.add_data("OBJ", this->_gobj);
        gout.coder(&gamb_coder);
        gout.run_write();
    }
}
