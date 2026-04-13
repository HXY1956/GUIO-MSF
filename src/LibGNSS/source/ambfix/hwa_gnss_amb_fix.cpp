#include "hwa_gnss_amb_fix.h"
#include <utility>

namespace hwa_gnss
{
    gnss_amb_fix::gnss_amb_fix()
    {
    }

    gnss_amb_fix::gnss_amb_fix(base_log spdlog, set_base *set)
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
        if (nullptr == set)
        {
            spdlog::critical("your std::set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = set;
        }

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "================> constructor of gnss_amb_fixdd");

        if (_gset)
        {
            auto gsetamb = dynamic_cast<set_amb *>(_gset);
            if (gsetamb)
            {
                _mode = gsetamb->amb_type();
                _fix_mode = gsetamb->fix_mode();
                _upd_mode = gsetamb->upd_mode();
                _ratio = gsetamb->lambda_ratio();
                _min_common_time = gsetamb->min_common_time();
                _wl_interval = gsetamb->wl_interval();
                _max_baseline_length = gsetamb->max_baseline_length();
                _map_EWL_decision = gsetamb->get_amb_decision("EWL");
                _map_WL_decision = gsetamb->get_amb_decision("WL");
                _map_NL_decision = gsetamb->get_amb_decision("NL");
            }

            auto gsetgen = dynamic_cast<set_gen *>(_gset);
            if (gsetgen)
            {
                this->_beg = gsetgen->beg();
                this->_end = gsetgen->end();
                this->_intv = gsetgen->sampling();
            }
        }
        this->_evaluator_EWL = new gnss_amb_bdeci(this->_map_EWL_decision["maxdev"], this->_map_EWL_decision["maxsig"],
                                            0.0, this->_map_EWL_decision["alpha"]);
        this->_evaluator_WL = new gnss_amb_bdeci(this->_map_WL_decision["maxdev"], this->_map_WL_decision["maxsig"],
                                           0.0, this->_map_WL_decision["alpha"]);
        this->_evaluator_NL = new gnss_amb_bdeci(this->_map_NL_decision["maxdev"], this->_map_NL_decision["maxsig"],
                                           0.0, this->_map_NL_decision["alpha"]);
    }

    void gnss_amb_fix::add_data(gnss_all_obs *obs, gnss_all_bias *bias, base_allpar *pars, gnss_all_obj *objs)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, " ============> gnss_amb_fix::add_data <===============");

        this->_gobs = obs;
        this->_gbias = bias;
        this->_gpar = pars;
        this->_gobj = objs;
    }
    void gnss_amb_fix::add_data(gnss_all_obs *obs, gnss_all_bias *bias, base_allpar *pars, gnss_data_upd *upd, gnss_all_obj *objs)
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, " ============> gnss_amb_fix::add_data <===============");

        this->_gobs = obs;
        this->_gbias = bias;
        this->_gpar = pars;
        this->_gupd = upd;
        this->_gobj = objs;
    }

    void gnss_amb_fix::add_data(gnss_all_obs *obs)
    {
        this->_gobs = obs;
    }

    void gnss_amb_fix::add_data(gnss_all_bias *bias)
    {
        this->_gbias = bias;
    }

    void gnss_amb_fix::add_data(base_allpar *pars)
    {
        this->_gpar = pars;
    }

    void gnss_amb_fix::add_data(gnss_all_obj *objs)
    {
        this->_gobj = objs;
    }

    gnss_amb_fix::~gnss_amb_fix()
    {
        delete _evaluator_EWL;
        _evaluator_EWL = nullptr;

        delete _evaluator_WL;
        _evaluator_WL = nullptr;

        delete _evaluator_NL;
        _evaluator_NL = nullptr;
    }

    bool gnss_amb_fix::ProcessBatch(base_time &beg, base_time &end)
    {
        return false;
    }

    double gnss_amb_fix::getWaveLen(std::string sat, std::string type)
    {
        gnss_data_obs_manager *gnss = new gnss_data_obs_manager(_spdlog);
        double l = 0;
        if (sat.substr(0, 1) == "G")
        {
            gnss->sat("G");
            if (type == "L1")
                l = gnss->wavelength(BAND_1);
            if (type == "L2")
                l = gnss->wavelength(BAND_2);
            if (type == "L3")
                l = gnss->wavelength(BAND_5);
            if (type == "WL")
                l = gnss->wavelength_WL(BAND_1, BAND_2);
            if (type == "NL")
                l = gnss->wavelength_NL(BAND_1, BAND_2);
            if (type == "EWL")
                l = gnss->wavelength_WL(BAND_2, BAND_5);
        }
        else if (sat.substr(0, 1) == "E")
        {
            gnss->sat("E");
            if (type == "L1")
                l = gnss->wavelength(BAND_1);
            if (type == "L2")
                l = gnss->wavelength(BAND_5);
            if (type == "L3")
                l = gnss->wavelength(BAND_6);
            if (type == "WL")
                l = gnss->wavelength_WL(BAND_1, BAND_5);
            if (type == "NL")
                l = gnss->wavelength_NL(BAND_1, BAND_5);
            if (type == "EWL")
                l = gnss->wavelength_WL(BAND_5, BAND_6);
        }
        else if (sat.substr(0, 1) == "C")
        {
            gnss->sat("C");
            if (type == "L1")
                l = gnss->wavelength(BAND_2);
            if (type == "L2")
                l = gnss->wavelength(BAND_7);
            if (type == "L3")
                l = gnss->wavelength(BAND_6);
            if (type == "WL")
                l = gnss->wavelength_WL(BAND_2, BAND_7);
            if (type == "NL")
                l = gnss->wavelength_NL(BAND_2, BAND_7);
            if (type == "EWL")
                l = gnss->wavelength_WL(BAND_7, BAND_6);
        }

        else
        {
        }
        delete gnss;
        return l;
    }

}
