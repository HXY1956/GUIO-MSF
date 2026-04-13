#include "hwa_gnss_data_upd.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_updrec::gnss_data_updrec()
    {
        obj = " ";
        npoint = 0;
        value = 0.0;
        sigma = 1E4;
        isRef = false;
    }

    gnss_data_upd::gnss_data_upd() : base_data()
    {
        id_type(base_data::UPD);
        _ewl_flag = base_time(EWL_IDENTIFY);
        _ewl24_flag = base_time(EWL24_IDENTIFY);
        _ewl25_flag = base_time(EWL25_IDENTIFY);
        _wl_flag = base_time(WL_IDENTIFY);
        _valid_beg[UPDTYPE::EWL_EPOCH] = FIRST_TIME;
        _valid_beg[UPDTYPE::EWL] = FIRST_TIME;
        _valid_beg[UPDTYPE::EWL24] = FIRST_TIME;
        _valid_beg[UPDTYPE::EWL25] = FIRST_TIME;
        _valid_beg[UPDTYPE::WL] = FIRST_TIME;
        _valid_beg[UPDTYPE::NL] = FIRST_TIME;
        _valid_beg[UPDTYPE::IFCB] = FIRST_TIME;
        _tend = FIRST_TIME;
        _est_upd_type = UPDTYPE::NONE;
        _wl_epo_mode = false;
        _wait_stream = true;
    }
    gnss_data_upd::gnss_data_upd(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::UPD);

        _ewl_flag = base_time(EWL_IDENTIFY);
        _ewl24_flag = base_time(EWL24_IDENTIFY);
        _ewl25_flag = base_time(EWL25_IDENTIFY);
        _wl_flag = base_time(WL_IDENTIFY);

        _valid_beg[UPDTYPE::EWL_EPOCH] = FIRST_TIME;
        _valid_beg[UPDTYPE::EWL] = FIRST_TIME;
        _valid_beg[UPDTYPE::EWL24] = FIRST_TIME;
        _valid_beg[UPDTYPE::EWL25] = FIRST_TIME;
        _valid_beg[UPDTYPE::WL] = FIRST_TIME;
        _valid_beg[UPDTYPE::NL] = FIRST_TIME;
        _valid_beg[UPDTYPE::IFCB] = FIRST_TIME;

        _tend = FIRST_TIME;

        _est_upd_type = UPDTYPE::NONE;
        _wl_epo_mode = false;

        _wait_stream = true;
    }

    /** @brief default destructor. */
    gnss_data_upd::~gnss_data_upd()
    {
        _upd.clear();
    }

    void gnss_data_upd::add_sat_upd(UPDTYPE upd_type, base_time epoch, std::string prn, gnss_data_updrec one_sat_upd)
    {
        _upd[upd_type][epoch][prn] = std::make_shared<gnss_data_updrec>(one_sat_upd);
        if (upd_type == UPDTYPE::WL && _upd[upd_type].size() > 1)
            wl_epo_mode(true);
    }

    void gnss_data_upd::add_epo_upd(UPDTYPE upd_type, base_time epoch, one_epoch_upd one_epo_upd)
    {
        _upd[upd_type][epoch] = one_epo_upd;
    }

    void gnss_data_upd::set_est_updtype(UPDTYPE mode)
    {
        _est_upd_type = mode;
    }

    UPDTYPE gnss_data_upd::get_est_updtype()
    {
        return _est_upd_type;
    }

    bool gnss_data_upd::upd_usable(const base_time &t, const std::string &str)
    {
        try
        {
            bool upd_usable;
            if (_est_upd_type == UPDTYPE::NL)
            {
                upd_usable = (_upd[_est_upd_type][t][str]->sigma < 0.100 && _upd[_est_upd_type][t][str]->npoint >= 3) || _upd[_est_upd_type][t][str]->isRef || _upd[_est_upd_type][t][str]->sigma <= 1e-4;
            }
            else
            {
                upd_usable = (_upd[_est_upd_type][t][str]->sigma < 0.120 && _upd[_est_upd_type][t][str]->npoint >= 2) || _upd[_est_upd_type][t][str]->isRef || _upd[_est_upd_type][t][str]->sigma <= 5e-4;
            }

            return upd_usable;
        }
        catch (...)
        {
            std::cout << "ERROR:gnss_data_upd::upd_usable wrong!!!" << std::endl;
            throw(-1);
        }
    }

    void gnss_data_upd::re_init_upd(const UPDTYPE &upd_type, const base_time &t, std::string str)
    {
        gnss_data_updrec one_upd;
        one_upd.npoint = 0;
        one_upd.value = 0.0;
        one_upd.sigma = 1E4;
        one_upd.isRef = false;
        gnss_data_upd::add_sat_upd(upd_type, t, str, one_upd);
    }

    one_epoch_upd &gnss_data_upd::get_epo_upd(const UPDTYPE &upd_type, const base_time &t)
    {
        if (_upd[upd_type].find(t) != _upd[upd_type].end())
        {
            return _upd[upd_type][t];
        }
        else
        {
            auto it_epo = _upd[upd_type].lower_bound(t);
            if (it_epo != _upd[upd_type].end())
            {
                if (it_epo->first.diff(t) >= 30)
                {
                    _wait_stream = false;
                    return _null_epoch_upd;
                }
                else
                {
                    return it_epo->second;
                }
            }
            else
            {
                return _null_epoch_upd;
            }

            // Second
            //auto it_epo = _upd[upd_type].lower_bound(t);
            //if (it_epo == _upd[upd_type].end())
            //{
            //    if (it_epo != _upd[upd_type].begin())
            //    {
            //        it_epo--;
            //        if (t.diff(it_epo->first) < 30) return it_epo->second;
            //    }
            //    return _null_epoch_upd;
            //}
            //else
            //{
            //    auto it_epo2 = it_epo;
            //    if (it_epo2 != _upd[upd_type].begin()) it_epo2--;
            //    double diff1 = it_epo->first.diff(t);
            //    double diff2 = t.diff(it_epo2->first);
            //    if (diff1 >= 30 && diff2 >= 30) {
            //        return _null_epoch_upd;
            //    }
            //    else {
            //        it_epo = diff1 <= diff2 ? it_epo : it_epo2;
            //        return it_epo->second;
            //    }
            //}

            // First
            //auto it_epo = _upd[upd_type].begin();
            //for (; it_epo != _upd[upd_type].end(); it_epo++)
            //{
            //    if (it_epo->first == _ewl_flag || it_epo->first == _wl_flag) continue;
            //    if (it_epo->first > t && it_epo->first.diff(t) < 30) break;
            //    if (it_epo->first < t && t.diff(it_epo->first) < 30) break;
            //}
            //if (it_epo == _upd[upd_type].end())
            //{
            //    return _null_epoch_upd;
            //}
            //else
            //{
            //    return it_epo->second;
            //}
        }
    }

    void gnss_data_upd::reset_upd(const UPDTYPE &upd_type, const base_time &t, const std::string &str, const double &value,
                           const double &sigma, const int &npoint, const double &ratio)
    {
        _upd[upd_type][t][str]->npoint = npoint;
        _upd[upd_type][t][str]->value = value;
        _upd[upd_type][t][str]->sigma = sigma;
        _upd[upd_type][t][str]->ratio = ratio;
    }

    void gnss_data_upd::reset_upd_value(const UPDTYPE &upd_type, const base_time &t, const std::string &str, const double &value)
    {
        _upd[upd_type][t][str]->value = value;
    }

    void gnss_data_upd::copy_upd(const UPDTYPE &upd_type, const base_time &pre_t, const std::string &str, const base_time &current_t,
                          const bool &is_first, const bool &is_site)
    {
        if (is_first == true)
        {
            gnss_data_upd::re_init_upd(upd_type, current_t, str);
        }
        else
        {
            gnss_data_updrec one_upd;
            one_upd.npoint = _upd[upd_type][pre_t][str]->npoint;
            one_upd.value = _upd[upd_type][pre_t][str]->value;
            one_upd.sigma = _upd[upd_type][pre_t][str]->sigma;
            one_upd.isRef = _upd[upd_type][pre_t][str]->isRef;
            if (is_site)
                _upd[upd_type][pre_t].erase(str);
            gnss_data_upd::add_sat_upd(upd_type, current_t, str, one_upd);
        }
        return;
    }

    void gnss_data_upd::set_valid_beg(const UPDTYPE &upd_type, const base_time &t)
    {
        if (_valid_beg[upd_type] > t)
            _valid_beg[upd_type] = t;
    }

    bool gnss_data_upd::upd_avail(const base_time &now)
    {
        //one_epoch_upd epoch_upd_wl = this->get_epo_upd(UPDTYPE::WL, now);
        //one_epoch_upd epoch_upd_nl = this->get_epo_upd(UPDTYPE::NL, now);
        //if ((epoch_upd_wl.size() < 5 || epoch_upd_nl.size() < 5) && _wait_stream) {
        //    return false;
        //}
        //else
        //{
        //    _wait_stream = true;
        //    return true;
        //}

        if (_tend >= now + _upd_intv)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void gnss_data_upd::set_end(const base_time &now)
    {
        if (now > _tend)
        {
            _upd_intv = now.diff(_tend);
            _tend = now;
        }
    }

} //namespace