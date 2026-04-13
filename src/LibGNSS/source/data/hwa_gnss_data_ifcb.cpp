#include "hwa_gnss_data_ifcb.h"

namespace hwa_gnss
{

    /** @brief default constructor. */
    gnss_data_ifcb_rec::gnss_data_ifcb_rec()
    {
        obj = " ";
        npoint = 0;
        value = 0.0;
        sigma = 1E4;
        isRef = false;
    }

    /** @brief default constructor. */
    gnss_data_ifcb::gnss_data_ifcb() : base_data()
    {

        id_type(base_data::IFCB);
    }

    gnss_data_ifcb::gnss_data_ifcb(base_log spdlog) : base_data(spdlog)
    {

        id_type(base_data::IFCB);
    }

    /** @brief default destructor. */
    gnss_data_ifcb::~gnss_data_ifcb()
    {
        _ifcb.clear();
    }

    void gnss_data_ifcb::add_sat_ifcb(base_time epoch, std::string prn, gnss_data_ifcb_rec one_sagnss_coder_ifcb)
    {
        _ifcb[epoch][prn] = std::make_shared<gnss_data_ifcb_rec>(one_sagnss_coder_ifcb);
    }

    void gnss_data_ifcb::add_epo_ifcb(base_time epoch, hwa_map_id_ifcb one_epo_ifcb)
    {
        _ifcb[epoch] = one_epo_ifcb;
    }

    bool gnss_data_ifcb::ifcb_usable(const base_time &t, const std::string &str)
    {
        try
        {
            bool ifcb_usable;
            ifcb_usable = (_ifcb[t][str]->sigma < 0.100 && _ifcb[t][str]->npoint >= 3) || _ifcb[t][str]->isRef || _ifcb[t][str]->sigma <= 1e-4;
            return ifcb_usable;
        }
        catch (...)
        {
            std::cout << "ERROR:gnss_data_ifcb::ifcb_usable wrong!!!" << std::endl;
            throw(-1);
        }
    }

    void gnss_data_ifcb::re_init_icfb(const base_time &t, std::string str)
    {
        gnss_data_ifcb_rec one_ifcb;
        one_ifcb.npoint = 0;
        one_ifcb.value = 0.0;
        one_ifcb.sigma = 1E4;
        one_ifcb.isRef = false;
        gnss_data_ifcb::add_sat_ifcb(t, str, one_ifcb);
    }

    //
    hwa_map_id_ifcb &gnss_data_ifcb::get_epo_ifcb(const base_time &t)
    {
        if (_ifcb.find(t) != _ifcb.end())
            return _ifcb[t];
        //else return _null_epoch_ifcb;
        else
        {
            auto latter = _ifcb.lower_bound(t);
            auto former = latter;
            if (latter == _ifcb.end())
            {
                if (latter != _ifcb.begin())
                {
                    latter--;
                    if (t.diff(latter->first) < 30)
                        return _ifcb[latter->first];
                    else
                        return _null_epoch_ifcb;
                }
                else
                    return _null_epoch_ifcb;
            }
            else
            {
                if (former != _ifcb.begin())
                    former--;
                double diff1 = latter->first.diff(t);
                double diff2 = t.diff(former->first);
                if (diff1 >= 30 && diff2 >= 30)
                    return _null_epoch_ifcb;
                else
                {
                    return (diff1 < diff2 ? _ifcb[former->first] : _ifcb[latter->first]);
                }
            }
        }
    }

    void gnss_data_ifcb::reset_icfb(const base_time &t, const std::string &str, const double &value,
                             const double &sigma, const int &npoint)
    {
        _ifcb[t][str]->npoint = npoint;
        _ifcb[t][str]->value = value;
        _ifcb[t][str]->sigma = sigma;
    }

    void gnss_data_ifcb::reset_icfb_value(const base_time &t, const std::string &str, const double &value)
    {
        _ifcb[t][str]->value = value;
    }

    void gnss_data_ifcb::copy_ifcb(const base_time &pre_t, const std::string &str, const base_time &current_t,
                            const bool &is_first, const bool &is_site)
    {
        if (is_first == true)
        {
            gnss_data_ifcb::re_init_icfb(current_t, str);
        }
        else
        {
            gnss_data_ifcb_rec one_ifcb;
            one_ifcb.npoint = _ifcb[pre_t][str]->npoint;
            one_ifcb.value = _ifcb[pre_t][str]->value;
            one_ifcb.sigma = _ifcb[pre_t][str]->sigma;
            one_ifcb.isRef = _ifcb[pre_t][str]->isRef;
            if (is_site)
                _ifcb[pre_t].erase(str);
            gnss_data_ifcb::add_sat_ifcb(current_t, str, one_ifcb);
        }
        return;
    }

    void gnss_data_ifcb::set_valid_beg(const base_time &t)
    {
        if (_valid_beg > t)
            _valid_beg = t;
    };

} //namespace