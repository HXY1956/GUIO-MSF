#include "hwa_gnss_data_ambinp.h"
#include "hwa_base_time.h"

namespace hwa_gnss
{
    gnss_data_ambinp_head::gnss_data_ambinp_head()
    {
        num_amb = 0;
        interval = 0.0;
    }

    gnss_data_ambinp_head::~gnss_data_ambinp_head()
    {
    }

    gnss_data_ambinp::gnss_data_ambinp()
    {

        id_type(base_data::AMBINP);
    }

    gnss_data_ambinp::gnss_data_ambinp(base_log spdlog) : base_data(spdlog)
    {

        id_type(base_data::AMBINP);
    }

    gnss_data_ambinp::~gnss_data_ambinp()
    {
    }

    void gnss_data_ambinp::add_hdinfo(gnss_data_ambinp_head *hd)
    {
        _ambinp_hd = hd;
    }

    void gnss_data_ambinp::add_ambpar(base_par ambpar)
    {
        _amb.push_back(ambpar);
    }

    void gnss_data_ambinp::add_neq(std::string parname, std::vector<double> &q)
    {
        _par_list.push_back(parname);
        _neq.push_back(q);
    }

    void gnss_data_ambinp::get_hdinfo(gnss_data_ambinp_head *hd)
    {
        hd = _ambinp_hd;
    }

    void gnss_data_ambinp::get_allambpar(std::vector<base_par> &ambpar)
    {
        ambpar = _amb;
    }

    void gnss_data_ambinp::get_neq(std::vector<std::string> &parlist, std::vector<std::vector<double>> &neq)
    {
        parlist = _par_list;
        neq = _neq;
    }

} //namespace