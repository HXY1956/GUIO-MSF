#include "hwa_gnss_all_sum.h"

using namespace std;

namespace hwa_gnss
{
    /** default constructor */
    gnss_all_sum::gnss_all_sum() : base_data()
    {
        id_type(base_data::ALLSUM);
    }

    gnss_all_sum::gnss_all_sum(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::ALLSUM);
    }
    /** default destructor */
    gnss_all_sum::~gnss_all_sum()
    {
    }

    void gnss_all_sum::setSat(const vector<string> &satlist)
    {
        _sat_list = satlist;
    }

    void gnss_all_sum::setSite(const vector<string> &sitelist)
    {
        _site_list = sitelist;
    }

    void gnss_all_sum::setRMS(const map<gnss_data_obscombtype, map<int, map<int, double>>> &rms)
    {
        _rmstable = rms;
    }

    void gnss_all_sum::setRMS_SLR(const map<string, map<int, map<int, double>>> &rms)
    {
        _rmstable_slr = rms;
    }

    void gnss_all_sum::setNEPO(const map<int, map<int, int>> &nepo)
    {
        _nepotbl = nepo;
    }

    void gnss_all_sum::setNEPO_SLR(const map<int, map<int, int>> &nepo)
    {
        _nepotbl_slr = nepo;
    }

    const vector<string> &gnss_all_sum::getSat() const
    {
        return _sat_list;
    }

    const vector<string> &gnss_all_sum::getSite() const
    {
        return _site_list;
    }

    const map<gnss_data_obscombtype, map<int, map<int, double>>> &gnss_all_sum::getRMS() const
    {
        return _rmstable;
    }

    const map<int, map<int, int>> &gnss_all_sum::getNEPO() const
    {
        return _nepotbl;
    }

    const map<string, map<int, map<int, double>>> &gnss_all_sum::getRMS_SLR() const
    {
        return _rmstable_slr;
    }

    const map<int, map<int, int>> &gnss_all_sum::getNEPO_SLR() const
    {
        return _nepotbl_slr;
    }

}
