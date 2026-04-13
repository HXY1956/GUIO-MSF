#include "hwa_gnss_data_bias.h"

using namespace std;

namespace hwa_gnss
{
#define DIFF_DCB (86400 * 31) // 31 days

    // constructor
    // ----------
    gnss_data_bias::gnss_data_bias() : base_data()
    {
        _beg = FIRST_TIME;
        _end = LAST_TIME;

        id_type(base_data::BIAS);
        id_group(base_data::GRP_model);
    }
    gnss_data_bias::gnss_data_bias(base_log spdlog) : base_data(spdlog)
    {
        _beg = FIRST_TIME;
        _end = LAST_TIME;

        id_type(base_data::BIAS);
        id_group(base_data::GRP_model);
    }

    // destructor
    // ----------
    gnss_data_bias::~gnss_data_bias() {}

    // get single code bias
    // ----------
    // ----------
    double gnss_data_bias::bias(bool meter)
    {
        if (meter)
            return _val;
        else
            return (_val / CLIGHT) * 1e9;
    }

    // add bias
    // ----------
    void gnss_data_bias::set(const base_time &beg, const base_time &end, double val, GOBS obs1, GOBS obs2)
    {
        _beg = beg;
        _end = end;

        _gobs = obs1;
        _ref = obs2;

        _val = val;
        return;
    }

    // add bias
    // ----------
    void gnss_data_bias::set(double val, GOBS obs1, GOBS obs2)
    {
        _gobs = obs1;
        _ref = obs2;

        _val = val;
        return;
    }

    // set reference signal
    void gnss_data_bias::ref(GOBS ref)
    {
        _ref = ref;
        return;
    }

    // test validity
    bool gnss_data_bias::valid(const base_time &epo)
    {
        bool ret = true;
        ;

        if (epo < _beg || epo > _end)
            ret = false;
        else
            ret = true;
        return ret;
    }

} // namespace
