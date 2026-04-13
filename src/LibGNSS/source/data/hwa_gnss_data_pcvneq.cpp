#include "hwa_gnss_data_pcvneq.h"

namespace hwa_gnss
{
    gnss_data_pcvneq::gnss_data_pcvneq()
        : _zen_beg(0.0),
          _zen_end(0.0),
          _dzen(0.0),
          _azi_beg(0.0),
          _azi_end(0.0),
          _dazi(0.0),
          _npar(0)
    {
        id_type(base_data::PCVNEQ);
    }

    gnss_data_pcvneq::gnss_data_pcvneq(base_log spdlog) : _zen_beg(0.0),
                                            _zen_end(0.0),
                                            _dzen(0.0),
                                            _azi_beg(0.0),
                                            _azi_end(0.0),
                                            _dazi(0.0),
                                            _npar(0),
                                            base_data(spdlog)
    {
        id_type(base_data::PCVNEQ);
    }
    void gnss_data_pcvneq::pcv_val(const int &i, const double &val)
    {
        _pcv_val[i] = val;
    }
    void gnss_data_pcvneq::pcv_val(const std::map<int, double> &vals)
    {
        _pcv_val = vals;
    }
    double gnss_data_pcvneq::pcv_val(const int &i)
    {
        return _pcv_val[i];
    }
    std::map<int, double> gnss_data_pcvneq::pcv_val()
    {
        return _pcv_val;
    }
    void gnss_data_pcvneq::pcv_neq(const int &i, const int &j, const double &val)
    {
        _pcv_neq[i][j] = val;
    }
    void gnss_data_pcvneq::pcv_neq(const int &i, const std::map<int, double> &vals)
    {
        _pcv_neq[i] = vals;
    }
    void gnss_data_pcvneq::pcv_neq(const std::map<int, std::map<int, double>> &vals)
    {
        _pcv_neq = vals;
    }
    double gnss_data_pcvneq::pcv_neq(const int &i, const int &j)
    {
        return _pcv_neq[i][j];
    }
    std::map<int, std::map<int, double>> gnss_data_pcvneq::pcv_neq()
    {
        return _pcv_neq;
    }
}