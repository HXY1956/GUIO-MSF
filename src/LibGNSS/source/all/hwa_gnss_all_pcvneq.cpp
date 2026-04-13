#include "hwa_gnss_all_pcvneq.h"

using namespace std;

namespace hwa_gnss
{
    gnss_all_pcvneq::gnss_all_pcvneq() : base_data()
    {
        id_type(base_data::ALLPCVNEQ);
    }
    gnss_all_pcvneq::gnss_all_pcvneq(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::ALLPCVNEQ);
    }

    gnss_all_pcvneq::~gnss_all_pcvneq()
    {
        _allpcvneq.clear();
    }

    map<string, shared_ptr<gnss_data_pcvneq>> gnss_all_pcvneq::allpcvneq()
    {
        return _allpcvneq;
    }
    void gnss_all_pcvneq::addpcvneq(const string &obj, shared_ptr<gnss_data_pcvneq> pcvneq)
    {
        _allpcvneq[obj] = pcvneq;
    }
    shared_ptr<gnss_data_pcvneq> gnss_all_pcvneq::pcvneq(const string &obj)
    {
        return _allpcvneq[obj];
    }
}