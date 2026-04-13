#include "hwa_gnss_data_ion.h"
#include "hwa_gnss_coder_ion.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_data_ion::gnss_data_ion() : base_data()
    {
        id_type(base_data::ION);
    }

    gnss_data_ion::gnss_data_ion(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::ION);
    }

    hwa_gnss::gnss_data_ion::~gnss_data_ion()
    {
    }

    void hwa_gnss::gnss_data_ion::set_header(const base_time &beg, const base_time &end, const double &intv, const int &num_amb, const std::set<std::string> &sats, const std::set<std::string> &recs)
    {
        this->_beg = beg;
        this->_end = end;
        this->_intv = intv;
        this->_sats = sats;
        this->_recs = recs;
        this->_numb = num_amb;
    }

    void hwa_gnss::gnss_data_ion::get_header(base_time &beg, base_time &end, double &intv, std::set<std::string> &sats, std::set<std::string> &recs)
    {
        beg = this->_beg;
        end = this->_end;
        intv = this->_intv;
        sats = this->_sats;
        recs = this->_recs;
    }

    double gnss_data_ion::get_value(const std::string &rec, const std::string &sat, const base_time &beg, const base_time &end)
    {
        auto key = std::make_pair(rec, sat);
        if (_ions.count(key) > 0)
        {
            auto ions = _ions[key];

            auto iter_find = _ions[key].lower_bound(beg);
            if (iter_find->second.beg <= beg && iter_find->second.end >= end)
                return iter_find->second.value;
            else
                return 0.0;
        }
        return 0.0;
    }

    void hwa_gnss::gnss_data_ion::add_record(const gnss_data_ion_record &record)
    {
        std::string rec = record.rec;
        std::string sat = record.sat;
        base_time beg = record.beg;
        base_time end = record.end;
        auto key = std::make_pair(rec, sat);
        this->_ions[key][beg] = record;

        if (this->_beg > beg)
            this->_beg = beg;
        if (this->_end < end)
            this->_end = end;
        if (this->_recs.count(rec) == 0)
            this->_recs.insert(rec);
        if (this->_sats.count(sat) == 0)
            this->_sats.insert(sat);
        this->_numb++;
    }

    std::map<base_time, gnss_data_ion_record> &gnss_data_ion::get_records(std::string rec, std::string sat)
    {
        //TODO COMMENT
        auto key = std::make_pair(rec, sat);
        // auto idx = _ions.count(key);
        std::map<base_time, gnss_data_ion_record> tmp;
        return _ions.at(key);
        // if (idx > 0)
        //     return _ions[key];
        // else
        //     return tmp;
    }

    gnss_data_ion_record::gnss_data_ion_record()
    {
    }

    hwa_gnss::gnss_data_ion_record::gnss_data_ion_record(const std::string &type, const std::string &rec, const std::string &sat, const double &value, const double &sigma, const base_time &beg, const base_time &end)
    {
        this->ion_type = type; // SION or VION
        this->value = value;
        this->sigma = sigma;
        this->sat = sat;
        this->rec = rec;
        this->beg = beg;
        this->end = end;
    }

    hwa_gnss::gnss_data_ion_record::~gnss_data_ion_record()
    {
    }

}