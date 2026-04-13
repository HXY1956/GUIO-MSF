#include <stdlib.h>
#include <iostream>
#include "hwa_gnss_data_eph.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_common.h"

namespace hwa_gnss
{
    gnss_data_eph::gnss_data_eph()
        : base_data(),
          _sat(""),
          _epoch(base_time::GPS),
          _validity(true),
          _gio_ptr(0)
    {
        _type = EPHGPS;
    }

    gnss_data_eph::gnss_data_eph(hwa_base::base_log spdlog)
        : base_data(spdlog),
          _sat(""),
          _epoch(base_time::GPS),
          _validity(true),
          _gio_ptr(0)
    {
        _type = EPHGPS;
    }
    // destructor
    // ----------
    gnss_data_eph::~gnss_data_eph()
    {
    }

    // gsys
    // ----------
    GSYS gnss_data_eph::gsys() const
    {

        GSYS tmp = GNS;
        if (_valid())
            tmp = gnss_sys::char2gsys(_sat[0]);
        return tmp;
    }

    // gsat
    // ----------
    std::string gnss_data_eph::gsat() const
    {

        std::string tmp = "";
        if (_valid())
            tmp = _sat;
        return tmp;
    }

    // get parameter value
    // ----------
    hwa_map_time_value gnss_data_eph::param(const NAVDATA &n)
    {
        hwa_map_time_value tmp;
        return tmp;
    }

    // get parameter value
    // ----------
    int gnss_data_eph::param(const NAVDATA &n, double val)
    {
        return 0;
    }

    // get parameter value
    // ----------
    bool gnss_data_eph::param_cyclic(const NAVDATA &n)
    {
        if (n == 2 || n == 6)
            return true;
        return false;
    }

    // valid
    // ----------
    bool gnss_data_eph::valid()
    {
        bool tmp = this->_valid();
        return tmp;
    }

    // clean data
    // ----------
    void gnss_data_eph::clear()
    {
        this->_clear();
        return;
    }

    // clean internal function
    // ----------
    void gnss_data_eph::_clear()
    {

        _sat.clear();
        _epoch = hwa_base::FIRST_TIME;
    }

    // clean internal function
    // ----------
    bool gnss_data_eph::_valid() const
    {

        if (!_validity ||
            _sat.empty() ||
            _sat == "" ||
            _epoch == hwa_base::FIRST_TIME)
            return false;

        return true;
    }

    // type std::set
    // ----------
    //void gnss_data_eph::set_types( unsigned int types ){
    //  _types = types;
    //}

    // return type
    // ----------
    //unsigned int gnss_data_eph::types() const
    //{
    // if( (_types & EPH_nav ) == EPH_nav  ) std::cout << "_types.nav   \n";
    // if( (_types & EPH_RTCM) == EPH_RTCM ) std::cout << "_types.rtcm  \n";
    // if( (_types & EPH_SP3 ) == EPH_SP3  ) std::cout << "_types.sp3   \n";
    // if( (_types & EPH_CLK ) == EPH_CLK  ) std::cout << "_types.clk   \n";
    // return _types;
    //}

} // namespace
