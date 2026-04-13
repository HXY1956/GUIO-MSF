#include "hwa_gnss_data_otl.h"

namespace hwa_gnss
{

    //Constructor
    gnss_data_otl::gnss_data_otl() : base_data()
    {
        id_type(base_data::OTL);
    }

    gnss_data_otl::gnss_data_otl(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::OTL);
    }

    //Destructor
    gnss_data_otl::~gnss_data_otl()
    {
    }

    std::string gnss_data_otl::site()
    {
        std::string tmp = _site;
        return tmp;
    }

    Matrix gnss_data_otl::data()
    {
        Matrix tmp = _data;
        return tmp;
    }

    double gnss_data_otl::lat()
    {
        double tmp = _lat;
        return tmp;
    }

    double gnss_data_otl::lon()
    {
        double tmp = _lon;
        return tmp;
    }

    // Set data
    // -------------
    void gnss_data_otl::setdata(const std::string &site, const double &lon, const double &lat, const Matrix &data)
    {
        _site = site;
        _lon = lon;
        _lat = lat;
        _data = data;
        return;
    }

} // namespace
