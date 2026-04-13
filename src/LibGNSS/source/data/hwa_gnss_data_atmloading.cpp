#include "hwa_gnss_data_atmloading.h"

namespace hwa_gnss
{
    gnss_data_atmloading::gnss_data_atmloading()
    {
        id_type(base_data::ATL);
    }

    //Destructor
    gnss_data_atmloading::~gnss_data_atmloading()
    {
    }

    double gnss_data_atmloading::lat()
    {
        double tmp = _lat;
        return tmp;
    }

    double gnss_data_atmloading::lon()
    {
        double tmp = _lon;
        return tmp;
    }

    Matrix gnss_data_atmloading::data()
    {
        Matrix tmp = _data;
        return tmp;
    }

    // Set data
    // -------------
    void gnss_data_atmloading::setdata(const double &lon, const double &lat, const Matrix &data)
    {
        _lon = lon;
        _lat = lat;
        _data = data;
        return;
    }

} // namespace
