#include "hwa_gnss_data_opl.h"

namespace hwa_gnss
{
    //Constructor
    gnss_data_opl::gnss_data_opl(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::OPL);
    }

    //Destructor
    gnss_data_opl::~gnss_data_opl()
    {
    }

    void gnss_data_opl::setdata(double a, double b, double c, double d, double e, double f, double g, double h)
    {
        lon = a;
        lat = b;
        rne_r[0] = c;
        rne_i[0] = d;
        rne_r[1] = e;
        rne_i[1] = f;
        rne_r[2] = g;
        rne_i[2] = h;
    }
}