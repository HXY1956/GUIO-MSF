#include "hwa_gnss_data_ambcon.h"

hwa_gnss::gnss_data_ambcon::gnss_data_ambcon() : base_data()
{
    id_type(base_data::AMBCON);
}

hwa_gnss::gnss_data_ambcon::gnss_data_ambcon(base_log spdlog) : base_data(spdlog)
{
    id_type(base_data::AMBCON);
}
