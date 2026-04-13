#include "hwa_gnss_data_recleo.h"

namespace hwa_gnss
{
    gnss_data_rec_leo::gnss_data_rec_leo() : gnss_data_rec()
    {
        id_type(REC_LEO);

        _overwrite = true;
    }

    gnss_data_rec_leo::gnss_data_rec_leo(base_log spdlog) : gnss_data_rec(spdlog)
    {
        id_type(REC_LEO);
        _overwrite = true;
    }
    gnss_data_rec_leo::~gnss_data_rec_leo()
    {
    }

}