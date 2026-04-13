#include <stdlib.h>
#include <iostream>
#include "hwa_gnss_data_nav.h"

namespace hwa_gnss
{
    // constructor
    // ----------
    gnss_data_nav::gnss_data_nav() : gnss_data_eph()
    {
        id_type(base_data::EPH);
        id_group(base_data::GRP_EPHEM);
    }

    gnss_data_nav::gnss_data_nav(hwa_base::base_log spdlog) : gnss_data_eph(spdlog)
    {
        id_type(base_data::EPH);
        id_group(base_data::GRP_EPHEM);
    }
    // destructor
    // ----------
    gnss_data_nav::~gnss_data_nav() {}

    // get GNSS NAV validity time [s]
    int gnss_data_nav::nav_validity(GSYS gs)
    {
        switch (gs)
        {
        case GPS:
            return MAX_GPS_TIMEDIFF;
        case GLO:
            return MAX_GLO_TIMEDIFF;
        case GAL:
            return MAX_GAL_TIMEDIFF;
        case BDS:
            return MAX_BDS_TIMEDIFF;
        case QZS:
            return MAX_QZS_TIMEDIFF;
        case SBS:
            return MAX_SBS_TIMEDIFF;
        case IRN:
            return MAX_IRN_TIMEDIFF;
        case GNS:
            return MAX_nav_TIMEDIFF;
        default:
            return MAX_nav_TIMEDIFF;
        }
        return MAX_nav_TIMEDIFF;
    }

    // healthy check
    // ----------
    bool gnss_data_nav::healthy() const
    {
        bool tmp = this->_healthy();
        return tmp;
    }

} // namespace
