#ifndef hwa_nav_H
#define hwa_nav_H

#include "hwa_gnss_data_Eph.h"
#include "hwa_gnss_sys.h"
#include <vector>
#include <set>

using namespace hwa_base;
using namespace hwa_set;

#define MAX_rinexn_REC 29 ///< maximum number of RINEXN records for any system !!
#define MAX_nav_TIMEDIFF 3600 * 2 ///< NAV GNS valitity interval [s]
#define MAX_GPS_TIMEDIFF 3600 * 2 ///< NAV GPS validity interval [s]
#define MAX_GLO_TIMEDIFF 60 * 17  ///< NAV GLO validity interval [s]
#define MAX_GAL_TIMEDIFF 3600 * 3 ///< NAV GAL validity interval [s]
#define MAX_BDS_TIMEDIFF 3600     ///< NAV BDS validity interval [s]
#define MAX_SBS_TIMEDIFF 360      ///< NAV SBS validity interval [s]
#define MAX_IRN_TIMEDIFF 3600 * 2 ///< NAV IRN validity interval [s]
#define MAX_QZS_TIMEDIFF 3600     ///< NAV QZS validity interval [s]

namespace hwa_gnss
{
    /** @brief navdata. */
    typedef double gnss_data_navDATA[MAX_rinexn_REC];

    /** @brief Class for navigation data storing. */
    class gnss_data_nav : public gnss_data_eph
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_nav();
        explicit gnss_data_nav(hwa_base::base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_nav();

        /** @brief get GNSS NAV validity (half-interval) [s]. */
        static int nav_validity(GSYS gs);

        /** @brief convert data to nav. */
        virtual int data2nav(std::string sat, const hwa_base::base_time &ep, const gnss_data_navDATA &data) { return -1; }

        /** @brief convert nav to data. */
        virtual int nav2data(gnss_data_navDATA &data) { return -1; }

        /** @brief get iod. */
        virtual int iod() const { return -1; }

        /** @brief get rec. */
        virtual int rec() const { return MAX_rinexn_REC; }

        /** @brief get health. */
        virtual bool healthy() const override;

        /** @brief convert health to std::string. */
        virtual std::string health_str() const override { return _health_str(); }

        /** @brief get chk. */
        virtual int chk(std::set<std::string> &msg) { return 1; }

        /** @brief get the number of frequency. */
        virtual int freq_num() const { return 255; }

    protected:
        /** @brief get the health state. */
        virtual bool _healthy() const { return true; }

        /** @brief convert health to std::string. */
        virtual std::string _health_str() const { return ""; }

    private:
    };

} // namespace

#endif
