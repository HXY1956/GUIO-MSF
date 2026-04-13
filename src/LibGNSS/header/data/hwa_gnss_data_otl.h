
/**
*
* @file        gotl.h
* @brief    Purpose: implementation of ocean tide loading
*/

#ifndef hwa_gnss_data_otl_h
#define hwa_gnss_data_otl_h

#include "hwa_base_eigendef.h"
#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_data_otl based base_data. */
    class gnss_data_otl : public base_data
    {
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        /** @brief default constructor. */
        gnss_data_otl();

        gnss_data_otl(base_log spdlog);
        /** @brief default destructor. */
        ~gnss_data_otl();

        /** @brief get the site. */
        std::string site();

        /** @brief get the latitude. */
        double lat();

        /** @brief get the lontitude. */
        double lon();

        /** @brief get the data. */
        Matrix data();

        /** @brief set the data. */
        void setdata(const std::string &site, const double &lon, const double &lat, const Matrix &data);

    private:
        std::string _site; ///< site
        Matrix _data; ///< data
        double _lat;  ///< latitude
        double _lon;  ///< lontitude
    };

} // namespace

#endif
