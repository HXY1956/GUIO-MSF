/**
* @file        gatmloading.h
* @brief    Purpose: implementation of atmospheric pressure loading
*/

#ifndef hwa_gnss_data_atmloading_H
#define hwa_gnss_data_atmloading_H

#include <string>
#include "hwa_base_eigendef.h"
#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_data_otl based base_data. */
    class gnss_data_atmloading : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_atmloading();

        /** @brief default destructor. */
        ~gnss_data_atmloading();

        /** @brief get the latitude. */
        double lat();

        /** @brief get the lontitude. */
        double lon();

        /** @brief get the _data. */
        Matrix data();

        /** @brief set the data. */
        void setdata(const double &lon, const double &lat, const Matrix &data);

    protected:
        double _lat;  ///< latitude
        double _lon;  ///< lontitude
        Matrix _data; ///< data
    };

} // namespace

#endif
