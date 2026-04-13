/**
*
* @file     gerror.h
* @brief    Storage the ion files data
*/

#ifndef hwa_gnss_data_ERROR_H
#define hwa_gnss_data_ERROR_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
*@brief     Class for storaging ambinp file data
*/
    class gnss_error_record
    {
    public:
        gnss_error_record();
        virtual ~gnss_error_record();


    public:
        base_time epoch;

        double obsP = 0.0;
        double obsL = 0.0;
        double pco = 0.0;
        double ion = 0.0;
        double trop = 0.0;
        double clk = 0.0;
        double satPos[3] = { 0.0 };
        double ifb = 0.0;
        double ifcb = 0.0;
    };
}
#endif
