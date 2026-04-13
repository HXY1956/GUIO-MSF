/**
* @file        windup.h
* @brief    windup model class
*
*/

#ifndef hwa_gnss_model_windup_H
#define hwa_gnss_model_windup_H

#include <string>

namespace hwa_gnss
{
    /** @brief class for windup model.    */
    class gnss_model_windup
    {
    public:
        /** @brief constructor    */
        gnss_model_windup(){};

        /** @brief default destructor. */
        virtual ~gnss_model_windup(){};

        /** @brief get correnctions of windup. */
        double Correction(const std::string & rec);

    protected:
    };
}

#endif