
/**
*
  input data
  ----------
  dmjd: modified julian date
  dlat: ellipsoidal latitude in radians
  dlon: longitude in radians
  dhgt: ellipsoidal height in m

  output data
  -----------
  pres: pressure in hPa
  temp: temperature in Celsius
  undu: Geoid undulation in m (from a 9x9 EGM based model)
*/

#ifndef hwa_gnss_model_gpt_H
#define hwa_gnss_model_gpt_H

#include <iostream>
#include <string.h>
#include <math.h>

namespace hwa_gnss
{

    /** @brief class for gnss_model_gpt. */
    class gnss_model_gpt
    {

    public:
        /** @brief default constructor. */
        gnss_model_gpt(){};

        /** @brief default destructor. */
        ~gnss_model_gpt(){};

        /**
        *@brief       GPT empirical model v1
        * dlat, dlon --> RADIANS !
        */
        int gpt_v1(double dmjd, double dlat, double dlon, double dhgt,
                   double &pres, double &temp, double &undu);

    protected:
        static double a_geoid[55];
        static double b_geoid[55];
        static double ap_mean[55];
        static double bp_mean[55];
        static double ap_amp[55];
        static double bp_amp[55];
        static double at_mean[55];
        static double bt_mean[55];
        static double at_amp[55];
        static double bt_amp[55];

    private:
    };
}

#endif
