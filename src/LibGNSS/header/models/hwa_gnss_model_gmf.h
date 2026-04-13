#ifndef hwa_gnss_model_gmf_H
#define hwa_gnss_model_gmf_H

#include <iostream>
#include <iomanip>
#include <string.h>
#include <math.h>
#include "hwa_base_const.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model_gmf. */
    class gnss_model_gmf
    {

    public:
        /** @brief default constructor. */
        gnss_model_gmf(){};

        /** @brief default destructor. */
        ~gnss_model_gmf(){};

        /**
        *@brief       GMF Global mapping function
        * dlat, dlon --> RADIANS ! 
        */
        int gmf(double dmjd, double dlat, double dlon, double dhgt, double zd,
                double &gmfh, double &gmfw, double &dgmfh, double &dgmfw);

    protected:
        static double ah_mean[55];
        static double bh_mean[55];
        static double ah_amp[55];
        static double bh_amp[55];
        static double aw_mean[55];
        static double bw_mean[55];
        static double aw_amp[55];
        static double bw_amp[55];

    private:
    };
}

#endif
