#ifndef hwa_gnss_model_eopEOP80_H
#define hwa_gnss_model_eopEOP80_H

#include "hwa_gnss_model_eop.h"

namespace hwa_gnss
{

    /** @brief class for gnss_model_eop80 based on gnss_model_eop. */
    class gnss_model_eop80 : public gnss_model_eop
    {

    public:
        /** @brief default constructor. */
        gnss_model_eop80();

        /** @brief default destructor. */
        virtual ~gnss_model_eop80();

        /** @brief Nutation Matrix. */
        Matrix nutMatrix(double mjd);

        /** @brief Precession Matrix. */
        Matrix precMatrix(double mjd_1);

    protected:
        /** @brief Frac part of double. */
        double frac(double x);
    };
}

#endif
