#ifndef hwa_gnss_model_ephplan_H
#define hwa_gnss_model_ephplan_H

#include "hwa_gnss_model_eop80.h"
#include "Eigen/Eigen"

namespace hwa_gnss
{
    class gnss_model_ephplan
    {
    public:
        /** @brief default constructor. */
        gnss_model_ephplan();

        /** @brief default destructor. */
        virtual ~gnss_model_ephplan();

        /** @brief Sun position. */
        Triple sunPos(double mjd, bool itrf = true);

        /** @brief Moon position. */
        Triple moonPos(double mjd);

        /** @brief Greenwich Mean Sidereal Time. */
        double gmst(double mjd);

    protected:
        /** @brief Frac part of double. */
        double frac(double x);

        gnss_model_eop80 _eop; ///< eop

    private:
        std::map<double, Triple> _record_itrf_sunpos;
    };
}

#endif
