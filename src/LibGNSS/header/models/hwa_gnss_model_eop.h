#ifndef hwa_gnss_model_eop_H
#define hwa_gnss_model_eop_H

#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_model_eop
    {

    public:
        /** @brief default constructor. */
        gnss_model_eop();

        /** @brief default destructor. */
        virtual ~gnss_model_eop();

        /** @brief Nutation Matrix. */
        Matrix nutMatrix(double mjd);

        /** @brief Precession Matrix. */
        Matrix precMatrix(double mjd);

        /** @brief Normalize angle into interval 0 - 2pi. */
        double _normangle(double x);
    };

} // namespace

#endif
