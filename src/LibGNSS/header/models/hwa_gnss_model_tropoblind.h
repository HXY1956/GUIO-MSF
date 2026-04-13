
/**
*
* @file        gtropoblind.h
* @brief    Purpose: implements troposphere model class (blind models)
*/

#ifndef hwa_gnss_model_tropoBLIND_H
#define hwa_gnss_model_tropoBLIND_H

#include "hwa_gnss_model_tropo.h"
#include "hwa_gnss_model_blindmops.h"
#include "hwa_gnss_model_gpt.h"

namespace hwa_gnss
{
    /** @brief class for gnss_model_blindtropos derive gnss_model_tropo. */
    class gnss_model_blindtropos : public gnss_model_tropo
    {
    public:
        /** @brief constructor _model. */
        gnss_model_blindtropos() { _model = new gnss_model_blindmops(); }

        gnss_model_blindtropos(base_log spdlog) { _model = new gnss_model_blindmops(spdlog); }
        /** @brief default destructor. */
        ~gnss_model_blindtropos() {}

        /** @brief Radians: Ell[0] and Ell[1]. */
        virtual double getZHD(const Triple &ell, const base_time &epo);

        /** @brief Radians: Ell[0] and Ell[1]. */
        virtual double getZWD(const Triple &ell, const base_time &epo);

    private:
        gnss_model_blindmops *_model; ///< model
    };

} // namespace

#endif
