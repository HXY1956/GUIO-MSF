/**
*
* @file        gbancroft.h
* @brief    Purpose: statistical function (1D)
*/

#ifndef hwa_gnss_model_BANCROFT_H
#define hwa_gnss_model_BANCROFT_H

#include "hwa_base_eigendef.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_base_mutex.h"
#include "hwa_set_base.h"

namespace hwa_gnss
{

    int gbancroft(const Matrix &BBpass, Vector &pos);

    inline double lorentz(const Vector &aa, const Vector &bb);

} // namespace

#endif
