#ifndef hwa_base_timesync_h
#define hwa_base_timesync_h

#include <cmath>

#include "hwa_base_log.h"
#include "hwa_base_time.h"

namespace hwa_base
{
    bool time_sync(const base_time &epo, double smp, double scl, base_log spdlog);
    bool time_sync(double dsec, double smp, double scl, base_log spdlog); // Added by Wei Zhang
} // namespace

#endif // # GSYNC_H
