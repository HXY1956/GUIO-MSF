#include <iostream>
#include "hwa_base_timesync.h"
#include "hwa_base_typeconv.h"

namespace hwa_base
{

    // sampling synchronization filter for epochs (return true if the epoch fits sampling)
    // ----------
    bool time_sync(const base_time &epo, double smp, double scl, base_log spdlog)
    {

        // OLD for >=1Hz only
        // if( int(round(epoch.sod()+epoch.dsec()))%int(sampl) != 0) return false;

        // do filtering with sampling rate
        //    but still consider ROUNDING e.g. possible CLOCK DRIFT cases!
        if (smp > 0)
        {
            // and||sampl >=1Hz (high-rate) requested, assume clock drift well-below the range of decimals of sampling rate
            if (scl > 0 && smp <= 1)
            {
                int smpl = (int)round(scl * smp);
                int iepo = (int)(round(epo.sod() * scl + epo.dsec() * scl)); // synced to .0 day i.e >=1Hz suggests .0 day epochs at least!

                if (smpl == 0)
                    return false; // to be save if mixed high/low-rate sampling occurs
                int resi = iepo % smpl;

#ifdef DEBUG

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, epo.str_ymdhms("high-rate data: ") + base_type_conv::dbl2str(epo.dsec()) + " i:" + base_type_conv::int2str(iepo) + " s:" + base_type_conv::int2str(smpl) + " m:" + base_type_conv::int2str(resi) + " d:" + base_type_conv::int2str(scl));
#endif
                if (resi != 0)
                    return false;
            }

            // inp && sampl < 1Hz, i.e. low-rate data only!
            else if (int(round(epo.sod() + epo.dsec())) % int(smp) != 0)
            {
#ifdef DEBUG
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, epo.str_ymdhms("low-rate data: ") + base_type_conv::dbl2str(epo.dsec()));
#endif
                return false;
            }

#ifdef DEBUG
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, epo.str_ymdhms("ok, epoch fits sampling: ") + base_type_conv::dbl2str(epo.dsec()));
#endif
        }
        return true;
    }

    bool time_sync(double dsec, double smp, double scl, base_log spdlog)
    {

        // OLD for >=1Hz only
        // if( int(round(epoch.sod()+epoch.dsec()))%int(sampl) != 0) return false;
        if (dsec < 0)
        {
            return false;
        }

        // do filtering with sampling rate
        //    but still consider ROUNDING e.g. possible CLOCK DRIFT cases!
        if (smp > 0)
        {
            // and||sampl >=1Hz (high-rate) requested, assume clock drift well-below the range of decimals of sampling rate
            if (scl > 0 && smp <= 1)
            {
                int smpl = (int)round(scl * smp);
                int iepo = (int)(round(dsec)); // synced to .0 day i.e >=1Hz suggests .0 day epochs at least!

                if (smpl == 0)
                    return false; // to be save if mixed high/low-rate sampling occurs
                int resi = iepo % smpl;

#ifdef DEBUG
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, epo.str_ymdhms("high-rate data: ") + base_type_conv::dbl2str(epo.dsec()) + " i:" + base_type_conv::int2str(iepo) + " s:" + base_type_conv::int2str(smpl) + " m:" + base_type_conv::int2str(resi) + " d:" + base_type_conv::int2str(scl));
#endif
                if (resi != 0)
                    return false;
            }

            // inp && sampl < 1Hz, i.e. low-rate data only!
            else if (int(round(dsec)) % int(smp) != 0)
            {
#ifdef DEBUG
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, epo.str_ymdhms("low-rate data: ") + base_type_conv::dbl2str(epo.dsec()));
#endif
                return false;
            }

#ifdef DEBUG
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, epo.str_ymdhms("ok, epoch fits sampling: ") + base_type_conv::dbl2str(epo.dsec()));
#endif
        }
        return true;
    }

} // namespace
