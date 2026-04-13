#ifndef hwa_set_all_h
#define hwa_set_all_h
#include <signal.h>
#include "hwa_set_ins.h"
#include "hwa_set_uwb.h"
#include "hwa_set_vis.h"
#include "hwa_set_tracker.h"
#include "hwa_set_proc.h"
#include "hwa_set_ppp.h"
#include "hwa_set_ign.h"

namespace hwa_set
{
    class set_msf :
        public virtual hwa_set::set_ins,
        public virtual hwa_set::set_uwb,
        public virtual hwa_set::set_vis,
        public virtual hwa_set::set_ppp,
        public virtual hwa_set::set_tracker,
        public virtual hwa_set::set_proc,
        public virtual hwa_set::set_ign
    {
    public:
        set_msf();
        virtual ~set_msf();

        void check();
        void help();

    private:

    };
}
#endif