#ifndef hwa_set_ppp_h
#define hwa_set_ppp_h

#include "hwa_set_amb.h"
#include "hwa_set_gbase.h"
#include "hwa_set_gen.h"
#include "hwa_set_gflt.h"
#include "hwa_set_gproc.h"
#include "hwa_set_inp.h"
#include "hwa_set_npp.h"
#include "hwa_set_rec.h"
#include "hwa_set_turboedit.h"
#include "hwa_set_param.h"
#include "hwa_set_out.h"

namespace hwa_set {
    class set_ppp : virtual public set_amb,
    virtual public set_gnss,
    virtual public set_gen,
    virtual public set_flt,
    virtual public set_gproc,
    virtual public set_inp,
    virtual public set_npp,
    virtual public set_rec,
    virtual public set_turboedit,
    virtual public set_out,
    virtual public set_par
    {
    public:
        set_ppp();
        ~set_ppp();

        void check() override;                                 // settings check
        void help() override;                                  // settings help

    protected:

    private:


    };
}

#endif
