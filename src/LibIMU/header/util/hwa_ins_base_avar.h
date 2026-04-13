#ifndef hwa_ins_base_avar_h
#define hwa_ins_base_avar_h
#include "hwa_ins_base_utility.h"

namespace hwa_ins
{
    class ins_avar
    {
    public:

        explicit ins_avar();

        ins_avar(int n, double ts, const Vector& r0, const Vector& tau);

        void init(int n, double ts, const Vector& r0, const Vector& tau);

        void update(const Vector& r);

        double operator()(int i)const;

    public:
        int n;            /// number of type
        double ts;        /// interval
        Vector flag, R, Rmax, Rmin, tstau, r0;

    };
}
#endif