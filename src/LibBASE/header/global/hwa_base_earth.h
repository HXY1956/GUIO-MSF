#pragma once
#include "hwa_base_eigendef.h"
#include "hwa_base_const.h"

namespace hwa_base {
    class base_earth {
    public:
        explicit base_earth(double a0 = glv.Re, double f0 = glv.f, double g0 = glv.g0);
        void Update(const Triple& pos, const Triple& vn);
        Triple v2dp(const Triple& vn, double ts);

    public:
        double a, b;                                         /// major & Minor Axissemi axis 
        double f, e, e2, wie;                                 /// flattening,eccentricity,..,angular of rotation
        double sb, sb2, sb4, cb, tb, sl, cl;                 /// sinB,..,..,cosB,tanB,sinL,cosL
        double RMh, RNh, cbRNh, f_RMh, f_RNh, f_cbRNh;         /// RM+H,RN+H,RMH*cosB
        Triple pos, vn;                             /// position,velocity
        Triple wnie, wnen, wnin, weie, gn, gcc;
        SO3 Cen, Cne;                             /// DCM from n-fram to e-frame
    };
}