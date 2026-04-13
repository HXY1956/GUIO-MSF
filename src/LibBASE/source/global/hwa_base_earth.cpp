#include "hwa_base_earth.h"

using namespace hwa_base;

base_earth::base_earth(double a0, double f0, double g0)
{
    a = a0; f = f0;
    gn = Triple(0, 0, -g0);
    b = (1 - f) * a;
    wie = glv.wie;
    e = sqrt(a * a - b * b) / a;
    e2 = e * e;
}

void base_earth::Update(const Triple& pos, const Triple& vn)
{
    this->pos = pos;  this->vn = vn;
    sb = sin(pos(0)), cb = cos(pos(0)), tb = sb / cb;
    sb2 = sb * sb, sb4 = sb2 * sb2;
    sl = sin(pos(1)), cl = cos(pos(1));
    Cen << -sl, -sb * cl, cb* cl, cl, -sb * sl, cb* sl, 0, cb, sb;
    Cne = Cen.transpose();
    double sq = 1 - e2 * sb * sb, sq2 = sqrt(sq);
    RMh = a * (1 - e2) / sq / sq2 + pos(2);    f_RMh = 1.0 / RMh;
    RNh = a / sq2 + pos(2);    cbRNh = cb * RNh;  f_RNh = 1.0 / RNh; f_cbRNh = 1.0 / cbRNh;
    wnie << 0, wie* cb, wie* sb;
    weie = Cen * wnie;
    wnen << -vn(1) * f_RMh, vn(0)* f_RNh, wnen(1)* tb;
    wnin = wnie + wnen;
    gn(2) = -(glv.g0 * (1 + 5.27094e-3 * sb2 + 2.32718e-5 * sb4) - 3.086e-6 * pos(2));
    gcc = gn - (wnie + wnin).cross(vn);
}

Triple base_earth::v2dp(const Triple& vn, double ts)
{
    return Triple(vn(1) * f_RMh, vn(0) * f_cbRNh, vn(2)) * ts;
}