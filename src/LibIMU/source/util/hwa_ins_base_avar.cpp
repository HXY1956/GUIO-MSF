#include "hwa_ins_base_avar.h"
using namespace hwa_ins;

hwa_ins::ins_avar::ins_avar()
{
}

ins_avar::ins_avar(int n, double ts, const Vector& r0, const Vector& tau)
{
    init(n, ts, r0, tau);
}

void hwa_ins::ins_avar::init(int n, double ts, const Vector & r0, const Vector & tau)
{
    this->n = n; this->ts = ts;
    flag = R = Rmax = Rmin = tstau = this->r0 = Vector::Zero(n);
    R = r0.array().square();
    tstau = tau.cwiseInverse()*ts;
    Rmax = 100 * R; Rmin = 0.01*R;
}

void ins_avar::update(const Vector& r)
{
    Vector dr = r - r0;
    Vector dr2 = dr.array().square();
    for (int i = 0; i < n; i++)
    {
        if (dr2[i] > R[i]) R[i] = dr2[i];
        else R[i] = (1 - tstau[i])*R[i] + tstau[i] * dr2[i];
        if (R[i] < Rmin[i]) R[i] = Rmin[i];
        if (R[i] > Rmax[i])
        {
            R[i] = Rmax[i];
            flag[i] = 1;
        }
        else
            flag[i] = 0;
    }
    r0 = r;
}

double ins_avar::operator()(int i)const
{
    return flag[i] ? hwa_base::glv.INF : sqrt(R[i]);
}

