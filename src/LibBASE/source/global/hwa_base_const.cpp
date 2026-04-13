#include "hwa_base_const.h"

const hwa_base::base_glv hwa_base::glv = hwa_base::base_glv();
hwa_base::base_glv::base_glv(double Re, double f, double wie0, double g0)
{
    this->Re = Re;
    this->f = f;
    this->wie = wie0;
    this->g0 = g0;
    mg = 1.0e-3 * g0;
    ug = 1.0e-6 * glv.g0;
    mgpsHz = mg / sqrt(1.0);
    ugpsHz = ug / sqrt(1.0);
    mgpsh = mg / sqrt(hur);
    ugpsh = ug / sqrt(hur);
}