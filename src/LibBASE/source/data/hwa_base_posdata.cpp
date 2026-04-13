#include "hwa_base_posdata.h"
#include "hwa_base_typeconv.h"

hwa_base::base_posdata::base_posdata()
{
    id_type(LCI_POS);
    _ptr = 0;
}
hwa_base::base_posdata::base_posdata(base_log spdlog) : base_data(spdlog)
{
    id_type(LCI_POS);
    _ptr = 0;
}

hwa_base::base_posdata::~base_posdata()
{
}

int hwa_base::base_posdata::add_pos(const double &t, const Triple &pos, const Triple &v)
{
    data_pos tmp = {t, pos, v};
    _vecpos.push_back(tmp);
    _setpos.insert(tmp);
    return 0;
}

int hwa_base::base_posdata::add_pos(const double &t, const Triple &pos, const Triple &v, const Triple &R_p, const Triple &R_v, const double &pdop, const int &n)
{
    data_pos tmp = {t, pos, v, R_p, R_v, pdop, n};
    _setpos.insert(tmp);
    return 0;
}

int hwa_base::base_posdata::add_pos(const double &t, const Triple &pos, const Triple &v, const Triple &R_p, const Triple &R_v, const double &pdop, const int &n, const bool &state)
{
    data_pos tmp = {t, pos, v, R_p, R_v, pdop, n, state};
    _setpos.insert(tmp);
    return 0;
}

void hwa_base::base_posdata::load(double &t, Triple &pos, Triple &v, bool direction)
{
    if (direction)
    {
        t = _vecpos[_ptr].t;
        pos = _vecpos[_ptr].pos;
        v = _vecpos[_ptr].vn;
        _ptr++;
    }
    else
    {
        _ptr--;
        t = _vecpos[_ptr].t;
        pos = _vecpos[_ptr].pos;
        v = _vecpos[_ptr].vn;
    }
}

void hwa_base::base_posdata::load(double &t, Triple &pos, Triple &v, bool direction, double t1)
{
    data_pos tmp{t1, Triple::Zero(), Triple::Zero()};
    std::set<data_pos>::const_iterator it_index = _setpos.upper_bound(tmp);
    if (it_index == _setpos.end())
    {
        t = 0.0;
        pos = v = Triple::Zero();
    }
    it_index--;
    t = it_index->t;
    pos = it_index->pos;
    v = it_index->vn;
    _ptr++;
}

void hwa_base::base_posdata::load(double &t, Triple &pos, Triple &v, Triple &Qpos, Triple &Qv,
                             double &PDOP, int &nSat, bool direction, double t1)
{
    
    data_pos tmp{t1, Triple::Zero(), Triple::Zero()};
    std::set<data_pos>::const_iterator it_index = _setpos.upper_bound(tmp);
    if (it_index == _setpos.end())
    {
        t = 0.0;
        pos = v = Triple::Zero();
    }
    it_index--;
    t = it_index->t;
    pos = it_index->pos;
    v = it_index->vn;
    PDOP = it_index->PDOP;
    nSat = it_index->nSat;
    Qpos = it_index->Rpos;
    Qv = it_index->Rvn;
    _ptr++;
}

void hwa_base::base_posdata::load(data_pos &data, double t1)
{
    int nSat;
    double t, PDOP;
    Triple pos, v, Qpos, Qv;
    bool state;
    data_pos tmp{t1, Triple::Zero(), Triple::Zero()};
    std::set<data_pos>::const_iterator it_index = _setpos.upper_bound(tmp);
    if (it_index == _setpos.end())
    {
        data = {0};
        return;
    }
    it_index--;
    t = it_index->t;
    pos = it_index->pos;
    v = it_index->vn;
    PDOP = it_index->PDOP;
    nSat = it_index->nSat;
    Qpos = it_index->Rpos;
    Qv = it_index->Rvn;
    state = it_index->amb_state;
    data = {t, pos, v, Qpos, Qv, PDOP, nSat, state};
}

bool hwa_base::base_posdata::IsValid(const double &t, const Triple &pos, const double &tot, const double &threshold)
{
    if (double_eq(t, 0.0) || pos == Triple::Zero())
        return false;
    if (fabs(tot - t) < threshold)
        return true;
    return false;
}

double hwa_base::base_posdata::beg_obs()
{
    if (_setpos.size() == 0)
        return 0;
    return _setpos.begin()->t;
}

double hwa_base::base_posdata::end_obs()
{
    if (_setpos.size() == 0)
        return 0;
    return _setpos.rbegin()->t;
}
