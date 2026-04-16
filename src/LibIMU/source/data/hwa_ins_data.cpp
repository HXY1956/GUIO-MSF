#include "hwa_ins_data.h"

using namespace std;

hwa_ins::ins_data::ins_data():base_data()
{

    first = end = 1;
    _imu_num = 1;
    id_type(ID_TYPE::IMUDATA);
}

hwa_ins::ins_data::ins_data(base_log spdlog) : base_data(spdlog)
{
    first = end = 1;
    _imu_num = 1;
    id_type(ID_TYPE::IMUDATA);
}
int hwa_ins::ins_data::add_IMU(const double& t, const Triple& wm, const Triple& vm)
{
    imu_unit tmp = { t,wm,vm };
    _imu_forward.push_back(tmp);
    return 0;
}

int hwa_ins::ins_data::add_IMU(const double& t, const Triple& wm, const Triple& vm, const Triple& mm)
{
    imu_unit tmp = imu_unit(t,wm,vm,mm);
    _imu_forward.push_back(tmp);
    return 0;
}

void hwa_ins::ins_data::sort_IMU()
{
    int id = 0;
    _imu_all_forward.clear();
    for (auto it = _imu_forward.begin() + 1; it != _imu_forward.end(); it++)
    {
        auto it_1 = it - 1;
        _imu_all_forward[id].push_back(*(it - 1));
        if (it->time <= it_1->time)
        {
            id++;
            _imu_num = id + 1;
        }
    }
    _imu_all_forward[id].push_back(*(_imu_forward.end() - 1));
    _imu_forward = _imu_all_forward[0];
    return;
}

void hwa_ins::ins_data::set_ts(double ts)
{
    _ts = ts;
}

bool hwa_ins::ins_data::load(vector<Triple>& wm, vector<Triple>& vm, double & t, double & ts, int nSamples, bool& status)
{
    wm.clear(); vm.clear();

    if (_imu_forward.size() < nSamples) {
        t = 0; 
        status = false;
        return false;
    }
    double _pre_time = t;
    //_pre_time = t;
    for (int j = 0; j < nSamples; j++)
    {
        t = _imu_forward.front().time;
        wm.push_back(_imu_forward.front().gyo);
        vm.push_back(_imu_forward.front().acc);
        _imu_forward.pop_front();
    }
    if (first == 1 ) {
        ts = nSamples * _ts;
        first = 0;
    }
    else ts = fabs(t - _pre_time);
    status = true;
    return true;
}

bool hwa_ins::ins_data::load(int id, vector<Triple>& wm, vector<Triple>& vm, double& t, double& ts, int nSamples, bool& status)
{
    wm.clear(); vm.clear();

    if(first_mimu.find(id)== first_mimu.end())
        first_mimu[id] = 1;

    if (_imu_all_forward[id].size() < nSamples) {
        t = 0;
        status = false;
        return false;
    }
    double _pre_time = t;
    //_pre_time = t;
    for (int j = 0; j < nSamples; j++)
    {
        t = _imu_all_forward[id].front().time;
        wm.push_back(_imu_all_forward[id].front().gyo);
        vm.push_back(_imu_all_forward[id].front().acc);
        _imu_all_forward[id].pop_front();
    }
    if (first_mimu[id] == 1) {
        ts = nSamples * _ts;
        first_mimu[id] = 0;
    }
    else ts = fabs(t - _pre_time);
    status = true;
    return true;
}


bool hwa_ins::ins_data::load(vector<Triple>& wm, vector<Triple>& vm, vector<Triple>& mm, double & t, double & ts, int nSamples, bool& status)
{
    wm.clear(); vm.clear(); mm.clear();

    if (_imu_forward.size() < nSamples){
        t = 0;  status = false; return false;
    }
    double _pre_time = t;
    for (int j = 0; j < nSamples; j++)
    {
        t = _imu_forward.front().time;
        wm.push_back(_imu_forward.front().gyo);
        vm.push_back(_imu_forward.front().acc);
        mm.push_back(_imu_forward.front().mag);
        _imu_forward.pop_front();
    }
    if (first == 1) {
        ts = nSamples * _ts;
        first = 0;
    }
    else ts = fabs(t - _pre_time);
    status = true;
    return true;
}

bool hwa_ins::ins_data::get(vector<Triple>& wm, vector<Triple>& vm, double & t, double & ts, int nSamples, int offset)
{
    wm.clear(); vm.clear();

    if (_imu_forward.size() < nSamples) {
        t = 0;  return false;
    }
    double _pre_time = t;
    for (int j = 0; j < nSamples; j++)
    {
        t = _imu_forward[offset + j].time;
        wm.push_back(_imu_forward[offset + j].gyo);
        vm.push_back(_imu_forward[offset + j].acc);
    }
    if (first == 1) {
        ts = nSamples * _ts;
        first = 0;
    }
    else ts = fabs(t - _pre_time);
    return true;
}

base_time hwa_ins::ins_data::erase_bef(base_time t)
{
    double t_ins = _imu_forward.front().time;
    double t_gnss = t.sow() + t.dsec();

    while (t_ins < t_gnss)
    {
        _imu_forward.pop_front();
        t_ins = _imu_forward.front().time;
    }

    return base_time(t.gwk(), t_ins);
}

base_time hwa_ins::ins_data::erase_bef(int id, base_time t)
{
    double t_ins = _imu_all_forward[id].front().time;
    double t_gnss = t.sow() + t.dsec();
    if(_imu_all_forward[id].size()==0)
        return base_time(t.gwk(), 0.0);

    while (t_ins < t_gnss)
    {
        _imu_all_forward[id].pop_front();
        t_ins = _imu_all_forward[id].front().time;
    }
    return base_time(t.gwk(), t_ins);
}

int hwa_ins::ins_data::size()
{
    return _imu_forward.size();
}

bool hwa_ins::ins_data::available(const base_time& now)
{
    try {
        double t = now.sow() + now.dsec();

        //zzwu according to old_great
        while (_imu_forward.size() < 5) base_time::gmsleep(500);
        if (_imu_forward.size() > 0 && t < _imu_forward.back().time)
            return true;
        else
            return false;
    }

    catch (...)
    {
        return false;
    }
}

double hwa_ins::ins_data::beg_obs()
{
    if (_imu_forward.size() == 0)return 0;
    return _imu_forward.front().time;
}

double hwa_ins::ins_data::end_obs()
{
    if (_imu_forward.size() == 0)return 0;
    return _imu_forward.back().time;
}

double hwa_ins::ins_data::beg_obs_mimu()
{
    double t = _imu_all_forward[0].front().time;
    if (_imu_all_forward.size() == 0)return 0;
    for (auto imui : _imu_all_forward)
    {
        if (imui.second.front().time < t)
            t = imui.second.front().time;
    }
    return t;
}

double hwa_ins::ins_data::end_obs_mimu()
{
    double t = _imu_all_forward[0].back().time;
    if (_imu_forward.size() == 0)return 0;
    for (auto imui : _imu_all_forward)
    {
        if (imui.second.back().time > t)
            t = imui.second.back().time;
    }
    return t;
}

bool hwa_ins::ins_data::reset_mimu_data()
{
    if (_imu_num == 1)
    {
        return false;
    }
    _imu_forward = _imu_all_forward[1];

    for (int imui = 1; imui < _imu_num; imui++)
    {
        _imu_all_forward[imui - 1] = _imu_all_forward[imui];
    }
    _imu_all_forward.erase(_imu_num - 1);
    return true;
}
