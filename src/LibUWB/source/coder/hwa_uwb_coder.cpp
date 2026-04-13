#include "hwa_uwb_coder.h"
#include "hwa_uwb_data.h"
#include "hwa_set_uwb.h"
#include <fstream>

using namespace std;

hwa_uwb::uwb_coder::uwb_coder(hwa_set::set_base* s, int sz) :
    base_coder(s, sz), _t(),
    _order("nooploop")
{
    _tt = _secs = 0.0;
    _year = _mon = _day = _hour = _mins = 0;
    _order = dynamic_cast<hwa_set::set_uwb*>(s)->order();
}

int hwa_uwb::uwb_coder::decode_head(char* buff, int sz, vector<string>& errmsg)
{
    _mutex.lock();

    // no header expected, but fill the buffer
    base_coder::_add2buffer(buff, sz);
    _mutex.unlock(); return -1;
}

int hwa_uwb::uwb_coder::decode_data(char* buff, int sz, int& cnt, vector<string>& errmsg)
{

    _mutex.lock();

    if (_order == "rangenet")
    {
        decode_rangenet(buff, sz, cnt, errmsg);
    }
    else if (_order == "tdis")
    {
        decode_tdis(buff, sz, cnt, errmsg);
    }
    else if (_order == "nooploop")
    {
        decode_nooploop(buff, sz, cnt, errmsg);
    }
    else if (_order == "viral")
    {
        decode_viral(buff, sz, cnt, errmsg);
    }
    else if (_order == "simulation") {
        decode_simulation(buff, sz, cnt, errmsg);
    }

    _mutex.unlock();
    return 0;
}

int hwa_uwb::uwb_coder::decode_rangenet(char* buff, int sz, int& cnt, vector<string>& errmsg)
{
    _mutex.lock();

    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string line;  string str;

    while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
    {
        stringstream ss(line);
        UWB_NODE tmp;
        string id;

        if (line[0] == '#')
        {
            base_coder::_consume(tmpsize);
            continue;
        }

        if (line[0] == '>')
        {
            ss >> str >> _year >> _mon >> _day >> _hour >> _mins >> _secs;
            _t.from_ymdhms(_year, _mon, _day, _hour, _mins, _secs);
            base_coder::_consume(tmpsize);
            continue;
        }
        ss >> str >> id >> str >> tmp.range >> str >> str >> str >> str >> str
            >> str >> str >> str >> str >> str >> str >> str >> str >> str >> str
            >> str >> str >> str >> str >> str >> str >> tmp.successrate;
        tmp.range /= 1000; tmp.successrate /= 100;

        map<string, base_data*>::iterator it = _data.begin();
        while (it != _data.end())
        {
            if (it->second->id_type() == base_data::UWBDATA)
            {
                _tt = _t.sow() + _t.dsec();
                ((uwb_data*)it->second)->add_uwb(_tt, id, tmp);
            }
            it++;
        }
        if (ss.fail()) {

            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "uwbfile", "warning: incorrect UWB data record: " + ss.str());
            else       cerr << "warning: incorrect UWB data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            _mutex.unlock(); return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;
    }

    _mutex.unlock();
    return 0;
}

int hwa_uwb::uwb_coder::decode_tdis(char* buff, int sz, int& cnt, vector<string>& errmsg)
{
    _mutex.lock();

    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string line;  string str;

    while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
    {
        stringstream ss(line);
        UWB_NODE tmp;
        string id;

        if (line[0] == '#')
        {
            base_coder::_consume(tmpsize);
            continue;
        }

        if (line[0] == '>')
        {
            ss >> str >> _year >> _mon >> _day >> _hour >> _mins >> _secs;
            _t.from_ymdhms(_year, _mon, _day, _hour, _mins, _secs);
            base_coder::_consume(tmpsize);
            continue;
        }

        if (line[0] == '!')
        {
            ss >> str >> id;
            _node_crt = id;
            base_coder::_consume(tmpsize);
            continue;
        }

        ss >> _tt >> tmp.range;

        map<string, base_data*>::iterator it = _data.begin();
        while (it != _data.end())
        {
            if (it->second->id_type() == base_data::UWBDATA)
            {
                ((uwb_data*)it->second)->add_uwb(_tt, _node_crt, tmp);
            }
            it++;
        }
        if (ss.fail()) {

            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "uwbfile", "warning: incorrect UWB data record: " + ss.str());
            else       cerr << "warning: incorrect UWB data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            _mutex.unlock(); return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;
    }

    _mutex.unlock();
    return 0;
}

int hwa_uwb::uwb_coder::decode_nooploop(char* buff, int sz, int& cnt, vector<string>& errmsg)
{
    _mutex.lock();

    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string line;  double str;
    char c; int id_int;
    int iter = 0;

    while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
    {
        stringstream ss(line);
        UWB_NODE tmp;
        string id;

        if (line[0] == '#')
        {
            base_coder::_consume(tmpsize);
            continue;
        }

        ss >> str;    //local time(s)
        ss >> c >> _tt; _tt = _tt / 1000.0;  //systime(s)

        ss >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str;   //systime, var, pos, vel
        ss >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str >> c >> str;   //g3, a3, att3, q4, voltage
        ss >> c >> iter;  //node num
        for (int i = 0; i < iter; i++)
        {
            ss >> c >> id_int;   //role
            if (id_int == 1)
            {
                ss >> c >> id_int;   //id
                stringstream ss1;
                ss1 << id_int;
                id = ss1.str(); _node_crt = id;
                ss >> c >> tmp.range;
                ss >> c >> tmp.fpRSSI;
                ss >> c >> tmp.rxRSSI;
                tmp.SNR = tmp.rxRSSI + 86.9897;    //Bandwidth=500MHz, 10*log10(Bandwidth)
            }
            else
            {
                ss >> c >> str >> c >> str >> c >> str >> c >> str;
                i--;
                continue;
            }
            map<string, base_data*>::iterator it = _data.begin();
            while (it != _data.end())
            {
                if (it->second->id_type() == base_data::UWBDATA)
                {
                    //_tt = _t.sow() + _t.dsec();
                    /*cout<< str << " "<< year << " " << mon << " " << day << " " << hour << " " << mins << " "
                        << secs << " " << t.sow()<<" "<< t.dsec() << endl;*/
                    ((uwb_data*)it->second)->add_uwb(_tt, _node_crt, tmp);
                }
                it++;
            }
        }
        if (ss.fail()) {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "uwbfile", "warning: incorrect UWB data record: " + ss.str());
            else       cerr << "warning: incorrect UWB data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            _mutex.unlock(); return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;
    }

    _mutex.unlock();
    return 0;
}

int hwa_uwb::uwb_coder::decode_viral(char* buff, int sz, int& cnt, vector<string>& errmsg)
{
    _mutex.lock();

    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string line;  double str;
    char c; int id_int;
    int iter = 0;

    while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
    {

        stringstream ss(line);
        UWB_NODE tmp;
        string id;

        if (line[0] == '#' || line[0] == 't' || line[0] == 'T')
        {
            base_coder::_consume(tmpsize);
            continue;
        }

        ss >> _tt;  //timeUWB(s)

        ss >> c >> id_int;   //msgId
        ss >> c >> id_int; //requester_id (tag_name)
        ss >> c >> id_int; //responder_id (anchor_name)
        stringstream ss1;
        ss1 << id_int;
        id = ss1.str(); _node_crt = id;
        ss >> c >> id_int; //requester_idx
        ss >> c >> id_int; //responder_idx
        ss >> c >> id_int; //range_status
        ss >> c >> id_int; //antenna ------ todo
        ss >> c >> id_int; //stopwatch_time

        ss >> c >> str; //distance
        ss >> c >> str; //coarse_range
        ss >> c >> str; //filtered_range
        tmp.range = str;

        ss >> c >> str; //distance_err
        ss >> c >> str; //coarse_range_err
        ss >> c >> str; //filtered_range_err
        tmp.noise = str;
        tmp.SNR = -20.0 * log10(tmp.noise);
        tmp.rxRSSI = tmp.SNR - 86.9897;
        tmp.fpRSSI = tmp.SNR - 86.9897;


        map<string, base_data*>::iterator it = _data.begin();
        while (it != _data.end())
        {
            if (it->second->id_type() == base_data::UWBDATA)
            {
                ((uwb_data*)it->second)->add_uwb(_tt, _node_crt, tmp);
            }
            it++;
        }

        if (ss.fail()) {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "uwbfile", "warning: incorrect UWB data record: " + ss.str());
            else       cerr << "warning: incorrect UWB data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            _mutex.unlock(); return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;
    }

    _mutex.unlock();
    return 0;
}


int hwa_uwb::uwb_coder::decode_simulation(char* buff, int sz, int& cnt, vector<string>& errmsg)
{
    _mutex.lock();

    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string line;  double str;
    char c; int id_int;
    int iter = 0;

    while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
    {

        stringstream ss(line);
        UWB_NODE tmp;
        string id;

        if (line[0] == '#' || line[0] == 't' || line[0] == 'T')
        {
            base_coder::_consume(tmpsize);
            continue;
        }

        ss >> _tt;  //timeUWB(s)

        ss >> c >> id_int; //responder_id
        stringstream ss1;
        ss1 << id_int;
        id = ss1.str(); _node_crt = id;

        ss >> c >> str; //range
        tmp.range = str;

        ss >> c >> str; //range_err

        tmp.noise = str;
        tmp.SNR = -20.0 * log10(tmp.noise);
        tmp.rxRSSI = tmp.SNR - 86.9897;
        tmp.fpRSSI = tmp.SNR - 86.9897;


        map<string, base_data*>::iterator it = _data.begin();
        while (it != _data.end())
        {
            if (it->second->id_type() == base_data::UWBDATA)
            {
                ((uwb_data*)it->second)->add_uwb(_tt, _node_crt, tmp);
            }
            it++;
        }

        if (ss.fail()) {
            if (_spdlog) SPDLOG_LOGGER_INFO(_spdlog, "uwbfile", "warning: incorrect UWB data record: " + ss.str());
            else       cerr << "warning: incorrect UWB data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            _mutex.unlock(); return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;
    }

    _mutex.unlock();
    return 0;
}
