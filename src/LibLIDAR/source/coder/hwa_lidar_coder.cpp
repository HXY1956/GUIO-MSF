#include "hwa_lidar_coder.h"
#include "hwa_lidar_data.h"
#include "hwa_set_lidar.h"

hwa_lidar::lidar_coder::lidar_coder(set_base * s, string version, int sz) :base_coder(s, version, sz),
_ts(0.1)
{
    _ts = dynamic_cast<set_lidar*>(s)->ts();
}

int hwa_lidar::lidar_coder::decode_head(char * buff, int sz, vector<string>& errmsg)
{
    // no header expected, but fill the buffer
    base_coder::_add2buffer(buff, sz);
    return -1;
}

//
int hwa_lidar::lidar_coder::decode_data(char * buff, int sz, int & cnt, vector<string>& errmsg)
{
    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string lidar_path = "laser\\data\\";
    string line;
    while ((tmpsize = base_coder::_getline(line, 0))>= 0)
    {
        for (int i = 0; i < line.size(); i++)
        {
            if (line[i] == ',' || line[i] == '*' || line[i] == ';')line[i] = ' ';
        }

        stringstream ss(line);
        string path;
        double time;
        ss >> time;
        if(time>1e11)
        time = time / 1e9;
        ss >> path;
        int idx_bin = path.find("bin");
        int idx_pcd = path.find("pcd");
        if (idx_bin == string::npos && idx_pcd == string::npos)
            path = path + ".bin";
    
        
        map<string, base_data*>::iterator it = _data.begin();
        while (it != _data.end())
        {
            if (it->second->id_type() == base_data::LIDARDATA)
            ((lidar_data*)it->second)->add_LIDAR(time, lidar_path+path);
            it++;
        }
        if (ss.fail())
        {
            //if (_log) _log->comment(1, "lidarfile", "warning: incorrect LIDAR data record: " + ss.str());
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "warning: incorrect LIDAR data record: " + ss.str());
            else
                cerr << "warning: incorrect IMU data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;

    }
    return 0;
}

