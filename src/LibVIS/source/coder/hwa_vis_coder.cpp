#include "hwa_vis_coder.h"
#include "hwa_vis_data.h"

using namespace hwa_set;
using namespace std;

hwa_vis::vis_coder::vis_coder(set_base* s, string version, int sz) : base_coder(s, version, sz),
_ts(0.1)
{
}

int hwa_vis::vis_coder::decode_head(char* buff, int sz, vector<string>& errmsg)
{
    _mutex.lock();

    // no header expected, but fill the buffer
    base_coder::_add2buffer(buff, sz);
    _mutex.unlock(); return -1;
}

int hwa_vis::vis_coder::decode_data(char* buff, int sz, int& cnt, vector<string>& errmsg)
{
    _mutex.lock();

    if (base_coder::_add2buffer(buff, sz) == 0) { _mutex.unlock(); return 0; };

    int tmpsize = 0;
    string line;
    string img0_path = "image//img0//data//";
    string img1_path = "image//img1//data//";

    while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
    {
        for (int i = 0; i < line.size(); i++)
        {
            if (line[i] == ',' || line[i] == '*' || line[i] == ';')line[i] = ' ';
        }

        stringstream ss(line);
        string path;
        double time;
        ss >> time;
        if (time > 1e11)
            time = time / 1e9;
        ss >> path;
        int idx_jpg = path.find("jpg");
        int idx_png = path.find("png");
        if (idx_jpg == string::npos && idx_png == string::npos) path = path + ".jpg";

        map<string, base_data*>::iterator it = _data.begin();
        while (it != _data.end())
        {
            if (it->second->id_type() == base_data::CAMDATA)
                ((vis_data*)it->second)->add_IMG(time, img0_path + path, img1_path + path);
            it++;
        }
        if (ss.fail())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "warning: incorrect IMG data record: " + ss.str());
            else
                cerr << "warning: incorrect IMU data record: " << ss.str() << endl;
            base_coder::_consume(tmpsize);
            _mutex.unlock(); return -1;
        }
        base_coder::_consume(tmpsize);
        cnt++;

    }

    _mutex.unlock();
    return 0;
}

