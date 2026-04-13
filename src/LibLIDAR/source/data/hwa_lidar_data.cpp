#include "hwa_lidar_data.h"

using namespace std;

hwa_lidar::lidar_data::lidar_data() :base_data()
{
    id_type(ID_TYPE::LIDARDATA);
}

int hwa_lidar::lidar_data::add_LIDAR(const double& t, const string& lidar_path)
{
#ifdef BMUTEX   
    boost::mutex::scoped_lock lock(_mutex);
#endif
    _gmutex.lock();

    lidarPath tmp = { t,lidar_path};
    _veclidar.push_back(tmp);

    _gmutex.unlock();
    return 0;
}


bool hwa_lidar::lidar_data::load(const double& imu_t, double& lidar_t, lidarPath& lidar_path)
{
    for (int i = 0; i < _veclidar.size(); i++)
    {
        if (fabs(_veclidar.at(i).t - imu_t) <= 0.005)
        {
            lidar_t = _veclidar.at(i).t;
            lidar_path = _veclidar.at(i);
            _veclidar.erase(_veclidar.begin(), _veclidar.begin() + i + 1);
            break;
        }
        if (_veclidar.at(i).t - imu_t > 0.005) return false;
    }
    return true;
}

int hwa_lidar::lidar_data::size()
{
    return _veclidar.size();
}


