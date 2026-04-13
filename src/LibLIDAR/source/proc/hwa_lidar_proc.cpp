#include "hwa_lidar_proc.h"
#include "hwa_set_lidar.h"
using namespace hwa_lidar;

hwa_lidar::lidar_proc::lidar_proc(set_base* _set)
{
    ///< basic patameters
    ts = dynamic_cast<set_lidar*>(_set)->ts();
    freq = dynamic_cast<set_lidar*>(_set)->freq();
    imu_ts = dynamic_cast<set_lidar*>(_set)->imu_ts();

    ///< initial the parameters
    R_lidar_imu = dynamic_cast<set_lidar*>(_set)->R_lidar_imu();
    t_lidar_imu = dynamic_cast<set_lidar*>(_set)->t_lidar_imu();
    T_lidar_imu = dynamic_cast<set_lidar*>(_set)->T_lidar_imu();

    use_scan = dynamic_cast<set_lidar*>(_set)->use_scan();
    use_map=dynamic_cast<set_lidar*>(_set)->use_map();


    estimate_extrinsic = dynamic_cast<set_lidar*>(_set)->estimate_extrinsic();
    if (estimate_extrinsic)
    {
        initial_lidar_extrinsic_rotation_cov = dynamic_cast<set_lidar*>(_set)->initial_lidar_extrinsic_rotation_cov();
        initial_lidar_extrinsic_translation_cov = dynamic_cast<set_lidar*>(_set)->initial_lidar_extrinsic_translation_cov();
    }
}

void hwa_lidar::lidar_proc::load_imuobs(const double &t,const std::vector<Triple> & wm,const std::vector<Triple> & vm)
{
    
    assert(wm.size() == vm.size());
    /* only one sample can be processed successfully*/
    assert(wm.size() == 1);
    for (int i = 0; i < wm.size(); i++)
    {
        imumsg tmp_imu;
        tmp_imu.t = t;
        tmp_imu.angular_velocity = wm[i]/imu_ts;
        tmp_imu.linear_acceleration = vm[i]/imu_ts;
        _vecimu.push_back(tmp_imu);
    }
    
}

void hwa_lidar::lidar_proc::load_lidarobs(const double &t, const lidarPath &lidar_path)
{
    cur_lidar_path = lidar_path;
}

LidarFrame hwa_lidar::lidar_proc::ProcessBatch(const double& t)
{
    LidarFrame res;
    return res;
}