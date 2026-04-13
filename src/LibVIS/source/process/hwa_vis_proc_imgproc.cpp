#include "hwa_vis_proc_imgproc.h"

void hwa_vis::vis_imgproc_base::load_imuobs(const double &t,const std::vector<Triple> & wm,const std::vector<Triple> & vm, double imu_ts)
{
    assert(wm.size() == vm.size());
    /* only one sample can be processed successfully */
    assert(wm.size() == 1);
    for (int i = 0; i < wm.size(); i++)
    {
        IMU_MSG tmp_imu;
        tmp_imu.t = t;
        tmp_imu.angular_velocity = wm[i]/imu_ts;
        tmp_imu.linear_acceleration = vm[i]/imu_ts;
        _vecimu.push_back(tmp_imu);
    }
}

void hwa_vis::vis_imgproc_base::load_imgobs(const double &t, const IMG_PATH &img_path)
{
    cur_img_path = img_path;
}

hwa_vis::PointCloud hwa_vis::vis_imgproc_base::ProcessBatch()
{
    PointCloud res;
    return res;
}

void hwa_vis::vis_imgproc_base::ProcessBatchT(const double& t, PointCloud & pointcloud)
{
    pointcloud = ProcessBatch();
}

void hwa_vis::vis_imgproc_base::add_camera_pose(SO3 R, Triple t)
{
    _pose_history.push_back(std::make_pair(R, t));
    while (_pose_history.size() > _pose_history_max_size)
        _pose_history.pop_front();
}
