#ifndef hwa_lidar_proc_h
#define hwa_lidar_proc_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "hwa_lidar_utility.h"
#include "hwa_base_string.h"
#include "hwa_set_base.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_lidar {
    /**
    * @class t_lidarproc
    * @brief t_lidarproc Class for reading parameters and virtual functions
    */
    class lidar_proc
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  _set     setbase control
        */
        explicit lidar_proc(set_base* _set);

        /** @brief destructor. */
        ~lidar_proc() {}

        /**
        * @brief save imu observation in _vecimu
        * all imu observations' timestamps are between cur frame and pre frame
        *
        * @param[in] t         imu observation time
        * @param[in] gyro     corresponding angular velocity observation
        * @param[in] acc     corresponding acceleration observation
        */
        void load_imuobs(const double &t, const std::vector<Triple> &gyro, const std::vector<Triple> &acc);

        /**
        * @brief get cur lidar observation
        * @param[in] t             lidar observation time
        * @param[in] lidar_path     corresponding lidar path
        */
        void load_lidarobs(const double &t,const lidarPath& lidar_path);

        /**
        * @brief lidar processing main function
        *
        * @param[in] t         cur lidar time
        */
        virtual LidarFrame ProcessBatch(const double& t);

        /**
        * @struct imumsg
        * @brief  imudata vector only used in glidarproc class
        */
        struct imumsg
        {
            double t=0;                                ///< time 
            Triple linear_acceleration;    ///< linear acceleration 
            Triple angular_velocity;        ///< angular velocity 
        };

    public:
        lidarPath cur_lidar_path;        ///< current path of lidar

        std::vector<imumsg> _vecimu;            ///< vector store imu data between pre frame and cur frame
        dataLIDAR _pointCloud;            ///< vector store pointcloude

        double ts = 0;                    ///< camera sample interval.
        int freq = 0;                    ///< camera sample freq. 
        double imu_ts = 0;                ///< imu sample interval. 
        int imu_freq = 0;                ///< imu sample freq 

        bool use_scan, use_map;            ///< use scan/PP or not
            

        /* Basic parameters */
        SO3 R_lidar_imu;    ///< rotation from lidar to IMU
        Triple t_lidar_imu;    ///< translation from lidar to IMU
        Eigen::Isometry3d T_lidar_imu;    ///< transformation from lidar to IMU
        /* Basic parameters */
        
        bool estimate_extrinsic;        ///< estimate extrinsic or not
        Triple initial_lidar_extrinsic_rotation_cov;        ///< initial rotation convariance of lidar extrinsic
        Triple initial_lidar_extrinsic_translation_cov;    ///< initial translation convariance of lidar extrinsic

    };
}

#endif