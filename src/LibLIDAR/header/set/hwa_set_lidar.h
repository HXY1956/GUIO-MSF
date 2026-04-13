#ifndef hwa_set_lidar_h
#define hwa_set_lidar_h

#define XMLKEY_LIDAR "lidar"
#include "hwa_lidar_utility.h"
#include "hwa_set_base.h"

using namespace hwa_base;

namespace hwa_set
{
    class set_lidar : public virtual set_base
    {
    public:
        /** @brief constructor. */
        set_lidar();

        /** @brief default constructor. */
        ~set_lidar();

        /** @brief empty function    */
        void check();

        /** @brief empty function    */
        void help();

        double start();
        double end();

        /**
        * @brief get lidardata interval
        * @return double    time interval of lidar data
        */
        double ts();

        /**
        * @brief get imudata interval
        * @return double    time interval of IMU data
        */
        double imu_ts();

        /**
        * @brief get lidardata frequency
        * @return int        frequency of lidar data
        */
        int freq();

        /**
        * @brief get imudata frequency
        * @return int        frequency of IMU data
        */
        int imu_freq();

        /**
        * @brief get rotation from lidar to imu
        * @return SO3    rotation from lidar to imu
        */
        SO3 R_lidar_imu();

        /**
        * @brief get translation from lidar to imu
        * @return Triple    translation from lidar to imu
        */
        Triple t_lidar_imu();

        /**
        * @brief get transformation from lidar to imu
        * @return Eigen::Isometry3d    transformation from lidar to imu
        */
        Eigen::Isometry3d T_lidar_imu();
        
        /*Lidar Parameters*/
        /**
        * @brief get number of scans
        * @return int    number of scans
        */
        int n_scans();    

        /**
        * @brief get window size of lidar frame
        * @return int    window size of lidar frame
        */
        int window_size();


        /**
        * @brief get sharp resolution of map
        * @return double    sharp resolution of map
        */
        double map_resolution_sharp();

        /**
        * @brief get surf resolution of map
        * @return double    surf resolution of map
        */
        double map_resolution_surf();

        /**
        * @brief get prior map resolution
        * @return double    prior map resolution
        */
        double prior_map_resolution();


        /**
        * @brief get status of scan use
        * @return bool        status of scan use
        */
        bool use_scan();

        /**
        * @brief get status of map use
        * @return bool        status of map use
        */
        bool use_map();

        /**
        * @brief get status of plane patch use
        * @return bool        status of plane patch use
        */
        bool use_pp();

        /**
        * @brief get status of segmenter use
        * @return bool        status of point cloud segmenter use
        */
        bool use_segmenter();
        /**
        * @brief get distortion
        * @return bool        distortion        
        */
        bool distortion();

        /**
        * @brief get close threshold
        * @return double         close threshold
        */
        double close_threshold();

        /**
        * @brief get noise of lidar scan observation
        * @return double         noise of lidar scan observation
        */
        double scan_observation_noise();

        /**
        * @brief get noise of lidar map observation
        * @return double         noise of lidar map observation
        */
        double map_observation_noise();
    
        /**
        * @brief whether estimate extrinsic
        * @return bool        status of estimating extrinsic
        */
        bool estimate_extrinsic();

        /**
        * @brief get initial lidar extrinsic rotation covariance
        * @return Triple        initial lidar extrinsic rotation covariance
        */
        Triple initial_lidar_extrinsic_rotation_cov();

        /**
        * @brief get initial lidar extrinsic rotation covariacne
        * @return Triple        initial lidar extrinsic rotation covariacne
        */
        Triple initial_lidar_extrinsic_translation_cov();

    protected:
    };
}
#endif
