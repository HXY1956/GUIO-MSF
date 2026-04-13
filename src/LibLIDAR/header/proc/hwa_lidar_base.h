#ifndef hwa_lidar_base_h
#define hwa_lidar_base_h

#include "hwa_lidar_utility.h"
#include "hwa_lidar_map.h"
#include "hwa_lidar_frame.h"
#include "hwa_lidar_proc_odometry.h"
#include "hwa_lidar_proc_mapping.h"
#include "hwa_lidar_proc.h"

using namespace hwa_base;

namespace hwa_lidar
{
    class lidar_base
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in] set setbase control
        */
        explicit lidar_base(set_base* set);
        
        /**
        * @brief destructor
        */
        ~lidar_base();
        
        SO3 skew(const Triple& v);

        void readChisquare_test();
        /**
        * @brief screen out the observations whose residual is larger than n times of the MSE
        * @para[in]    H        jacobian matrix of the lidar residual function
        * @para[in] r        residual vector of the lidar residual function
        * @para[in] ther    to identify the threshold of this function
        */
        void Huber(Matrix &H, Vector &r, float ther);
        
        /**
        * @brief calculate the measurement jacobian for the lidar observations
        * @para[in] obs            observation of the lidar measurement
        * @para[out] H            jacobian matrix of the lidar residual function
        * @para[out] r            residual vector of the lidar residual function
        * @para[in] id            ID of current lidar observation
        * @para[in] use_3d        H of corner observation will consider 3D directions.
        * @para[in] ther        a value for Huber function as its' thershold
        */
        void lidarMeasurementJacobian(lidarMappingObs obs, Matrix &H, Vector &r, int id,bool use_3d, float ther=1);

        /**
        * @brief calculate the measurement jacobian for the lidar observations
        * @para[in] obs            observation of the lidar measurement
        * @para[out] H            jacobian matrix of the lidar residual function
        * @para[out] r            residual vector of the lidar residual function
        * @para[in] id            ID of current lidar observation
        * @para[in] use_3d        whether H of corner observation will consider 3D directions.
        * @para[in] ther        a value for Huber function as its' thershold
        */
        void lidarMeasurementJacobian(lidarOdometryObs obs, Matrix &H, Vector &r,int id, bool use_3d, float ther=1);
        
        /**
        * @brief calculate the measurement jacobian for the lidar observations
        * @para[in] obs            observation of the lidar measurement
        * @para[out] H            jacobian matrix of the lidar residual function
        * @para[out] r            residual vector of the lidar residual function
        * @para[in] id            ID of current project
        * @para[in] project_id    ID of current lidar observation
        * @para[in] use_3d        whether H of corner observation will consider 3D directions.
        * @para[in] ther        a value for Huber function as its' thershold
        */
        void lidarMeasurementJacobian(lidarOdometryObs obs, Matrix &H, Vector &r, int project_id,int id, bool use_3d, float ther = 1);
        
        /**
        * @brief calculate the measurement jacobian for the lidar observations
        * @para[in] obs            observation of the lidar measurement
        * @para[out] H            jacobian matrix of the lidar residual function
        * @para[in] use_3d        whether H of corner observation will consider 3D directions.
        * @para[in] ther        a value for Huber function as its' thershold
        */
        void lidarMeasurementJacobian_priormap(lidarMappingObs obs, Matrix &H, Vector &r, bool use_3d, float ther);
    
    public:
        std::map<int, double> chi_squared_test_table;

        Eigen::Isometry3d T_lidar_imu;        ///< transformation from lidar to IMU
        Triple t_lidar_imu;        ///< translation from lidar to IMU
        SO3 R_lidar_imu;        ///< rotation from lidar to IMU

        bool use_scan;                        ///< use scan or not
        bool use_map;                        ///< use maping or not
        bool use_pp;                        ///< use planar patch or not
        bool use_segmenter;                    ///< use point cloud segmenter or not

        bool use_corrdistort;                ///< correct distortion or not
        int window_size = 0;                ///< size of lidar frames(after state augmentation,don't contain the first frame)
        
        LidarStateServer lidar_states;        ///< store states of lidar
        LIDARState lidar_extrinsic;         ///< store extrinsic for estimate
        LidarStateIDType lidar_state_id;    ///< ID of lidar states
        LidarStateIDType lidar_next_id;        ///< ID of next lidar

        //transformation parameters in lidarmapping from w frame to e frame
        //SO3 R_w_e;
        //Triple t_w_e;
        
        int max_lidarstate_size=0;            ///< maximum size of lidar states
        double scan_observation_noise=0;    ///< noise of scan observation
        double map_observation_noise=0;        ///< noise of map observation
        int num_of_lidar=0;                    ///< number of lidar

        bool first_odo=false;                ///< whether is using odometry update firstly
        bool first_map = true;                ///< whether is using map update firstly
        
    protected:
        lidar_frame *lidarproc;            ///< registering point clouds received from multi-laser lidars.
    };
    
}

#endif