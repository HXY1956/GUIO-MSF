#ifndef hwa_lidar_proc_odometry_h
#define hwa_lidar_proc_odometry_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <iostream>
#include <time.h>
#include "hwa_lidar_utility.h"
#include "hwa_set_base.h"

namespace hwa_lidar
{
    /**
     * @brief Implementation of the LOAM laser odometry component.
     */
    class lidar_proc_odometry
    {
    public:
        /** @brief default constructor. */
        lidar_proc_odometry();

        /** @brief default destructor. */
        ~lidar_proc_odometry() {};

        /**
        * @brief main function to find correspond "scan to scan" observation
        *
        * @param[in] frame1        last lidar frame
        * @param[in] frame2        current lidar frame
        */
        void process(LidarFrame& frame1, LidarFrame& frame2);

        /**
        * @brief remove the distortion of pointcloud in current lidar frame
        *
        * @param[in] frame1        last lidar frame
        * @param[in] frame2        current lidar frame
        */
        void removeDistortion(LidarFrame& frame1, LidarFrame& frame2);

        //ceres optimize,need to find how to add ceres lib to this solution
        //void ceresOptimize();


        /**@brief get association between lidar frames
        *
        * @param[in] buffer            lidar frame data buffer
        */
        void data_association(std::vector<LidarFrame> &buffer);

    protected:
        /** @brief    remove the history information, etc. */
        void reset();

        /** 
         * @brief Transform the given point to the start of the sweep.
         * 
         * @param[in] pi            the point to transform
         * @param[out] po            the point instance for storing the result
         * @param[in] distortion    use distortion or not
         */
        void transformToStart(const pcl::PointXYZI pi, pcl::PointXYZI & po,bool distortion=false);

        /**
         * @brief Transform the given point to the end of the sweep.
         *
         * @param[in] pi            the point to transform
         * @param[out] po            the point instance for storing the result
         * @param[in] distortion    use distortion or not
         */
        void transformToEnd(const pcl::PointXYZI pi, pcl::PointXYZI & po,bool distortion=false);

    public:
        ///< transformation from last frame to current frame
        SO3 last_curr_rot;        ///< rotation from last frame to current frame
        Triple last_curr_trans;    ///< translation from last frame to current frame
        ///< transfprmation from current frame to last frame
        SO3 curr_last_rot;        ///< rotation from current frame to last frame
        Triple curr_last_trans;    ///< translation from current frame to last frame

        float scanPeriod_=0.1;       ///< time per scan
        long frameCount_=0;        ///< number of processed frames
        bool systemInited_;      ///< initialization flag

        pcl::KdTreeFLANN<pcl::PointXYZI> lastCornerKDTree_;   ///< last corner cloud KD-tree
        pcl::KdTreeFLANN<pcl::PointXYZI> lastSurfaceKDTree_;  ///< last surface cloud KD-tree

        CorrespondCornerFeature correspondCornerFeature_;    ///< TODO
        CorrespondSurfFeature   correspondSurfFeature_;        ///< TODO
        lidarOdometryObs lidarOdoObs;    ///< observation of lidar odometrys

        /**@brief get association between lidar frames used in lidar_PP
        *
        * @param[in] buffer            lidar frame data buffer
        */
        //void data_association(vector<LidarFrame> &buffer);    
    
        std::map<int, std::vector<int>> associations;        ///< association between lidar frames(used in lidar_PP and segmenter)


        void segmenter_data_association(std::vector<LidarFrame> &buffer);

    };
}///< end of namespace hwa_lidar
#endif