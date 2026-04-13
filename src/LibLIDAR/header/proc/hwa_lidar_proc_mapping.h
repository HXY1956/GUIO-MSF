#ifndef hwa_lidar_proc_mapping_h
#define hwa_lidar_proc_mapping_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include "hwa_lidar_utility.h"
#include "hwa_set_base.h"

namespace hwa_lidar
{
    /**
     *@brief Clas for build lidar pointcloud map, 
     */
    class lidar_proc_mapping
    {
    public:
        /** @brief default constructor. */
        lidar_proc_mapping(double resolution=32.0);

        /** @brief default destructor. */
        ~lidar_proc_mapping() {};

        /**
        * @brief add pointcloud in lidar frame to the lidarmap 
        *
        * @param[in] frame    lidar frame(with the pointcloud after distortion correction)
        */
        void addPointcloudToMap(const LidarFrame &frame);

        /**
        * @brief lidar processing main function
        *
        * @param[in] frame    lidar frame(with the pointcloud after distortion correction)
        */
        void process(const LidarFrame &frame);

        //use ceres to optimize the problem
        //void ceresOptimize();
        
        /**
        * @brief clear the history information
        */
        void reset();

    protected:
        /**
        * @brief transform point from w frame to e frame
        */
        void transformAssociateToMap();

        /**
        * @brief transform point from lidar frame to w frame
        *
        * @param[in] pi        coordinate of point in lidar frame
        * @param[out] po    coordinate of point in w frame
        */
        void pointAssociateToMap(pcl::PointXYZI & pi, pcl::PointXYZI & po);

        /**
        * @brief downsize the lidar map
        *
        * @param[in] cloud            data buffer of lidar point cloud 
        * @param[in] leaf_size        parameter of down sampling
        */
        void downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double& leaf_size);

        /**
        * @brief downsize the lidar map
        *
        * @param[in] cloud            data buffer of lidar point cloud
        * @param[in] filtered        data buffer of lidar point cloud after downsize
        * @param[in] leaf_size        parameter of down sampling
        */
        void downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered, const double& leaf_size);

    public:
        bool systemInited_ = false;            ///< initialization flag
        ///< store the pose of first lidar frame(used as the transform between w frame and e frame)
        SO3 first_R_l_e;        ///< attitude of first lidar frame in the global frame
        Triple first_t_l_e;        ///< position of first lidar frame in the global frame
        ///< pose of current lidar frame
        SO3 curr_R_l_e;            ///< attitude of current lidar frame in the global frame
        Triple curr_t_l_e;            ///< position of current lidar frame in the global frame

        SO3 curr_R_l_w;            ///< attitude of current lidar frame in the world frame
        Triple curr_t_l_w;            ///< position of current lidar frame in the world frame

        ///< store the submap points used in last frame(used for visibility)
        std::vector<Triple> submap_corner;    ///< submap points of corner
        std::vector<Triple> submap_surf;    ///< submap points of surface

        ///< store the observation of lidar mapping
        CorrespondCornerFeature correspondCornerFeature_;    ///< TODO
        CorrespondSurfFeature correspondSurfFeature_;        ///< TODO
        lidarMappingObs lidarMapObs;            ///< lidar mapping observation            

    ///< parameters:used for construct the lidar pointcloud map
    public:
        int laserCloudCenWidth=10;                ///< TODO
        int laserCloudCenHeight=10;                ///< TODO
        int laserCloudCenDepth=5;                ///< TODO
        // Length width and height of cube
        int laserCloudWidth=21;                    ///< TODO
        int laserCloudHeight=21;                ///< TODO
        int laserCloudDepth=11;                    ///< TODO

        // cube all number
        int laserCloudNum=4851;        ///< laserCloudWidth * laserCloudHeight * laserCloudDepth

        // effective cube number 125
        int laserCloudValidNum=0;                ///< TODO
        int laserCloudSurroundNum=0;            ///< TODO

        // Record the index of the valid cube in the submap
        // 5*5*5
        int laserCloudValidInd[125];            ///< TODO
        int laserCloudSurroundInd[125];            ///< TODO

        ///< surround points in submap to build kdtree
        // A submap composed of valid 125 cubes is also used to visualize temporary variables
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap;    ///< TODO
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap;        ///< TODO

        ///< points in every cube
        //Used to store all point cloud features
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray[4851];    ///< TODO
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray[4851];        ///< TODO

        ///< kd-tree
        //Used when matching observations
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap;            ///< TODO
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap;            ///< TODO

        //input point cloud in cur frame after removeNAN and downSample 
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack;            ///< TODO
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack;            ///< TODO
    };
}//end of namespace hwa_lidar
#endif