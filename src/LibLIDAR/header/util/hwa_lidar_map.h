#ifndef hwa_lidar_map_h
#define hwa_lidar_map_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include "hwa_lidar_utility.h"
#include "hwa_set_base.h"
#include "hwa_base_mutex.h"

using namespace hwa_base;

namespace hwa_lidar
{
    /**
     *@brief Clas for build lidar pointcloud map,
     */
    class lidar_map
    {
    public:
        /** @brief default constructor. */
        lidar_map(double resolution = 32.0);

        /** @brief default destructor. */
        ~lidar_map() {};

        /**@brief clear the history information        */
        void clearMeasurement();

        /**
        * @brief read prior lidar map
        *
        * @param[in] corner_path    the file path of the stored lidarmap file of corner
        * @param[in] surf_path        the file path of the stored lidarmap file of surface
        * @param[in] xyz            TODO
        */
        void getPriorMap(std::string &corner_path, std::string &surf_path,bool xyz);

        /**
        * @brief find correspond surrounding pointcloud for current lidar frame
        *
        * @param[in] center            center point
        * @param[in] boxsize        size of the surrounding box
        * @return
            @retval    ture            update successfully
            @retval    false            not enough points in the submap
        */
        bool extractSurroundingPriorMap(Triple &center, float boxsize);

        /**
        * @brief match current scan to the prior map
        *
        * @param[in] frame            current lidar frame
        * @return
            @retval    ture            update successfully
            @retval    false            align failed
        */
        bool alignToPriorMap(LidarFrame &frame);

        /**
        * @brief Set/change the octree voxel resolution
        *
        * @param[in] resolution        length of voxels at lowest tree level
        */
        void setResolution(double resolution);

    public:
        bool loadmap;                        ///< map loaded or not
        Triple last_mapcenter;        ///< last center of map
        pcl::PointXYZI pointOri, pointSel;    ///< TODO

        ///< prior map
        pcl::PointCloud<pcl::PointXYZI>::Ptr priorCornerMap;    ///< TODO
        pcl::PointCloud<pcl::PointXYZI>::Ptr priorSurfMap;        ///< TODO

        ///< prior submap
        pcl::PointCloud<pcl::PointXYZI>::Ptr CornerSubmap;        ///< TODO
        pcl::PointCloud<pcl::PointXYZI>::Ptr SurfSubmap;        ///< TODO

        ///< kd-tree
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromPriorMap;        ///< TODO
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromPriorMap;        ///< TODO

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> priorCornerMapOctree;    ///< TODO
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> priorSurfMapOctree;        ///< TODO

        ///< use for downsize the pointcloud
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;        ///< TODO
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;            ///< TODO

        ///< temp paramter
        std::vector<int> pointSearchInd;                ///< index of point searching        
        std::vector<float> pointSearchSqDis;            ///< distance of point searching

        ///< store the submap points used in last frame(used for visibility)
        std::vector<Triple> submap_corner;            ///< TODO
        std::vector<Triple> submap_surf;            ///< TODO

        ///< store the observation of lidar mapping
        CorrespondCornerFeature correspondCornerFeature_;    ///< TODO
        CorrespondSurfFeature   correspondSurfFeature_;        ///< TODO
        lidarMappingObs lidarMapObs;                        ///< observation of lidar mapping

    protected:
        /**@brief Format conversion    */
        pcl::PointCloud<pcl::PointXYZI> pclXYZ2XYZI(pcl::PointCloud<pcl::PointXYZ> &cloud);

    };

}// end of namespace hwa_lidar

#endif

