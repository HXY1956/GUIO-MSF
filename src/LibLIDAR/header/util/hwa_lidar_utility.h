#ifndef hwa_lidar_utility_h
#define hwa_lidar_utility_h

#include <string>
#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "hwa_base_eigendef.h"

using namespace hwa_base;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using CloudPtr = CloudType::Ptr;
using CloudRGBType = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudRGBPtr = CloudRGBType::Ptr;

namespace hwa_lidar {

    struct KeyFrame
    {
        static int next_id;
        int id;
        double time;
        SO3 R;
        Triple t;
        CloudPtr cloud;
        KeyFrame() {}
        KeyFrame(double _time, SO3 _R, Triple _t, CloudPtr _cloud) : time(_time), R(_R), t(_t), cloud(_cloud) {
            id = next_id++;
        }
    };

};

namespace hwa_lidar
{
    typedef long long int LidarStateIDType;
    typedef pcl::PointXYZI PointType;

    struct lidarPath
    {
        double t;           ///< time
        std::string lidar_path;    ///< path of lidar
    };

    struct dataLIDAR
    {
        double t=0;           ///< time
        pcl::PointCloud<pcl::PointXYZI> lidar;  ///< buffer of lidar point cloud
    };

    struct LidarState
    {
        LidarStateIDType id;                ///< id of lidar state
        double time = 0;                    ///< time when the lidar is recorded
        Eigen::Quaterniond orientation;        ///< attitude of lidar state
        Triple position;            ///< position of lidar state
        bool isKeyFrame;                    ///< identification of current lidar

        /**
        * @brief Constructor
        * set initial parameter
        */
        LidarState() : id(0), time(0),
            orientation(Eigen::Quaterniond::Identity()),
            position(Triple::Zero()),
            isKeyFrame(false) {}

        /**
        * @brief Constructor
        * set initial parameter by StateID
        */
        explicit LidarState(const LidarStateIDType& new_id) : id(new_id), time(0),
            orientation(Eigen::Quaterniond::Identity()),
            position(Triple::Zero()),
            isKeyFrame(false) {}

    };

    /**
    * @struct Correspond corner points between different lidar frame
    * @brief  observation used in back end updating
    */
    struct CorrespondCornerFeature
    {
        std::vector<Triple> currCornerPointCloud_inlast;    ///< current Corner Point Cloud in last
        std::vector<Triple> currCornerPointCloud;            ///< current Corner Point Cloud 
        std::vector<Triple> correspondCornerPointCloudA;    ///< correspond Corner Point Cloud A
        std::vector<Triple> correspondCornerPointCloudB;    ///< correspond Corner Point Cloud B
    };

    /**
    * @struct Correspond surf points between different lidar frame
    * @brief  observation used in back end updating
    */
    struct CorrespondSurfFeature
    {
        std::vector<Triple> currSurfPointCloud_inlast;        ///< current surf Point Cloud in last
        std::vector<Triple> currSurfPointCloud;            ///< current surf Point Cloud
        std::vector<Triple> correspondSurfPointCloudA;        ///< correspond surf Point Cloud A
        std::vector<Triple> correspondSurfPointCloudB;        ///< correspond surf Point Cloud B
        std::vector<Triple> correspondSurfPointCloudC;        ///< correspond surf Point Cloud C
        std::vector<Triple> norm;                            ///< normal vector
        std::vector <double> negative_OA_dot_norm;                    ///< TODO
    };

    /**
    * @struct observation of lidar odometry
    * @brief  correspond points and pose between two lidar frames
    */
    struct lidarOdometryObs
    {
        ///< pose of last lidarframe
        SO3 last_R_l_e;            ///< last rotation from lidar to world frame
        Triple last_t_l_e;            ///< last translation from lidar to world frame
        ///< pose of current lidarframe
        SO3 curr_R_l_e;            ///< current rotation from lidar to world frame
        Triple curr_t_l_e;            ///< current translation from lidar to world frame

        CorrespondCornerFeature cornerFeature;        ///< corner feature
        CorrespondSurfFeature surfFeature;            ///< surf feature

        lidarOdometryObs()
        {
            last_R_l_e = SO3::Identity();        ///< last rotation from lidar to world frame
            last_t_l_e = Triple::Zero();            ///< last translation from lidar to world frame
            curr_R_l_e = SO3::Identity();        ///< current rotation from lidar to world frame
            curr_t_l_e = Triple::Zero();            ///< current translation from lidar to world frame
        }
    };

    /**
    * @struct observation of lidar mapping
    * @brief  corespond points and pose between lidar frame and lidar map
    */
    struct lidarMappingObs
    {
        ///< pose of current lidarframe
        SO3 curr_R_l_e;                ///< last rotation from lidar to world frame
        Triple curr_t_l_e;                ///< last translation from lidar to world frame
        SO3 last_R_l_e;                ///< current rotation from lidar to world frame
        Triple last_t_l_e;                ///< current translation from lidar to world frame

        CorrespondCornerFeature cornerFeature;        ///< corner feature
        CorrespondSurfFeature surfFeature;            ///< surf feature

        lidarMappingObs()
        {
            curr_R_l_e = SO3::Identity();    ///< last rotation from lidar to world frame
            curr_t_l_e = Triple::Zero();        ///< last translation from lidar to world frame
            last_R_l_e = SO3::Identity();    ///< current rotation from lidar to world frame
            last_t_l_e = Triple::Zero();        ///< current translation from lidar to world frame
        }
    };

    /**
    * @struct to store information in one lidarframe
    * @brief  contain the pointclouds after classification and current pose
    */
    struct LidarFrame
    {
        LidarStateIDType id;                        ///< id of lidar state

        bool empty;                            ///< whether is empty

        CloudPtr LessSurf;    ///< less surf point clouds
        CloudPtr LessSharp;    ///< less sharp point clouds
        CloudPtr Surf;        ///< surf point clouds
        CloudPtr Sharp;        ///< sharp point clouds
        CloudPtr fullCloud;    ///< full point clouds

        std::vector<Triple> pcs;        ///< center points
        std::vector<Triple> ncs;        ///< normal vector
        std::vector<std::vector<Triple>> _near_points;
        std::vector<Triple> centroids;
        std::vector<Triple> directions;
        ///< store the lidar frame pose
        SO3 R_l_e;                ///< rotation from lidar to world frame
        Triple t_l_e;                ///< translation from lidar to world frame

        LidarFrame()
        {
            id = 0;                ///< lidar frame ID
            empty = true;        ///< whether is empty

            LessSurf = std::make_shared<CloudType>();
            LessSharp = std::make_shared<CloudType>();
            Surf = std::make_shared<CloudType>();
            Sharp = std::make_shared<CloudType>();
            fullCloud = std::make_shared<CloudType>();

            pcs.clear();
            ncs.clear();
            _near_points.clear();

            centroids.clear();
            directions.clear();

            R_l_e = SO3::Identity();    ///< rotation from lidar to world frame
            t_l_e = Triple::Zero();        ///< translation from lidar to world frame
        }
    };
    ///< tools:change the type to store the pointcloud position
    /** @brief PointXYZI to Triple    */
    std::vector<Triple> PointXYZI2Triple(pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi, SO3 R_l_e = SO3::Identity(), Triple t_l_e = Triple::Zero());

    /** @brief PointXYZI to Triple    */
    std::vector<Triple> PointXYZI2Triple(pcl::PointCloud<pcl::PointXYZI>& xyzi, SO3 R_l_e = SO3::Identity(), Triple t_l_e = Triple::Zero());

    /** @brief Triple to PointXYZI    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr Triple2PointXYZI(std::vector<Triple>& xyz, SO3 R_l_e = SO3::Identity(), Triple t_l_e = Triple::Zero());

    //pcl::PointCloud<pcl::PointXYZI> Triple2PointXYZI(vector<Triple>& xyz);
    ///< tools:transform the PointXYZI
    /** @brief position translation        */
    void transPointXYZI(pcl::PointXYZI& p, SO3 R = SO3::Identity(), Triple t = Triple::Zero());

    ///< tools:transform the PointXYZI
    /** @brief position translation        */
    void transPointXYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi, SO3 R = SO3::Identity(), Triple t = Triple::Zero());
    /**
    * @typedef LidarStateServer
    * @brief store historical lidar information
    */
    typedef std::map<LidarStateIDType, LidarState, std::less<int>,
        Eigen::aligned_allocator<std::pair<const LidarStateIDType, LidarState>>> LidarStateServer;

    ///**
    //* @brief transform a vector to skew-symmetric matrix
    //* @param[in] v
    //* @return SO3
    //*/
    // SO3 skew(const Triple& v);

     /**
     * @brief get the rotation from navigation frame to ECEF frame 
     * @param[in] BLH                    BLH coordinate
     * @return SO3 XYZ        ECEF coordinates
     */
     SO3 R_ENU_ECEF(const Triple &BLH);

     /**
     * @brief transform 
     * @param[in] X                        ECEF coordinates
     * @return Triple BLH        BLH coordinate
     */
     Triple XYZ2BLH(const Triple &X);

     /**
     * @brief calculate the skew Symmetric matrix
     * @param[in] v                    vector
     * @return SO3        the skew Symmetric matrix
     */
     SO3 skewSymmetric(const Triple& w);

     /** brief Calculate the squared difference of the given two points.
     *
     * @param[in] a            The first point.
     * @param[in] b            The second point.
     * @return float        The squared difference between point a and b.
     */
     template <typename PointT>
     inline float calcSquaredDiff(const PointT& a, const PointT& b)
     {
         float diffX = a.x - b.x;
         float diffY = a.y - b.y;
         float diffZ = a.z - b.z;
         return diffX * diffX + diffY * diffY + diffZ * diffZ;
     }

     /**
     * @brief split the string by "seperator"
     * @param[in] s                        string need to be splited
     * @param[in] seperator                symbol used for split the string.
     * @return vector<std::string>        vector of string after spliting
     */
     std::vector<std::string> split(const std::string &s, const std::string &seperator);

     template<typename PointT = PointType>
     inline void downSampleChunked(
         typename pcl::PointCloud<PointT>::Ptr cloud,
         double leaf_size = 0.2,
         double chunk_size = 50)
     {
         if (cloud->empty())
             return;

         std::vector<int> index;
         pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

         if (cloud->empty())
             return;

         PointT min_pt, max_pt;
         pcl::getMinMax3D(*cloud, min_pt, max_pt);

         struct GridKey
         {
             int x;
             int y;
             int z;

             bool operator==(const GridKey& other) const
             {
                 return x == other.x &&
                     y == other.y &&
                     z == other.z;
             }
         };

         struct Hash
         {
             size_t operator()(const GridKey& k) const
             {
                 return ((size_t)k.x * 73856093) ^
                     ((size_t)k.y * 19349663) ^
                     ((size_t)k.z * 83492791);
             }
         };

         std::unordered_map<
             GridKey,
             typename pcl::PointCloud<PointT>::Ptr,
             Hash> chunks;

         for (const auto& pt : cloud->points)
         {
             GridKey key;

             key.x = static_cast<int>(
                 std::floor((pt.x - min_pt.x) / chunk_size));

             key.y = static_cast<int>(
                 std::floor((pt.y - min_pt.y) / chunk_size));

             key.z = static_cast<int>(
                 std::floor((pt.z - min_pt.z) / chunk_size));

             auto& chunk = chunks[key];

             if (!chunk)
                 chunk.reset(new pcl::PointCloud<PointT>);

             chunk->push_back(pt);
         }

         typename pcl::PointCloud<PointT>::Ptr result(
             new pcl::PointCloud<PointT>);

         pcl::VoxelGrid<PointT> vg;

         for (auto& kv : chunks)
         {
             auto& chunk = kv.second;

             if (chunk->empty())
                 continue;

             typename pcl::PointCloud<PointT>::Ptr filtered(
                 new pcl::PointCloud<PointT>);

             vg.setInputCloud(chunk);
             vg.setLeafSize(
                 leaf_size,
                 leaf_size,
                 leaf_size);

             vg.filter(*filtered);

             *result += *filtered;
         }

         result->width = result->size();
         result->height = 1;
         result->is_dense = false;

         *cloud = *result;
     }

     template<typename PointT = PointType>
     inline void downSample(typename pcl::PointCloud<PointT>::Ptr cloud, const double& leaf_size)
     {
         if (cloud->points.size() == 0)
             return;
         typename pcl::PointCloud<PointT>::Ptr filtered(new typename pcl::PointCloud<PointT>);
         std::vector<int> index;
         pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
         if (cloud->empty())
             return;
         PointT first_pt = cloud->points[0];

         for (int i = 0; i < cloud->points.size(); i++)
         {
             cloud->points[i].x = cloud->points[i].x - first_pt.x;
             cloud->points[i].y = cloud->points[i].y - first_pt.y;
             cloud->points[i].z = cloud->points[i].z - first_pt.z;
         }

         PointT min_pt, max_pt;
         pcl::getMinMax3D(*cloud, min_pt, max_pt);

         std::cout
             << "range = "
             << max_pt.x - min_pt.x << ", "
             << max_pt.y - min_pt.y << ", "
             << max_pt.z - min_pt.z
             << std::endl;

         std::cout
             << "leaf_size = "
             << leaf_size
             << std::endl;

         cloud->is_dense = false;
         pcl::VoxelGrid<PointT> downer;
         downer.setInputCloud(cloud);
         downer.setLeafSize(leaf_size, leaf_size, leaf_size);
         downer.filter(*filtered);
         cloud->clear();

         for (int i = 0; i < filtered->points.size(); i++)
         {
             filtered->points[i].x = filtered->points[i].x + first_pt.x;
             filtered->points[i].y = filtered->points[i].y + first_pt.y;
             filtered->points[i].z = filtered->points[i].z + first_pt.z;
         }

         *cloud = *filtered;
     }

     template<typename PointT = PointType>
     inline void downSample(typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr filtered, const double& leaf_size)
     {
         if (cloud->points.size() == 0)
             return;
         typename pcl::PointCloud<PointT>::Ptr pcloud_ptr(new typename pcl::PointCloud<PointT>);
         std::vector<int> index;
         pcl::removeNaNFromPointCloud(*cloud, *pcloud_ptr, index);
         if (pcloud_ptr->empty())
             return;
         PointT first_pt = pcloud_ptr->points[0];

         for (int i = 0; i < pcloud_ptr->points.size(); i++)
         {
             pcloud_ptr->points[i].x = pcloud_ptr->points[i].x - first_pt.x;
             pcloud_ptr->points[i].y = pcloud_ptr->points[i].y - first_pt.y;
             pcloud_ptr->points[i].z = pcloud_ptr->points[i].z - first_pt.z;
         }

         pcloud_ptr->is_dense = false;

         pcl::VoxelGrid<PointT> downer;
         downer.setInputCloud(pcloud_ptr);
         downer.setLeafSize(leaf_size, leaf_size, leaf_size);
         downer.filter(*filtered);
         cloud->clear();

         for (int i = 0; i < filtered->points.size(); i++)
         {
             filtered->points[i].x = filtered->points[i].x + first_pt.x;
             filtered->points[i].y = filtered->points[i].y + first_pt.y;
             filtered->points[i].z = filtered->points[i].z + first_pt.z;
         }
     }
}
#endif
