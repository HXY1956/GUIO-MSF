#ifndef hwa_lidar_utility_h
#define hwa_lidar_utility_h

#include <string>
#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "hwa_base_eigendef.h"

using namespace hwa_base;

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

    struct LIDARState
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
        LIDARState() : id(0), time(0),
            orientation(Eigen::Quaterniond::Identity()),
            position(Triple::Zero()),
            isKeyFrame(false) {}

        /**
        * @brief Constructor
        * set initial parameter by StateID
        */
        explicit LIDARState(const LidarStateIDType& new_id) : id(new_id), time(0),
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

        pcl::PointCloud<pcl::PointXYZI> LessSurf;    ///< less surf point clouds
        pcl::PointCloud<pcl::PointXYZI> LessSharp;    ///< less sharp point clouds
        pcl::PointCloud<pcl::PointXYZI> Surf;        ///< surf point clouds
        pcl::PointCloud<pcl::PointXYZI> Sharp;        ///< sharp point clouds
        pcl::PointCloud<pcl::PointXYZI> fullCloud;    ///< full point clouds

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

            LessSurf.clear();
            LessSharp.clear();
            Surf.clear();
            Sharp.clear();
            fullCloud.clear();

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
    typedef std::map<LidarStateIDType, LIDARState, std::less<int>,
        Eigen::aligned_allocator<std::pair<const LidarStateIDType, LIDARState>>> LidarStateServer;

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
}
#endif
