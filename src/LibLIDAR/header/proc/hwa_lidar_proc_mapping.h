#ifndef hwa_lidar_proc_mapping_h
#define hwa_lidar_proc_mapping_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
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
        CloudPtr laserCloudCornerFromMap;    ///< TODO
        CloudPtr laserCloudSurfFromMap;        ///< TODO

        ///< points in every cube
        //Used to store all point cloud features
        CloudPtr laserCloudCornerArray[4851];    ///< TODO
        CloudPtr laserCloudSurfArray[4851];        ///< TODO

        ///< kd-tree
        //Used when matching observations
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap;            ///< TODO
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap;            ///< TODO

        //input point cloud in cur frame after removeNAN and downSample 
        CloudPtr laserCloudCornerStack;            ///< TODO
        CloudPtr laserCloudSurfStack;            ///< TODO
    };

    class lidar_proc_global_mapping {
    public:
        void run();
        void stop();
        bool isRun() {
            return isRunning;
        }
        void push(double _time, SO3 _R, Triple _t, CloudPtr _fullCloud) {
            {
                std::lock_guard<std::mutex> lock(keyframe_mutex);
                keyframe_queue.emplace_back(KeyFrame(_time, _R, _t, _fullCloud));
            }
            keyframe_cv.notify_one();
        }
        void process();
        void processKeyFrame(const KeyFrame& kf);
        void savePCDFileBinary(
            const std::string& save_path,
            const double& leaf_size);
        void setReferencePose(const SO3& R_l_w, const Triple& t_l_w) {
            first_R_l_e = R_l_w;
            first_t_l_e = t_l_w;
        }
        void transformAssociateToMap(const SO3& R_l_w, const Triple& t_l_w);
        CloudRGBPtr point_management(CloudPtr cloudin);

    private:
        bool isRunning = false;
        CloudRGBPtr global_map;
        std::deque<KeyFrame> keyframe_queue;
        std::mutex keyframe_mutex;
        std::mutex global_map_mutex;
        std::condition_variable keyframe_cv;
        std::thread mapping_thread;
        SO3 first_R_l_e;
        Triple first_t_l_e;
        SO3 curr_R_l_w;
        Triple curr_t_l_w;
        int laserCloudWidth = 2;
        int laserCloudHeight = 2;
        int laserCloudDepth = 1;
    };

    inline void jetColor(float t, uint8_t& r, uint8_t& g, uint8_t& b)
    {
        t = std::max(0.0f, std::min(1.0f, t));

        float rf = std::min(std::max(1.5f - std::abs(4.0f * t - 3.0f), 0.0f), 1.0f);
        float gf = std::min(std::max(1.5f - std::abs(4.0f * t - 2.0f), 0.0f), 1.0f);
        float bf = std::min(std::max(1.5f - std::abs(4.0f * t - 1.0f), 0.0f), 1.0f);

        r = static_cast<uint8_t>(rf * 255);
        g = static_cast<uint8_t>(gf * 255);
        b = static_cast<uint8_t>(bf * 255);
    }

    inline void binaryColor(float t, uint8_t& r, uint8_t& g, uint8_t& b)
    {
        if (t < 0.2f)
        {
            r = 0;
            g = 0;
            b = 255;
        }
        else
        {
            r = 255;
            g = 0;
            b = 0;
        }
    }
}//end of namespace hwa_lidar
#endif