#ifndef hwa_lidar_proc_segmenter_h
#define hwa_lidar_proc_segmenter_h

#include "hwa_lidar_utility.h"
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace hwa_base;

//struct cloud_info
//{
//    int startRingIndex[N_SCAN*Horizon_SCAN];
//    int endRingIndex[N_SCAN*Horizon_SCAN];
//
//    float startOrientation;
//    float endOrientation;
//    float orientationDiff;
//
//    bool segmentedCloudGroundFlag[N_SCAN*Horizon_SCAN]; // true - ground point, false - other points
//    int segmentedCloudColInd[N_SCAN*Horizon_SCAN];// point column index in range image
//    float segmentedCloudRange[N_SCAN*Horizon_SCAN];// point range
//};

namespace hwa_lidar {
    class lidar_proc_segmenter
    {
    public:
        lidar_proc_segmenter(int n_scan = 16);
        ~lidar_proc_segmenter();
    
        // 0. set initial parameters;
        void systemInitialization(int n_scan);

        void process(pcl::PointCloud<pcl::PointXYZI>::Ptr input);

        std::vector<Triple> getCentroids() { return centroids; }

        std::vector<Triple> getMaindirections() { return directions; }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getClusterCloud() { return outcloud; }
    private:
        // 1. Convert ros message to pcl point cloud
        void copyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
        // 2. Start and end angle of a scan
        void findStartEndAngle();
        // 3. Range image projection
        void projectPointCloud();
        // 4. Mark ground points
        void groundRemoval();
        // 5. Point cloud segmentation
        void cloudSegmentation();
        // 6. Publish all clouds and oulier remove
        void publishCloud();
        // 7. Reset parameters for next iteration
        void resetParameters();
        void makeColoredPoint(pcl::PointXYZRGB & PtColored, pcl::PointXYZ const & Pt, uint32_t rgb);

        void labelComponents(int row, int col);

    private:
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
        pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
        pcl::PointCloud<pcl::PointXYZI>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

        pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
        //pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloud;
        //pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloudPure;
        //pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud;

        pcl::PointXYZI nanPoint;

        cv::Mat rangeMat; // range matrix for range image
        cv::Mat labelMat; // label matrix for segmentaiton marking
        cv::Mat groundMat; // ground matrix for ground cloud marking
        int labelCount;

        float startOrientation;
        float endOrientation;

        //cloud_info segMsg;

        std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process
        
        uint16_t *allPushedIndX; // array for tracking points of a segmented object
        uint16_t *allPushedIndY;
        
        uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
        uint16_t *queueIndY;
        
        int N_SCAN = 16;
        int Horizon_SCAN = 1800;
        float ang_res_x = 0.2;
        float ang_res_y = 2.0;
        float ang_bottom = 15.0 + 0.1;
        int groundScanInd = 7;
        //const float ang_res_x = 0.2;
        //const float ang_res_y = 0.427;
        //const float ang_bottom = 24.9;
        //const int groundScanInd = 50;

        const bool loopClosureEnableFlag = false;
        const double mappingProcessInterval = 0.3;

        const float scanPeriod = 0.1;
        const int systemDelay = 0;
        const int imuQueLength = 200;

        const float sensorMinimumRange = 1.0;
        const float sensorMountAngle = 0.0;
        const float segmentTheta = 60.0 / 180.0*M_PI; // decrese this value may improve accuracy
        const int segmentValidPointNum = 5;
        const int segmentValidLineNum = 3;
        float segmentAlphaX = ang_res_x / 180.0 * M_PI;
        float segmentAlphaY = ang_res_y / 180.0 * M_PI;


        const int edgeFeatureNum = 2;
        const int surfFeatureNum = 4;
        const int sectionsTotal = 6;
        const float edgeThreshold = 0.1;
        const float surfThreshold = 0.1;
        const float nearestFeatureSearchSqDist = 25;

        bool useCloudring = false;


        //output parameters
        std::map<int, std::vector<pcl::PointXYZI>> indexs;
        std::vector<Triple> centroids;
        std::vector<Triple> directions;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud;

    };
}
#endif