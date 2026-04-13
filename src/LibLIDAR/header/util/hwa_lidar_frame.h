#ifndef hwa_lidar_frame_h
#define hwa_lidar_frame_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "hwa_set_lidar.h"
#include "hwa_base_mutex.h"
#include "hwa_base_eigendef.h"
#include "hwa_lidar_utility.h"
#include "hwa_lidar_proc.h"
#include "hwa_lidar_proc_segmenter.h"
using namespace hwa_base;
using namespace hwa_set;

typedef std::pair<size_t, size_t> IndexRange;

namespace hwa_lidar
{ 
    class lidar_frame:public lidar_proc
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  _set     setbase control
        */
        explicit lidar_frame(set_base* _set);
        
        /** @brief default destructor. */
        ~lidar_frame() {};

        /**
        * @brief lidar frame main function
        *
        * @param[in] t                cur lidar time
        * @return 
            @retvec LidarFrame        Lidar frame data after processing
        */
        virtual LidarFrame PreProcessPointCloud(const double& t);

        /**
        * @brief read lidar pointcloud from bin file
        *
        * @param[in] in_file        lidar file path
        * @return
            @retvec PointCloud        point cloud data 
        */
        pcl::PointCloud<pcl::PointXYZI> readKittiBinData(const std::string &in_file);

        /**
        * @brief read lidar pointcloud from txt file
        *
        * @param[in] in_file    lidar file path
        * @return
            @retvec PointCloud        point cloud data
        */
        pcl::PointCloud<pcl::PointXYZI> readKittiTxtData(const std::string &filepath);

        /**
        * @brief read lidar pointcloud from PCD file
        *
        * @param[in] in_file    lidar file path
        * @return
            @retvec PointCloud        point cloud data
        */
        pcl::PointCloud<pcl::PointXYZI> readPCDData(std::string &in_file);
        /**
        * @brief classification of the lidar pointcloud
        *
        * @param[in] laserCloudIn    lidar pointcloud
        */
        //void handleCloud(pcl::PointCloud<pcl::PointXYZI>& laserCloudIn);

        /**
        * @brief remove the close points in lidar pointcloud
        *
        * @param[in] cloud_in    lidar pointcloud
        * @param[in] thres        distance thershold of the close point
        * @param[out] cloud_out    lidar pointcloud
        */
        void removeClosedPointCloud(pcl::PointCloud<pcl::PointXYZI> &cloud_in,
            pcl::PointCloud<pcl::PointXYZI> &cloud_out, float thres);

        /**
        * @brief clear the history information
        */
        void clear();

        /**
        * @brief Extract features from lidar frames
        *
        * @param[in] laserCloudIn    lidar pointcloud
        */
        void ExtractFeatures(pcl::PointCloud<pcl::PointXYZI>& laserCloudIn/*, const Time& scanTime*/);

        /**
        * @brief Extract planar patches from lidar frames
        *
        * @param[in] pcs            center points
        * @param[in] ncs            cormal vector
        * @param[in] near_Points    points near the center points
        * @param[in] Qcs            noise convariance
        * @param[in] near_num        number of near poins
        */
        void ExtractPlanarPatches(std::vector<Triple> &pcs, std::vector<Triple> &ncs, std::vector<std::vector<Triple>> &near_Points, std::vector<Matrix> Qcs, int near_num = 5);

        /**
        * @brief calculate planar patches from lidar frames
        *
        * @param[in] points            points data buffer
        * @param[in] pc                center point
        * @param[in] nc                normal vector
        * @return
            @retvec =true            calculate successfully
            @retvec =false            failed
        */
        bool CalPlanarPatch(std::vector<Triple> &points, Triple &pc, Triple &nc);

        /**
        * @brief Merge planar patches after get them
        *
        * @param[in] pcs            center points
        * @param[in] ncs            cormal vector
        * @param[in] near_Points    points near the center points
        * @param[in] Qcs            noise convariance
        * @param[in] iter            number of iteration
        */
        void MergePlanarPatches(std::vector<Triple> &pcs, std::vector<Triple> &ncs, std::vector<std::vector<Triple>> &near_points, std::vector<Matrix> Qcs, int iter = 2);

        /**
        * @brief find the largest number
        * @param[in] a1                number 1
        * @param[in] a2                number 2
        * @param[in] a3                number 3
        * @param[out] max            maximum number
        * @return
            @retvec =1                number 1 is the largest
            @retvec =2                number 2 is the largest
            @retvec =3                number 3 is the largest
        */
        int max3(double a1, double a2, double a3, double &max);
    public:
        std::vector<pcl::PointCloud<pcl::PointXYZI> > _laserCloudScans;        ///< lidar scan point cloud data
        pcl::PointCloud<pcl::PointXYZI> _laserCloud;                        ///< full resolution input cloud
        std::vector<IndexRange> _scanIndices;                                ///< start and end indices of the individual scans withing the full resolution cloud
        pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;      ///< sharp corner points cloud
        pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;  ///< less sharp corner points cloud
        pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;      ///< flat surface points cloud
        pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat;     ///< less flat surface points cloud

        // use for plane patch tracking    
        bool use_pp;                            ///< use planar patch or not
        std::vector<Triple> _pcs;                ///< data buffer of center points
        std::vector<Triple> _ncs;                ///< data buffer of normal vector
        std::vector<Matrix> _Qcs;                ///< data buffer of noise convariance matrix
        std::vector<std::vector<Triple>> _near_points;

        // use for LeGo LOAM segmenter
        bool use_segmenter;
        lidar_proc_segmenter segmenter;

        int n_scans=0;                            ///< number of scans
        double close_threshold=0;                ///< TODO
        double scanPeriod=0;                    ///< lidar scan period
        double time=0;                            ///< timestamp of the lidar
        dataLIDAR lidar_msg;                    ///< original observation data
    };

}//end of namespace hwa_lidar

#endif
