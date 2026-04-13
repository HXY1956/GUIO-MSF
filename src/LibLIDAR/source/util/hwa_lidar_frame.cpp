#include "hwa_lidar_frame.h"
#include <iostream>

using namespace std;

namespace hwa_lidar
{ 
    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];

    hwa_lidar::lidar_frame::lidar_frame(set_base* _set) :lidar_proc(_set)
    {
        n_scans = dynamic_cast<set_lidar*>(_set)->n_scans();
        close_threshold = dynamic_cast<set_lidar*>(_set)->close_threshold();
        use_pp = dynamic_cast<set_lidar*>(_set)->use_pp();
        scanPeriod = 1.0 / double(dynamic_cast<set_lidar*>(_set)->freq());
        use_segmenter = dynamic_cast<set_lidar*>(_set)->use_segmenter();

        if (use_segmenter) segmenter.systemInitialization(n_scans);    }

    bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

    void lidar_frame::clear()
    {
        _laserCloudScans.clear();
        _laserCloud.clear();
        _scanIndices.clear();
        _cornerPointsSharp.clear();
        _cornerPointsLessSharp.clear();
        _surfacePointsFlat.clear();
        _surfacePointsLessFlat.clear();
        _pcs.clear();
        _ncs.clear();
        _Qcs.clear();
        _near_points.clear();
    }
    
    void lidar_frame::removeClosedPointCloud(pcl::PointCloud<pcl::PointXYZI> &cloud_in,
        pcl::PointCloud<pcl::PointXYZI> &cloud_out, float thres)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    //read lidar file (format:txt)
    pcl::PointCloud<pcl::PointXYZI> lidar_frame::readKittiTxtData(const std::string &filepath)
    {
        pcl::PointCloud<pcl::PointXYZI> pointCloudPtrTmp;
        // load point cloud
        ifstream input;
        input.open(filepath);
        if (!input.good())
        {
            std::cout << "Could not read file: " << filepath << std::endl;
            exit(EXIT_FAILURE);
        }
        while (!input.eof()) {
            string line;
            getline(input, line);
            if (line == "")
                continue;
            vector<std::string> arr = split(line, " ");
            pcl::PointXYZI mPoint;
            mPoint.x = stod(arr[0]);
            mPoint.y = stod(arr[1]);
            mPoint.z = stod(arr[2]);
            pointCloudPtrTmp.points.push_back(mPoint);
        }

        return pointCloudPtrTmp;
    }

    //read lidar file (format:bin)
    pcl::PointCloud<pcl::PointXYZI> lidar_frame::readKittiBinData(const std::string &in_file)
    {
        //load point cloud
        std::ifstream input(in_file.c_str(), std::ios::in | std::ios::binary);
        if (!input.good()) {
            std::cout << "Could not read file: " << in_file << std::endl;
            exit(EXIT_FAILURE);
        }
        input.seekg(0, std::ios::beg);


        //int lidar_line_max = 64;

        pcl::PointCloud<pcl::PointXYZI> pointCloudPtrTmp;
        for (int i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI mPoint;
            input.read((char *)&mPoint.x, sizeof(float));
            input.read((char *)&mPoint.y, sizeof(float));
            input.read((char *)&mPoint.z, sizeof(float));
            input.read((char *)&mPoint.intensity, sizeof(float));
            pointCloudPtrTmp.points.push_back(mPoint);
        }
        input.close();

        return pointCloudPtrTmp;
    }

    //read lidar file (format:ascii with header)
    pcl::PointCloud<pcl::PointXYZI> lidar_frame::readPCDData(std::string &in_file)
    {
        pcl::PointCloud<pcl::PointXYZI> pointCloudPtrTmp;
        ifstream input;
        input.open(in_file);
        if (!input.good())
        {
            std::cout << "Could not read PCD file : " << in_file << std::endl;
            exit(EXIT_FAILURE);
        }
        while (!input.eof()) {
            string line;
            getline(input, line);
            if (line == "")
                continue;
            vector<std::string> arr = split(line, " ");
            stringstream sin(arr[0]);
            double digit_test;
            if (!(sin >> digit_test))
                continue;
            pcl::PointXYZI mPoint;
            mPoint.x = stod(arr[0]);
            mPoint.y = stod(arr[1]);
            mPoint.z = stod(arr[2]);
            pointCloudPtrTmp.points.push_back(mPoint);
        }
        return pointCloudPtrTmp;
    }


    LidarFrame hwa_lidar::lidar_frame::PreProcessPointCloud(const double& t)
    {
        cout << "==================================" << endl;
        cout << "start process lidar" << endl;

        pcl::PointCloud<pcl::PointXYZI> lidarPoints;
        if (cur_lidar_path.lidar_path.find("pcd") != string::npos)
            lidarPoints = readPCDData(cur_lidar_path.lidar_path);///< get pointCloud from pcd
        else
            lidarPoints = readKittiBinData(cur_lidar_path.lidar_path);///< get pointCloud from bin
        
        ExtractFeatures(lidarPoints);

        if (use_pp)
        {
            //extract planar features
            ExtractPlanarPatches(_pcs, _ncs, _near_points, _Qcs,15 );
            MergePlanarPatches(_pcs, _ncs, _near_points, _Qcs,3);
        }

        if (use_segmenter)
        {
            segmenter.process(lidarPoints.makeShared());
        }


        LidarFrame frame =LidarFrame();
        frame.fullCloud.clear();
        frame.fullCloud = _laserCloud;
        frame.LessSharp.clear();
        frame.LessSharp = _cornerPointsLessSharp;
        frame.LessSurf.clear();
        frame.LessSurf = _surfacePointsLessFlat;
        frame.Sharp.clear();
        frame.Sharp = _cornerPointsSharp;
        frame.Surf.clear();
        frame.Surf = _surfacePointsFlat;
        if (use_pp)
        {
            frame.ncs = _ncs;
            frame.pcs = _pcs;
            frame._near_points = _near_points;
        }
        if (use_segmenter)
        {
            frame.centroids = segmenter.getCentroids();
            frame.directions = segmenter.getMaindirections();
            assert(frame.centroids.size() == frame.directions.size());
        }

        return frame;
    }

    void lidar_frame::ExtractFeatures(pcl::PointCloud<pcl::PointXYZI> &laserCloudIn)
    {
        clear();

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        removeClosedPointCloud(laserCloudIn, laserCloudIn, close_threshold);

        ///< determine scan start and end orientations
        int cloudSize = laserCloudIn.size();
        float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
        float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
            laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
        if (endOri - startOri > 3 * M_PI) {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI) {
            endOri += 2 * M_PI;
        }


        std::vector<int> scanStartInd(n_scans, 0);
        std::vector<int> scanEndInd(n_scans, 0);
        

        bool halfPassed = false;
        int count = cloudSize;
        pcl::PointXYZI point;
        _laserCloudScans.resize(n_scans);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn[i].x;
            point.y = laserCloudIn[i].y;
            point.z = laserCloudIn[i].z;

            ///< calculate vertical point angle and scan ID
            float angle = std::atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / float(M_PI);
            int scanID = 0;
            if (n_scans == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (n_scans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (n_scans == 32)
            {
                scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (n_scans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (n_scans == 64)
            {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = n_scans / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                break;
            }

            ///< calculate horizontal point angle
            float ori = -std::atan2(point.y, point.x);
            if (!halfPassed)
            {
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;

                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            ///< calculate relative scan time based on point orientation
            float relTime = (ori - startOri) / (endOri - startOri);
            point.intensity = scanID + scanPeriod * relTime;


            _laserCloudScans[scanID].push_back(point);
        }
        cloudSize = count;
        printf("points size %d \n", cloudSize);

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (int i = 0; i < n_scans; i++)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += _laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }
        _laserCloud = *laserCloud;

        ///< extract feature
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }


        for (int i = 0; i < n_scans; i++)
        {
            if (scanEndInd[i] - scanStartInd[i] < 6)
                continue;
            pcl::PointCloud<pcl::PointXYZI>::Ptr surfacePointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
            for (int j = 0; j < 6; j++)
            {

                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);
                
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > 0.1)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {
                            cloudLabel[ind] = 2;
                            _cornerPointsSharp.push_back(laserCloud->points[ind]);
                            _cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            _cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind - l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < 0.1)
                    {

                        cloudLabel[ind] = -1;
                        _surfacePointsFlat.push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind - l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                    {
                        surfacePointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
            }

            pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
            downSizeFilter.setInputCloud(surfacePointsLessFlatScan);
            downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            _surfacePointsLessFlat += surfPointsLessFlatScanDS;
        }


    }


    void lidar_frame::ExtractPlanarPatches(vector<Triple> &pcs, vector<Triple> &ncs, vector<vector<Triple>> &near_Points, vector<Matrix> Qcs ,int near_num)
    {
        cout << "surf_size:" << _surfacePointsFlat.points.size() << endl;
        cout << "Less surf size:" << _surfacePointsLessFlat.points.size() << endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr kdtreeSurf(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::KdTreeFLANN<pcl::PointXYZI> SurfaceKDTree_;
        std::vector<int> indices;

        //< TODO: How to choose candidate points
        pcl::removeNaNFromPointCloud(_surfacePointsLessFlat, *kdtreeSurf, indices);

        SurfaceKDTree_.setInputCloud(kdtreeSurf);

        std::vector<int> pointSearchInd;//index
        std::vector<float> pointSearchSqDis;//distance
        for (int i = 0; i < _surfacePointsFlat.points.size(); i++)
        {
            pcl::PointXYZI point = _surfacePointsFlat.points[i];
            SurfaceKDTree_.nearestKSearch(point, near_num, pointSearchInd, pointSearchSqDis);
            
            if (pointSearchSqDis[near_num - 1] < 4)            {
                std::vector<Triple> nearCorners;
                Triple center(0, 0, 0);
                for (int j = 0; j < near_num; j++)
                {
                    Triple tmp(kdtreeSurf->points[pointSearchInd[j]].x,
                        kdtreeSurf->points[pointSearchInd[j]].y,
                        kdtreeSurf->points[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                // the point center
                center = center / near_num;

                // covariance
                SO3 covMat = SO3::Zero();
                vector<double> num_x, num_y, num_z;
                for (int j = 0; j < near_num; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    num_x.push_back(tmpZeroMean.x());
                    num_y.push_back(tmpZeroMean.y());
                    num_z.push_back(tmpZeroMean.z());
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }
                double xx = covMat(0, 0);
                double xy = covMat(0, 1);
                double xz = covMat(0, 2);
                double yy = covMat(1, 1);
                double yz = covMat(1, 2);
                double zz = covMat(2, 2);
                double det_x = yy * zz - yz * yz;
                double det_y = xx * zz - xz * xz;
                double det_z = xx * yy - xy * xy;
                double det_max;
                int max_index = max3(det_x, det_y, det_z, det_max);
                if (det_max <= 0.0) cerr << "det_max<0.0 --- not span a plane" << endl;
                Triple dir;
                if (max_index == 1) dir = Triple(det_x, xz*yz - xy * zz, xy*yz - xz * yy);
                else if (max_index == 2) dir = Triple(xz*yz - xy * zz, det_y, xy*xz - yz * xx);
                else dir = Triple(xy*yz - xz * yy, xy*xz - yz * xx, det_z);
                Triple norm_dir = dir.normalized();

                //使用A-LOAM的方法计算法向量和中心店
                // 求面的法向量就不是用的PCA了（虽然论文中说还是PCA），使用的是最小二乘拟合，是为了提效？不确定
                // 假设平面不通过原点，则平面的一般方程为Ax + By + Cz + 1 = 0，用这个假设可以少算一个参数，提效。
                //Eigen::Matrix<double, 5, 3> matA0;
                Matrix matA0 = Matrix::Zero(near_num, 3);
                //Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                Matrix matB0 = -1 * Matrix::Ones(near_num, 1);
                // 用上面的2个矩阵表示平面方程就是 matA0 * norm（A, B, C） = matB0，这是个超定方程组，因为数据个数超过未知数的个数
                for (int j = 0; j < near_num; j++)
                {
                    matA0(j, 0) = kdtreeSurf->points[pointSearchInd[j]].x;
                    matA0(j, 1) = kdtreeSurf->points[pointSearchInd[j]].y;
                    matA0(j, 2) = kdtreeSurf->points[pointSearchInd[j]].z;
                }
                // 求解这个最小二乘问题，可得平面的法向量，find the norm of plane
                Triple norm = matA0.colPivHouseholderQr().solve(matB0);
                // Ax + By + Cz + 1 = 0，全部除以法向量的模长，方程依旧成立，而且使得法向量归一化了
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                // Here n(pa, pb, pc) is unit norm of plane
                bool planeValid = true;
                for (int j = 0; j < near_num; j++)
                {
                    // 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离公式 = fabs(Ax0 + By0 + Cz0 + D) / sqrt(A^2 + B^2 + C^2)
                    if (fabs(norm(0) * kdtreeSurf->points[pointSearchInd[j]].x +
                        norm(1) * kdtreeSurf->points[pointSearchInd[j]].y +
                        norm(2) * kdtreeSurf->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;// 平面没有拟合好，平面“不够平”
                        break;
                    }
                }

                Triple delta_norm1 = norm - norm_dir; Triple delta_norm2 = norm + norm_dir;
                if ((delta_norm1.norm() <= 0.2 || delta_norm2.norm() <= 0.2) && planeValid)
                
                {
                    //successful
                    pcs.push_back(center);
                    ncs.push_back(norm_dir);
                    near_Points.push_back(nearCorners);
                }
                //计算Qc
                /*Eigen::Matrix<double, 6, 6> Qc = Eigen::Matrix<double, 6, 6>::Zero();
                Eigen::Matrix<double, 3, 3> Qj = 2 * Eigen::Matrix<double, 3, 3>::Identity();    ///raw point measurement noise
                double d_A_x, d_A_y, d_A_z, d_B_x, d_B_y, d_B_z, d_C_x, d_C_y, d_C_z, d_D_x, d_D_y, d_D_z;
                double _A, _B, _C, _D;
                if ((delta_norm1.norm() <= 0.2 || delta_norm2.norm() <= 0.2) && planeValid)
                {
                    for (int i = 0; i < near_num; i++)
                    {
                        Eigen::Matrix<double, 6, 3> H = Eigen::Matrix<double, 6, 3>::Zero();
                        auto x = num_x.at(i);
                        auto y = num_y.at(i);
                        auto z = num_z.at(i);
                        if (max_index == 1)
                        {
                            _A = yy * zz - yz * yz;  _B = xz * yz - xy * zz;   _C = xy * yz - xz * yy;
                            _D = sqrt(_A*_A+ _B*_B + _C*_C);
                            d_A_x = 0;
                            d_A_y = 2 * zz*y - 2 * yz*z;
                            d_A_z = 2 * yy*z - 2 * yz*y;
                            d_B_x = yz * z - zz * y;
                            d_B_y = xz * z - zz * x;
                            d_B_z = xz * y + yz * x - 2 * xy*z;
                            d_C_x = yz * y - yy * z;
                            d_C_y = yz * x + xy * z - 2 * xz*y;
                            d_C_z = xy * y - yy * x;
                            d_D_x = (_A*d_A_x + _B * d_B_x + _C * d_C_x) / _D;
                            d_D_y = (_A*d_A_y + _B * d_B_y + _C * d_C_y) / _D;
                            d_D_z = (_A*d_A_z + _B * d_B_z + _C * d_C_z) / _D;
                            H(0, 0) = (d_A_x*_D - _A * d_D_x) / _D / _D / (1.0 + 1.0 / near_num);
                            H(0, 1) = (d_A_y*_D - _A * d_D_y) / _D / _D / (1.0 + 1.0 / near_num);
                            H(0, 2) = (d_A_z*_D - _A * d_D_z) / _D / _D / (1.0 + 1.0 / near_num);
                            H(1, 0) = (d_B_x*_D - _B * d_D_x) / _D / _D / (1.0 + 1.0 / near_num);
                            H(1, 1) = (d_B_y*_D - _B * d_D_y) / _D / _D / (1.0 + 1.0 / near_num);
                            H(1, 2) = (d_B_z*_D - _B * d_D_z) / _D / _D / (1.0 + 1.0 / near_num);
                            H(2, 0) = (d_C_x*_D - _C * d_D_x) / _D / _D / (1.0 + 1.0 / near_num);
                            H(2, 1) = (d_C_y*_D - _C * d_D_y) / _D / _D / (1.0 + 1.0 / near_num);
                            H(2, 2) = (d_C_z*_D - _C * d_D_z) / _D / _D / (1.0 + 1.0 / near_num);
                            H(3, 0) = 1.0 / near_num;  
                            H(4, 1) = 1.0 / near_num;  
                            H(5, 2) = 1.0 / near_num;
                        }
                        else if (max_index == 2)
                        {
                            _A = xz * yz - xy * zz;  _B = xx * zz - xz * xz;   _C = xy * xz - yz * xx;
                            _D = sqrt(_A*_A + _B * _B + _C * _C);
                            d_A_x = yz * z - zz * y;
                            d_A_y = xz * z - zz * x;
                            d_A_z = xz * y + yz * x - 2 * xy * z;
                            d_B_x = 2 * zz * x - 2 * xz *z;
                            d_B_y = 0;
                            d_B_z = 2 * xx *z - 2 * xz * x;
                            d_C_x = xy * z + xz * y - 2 *yz * z;
                            d_C_y = xz * x - xx * z;
                            d_C_z = xy * x - xx * y;
                            d_D_x = (_A*d_A_x + _B * d_B_x + _C * d_C_x) / _D;
                            d_D_y = (_A*d_A_y + _B * d_B_y + _C * d_C_y) / _D;
                            d_D_z = (_A*d_A_z + _B * d_B_z + _C * d_C_z) / _D;
                            H(0, 0) = (d_A_x*_D - _A * d_D_x) / _D / _D / (1.0 + 1.0 / near_num);
                            H(0, 1) = (d_A_y*_D - _A * d_D_y) / _D / _D / (1.0 + 1.0 / near_num);
                            H(0, 2) = (d_A_z*_D - _A * d_D_z) / _D / _D / (1.0 + 1.0 / near_num);
                            H(1, 0) = (d_B_x*_D - _B * d_D_x) / _D / _D / (1.0 + 1.0 / near_num);
                            H(1, 1) = (d_B_y*_D - _B * d_D_y) / _D / _D / (1.0 + 1.0 / near_num);
                            H(1, 2) = (d_B_z*_D - _B * d_D_z) / _D / _D / (1.0 + 1.0 / near_num);
                            H(2, 0) = (d_C_x*_D - _C * d_D_x) / _D / _D / (1.0 + 1.0 / near_num);
                            H(2, 1) = (d_C_y*_D - _C * d_D_y) / _D / _D / (1.0 + 1.0 / near_num);
                            H(2, 2) = (d_C_z*_D - _C * d_D_z) / _D / _D / (1.0 + 1.0 / near_num);
                            H(3, 0) = 1.0 / near_num;  
                            H(4, 1) = 1.0 / near_num;  
                            H(5, 2) = 1.0 / near_num;
                        }
                        else
                        {
                            _A = xy * yz - xz * yy;  _B = xy * xz - yz * xx;   _C = xx * yy - xy * xy;
                            _D = sqrt(_A*_A + _B * _B + _C * _C);
                            d_A_x = yz * y - yy * z;
                            d_A_y = xy * z + yz * x - 2 * xz * y;
                            d_A_z = xy * y - yy * x;
                            d_B_x = xy * z + xz * y - 2 * yz * x;
                            d_B_y = xz * x - xx * z;
                            d_B_z = xy * x - xx * y;
                            d_C_x = 2 * yy * x - 2 * xy * y;
                            d_C_y = 2 * xx * y - 2 * xy * x;
                            d_C_z = 0;
                            d_D_x = (_A*d_A_x + _B * d_B_x + _C * d_C_x) / _D;
                            d_D_y = (_A*d_A_y + _B * d_B_y + _C * d_C_y) / _D;
                            d_D_z = (_A*d_A_z + _B * d_B_z + _C * d_C_z) / _D;
                            H(0, 0) = (d_A_x*_D - _A * d_D_x) / _D / _D / (1 + 1.0 / near_num);
                            H(0, 1) = (d_A_y*_D - _A * d_D_y) / _D / _D / (1 + 1.0 / near_num);
                            H(0, 2) = (d_A_z*_D - _A * d_D_z) / _D / _D / (1 + 1.0 / near_num);
                            H(1, 0) = (d_B_x*_D - _B * d_D_x) / _D / _D / (1 + 1.0 / near_num);
                            H(1, 1) = (d_B_y*_D - _B * d_D_y) / _D / _D / (1 + 1.0 / near_num);
                            H(1, 2) = (d_B_z*_D - _B * d_D_z) / _D / _D / (1 + 1.0 / near_num);
                            H(2, 0) = (d_C_x*_D - _C * d_D_x) / _D / _D / (1 + 1.0 / near_num);
                            H(2, 1) = (d_C_y*_D - _C * d_D_y) / _D / _D / (1 + 1.0 / near_num);
                            H(2, 2) = (d_C_z*_D - _C * d_D_z) / _D / _D / (1 + 1.0 / near_num);
                            H(3, 0) = 1.0 / near_num;  
                            H(4, 1) = 1.0 / near_num;  
                            H(5, 2) = 1.0 / near_num;
                        }
                        Qc += H * Qj * H.transpose();
                    }
                    _Qcs.push_back(Qc);*/
                    //cout << "Qc:" << endl << setprecision(10) << Qc << endl;
                }
                //Eigen::Matrix<double, 6, 6> Qc = Eigen::Matrix<double, 6, 6>::Zero();
                //Eigen::Matrix<double, 3, 3> Qj = 0.5*Eigen::Matrix<double, 3, 3>::Identity();
                //Triple delta_norm1 = norm - norm_dir; Triple delta_norm2 = norm + norm_dir;
                //if ((delta_norm1.norm() <= 0.2 || delta_norm2.norm() <= 0.2) && planeValid)
                //{
                //    cout << "loam_planevalid:" << planeValid << endl;
                //    cout << "loam_normnal:" << setprecision(10) << norm.transpose() << endl;
                //    if (delta_norm1.norm() <= 0.2) cout << "delxa_norm:" << setprecision(10) << delta_norm1.transpose() << endl;
                //    if (delta_norm2.norm() <= 0.2) cout << "delxa_norm:" << setprecision(10) << delta_norm2.transpose() << endl;
                //    cout << endl;
                //    //D是det_z
                //    for (int i = 0; i < 5; i++)
                //    {
                //        Eigen::Matrix<double, 6, 3> H = Eigen::Matrix<double, 6, 3>::Zero();
                //        auto x = num_x.at(i);
                //        auto y = num_y.at(i);
                //        auto z = num_z.at(i);
                //        H(0, 0) = (y*yz - z * yy)*det_z - (yz*xy - xz * yy)*(2 * x*yy - 2 * y*xy);
                //        H(0, 0) = H(0, 0) / det_z / det_z;
                //        H(0, 1) = (x*yz + z * xy - 2 * y*xz)*det_z - (yz*xy - xz * yy)*(2 * y*xx - 2 * x*xy);
                //        H(0, 1) = H(0, 1) / det_z / det_z;
                //        H(0, 2) = (y*xy - x * yy) / det_z;
                //        H(1, 0) = (y*xz + z * xy - 2 * x*yz)*det_z - (xy*xz - xx * yz)*(2 * yy*x - 2 * y*xy);
                //        H(1, 0) = H(1, 0) / det_z / det_z;
                //        H(1, 1) = (x*xz - z * xx)*det_z - (xy*xz - xx * yz)*(2 * y*xx - 2 * x*xy);
                //        H(1, 1) = H(1, 1) / det_z / det_z;
                //        H(1, 2) = (x*xy - y * xx) / det_z;
                //        H(3, 3) = 1 / 5.0;
                //        H(4, 4) = 1 / 5.0;
                //        H(5, 5) = 1 / 5.0;
                //        Qc += H * Qj*H.transpose();
                //    }
                //    cout << "Qc:" << endl << setprecision(10) << Qc << endl;
            //}
            else
            {
                cerr << "!!!wrong pointSearchSqDis!!!" << endl;
            }

        }
    }
    
    int lidar_frame::max3(double a1, double a2, double a3, double &max)
    {
        if (a1 >= a2 && a1 >= a3)
        {
            max = a1;
            return 1;
        }
        if (a2 >= a1 && a2 >= a3)
        {
            max = a2;
            return 2;
        }
        if (a3 >= a1 && a3 >= a2)
        {
            max = a3;
            return 3;
        }
    }

    bool lidar_frame::CalPlanarPatch(vector<Triple> &points, Triple &pc, Triple &nc)
    {
        std::vector<Triple> nearCorners;
        Triple center(0, 0, 0);
        for (int j = 0; j < points.size(); j++)
        {
            center = center + points.at(j);
            nearCorners.push_back(points.at(j));
        }
        // center
        center = center / points.size();

        // covariance
        SO3 covMat = SO3::Zero();
        
        for (int j = 0; j < points.size(); j++)
        {
            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        double xx = covMat(0, 0);
        double xy = covMat(0, 1);
        double xz = covMat(0, 2);
        double yy = covMat(1, 1);
        double yz = covMat(1, 2);
        double zz = covMat(2, 2);
        double det_x = yy * zz - yz * yz;
        double det_y = xx * zz - xz * xz;
        double det_z = xx * yy - xy * xy;
        double det_max;
        int max_index = max3(det_x, det_y, det_z, det_max);
        if (det_max <= 0.0) cerr << "det_max<0.0 --- not span a plane" << endl;
        Triple dir;
        if (max_index == 1) dir = Triple(det_x, xz*yz - xy * zz, xy*yz - xz * yy);
        else if (max_index == 2) dir = Triple(xz*yz - xy * zz, det_y, xy*xz - yz * xx);
        else dir = Triple(xy*yz - xz * yy, xy*xz - yz * xx, det_z);
        Triple norm_dir = dir.normalized();

        //使用A-LOAM的方法计算法向量和中心店
        // 求面的法向量就不是用的PCA了（虽然论文中说还是PCA），使用的是最小二乘拟合，是为了提效？不确定
        // 假设平面不通过原点，则平面的一般方程为Ax + By + Cz + 1 = 0，用这个假设可以少算一个参数，提效。
        //Eigen::Matrix<double, 5, 3> matA0;
        Matrix matA0 = Matrix::Zero(points.size(), 3);
        //Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        Matrix matB0 = -1 * Matrix::Ones(points.size(), 1);
        // 用上面的2个矩阵表示平面方程就是 matA0 * norm（A, B, C） = matB0，这是个超定方程组，因为数据个数超过未知数的个数
        for (int j = 0; j < points.size(); j++)
        {
            matA0(j, 0) = points.at(j).x();
            matA0(j, 1) = points.at(j).y();
            matA0(j, 2) = points.at(j).z();
        }
        // 求解这个最小二乘问题，可得平面的法向量，find the norm of plane
        Triple norm = matA0.colPivHouseholderQr().solve(matB0);
        // Ax + By + Cz + 1 = 0，全部除以法向量的模长，方程依旧成立，而且使得法向量归一化了
        double negative_OA_dot_norm = 1 / norm.norm();
        norm.normalize();

        // Here n(pa, pb, pc) is unit norm of plane
        bool planeValid = true;
        for (int j = 0; j < points.size(); j++)
        {
            // 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离公式 = fabs(Ax0 + By0 + Cz0 + D) / sqrt(A^2 + B^2 + C^2)
            if (fabs(norm(0) * points.at(j).x() +
                norm(1) * points.at(j).y() +
                norm(2) * points.at(j).z() + negative_OA_dot_norm) > 0.2)
            {
                planeValid = false;// 平面没有拟合好，平面“不够平”
                break;
            }
        }
        //if (norm(0,0) < 0) norm = -norm;
        //if (norm_dir(0,0) < 0) norm_dir = -norm_dir;

        Triple delta_norm1 = norm - norm_dir; Triple delta_norm2 = norm + norm_dir;
        if ((delta_norm1.norm() <= 0.2 || delta_norm2.norm() <= 0.2) && planeValid)
        {
            //该点是成功的面点

            pc = center;
            nc = norm_dir;
            return true;
        }
        else return false;
    }


    void lidar_frame::MergePlanarPatches(vector<Triple> &pcs, vector<Triple> &ncs, vector<vector<Triple>> &near_points, vector<Matrix> Qcs, int iter_number)
    {
        cout << "before_merge_size:" << pcs.size() << endl;
        for (int k = 0; k < iter_number; k++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcs_pcl(new pcl::PointCloud<pcl::PointXYZI>());
            pcs_pcl = Triple2PointXYZI(pcs);
        
            pcl::KdTreeFLANN<pcl::PointXYZI> KDTree_pppc;
            KDTree_pppc.setInputCloud(pcs_pcl);
            int count = 0;
            int count_new = 0;
            map<int, int> indexs_front, indexs_back;
            std::vector<int> pointSearchInd;//index
            std::vector<float> pointSearchSqDis;//distance
            //
            for (int i = 0; i < pcs_pcl->points.size(); i++)
            {
                pcl::PointXYZI point = pcs_pcl->points[i];
                KDTree_pppc.nearestKSearch(point, 2, pointSearchInd, pointSearchSqDis);

                int search_id;
                if (i != pointSearchInd.at(0)) search_id = pointSearchInd.at(0);
                else search_id = pointSearchInd.at(1);
                Triple cur_point = Triple(pcs_pcl->points[i].x, pcs_pcl->points[i].y, pcs_pcl->points[i].z);
                Triple search_point = Triple(pcs_pcl->points[search_id].x, pcs_pcl->points[search_id].y, pcs_pcl->points[search_id].z);

                
                if (pointSearchSqDis[1] < 4 && i < search_id)                {
                    // calculate residual
                    Triple res_norm1 = ncs.at(i) - ncs.at(search_id);
                    Triple res_norm2 = ncs.at(i) + ncs.at(search_id);
                    double res_distance = ncs.at(i).transpose()*(cur_point - search_point);
            
                    // To avoid
                    //a0->b1
                    //a1->b1
                    //a0->b1
                    //b1->b2
                    
                    
                    /*
                    Eigen::Matrix<double, 4, 1> r_m = Eigen::Matrix<double, 4, 1>::Zero();
                    Eigen::Matrix<double, 4, 6> H_s = Eigen::Matrix<double, 4, 6>::Zero();
                    Eigen::Matrix<double, 4, 6> H_n = Eigen::Matrix<double, 4, 6>::Zero();
                    r_m.block(0, 0, 3, 1) = ncs.at(i) - ncs.at(search_id);
                    r_m(3, 0) = ncs.at(i).transpose()*(cur_point - search_point);
                    H_s.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
                    H_s.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Zero();
                    H_s.block(3, 0, 1, 3) = cur_point.transpose() - search_point.transpose();
                    H_s.block(3, 3, 1, 3) = ncs.at(i).transpose();
                    H_n.block(0, 0, 3, 3) = -1 * Eigen::Matrix<double, 3, 3>::Identity();
                    H_n.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Zero();
                    H_n.block(3, 0, 1, 3) = Eigen::Matrix<double, 1, 3>::Zero();
                    H_n.block(3, 3, 1, 3) = -1 * ncs.at(i).transpose();
                    Matrix P1 = H_s * Qcs.at(i) * H_s.transpose();
                    Matrix P2 = H_n * Qcs.at(search_id) * H_n.transpose();
                    double gamma = r_m.transpose() * (P1 + P2).ldlt().solve(r_m);
                    */
                    //cout << "gamma:" << gamma << endl;
                    
                    auto repeat_iter1 = indexs_back.find(search_id);
                    auto repear_iter2 = indexs_back.find(i);
                    /*
                    if (gamma <= 0.102587 && repeat_iter1 == indexs_back.end() && repear_iter2 == indexs_back.end())
                    {
                        
                        count++;
                        indexs_front.insert(make_pair(i, search_id));
                        indexs_back.insert(make_pair(search_id, i));
                        //count_new++;
                        //cout << "gamma pass"  << endl;
                    }
                    */
                    
                    if ((res_norm1.norm() < 0.2 || res_norm2.norm() < 0.2) && res_distance <= 0.8 && repeat_iter1 == indexs_back.end() && repear_iter2 == indexs_back.end())
                    {
                        count++;
                        indexs_front.insert(make_pair(i, search_id));
                        indexs_back.insert(make_pair(search_id, i));
                    }
                    
                }

            }
        
            cout << "same_plane_count:" << count << endl;


            //进行merge操作 ,重新计算pc nc
            //这里可能会有问题，属于重复添加
            // a0->b0
            // a1->b0 这种情况可能会出现
            vector<Triple> merge_ncs, merge_pcs;
            vector<vector<Triple>> merge_near_points;
            for (int i = 0; i < ncs.size(); i++)
            {
                map<int, int > ::iterator iter_front, iter_back;
                iter_front = indexs_front.find(i);
                iter_back = indexs_back.find(i);

                //前向找不到，后向也找不到，可以直接加入
                if (iter_front == indexs_front.end() && iter_back == indexs_back.end())
                {
                    //not find
                    merge_ncs.push_back(ncs.at(i));
                    merge_pcs.push_back(pcs.at(i));
                    merge_near_points.push_back(near_points.at(i));
                }
                if (iter_front != indexs_front.end())
                {
                    //find
                    vector<Triple> tmp_merge_points;
                    auto first_near_points = near_points.at(iter_front->first);
                    auto second_near_points = near_points.at(iter_front->second);
                    tmp_merge_points.insert(tmp_merge_points.end(), first_near_points.begin(), first_near_points.end());
                    tmp_merge_points.insert(tmp_merge_points.end(), second_near_points.begin(), second_near_points.end());

                    //重新计算 nc和pc
                    Triple merge_pc, merge_nc;
                    if (CalPlanarPatch(tmp_merge_points, merge_pc, merge_nc))
                    {
                        merge_ncs.push_back(merge_nc);
                        merge_pcs.push_back(merge_pc);
                        merge_near_points.push_back(tmp_merge_points);
                    }
                    else
                    {
                        
                        merge_ncs.push_back(ncs.at(i));
                        merge_pcs.push_back(pcs.at(i));
                        merge_near_points.push_back(first_near_points);
                    }
                }
            }

            ncs.clear();
            ncs = merge_ncs;
            pcs.clear();
            pcs = merge_pcs;
            near_points.clear();
            near_points = merge_near_points;
            cout << "after_merge_size:" << ncs.size() << endl;
            
        }


    }
}