#include "hwa_lidar_frame.h"
#include <iostream>
#include <cmath>

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

        //if (use_segmenter) segmenter.systemInitialization(n_scans);    
    }

    bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

    void lidar_frame::clear()
    {
        _laserCloudScans.clear();
        _laserCloud = std::make_shared<CloudType>();
        _scanIndices.clear();
        _cornerPointsSharp = std::make_shared<CloudType>();
        _cornerPointsLessSharp = std::make_shared<CloudType>();
        _surfacePointsFlat = std::make_shared<CloudType>();
        _surfacePointsLessFlat = std::make_shared<CloudType>();
        _pcs.clear();
        _ncs.clear();
        _Qcs.clear();
        _near_points.clear();
    }
    
    void lidar_frame::removeClosedPointCloud(CloudPtr cloud_in,
        CloudPtr cloud_out, float thres)
    {
        if (cloud_in != cloud_out)
        {
            cloud_out->header = cloud_in->header;
            cloud_out->points.resize(cloud_in->points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in->points.size(); ++i)
        {
            if (cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y + cloud_in->points[i].z * cloud_in->points[i].z < thres * thres)
                continue;
            cloud_out->points[j] = cloud_in->points[i];
            j++;
        }
        if (j != cloud_in->points.size())
        {
            cloud_out->points.resize(j);
        }

        cloud_out->height = 1;
        cloud_out->width = static_cast<uint32_t>(j);
        cloud_out->is_dense = true;
    }

    //read lidar file (format:txt)
    CloudPtr lidar_frame::readKittiTxtData(const std::string &filepath)
    {
        CloudPtr pointCloudPtrTmp = std::make_shared<CloudType>();

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
            pointCloudPtrTmp->points.push_back(mPoint);
        }

        return pointCloudPtrTmp;
    }

    //read lidar file (format:bin)
    CloudPtr lidar_frame::readKittiBinData(const std::string &in_file)
    {
        //load point cloud
        std::ifstream input(in_file.c_str(), std::ios::in | std::ios::binary);
        if (!input.good()) {
            std::cout << "Could not read file: " << in_file << std::endl;
            exit(EXIT_FAILURE);
        }
        input.seekg(0, std::ios::beg);

        CloudPtr pointCloudPtrTmp = std::make_shared<CloudType>();
        for (int i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI mPoint;
            input.read((char *)&mPoint.x, sizeof(float));
            input.read((char *)&mPoint.y, sizeof(float));
            input.read((char *)&mPoint.z, sizeof(float));
            input.read((char *)&mPoint.intensity, sizeof(float));
            pointCloudPtrTmp->points.push_back(mPoint);
        }
        input.close();

        return pointCloudPtrTmp;
    }

    //read lidar file (format:ascii with header)
    CloudPtr lidar_frame::readPCDData(std::string &in_file)
    {
        CloudPtr pointCloudPtrTmp = std::make_shared<CloudType>();
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
            PointType mPoint;
            mPoint.x = stod(arr[0]);
            mPoint.y = stod(arr[1]);
            mPoint.z = stod(arr[2]);
            pointCloudPtrTmp->points.push_back(mPoint);
        }
        return pointCloudPtrTmp;
    }


    LidarFrame hwa_lidar::lidar_frame::PreProcessPointCloud(const double& t)
    {
        cout << "==================================" << endl;
        cout << "start process lidar" << endl;

        CloudPtr lidarPoints = std::make_shared<CloudType>();
        if (cur_lidar_path.lidar_path.find("pcd") != string::npos)
            lidarPoints = readPCDData(cur_lidar_path.lidar_path);///< get pointCloud from pcd
        else
            lidarPoints = readKittiBinData(cur_lidar_path.lidar_path);///< get pointCloud from bin
        
        ExtractFeatures(lidarPoints);

        if (use_pp)
        {
            //extract planar features
            ExtractPlanarPatches(_pcs, _ncs, _near_points, _Qcs, 8);
            MergePlanarPatches(_pcs, _ncs, _near_points, _Qcs,3);
        }

        if (use_segmenter)
        {
            segmenter.process(lidarPoints);
        }

        LidarFrame frame =LidarFrame();
        frame.fullCloud->clear();
        frame.fullCloud = _laserCloud;
        frame.LessSharp->clear();
        frame.LessSharp = _cornerPointsLessSharp;
        frame.LessSurf->clear();
        frame.LessSurf = _surfacePointsLessFlat;
        frame.Sharp->clear();
        frame.Sharp = _cornerPointsSharp;
        frame.Surf->clear();
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

    void lidar_frame::ExtractFeatures(CloudPtr laserCloudIn)
    {
        clear();

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        removeClosedPointCloud(laserCloudIn, laserCloudIn, close_threshold);

        ///< determine scan start and end orientations
        int cloudSize = laserCloudIn->size();
        float startOri = -std::atan2(laserCloudIn->operator[](0).y, laserCloudIn->operator[](0).x);
        float endOri = -std::atan2(laserCloudIn->operator[](cloudSize - 1).y,
            laserCloudIn->operator[](cloudSize - 1).x) + 2 * float(M_PI);
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
        PointType point;

        _laserCloudScans.resize(n_scans);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn->operator[](i).x;
            point.y = laserCloudIn->operator[](i).y;
            point.z = laserCloudIn->operator[](i).z;

            if (!pcl::isFinite(point) ||
                std::abs(point.x) > 1000 ||
                std::abs(point.y) > 1000 ||
                std::abs(point.z) > 1000)
            {
                count--;
                continue;
            }

            ///< calculate vertical point angle and scan ID
            float angle = std::atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / float(M_PI);
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

        CloudPtr laserCloud(new CloudType);
        for (int i = 0; i < n_scans; i++)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += _laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }
        _laserCloud = laserCloud;

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
                        if (cloudCurvature[ind] > 1e5) continue;
                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {
                            cloudLabel[ind] = 2;
                            _cornerPointsSharp->push_back(laserCloud->points[ind]);
                            _cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            _cornerPointsLessSharp->push_back(laserCloud->points[ind]);
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

                            cloudNeighborPicked[ind + l] = 1;
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
                        _surfacePointsFlat->push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 8)
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

                            cloudNeighborPicked[ind + l] = 1;
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

            pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

            downSampleChunked<pcl::PointXYZI>(surfacePointsLessFlatScan, 0.1, 20.0);

            *_surfacePointsLessFlat += *surfacePointsLessFlatScan;
        }
    }

    void lidar_frame::ExtractPlanarPatches(vector<Triple> &pcs, vector<Triple> &ncs, vector<vector<Triple>> &near_Points, vector<Matrix> Qcs ,int near_num)
    {
        cout << "surf_size:" << _surfacePointsFlat->points.size() << endl;
        cout << "Less surf size:" << _surfacePointsLessFlat->points.size() << endl;

        CloudPtr kdtreeSurf(new CloudType);
        pcl::KdTreeFLANN<PointType> SurfaceKDTree_;
        std::vector<int> indices;

        //< TODO: How to choose candidate points
        pcl::removeNaNFromPointCloud(*_surfacePointsLessFlat, *kdtreeSurf, indices);

        SurfaceKDTree_.setInputCloud(kdtreeSurf);

        std::vector<int> pointSearchInd;//index
        std::vector<float> pointSearchSqDis;//distance
        for (int i = 0; i < _surfacePointsFlat->points.size(); i++)
        {
            pcl::PointXYZI point = _surfacePointsFlat->points[i];
            SurfaceKDTree_.nearestKSearch(point, near_num, pointSearchInd, pointSearchSqDis);
            
            if (pointSearchSqDis[near_num - 1] < 4)            {
                std::vector<Triple> nearCorners;
                Triple center(0, 0, 0);

                // ===== 计算邻域中心点 =====
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

                // ===== 方法1：PCA估计法向量 =====
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


    void lidar_frame::MergePlanarPatches(
        vector<Triple>& pcs,
        vector<Triple>& ncs,
        vector<vector<Triple>>& near_points,
        vector<Matrix> Qcs,
        int iter_number)
    {
        cout << "before_merge_size:" << pcs.size() << endl;

        for (int iter = 0; iter < iter_number; iter++)
        {
            int N = pcs.size();
            if (N < 2) break;

            // ===== 1. 构建KD树 =====
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcs_pcl(new pcl::PointCloud<pcl::PointXYZI>());
            pcs_pcl = Triple2PointXYZI(pcs);

            pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
            kdtree.setInputCloud(pcs_pcl);

            // ===== 2. 并查集初始化 =====
            vector<int> parent(N);
            for (int i = 0; i < N; i++) parent[i] = i;

            function<int(int)> find_root = [&](int x) {
                if (parent[x] != x)
                    parent[x] = find_root(parent[x]);
                return parent[x];
                };

            auto union_set = [&](int a, int b) {
                int ra = find_root(a);
                int rb = find_root(b);
                if (ra != rb)
                    parent[ra] = rb;
                };

            // ===== 3. 建立连接（plane merge条件）=====
            const int K = min(5, N);
            std::vector<int> idx(K);
            std::vector<float> dist(K);

            for (int i = 0; i < N; i++)
            {
                pcl::PointXYZI query = pcs_pcl->points[i];

                if (kdtree.nearestKSearch(query, K, idx, dist) < K)
                    continue;

                for (int j = 1; j < K; j++) // 跳过自己
                {
                    int nb = idx[j];

                    // ---------- 1. 法向量一致性 ----------
                    double cos_theta = ncs[i].dot(ncs[nb]);
                    if (fabs(cos_theta) < 0.95) continue; // ~18°

                    // ---------- 2. 平面距离一致性 ----------
                    // plane: n·x + d = 0
                    double d_i = fabs(ncs[i].dot(pcs[i]));
                    double d_j = fabs(ncs[nb].dot(pcs[nb]));

                    double dist_plane = fabs(d_i - d_j);

                    if (dist_plane > 0.2) continue;

                    // ---------- 3. 空间距离限制 ----------
                    if (dist[j] > 4.0) continue;

                    // ---------- 满足条件，合并 ----------
                    union_set(i, nb);
                }
            }

            // ===== 4. 聚类（根据并查集）=====
            unordered_map<int, vector<int>> clusters;
            for (int i = 0; i < N; i++)
            {
                int root = find_root(i);
                clusters[root].push_back(i);
            }

            cout << "cluster_size: " << clusters.size() << endl;

            // ===== 5. 对每个cluster重新拟合平面 =====
            vector<Triple> new_pcs;
            vector<Triple> new_ncs;
            vector<vector<Triple>> new_near_points;

            for (auto& cluster : clusters)
            {
                vector<Triple> all_points;

                for (int id : cluster.second)
                {
                    auto& pts = near_points[id];
                    all_points.insert(all_points.end(), pts.begin(), pts.end());
                }

                Triple pc, nc;

                if (CalPlanarPatch(all_points, pc, nc))
                {
                    new_pcs.push_back(pc);
                    new_ncs.push_back(nc);
                    new_near_points.push_back(all_points);
                }
                else
                {
                    // fallback：选第一个
                    int id = cluster.second[0];
                    new_pcs.push_back(pcs[id]);
                    new_ncs.push_back(ncs[id]);
                    new_near_points.push_back(near_points[id]);
                }
            }

            pcs = new_pcs;
            ncs = new_ncs;
            near_points = new_near_points;

            cout << "after_merge_size: " << pcs.size() << endl;
        }
    }}