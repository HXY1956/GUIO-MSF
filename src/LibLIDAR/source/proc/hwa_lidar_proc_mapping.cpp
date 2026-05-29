#include "hwa_lidar_proc_mapping.h"
#include "hwa_lidar_frame.h"
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>

namespace hwa_lidar
{
    lidar_proc_mapping::lidar_proc_mapping(double resolution)
    {
        laserCloudCornerFromMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<pcl::PointXYZI>());

        ///< kd-tree
        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());


        for (int i = 0; i < laserCloudNum; i++)
        {
            laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
            laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

    }

    void lidar_proc_mapping::reset()
    {
        ///< Clear the corresponding point set of the previous frame
        correspondCornerFeature_.currCornerPointCloud.clear();
        correspondCornerFeature_.correspondCornerPointCloudA.clear();
        correspondCornerFeature_.correspondCornerPointCloudB.clear();
        correspondCornerFeature_.currCornerPointCloud_inlast.clear();

        correspondSurfFeature_.currSurfPointCloud.clear();
        correspondSurfFeature_.correspondSurfPointCloudA.clear();
        correspondSurfFeature_.correspondSurfPointCloudB.clear();
        correspondSurfFeature_.correspondSurfPointCloudC.clear();
        correspondSurfFeature_.currSurfPointCloud_inlast.clear();
        correspondSurfFeature_.negative_OA_dot_norm.clear();
        correspondSurfFeature_.norm.clear();

        lidarMapObs = lidarMappingObs();
    }


    void lidar_proc_mapping::addPointcloudToMap(const LidarFrame &frame)
    {
        if (frame.empty)
            return;

        if (!systemInited_)
        {
            first_R_l_e = frame.R_l_e;
            first_t_l_e = frame.t_l_e;
            systemInited_ = true;
        }
        curr_R_l_e = frame.R_l_e;
        curr_t_l_e = frame.t_l_e;

        transformAssociateToMap();

        for (int i = 0; i < frame.LessSharp->points.size(); i++)
        {
            pcl::PointXYZI pointSel;
            pcl::PointXYZI point = frame.LessSharp->points[i];
            pointAssociateToMap(point, pointSel);

            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

            if (pointSel.x + 25.0 < 0)
                cubeI--;
            if (pointSel.y + 25.0 < 0)
                cubeJ--;
            if (pointSel.z + 25.0 < 0)
                cubeK--;

            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudCornerArray[cubeInd]->push_back(pointSel);
            }
        }
        for (int i = 0; i < frame.LessSurf->points.size(); i++)
        {
            pcl::PointXYZI pointSel;
            pcl::PointXYZI point = frame.LessSurf->points[i];
            pointAssociateToMap(point, pointSel);

            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

            if (pointSel.x + 25.0 < 0)
                cubeI--;
            if (pointSel.y + 25.0 < 0)
                cubeJ--;
            if (pointSel.z + 25.0 < 0)
                cubeK--;

            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudSurfArray[cubeInd]->push_back(pointSel);
            }
        }

        for (int i = 0; i < laserCloudValidNum; i++)
        {
            int ind = laserCloudValidInd[i];

            if (laserCloudCornerArray[ind]->size() < 1)
                continue;
            pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
            downSample(laserCloudCornerArray[ind], tmpCorner, 0.2);
            laserCloudCornerArray[ind] = tmpCorner;

            pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
            downSample(laserCloudSurfArray[ind], tmpSurf, 0.4);
            laserCloudSurfArray[ind] = tmpSurf;
        }

    }

    //构建当前帧与局部地图（submap）的几何约束（点-线、点-面），为后端位姿优化/滤波提供观测
    void lidar_proc_mapping::process(const LidarFrame &frame)
    {
        if (frame.empty)
        {
            std::cout << "ERROR:the pose of lidar frame need to be initialized!" << std::endl;
            getchar();
        }

        reset();

        if (!systemInited_)
        {
            first_R_l_e = frame.R_l_e;
            first_t_l_e = frame.t_l_e;
            systemInited_ = true;
            return;
        }

        curr_R_l_e = frame.R_l_e;
        curr_t_l_e = frame.t_l_e;

        transformAssociateToMap();
        int centerCubeI = int((curr_t_l_w.x() + 25.0) / 50.0) + laserCloudCenWidth;
        int centerCubeJ = int((curr_t_l_w.y() + 25.0) / 50.0) + laserCloudCenHeight;
        int centerCubeK = int((curr_t_l_w.z() + 25.0) / 50.0) + laserCloudCenDepth;

        if (curr_t_l_w.x() + 25.0 < 0)
            centerCubeI--;
        if (curr_t_l_w.y() + 25.0 < 0)
            centerCubeJ--;
        if (curr_t_l_w.z() + 25.0 < 0)
            centerCubeK--;

        while (centerCubeI < 3)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                for (int k = 0; k < laserCloudDepth; k++)
                {
                    int i = laserCloudWidth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; i >= 1; i--)
                    {
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeI++;
            laserCloudCenWidth++;
        }

        while (centerCubeI >= laserCloudWidth - 3)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                for (int k = 0; k < laserCloudDepth; k++)
                {
                    int i = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; i < laserCloudWidth - 1; i++)
                    {
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeI--;
            laserCloudCenWidth--;
        }

        while (centerCubeJ < 3)
        {
            for (int i = 0; i < laserCloudWidth; i++)
            {
                for (int k = 0; k < laserCloudDepth; k++)
                {
                    int j = laserCloudHeight - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; j >= 1; j--)
                    {
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeJ++;
            laserCloudCenHeight++;
        }

        while (centerCubeJ >= laserCloudHeight - 3)
        {
            for (int i = 0; i < laserCloudWidth; i++)
            {
                for (int k = 0; k < laserCloudDepth; k++)
                {
                    int j = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; j < laserCloudHeight - 1; j++)
                    {
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeJ--;
            laserCloudCenHeight--;
        }

        while (centerCubeK < 3)
        {
            for (int i = 0; i < laserCloudWidth; i++)
            {
                for (int j = 0; j < laserCloudHeight; j++)
                {
                    int k = laserCloudDepth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; k >= 1; k--)
                    {
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeK++;
            laserCloudCenDepth++;
        }

        while (centerCubeK >= laserCloudDepth - 3)
        {
            for (int i = 0; i < laserCloudWidth; i++)
            {
                for (int j = 0; j < laserCloudHeight; j++)
                {
                    int k = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; k < laserCloudDepth - 1; k++)
                    {
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeK--;
            laserCloudCenDepth--;
        }

        laserCloudValidNum = 0;
        laserCloudSurroundNum = 0;

        for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
        {
            for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
            {
                for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
                {
                    if (i >= 0 && i < laserCloudWidth &&
                        j >= 0 && j < laserCloudHeight &&
                        k >= 0 && k < laserCloudDepth)
                    {
                        laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        laserCloudValidNum++;
                        laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        laserCloudSurroundNum++;
                    }
                }
            }
        }
        ///< points in the choosed submap cubes
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < laserCloudValidNum; i++)
        {
            *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
            *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }

        int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
        
        std::vector<int> index;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputSurf(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::removeNaNFromPointCloud(*frame.Sharp, *inputSharp, index);
        pcl::removeNaNFromPointCloud(*frame.Surf, *inputSurf, index);

        ///< downsize the feature points
        laserCloudCornerStack.reset(new pcl::PointCloud<pcl::PointXYZI>());
        downSample(inputSharp,laserCloudCornerStack, 0.2);
        
        laserCloudSurfStack.reset(new pcl::PointCloud<pcl::PointXYZI>());
        downSample(inputSurf, laserCloudSurfStack, 0.4);

        std::cout << "input size(surf/corner):" << laserCloudSurfStack->points.size() << "," << laserCloudCornerStack->points.size() << std::endl;
        if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
        {
            std::cout << "submap size(surf/corner):" << laserCloudSurfFromMapNum << "," << laserCloudCornerFromMapNum << std::endl;
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

            //for (int i = 0; i < laserCloudCornerStack->points.size(); i++)
            //{
            //    pcl::PointXYZI pointOri, pointSel;
            //    std::vector<int> pointSearchInd; std::vector<float> pointSearchSqDis;
            //    pointOri = laserCloudCornerStack->points[i];
            //    pointAssociateToMap(pointOri, pointSel);
            //    kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            //    if (pointSearchSqDis[4] < 1)
            //    {
            //        std::vector<Triple> nearCorners;
            //        Triple center(0, 0, 0);
            //        for (int j = 0; j < 5; j++)
            //        {
            //            Triple tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
            //                laserCloudCornerFromMap->points[pointSearchInd[j]].y,
            //                laserCloudCornerFromMap->points[pointSearchInd[j]].z);
            //            center = center + tmp;
            //            nearCorners.push_back(tmp);
            //        }
            //        center = center / 5.0;
            //        SO3 covMat = SO3::Zero();
            //        for (int j = 0; j < 5; j++)
            //        {
            //            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
            //            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            //        }
            //        // 计算协方差矩阵的特征值和特征向量，用于判断这5个点是不是呈线状分布，此为PCA的原理
            //        Eigen::SelfAdjointEigenSolver<SO3> saes(covMat);
            //        // if is indeed line feature
            //        // note Eigen library sort eigenvalues in increasing order
            //        Triple unit_direction = saes.eigenvectors().col(2);// 如果5个点呈线状分布，最大的特征值对应的特征向量就是该线的方向向量
            //        Triple curr_point(pointOri.x, pointOri.y, pointOri.z);
            //        Triple curr_point_w(pointSel.x, pointSel.y, pointSel.z);
            //        if (saes.eigenvalues()[2] > 4 * saes.eigenvalues()[1])// 如果最大的特征值 >> 其他特征值，则5个点确实呈线状分布，否则认为直线“不够直
            //        {
            //            Triple point_on_line = center;
            //            Triple point_a, point_b;
            //            // 从中心点沿着方向向量向两端移动0.1m，构造线上的两个点
            //            point_a = 0.1 * unit_direction + point_on_line;
            //            point_b = -0.1 * unit_direction + point_on_line;
            //            Triple v_ij = point_a - point_b;
            //            Triple v_lj = curr_point_w - point_b;
            //            Triple v_lji = v_lj.cross(v_ij);
            //            double length_ij = v_ij.norm();
            //            double area = v_lji.norm();
            //            double d_e = area / length_ij / 2;
            //            if (fabs(d_e) < 0.5)
            //            {
            //                correspondCornerFeature_.currCornerPointCloud.push_back(curr_point);
            //                correspondCornerFeature_.currCornerPointCloud_inlast.push_back(curr_point_w);
            //                correspondCornerFeature_.correspondCornerPointCloudA.push_back(point_a);
            //                correspondCornerFeature_.correspondCornerPointCloudB.push_back(point_b);
            //            }
            //        }
            //    }
            //}

            for (int i = 0; i < laserCloudSurfStack->points.size(); i++)
            {
                pcl::PointXYZI pointOri, pointSel;
                std::vector<int> pointSearchInd; std::vector<float> pointSearchSqDis;
                pointOri = laserCloudSurfStack->points[i];
                pointAssociateToMap(pointOri, pointSel);

                kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                // 用上面的2个矩阵表示平面方程就是 matA0 * norm（A, B, C） = matB0，这是个超定方程组，因为数据个数超过未知数的个数
                if (pointSearchSqDis[4] < 1.0)
                {
                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                    }
                    ///< find the norm of plane
                    Triple norm = matA0.colPivHouseholderQr().solve(matB0);
                    // Ax + By + Cz + 1 = 0，全部除以法向量的模长，方程依旧成立，而且使得法向量归一化了
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    ///< Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离公式 = fabs(Ax0 + By0 + Cz0 + D) / sqrt(A^2 + B^2 + C^2)
                        double distance = fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                            norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                            norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm);
                        ///< if OX * n larger than 0.2, then plane is not fit well
                        if (distance > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Triple curr_point(pointOri.x, pointOri.y, pointOri.z);
                    Triple curr_point_w(pointSel.x, pointSel.y, pointSel.z);
                    Triple point_a(laserCloudSurfFromMap->points[pointSearchInd[0]].x, laserCloudSurfFromMap->points[pointSearchInd[0]].y, laserCloudSurfFromMap->points[pointSearchInd[0]].z);
                    Triple point_b(laserCloudSurfFromMap->points[pointSearchInd[1]].x, laserCloudSurfFromMap->points[pointSearchInd[1]].y, laserCloudSurfFromMap->points[pointSearchInd[1]].z);
					Triple point_c(laserCloudSurfFromMap->points[pointSearchInd[2]].x, laserCloudSurfFromMap->points[pointSearchInd[2]].y, laserCloudSurfFromMap->points[pointSearchInd[2]].z);
                    if (planeValid)
                    {
                        double residual = norm.dot(curr_point_w) + negative_OA_dot_norm;
                        if (fabs(residual) < 0.8)
                        {
                            correspondSurfFeature_.currSurfPointCloud.push_back(curr_point);
                            correspondSurfFeature_.currSurfPointCloud_inlast.push_back(curr_point_w);
                            correspondSurfFeature_.correspondSurfPointCloudA.push_back(point_a);
                            correspondSurfFeature_.correspondSurfPointCloudB.push_back(point_b);
                            correspondSurfFeature_.correspondSurfPointCloudC.push_back(point_c);
                            correspondSurfFeature_.norm.push_back(norm);
                            correspondSurfFeature_.negative_OA_dot_norm.push_back(negative_OA_dot_norm);
                        }
                    }
                }

            }
        }

        lidarMapObs.cornerFeature = correspondCornerFeature_;
        lidarMapObs.surfFeature = correspondSurfFeature_;
        lidarMapObs.curr_R_l_e = frame.R_l_e;
        lidarMapObs.curr_t_l_e = frame.t_l_e;
        lidarMapObs.last_R_l_e = first_R_l_e;
        lidarMapObs.last_t_l_e = first_t_l_e;
    }

    void lidar_proc_mapping::transformAssociateToMap()
    {
        curr_R_l_w = first_R_l_e.transpose() * curr_R_l_e;
        curr_t_l_w = first_R_l_e.transpose() * (curr_t_l_e - first_t_l_e);
    }

    void lidar_proc_mapping::pointAssociateToMap(pcl::PointXYZI & pi, pcl::PointXYZI & po)
    {
        Triple point_curr(pi.x, pi.y, pi.z);
        Triple point_w;
        point_w = curr_R_l_w * point_curr + curr_t_l_w;
        po.x = point_w.x();
        po.y = point_w.y();
        po.z = point_w.z();
        po.intensity = pi.intensity;
    }

    void lidar_proc_mapping::downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double& leaf_size)
    {
        if (cloud->points.size() == 0)
            return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
        if (cloud->empty())
            return;
        pcl::PointXYZI first_pt = cloud->points[0];

        ///> 为避免数值过大，降采样前先平移
        for (int i = 0; i < cloud->points.size(); i++)
        {
            cloud->points[i].x = cloud->points[i].x - first_pt.x;
            cloud->points[i].y = cloud->points[i].y - first_pt.y;
            cloud->points[i].z = cloud->points[i].z - first_pt.z;
        }

        ///> 降采样
        cloud->is_dense = false;
        pcl::VoxelGrid<pcl::PointXYZI> downer;
        downer.setInputCloud(cloud);
        downer.setLeafSize(leaf_size, leaf_size, leaf_size);
        downer.filter(*filtered);
        cloud->clear();

        ///> 移回去
        for (int i = 0; i < filtered->points.size(); i++)
        {
            filtered->points[i].x = filtered->points[i].x + first_pt.x;
            filtered->points[i].y = filtered->points[i].y + first_pt.y;
            filtered->points[i].z = filtered->points[i].z + first_pt.z;
        }

        ///> 覆盖原点云
        *cloud = *filtered;
    }

    void lidar_proc_mapping::downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered, const double& leaf_size)
    {
        if (cloud->points.size() == 0)
            return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<int> index; 
        pcl::removeNaNFromPointCloud(*cloud, *pcloud_ptr, index);
        if (cloud->empty())
            return;
        pcl::PointXYZI first_pt = pcloud_ptr->points[0];

        ///> 为避免数值过大，降采样前先平移
        for (int i = 0; i < pcloud_ptr->points.size(); i++)
        {
            pcloud_ptr->points[i].x = pcloud_ptr->points[i].x - first_pt.x;
            pcloud_ptr->points[i].y = pcloud_ptr->points[i].y - first_pt.y;
            pcloud_ptr->points[i].z = pcloud_ptr->points[i].z - first_pt.z;
        }

        ///> 降采样
        pcloud_ptr->is_dense = false;
        
        pcl::VoxelGrid<pcl::PointXYZI> downer;
        downer.setInputCloud(pcloud_ptr);
        downer.setLeafSize(leaf_size, leaf_size, leaf_size);
        downer.filter(*filtered);
        cloud->clear();

        ///> 移回去
        for (int i = 0; i < filtered->points.size(); i++)
        {
            filtered->points[i].x = filtered->points[i].x + first_pt.x;
            filtered->points[i].y = filtered->points[i].y + first_pt.y;
            filtered->points[i].z = filtered->points[i].z + first_pt.z;
        }
    }
}

namespace hwa_lidar {

    void lidar_proc_global_mapping::transformAssociateToMap(const SO3& curr_R_l_e, const Triple& curr_t_l_e)
    {
        curr_R_l_w = first_R_l_e.transpose() * curr_R_l_e;
        curr_t_l_w = first_R_l_e.transpose() * (curr_t_l_e - first_t_l_e);
    }

    void lidar_proc_global_mapping::run()
    {
        isRunning = true;
        
        global_map = std::make_shared<CloudType>();

        mapping_thread = std::thread(
            &lidar_proc_global_mapping::process,
            this
        );
    }

    void lidar_proc_global_mapping::stop()
    {
        {
            std::lock_guard<std::mutex> lock(keyframe_mutex);

            isRunning = false;
        }

        keyframe_cv.notify_all();

        if (mapping_thread.joinable())
        {
            mapping_thread.join();
        }

        savePCDFileBinary("./map/global_map.pcd",
            0.2);
    }

    void lidar_proc_global_mapping::process()
    {
        while (true)
        {
            KeyFrame lidarframe;

            {
                std::unique_lock<std::mutex> lock(keyframe_mutex);

                keyframe_cv.wait(lock, [&]()
                    {
                        return !keyframe_queue.empty() || !isRunning;
                    });

                if (!isRunning && keyframe_queue.empty())
                {
                    break;
                }

                lidarframe = keyframe_queue.front();

                keyframe_queue.pop_front();
            }

            processKeyFrame(lidarframe);
        }
    }

    CloudPtr lidar_proc_global_mapping::point_management(CloudPtr cloudin) {
		CloudPtr cloudout(new CloudType());
        for (int i = 0; i < cloudin->points.size(); i++)
        {
            pcl::PointXYZI pointSel = cloudin->points[i];

            int cubeI = int((pointSel.x + 25.0) / 50.0);
            int cubeJ = int((pointSel.y + 25.0) / 50.0);
            int cubeK = int((pointSel.z + 25.0) / 50.0);

            if (pointSel.x + 25.0 < 0)
                cubeI--;
            if (pointSel.y + 25.0 < 0)
                cubeJ--;
            if (pointSel.z + 25.0 < 0)
                cubeK--;

            if (abs(cubeI) < laserCloudWidth &&
                abs(cubeJ) < laserCloudHeight &&
                abs(cubeK) < laserCloudDepth)
            {
				cloudout->points.push_back(pointSel);
            }
        }
        return cloudout;
    }

    void lidar_proc_global_mapping::processKeyFrame(
        const KeyFrame& kf)
    {
        CloudPtr dsCloud(new CloudType());

        CloudPtr cloud_in = point_management(kf.cloud);

        lidar_proc_mapping::downSample(cloud_in, dsCloud, 0.2);

        CloudPtr worldCloud(new CloudType());

        //transformAssociateToMap(kf.R.matrix(), kf.t);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T.block<3, 3>(0, 0) = kf.R.matrix();
        T.block<3, 1>(0, 3) = kf.t;

        pcl::transformPointCloud(
            *dsCloud,
            *worldCloud,
            T
        );

        {
            std::lock_guard<std::mutex> lock(global_map_mutex);

            *global_map += *worldCloud;
        }
    }

    void lidar_proc_global_mapping::savePCDFileBinary(
        const std::string& save_path,
        const double& leaf_size)
    {
        CloudPtr save_cloud(new CloudType());

        {
            std::lock_guard<std::mutex> lock(global_map_mutex);

            *save_cloud = *global_map;
        }

        if (save_cloud->empty())
        {
            std::cout << "[Global Mapping] Empty global map.\n";
            return;
        }

        std::cout << "[Global Mapping] Raw points: "
            << save_cloud->size() << std::endl;

        lidar_proc_mapping::downSample(
            save_cloud,
            leaf_size
        );

        std::cout << "[Global Mapping] Downsampled points: "
            << save_cloud->size() << std::endl;

        std::filesystem::path p(save_path);

        if (!p.parent_path().empty())
        {
            std::filesystem::create_directories(
                p.parent_path()
            );
        }

        int ret = pcl::io::savePCDFileBinary(
            save_path,
            *save_cloud
        );

        if (ret == 0)
        {
            std::cout << "[Global Mapping] Saved map to:\n"
                << save_path << std::endl;
        }
        else
        {
            std::cout << "[Global Mapping] Failed to save map.\n";
        }
    }
}

