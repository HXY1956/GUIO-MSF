#include "hwa_lidar_proc_mapping.h"

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
            return;///< waiting for new data to arrive...

        if (!systemInited_)
        {
            first_R_l_e = frame.R_l_e;
            first_t_l_e = frame.t_l_e;
            //第一帧会进入该函数，改变初始化状态
            systemInited_ = true;
        }
        curr_R_l_e = frame.R_l_e;
        curr_t_l_e = frame.t_l_e;

        ///< get the pose of current frame
        transformAssociateToMap();

        //for (int i = 0; i < laserCloudCornerStack->points.size(); i++)
        for (int i = 0; i < frame.LessSharp.points.size(); i++)
        {
            pcl::PointXYZI pointSel;
            // Lidar坐标系转到world坐标系
            //pointAssociateToMap(laserCloudCornerStack->points[i], pointSel);
            pcl::PointXYZI point = frame.LessSharp.points[i];
            pointAssociateToMap(point, pointSel);
            // 计算本次的特征点的IJK坐标，进而确定添加到哪个cube中
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
        //for (int i = 0; i < laserCloudSurfStack->points.size(); i++)
        for (int i = 0; i < frame.LessSurf.points.size(); i++)
        {
            pcl::PointXYZI pointSel;
            pcl::PointXYZI point = frame.LessSurf.points[i];
            //pcl::PointXYZI point = laserCloudSurfStack->points[i];
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

        // whd那边是0.2和0.4
        // 因为新增加了点云，对之前已经存有点云的cube全部重新进行一次降采样
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
        }

        curr_R_l_e = frame.R_l_e;
        curr_t_l_e = frame.t_l_e;

        ///< get the pose of current frame
        transformAssociateToMap();
        ///< get the cube position of current frame center
        int centerCubeI = int((curr_t_l_w.x() + 25.0) / 50.0) + laserCloudCenWidth;
        int centerCubeJ = int((curr_t_l_w.y() + 25.0) / 50.0) + laserCloudCenHeight;
        int centerCubeK = int((curr_t_l_w.z() + 25.0) / 50.0) + laserCloudCenDepth;

        if (curr_t_l_w.x() + 25.0 < 0)
            centerCubeI--;
        if (curr_t_l_w.y() + 25.0 < 0)
            centerCubeJ--;
        if (curr_t_l_w.z() + 25.0 < 0)
            centerCubeK--;

        // 是IJK坐标系我们是可以移动的，所以这6个while loop
            // 的作用就是调整IJK坐标系（也就是调整所有cube位置），使得载体在IJK坐标系的坐标范围处于
            // 3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18，目的是为了防止后续向
            // 四周拓展cube（图中的黄色cube就是拓展的cube）时，index（即IJK坐标）成为负数。
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
                    for (; i >= 1; i--)// 在I方向上，将cube[I] = cube[I-1],最后一个空出来的cube清空点云，实现IJK坐标系向I轴负方向移动一个cube的
                                       // 效果，从相对运动的角度看，就是图中的五角星在IJK坐标系下向I轴正方向移动了一个cube，如下面的动图所示，所
                                       // 以centerCubeI最后++，laserCloudCenWidth也会++，为下一帧Mapping时计算五角星的IJK坐标做准备。
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

        // 向IJ坐标轴的正负方向各拓展2个cube，K坐标轴的正负方向各拓展1个cube，上图中五角星所在的蓝色cube就是当前位置
        // 所处的cube，拓展的cube就是黄色的cube，这些cube就是submap的范围
        for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
        {
            for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
            {
                for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
                {
                    if (i >= 0 && i < laserCloudWidth &&
                        j >= 0 && j < laserCloudHeight &&
                        k >= 0 && k < laserCloudDepth)// 如果坐标合法
                    {
                        // 记录submap中的所有cube的index，记为有效index
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
            // 将有效index的cube中的点云叠加到一起组成submap的特征点云
            *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
            *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }

        int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
        
        std::vector<int> index;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputSurf(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::removeNaNFromPointCloud(frame.LessSharp, *inputSharp, index);
        pcl::removeNaNFromPointCloud(frame.LessSurf, *inputSurf, index);
        
        ///< downsize the feature points
        laserCloudCornerStack.reset(new pcl::PointCloud<pcl::PointXYZI>());
        downSample(inputSharp,laserCloudCornerStack,0.2);
        

        laserCloudSurfStack.reset(new pcl::PointCloud<pcl::PointXYZI>());
        downSample(inputSurf, laserCloudSurfStack, 0.4);

        ///< construct the 
        //首帧根本不会进来
        std::cout << "input size(surf/corner):" << laserCloudSurfStack->points.size() << "," << laserCloudCornerStack->points.size() << std::endl;
        if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
        {
            std::cout << "submap size(surf/corner):" << laserCloudSurfFromMapNum << "," << laserCloudCornerFromMapNum << std::endl;
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

            //寻找合适的边缘点匹配的观测值
            ///< find the corresponding points for corner features
            for (int i = 0; i < laserCloudCornerStack->points.size(); i++)
            {
                pcl::PointXYZI pointOri, pointSel;
                std::vector<int> pointSearchInd; std::vector<float> pointSearchSqDis;
                ///< transform the point to w frame
                pointOri = laserCloudCornerStack->points[i];

                // 需要注意的是submap中的点云都是world坐标系，而当前帧的点云都是Lidar坐标系，所以
                // 在搜寻最近邻点时，先用预测的Mapping位姿w_curr，将Lidar坐标系下的特征点变换到world坐标系下
                pointAssociateToMap(pointOri, pointSel);

                // 在submap的corner特征点（target）中，寻找距离当前帧corner特征点（source）最近的5个点
                kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[4] < 1)
                {
                    std::vector<Triple> nearCorners;
                    Triple center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Triple tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                            laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                            laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                        center = center + tmp;
                        nearCorners.push_back(tmp);
                    }
                    // 计算这个5个最近邻点的中心
                    center = center / 5.0;
                    // 协方差矩阵
                    SO3 covMat = SO3::Zero();
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                    }
                    // 计算协方差矩阵的特征值和特征向量，用于判断这5个点是不是呈线状分布，此为PCA的原理
                    Eigen::SelfAdjointEigenSolver<SO3> saes(covMat);

                    // if is indeed line feature
                    // note Eigen library sort eigenvalues in increasing order
                    Triple unit_direction = saes.eigenvectors().col(2);// 如果5个点呈线状分布，最大的特征值对应的特征向量就是该线的方向向量
                    Triple curr_point(pointOri.x, pointOri.y, pointOri.z);
                    Triple curr_point_w(pointSel.x, pointSel.y, pointSel.z);
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])// 如果最大的特征值 >> 其他特征值，则5个点确实呈线状分布，否则认为直线“不够直
                    {
                        Triple point_on_line = center;
                        Triple point_a, point_b;
                        // 从中心点沿着方向向量向两端移动0.1m，构造线上的两个点
                        point_a = 0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;

                        Triple v_lj = point_a - point_b;
                        Triple v_li = curr_point_w - point_b;
                        Triple v_lji = v_lj.cross(v_li);
                        double length_lj = sqrt(v_lj.x()*v_lj.x() + v_lj.y()*v_lj.y() + v_lj.z()*v_lj.z());
                        double length_ljxi = sqrt(v_lji.x()*v_lji.x() + v_lji.y()*v_lji.y() + v_lji.z()*v_lji.z());
                        double d_e = length_ljxi / length_lj;

                        if (fabs(d_e) > 1e-6)
                        {
                            correspondCornerFeature_.currCornerPointCloud.push_back(curr_point);
                            correspondCornerFeature_.currCornerPointCloud_inlast.push_back(curr_point_w);
                            correspondCornerFeature_.correspondCornerPointCloudA.push_back(point_a);
                            correspondCornerFeature_.correspondCornerPointCloudB.push_back(point_b);
                        }
                    }
                }
                /*
                else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
                {
                    Triple center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Triple tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                            laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                            laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                        center = center + tmp;
                    }
                    center = center / 5.0;
                    Triple curr_point(pointOri.x, pointOri.y, pointOri.z);
                    ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                }
                */
            }

            //寻找合适的平面点匹配的观测值
            ///< find the correspond points for surf features
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
                        ///< if OX * n larger than 0.2, then plane is not fit well
                        // 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离公式 = fabs(Ax0 + By0 + Cz0 + D) / sqrt(A^2 + B^2 + C^2)
                        if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                            norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                            norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Triple curr_point(pointOri.x, pointOri.y, pointOri.z);
                    Triple curr_point_w(pointSel.x, pointSel.y, pointSel.z);
                    if (planeValid)
                    {
                        double residual = norm.dot(curr_point_w) + negative_OA_dot_norm;
                        if (fabs(residual) > 1e-6)
                        {
                            correspondSurfFeature_.currSurfPointCloud.push_back(curr_point);
                            correspondSurfFeature_.currSurfPointCloud_inlast.push_back(curr_point_w);
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

        
        //首帧不会进入该函数
        // used for glfw
        if (systemInited_)
        {
            laserCloudCornerFromMap->clear();
            laserCloudSurfFromMap->clear();
            for (int i = 0; i < laserCloudValidNum; i++)
            {
                *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
            }
            submap_surf = PointXYZI2Triple(laserCloudSurfFromMap, first_R_l_e, first_t_l_e);
            submap_corner = PointXYZI2Triple(laserCloudCornerFromMap, first_R_l_e, first_t_l_e);
            
        }
    }

    //vector<Triple> lidar_proc_mapping::PointXYZI2Triple_ECEF(pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi)
    //{
    //    vector<Triple> xyz;
    //    for (int i = 0; i < xyzi->points.size(); i++)
    //    {
    //        xyz.push_back(first_R_l_e*Triple(xyzi->points[i].x, xyzi->points[i].y, xyzi->points[i].z)+first_t_l_e);
    //    }
    //    return xyz;
    //}

    void lidar_proc_mapping::transformAssociateToMap()
    {
        curr_R_l_w = first_R_l_e.transpose()*curr_R_l_e;
        curr_t_l_w = first_R_l_e.transpose()*(curr_t_l_e-first_t_l_e);
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
        pcl::PointXYZI first_pt = cloud->points[0];
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

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
        *cloud = std::move(*filtered);
    }

    void lidar_proc_mapping::downSample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered, const double& leaf_size)
    {
        if (cloud->points.size() == 0)
            return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<int> index; 
        pcl::removeNaNFromPointCloud(*cloud, *pcloud_ptr, index);
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

        ///> 覆盖原点云
        //*cloud = std::move(*filtered);
    }



    //void lidar_proc_mapping::ceresOptimize()
    //{
    //    int surf_size = lidarMapObs.surfFeature.currSurfPointCloud.size();
    //    int corner_size = lidarMapObs.cornerFeature.currCornerPointCloud.size();
    //    CorrespondCornerFeature& cornerf = lidarMapObs.cornerFeature;
    //    CorrespondSurfFeature& surff = lidarMapObs.surfFeature;

    //    //q_curr_last(x, y, z, w), t_curr_last

    //    Eigen::Quaterniond q_c_w(curr_R_l_w);
    //    Triple t_c_w = curr_t_l_w;
    //    double parameters[7] = { q_c_w.x(), q_c_w.y(), q_c_w.z(), q_c_w.w(), t_c_w.x(), t_c_w.y(), t_c_w.z() };
    //    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
    //    Eigen::Map<Triple> t_w_curr(parameters + 4);

    //    //build optimize problem
    //    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    //    ceres::LocalParameterization *q_parameterization =
    //        new ceres::EigenQuaternionParameterization();
    //    ceres::Problem::Options problem_options;

    //    ceres::Problem problem(problem_options);
    //    problem.AddParameterBlock(parameters, 4, q_parameterization);
    //    problem.AddParameterBlock(parameters + 4, 3);

    //    if (corner_size <= 0 || surf_size <= 0)
    //    {
    //        cout << "There are some problem in the lidar odometry observation!" << endl;
    //        return;
    //    }

    //    for (int i = 0; i < surf_size; i++)
    //    {
    //        //最后一个参数，经过雷达畸变矫正后设为1.0
    //        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(surff.currSurfPointCloud[i], surff.norm[i],surff.negative_OA_dot_norm[i]);
    //        problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
    //        //surf_correspondence++;
    //    }

    //    for (int i = 0; i < corner_size; i++)
    //    {
    //        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(cornerf.currCornerPointCloud[i], cornerf.correspondCornerPointCloudA[i], cornerf.correspondCornerPointCloudB[i], 1.0);
    //        problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
    //        //corner_correspondence++;
    //    }

    //    ceres::Solver::Options options;
    //    options.linear_solver_type = ceres::DENSE_QR;
    //    options.max_num_iterations = 4;
    //    options.minimizer_progress_to_stdout = false;
    //    options.check_gradients = false;
    //    options.gradient_check_relative_precision = 1e-4;
    //    ceres::Solver::Summary summary;
    //    ceres::Solve(options, &problem, &summary);

    //    //renew the pose of current lidar frame
    //    /*SO3 curr_R_l_e = last_rot * q_curr_last.toRotationMatrix();
    //    Triple curr_t_l_e = last_trans + last_rot * t_curr_last;*/

    //}
}

