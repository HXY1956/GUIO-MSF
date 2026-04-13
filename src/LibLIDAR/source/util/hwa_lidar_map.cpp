#include "hwa_lidar_map.h"

using namespace std;

namespace hwa_lidar
{
    lidar_map::lidar_map(double resolution):priorCornerMapOctree(resolution), priorSurfMapOctree(resolution)
    {
        ///< priorMap
        loadmap = false;
        last_mapcenter = Triple(-999, -999, -999);
        priorCornerMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
        priorSurfMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
        kdtreeCornerFromPriorMap=new pcl::KdTreeFLANN<pcl::PointXYZI>();
        kdtreeSurfFromPriorMap=new pcl::KdTreeFLANN<pcl::PointXYZI>();

        CornerSubmap.reset(new pcl::PointCloud<pcl::PointXYZI>());
        SurfSubmap.reset(new pcl::PointCloud<pcl::PointXYZI>());

        downSizeFilterCorner.setLeafSize(0.4f, 0.4f, 0.4f);
        downSizeFilterSurf.setLeafSize(0.8f, 0.8f, 0.8f);

        //priorCornerMapOctree.setResolution(32.0);
        //priorSurfMapOctree.setResolution(32.0);
    }

    void lidar_map::clearMeasurement()
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
    }

    pcl::PointCloud<pcl::PointXYZI> lidar_map::pclXYZ2XYZI(pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        pcl::PointCloud<pcl::PointXYZI> xyzi;
        if (cloud.size() == 0)
            return xyzi;
        for (int i = 0; i < cloud.size(); i++)
        {
            pcl::PointXYZI tmp_p;
            tmp_p.x=cloud[i].x;
            tmp_p.y = cloud[i].y;
            tmp_p.z = cloud[i].z;
            xyzi.push_back(tmp_p);
        }
        return xyzi;
    }

    //whd+new
    void lidar_map::getPriorMap(string &corner_path, string &surf_path,bool xyz)
    {
        priorCornerMap->clear();
        priorSurfMap->clear();
        if (xyz)
        {
            pcl::PointCloud<pcl::PointXYZ> tmp_corner, tmp_surf;
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(corner_path, tmp_corner) == -1
                || pcl::io::loadPCDFile<pcl::PointXYZ>(surf_path, tmp_surf) == -1)
            {
                PCL_ERROR("Couldn't read file from the given path");
                return;
            }
            *priorCornerMap = pclXYZ2XYZI(tmp_corner);
            *priorSurfMap = pclXYZ2XYZI(tmp_surf);
        }
        else
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(corner_path, *priorCornerMap) == -1
                || pcl::io::loadPCDFile<pcl::PointXYZI>(surf_path, *priorSurfMap) == -1)
            {
                PCL_ERROR("Couldn't read file from the given path");
                return;
            }
        }
        std::cout << "Loaded " << priorCornerMap->size() << " corner points from the given path" << std::endl;
        std::cout << "Loaded " << priorSurfMap->size() << " surf points from the given path" << std::endl;

        ///< add the priormap to the octree
        priorCornerMapOctree.setResolution(32.0);
        priorSurfMapOctree.setResolution(32.0);
        priorCornerMapOctree.setInputCloud(priorCornerMap);
        priorSurfMapOctree.setInputCloud(priorSurfMap);
        priorCornerMapOctree.addPointsFromInputCloud();
        priorSurfMapOctree.addPointsFromInputCloud();
        std::cout << "Sucess build the prior map octree!" << endl;
        cin.get();
        loadmap = true;
    }

    bool lidar_map::extractSurroundingPriorMap(Triple &center, float boxsize)
    {
        if (!loadmap)
        {
            std::cout << "Please get prior map first!" << endl;
            return false;
        }
        ///< cheak the distance to the last time calculating the surrounding map(if within the threshold remain using last surrounding map) 
        if ((center - last_mapcenter).norm() < 10)
        {
            if (CornerSubmap->points.size() > 10 && SurfSubmap->points.size() > 10)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        ///< get the surrounding box
        Eigen::Vector3f min_pt, max_pt,min_pt1,max_pt1;
        min_pt.x() = center.x() - boxsize;
        min_pt.y() = center.y() - boxsize;
        min_pt.z() = center.z() - boxsize;
        max_pt.x() = center.x() + boxsize;
        max_pt.y() = center.y() + boxsize;
        max_pt.z() = center.z() + boxsize;
        min_pt1 = min_pt;
        max_pt1 = max_pt;

        kdtreeCornerFromPriorMap=new pcl::KdTreeFLANN<pcl::PointXYZI>();
        kdtreeSurfFromPriorMap=new pcl::KdTreeFLANN<pcl::PointXYZI>();
        CornerSubmap->clear();
        SurfSubmap->clear();

        //std::cout << "priorSurfMapOctree size:" << priorSurfMapOctree.getInputCloud()->points.size() << endl;
        //std::cout << "priorCornerMapOctree size:" << priorCornerMapOctree.getInputCloud()->points.size() << endl;

        ///< get the pointcloud within the box
        std::vector<int> cornerIdxVec, surfIdxVec;
        priorCornerMapOctree.boxSearch(min_pt, max_pt, cornerIdxVec);
        priorSurfMapOctree.boxSearch(min_pt1, max_pt1, surfIdxVec);

        //std::cout << "cornerIdxVec size:" << cornerIdxVec.size() << endl;
        //std::cout << "surfIdxVec size:" << surfIdxVec.size() << endl;

        //std::cout << "priorCornerMap size:"<<priorCornerMap->points.size() << endl;
        //std::cout << "priorSurfMap size:"<<priorSurfMap->points.size() << endl;

        if (cornerIdxVec.size() < 10 || surfIdxVec.size() < 10)
        {
            cout << "not enough points in the submap!" << endl;
            return false;
        }

        for (int i = 0; i < cornerIdxVec.size(); i++)
            CornerSubmap->points.push_back(priorCornerMap->points[cornerIdxVec[i]]);
        for (int i = 0; i < surfIdxVec.size(); i++)
            SurfSubmap->points.push_back(priorSurfMap->points[surfIdxVec[i]]);

        /*cout << "CornerSubmap_size:" << CornerSubmap->points.size() << endl;
        cout << "SurfSubmap_size:" << SurfSubmap->points.size() << endl;
        getchar();*/

        ///< build the corresponding kdtree
        kdtreeCornerFromPriorMap.setInputCloud(CornerSubmap);
        kdtreeSurfFromPriorMap.setInputCloud(SurfSubmap);


        
        ///< update the last_mapcenter
        last_mapcenter = center;
        return true;
    }

    void lidar_map::setResolution(double resolution)
    {
        priorCornerMapOctree.setResolution(resolution);
        priorSurfMapOctree.setResolution(resolution);
    }

    bool lidar_map::alignToPriorMap(LidarFrame &frame)
    {
        if (!loadmap || !extractSurroundingPriorMap(frame.t_l_e, 100.0))
        {
            std::cout << "corner point:" << frame.t_l_e.transpose() << endl;
            std::cout << "Please get the prior map first!" << endl;
            return false;
        }

        //kdtreeCornerFromPriorMap->setInputCloud(CornerSubmap);
        //kdtreeSurfFromPriorMap->setInputCloud(SurfSubmap);
        //cout<<kdtreeCornerFromPriorMap.
        cout << "CornerKdtree_size:" << kdtreeCornerFromPriorMap.getInputCloud()->points.size() << endl;
        cout << "SurfKdtree_size:" << kdtreeSurfFromPriorMap.getInputCloud()->points.size() << endl;

        if (kdtreeCornerFromPriorMap.getInputCloud()->points.size() < 10 || kdtreeSurfFromPriorMap.getInputCloud()->points.size() < 10)
        {
            std::cout << "submap error!" << endl;
            
            getchar();
        }
        

        clearMeasurement();

        ///< get the feature points
        std::vector<int> index;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputSurf(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::removeNaNFromPointCloud(frame.LessSharp, *inputSharp, index);
        pcl::removeNaNFromPointCloud(frame.LessSurf, *inputSurf, index);

        //cout << "test4-lesssharp:" << inputSharp->points.size() << endl;
        //cout << "test4-lesssurf:" << inputSurf->points.size() << endl;

        ///< downsize the feature points
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>());
        downSizeFilterCorner.setInputCloud(inputSharp);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>());
        downSizeFilterSurf.setInputCloud(inputSurf);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        cout << "test5-lesssharp:" << laserCloudCornerStack->points.size() << endl;
        cout << "test5-lesssurf:" << laserCloudSurfStack->points.size() << endl;

        cout << "corner feature:" << laserCloudCornerStackNum << endl;
        cout << "surf feature:" << laserCloudSurfStackNum << endl;


        ///< find the corresponding points for corner features
        for (int i = 0; i < laserCloudCornerStackNum; i++)
        {
            ///< transform the point to w frame
            pointOri = laserCloudCornerStack->points[i];
            pointSel = pointOri;
            transPointXYZI(pointSel, frame.R_l_e, frame.t_l_e);

            pointSearchInd.clear();
            pointSearchSqDis.clear();
            kdtreeCornerFromPriorMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            //lsy  Ô­À´ÊÇ1
            if (pointSearchSqDis[4] < 9)
            {
                std::vector<Triple> nearCorners;
                Triple center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                    Triple tmp(CornerSubmap->points[pointSearchInd[j]].x,
                        CornerSubmap->points[pointSearchInd[j]].y,
                        CornerSubmap->points[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;

                SO3 covMat = SO3::Zero();
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }

                Eigen::SelfAdjointEigenSolver<SO3> saes(covMat);

                // if is indeed line feature
                // note Eigen library sort eigenvalues in increasing order
                Triple unit_direction = saes.eigenvectors().col(2);
                Triple curr_point(pointOri.x, pointOri.y, pointOri.z);
                Triple curr_point_w(pointSel.x, pointSel.y, pointSel.z);
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                {
                    Triple point_on_line = center;
                    Triple point_a, point_b;
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
                    else
                    {
                        continue;
                    }
                }
            }
        }

        ///< find the correspond points for surf features
        for (int i = 0; i < laserCloudSurfStackNum; i++)
        {
            pointOri = laserCloudSurfStack->points[i];
            pointSel = pointOri;
            transPointXYZI(pointSel, frame.R_l_e, frame.t_l_e);
            pointSearchInd.clear();
            pointSearchSqDis.clear();
            kdtreeSurfFromPriorMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 9.0)
            {

                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = SurfSubmap->points[pointSearchInd[j]].x;
                    matA0(j, 1) = SurfSubmap->points[pointSearchInd[j]].y;
                    matA0(j, 2) = SurfSubmap->points[pointSearchInd[j]].z;
                }
                ///< find the norm of plane
                Triple norm = matA0.colPivHouseholderQr().solve(matB0);
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                ///< Here n(pa, pb, pc) is unit norm of plane
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    ///< if OX * n larger than 0.2, then plane is not fit well
                    if (fabs(norm(0) * SurfSubmap->points[pointSearchInd[j]].x +
                        norm(1) * SurfSubmap->points[pointSearchInd[j]].y +
                        norm(2) * SurfSubmap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
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
                    else
                    {
                        continue;
                    }
                }
            }

        }

        lidarMapObs.cornerFeature = correspondCornerFeature_;
        lidarMapObs.surfFeature = correspondSurfFeature_;
        lidarMapObs.curr_R_l_e = frame.R_l_e;
        lidarMapObs.curr_t_l_e = frame.t_l_e;
        lidarMapObs.last_R_l_e = SO3::Identity();
        lidarMapObs.last_t_l_e = Triple::Zero();

        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_Corner, tmp_Surf;
        tmp_Corner.reset(new pcl::PointCloud<pcl::PointXYZI>());
        tmp_Surf.reset(new pcl::PointCloud<pcl::PointXYZI>());
        *tmp_Corner += *CornerSubmap;
        *tmp_Surf += *SurfSubmap;

        submap_surf = PointXYZI2Triple(tmp_Surf);
        submap_corner = PointXYZI2Triple(tmp_Corner);

        return true;

    }
}
