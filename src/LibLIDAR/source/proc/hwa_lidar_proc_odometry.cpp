#include <iostream>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "hwa_lidar_proc_odometry.h"
using namespace std;

namespace hwa_lidar
{
    using std::sin;
    using std::cos;
    using std::asin;
    using std::atan2;
    using std::sqrt;
    using std::fabs;
    using std::pow;

    lidar_proc_odometry::lidar_proc_odometry()
    {
    }

    void lidar_proc_odometry::reset()
    {
        correspondCornerFeature_.currCornerPointCloud.clear();
        correspondCornerFeature_.correspondCornerPointCloudA.clear();
        correspondCornerFeature_.correspondCornerPointCloudB.clear();
        correspondCornerFeature_.currCornerPointCloud_inlast.clear();
        correspondSurfFeature_.currSurfPointCloud.clear();
        correspondSurfFeature_.correspondSurfPointCloudA.clear();
        correspondSurfFeature_.correspondSurfPointCloudB.clear();
        correspondSurfFeature_.correspondSurfPointCloudC.clear();
        correspondSurfFeature_.currSurfPointCloud_inlast.clear();
        correspondSurfFeature_.norm.clear();
        lidarOdoObs = lidarOdometryObs();

        last_curr_rot = SO3::Identity();
        last_curr_trans = Triple::Zero();
    }

    void lidar_proc_odometry::transformToStart(const pcl::PointXYZI pi, pcl::PointXYZI& po, bool distortion)
    {
        double s = 1.0;
        if (distortion)
            s = (pi.intensity - int(pi.intensity)) / scanPeriod_;
        else
            s = 1.0;
        Eigen::Quaterniond curr_last_q(curr_last_rot);
        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, curr_last_q);
        SO3 R_point_last = q_point_last.toRotationMatrix();
        Triple t_point_last = s * curr_last_trans;

        Triple point(pi.x, pi.y, pi.z);
        Triple un_point;
        un_point = R_point_last * point + t_point_last;

        po.x = un_point.x();
        po.y = un_point.y();
        po.z = un_point.z();
        po.intensity = pi.intensity;
    }

    // transform all lidar points to the start of the next frame
    void lidar_proc_odometry::transformToEnd(const pcl::PointXYZI pi, pcl::PointXYZI& po, bool distortion)
    {
        // undistort point first
        pcl::PointXYZI un_point_tmp;
        transformToStart(pi, un_point_tmp, distortion);

        Triple un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
        Triple point_end = curr_last_rot.transpose() * (un_point - curr_last_trans);

        po.x = point_end.x();
        po.y = point_end.y();
        po.z = point_end.z();

        //Remove distortion time info
        po.intensity = int(pi.intensity);
    }

    void lidar_proc_odometry::removeDistortion(LidarFrame& frame1, LidarFrame& frame2)
    {
        /*
         * @brief:correct the distortion of pointcloud in frame2
         * !!!: now doesn't correct the distortion of the "fullCloud"
         */
        if (frame1.empty || frame2.empty)
        {
            std::cout << "ERROR:the pose of lidar frames need to be initialized!" << endl;
            getchar();
        }

        curr_last_rot = frame1.R_l_e.transpose() * frame2.R_l_e;
        curr_last_trans = frame1.R_l_e.transpose() * (frame2.t_l_e - frame1.t_l_e);
        if (1)
        {
            int cornerPointsLessSharpNum = frame2.LessSharp.points.size();
            for (int i = 0; i < cornerPointsLessSharpNum; i++)
            {
                transformToEnd(frame2.LessSharp.points[i], frame2.LessSharp.points[i], true);
            }

            int surfPointsLessFlatNum = frame2.LessSurf.points.size();
            for (int i = 0; i < surfPointsLessFlatNum; i++)
            {
                transformToEnd(frame2.LessSurf.points[i], frame2.LessSurf.points[i], true);
            }

            int surfPointsFlatNum = frame2.Surf.points.size();
            for (int i = 0; i < surfPointsFlatNum; i++)
            {
                transformToEnd(frame2.Surf.points[i], frame2.Surf.points[i], true);
            }

            int cornerPointsSharpNum = frame2.Sharp.points.size();
            for (int i = 0; i < cornerPointsSharpNum; i++)
            {
                transformToEnd(frame2.Sharp.points[i], frame2.Sharp.points[i], true);
            }
        }
    }

    void lidar_proc_odometry::process(LidarFrame& frame1, LidarFrame& frame2)
    {
        if (frame1.empty || frame2.empty)
        {
            std::cout << "ERROR:the pose of lidar frames need to be initialized!" << endl;
            getchar();
        }
        ///< clear the history information
        reset();
        ///< set KDTree
        pcl::PointCloud<pcl::PointXYZI>::Ptr kdtreeSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr kdtreeSurf(new pcl::PointCloud<pcl::PointXYZI>());

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(frame1.LessSharp, *kdtreeSharp, indices);
        pcl::removeNaNFromPointCloud(frame1.LessSurf, *kdtreeSurf, indices);

        lastCornerKDTree_.setInputCloud(kdtreeSharp);
        lastSurfaceKDTree_.setInputCloud(kdtreeSurf);

        ///< transformation from frame2 to frame1  curr-》》last
        curr_last_rot = frame1.R_l_e.transpose() * frame2.R_l_e;
        curr_last_trans = frame1.R_l_e.transpose() * (frame2.t_l_e - frame1.t_l_e);

        size_t lastCornerCloudSize = kdtreeSharp->points.size();
        size_t lastSurfaceCloudSize = kdtreeSurf->points.size();

        if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            size_t cornerPointsSharpNum = frame2.Sharp.points.size();
            size_t surfPointsFlatNum = frame2.Surf.points.size();

            ///< position of feature which is projected to the last lidar frame
            pcl::PointXYZI pointSel;

            //project to last frame
            for (int i = 0; i < cornerPointsSharpNum; i++)
            {
                //pcl::PointXYZI pointSel;
                pointSel = pcl::PointXYZI();
                transformToStart(frame2.Sharp.points[i], pointSel);///< transform to the start time of the scan
                lastCornerKDTree_.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);///< find the closest point to pointSel in lastCornerCloud_


                int closestPointInd = -1;
                int minPointInd2 = -1;
                if (pointSearchSqDis[0] < 25)
                {

                    closestPointInd = pointSearchInd[0];
                    int closestPointScan = int(kdtreeSharp->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = 25;
                    for (int j = closestPointInd + 1; j < kdtreeSharp->points.size(); j++)
                    {
                        ///< if in the same scan line, continue
                        if (int(kdtreeSharp->points[j].intensity) <= closestPointScan)
                            continue;

                        ///< if not in nearby scans, end the loop
                        if (int(kdtreeSharp->points[j].intensity) > (closestPointScan + 2.5))
                            break;

                        pointSqDis = calcSquaredDiff(kdtreeSharp->points[j], pointSel);

                        //lsy change
                        //if (int(kdtreeSharp->points[j].intensity) > closestPointScan)
                        //{
                        if (pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        //}
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--)
                    {
                        ///< if in the same scan line, continue
                        if (int(kdtreeSharp->points[j].intensity) >= closestPointScan)
                            continue;

                        if (int(kdtreeSharp->points[j].intensity) < closestPointScan - 2.5)
                            break;

                        pointSqDis = calcSquaredDiff(kdtreeSharp->points[j], pointSel);

                        //lsy change
                        //if (int(kdtreeSharp->points[j].intensity) < closestPointScan)
                        //{
                        if (pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        //}


                    }
                }


                if (closestPointInd >= 0 && minPointInd2 >= 0) ///< both closestPointInd and minPointInd2 is valid
                {

                    Triple currCornerPoint(frame2.Sharp.points[i].x,
                        frame2.Sharp.points[i].y,
                        frame2.Sharp.points[i].z);
                    Triple correspondCornerPointA(kdtreeSharp->points[closestPointInd].x,
                        kdtreeSharp->points[closestPointInd].y,
                        kdtreeSharp->points[closestPointInd].z);
                    Triple correspondCornerPointB(kdtreeSharp->points[minPointInd2].x,
                        kdtreeSharp->points[minPointInd2].y,
                        kdtreeSharp->points[minPointInd2].z);

                    pcl::PointXYZI currCornerPoint_inlast;
                    transformToStart(frame2.Sharp.points[i], currCornerPoint_inlast);

                    Triple currCornerPoint_tolast(currCornerPoint_inlast.x,
                        currCornerPoint_inlast.y,
                        currCornerPoint_inlast.z);

                    correspondCornerFeature_.currCornerPointCloud.push_back(currCornerPoint);
                    correspondCornerFeature_.currCornerPointCloud_inlast.push_back(currCornerPoint_tolast);
                    correspondCornerFeature_.correspondCornerPointCloudA.push_back(correspondCornerPointA);
                    correspondCornerFeature_.correspondCornerPointCloudB.push_back(correspondCornerPointB);

                }

            }

            for (int i = 0; i < surfPointsFlatNum; i++)
            {
                pointSearchInd.clear();
                pointSearchSqDis.clear();
                pointSel = pcl::PointXYZI();
                transformToStart(frame2.Surf.points[i], pointSel);

                lastSurfaceKDTree_.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);


                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                if (pointSearchSqDis[0] < 25)
                {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScan = int(kdtreeSurf->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                    for (int j = closestPointInd + 1; j < kdtreeSurf->points.size(); j++)
                    {
                        if (int(kdtreeSurf->points[j].intensity) > closestPointScan + 2.5)
                        {
                            break;
                        }

                        pointSqDis = calcSquaredDiff(kdtreeSurf->points[j], pointSel);

                        if (int(kdtreeSurf->points[j].intensity) <= closestPointScan && pointSqDis < minPointSqDis2)
                        {

                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;

                        }
                        else if (int(kdtreeSurf->points[j].intensity) > closestPointScan && pointSqDis < minPointSqDis3)
                        {

                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;

                        }
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--)
                    {
                        if (int(kdtreeSurf->points[j].intensity) < closestPointScan - 2.5)
                        {
                            break;
                        }

                        pointSqDis = calcSquaredDiff(kdtreeSurf->points[j], pointSel);

                        if (int(kdtreeSurf->points[j].intensity) >= closestPointScan && pointSqDis < minPointSqDis2)
                        {

                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;

                        }
                        else if (int(kdtreeSurf->points[j].intensity) < closestPointScan && pointSqDis < minPointSqDis3)
                        {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }

                    if (minPointInd2 >= 0 && minPointInd3 >= 0)
                    {

                        Triple currSurfPoint(frame2.Surf.points[i].x,
                            frame2.Surf.points[i].y,
                            frame2.Surf.points[i].z);
                        Triple correspondSurfPointA(kdtreeSurf->points[closestPointInd].x,
                            kdtreeSurf->points[closestPointInd].y,
                            kdtreeSurf->points[closestPointInd].z);
                        Triple correspondSurfPointB(kdtreeSurf->points[minPointInd2].x,
                            kdtreeSurf->points[minPointInd2].y,
                            kdtreeSurf->points[minPointInd2].z);
                        Triple correspondSurfPointC(kdtreeSurf->points[minPointInd3].x,
                            kdtreeSurf->points[minPointInd3].y,
                            kdtreeSurf->points[minPointInd3].z);

                        pcl::PointXYZI currSurfPoint_inlast;
                        transformToStart(frame2.Surf.points[i], currSurfPoint_inlast);
                        Triple currSurfPoint_tolast(currSurfPoint_inlast.x,
                            currSurfPoint_inlast.y,
                            currSurfPoint_inlast.z);


                        correspondSurfFeature_.currSurfPointCloud.push_back(currSurfPoint);
                        correspondSurfFeature_.currSurfPointCloud_inlast.push_back(currSurfPoint_tolast);
                        correspondSurfFeature_.correspondSurfPointCloudA.push_back(correspondSurfPointA);
                        correspondSurfFeature_.correspondSurfPointCloudB.push_back(correspondSurfPointB);
                        correspondSurfFeature_.correspondSurfPointCloudC.push_back(correspondSurfPointC);

                    }
                }
            }
        }
        else
        {
            cerr << "not lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100" << endl;
        }


        ///< store the observation of lidar odometry
        lidarOdoObs.cornerFeature = correspondCornerFeature_;
        lidarOdoObs.surfFeature = correspondSurfFeature_;
        lidarOdoObs.last_R_l_e = frame1.R_l_e;
        lidarOdoObs.last_t_l_e = frame1.t_l_e;
        lidarOdoObs.curr_R_l_e = frame2.R_l_e;
        lidarOdoObs.curr_t_l_e = frame2.t_l_e;
    }

    void lidar_proc_odometry::data_association(vector<LidarFrame>& buffer)
    {
        //
        cout << "---!!!data-association!!!---" << endl;
        int eft_buf_size = buffer.size() - 2;
        assert(eft_buf_size > 0);

        associations.clear();
        for (int i = 2; i < buffer.size(); i++)
        {
            auto& oldest_lidar = buffer.at(1);
            auto& cur_lidar = buffer.at(i);

            //build kd-tree
            pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>());
            points = Triple2PointXYZI(cur_lidar.pcs);
            pcl::KdTreeFLANN<pcl::PointXYZI> KDTree_pppc;
            KDTree_pppc.setInputCloud(points);

            //calculate relative pose
            SO3 R_old_cur = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e;
            Triple t_old_cur = cur_lidar.R_l_e.transpose() * (oldest_lidar.t_l_e - cur_lidar.t_l_e);
            Triple t_cur_old = oldest_lidar.R_l_e.transpose() * (cur_lidar.t_l_e - oldest_lidar.t_l_e);

            int eft_number = 0;

            std::vector<int> pointSearchInd;//index
            std::vector<float> pointSearchSqDis;//distance
            for (int i = 0; i < oldest_lidar.pcs.size(); i++)
            {
                pcl::PointXYZI point;
                point.x = oldest_lidar.pcs.at(i).x();
                point.y = oldest_lidar.pcs.at(i).y();
                point.z = oldest_lidar.pcs.at(i).z();
                // project to cur frame
                transPointXYZI(point, R_old_cur, t_old_cur);
                KDTree_pppc.nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[0] < 1.0)
                {
                    //calculate residual vector
                    Triple res_norm1 = cur_lidar.ncs.at(pointSearchInd.at(0)) - R_old_cur * oldest_lidar.ncs.at(i);
                    Triple res_norm2 = cur_lidar.ncs.at(pointSearchInd.at(0)) - R_old_cur * (-oldest_lidar.ncs.at(i));
                    double res_distance = cur_lidar.ncs.at(pointSearchInd.at(0)).transpose() * (cur_lidar.pcs.at(pointSearchInd.at(0)) - Triple(point.x, point.y, point.z));

                    //
                    Triple& oldest_pc = oldest_lidar.pcs.at(i);
                    Triple& oldest_nc = oldest_lidar.ncs.at(i);
                    Triple oldest_phi = oldest_nc * (oldest_nc.transpose() * oldest_pc);
                    Triple oldest_n = oldest_phi / oldest_phi.norm();


                    Triple& cur_pc = cur_lidar.pcs.at(pointSearchInd.at(0));
                    Triple& cur_nc = cur_lidar.ncs.at(pointSearchInd.at(0));
                    Triple cur_phi = cur_nc * (cur_nc.transpose() * cur_pc);
                    Triple cur_n = cur_phi / cur_phi.norm();


                    Triple residual = cur_phi - R_old_cur * oldest_n * (oldest_phi.norm() - t_cur_old.transpose() * oldest_n);
                    // 
                    if ((res_norm1.norm() < 0.2 || res_norm2.norm() < 0.2) && res_distance < 0.2)
                    {

                        auto iter = associations.find(i);
                        if (iter == associations.end())
                        {
                            //new correspond
                            vector<int> corr_indexs;
                            corr_indexs.push_back(pointSearchInd.at(0));
                            associations.insert(make_pair(i, corr_indexs));

                        }
                        else
                        {
                            //old correspond
                            iter->second.push_back(pointSearchInd.at(0));

                        }

                        eft_number++;
                    }
                }

            }
            cout << "pass_number:" << eft_number << endl;
            cout << "all_number:" << oldest_lidar.pcs.size() << endl;
        }

    }


    void lidar_proc_odometry::segmenter_data_association(vector<LidarFrame>& buffer)
    {
        cout << "---!!!data-association!!!---" << endl;
        int eft_buf_size = buffer.size() - 2;
        assert(eft_buf_size > 0);
        associations.clear();

        for (int i = 2; i < buffer.size(); i++)
        {
            auto& oldest_lidar = buffer.at(1);
            auto& cur_lidar = buffer.at(i);

            //build kd-tree
            pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>());
            points = Triple2PointXYZI(cur_lidar.centroids);
            if (points->points.size() == 0) continue;
            pcl::KdTreeFLANN<pcl::PointXYZI> KDTree_center;
            KDTree_center.setInputCloud(points);

            //计算两帧之间相对位置
            SO3 R_old_cur = cur_lidar.R_l_e.transpose() * oldest_lidar.R_l_e;
            Triple t_old_cur = cur_lidar.R_l_e.transpose() * (oldest_lidar.t_l_e - cur_lidar.t_l_e);
            Triple t_cur_old = oldest_lidar.R_l_e.transpose() * (cur_lidar.t_l_e - oldest_lidar.t_l_e);


            //遍历每个点
            int eft_number = 0;
            std::vector<int> pointSearchInd;//近邻索引
            std::vector<float> pointSearchSqDis;//对应距离平方
            for (int i = 0; i < oldest_lidar.centroids.size(); i++)
            {
                //对每一个at(1) 的pcs 和ncs去寻找 at(2) 中的最邻近的 pcs
                //寻找最近的点匹配序列
                pcl::PointXYZI point;
                point.x = oldest_lidar.centroids.at(i).x();
                point.y = oldest_lidar.centroids.at(i).y();
                point.z = oldest_lidar.centroids.at(i).z();
                //投影到cur_frame下
                transPointXYZI(point, R_old_cur, t_old_cur);
                KDTree_center.nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[0] < 2.0)
                {
                    //calculate residual
                    Triple res_distance = cur_lidar.centroids.at(pointSearchInd.at(0)) - Triple(point.x, point.y, point.z);
                    Triple res_direction = cur_lidar.directions.at(pointSearchInd.at(0)) - R_old_cur * oldest_lidar.directions.at(i);
                    //cout << "res_distance:" << res_distance.transpose() << endl;
                    //cout << "cur_dir:" << cur_lidar.directions.at(pointSearchInd.at(0)).transpose() << endl;
                    //cout << "old_dir:" << oldest_lidar.directions.at(i).transpose() << endl;
                    //cout << "res_direction:" << res_direction.transpose() << endl << endl;
                    //remove outlier
                    double norm_distance = sqrt(res_distance(0) * res_distance(0) + res_distance(1) * res_distance(1));
                    double norm_direction = res_direction(2);

                    if (norm_distance < 0.5 && norm_direction < 0.2)
                    {

                        //加入map索引
                        auto iter = associations.find(i);
                        if (iter == associations.end())
                        {
                            //新点
                            vector<int> corr_indexs;
                            corr_indexs.push_back(pointSearchInd.at(0));
                            associations.insert(make_pair(i, corr_indexs));

                        }
                        else
                        {
                            //已存在索引的老点
                            iter->second.push_back(pointSearchInd.at(0));

                        }

                        eft_number++;
                    }
                }
            }

        }
        //cout << "index1__size:" << associations.size() << endl;
    }

    //void lidar_proc_odometry::ceresOptimize()
    //{
    //    cout << "before ceres optimize" << endl;
    //    cout << "relative att:" << curr_last_rot.eulerAngles(0, 1, 2).transpose()*180/M_PI << endl;
    //    cout << "relative pos:" << curr_last_trans.transpose() << endl;
    //    //这里的优化对象实际上是两帧之间的相对位姿
    //    int surf_size = lidarOdoObs.surfFeature.currSurfPointCloud.size();
    //    int corner_size = lidarOdoObs.cornerFeature.currCornerPointCloud.size();
    //    /*cout << "surf_size:" << surf_size << endl;
    //    cout << "corner_size:" << corner_size << endl;
    //    getchar();*/

    //    CorrespondCornerFeature& cornerf = lidarOdoObs.cornerFeature;
    //    CorrespondSurfFeature& surff = lidarOdoObs.surfFeature;

    //    SO3 last_R = lidarOdoObs.last_R_l_e;
    //    Triple last_t = lidarOdoObs.last_t_l_e;

    //    double para_q[4] = { 0,0,0,1 };
    //    double para_t[3] = { 0,0,0};

    //    Eigen::Map<Eigen::Quaterniond> q_curr_last(para_q);
    //    Eigen::Map<Triple> t_curr_last(para_t);

    //    q_curr_last= Eigen::Quaterniond(curr_last_rot);
    //    t_curr_last = curr_last_trans;

    //    clock_t start, stop;

    //    //build optimize problem
    //    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    //    ceres::LocalParameterization *q_parameterization =
    //        new ceres::EigenQuaternionParameterization();
    //    ceres::Problem::Options problem_options;

    //    ceres::Problem problem(problem_options);
    //    problem.AddParameterBlock(para_q, 4, q_parameterization);
    //    problem.AddParameterBlock(para_t, 3);

    //    if (corner_size <= 0 || surf_size <= 0)
    //    {
    //        cout << "There are some problem in the lidar odometry observation!" << endl;
    //        getchar();
    //        return;
    //    }

    //    for (int i = 0; i < surf_size; i++)
    //    {
    //        //最后一个参数，经过雷达畸变矫正后设为1.0
    //        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(surff.currSurfPointCloud[i], surff.correspondSurfPointCloudA[i], surff.correspondSurfPointCloudB[i], surff.correspondSurfPointCloudC[i], 1.0);
    //        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
    //    }

    //    for (int i = 0; i < corner_size; i++)
    //    {
    //        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(cornerf.currCornerPointCloud[i], cornerf.correspondCornerPointCloudA[i], cornerf.correspondCornerPointCloudB[i], 1.0);
    //        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
    //    }

    //    start = clock();
    //    ceres::Solver::Options options;
    //    options.linear_solver_type = ceres::DENSE_QR;
    //    options.max_num_iterations = 4;
    //    options.minimizer_progress_to_stdout = false;
    //    ceres::Solver::Summary summary;
    //    ceres::Solve(options, &problem, &summary);
    //    stop = clock();
    //    cout<<summary.BriefReport()<<endl;
    //    cout << "optimize cost time:" << (double)(stop - start) / CLK_TCK << endl;

    //    lidarOdoObs.curr_R_l_e= last_R * q_curr_last.toRotationMatrix();
    //    lidarOdoObs.curr_t_l_e= last_t + last_R * t_curr_last;

    //    curr_last_rot = q_curr_last;
    //    curr_last_trans = t_curr_last;

    //    cout << "after ceres optimize" << endl;
    //    cout << "relative att:" <<curr_last_rot.eulerAngles(0, 1, 2).transpose() * 180 / M_PI << endl;
    //    cout << "relative pos:" << curr_last_trans.transpose() << endl;


    //}
}