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
            int cornerPointsLessSharpNum = frame2.LessSharp->points.size();
            for (int i = 0; i < cornerPointsLessSharpNum; i++)
            {
                transformToEnd(frame2.LessSharp->points[i], frame2.LessSharp->points[i], true);
            }

            int surfPointsLessFlatNum = frame2.LessSurf->points.size();
            for (int i = 0; i < surfPointsLessFlatNum; i++)
            {
                transformToEnd(frame2.LessSurf->points[i], frame2.LessSurf->points[i], true);
            }

            int surfPointsFlatNum = frame2.Surf->points.size();
            for (int i = 0; i < surfPointsFlatNum; i++)
            {
                transformToEnd(frame2.Surf->points[i], frame2.Surf->points[i], true);
            }

            int cornerPointsSharpNum = frame2.Sharp->points.size();
            for (int i = 0; i < cornerPointsSharpNum; i++)
            {
                transformToEnd(frame2.Sharp->points[i], frame2.Sharp->points[i], true);
            }
        }
    }

    int lidar_proc_odometry::max3(double a1, double a2, double a3, double& max)
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

    void lidar_proc_odometry::process(LidarFrame& frame1, LidarFrame& frame2)
    {
        if (frame1.empty || frame2.empty)
        {
            std::cerr << "ERROR: The pose of lidar frames needs to be initialized!" << std::endl;
            getchar();
            return;
        }

        reset();

        pcl::PointCloud<pcl::PointXYZI>::Ptr kdtreeSharp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr kdtreeSurf(new pcl::PointCloud<pcl::PointXYZI>());

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*frame1.LessSharp, *kdtreeSharp, indices);
        pcl::removeNaNFromPointCloud(*frame1.LessSurf, *kdtreeSurf, indices);

        lastCornerKDTree_.setInputCloud(kdtreeSharp);
        //lastSurfaceKDTree_.setInputCloud(kdtreeSurf);

        curr_last_rot = frame1.R_l_e.transpose() * frame2.R_l_e;
        curr_last_trans = frame1.R_l_e.transpose() * (frame2.t_l_e - frame1.t_l_e);

        size_t lastCornerCloudSize = kdtreeSharp->points.size();
        size_t lastSurfaceCloudSize = kdtreeSurf->points.size();

        if (lastCornerCloudSize < 10 || lastSurfaceCloudSize < 100)
        {
            std::cerr << "Not enough points for association: corners=" << lastCornerCloudSize
                << " surfaces=" << lastSurfaceCloudSize << std::endl;
            return;
        }

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // ==================== Corner Points ====================
        for (int i = 0; i < frame2.Sharp->points.size(); i++)
        {
            pcl::PointXYZI pointSel;
            transformToStart(frame2.Sharp->points[i], pointSel);

            lastCornerKDTree_.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] > 5.0f) continue;

            int closestPointInd = pointSearchInd[0];
            int minPointInd2 = -1;
            float minPointSqDis2 = 5.0f;
            int closestPointScan = int(kdtreeSharp->points[closestPointInd].intensity);

            for (int j = closestPointInd + 1; j < kdtreeSharp->points.size(); j++)
            {
                if (int(kdtreeSharp->points[j].intensity) > closestPointScan + 2.5) break;
                if (int(kdtreeSharp->points[j].intensity) <= closestPointScan) continue;

                float sqDis = calcSquaredDiff(kdtreeSharp->points[j], pointSel);
                if (sqDis < minPointSqDis2)
                {
                    minPointSqDis2 = sqDis;
                    minPointInd2 = j;
                }
            }
            for (int j = closestPointInd - 1; j >= 0; j--)
            {
                if (int(kdtreeSharp->points[j].intensity) < closestPointScan - 2.5) break;
                if (int(kdtreeSharp->points[j].intensity) >= closestPointScan) continue;

                float sqDis = calcSquaredDiff(kdtreeSharp->points[j], pointSel);
                if (sqDis < minPointSqDis2)
                {
                    minPointSqDis2 = sqDis;
                    minPointInd2 = j;
                }
            }

            if (closestPointInd >= 0 && minPointInd2 >= 0)
            {
                correspondCornerFeature_.currCornerPointCloud.push_back(
                    Triple(frame2.Sharp->points[i].x, frame2.Sharp->points[i].y, frame2.Sharp->points[i].z));
                pcl::PointXYZI curr_in_last;
                transformToStart(frame2.Sharp->points[i], curr_in_last);
                correspondCornerFeature_.currCornerPointCloud_inlast.push_back(
                    Triple(curr_in_last.x, curr_in_last.y, curr_in_last.z));

                correspondCornerFeature_.correspondCornerPointCloudA.push_back(
                    Triple(kdtreeSharp->points[closestPointInd].x, kdtreeSharp->points[closestPointInd].y,
                        kdtreeSharp->points[closestPointInd].z));
                correspondCornerFeature_.correspondCornerPointCloudB.push_back(
                    Triple(kdtreeSharp->points[minPointInd2].x, kdtreeSharp->points[minPointInd2].y,
                        kdtreeSharp->points[minPointInd2].z));
            }
        }

        // ==================== Surface Points ====================
    //    for (int i = 0; i < frame2.Surf.points.size(); i++)
    //    {
    //        pcl::PointXYZI pointSel;
    //        transformToStart(frame2.Surf.points[i], pointSel);

    //        lastSurfaceKDTree_.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
    //        if (pointSearchSqDis[0] > 5.0f) continue;

    //        int closestPointInd = pointSearchInd[0];
    //        int closestScan = int(kdtreeSurf->points[closestPointInd].intensity);

    //        // Find two more points to form a plane
    //        int minInd2 = -1, minInd3 = -1;
    //        float minDis2 = 5.0f, minDis3 = 5.0f;

    //        auto searchNearby = [&](int start, int end, int step)
    //            {
    //                for (int j = start; j != end; j += step)
    //                {
    //                    int scanIdx = int(kdtreeSurf->points[j].intensity);
    //                    if (step > 0 && scanIdx > closestScan + 2.5) break;
    //                    if (step < 0 && scanIdx < closestScan - 2.5) break;

    //                    float sqDis = calcSquaredDiff(kdtreeSurf->points[j], pointSel);
    //                    if (scanIdx <= closestScan && sqDis < minDis2) { minDis2 = sqDis; minInd2 = j; }
    //                    else if (scanIdx > closestScan && sqDis < minDis3) { minDis3 = sqDis; minInd3 = j; }
    //                }
    //            };
    //        searchNearby(closestPointInd + 1, kdtreeSurf->points.size(), 1);
    //        searchNearby(closestPointInd - 1, -1, -1);

    //        if (minInd2 >= 0 && minInd3 >= 0)
    //        {
    //            std::vector<Triple> planePts = {
    //                Triple(kdtreeSurf->points[closestPointInd].x,
    //                       kdtreeSurf->points[closestPointInd].y,
    //                       kdtreeSurf->points[closestPointInd].z),
    //                Triple(kdtreeSurf->points[minInd2].x,
    //                       kdtreeSurf->points[minInd2].y,
    //                       kdtreeSurf->points[minInd2].z),
    //                Triple(kdtreeSurf->points[minInd3].x,
    //                       kdtreeSurf->points[minInd3].y,
    //                       kdtreeSurf->points[minInd3].z)
    //            };

    //            // Compute plane normal using SVD
    //            Triple center(0, 0, 0);
    //            for (auto& pt : planePts) center += pt;
    //            center /= 3.0;

    //            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    //            for (auto& pt : planePts)
    //            {
    //                Eigen::Vector3d demean = pt - center;
    //                cov += demean * demean.transpose();
    //            }
    //            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    //            Triple normal(es.eigenvectors().col(0).x(),
    //                es.eigenvectors().col(0).y(),
    //                es.eigenvectors().col(0).z());
    //            normal.normalize();

    //            // Ensure consistent direction
    //            Triple p_last(pointSel.x, pointSel.y, pointSel.z);
				//Triple p_curr(frame2.Surf.points[i].x, frame2.Surf.points[i].y, frame2.Surf.points[i].z);
    //            double dist_plane = normal.dot(p_last - center);
    //            if (dist_plane < 0)
    //            {
    //                normal = - normal;
    //            }

    //            // Adaptive threshold
    //            double range = p_last.norm();
    //            double threshold = std::max(0.05, 0.02 * range);
    //            if (fabs(dist_plane) > threshold) continue;

    //            correspondSurfFeature_.currSurfPointCloud.push_back(p_curr);
    //            correspondSurfFeature_.currSurfPointCloud_inlast.push_back(p_last);
    //            correspondSurfFeature_.correspondSurfPointCloudA.push_back(planePts[0]);
    //            correspondSurfFeature_.correspondSurfPointCloudB.push_back(planePts[1]);
    //            correspondSurfFeature_.correspondSurfPointCloudC.push_back(planePts[2]);
    //            correspondSurfFeature_.norm.push_back(normal);
    //        }
    //    }

        // Store observation
        lidarOdoObs.cornerFeature = correspondCornerFeature_;
        //lidarOdoObs.surfFeature = correspondSurfFeature_;
        lidarOdoObs.last_R_l_e = frame1.R_l_e;
        lidarOdoObs.last_t_l_e = frame1.t_l_e;
        lidarOdoObs.curr_R_l_e = frame2.R_l_e;
        lidarOdoObs.curr_t_l_e = frame2.t_l_e;
    }

    void lidar_proc_odometry::data_association(vector<LidarFrame>& buffer)
    {
        cout << "---!!!data-association (improved)!!!---" << endl;

        int buf_size = buffer.size();
        assert(buf_size > 2);

        associations.clear();
        auto& oldest = buffer.at(1); //the first frame is used for reference

        for (int j = 2; j < buf_size; j++)
        {
            auto& cur = buffer.at(j);
            if (cur.pcs.empty()) continue;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            cloud = Triple2PointXYZI(cur.pcs);

            pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
            kdtree.setInputCloud(cloud);

            SO3 R = cur.R_l_e.transpose() * oldest.R_l_e;
            Triple t = cur.R_l_e.transpose() * (oldest.t_l_e - cur.t_l_e);

            int match_cnt = 0;

            const int K = std::min(3, static_cast<int>(cur.pcs.size()));
            std::vector<int> idx(K);
            std::vector<float> dist(K);

            // ===== 遍历 oldest 平面 =====
            for (int i = 0; i < oldest.pcs.size(); i++)
            {
                pcl::PointXYZI pt;
                pt.x = oldest.pcs[i].x();
                pt.y = oldest.pcs[i].y();
                pt.z = oldest.pcs[i].z();

                // ===== 投影到当前帧 =====
                transPointXYZI(pt, R, t);

                if (kdtree.nearestKSearch(pt, K, idx, dist) < K)
                    continue;

                int best_id = -1;
                double best_res = 1e9;

                Triple p_old = oldest.pcs[i];
                Triple n_old = oldest.ncs[i];

                // ===== 遍历候选 =====
                for (int k = 0; k < K; k++)
                {
                    int id = idx[k];

                    Triple p_cur = cur.pcs[id];
                    Triple n_cur = cur.ncs[id];

                    // ===== 法向量变换到当前帧 =====
                    Triple n_old_cur = R * n_old;

                    // ===== 方向统一（关键）=====
                    if (n_old_cur.dot(n_cur) < 0)
                        n_old_cur = -n_old_cur;

                    // ===== 1. 法向量约束（更严格）=====
                    if (n_old_cur.dot(n_cur) < 0.97) continue;

                    // ===== 2. 点到平面距离 =====
                    Triple p_old_trans = R * p_old + t;

                    double dist_plane = fabs(
                        n_cur.dot(p_old_trans - p_cur)
                    );

                    // ===== 3. 自适应阈值 =====
                    double range = p_old.norm();
                    double threshold = std::max(0.05, 0.02 * range);

                    if (dist_plane > threshold) continue;

                    // ===== 选最优 =====
                    if (dist_plane < best_res)
                    {
                        best_res = dist_plane;
                        best_id = id;
                    }
                }

                // ===== 保存关联 =====
                if (best_id != -1)
                {
                    auto iter = associations.find(i);
                    if (iter == associations.end())
                    {
                        std::map<int, int> corr;
                        corr[j] = best_id;
                        associations.insert(make_pair(i, corr));
                    }
                    else
                    {
                        iter->second[j] = best_id;
                    }

                    match_cnt++;
                }
            }

            cout << "frame " << j
                << " match_cnt: " << match_cnt
                << " / " << oldest.pcs.size() << endl;
        }
    }


    void lidar_proc_odometry::segmenter_data_association(vector<LidarFrame>& buffer)
    {
        cout << "---!!!data-association!!!---" << endl;
        int eft_buf_size = buffer.size() - 1;
        assert(eft_buf_size > 0);
        associations.clear();

        for (int j = 1; j < buffer.size(); j++)
        {
            auto& oldest_lidar = buffer.at(0);
            auto& cur_lidar = buffer.at(j);

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
                            std::map<int,int> corr_indexs;
                            corr_indexs[j] = pointSearchInd.at(0);
                            associations.insert(make_pair(i, corr_indexs));

                        }
                        else
                        {
                            iter->second.insert(make_pair(j, pointSearchInd.at(0)));
                        }

                        eft_number++;
                    }
                }
            }

        }
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