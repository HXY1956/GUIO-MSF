#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "hwa_lidar_proc_segmenter.h"
#include <boost/random.hpp>

using namespace std;

hwa_lidar::lidar_proc_segmenter::lidar_proc_segmenter(int N_Scan): N_SCAN(N_Scan)
{
}

hwa_lidar::lidar_proc_segmenter::~lidar_proc_segmenter()
{
    try{
        if (allPushedIndX != nullptr) delete allPushedIndX;
        if (allPushedIndY != nullptr) delete allPushedIndY;
        if (queueIndX != nullptr) delete queueIndX;
        if (queueIndY != nullptr) delete queueIndY;
    }
    catch (...) {
		std::cerr << "DELETE POINTERS FAILED!" << std::endl;
    }
}

void hwa_lidar::lidar_proc_segmenter::systemInitialization(int n_scan)
{
    if (n_scan == 64)
    {
        N_SCAN = 64;
        ang_res_x = 0.2;
        ang_res_y = 0.427;
        ang_bottom = 24.9;
        groundScanInd = 50;
        //10000000
#ifdef WIN32
#pragma comment(linker, "/STACK:1073741824")
#endif
        segmentAlphaX = ang_res_x / 180.0 * M_PI;
        segmentAlphaY = ang_res_y / 180.0 * M_PI;
    }


    laserCloudIn = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    fullCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    fullInfoCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

    groundCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    //segmentedCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    //segmentedCloudPure = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    //outlierCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    outcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);;

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second = 0; neighborIterator.push_back(neighbor);
    neighbor.first = 0; neighbor.second = 1; neighborIterator.push_back(neighbor);
    neighbor.first = 0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    neighbor.first = 1; neighbor.second = 0; neighborIterator.push_back(neighbor);


    allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    

    queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];


    outcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    resetParameters();
}

void hwa_lidar::lidar_proc_segmenter::makeColoredPoint(pcl::PointXYZRGB & PtColored, pcl::PointXYZ const & Pt, uint32_t rgb) {
    PtColored.x = Pt.x;
    PtColored.y = Pt.y;
    PtColored.z = Pt.z;
    PtColored.rgb = *reinterpret_cast<float*>(&rgb);
}


void hwa_lidar::lidar_proc_segmenter::labelComponents(int row, int col) {
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY;

    vector<bool> lineCountFlag(N_SCAN, false);

    queueIndX[0] = row;
    queueIndY[0] = col;
    
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;

    int allPushedIndSize = 1;
    while (queueSize > 0) {
        // Pop point
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        
        --queueSize;
        ++queueStartInd;
        // Mark popped point
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;
        // Loop through all the neighboring grids of popped grid
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter) {
            // new index
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            // index should be within the boundary
            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;
            // at range image margin (left or right side)
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            // prevent infinite loop (caused by put already examined point back)
            if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            angle = atan2(d2*sin(alpha), (d1 - d2 * cos(alpha)));

            if (angle > segmentTheta) {

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
    
                
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag.at(thisIndX) = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                
                ++allPushedIndSize;
            }
        }
    }
    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum) {
        int lineCount = 0;
        for (size_t i = 0; i < N_SCAN; ++i)
            if (lineCountFlag.at(i) == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;
    }
    // segment is valid, mark these points
    if (feasibleSegment == true) {
        ++labelCount;
    }
    else { // segment is invalid, mark these points
        for (size_t i = 0; i < allPushedIndSize; ++i) {
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            
        }
    }

    //if (lineCountFlag != nullptr) delete lineCountFlag;
}



void hwa_lidar::lidar_proc_segmenter::copyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
    laserCloudIn = input;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
}
// 2. Start and end angle of a scan
void hwa_lidar::lidar_proc_segmenter::findStartEndAngle()
{
    // start and end orientation of this cloud
    //segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    //segMsg.endOrientation = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
    //    laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
    //if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
    //    segMsg.endOrientation -= 2 * M_PI;
    //}
    //else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
    //    segMsg.endOrientation += 2 * M_PI;
    //segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
}

//投影至图像上
// 3. Range image projection
void hwa_lidar::lidar_proc_segmenter::projectPointCloud()
{
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    pcl::PointXYZI thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i) {

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        // find the row and column index in the iamge for this point
        if (0) {
            //rowIdn = laserCloudInRing->points[i].ring;
        }
        else {
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        }
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;

        rangeMat.at<float>(rowIdn, columnIdn) = range;

        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

        index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
        fullInfoCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
    }
}
// 4. Mark ground points
void hwa_lidar::lidar_proc_segmenter::groundRemoval()
{
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
        for (size_t i = 0; i < groundScanInd; ++i) {

            lowerInd = j + (i)*Horizon_SCAN;
            upperInd = j + (i + 1)*Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1) {
                // no info to check, invalid points
                groundMat.at<int8_t>(i, j) = -1;
                continue;
            }

            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY * diffY)) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10) {
                groundMat.at<int8_t>(i, j) = 1;
                groundMat.at<int8_t>(i + 1, j) = 1;
            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX) {
                labelMat.at<int>(i, j) = -1;
            }
        }
    }

    if(1){
    //if (pubGroundCloud.getNumSubscribers() != 0) {
        for (size_t i = 0; i <= groundScanInd; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (groundMat.at<int8_t>(i, j) == 1)
                    groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
            }
        }
    }
}
// 5. Point cloud segmentation
void hwa_lidar::lidar_proc_segmenter::cloudSegmentation()
{

    // segmentation process
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (labelMat.at<int>(i, j) == 0)
                labelComponents(i, j);
        
    int sizeOfSegCloud = 0;
    // extract segmented cloud for lidar odometry
    for (size_t i = 0; i < N_SCAN; ++i) {

        //segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
                // outliers that will not be used for optimization (always continue)
                if (labelMat.at<int>(i, j) == 999999) {
                    if (i > groundScanInd && j % 5 == 0) {
                        //outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                        continue;
                    }
                    else {
                        continue;
                    }
                }
                // majority of ground points are skipped
                if (groundMat.at<int8_t>(i, j) == 1) {
                    if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
                        continue;
                }
                // mark ground points so they will not be considered as edge features later
                //segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1);
                // mark the points' column index for marking occlusion later
                //segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                // save range info
                //segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
                // save seg cloud
                //segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                // size of seg cloud
                ++sizeOfSegCloud;
            }
        }

        //segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
    }


    // extract segmented cloud for visualization
    //if (1) {
    //    //if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
    //    for (size_t i = 0; i < N_SCAN; ++i) {
    //        for (size_t j = 0; j < Horizon_SCAN; ++j) {
    //            if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999) {
    //                segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
    //                segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
    //            }
    //        }
    //    }
    //}





    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    //std::map<int, std::vector<pcl::PointXYZI>> indexs;
    for (size_t i = 0; i < N_SCAN; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999) {
                auto tmp = indexs.find(labelMat.at<int>(i, j));
                if (tmp != indexs.end())
                {
                    //
                    indexs.at(labelMat.at<int>(i, j)).push_back(fullCloud->points[j + i * Horizon_SCAN]);

                }
                else
                {
                    std::vector<pcl::PointXYZI> tmp_points;
                    tmp_points.push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    indexs.insert(std::make_pair(labelMat.at<int>(i, j), tmp_points));
                }
                //segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                //segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
            }
        }
    }
}
// 6. Publish all clouds
void hwa_lidar::lidar_proc_segmenter::publishCloud()
{
    boost::random::mt19937 randomGen;
    boost::random::uniform_int_distribution<> dist(0, 255);
    //求取中心点和点云方差/主方向
    int max_size = indexs.size();
    int count = 0;
    for (auto iter : indexs)
    {
        uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        auto one = iter.second;
        pcl::PointCloud<pcl::PointXYZI>::Ptr one_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < one.size(); i++)
        {
            one_cloud->points.push_back(one.at(i));
        }
        Eigen::Vector4d center;
        SO3 cov;
        pcl::compute3DCentroid(*one_cloud, center);
        pcl::computeCovarianceMatrix(*one_cloud, center, cov);
        Eigen::EigenSolver<Matrix> es(cov);
        //cout << "The eigenvalues of A are:" <<setprecision(12) << es.eigenvalues() << endl;

        //cout << es.eigenvalues().row(0).real() << endl;
        pcl::PointXYZI minPtX,minPtY,maxPtX, maxPtY;
        pcl::getMinMax3D(*one_cloud, minPtX, maxPtX);
        minPtY = minPtX; maxPtY = maxPtX;
        double height = maxPtY.z - minPtY.z;
        //from RuleBasedDetector
        minPtX.y = (minPtX.y + maxPtX.y) / 2;
        maxPtX.y = minPtX.y;
        Eigen::Vector4d minVecX, minVecY, maxVecX, maxVecY;
        minVecX(0) = minPtX.x; minVecX(1) = minPtX.y; minVecX(2) = 1; minVecX(3) = 0;
        maxVecX(0) = maxPtX.x; maxVecX(1) = maxPtX.y; maxVecX(2) = 1; maxVecX(3) = 0;

        minPtY.x = (minPtY.x + maxPtY.x) / 2;
        maxPtY.x = minPtY.x;

        minVecY(0) = minPtY.x; minVecY(1) = minPtY.y; minVecY(2) = 1; minVecY(3) = 0;
        maxVecY(0) = maxPtY.x; maxVecY(1) = maxPtY.y; maxVecY(2) = 1; maxVecY(3) = 0;

        double xyBound = std::max((maxVecX - minVecX).norm(), (maxVecY - minVecY).norm());
        

        double real1 = es.eigenvalues().row(0).real().value();
        double real2 = es.eigenvalues().row(1).real().value();
        double real3 = es.eigenvalues().row(2).real().value();
        //if (xyBound <= 1 && real1 > 5 * real2 && real1 > 5 * real3 && height > 1)
        //if(height>1 &&real1>5*real2 && real1>5*real3)
        if(height>1)
        //if (1)
        {
            Triple main_direction = es.eigenvectors().col(0).real();
            if (main_direction(2) < 0) main_direction = -main_direction;

            centroids.push_back(Triple(center(0),center(1),center(2)));
            directions.push_back(main_direction);

            //for publish point cloud
            for (int i = 0; i < one.size(); i++)
            {
                pcl::PointXYZ mPoint;
                mPoint.x = one.at(i).x;
                mPoint.y = one.at(i).y;
                mPoint.z = one.at(i).z;
                pcl::PointXYZRGB PtColored;
                makeColoredPoint(PtColored, mPoint, rgb);
                outcloud->points.push_back(PtColored);
            }
            count++;
        }
    }
    cout << "all_size:" << max_size << endl;
    cout << "pass_size:" << count << endl;
    assert(count > 0);


}
// 7. Reset parameters for next iteration
void hwa_lidar::lidar_proc_segmenter::resetParameters()
{
    laserCloudIn->clear();
    groundCloud->clear();
    //segmentedCloud->clear();
    //segmentedCloudPure->clear();
    //outlierCloud->clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);

    indexs.clear();
    centroids.clear();
    directions.clear();
    outcloud->clear();

}

ofstream tmp_write("lidar_time_info.txt");
void hwa_lidar::lidar_proc_segmenter::process(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
    clock_t startTime, endTime;
    // 0. Reset parameters for next iteration
    resetParameters();

    // 1. Convert ros message to pcl point cloud
    copyPointCloud(input);

    // 2. Start and end angle of a scan
    //起始角度和终止角度
    findStartEndAngle();
    // 3. Range image projection
    projectPointCloud();
    // 4. Mark ground points
    groundRemoval();
    // 5. Point cloud segmentation
    cloudSegmentation();
    // 6. Publish all clouds
    publishCloud();



}