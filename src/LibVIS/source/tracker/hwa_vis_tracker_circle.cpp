#include "hwa_vis_tracker_circle.h"
#include "hwa_vis_tracker_hungarian.h"

using namespace hwa_vis;

void vis_circle_tracker::computeRectificationMaps(
    cv::Mat& map11, cv::Mat& map12,
    cv::Mat& map21, cv::Mat& map22,
    cv::Mat& Q) {
    cv::Mat R1, R2, P1, P2;
    stereoRectify(K1_, D1_, K2_, D2_, imgSize, R_, T_,
        R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, imgSize);
    initUndistortRectifyMap(K1_, D1_, R1, P1, imgSize, CV_16SC2, map11, map12);
    initUndistortRectifyMap(K2_, D2_, R2, P2, imgSize, CV_16SC2, map21, map22);
}

cv::Point3f vis_circle_tracker::reprojectTo3D(float u, float v, float d, const cv::Mat& Q)
{
    CV_Assert(Q.type() == CV_32F && Q.rows == 4 && Q.cols == 4);
    cv::Mat_<float> uv(4, 1);
    uv << u, v, d, 1.0f;
    cv::Mat_<float> XYZ = Q * uv;
    return cv::Point3f(XYZ(0) / XYZ(3), XYZ(1) / XYZ(3), XYZ(2) / XYZ(3));
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vis_circle_tracker::cvMatToEigen(const cv::Mat& mat) {
    if (!(mat.type() == CV_32F || mat.type() == CV_64F)) {
        throw std::runtime_error("cvMatToEigen only supports CV_32F or CV_64F types.");
    }

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> eigenMat(mat.rows, mat.cols);

    for (int i = 0; i < mat.rows; ++i)
        for (int j = 0; j < mat.cols; ++j)
            eigenMat(i, j) = static_cast<T>(mat.at<T>(i, j));

    return eigenMat;
}

bool vis_circle_tracker::DetectHalconCircle(const cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<float>& radius) {
    cv::Mat gray;
    if (img.channels() == 3) cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else gray = img.clone();

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 200, 100, 30, 10, 150);
    if (circles.empty()) return false;

    std::sort(circles.begin(), circles.end(), [](cv::Vec3f a, cv::Vec3f b) {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
        });

    for (int i = 0; i < circles.size(); ++i) {
        centers.push_back(cv::Point2f(circles[i][0], circles[i][1]));
        radius.push_back(circles[i][2]);
    }

    return true;
}

bool vis_circle_tracker::DetectHalconCircle(const cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<float>& radius, cv::Rect roi) {
    cv::Mat gray;
    if (img.channels() == 3) cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else gray = img.clone();
    std::vector<cv::Vec3f> circles;

    roi &= cv::Rect(0, 0, gray.cols, gray.rows);
    if (roi.empty()) return false;
    cv::Mat roiGray = gray(roi).clone();

    cv::HoughCircles(roiGray, circles, cv::HOUGH_GRADIENT,
        1,        
        mindist,      
        canny, acc,  
        minradius, maxradius);

    std::sort(circles.begin(), circles.end(), [](cv::Vec3f a, cv::Vec3f b) {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
        });

    centers.clear(); radius.clear();
    for (int i = 0; i < circles.size(); ++i) {
        centers.push_back(cv::Point2f(circles[i][0] + roi.x, circles[i][1] + roi.y));
        radius.push_back(circles[i][2]);
    }

    return true;
}

bool vis_circle_tracker::TriangulateCircleCenters(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3f>& pts_3d) {

    std::vector<cv::Point2f> undistorted1, undistorted2;
    cv::Vec4d D1, D2;
    std::memcpy(D1.val, D1_.ptr<double>(0), 4 * sizeof(double));
    std::memcpy(D2.val, D2_.ptr<double>(0), 4 * sizeof(double));
    cv::undistortPoints(pts1, undistorted1, K1_, D1);
    cv::undistortPoints(pts2, undistorted2, K2_, D2);

    cv::Mat proj1 = cv::Mat::eye(3, 4, CV_64F);
    cv::Mat Rt; cv::hconcat(R_, T_, Rt);
    cv::Mat proj2 = Rt;

    cv::Mat pt1_h(2, pts1.size(), CV_64F), pt2_h(2, pts2.size(), CV_64F);
    for (size_t i = 0; i < undistorted1.size(); ++i) {
        pt1_h.at<double>(0, i) = undistorted1[i].x;
        pt1_h.at<double>(1, i) = undistorted1[i].y;
        pt2_h.at<double>(0, i) = undistorted2[i].x;
        pt2_h.at<double>(1, i) = undistorted2[i].y;
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(proj1, proj2, pt1_h, pt2_h, pts_4d);

    pts_3d.clear();
    for (int i = 0; i < pts_4d.cols; ++i) {
        cv::Mat col = pts_4d.col(i);
        col /= col.at<double>(3);
        pts_3d.emplace_back(col.at<double>(0), col.at<double>(1), col.at<double>(2));
    }
    return true;
}

bool vis_circle_tracker::SVDRegister(std::vector<cv::Point3f>& observed_pts, cv::Mat& R, cv::Mat& t) {
    
    hwa_vis::Hungarian::match_points_hungarian(model_pts, observed_pts);
    cv::Point3f cm(0, 0, 0), co(0, 0, 0);
    for (int i = 0; i < model_pts.size(); ++i) {
        cm += model_pts[i]; co += observed_pts[i];
    }
    cm *= (1.0f / model_pts.size());
    co *= (1.0f / observed_pts.size());

    cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < model_pts.size(); ++i) {
        cv::Mat vm = (cv::Mat_<double>(3, 1) << model_pts[i].x - cm.x, model_pts[i].y - cm.y, model_pts[i].z - cm.z);
        cv::Mat vo = (cv::Mat_<double>(1, 3) << observed_pts[i].x - co.x, observed_pts[i].y - co.y, observed_pts[i].z - co.z);
        H += vm * vo;
    }

    cv::Mat U, S, Vt;
    cv::SVD::compute(H, S, U, Vt);
    R = Vt.t() * U.t();
    if (cv::determinant(R) < 0) {
        Vt.row(2) *= -1;
        R = Vt.t() * U.t();
    }

    cv::Mat cm_mat = (cv::Mat_<double>(3, 1) << cm.x, cm.y, cm.z);
    cv::Mat co_mat = (cv::Mat_<double>(3, 1) << co.x, co.y, co.z);
    t = co_mat - R * cm_mat;
    return true;
}

void vis_circle_tracker::Ipush(double TrackerT, SO3 R, Triple t) {
    std::lock_guard<std::mutex> lock(trackermtx_);
    TrackerPVT.push(std::make_pair(TrackerT, std::make_pair(R, t)));
}

bool vis_circle_tracker::Iget(double& TrakcerT, SO3& R, Triple& t) {
    std::lock_guard<std::mutex> lock(trackermtx_);
    if (TrackerPVT.empty()) return false;
    TrakcerT = TrackerPVT.front().first;
    R = TrackerPVT.front().second.first;
    t = TrackerPVT.front().second.second;
    return true;

}

bool vis_circle_tracker::Ipop() {
    std::lock_guard<std::mutex> lock(trackermtx_);
    if (TrackerPVT.empty()) return false;
    TrackerPVT.pop();
    return true;
}

bool vis_circle_tracker::Iempty() {
    std::lock_guard<std::mutex> lock(trackermtx_);
    return TrackerPVT.empty();
}

void vis_circle_tracker::Iclear() {
    std::lock_guard<std::mutex> lock(trackermtx_);
    while (!TrackerPVT.empty()) TrackerPVT.pop();
}


// ********************************** ONE HICON CAM && ONE DAHENG CAM
vis_circle_tracker_dh::vis_circle_tracker_dh(std::shared_ptr<vis_daheng_reader> dahengCam,
    std::shared_ptr<vis_hicon_reader> hiconCam,
    const std::string& save_dir) : vis_circle_tracker(), m_dahengCam(dahengCam), m_hiconCam(hiconCam){
    m_saveDir = save_dir;
    CreateDirectoryA(m_saveDir.c_str(), NULL);
    m_running = true;
    imgSize = hiconCam->GetSize();
    Init();
    openfile();
}

bool vis_circle_tracker_dh::Init() {
    K1_ = (cv::Mat_<double>(3, 3) << 1754.28818435136,	0,	1591.39828203330, 0,	1754.94942627897,	1059.44236263434, 0,	0,	1);  //Daheng
    K2_ = (cv::Mat_<double>(3, 3) << 4685.99782086595,	0,	2085.27139560978, 0,	4683.37221512244,	1511.77007734693,  0,	 0,	 1);  //Hicon
    R_ = (cv::Mat_<double>(3, 3) << 0.996543891596545 ,- 0.00160582006987905, - 0.0830523537506138,
        - 0.000421695106083328,	0.999702450434656, - 0.0243891936763291,
        0.0830668062155481,	0.0243399247502362,	0.996246693228289);
    T_ = (cv::Mat_<double>(3, 1) << 167.268094436464,- 1.52523050167574, 42.2949216925507);
    T_ = T_ * 1e-3;
    D1_ = (cv::Mat_<double>(1, 4) << -0.134238025302032, 0.166464677154810, -0.00151333960456252, 0.00125131992652708);
    D2_ = (cv::Mat_<double>(1, 4) << -0.0254746758829069, - 0.0284830498727587, -0.000818815947557944, 0.00748830540497658);
    model_pts = {
        {static_cast<float>(0.0511 * cos(14.0 / 180.0 * 3.1415926)), static_cast<float>(0.0441 * sin(14.0 / 180.0 * 3.1415926)), 0.0f},
        {0.0f, 0.0592f, 0.0f},
        {static_cast<float>(0.0493 * cos(194.0 / 180.0 * 3.1415926)), static_cast<float>(0.0493 * sin(194.0 / 180.0 * 3.1415926)), 0.0f},
        {0.0f, -0.0437f, 0.0f}
    };
    TrackerOpenStatus = true;
    return true;
}

bool vis_circle_tracker_dh::sequential2cam() {
    int m1 = dahengImgList.size();
    int m2 = hiconImgList.size();
    if (m1 == 0 || m2 == 0) return false;

    double duration = hiconImgList.front().first.second - dahengImgList.front().first.second;
    
    while (abs(duration) > 0.02) {
        while (duration < -0.02 && hiconImgList.size()> 0) {
            hiconImgList.pop();
            if (hiconImgList.size() == 0) return false;
            duration = hiconImgList.front().first.second - dahengImgList.front().first.second;
        }
        while (duration > 0.02 && dahengImgList.size() > 0) {
            dahengImgList.pop();
            if (dahengImgList.size() == 0) return false;
            duration = hiconImgList.front().first.second - dahengImgList.front().first.second;
        }
    }
    return true;
}

void vis_circle_tracker_dh::AddImage() {
    std::lock_guard<std::mutex> lock(listmtx_);
    if (!m_dahengCam->GetImage().empty() && (dahengImgList.size() == 0 || dahengImgList.back().first.first != m_dahengCam->GetFrameCount()))
        dahengImgList.push(std::make_pair(std::make_pair(m_dahengCam->GetFrameCount(), m_dahengCam->GetImageTime()), m_dahengCam->GetImage()));

    if (!m_hiconCam->GetImage().empty() && (hiconImgList.size() == 0 || hiconImgList.back().first.first != m_hiconCam->GetFrameCount()))
        hiconImgList.push(std::make_pair(std::make_pair(m_hiconCam->GetFrameCount(), m_hiconCam->GetImageTime()), m_hiconCam->GetImage()));
};

void vis_circle_tracker_dh::TwoMonoTrackingLoop() {
    std::unique_lock<std::mutex> lock(listmtx_);
    if (!sequential2cam()) return;
    cv::Mat img1 = dahengImgList.front().second;
    cv::Mat img2 = hiconImgList.front().second;
    double time = dahengImgList.front().first.second;
    dahengImgList.pop();
    hiconImgList.pop();
    lock.unlock();
    cv::Mat map11, map12, map21, map22, Q;
    computeRectificationMaps(map11, map12, map21, map22, Q);
    cv::Mat imgL, imgR;
    cv::remap(img1, imgL, map11, map12, cv::INTER_LINEAR);
    cv::remap(img2, imgR, map21, map22, cv::INTER_LINEAR);

    std::vector<cv::Point2f> center1, center2;
    std::vector<float> r1, r2;
    if (!DetectHalconCircle(imgL, center1, r1)) return;
    if (!DetectHalconCircle(imgR, center2, r2)) return;

    for (size_t i = 0; i < center1.size(); ++i) {
        cv::Point2f center = center1[i];
        int radius = static_cast<int>(r1[i]);
        cv::circle(imgL, center, radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(imgL, center, 2, cv::Scalar(255, 0, 0), -1);
    }

    for (size_t i = 0; i < center2.size(); ++i) {
        cv::Point2f center = center2[i];
        int radius = static_cast<int>(r2[i]);
        cv::circle(imgR, center, radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(imgR, center, 2, cv::Scalar(255, 0, 0), -1);
    }

    cv::imshow("Hicon_Circle", imgL);
    cv::imshow("Daheng_Circle", imgR);
    cv::waitKey(1);

    std::vector<cv::Point3f> triangulated;
    //if (!TriangulateCircleCenters(center1, center2, triangulated)) return;
    for (int i = 0; i < center1.size(); i++) {
        float d = center1[i].x - center2[i].x;
        cv::Point3f p3d = reprojectTo3D(center1[i].x, center1[i].y, d, Q);
        triangulated.push_back(p3d);
    }
    cv::Mat R, t;
    if (triangulated.size() == model_pts.size()) {
        if (!SVDRegister(triangulated, R, t)) return;

        outfile << std::setprecision(2) << time
            << std::fixed << std::setprecision(4)
            << " R=[" << R.at<double>(0, 0) << "," << R.at<double>(0, 1) << "," << R.at<double>(0, 2) << ";"
            << R.at<double>(1, 0) << "," << R.at<double>(1, 1) << "," << R.at<double>(1, 2) << ";"
            << R.at<double>(2, 0) << "," << R.at<double>(2, 1) << "," << R.at<double>(2, 2) << "]"
            << " T=[" << t.at<double>(0) << "," << t.at<double>(1) << "," << t.at<double>(2) << "]"
            << std::endl;

        Matrix Rot = cvMatToEigen<double>(R);
        Vector Trans = cvMatToEigen<double>(t);
        Ipush(time, Rot, Trans);
        intialize = true;
    }
    else {
        outfile << std::setprecision(2) << time
            << std::fixed << std::setprecision(4) << "," << triangulated[0].x << "," << triangulated[0].y << "," << triangulated[0].z << std::endl;
    }
}

// ********************************** TWO HICON CAM
vis_circle_tracker_hh::vis_circle_tracker_hh(std::shared_ptr<vis_hicon_reader> hiconCam1,
    std::shared_ptr<vis_hicon_reader> hiconCam2,
    const std::string& save_dir) : vis_circle_tracker(), m_hiconCam1(hiconCam1), m_hiconCam2(hiconCam2) {
    m_saveDir = save_dir;
    CreateDirectoryA(m_saveDir.c_str(), NULL);
    m_running = true;
    imgSize = hiconCam1->GetSize();
    Init();
    openfile();
}

bool vis_circle_tracker_hh::Init() {
    K1_ = (cv::Mat_<double>(3, 3) << 3.466446011368806e+03, 0, 1.073347041971814e+03, 0, 3.471233346099928e+03, 7.702476058877435e+02, 0, 0, 1);  
    K2_ = (cv::Mat_<double>(3, 3) << 3472.83360258512,0,1020.50449465511, 0, 3479.90423641563,833.391831939668, 0, 0, 1);
    R_ = (cv::Mat_<double>(3, 3) << 0.999218219444837,-0.0162255337778892,-0.0360511023841537,
        0.0148915455944359,	0.999205487200969, - 0.0369680431898104,
        0.0366222855553690,	0.0364022856575989,	0.998665951056510);
    T_ = (cv::Mat_<double>(3, 1) << 162.028173255216,1.79902631792135, 26.2426369395162);
    T_ = T_ * 1e-3;
    D1_ = (cv::Mat_<double>(1, 4) << -0.033048060090956, 0.336242268640936, -0.00231281779343628, 0.00415222736270899);
    D2_ = (cv::Mat_<double>(1, 4) << -0.0242009644089849,- 0.0741697347122390, 0.00175028728989308, 0.00348415300701266);
    model_pts = {
        {static_cast<float>(0.0191 * cos(282.0 / 180.0 * 3.1415926)), static_cast<float>(0.0191 * sin(282.0 / 180.0 * 3.1415926)), 0.0f},
        {0.0452f, 0.0f, 0.0f},
        {static_cast<float>(0.0391 * cos(104.0 / 180.0 * 3.1415926)), static_cast<float>(0.0391 * sin(104.0 / 180.0 * 3.1415926)), 0.0f},
        {-0.0437f, 0.0f, 0.0f},
        {static_cast<float>(0.0542 * cos(282.0 / 180.0 * 3.1415926)), static_cast<float>(0.0542 * sin(282.0 / 180.0 * 3.1415926)), 0.0f}
    };
    TrackerOpenStatus = true;
    return true;
}

bool vis_circle_tracker_hh::sequential2cam() {
    int m1 = hiconImgList1.size();
    int m2 = hiconImgList2.size();
    if (m1 == 0 || m2 == 0) return false;

    double duration = hiconImgList1.front().first.second - hiconImgList2.front().first.second;

    while (abs(duration) > 0.02) {
        while (duration < -0.02 && hiconImgList1.size()> 0) {
            hiconImgList1.pop();
            if (hiconImgList1.size() == 0) return false;
            duration = hiconImgList1.front().first.second - hiconImgList2.front().first.second;
        }
        while (duration > 0.02 && hiconImgList2.size() > 0) {
            hiconImgList2.pop();
            if (hiconImgList2.size() == 0) return false;
            duration = hiconImgList1.front().first.second - hiconImgList2.front().first.second;
        }
    }
    return true;
}

void vis_circle_tracker_hh::AddImage() {
    std::lock_guard<std::mutex> lock(listmtx_);
    if (!m_hiconCam1->GetImage().empty() && (hiconImgList1.size() == 0 || hiconImgList1.back().first.first != m_hiconCam1->GetFrameCount()))
        hiconImgList1.push(std::make_pair(std::make_pair(m_hiconCam1->GetFrameCount(), m_hiconCam1->GetImageTime()), m_hiconCam1->GetImage()));

    if (!m_hiconCam2->GetImage().empty() && (hiconImgList2.size() == 0 || hiconImgList2.back().first.first != m_hiconCam2->GetFrameCount()))
        hiconImgList2.push(std::make_pair(std::make_pair(m_hiconCam2->GetFrameCount(), m_hiconCam2->GetImageTime()), m_hiconCam2->GetImage()));
};

void vis_circle_tracker_hh::TwoMonoTrackingLoop() {
    std::unique_lock<std::mutex> lock(listmtx_);
    if (!sequential2cam()) return;
    cv::Mat img1 = hiconImgList1.front().second;
    cv::Mat img2 = hiconImgList2.front().second;
    double time = hiconImgList1.front().first.second;
    hiconImgList1.pop();
    hiconImgList2.pop();
    lock.unlock();
    cv::Mat map11, map12, map21, map22, Q;
    computeRectificationMaps(map11, map12, map21, map22, Q);
    cv::Mat imgL, imgR;
    cv::remap(img1, imgL, map11, map12, cv::INTER_LINEAR);
    cv::remap(img2, imgR, map21, map22, cv::INTER_LINEAR);

    std::vector<cv::Point2f> center1, center2;
    std::vector<float> r1, r2;
    if (!DetectHalconCircle(imgL, center1, r1)) return;
    if (!DetectHalconCircle(imgR, center2, r2)) return;

    for (size_t i = 0; i < center1.size(); ++i) {
        cv::Point2f center = center1[i];
        int radius = static_cast<int>(r1[i]);
        cv::circle(imgL, center, radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(imgL, center, 2, cv::Scalar(255, 0, 0), -1);
    }

    for (size_t i = 0; i < center2.size(); ++i) {
        cv::Point2f center = center2[i];
        int radius = static_cast<int>(r2[i]);
        cv::circle(imgR, center, radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(imgR, center, 2, cv::Scalar(255, 0, 0), -1);
    }

    cv::imshow("Hicon_Circle1", imgL);
    cv::imshow("Hicon_Circle2", imgR);
    cv::waitKey(1);

    std::vector<cv::Point3f> triangulated;
    //if (!TriangulateCircleCenters(center1, center2, triangulated)) return;
    for (int i = 0; i < center1.size(); i++) {
        float d = center1[i].x - center2[i].x;
        cv::Point3f p3d = reprojectTo3D(center1[i].x, center1[i].y, d, Q);
        triangulated.push_back(p3d);
    }

    cv::Mat R, t;
    if (triangulated.size() == model_pts.size()) {
        if (!SVDRegister(triangulated, R, t)) return;

        outfile << std::setprecision(2) << time
            << std::fixed << std::setprecision(4)
            << " R=[" << R.at<double>(0, 0) << "," << R.at<double>(0, 1) << "," << R.at<double>(0, 2) << ";"
            << R.at<double>(1, 0) << "," << R.at<double>(1, 1) << "," << R.at<double>(1, 2) << ";"
            << R.at<double>(2, 0) << "," << R.at<double>(2, 1) << "," << R.at<double>(2, 2) << "]"
            << " T=[" << t.at<double>(0) << "," << t.at<double>(1) << "," << t.at<double>(2) << "]"
            << std::endl;

        Matrix Rot = cvMatToEigen<double>(R);
        Vector Trans = cvMatToEigen<double>(t);
        Ipush(time, Rot, Trans);
        intialize = true;
    }
    else {
        outfile << std::setprecision(2) << time
            << std::fixed << std::setprecision(4) << "," << triangulated[0].x << "," << triangulated[0].y << "," << triangulated[0].z << std::endl;
    }
}

// ********************************** TWO DAHENG CAM
vis_circle_tracker_dd::vis_circle_tracker_dd(std::shared_ptr<vis_daheng_reader> dahengCam1,
    std::shared_ptr<vis_daheng_reader> dahengCam2,
    const std::string& save_dir) : vis_circle_tracker(), m_dahengCam1(dahengCam1), m_dahengCam2(dahengCam2) {
    m_saveDir = save_dir;
    CreateDirectoryA(m_saveDir.c_str(), NULL);
    m_running = true;
    imgSize = dahengCam1->GetSize();
    Init();
    openfile();
}

bool vis_circle_tracker_dd::Init() {
    K1_ = (cv::Mat_<double>(3, 3) << 1754.28818435136, 0, 1591.39828203330, 0, 1754.94942627897, 1059.44236263434, 0, 0, 1);  //Daheng
    K2_ = (cv::Mat_<double>(3, 3) << 4685.99782086595, 0, 2085.27139560978, 0, 4683.37221512244, 1511.77007734693, 0, 0, 1);  //Daheng
    R_ = (cv::Mat_<double>(3, 3) << 0.996543891596545, -0.00160582006987905, -0.0830523537506138,
        -0.000421695106083328, 0.999702450434656, -0.0243891936763291,
        0.0830668062155481, 0.0243399247502362, 0.996246693228289);
    T_ = (cv::Mat_<double>(3, 1) << 167.268094436464, -1.52523050167574, 42.2949216925507);
    T_ = T_ * 1e-3;
    D1_ = (cv::Mat_<double>(1, 4) << -0.134238025302032, 0.166464677154810, -0.00151333960456252, 0.00125131992652708);
    D2_ = (cv::Mat_<double>(1, 4) << -0.0254746758829069, -0.0284830498727587, -0.000818815947557944, 0.00748830540497658);
    model_pts = {
        {static_cast<float>(0.0191 * cos(282.0 / 180.0 * 3.1415926)), static_cast<float>(0.0191 * sin(282.0 / 180.0 * 3.1415926)), 0.0f},
        {0.0452f, 0.0f, 0.0f},
        {static_cast<float>(0.0391 * cos(104.0 / 180.0 * 3.1415926)), static_cast<float>(0.0391 * sin(104.0 / 180.0 * 3.1415926)), 0.0f},
        {-0.0437f, 0.0f, 0.0f},
        {static_cast<float>(0.0542 * cos(282.0 / 180.0 * 3.1415926)), static_cast<float>(0.0542 * sin(282.0 / 180.0 * 3.1415926)), 0.0f}
    };
    TrackerOpenStatus = true;
    return true;
}

bool vis_circle_tracker_dd::sequential2cam() {
    int m1 = dahengImgList1.size();
    int m2 = dahengImgList2.size();
    if (m1 == 0 || m2 == 0) return false;

    double duration = dahengImgList1.front().first.second - dahengImgList2.front().first.second;

    while (abs(duration) > 0.02) {
        while (duration < -0.02 && dahengImgList1.size()> 0) {
            dahengImgList1.pop();
            if (dahengImgList1.size() == 0) return false;
            duration = dahengImgList1.front().first.second - dahengImgList2.front().first.second;
        }
        while (duration > 0.02 && dahengImgList2.size() > 0) {
            dahengImgList2.pop();
            if (dahengImgList2.size() == 0) return false;
            duration = dahengImgList1.front().first.second - dahengImgList2.front().first.second;
        }
    }
    return true;
}

void vis_circle_tracker_dd::AddImage() {
    std::lock_guard<std::mutex> lock(listmtx_);
    if (!m_dahengCam1->GetImage().empty() && (dahengImgList1.size() == 0 || dahengImgList1.back().first.first != m_dahengCam1->GetFrameCount()))
        dahengImgList1.push(std::make_pair(std::make_pair(m_dahengCam1->GetFrameCount(), m_dahengCam1->GetImageTime()), m_dahengCam1->GetImage()));

    if (!m_dahengCam2->GetImage().empty() && (dahengImgList2.size() == 0 || dahengImgList2.back().first.first != m_dahengCam2->GetFrameCount()))
        dahengImgList2.push(std::make_pair(std::make_pair(m_dahengCam2->GetFrameCount(), m_dahengCam2->GetImageTime()), m_dahengCam2->GetImage()));
};

void vis_circle_tracker_dd::TwoMonoTrackingLoop() {
    std::unique_lock<std::mutex> lock(listmtx_);
    if (!sequential2cam()) return;
    cv::Mat img1 = dahengImgList1.front().second;
    cv::Mat img2 = dahengImgList2.front().second;
    double time = dahengImgList1.front().first.second;
    dahengImgList1.pop();
    dahengImgList2.pop();
    lock.unlock();

    cv::Mat map11, map12, map21, map22, Q;
    computeRectificationMaps(map11, map12, map21, map22, Q);
    cv::Mat imgL, imgR;
    cv::remap(img1, imgL, map11, map12, cv::INTER_LINEAR);
    cv::remap(img2, imgR, map21, map22, cv::INTER_LINEAR);

    std::vector<cv::Point2f> center1, center2;
    std::vector<float> r1, r2;
    if (!DetectHalconCircle(imgL, center1, r1)) return;
    if (!DetectHalconCircle(imgR, center2, r2)) return;

    for (size_t i = 0; i < center1.size(); ++i) {
        cv::Point2f center = center1[i];
        int radius = static_cast<int>(r1[i]);
        cv::circle(imgL, center, radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(imgL, center, 2, cv::Scalar(255, 0, 0), -1);
    }

    for (size_t i = 0; i < center2.size(); ++i) {
        cv::Point2f center = center2[i];
        int radius = static_cast<int>(r2[i]);
        cv::circle(imgR, center, radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(imgR, center, 2, cv::Scalar(255, 0, 0), -1);
    }

    cv::imshow("Daheng_Circle1", imgL);
    cv::imshow("Daheng_Circle2", imgR);
    cv::waitKey(1);

    std::vector<cv::Point3f> triangulated;
    //if (!TriangulateCircleCenters(center1, center2, triangulated)) return;
    for (int i = 0; i < center1.size(); i++) {
        float d = center1[i].x - center2[i].x;
        cv::Point3f p3d = reprojectTo3D(center1[i].x, center1[i].y, d, Q);
        triangulated.push_back(p3d);
    }

    cv::Mat R, t;
    if (triangulated.size() == model_pts.size()) {
        if (!SVDRegister(triangulated, R, t)) return;

        outfile << std::setprecision(2) << time
            << std::fixed << std::setprecision(4)
            << " R=[" << R.at<double>(0, 0) << "," << R.at<double>(0, 1) << "," << R.at<double>(0, 2) << ";"
            << R.at<double>(1, 0) << "," << R.at<double>(1, 1) << "," << R.at<double>(1, 2) << ";"
            << R.at<double>(2, 0) << "," << R.at<double>(2, 1) << "," << R.at<double>(2, 2) << "]"
            << " T=[" << t.at<double>(0) << "," << t.at<double>(1) << "," << t.at<double>(2) << "]"
            << std::endl;

        Matrix Rot = cvMatToEigen<double>(R);
        Vector Trans = cvMatToEigen<double>(t);
        Ipush(time, Rot, Trans);
        intialize = true;
    }
    else {
        outfile << std::setprecision(2) << time
            << std::fixed << std::setprecision(4) << "," << triangulated[0].x << "," << triangulated[0].y << "," << triangulated[0].z << std::endl;
    }
}