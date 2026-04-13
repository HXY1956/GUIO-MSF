#include "hwa_qt_imagetracker.h"

namespace hwa_qt {
    qt_img_tracker::qt_img_tracker(std::shared_ptr<hwa_set::set_tracker> _gset, QObject* parent) : QObject(parent), interval(_gset->interval()) {
        Init();
    }

    qt_img_tracker::~qt_img_tracker() {
    }

    void qt_img_tracker::Init() {
        K1_ = (cv::Mat_<double>(3, 3) << 3357.62354667207, 0, 930.847906353497,
            0, 3366.36912466459, 857.358067509797,
            0, 0, 1);
        K2_ = (cv::Mat_<double>(3, 3) << 3360.02655011249, 0, 987.599639705833,
            0, 3360.01168545753, 818.332768227918,
            0, 0, 1);
        R_ = (cv::Mat_<double>(3, 3) << 0.999864678833482, 0.0163779727370347, -0.00154467803866889,
            -0.0163208104242598, 0.999364307680455, 0.0316956098136602,
            0.00206280593211534, -0.0316661103293311, 0.999496374324738);
        T_ = (cv::Mat_<double>(3, 1) << -94.2572502222099, 1.55031052220627, -2.28540673469918);
        T_ = T_ * 1e-3;
        D1_ = (cv::Mat_<double>(1, 4) << -0.0599750942351191, 0.149162181047492, 0.00307837876539663, -0.00357717097824795);
        D2_ = (cv::Mat_<double>(1, 4) << -0.0929479665571648, 0.444828994747058, 0.00201632617650681, -0.00323471069999784);
        model_pts = {
            {static_cast<float>(0.0511 * cos(14.0 / 180.0 * 3.1415926)), static_cast<float>(0.0441 * sin(14.0 / 180.0 * 3.1415926)), 0.0f},
            {0.0f, 0.0592f, 0.0f},
            {static_cast<float>(0.0493 * cos(194.0 / 180.0 * 3.1415926)), static_cast<float>(0.0493 * sin(194.0 / 180.0 * 3.1415926)), 0.0f},
            {0.0f, -0.0437f, 0.0f}
        };
        isFirst = true;
    }

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> qt_img_tracker::cvMatToEigen(const cv::Mat& mat) {
        if (!(mat.type() == CV_32F || mat.type() == CV_64F)) {
            throw std::runtime_error("cvMatToEigen only supports CV_32F or CV_64F types.");
        }

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> eigenMat(mat.rows, mat.cols);

        for (int i = 0; i < mat.rows; ++i)
            for (int j = 0; j < mat.cols; ++j)
                eigenMat(i, j) = static_cast<T>(mat.at<T>(i, j));

        return eigenMat;
    }

    void qt_img_tracker::ComputeRectificationMaps(
        cv::Mat& map11, cv::Mat& map12,
        cv::Mat& map21, cv::Mat& map22,
        cv::Mat& Q) {
        cv::Mat R1, R2, P1, P2;
        stereoRectify(K1_, D1_, K2_, D2_, imgSize, R_, T_,
            R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, imgSize);
        Q.convertTo(Q, CV_32F);
        initUndistortRectifyMap(K1_, D1_, R1, P1, imgSize, CV_16SC2, map11, map12);
        initUndistortRectifyMap(K2_, D2_, R2, P2, imgSize, CV_16SC2, map21, map22);
    }

    bool qt_img_tracker::SVDRegister(std::vector<cv::Point3f>& observed_pts, cv::Mat& R, cv::Mat& t) {
        //hwa_vis::Hungarian::match_points_hungarian(model_pts, observed_pts);
        matcher.update(model_pts, observed_pts);
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

    cv::Point3f qt_img_tracker::reprojectTo3D(float u, float v, float d)
    {
        CV_Assert(Q.type() == CV_32F && Q.rows == 4 && Q.cols == 4);
        cv::Mat_<float> uv(4, 1);
        uv << u, v, d, 1.0f;
        cv::Mat_<float> XYZ = Q * uv;
        return cv::Point3f(XYZ(0) / XYZ(3), XYZ(1) / XYZ(3), XYZ(2) / XYZ(3));
    }

    int qt_img_tracker::CalInterval(double t) {
        return t / interval;
    }

    _OneBatch qt_img_tracker::Interpolation(int num) {
        double time = num * interval;
        double d1 = time - PreTime;
        double d2 = CurrTime - time;
        double d_all = CurrTime - PreTime;
        _OneBatch Res;
        Res.Q = PreQ.slerp(d1 / d_all, CurrQ);
        Res.T = (PreT * d2 + CurrT * d1) / d_all;
        Res.Time = time;
        return Res;
    }

    void qt_img_tracker::NewPts(int id, double timestamp, std::vector<cv::Point2f> _centers) {
        pointmap[id]->operator[](timestamp) = _centers;
    }

    void qt_img_tracker::Isend() {
        int n1 = CalInterval(PreTime);
        int n2 = CalInterval(CurrTime);
        if (n2 <= n1) return;
        for (int i = n1 + 1; i <= n2; i++) {
            _OneBatch Nav = Interpolation(i);
            emit NewPose(_OneBatch(Nav.Q, Nav.T, Nav.Time));
        }
    }

    void qt_img_tracker::Iswap() {
        PreT = CurrT;
        PreTime = CurrTime;
        PreQ = CurrQ;
    }

    void qt_img_tracker::oneCycle() {
        if (stop.load()) {
            pointmap.clear();
            cycleTimer->stop();
            thread()->quit();
            return;
        }

        Vpts pts1, pts2;
        double timestamp1, timestamp2;
        bool timeout = false;

        if (d1->empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }
        auto iter1 = d1->begin();
        timestamp1 = iter1->first;
        pts1 = iter1->second;

        while (true)
        {
            auto iter2 = d2->lower_bound(timestamp1);
            if (iter2 == d2->end()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return;
            }
            d1->erase(d1->begin());

            timestamp2 = iter2->first;
            if (timestamp2 - timestamp1 > 0.5) {
                timeout = true;
            }
            else {
                pts2 = std::move(iter2->second);
                d2->erase(d2->begin(), std::next(iter2));
            }
            break;
        }

        if (timeout) {
            return;
        }
        if (pts1.size() != 4 || pts2.size() != 4) {
            return;
        }

        std::vector<cv::Point3f> triangulated;
        cv::Mat R, t;
        for (int i = 0; i < pts1.size(); i++) {
            float d = pts1[i].x - pts2[i].x;
            cv::Point3f p3d = reprojectTo3D(pts1[i].x, pts1[i].y, d);
            triangulated.push_back(p3d);
        }
        if (!SVDRegister(triangulated, R, t)) return;

        Matrix R_dyn = cvMatToEigen<double>(R);
        SO3 R_fixed = R_dyn.cast<double>();
        CurrTime = timestamp1;
        CurrQ = Eigen::Quaterniond(R_fixed);
        CurrQ.normalize();
        CurrT = cvMatToEigen<double>(t);
        if (isFirst) {
            Iswap();
            isFirst = false;
        }
        Isend();
        Iswap();
    }

    void qt_img_tracker::startWork() {
        if (!track) return;
        if (m_running) return;
        m_running = true;
        pointmap[0] = std::make_shared<std::map<double, std::vector<cv::Point2f>>>();
        pointmap[1] = std::make_shared<std::map<double, std::vector<cv::Point2f>>>();
        d1 = pointmap[0];
        d2 = pointmap[1];
        cycleTimer = new QTimer(this);
        connect(cycleTimer, &QTimer::timeout, this, &qt_img_tracker::oneCycle);
        cycleTimer->start(0);
    }

    void qt_img_tracker::stopWork()
    {
        m_running = false;
        stop = true;
    }
}