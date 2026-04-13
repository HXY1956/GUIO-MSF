#ifndef hwa_qt_imagetracker_h
#define hwa_qt_imagetracker_h
#include "hwa_qt_global.h"
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QLabel>
#include <QTimer>
#include <atomic>
#include <vector>
#include <string>
#include <thread>
#include <memory>
#include <mutex>                        
#include <chrono>     
#include <fstream>
#include <iostream>
#include <sstream>
#include "hwa_set_tracker.h"
#include "hwa_base_glob.h"
#include "hwa_vis_tracker_hungarian.h"

namespace hwa_qt {
    class qt_img_tracker : public QObject {
        Q_OBJECT
    public:
        qt_img_tracker(std::shared_ptr<hwa_set::set_tracker> _gset, QObject* parent = nullptr);
        ~qt_img_tracker() override;
        void Init();
        template<typename T>
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cvMatToEigen(const cv::Mat& mat);
        void ComputeRectificationMaps(
            cv::Mat& map11, cv::Mat& map12,
            cv::Mat& map21, cv::Mat& map22,
            cv::Mat& Q);
        bool SVDRegister(std::vector<cv::Point3f>& observed_pts, cv::Mat& R, cv::Mat& t);
        cv::Point3f reprojectTo3D(float u, float v, float d);
        int CalInterval(double t);
        _OneBatch Interpolation(int num);
        void Iswap();
        void SetSize(const cv::Size& c) {
            imgSize = c;
        }
        void SetQ(const cv::Mat& _Q) {
            Q = _Q;
        }
        void SetTrack(bool _track) {
            track = _track;
        }

    public slots:
        void startWork();
        void stopWork();
        void NewPts(int id, double timestamp, std::vector<cv::Point2f> _centers);

    signals:
        void NewPose(_OneBatch Res);

    protected:
        void Isend();
        void oneCycle();

    private:
        std::atomic<bool> stop{ false };
        bool m_running = false;
        QTimer* cycleTimer = nullptr;
        Mpts d1, d2;
        MMpts pointmap;
        std::atomic<bool> track{ false };
        bool isFirst;
        Eigen::Quaterniond PreQ;
        Triple PreT;
        Eigen::Quaterniond CurrQ;
        Triple CurrT;
        double PreTime;
        double CurrTime;
        double interval;
        cv::Mat K1_, K2_;
        cv::Mat R_, T_;
        cv::Mat D1_;
        cv::Mat D2_;
        cv::Size imgSize;
        std::vector<cv::Point3f> model_pts;
        cv::Mat Q;
        hwa_vis::Point4Matcher matcher;
    };
}

#endif