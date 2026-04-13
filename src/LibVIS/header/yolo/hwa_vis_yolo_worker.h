#ifndef hwa_vis_yolo_worker_h
#define hwa_vis_yolo_worker_h
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QTimer>
#include <atomic>
#include <vector>
#include <string>
#include <thread>
#include <memory>
#include <condition_variable> 
#include <mutex>              
#include <atomic>             
#include <chrono>     
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include "hwa_set_tracker.h"
#include "hwa_set_proc.h"
#include "hwa_vis_yolo_v8model.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_vis {
    class vis_yolo_worker : public QObject {
        Q_OBJECT
    public slots:
        void startWork();
        void stopWork();
        void NewImage(double timestamp, cv::Mat img);
    signals:
        void NewPts(int id, double timestamp, std::vector<cv::Point2f> pts);
        void NewImage2Show(double timestamp, cv::Mat img);

    public:
        vis_yolo_worker(std::shared_ptr<hwa_set::set_base> _gset, int ID, std::shared_ptr<vis_yolo_v8ov> _yolo, QObject* parent = nullptr);
        ~vis_yolo_worker() override;
        void SetTrack(bool flag) {
            track = flag;
        }

    protected:
        void DetectHalconCircle(const cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<float>& radius, cv::Rect roi);
        void OneEpoch();

    private:
        std::atomic<bool> stop{ false };
        QTimer* cycleTimer = nullptr;
        std::atomic<bool> track{ false };
        int ID;
        bool detect;
        int camskip;
        int framecount = -1;
        std::map<double, cv::Mat> imagecoder;
        std::shared_ptr<vis_yolo_v8ov> yolo;
        double mindist;
        double canny;
        double acc;
        int minradius;
        int maxradius;
        std::string type;
    };
}


#endif