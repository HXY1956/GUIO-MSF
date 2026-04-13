#include "hwa_vis_yolo_worker.h"
#include <QMetaObject>
#include <QSize>
#include <QDebug>
#include <QDateTime>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <memory>

using namespace std;
using namespace hwa_vis;

vis_yolo_worker::vis_yolo_worker(std::shared_ptr<hwa_set::set_base> _gset, int _ID, std::shared_ptr<vis_yolo_v8ov> _yolo, QObject* parent)
    : QObject(parent), yolo(_yolo), ID(_ID){
    detect = dynamic_cast<set_vis*>(_gset.get())->detect();
    camskip = dynamic_cast<set_tracker*>(_gset.get())->camskip();
    mindist = dynamic_cast<set_tracker*>(_gset.get())->mindist();
    canny = dynamic_cast<set_tracker*>(_gset.get())->canny();
    acc = dynamic_cast<set_tracker*>(_gset.get())->acc();
    minradius = dynamic_cast<set_tracker*>(_gset.get())->minradius();
    maxradius = dynamic_cast<set_tracker*>(_gset.get())->maxradius();
    type = dynamic_cast<set_tracker*>(_gset.get())->type();
}

vis_yolo_worker::~vis_yolo_worker() { 
}

void vis_yolo_worker::DetectHalconCircle(const cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<float>& radius, cv::Rect roi) {
    cv::Mat gray;
    if (img.channels() == 3) cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else gray = img.clone();
    std::vector<cv::Vec3f> circles;

    roi &= cv::Rect(0, 0, gray.cols, gray.rows);
    if (roi.empty()) return;
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
}

void vis_yolo_worker::NewImage(double timestamp, cv::Mat img) {
    imagecoder[timestamp] = img;
}

void vis_yolo_worker::OneEpoch() {
    if (stop.load()) {
        imagecoder.clear();
        cycleTimer->stop();
        return;
    }
    if (imagecoder.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return;
    } 
    if (++framecount != camskip) {
        imagecoder.erase(imagecoder.begin());
        return;
    }
    framecount = -1;
    auto node = imagecoder.extract(imagecoder.begin());
    double timestamp = node.key();
    cv::Mat img = std::move(node.mapped());

    if (detect) {
        std::vector<Detection> dets;
        dets = yolo->detect(img);
        if (type == "BGR") cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        for (const auto& d : dets) {
            cv::rectangle(img, d.box, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, d.classname + std::to_string(ID),
                d.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 1, { 0, 255, 0 }, 2);

            std::vector<cv::Point2f> centers;
            std::vector<float> r;
            DetectHalconCircle(img, centers, r, d.box);
            for (size_t i = 0; i < centers.size(); ++i) {
                cv::Point2f center = centers[i];
                int radius = static_cast<int>(r[i]);
                cv::circle(img, center, 5, cv::Scalar(0, 0, 255), -1);
                cv::Point textOrg(center.x - 4, center.y);
                cv::putText(img, "b" + std::to_string(i),
                    textOrg,
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 0), 2);
            }
            if (track) emit NewPts(ID, timestamp, centers);
        }
    }
    emit NewImage2Show(timestamp, img);
}

void vis_yolo_worker::startWork() {
    cycleTimer = new QTimer(this);
    connect(cycleTimer, &QTimer::timeout, this, &vis_yolo_worker::OneEpoch);
    cycleTimer->start(0);
}

void vis_yolo_worker::stopWork()
{
    stop = true;
}
