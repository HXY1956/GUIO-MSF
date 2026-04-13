#include "hwa_pick_imagedownloader.h"
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
using namespace hwa_pick;

qt_image_downloader::qt_image_downloader(QObject* parent) :
    QObject(parent), WritingCounter(0) {
}

qt_image_downloader::~qt_image_downloader() {
}

void qt_image_downloader::NewImage(int id, double timestamp, cv::Mat Image) {
    imagedownloader[id]->operator[](timestamp) = Image;
}

double diff(double t1, double t2) {
    return std::abs(t1 - t2);
}

void qt_image_downloader::OneEpoch() {
    if (stop.load()) {
        imagedownloader.clear();
        cycleTimer->stop();
        d1->clear();
        d2->clear();
        return;
    }

    while (!d1->empty()) {
        double timestamp = d1->begin()->first;
        auto it = d2->lower_bound(timestamp);
        if (it == d2->end()) {
            return;
        }
        auto iter = it;
        if (it != d2->begin()) {
            auto pre_it = std::prev(it);
            iter = diff(pre_it->first, timestamp) < diff(it->first, timestamp) ? pre_it : it;
        }
        if (diff(iter->first, timestamp) > 1.0 / framerate) {
            d1->erase(d1->begin());
            continue;
        }
        cv::Mat imgl = d1->begin()->second;
        cv::Mat imgr = iter->second;
        double t1 = iter->first;
        d1->erase(d1->begin());
        d2->erase(d2->begin(), std::next(iter));

        std::ostringstream leftPath, rightPath;
        leftPath << leftpath_ << "\\" << std::setw(5) << std::setfill('0') << WritingCounter << ".bmp";
        rightPath << rightpath_ << "\\" << std::setw(5) << std::setfill('0') << WritingCounter << ".bmp";
        if (cv::imwrite(leftPath.str(), imgl) &&
            cv::imwrite(rightPath.str(), imgr))
        {
            WritingCounter++;
            std::cerr << "Writing " << std::setw(5) << std::setfill('0') << WritingCounter << ".bmp" << std::endl;
        }
    }

}

void qt_image_downloader::startWork() {
    if (m_running) return;
    m_running = true;
    imagedownloader[0] = std::make_shared<std::map<double, cv::Mat>>();
    imagedownloader[1] = std::make_shared<std::map<double, cv::Mat>>();
    d1 = imagedownloader[0];
    d2 = imagedownloader[1];
    leftpath_ = Dir + "\\Cam" + std::to_string(0);
    rightpath_ = Dir + "\\Cam" + std::to_string(1);
    CreateDirectoryA(leftpath_.c_str(), NULL);
    CreateDirectoryA(rightpath_.c_str(), NULL);

    cycleTimer = new QTimer(this);
    connect(cycleTimer, &QTimer::timeout, this, &qt_image_downloader::OneEpoch);
    cycleTimer->start(0);
}

void qt_image_downloader::stopWork()
{
    m_running = false;
    stop = true;
}

