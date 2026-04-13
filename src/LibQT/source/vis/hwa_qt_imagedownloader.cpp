#include "hwa_qt_imagedownloader.h"
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace hwa_qt {
    qt_image_downloader::qt_image_downloader(QObject* parent) :
        QObject(parent), WritingCounter(0) {

    }

    qt_image_downloader::~qt_image_downloader() {
    }

    void qt_image_downloader::NewImage(int id, double timestamp, cv::Mat Image) {
        imagedownloader[id]->operator[](timestamp) = Image;
    }

    void qt_image_downloader::OneEpoch() {
        if (stop.load()) {
            imagedownloader.clear();
            cycleTimer->stop();
            thread()->quit();
            return;
        }

        if (d1->empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }

        for (auto iter = d1->begin(); iter != d1->end(); iter++) {
            double timestamp = iter->first;
            auto it = d2->lower_bound(timestamp);
            if (it == d2->end()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                d1->erase(d1->begin(), iter);
                break;
            }
            std::ostringstream leftPath, rightPath;
            leftPath << leftpath_ << "\\" << std::setw(5) << std::setfill('0') << WritingCounter << ".bmp";
            rightPath << rightpath_ << "\\" << std::setw(5) << std::setfill('0') << WritingCounter << ".bmp";
            if (cv::imwrite(leftPath.str(), iter->second) &&
                cv::imwrite(rightPath.str(), it->second))
            {
                WritingCounter++;
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
}



