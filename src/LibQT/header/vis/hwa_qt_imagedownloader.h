#ifndef hwa_qt_imagedownloader_h
#define hwa_qt_imagedownloader_h
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

namespace hwa_qt {
    class qt_image_downloader : public QObject {
        Q_OBJECT
    public slots:
        void startWork();
        void stopWork();
        void NewImage(int id, double timestamp, cv::Mat Image);

    public:
        qt_image_downloader(QObject* parent = nullptr);
        ~qt_image_downloader() override;
        std::string GetDir() {
            return Dir;
        }
        void SetDir(std::string str) {
            Dir = str;
        }

    protected:
        void OneEpoch();

    private:
        bool m_running = false;
        std::atomic<bool> stop{ false };
        QTimer* cycleTimer = nullptr;
        std::shared_ptr<std::map<double, cv::Mat>> d1, d2;
        std::string leftpath_, rightpath_;
        std::map<int, std::shared_ptr<std::map<double, cv::Mat>>> imagedownloader;
        std::string Dir;
        int WritingCounter;
    };
}


#endif