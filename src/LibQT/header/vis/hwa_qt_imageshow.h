#ifndef hwa_qt_imageshow_h
#define hwa_qt_imageshow_h
#include "hwa_qt_global.h"
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QLabel>
#include <QDateTime>
#include <QTimer>
#include <atomic>
#include <vector>
#include <string>
#include <thread>
#include <memory>
#include <mutex>              
#include <atomic>             
#include <chrono>     
#include <fstream>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace hwa_qt {
    class qt_image_shower : public QObject {
        Q_OBJECT
    public slots:
        void NewImage(double timestamp, cv::Mat img);
        void startWork();
        void stopWork();
    signals:
        void NewPixmap(QPixmap q);

    public:
        qt_image_shower(int ID_, std::shared_ptr<std::mutex> vwmutex = nullptr, std::shared_ptr<cv::VideoWriter> vw = nullptr, QObject* parent = nullptr);
        ~qt_image_shower() override;
        void SetFormat(std::string f) {
            if (f == "BGR") format = QImage::Format_BGR888;
            else format = QImage::Format_Grayscale8;
        }
        void displayMat(const cv::Mat& frame);
        void writeimg(cv::Mat img);

    protected:
        void OneEpoch();

    private:
        std::atomic<bool> stop{ false };
        bool m_running = false;
        QTimer* cycleTimer = nullptr;
        int ID;
        std::shared_ptr<cv::VideoWriter> vw_;
        std::map<double, cv::Mat> imagepresenter;
        std::shared_ptr<std::mutex> vwmutex_;
        std::mutex download_;
        QImage::Format format;
    };
}

#endif

