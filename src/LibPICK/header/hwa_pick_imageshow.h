#pragma once
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
#include <opencv2/opencv.hpp>

namespace hwa_pick {

    class qt_image_shower : public QObject {
        Q_OBJECT
    public slots:
        void NewImage(double timestamp, cv::Mat img);
        void startWork();
        void stopWork();
    signals:
        void NewPixmap(QPixmap q);
        void NewImage2Download(int _id, double timestamp, cv::Mat img);

    public:
        qt_image_shower(int ID_, QObject* parent = nullptr);
        ~qt_image_shower() override;
        void SetFormat(std::string f) {
            if (f == "BGR") format = QImage::Format_BGR888;
            else format = QImage::Format_Grayscale8;
        }
        void displayMat(const cv::Mat& frame);
        void SetDownload(bool flag) {
            download = flag;
        }

    protected:
        void OneEpoch();

    private:
        std::atomic<bool> stop{ false };
        std::atomic<bool> download{ false };
        bool m_running = false;
        QTimer* cycleTimer = nullptr;
        int ID;
        std::map<double, cv::Mat> imagepresenter;
        QImage::Format format;
        int framecount = 0;

    };
}

