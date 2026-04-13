#include "hwa_qt_imageshow.h"
#include "hwa_qt_imagereader.h"

namespace hwa_qt {
    qt_image_shower::qt_image_shower(int id, std::shared_ptr<std::mutex> vwmutex, std::shared_ptr<cv::VideoWriter> vw, QObject* parent) :
        ID(id), vwmutex_(vwmutex), vw_(vw), QObject(parent) {
    };
    qt_image_shower::~qt_image_shower() {
    };

    void qt_image_shower::NewImage(double timestamp, cv::Mat img) {
        imagepresenter[timestamp] = img;
    }

    void qt_image_shower::displayMat(const cv::Mat& frame) {
        if (frame.empty()) {
            return;
        }

        QImage outimg(
            frame.data,
            frame.cols,
            frame.rows,
            static_cast<int>(frame.step),
            format
        );
        QPixmap px = QPixmap::fromImage(outimg);
        emit NewPixmap(px);
    }

    void qt_image_shower::writeimg(cv::Mat img) {
        if (!vw_ || !vw_->isOpened()) return;
        std::lock_guard<std::mutex> lock(*vwmutex_);
        vw_->write(img);
    };

    void qt_image_shower::OneEpoch() {
        if (stop.load()) {
            imagepresenter.clear();
            cycleTimer->stop();
            thread()->quit();
            return;
        }
        cv::Mat img;
        double timestamp;
        if (imagepresenter.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return;
        }
        auto node = imagepresenter.extract(imagepresenter.begin());
        timestamp = node.key();
        img = std::move(node.mapped());
        displayMat(img);
        writeimg(img);
    }

    void qt_image_shower::startWork() {
        if (m_running) return;
        m_running = true;
        cycleTimer = new QTimer(this);
        connect(cycleTimer, &QTimer::timeout, this, &qt_image_shower::OneEpoch);
        cycleTimer->start(0);
    }

    void qt_image_shower::stopWork()
    {
        m_running = false;
        stop = true;
    }
}