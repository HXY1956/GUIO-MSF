#include "hwa_pick_imageshow.h"
using namespace hwa_pick;

qt_image_shower::qt_image_shower(int id, QObject* parent):
    ID(id), QObject(parent){
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
    if (download && framecount++ == 0) {
        framecount = 0;
        emit NewImage2Download(ID, timestamp, img);
    }
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