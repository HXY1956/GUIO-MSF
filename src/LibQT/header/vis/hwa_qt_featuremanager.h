#ifndef hwa_qt_featuremanager_h
#define hwa_qt_featuremanager_h

#include "hwa_qt_global.h"
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QLabel>
#include <QTimer>
#include "filesystem"
#include "hwa_set_base.h"
#include "hwa_set_proc.h"
#ifndef Q_MOC_RUN
#include "hwa_vis_base.h"
#include "hwa_vis_coder_stereousb.h"
#endif

namespace hwa_qt {
    class qt_feature_manager : public QObject {
        Q_OBJECT
    public:
        qt_feature_manager(hwa_set::set_base* _gset, USBInformation _usbInfo, QObject* parent = nullptr);
        ~qt_feature_manager();
        void OneCycle();

    public slots:
        void startWork();
        void stopWork();
        void Accept_vel(double timestamp, Triple vel);
    signals:
        void NewMap(double timestamp, hwa_vis::PointCloud msg);
        void IsOpen(bool flag);
        void NewImg(double timestamp, cv::Mat img);

    private:
        std::map<double, cv::Vec3f> vel_map;
        std::atomic<bool> stop{ false };
        bool m_running = false;
        QTimer* cycleTimer = nullptr;
        std::unique_ptr<std::thread> captureThread;
        hwa_set::set_base* gset;
        hwa_vis::vis_base camobj;
        std::ofstream TimeCostDebugOutFile;
        bool TimeCostDebugStatus;
        USBInformation usbInfo;
        std::map<double, hwa_vis::MapServer> mapserver;
        std::unique_ptr<hwa_vis::vis_usb_reader<cv::cuda::GpuMat>> usbCam;
        int framecount;
        double vis_internal;
    };
}

#endif
