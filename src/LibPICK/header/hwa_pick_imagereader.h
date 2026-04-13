#pragma once
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QTimer>
#include <map>
#include "hwa_base_glob.h"
#include "hwa_vis_coder_Hicon.h"

namespace hwa_pick {
    class qt_image_reader : public QObject {
        Q_OBJECT
    public slots:
        void startWork();
        void stopWork();
    signals:
        void NewImage(double timestamp, cv::Mat img);

    public:
        qt_image_reader(int ID, QObject* parent = nullptr);
        ~qt_image_reader() override;
        void _stop() { stop = true; }
        void SetDevideInfo(MV_CC_DEVICE_INFO* _pDeviceInfo) {
            pDeviceInfo = _pDeviceInfo;
        }
        bool Init();
        std::shared_ptr<hwa_vis::vis_hicon_reader> HC;

    protected:
        void OneEpoch();

    private:
        bool m_running = false;
        std::atomic<bool> stop{ false };
        QTimer* cycleTimer = nullptr;
        int ID;
        MV_CC_DEVICE_INFO* pDeviceInfo;
    };
}

