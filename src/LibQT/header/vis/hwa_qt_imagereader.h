#ifndef hwa_qt_imagereader_h
#define hwa_qt_imagereader_h
#include "hwa_qt_global.h"
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QTimer>
#include <map>
#include "hwa_base_glob.h"
#include "hwa_vis_coder_hicon.h"

namespace hwa_qt {
    class qt_image_reader : public QObject {
        Q_OBJECT
    public slots:
        void startWork();
        void stopWork();
    signals:
        void NewImage(double timestamp, cv::Mat img);
        void NewImage2Download(int _id, double timestamp, cv::Mat img);

    public:
        qt_image_reader(int ID, QObject* parent = nullptr);
        ~qt_image_reader() override;
        void _stop() { stop = true; }
        void SetDevideInfo(MV_CC_DEVICE_INFO* _pDeviceInfo) {
            pDeviceInfo = _pDeviceInfo;
        }
        void SetMap(const cv::Mat& _map1, const cv::Mat& _map2) {
            map1 = _map1;
            map2 = _map2;
        }
        void SetDownload(bool flag) {
            download = flag;
        }
        bool Init();
        std::shared_ptr<hwa_vis::vis_hicon_reader> HC;

    protected:
        void OneEpoch();

    private:
        bool m_running = false;
        std::atomic<bool> stop{ false };
        std::atomic<bool> download{ false };
        QTimer* cycleTimer = nullptr;
        int ID;
        cv::Mat map1;
        cv::Mat map2;
        MV_CC_DEVICE_INFO* pDeviceInfo;
        ImgMap imagecoder;
        int framecount = 0;
    };
}

#endif
