#include "hwa_qt_imagereader.h"

namespace hwa_qt {
    qt_image_reader::qt_image_reader(int _ID, QObject* parent)
        : ID(_ID), QObject(parent) {
    }

    qt_image_reader::~qt_image_reader() {
    }

    bool qt_image_reader::Init() {
        std::string camDir = "";
        void* handle = nullptr;
        hwa_vis::CameraContext ctx = { handle, camDir, MV_Image_Bmp, ID, pDeviceInfo };
        HC = std::make_shared<hwa_vis::vis_hicon_reader>(ctx);
        if (!HC->OpenCam()) {
            delete pDeviceInfo;
            std::cerr << " OpenCamera Failed" << std::endl;
            return false;
        }
        return true;
    }

    void qt_image_reader::OneEpoch() {
        if (stop.load()) {
            HC->CloseCam();
            cycleTimer->stop();
            return;
        }
        int ret = MV_CC_GetImageBuffer(HC->context.handle, &HC->stOutFrame, 10000);
        if (ret == MV_OK) {
            double img_start = std::chrono::duration<double>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            img_start -= ProjectStartTime;
            HC->ConvertToCVMat(HC->image);
            cv::Mat img;
            cv::remap(HC->image.clone(), img, map1, map2, cv::INTER_LINEAR);
            emit NewImage(img_start, img);
            if (download && framecount++ == 5) {
                framecount = 0;
                emit NewImage2Download(ID, img_start, img);
            }
            MV_CC_FreeImageBuffer(HC->context.handle, &HC->stOutFrame);
        }
        else {
            HC->ShowErrorMsg(ret);
        }
    }

    void qt_image_reader::startWork() {
        cycleTimer = new QTimer(this);
        connect(cycleTimer, &QTimer::timeout, this, &qt_image_reader::OneEpoch);
        cycleTimer->start(0);
    }

    void qt_image_reader::stopWork()
    {
        m_running = false;
        stop = true;
    }
}

