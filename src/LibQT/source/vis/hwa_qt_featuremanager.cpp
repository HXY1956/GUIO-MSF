#include "hwa_qt_featuremanager.h"

namespace hwa_qt {
    qt_feature_manager::qt_feature_manager(hwa_set::set_base* _gset, USBInformation _usbInfo, QObject* parent) : QObject(parent),
        gset(_gset), usbInfo(_usbInfo), camobj(_gset, usbInfo.ID), framecount(0)
    {
        camobj.imgproc = std::make_unique<hwa_vis::vis_stereo_lk_gpu>(gset, usbInfo.ID);
        vis_internal = dynamic_cast<hwa_set::set_proc*>(gset)->visinterval();
    }
    qt_feature_manager::~qt_feature_manager() {

    }

    void qt_feature_manager::Accept_vel(double timestamp, Triple vel) {
        cv::Vec3f res;
        res[0] = vel[0];
        res[1] = vel[1];
        res[2] = vel[2];
        vel_map[timestamp] = res;
    }

    void qt_feature_manager::OneCycle() {
        if (stop.load()) {
            cycleTimer->stop();
            usbCam->Stop();
            if (captureThread && captureThread->joinable())
                captureThread->join();
            return;
        }
        auto tmp = dynamic_cast<hwa_vis::vis_stereo_lk_gpu*>(camobj.imgproc.get());
        if (!usbCam->Iget(tmp->imageGroup.first, tmp->imageGroup.second.first, tmp->imageGroup.second.second)) {
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000.0 / usbInfo.framerate));
            return;
        }
        if (framecount++ != usbInfo.camskip) {
            usbCam->Ipop();
            return;
        }
        framecount = 0;
        double timestamp = tmp->imageGroup.first;
        timestamp = std::round(timestamp / vis_internal) * vis_internal;
        auto iter = vel_map.lower_bound(timestamp);
        if (iter != vel_map.end()) camobj.imgproc->mean_ang_vel = iter->second;
        else camobj.imgproc->mean_ang_vel = cv::Vec3f::zeros();
        camobj._pointcloud = camobj.imgproc->ProcessBatch();
        emit NewMap(timestamp, camobj._pointcloud);
        cv::Mat frame_copy = camobj.imgproc->get_out_img();
        emit NewImg(timestamp, frame_copy(cv::Rect(0, 0, frame_copy.cols / 2, frame_copy.rows)));
    }

    void qt_feature_manager::startWork() {
        if (m_running) return;
        m_running = true;
        std::stringstream usbSavePath;
        usbSavePath << usbInfo.dir << "\\CamData\\vis_usb_reader\\" << usbInfo.usbname;
        std::filesystem::create_directories(usbSavePath.str().c_str());
        usbCam = std::make_unique<hwa_vis::vis_usb_reader<cv::cuda::GpuMat>>(usbInfo.ID, usbInfo.usbname, usbInfo.devicepathlist, usbSavePath.str(), false,
            true, usbInfo.framesize, usbInfo.framerate);
        if (usbCam->Init()) {
            captureThread = std::make_unique<std::thread>(
                &hwa_vis::vis_usb_reader_base::CaptureLoop, usbCam.get());
            emit IsOpen(true);
        }
        else {
            emit IsOpen(false);
        }

        cycleTimer = new QTimer(this);
        connect(cycleTimer, &QTimer::timeout, this, &qt_feature_manager::OneCycle);
        cycleTimer->start(0);
    }

    void qt_feature_manager::stopWork()
    {
        m_running = false;
        stop = true;
    }
}

