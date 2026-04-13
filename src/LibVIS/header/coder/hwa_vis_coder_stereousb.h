#ifndef hwa_vis_coder_stereousb_h
#define hwa_vis_coder_stereousb_h
#define _WINSOCKAPI_
#define NOMINMAX
#include <windows.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/core/cuda.hpp>
#include <cuda_runtime.h>
#include "hwa_vis_proc_Standard.h"
#include "hwa_base_glob.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis {
    class vis_usb_reader_base {
    public:
        virtual ~vis_usb_reader_base() = default;
        virtual bool Init() = 0;
        virtual void CaptureLoop() = 0;
        virtual bool GetOpenStatus() = 0;
        virtual bool GetOutStatus() = 0;
        virtual int GetDeviceID() = 0;
        virtual void Stop() = 0;
        virtual bool Iget(double& ImgT, cv::Mat& l, cv::Mat& r) { return false; }
        virtual bool Iget(double& ImgT, cv::cuda::GpuMat& l, cv::cuda::GpuMat& r) { return false; }
        virtual bool Ipop() = 0;
        virtual bool Iempty() = 0;
        virtual void Iclear() = 0;
        virtual void Iwrite_thread() = 0;
        virtual void Ishow_thread() = 0;
    };

    template <typename MatType>
    class vis_usb_reader : public vis_usb_reader_base {
    public:
        enum FRAME_TYPE {
            GPU,
            CPU
        };

        using FramePair = std::pair<MatType, MatType>;
        using TimestampedFrame = std::pair<double, FramePair>;

        vis_usb_reader(int deviceID, const std::string name, const std::string path, const std::string& saveDir, bool Out, bool PVT, std::pair<int, int> _size, int _frame_rate);
        ~vis_usb_reader() { Stop(); }
        void Stop() override {
            stop = true;
        }
        bool Init() override;
        void CaptureLoop() override;
        void Ipush(MatType l, MatType r);
        void Ipush(double ImgT, MatType l, MatType r);
        bool Iget(double& ImgT, MatType& l, MatType& r) override;
        bool Ipop() override;
        bool Iempty() override;
        void Iclear() override;
        void Iwrite_thread() override;
        void Ishow_thread() override;
        MatType GetFrame() {
            return frame;
        }
        bool GetOpenStatus() override { return OpenStatus; }
        bool GetOutStatus() override { return TerminalOut; }
        int GetDeviceID() override { return deviceID_; }

    private:
        std::atomic<bool> stop{ false };
        int deviceID_;
        FRAME_TYPE Frametype = GPU;
        std::string deviceName_;
        std::string devicePath_;
        GstElement* _pipeline;
        GstElement* sink;
        std::pair<int, int> Isize;
        int _framerate;
        int camskip;
        double EpochTime;
        bool OpenStatus;
        std::string saveDir_;
        std::string leftpath_;
        std::string rightpath_;
        int USBCamCounter = 0;
        int WritingCounter = 0;
        std::queue<TimestampedFrame> imagecoder;
        std::queue<FramePair> ImageWriteQueue;
        bool _isFirstImage = true;
        bool TerminalOut = false;
        bool PVTStatus = false;
        double ImageTime;
        std::mutex Imgmutex_;
        std::mutex Writemutex_;
        std::mutex frame_mutex;
        std::condition_variable _cv_show_status;
        std::condition_variable _cv_write_status;
        std::ofstream outfile1;
        std::ofstream outfile;
        MatType frame;
    };
};

#endif