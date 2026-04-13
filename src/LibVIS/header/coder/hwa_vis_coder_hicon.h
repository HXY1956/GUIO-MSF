#ifndef hwa_vis_coder_hicon_h
#define hwa_vis_coder_hicon_h
#include <stdio.h>
#include <process.h>
#include <conio.h>
#include <string>
#include <chrono>
#include <thread>
#include <direct.h>   
#include <ctime>        
#include <iomanip>      
#include <sstream>     
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "MvCameraControl.h"
#include "hwa_base_glob.h"
#include "hwa_vis_proc_Standard.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis{
    struct CameraContext {
        void* handle;
        std::string saveDir;
        MV_SAVE_IAMGE_TYPE imageType;
        int deviceIndex;
        MV_CC_DEVICE_INFO* pDeviceInfo;
    };

    class vis_hicon_reader {
    public:
        vis_hicon_reader(CameraContext _context);
        vis_hicon_reader();
        ~vis_hicon_reader();

        int OpenCam();
        void CloseCam();
        void CameraThread();
        void SaveImageToFile();
        void SaveLatestImageToFile();
        bool ConvertToCVMat(cv::Mat& image);
        void ShowErrorMsg(int nErrorNum);
        cv::Mat GetImage() {
            std::lock_guard<std::mutex> lock(imagemutex_);
            cv::Mat image1;
            cv::resize(image, image1, cv::Size(640, 480));
            return image1;
        };
        double GetImageTime(){
            std::lock_guard<std::mutex> lock(imagemutex_);
            return img_start;
        }
        int GetFrameCount() {
            std::lock_guard<std::mutex> lock(HKmutex_);
            return frameCount;
        };
        cv::Size GetSize() {
            return size;
        }
    public:
        CameraContext context;
        MV_FRAME_OUT stOutFrame = { 0 };
        cv::Mat image;

    private:
        cv::Size size;
        std::string g_saveDirectory;
        double img_start;
        double img_end;
        std::queue<std::pair<int, cv::Mat>> image_queue;
        int frameCount;
        std::mutex HKmutex_;
        std::mutex imagemutex_;
        std::ofstream outfile;
    };
}

#endif