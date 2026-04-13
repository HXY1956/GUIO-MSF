#ifndef hwa_vis_coder_daheng_h
#define hwa_vis_coder_daheng_h
#include <iostream>
#include "GalaxyIncludes.h"
#include <string>
#include <sstream>
#include <stdio.h>
#include <chrono>
#include <iomanip> 
#include <thread>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "hwa_base_glob.h"
#include "hwa_vis_proc_standard.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis {
    class vis_daheng_reader {
    public:
        vis_daheng_reader(int index, const std::string& saveDir);
        vis_daheng_reader();
        ~vis_daheng_reader();

        bool Init();                 
        void Start();               
        void Stop();                
        void ThreadLoop();         
        void SaveImageToFile();
        void SaveLatestImageToFile();

        bool ConvertToCVMat(cv::Mat& image);
        cv::Mat GetImage() {
            std::lock_guard<std::mutex> lock(imagemutex_);
            return image;
        };
        int GetFrameCount() {
            std::lock_guard<std::mutex> lock(DHmutex_);
            return frameCount;
        };
        double GetImageTime(){
            std::lock_guard<std::mutex> lock(imagemutex_);
            return img_start;
        }
        cv::Size GetSize() {
            std::lock_guard<std::mutex> lock(imagemutex_);
            return image.size();
        }

    private:
        int deviceIndex;
        std::string saveDir;
        CGXDevicePointer pDevice;
        CGXStreamPointer pStream;
        CGXFeatureControlPointer pRemoteControl;
        CImageDataPointer pImgData;
        bool bRun = false;
        double img_start;
        double img_end;
        cv::Mat image;
        std::queue<std::pair<int,cv::Mat>> image_queue;
        int frameCount;
        std::mutex DHmutex_;
        std::mutex imagemutex_;
        std::ofstream outfile;
    };

    extern GxIAPICPP::gxdeviceinfo_vector dahengList;

} // namespace cam

#endif