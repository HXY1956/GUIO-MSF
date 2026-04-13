#include "hwa_vis_coder_stereousb.h"
#include <DbgHelp.h>
#pragma comment(lib,"Dbghelp.lib")
#include <psapi.h> 
#include <type_traits>

template<typename MatType>
hwa_vis::vis_usb_reader<MatType>::vis_usb_reader(int deviceID, const std::string name, const std::string path, const std::string& saveDir, bool Out, bool PVT, std::pair<int,int> _size, int rate)
    : deviceID_(deviceID), deviceName_(name), devicePath_(path), saveDir_(saveDir), TerminalOut(Out), PVTStatus(PVT), Isize(_size), _framerate(rate) {
   OpenStatus = false;
}

template<typename MatType>
bool hwa_vis::vis_usb_reader<MatType>::Init() {
    gst_init(nullptr, nullptr);
    std::string pipeline;
    pipeline =
        "mfvideosrc device-path=\"" + devicePath_ + "\" ! "
        "video/x-raw,format=NV12,width=" + std::to_string(Isize.first) + ",height=" + std::to_string(Isize.second) + ",framerate=" + std::to_string(_framerate) + "/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "videoconvert ! "
        "video/x-raw,format=GRAY8 ! "
        "appsink name=sink drop=true max-buffers=1 sync=false";

    GError* error = nullptr;
    _pipeline = gst_parse_launch(pipeline.c_str(), &error);
    if (!_pipeline) {
        OpenStatus = false;
        std::cerr << "Failed to open USB stereo camera." + std::to_string(deviceID_) <<" due to: " << error->message << std::endl;
        return false;
    }
    sink = gst_bin_get_by_name(GST_BIN(_pipeline), "sink");
    gst_element_set_state(_pipeline, GST_STATE_PLAYING);

    OpenStatus = true;
    EpochTime = 1000.0 / _framerate;
    camskip = _framerate / 10;
    std::ostringstream out;
    out << saveDir_ << "\\CamStamp.txt";
    outfile.open(out.str(), std::ios::out | std::ios::trunc);
    std::ostringstream out1;
    out1 << saveDir_ << "\\CamTest.txt";
    outfile1.open(out1.str(), std::ios::out | std::ios::trunc);
    return true;
}

template<typename MatType>
void hwa_vis::vis_usb_reader<MatType>::CaptureLoop() {
    leftpath_ = saveDir_ + "\\cam0";
    rightpath_ = saveDir_ + "\\cam1";
    CreateDirectoryA(leftpath_.c_str(), NULL);
    CreateDirectoryA(rightpath_.c_str(), NULL);
    Iclear();
    bool type = (Frametype == CPU);
    stop = false;
    while (!stop) {
        double t0 = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) break;
        ImageTime = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        ImageTime -= ProjectStartTime;
        outfile1 << std::setiosflags(std::ios::fixed) << std::setprecision(3) << ImageTime << ": " <<"Read Cost: "<< ImageTime - t0 + ProjectStartTime<<" ";
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo _map;
        MatType left, right;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (gst_buffer_map(buffer, &_map, GST_MAP_READ)) {
                if constexpr (std::is_same_v<MatType, cv::Mat>) {
                    frame = cv::Mat(Isize.second, Isize.first, CV_8UC1, (void*)_map.data);
                }
                else {
                    cv::Mat frame_temp = cv::Mat(Isize.second, Isize.first, CV_8UC1, (void*)_map.data);
                    frame.upload(frame_temp);
                }
                gst_buffer_unmap(buffer, &_map);
            }
            left = frame(cv::Rect(0, 0, Isize.first / 2, Isize.second));
            right = frame(cv::Rect(Isize.first / 2, 0, Isize.first / 2, Isize.second));
        }
        _cv_show_status.notify_one();
        outfile << std::setiosflags(std::ios::fixed)<<std::setprecision(3) << ImageTime << "," << std::setw(5) << std::setfill('0') << USBCamCounter << ".bmp" << std::endl;
        if (USBCamCounter % camskip == 0) {
            if(TerminalOut) Ipush(left, right);
            if(PVTStatus) Ipush(ImageTime, left, right);
        }
        ++USBCamCounter;
        gst_sample_unref(sample);

    }
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
}

template<typename MatType>
void hwa_vis::vis_usb_reader<MatType>::Ishow_thread() {
    while (!stop) {
        std::unique_lock<std::mutex> lock(frame_mutex);
        _cv_show_status.wait(lock, [this]() { return !frame.empty() || stop; });
        if (stop) break;
        MatType frame_copy = frame.clone();
        lock.unlock();
        double scale = 0.25;
        if constexpr (std::is_same_v<MatType, cv::Mat>) {
            cv::Mat small_frame;
            cv::resize(frame_copy, small_frame, cv::Size(), scale, scale);
            cv::imshow(deviceName_, small_frame);
        }
        else if constexpr (std::is_same_v<MatType, cv::cuda::GpuMat>) {
            cv::Mat frame_cpu;
            cv::Mat small_frame;
            frame_copy.download(frame_cpu);
            cv::resize(frame_cpu, small_frame, cv::Size(), scale, scale);
            cv::imshow(deviceName_, small_frame);
        }
        cv::waitKey(1);
    }
}

template<typename MatType>
void hwa_vis::vis_usb_reader<MatType>::Iwrite_thread() {
    while (!stop) {
        std::unique_lock<std::mutex> lock(Writemutex_);
        _cv_write_status.wait(lock, [this]() { return !ImageWriteQueue.empty() || stop; });
        if (stop) break;
        std::pair<MatType, MatType> image_pair = ImageWriteQueue.front();
        ImageWriteQueue.pop();
        lock.unlock();

        std::ostringstream leftPath, rightPath;
        leftPath << leftpath_ << "\\" << std::setw(5) << std::setfill('0') << camskip * WritingCounter << ".bmp";
        rightPath << rightpath_ << "\\" << std::setw(5) << std::setfill('0') << camskip * WritingCounter << ".bmp";
        if constexpr (std::is_same_v<MatType, cv::Mat>) {
            if (cv::imwrite(leftPath.str(), image_pair.first) &&
                cv::imwrite(rightPath.str(), image_pair.second))
            {
                std::cerr << "Stereo Camera Write Successful: "
                    << std::setw(5) << std::setfill('0')
                    << camskip * WritingCounter << std::endl;
                WritingCounter++;
            }
        }
        else if constexpr (std::is_same_v<MatType, cv::cuda::GpuMat>) {
            cv::Mat left_cpu, right_cpu;
            image_pair.first.download(left_cpu);
            image_pair.second.download(right_cpu);
            if (cv::imwrite(leftPath.str(), left_cpu) &&
                cv::imwrite(rightPath.str(), right_cpu))
            {
                std::cerr << "Stereo Camera Write Successful: "
                    << std::setw(5) << std::setfill('0')
                    << camskip * WritingCounter << std::endl;
                WritingCounter++;
            }
        }
    }
}

template<typename MatType>
bool hwa_vis::vis_usb_reader<MatType>::Iget(double& ImgT, MatType& l, MatType& r) {
    std::lock_guard<std::mutex> lock(Imgmutex_);
    if (imagecoder.empty()) return false;
    ImgT = imagecoder.front().first;
    l = imagecoder.front().second.first;
    r = imagecoder.front().second.second;
    return true;
}

template<typename MatType>
void hwa_vis::vis_usb_reader<MatType>::Ipush(MatType l, MatType r) {
    std::pair<MatType, MatType> cloned_pair = { l.clone(), r.clone() };
    {
        std::lock_guard<std::mutex> lock(Writemutex_);
        ImageWriteQueue.push(cloned_pair);
        std::cerr << deviceName_ + " Queue size: " << ImageWriteQueue.size() << std::endl;
    }
    _cv_write_status.notify_one();
}

template<typename MatType>
void hwa_vis::vis_usb_reader<MatType>::Ipush(double ImgT, MatType l, MatType r) {
    std::lock_guard<std::mutex> lock(Imgmutex_);
    imagecoder.push(std::make_pair(ImgT, std::make_pair(l.clone(), r.clone())));
}

template<typename MatType>
bool hwa_vis::vis_usb_reader<MatType>::Ipop() {
    std::lock_guard<std::mutex> lock(Imgmutex_);
    if (imagecoder.empty()) return false;
    imagecoder.pop();
    return true;
}

template<typename MatType>
bool hwa_vis::vis_usb_reader<MatType>::Iempty(){
    std::lock_guard<std::mutex> lock(Imgmutex_);
    return imagecoder.empty();
}

template<typename MatType>
void hwa_vis::vis_usb_reader<MatType>::Iclear() {
    std::lock_guard<std::mutex> lock(Imgmutex_);
    while (!imagecoder.empty()) imagecoder.pop();
}

template class hwa_vis::vis_usb_reader<cv::Mat>;
template class hwa_vis::vis_usb_reader<cv::cuda::GpuMat>;
