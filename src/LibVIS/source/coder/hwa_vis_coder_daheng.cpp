#include "hwa_vis_coder_daheng.h"

hwa_vis::vis_daheng_reader::vis_daheng_reader(int index, const std::string& saveDir): deviceIndex(index), saveDir(saveDir) { frameCount = 0; }
hwa_vis::vis_daheng_reader::vis_daheng_reader() { frameCount = 0; deviceIndex = 0; }
hwa_vis::vis_daheng_reader::~vis_daheng_reader() {Stop();}
namespace hwa_vis {
    GxIAPICPP::TList_vector hwa_vis::dahengList;
}

bool hwa_vis::vis_daheng_reader::Init() {
    pDevice = IGXFactory::GetInstance().OpenDeviceBySN(dahengList[deviceIndex].GetSN(), GX_ACCESS_EXCLUSIVE);
    pRemoteControl = pDevice->GetRemoteFeatureControl();
    if (pDevice->GetStreamCount() > 0)
        pStream = pDevice->OpenStream(0);
    else
        return false;
    pRemoteControl->GetEnumFeature("UserSetSelector")->SetValue("Default");
    pRemoteControl->GetCommandFeature("UserSetLoad")->Execute();
    return true;
}

void hwa_vis::vis_daheng_reader::Start() {
    bRun = true;
    pStream->StartGrab();
    pRemoteControl->GetCommandFeature("AcquisitionStart")->Execute();
}

void hwa_vis::vis_daheng_reader::Stop() {
    if (!bRun) return;
    bRun = false;
    pRemoteControl->GetCommandFeature("AcquisitionStop")->Execute();
    pStream->StopGrab();
    pStream->Close();
    pDevice->Close();
    outfile.close();
}

void hwa_vis::vis_daheng_reader::ThreadLoop() {
    Start();
    outfile.open(saveDir + "\\Stamp\\DahengStamp.txt", std::ios::out | std::ios::trunc);
    while (bRun) {
        pImgData = pStream->DQBuf(1000);
        if (GX_FRAME_STATUS_SUCCESS == pImgData->GetStatus()) {
            {
                std::lock_guard<std::mutex> lock(imagemutex_);
                image = cv::Mat(pImgData->GetHeight(), pImgData->GetWidth(), CV_8UC1, const_cast<void*>(pImgData->GetBuffer()));
                //cv::resize(image, image, cv::Size(640, 480));
                //cv::imshow("frame", image);
                //cv::waitKey(1);
                img_start = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                img_start -= ProjectStartTime;
                outfile << frameCount << "  " << img_start << std::endl;
            }
            {
                std::lock_guard<std::mutex> lock(DHmutex_);
                if (!image.empty())image_queue.push(std::make_pair(frameCount, image));
                frameCount++;
            }
        }
        pStream->QBuf(pImgData);
    }
}

void hwa_vis::vis_daheng_reader::SaveImageToFile() {
    std::lock_guard<std::mutex> lock(DHmutex_);
    if (image_queue.size() == 0) return;
    std::stringstream pathStream;
    pathStream << saveDir << "\\frame_" << std::setw(4) << std::setfill('0') << image_queue.front().first << ".bmp";
    std::string fullPath = pathStream.str();
    bool ok = cv::imwrite(fullPath, image_queue.front().second);
    image_queue.pop();
    if (!ok) {
        std::cerr << "[Daheng] OpenCV failed to write image: " << fullPath << std::endl;
    }
}

void hwa_vis::vis_daheng_reader::SaveLatestImageToFile() {
    std::lock_guard<std::mutex> lock(DHmutex_);
    if (image_queue.size() == 0) return;
    std::stringstream pathStream;
    pathStream << saveDir << "\\frame_" << std::setw(4) << std::setfill('0') << image_queue.back().first << ".bmp";
    std::string fullPath = pathStream.str();
    bool ok = cv::imwrite(fullPath, image_queue.back().second);
    image_queue = {};
    if (!ok) {
        std::cerr << "[Daheng] OpenCV failed to write image: " << fullPath << std::endl;
    }
}

bool hwa_vis::vis_daheng_reader::ConvertToCVMat(cv::Mat& image){
    if (!GX_FRAME_STATUS_SUCCESS == pImgData->GetStatus()) return false;
    int width = pImgData->GetWidth();
    int height = pImgData->GetHeight();
    GX_PIXEL_FORMAT_ENTRY pixelFormat = pImgData->GetPixelFormat();
    const void* rawBuffer = pImgData->GetBuffer();
    image = cv::Mat(height, width, CV_8UC1, const_cast<void*>(rawBuffer)).clone();
    return true;
}

