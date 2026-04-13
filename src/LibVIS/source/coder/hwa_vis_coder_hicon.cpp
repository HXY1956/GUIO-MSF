#include "hwa_vis_coder_hicon.h"

hwa_vis::vis_hicon_reader::~vis_hicon_reader() { CloseCam(); };
hwa_vis::vis_hicon_reader::vis_hicon_reader() { frameCount = 0;}
hwa_vis::vis_hicon_reader::vis_hicon_reader(CameraContext _context) { context = _context; frameCount = 0; }

void hwa_vis::vis_hicon_reader::ShowErrorMsg(int nErrorNum)
{
    std::string errorMsg = "";
    switch (nErrorNum)
    {
    case MV_E_HANDLE:           errorMsg += "Error or invalid handle ";                                         break;
    case MV_E_SUPPORT:          errorMsg += "Not supported function ";                                          break;
    case MV_E_BUFOVER:          errorMsg += "Cache is full ";                                                   break;
    case MV_E_CALLORDER:        errorMsg += "Function calling order error ";                                    break;
    case MV_E_PARAMETER:        errorMsg += "Incorrect parameter ";                                             break;
    case MV_E_RESOURCE:         errorMsg += "Applying resource failed ";                                        break;
    case MV_E_NODATA:           errorMsg += "No data ";                                                         break;
    case MV_E_PRECONDITION:     errorMsg += "Precondition error, or running environment changed ";              break;
    case MV_E_VERSION:          errorMsg += "Version mismatches ";                                              break;
    case MV_E_NOENOUGH_BUF:     errorMsg += "Insufficient memory ";                                             break;
    case MV_E_ABNORMAL_IMAGE:   errorMsg += "Abnormal image, maybe incomplete image because of lost packet ";   break;
    case MV_E_UNKNOW:           errorMsg += "Unknown error ";                                                   break;
    case MV_E_GC_GENERIC:       errorMsg += "General error ";                                                   break;
    case MV_E_GC_ACCESS:        errorMsg += "Node accessing condition error ";                                  break;
    case MV_E_ACCESS_DENIED:	errorMsg += "No permission ";                                                   break;
    case MV_E_BUSY:             errorMsg += "Device is busy, or network disconnected ";                         break;
    case MV_E_NETER:            errorMsg += "Network error ";                                                   break;
    }

    std::cerr<< errorMsg << std::endl;
}


void hwa_vis::vis_hicon_reader::CloseCam(){
    MV_CC_StopGrabbing(context.handle);
    MV_CC_CloseDevice(context.handle);
    MV_CC_DestroyHandle(context.handle);
    outfile.close();
}

int hwa_vis::vis_hicon_reader::OpenCam() {
    
    int nRet = MV_CC_CreateHandle(&context.handle, context.pDeviceInfo);
    if (nRet != MV_OK || !context.handle) {
        ShowErrorMsg(nRet);
        return 0;
    } 
    if (MV_CC_OpenDevice(context.handle) != MV_OK) return 0;

    MVCC_INTVALUE stWidth = {}, stHeight = {};
    MV_CC_GetIntValue(context.handle, "Width", &stWidth);
    MV_CC_GetIntValue(context.handle, "Height", &stHeight);
    size.width = stWidth.nCurValue;
    size.height = stHeight.nCurValue;
    int HeartBeatTimeout = 2000;
    nRet = MV_CC_SetHeartBeatTimeout(context.handle, HeartBeatTimeout);
    if (MV_OK != nRet)
    {
        ShowErrorMsg(nRet);
    }

    if (context.pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        int packetSize = MV_CC_GetOptimalPacketSize(context.handle);
        if (packetSize > 0)
            MV_CC_SetIntValueEx(context.handle, "GevSCPSPacketSize", packetSize);
    }

    MV_CC_SetEnumValue(context.handle, "TriggerMode", 0);
    //MV_CC_SetBoolValue(context.handle, "AcquisitionFrameRateEnable", false);
    //MV_CC_SetFloatValue(context.handle, "AcquisitionFrameRate", 30.0);

    nRet = MV_CC_StartGrabbing(context.handle);
    if (nRet != MV_OK) {
        ShowErrorMsg(nRet);
        return 0;
    }
    return 1;
};

void hwa_vis::vis_hicon_reader::CameraThread() {
    outfile.open(context.saveDir + "\\Stamp\\HiconStamp.txt", std::ios::out | std::ios::trunc);

    while (true) {
        int ret = MV_CC_GetImageBuffer(context.handle, &stOutFrame, 10000);
        if (ret == MV_OK) {
            {
                std::lock_guard<std::mutex> lock(imagemutex_);
                img_start = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                img_start -= ProjectStartTime;
                outfile << frameCount << "  " << img_start << std::endl;
                ConvertToCVMat(image);
            }
            {
                std::lock_guard<std::mutex> lock(HKmutex_);
                image_queue.push(std::make_pair(frameCount, image));
                MV_CC_FreeImageBuffer(context.handle, &stOutFrame);
                frameCount++;
            }
        }
        else {
            ShowErrorMsg(ret);
        }
    }
}

void hwa_vis::vis_hicon_reader::SaveImageToFile() {
    std::lock_guard<std::mutex> lock(HKmutex_);
    if (image_queue.size() == 0) return;
    std::stringstream pathStream;
    pathStream << context.saveDir << "\\frame_" << std::setw(4) << std::setfill('0') << image_queue.front().first << ".bmp";
    std::string fullPath = pathStream.str();
    bool ok = cv::imwrite(fullPath, image_queue.front().second);
    image_queue.pop();
    if (!ok) {
        std::cerr << "[Hicon] OpenCV failed to write image: " << fullPath << std::endl;
    }
}

void hwa_vis::vis_hicon_reader::SaveLatestImageToFile() {
    std::lock_guard<std::mutex> lock(HKmutex_);
    if (image_queue.size() == 0) return;
    std::stringstream pathStream;
    pathStream << context.saveDir << "\\frame_" << std::setw(4) << std::setfill('0') << image_queue.back().first << ".bmp";
    std::string fullPath = pathStream.str();
    bool ok = cv::imwrite(fullPath, image_queue.back().second);
    image_queue = {};
    if (!ok) {
        std::cerr << "[Hicon] OpenCV failed to write image: " << fullPath << std::endl;
    }
}

bool hwa_vis::vis_hicon_reader::ConvertToCVMat(cv::Mat& image){
    unsigned char* pData = (unsigned char*)stOutFrame.pBufAddr;
    MV_FRAME_OUT_INFO_EX* pInfo = &stOutFrame.stFrameInfo;

    if (!pData || !pInfo) {
        return false;
    }

    int width = pInfo->nWidth;
    int height = pInfo->nHeight;
    MvGvspPixelType pixel_type = pInfo->enPixelType;

    if (pixel_type == PixelType_Gvsp_BayerRG8 || pixel_type == PixelType_Gvsp_BayerBG8) {
        cv::Mat bayer(height, width, CV_8UC1, pData);
        cv::cvtColor(bayer, image, cv::COLOR_BayerRG2GRAY);
        return true;
    }
    else if (pixel_type == PixelType_Gvsp_BayerGB8) {
        cv::Mat bayer(height, width, CV_8UC1, pData);
        cv::cvtColor(bayer, image, cv::COLOR_BayerGB2GRAY);
    }
    else if (pixel_type == PixelType_Gvsp_BayerGR8) {
        cv::Mat bayer(height, width, CV_8UC1, pData);
        cv::cvtColor(bayer, image, cv::COLOR_BayerGR2GRAY);
        return true;
    }
    else if (pixel_type == PixelType_Gvsp_Mono8) {
        image = cv::Mat(height, width, CV_8UC1, pData).clone();
        return true;
    }
    else if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
        cv::Mat rgb = cv::Mat(height, width, CV_8UC3, pData).clone();
        cv::cvtColor(rgb, image, cv::COLOR_BGR2GRAY);
        return true;
    }
    else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat rgb(height, width, CV_8UC3, pData);
        cv::cvtColor(rgb, image, cv::COLOR_RGB2GRAY);
        return true;
    }
    else {
        std::cerr << "Unsupported pixel type: " << std::hex << pixel_type << std::endl;
        return false;
    }
}
