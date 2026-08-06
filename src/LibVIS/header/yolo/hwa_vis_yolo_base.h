#ifndef HWA_VIS_YOLO_BASE_H 
#define HWA_VIS_YOLO_BASE_H

#include <opencv2/opencv.hpp>

namespace hwa_vis {
    struct Detection {
        int class_id;
        std::string classname;
        float confidence;
        cv::Rect box;
    };

    struct DetectBox {
        float x0, y0, x1, y1;
    };
}

#endif