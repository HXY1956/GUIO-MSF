#ifndef hwa_vis_yolo_v8model_h
#define hwa_vis_yolo_v8model_h
#include <openvino/openvino.hpp>
#include <openvino/runtime/properties.hpp>
#include <opencv2/opencv.hpp>
#include <condition_variable> 
#include <mutex>
#include <opencv2/videoio.hpp>
#include "hwa_set_vis.h"
#include "hwa_base_SharedResource.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;
using namespace hwa_set;

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

    class vis_yolo_v8ov {
    public:
        vis_yolo_v8ov() {};
        vis_yolo_v8ov(std::shared_ptr<hwa_set::set_base> _gset);
        ~vis_yolo_v8ov() {};
        std::vector<Detection> detect(const cv::Mat& frame);
        void SetCLS(const int _cls) { cls = _cls; }
        void SetNumPred(const int _numpred) { num_pred = _numpred; }
        void SetLabelList(const  std::vector<std::string> _label_list) {
            label_list = _label_list;
            assert(label_list.size() == cls);
        }
        void preprocess(const cv::Mat& img, cv::Mat& out);
        void postprocess(const float* raw,
            const cv::Size& img_size,
            std::vector<Detection>& dets);

    private:
        bool LogOut = false;
        ov::Core core_;
        ov::CompiledModel compiled_;
        ov::InferRequest req_;
        cv::Size net_size_;
        std::vector<std::string> label_list;
        std::string model_path;
        cv::Size input_size;
        int cls;
        std::vector<std::string> clsname;
        int num_pred;
        float conf_th_, nms_th_;
    };
}

#endif

