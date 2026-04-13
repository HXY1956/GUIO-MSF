#include "hwa_vis_yolo_v8model.h"
#include <algorithm>

using namespace hwa_vis;

vis_yolo_v8ov::vis_yolo_v8ov(std::shared_ptr<hwa_set::set_base> _gset)
{
    try {
        model_path = dynamic_cast<hwa_set::set_vis*>(_gset.get())->modelpath();
        input_size = cv::Size(dynamic_cast<hwa_set::set_vis*>(_gset.get())->inputsize().first, dynamic_cast<hwa_set::set_vis*>(_gset.get())->inputsize().second);
        conf_th_ = dynamic_cast<hwa_set::set_vis*>(_gset.get())->conf();
        nms_th_ = dynamic_cast<hwa_set::set_vis*>(_gset.get())->nms();
        num_pred = dynamic_cast<hwa_set::set_vis*>(_gset.get())->numpred();
        cls = dynamic_cast<hwa_set::set_vis*>(_gset.get())->classnumber();
        clsname = dynamic_cast<hwa_set::set_vis*>(_gset.get())->clsname();
        compiled_ = core_.compile_model(model_path, "CPU");
        req_ = compiled_.create_infer_request();
    }
    catch (const ov::Exception& e) {
        std::cerr << "OpenVINO Exception: " << e.what() << std::endl;
    }
}

void vis_yolo_v8ov::preprocess(const cv::Mat& img, cv::Mat& out) {
    cv::Mat bgr;
    cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
    cv::dnn::blobFromImage(bgr, out, 1.0 / 255.0, input_size, cv::Scalar(), true, false, CV_32F);
}

std::vector<Detection> vis_yolo_v8ov::detect(const cv::Mat& frame) {
    cv::Mat blog;
    preprocess(frame, blog);
    ov::Tensor input_tensor = ov::Tensor(ov::element::f32, { 1, 3, 640, 640 }, blog.data);
    req_.set_input_tensor(input_tensor);
    req_.infer();
    const float* raw = req_.get_output_tensor().data<const float>();
    auto shape = req_.get_output_tensor().get_shape();
    std::vector<Detection> dets;
    postprocess(raw,frame.size(), dets);
    return dets;
}

void Scale(float& x, int size0, int size1) {
    x = x * size1 / size0;
}

void vis_yolo_v8ov::postprocess(const float* ptr,
    const cv::Size& img_size,
    std::vector<Detection>& dets) {
    for (int i = 0; i < num_pred; ++i) {
        float x = ptr[i];
        float y = ptr[num_pred + i];
        float w = ptr[num_pred * 2 + i] + 10;
        float h = ptr[num_pred * 3 + i] + 10;
        float x0 = x - w * 0.5f;
        float y0 = y - h * 0.5f;
        Scale(x0, 640, img_size.width);
        Scale(y0, 640, img_size.height);
        Scale(w, 640, img_size.width);
        Scale(h, 640, img_size.height);

        for (int c = 0; c < cls; ++c) {
            float conf = ptr[num_pred * (4 + c) + i];
            if (conf < conf_th_) continue;
            Detection det;
            det.class_id = c;
            det.classname = clsname[c];
            det.confidence = conf;
            det.box = cv::Rect(x0, y0, w, h);
            dets.push_back(det);
        }
    }

    if (dets.empty()) return;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> indices;
    for (const auto& d : dets) {
        boxes.emplace_back(d.box);
        scores.emplace_back(d.confidence);
    }
    cv::dnn::NMSBoxes(boxes, scores, conf_th_, nms_th_, indices);
    std::vector<Detection> result;

    for (int i : indices) {
        result.push_back(dets[i]);
    }
    dets.swap(result);
}