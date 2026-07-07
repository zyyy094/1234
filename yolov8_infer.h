#ifndef YOLOV8_INFER_H
#define YOLOV8_INFER_H

#include "common.h"
#include <onnxruntime_cxx_api.h>
#include <vector>

class YOLOv8Infer {
public:
    YOLOv8Infer();
    ~YOLOv8Infer();
    bool loadModel(const std::string& model_path);
    std::vector<Detection> detect(const cv::Mat& frame);

private:
    Ort::Env env;
    Ort::Session session{nullptr};

    // Letterbox 预处理参数
    float m_scale = 1.0f;
    int m_pad_x = 0;
    int m_pad_y = 0;

    std::vector<float> preprocess(const cv::Mat& frame);
    std::vector<Detection> postprocess(const float* output, int orig_w, int orig_h,
                                       int num_anchors, int num_classes, int layout);
};

#endif
