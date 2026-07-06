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
    Ort::Session session;
    std::vector<float> preprocess(const cv::Mat& frame);
    std::vector<Detection> postprocess(float* output, int img_w, int img_h);
};

#endif