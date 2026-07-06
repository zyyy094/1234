#ifndef YOLOV8_INFER_H
#define YOLOV8_INFER_H

#include <onnxruntime_cxx_api.h>
#include "common.h"

class YoloInfer {
private:
    Ort::Env env;
    Ort::SessionOptions opts;
    Ort::Session session;
public:
    YoloInfer(const std::string& model_path, int thread_num = 2);
    std::vector<Detection> infer(const cv::Mat& roi_img);
};

#endif