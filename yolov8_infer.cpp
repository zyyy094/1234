#include "yolov8_infer.h"
#include <iostream>

YoloInfer::YoloInfer(const std::string& model_path, int thread_num)
    : env(ORT_LOGGING_LEVEL_WARNING, "yolo"), opts(Ort::SessionOptions()), session(nullptr) {
    opts.SetIntraOpNumThreads(thread_num);
    session = Ort::Session(env, model_path.c_str(), opts);
}

static std::vector<float> preprocess(const cv::Mat& frame) {
    cv::Mat resized, float_img;
    resize(frame, resized, cv::Size(INPUT_W, INPUT_H));
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    std::vector<float> input_tensor(3 * INPUT_W * INPUT_H);
    for (int c = 0; c < 3; ++c)
        for (int h = 0; h < INPUT_H; ++h)
            for (int w = 0; w < INPUT_W; ++w)
                input_tensor[c * INPUT_H * INPUT_W + h * INPUT_W + w] = float_img.at<cv::Vec3f>(h, w)[c];
    return input_tensor;
}

static std::vector<Detection> postprocess(float* output, int img_w, int img_h) {
    int num_dets = 2100, num_classes = 80;
    float scale_w = (float)img_w / INPUT_W;
    float scale_h = (float)img_h / INPUT_H;
    std::vector<Detection> detections;

    for (int i = 0; i < num_dets; ++i) {
        float* ptr = output + i * (num_classes + 4);
        float max_conf = 0;
        int max_id = 0;
        for (int c = 0; c < num_classes; ++c) {
            float conf = ptr[4 + c];
            if (conf > max_conf) {
                max_conf = conf;
                max_id = c;
            }
        }
        if (max_conf < CONF_THRESH) continue;

        bool is_target = false;
        for (int cls : TARGET_CLASSES) {
            if (max_id == cls) {
                is_target = true;
                break;
            }
        }
        if (!is_target) continue;

        float cx = ptr[0], cy = ptr[1], w = ptr[2], h = ptr[3];
        int x = std::max(0, (int)((cx - w / 2) * scale_w));
        int y = std::max(0, (int)((cy - h / 2) * scale_h));
        int width = std::min((int)(w * scale_w), img_w - x);
        int height = std::min((int)(h * scale_h), img_h - y);
        if (width <= 0 || height <= 0) continue;

        Detection d;
        d.class_id = max_id;
        d.confidence = max_conf;
        d.box = cv::Rect(x, y, width, height);
        d.name = class_names.count(max_id) ? class_names[max_id] : "object";
        detections.push_back(d);
    }

    if (detections.empty()) return {};
    std::vector<int> indices;
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    for (auto& d : detections) {
        boxes.push_back(d.box);
        confs.push_back(d.confidence);
    }
    cv::dnn::NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH, indices);

    std::vector<Detection> result;
    for (int idx : indices) result.push_back(detections[idx]);
    return result;
}

std::vector<Detection> YoloInfer::infer(const cv::Mat& roi_img) {
    std::vector<float> input_data = preprocess(roi_img);
    std::vector<int64_t> input_shape = {1, 3, INPUT_H, INPUT_W};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
        input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

    const char* input_names[] = {"images"};
    const char* output_names[] = {"output0"};
    Ort::Value output_tensor{nullptr};

    session.Run(Ort::RunOptions{}, input_names, &input_tensor, 1, output_names, &output_tensor, 1);
    float* out_ptr = output_tensor.GetTensorMutableData<float>();
    return postprocess(out_ptr, roi_img.cols, roi_img.rows);
}