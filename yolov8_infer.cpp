#include "yolov8_infer.h"
#include <opencv2/dnn.hpp>
#include <iostream>

using namespace std;
using namespace cv;

YOLOv8Infer::YOLOv8Infer() : env(ORT_LOGGING_LEVEL_WARNING, "yolo") {}

YOLOv8Infer::~YOLOv8Infer() {}

bool YOLOv8Infer::loadModel(const string& model_path) {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(2);
    try {
        session = Ort::Session(env, model_path.c_str(), opts);
        cout << "[INFO] YOLO 模型加载成功" << endl;
        return true;
    } catch (...) {
        cerr << "[ERROR] YOLO 模型加载失败" << endl;
        return false;
    }
}

vector<float> YOLOv8Infer::preprocess(const Mat& frame) {
    Mat resized, float_img;
    resize(frame, resized, Size(INPUT_W, INPUT_H));
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    vector<float> tensor(3 * INPUT_W * INPUT_H);
    for (int c = 0; c < 3; ++c)
        for (int h = 0; h < INPUT_H; ++h)
            for (int w = 0; w < INPUT_W; ++w)
                tensor[c * INPUT_H * INPUT_W + h * INPUT_W + w] = float_img.at<Vec3f>(h, w)[c];
    return tensor;
}

vector<Detection> YOLOv8Infer::postprocess(float* output, int img_w, int img_h) {
    int num_dets = 2100, num_classes = 80;
    float scale_w = (float)img_w / INPUT_W;
    float scale_h = (float)img_h / INPUT_H;
    vector<Detection> detections;

    for (int i = 0; i < num_dets; ++i) {
        float* ptr = output + i * (num_classes + 4);
        float max_conf = 0;
        int max_id = 0;
        for (int c = 0; c < num_classes; ++c) {
            float conf = ptr[4 + c];
            if (conf > max_conf) { max_conf = conf; max_id = c; }
        }
        if (max_conf < CONF_THRESH) continue;

        bool is_target = false;
        for (int cls : TARGET_CLASSES) {
            if (max_id == cls) { is_target = true; break; }
        }
        if (!is_target) continue;

        float cx = ptr[0], cy = ptr[1], w = ptr[2], h = ptr[3];
        int x = max(0, (int)((cx - w / 2) * scale_w));
        int y = max(0, (int)((cy - h / 2) * scale_h));
        int width = min((int)(w * scale_w), img_w - x);
        int height = min((int)(h * scale_h), img_h - y);
        if (width <= 0 || height <= 0) continue;

        Detection d;
        d.class_id = max_id;
        d.confidence = max_conf;
        d.box = Rect(x, y, width, height);
        auto it = CLASS_NAMES.find(max_id);
        d.name = (it != CLASS_NAMES.end()) ? it->second : "object";
        detections.push_back(d);
    }

    if (detections.empty()) return {};
    vector<int> indices;
    vector<Rect> boxes;
    vector<float> confs;
    for (auto& d : detections) {
        boxes.push_back(d.box);
        confs.push_back(d.confidence);
    }
    dnn::NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH, indices);

    vector<Detection> result;
    for (int idx : indices) result.push_back(detections[idx]);
    return result;
}

vector<Detection> YOLOv8Infer::detect(const Mat& frame) {
    vector<float> input_data = preprocess(frame);
    vector<int64_t> input_shape = {1, 3, INPUT_H, INPUT_W};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
        input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

    const char* in_names[] = {"images"};
    const char* out_names[] = {"output0"};
    Ort::Value output_tensor{nullptr};

    try {
        session.Run(Ort::RunOptions{}, in_names, &input_tensor, 1,
                    out_names, &output_tensor, 1);
        float* out_ptr = output_tensor.GetTensorMutableData<float>();
        return postprocess(out_ptr, frame.cols, frame.rows);
    } catch (...) {
        cerr << "[WARN] 推理异常" << endl;
        return {};
    }
}