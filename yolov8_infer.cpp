#include "yolov8_infer.h"
#include <opencv2/dnn.hpp>
#include <iostream>
#include <algorithm>
#include <map>

using namespace std;
using namespace cv;
using namespace Ort;

YOLOv8Infer::YOLOv8Infer() : env(ORT_LOGGING_LEVEL_WARNING, "yolo") {}
YOLOv8Infer::~YOLOv8Infer() {}

bool YOLOv8Infer::loadModel(const string& model_path) {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(2);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    try {
        session = Ort::Session(env, model_path.c_str(), opts);
        cout << "[INFO] YOLO 模型加载成功" << endl;
        return true;
    } catch (...) {
        cerr << "[ERROR] YOLO 模型加载失败" << endl;
        return false;
    }
}

// ===== Letterbox 预处理（保持宽高比，填充灰色 114） =====
vector<float> YOLOv8Infer::preprocess(const Mat& frame) {
    float scale = min((float)INPUT_W / frame.cols, (float)INPUT_H / frame.rows);
    int new_w = (int)(frame.cols * scale);
    int new_h = (int)(frame.rows * scale);

    Mat resized;
    resize(frame, resized, Size(new_w, new_h), 0, 0, INTER_LINEAR);

    Mat padded(INPUT_H, INPUT_W, CV_8UC3, Scalar(114, 114, 114));
    m_pad_x = (INPUT_W - new_w) / 2;
    m_pad_y = (INPUT_H - new_h) / 2;
    resized.copyTo(padded(Rect(m_pad_x, m_pad_y, new_w, new_h)));
    m_scale = scale;

    // BGR→RGB, 归一化, CHW
    Mat rgb;
    cvtColor(padded, rgb, COLOR_BGR2RGB);

    vector<float> tensor(3 * INPUT_H * INPUT_W);
    for (int c = 0; c < 3; ++c)
        for (int h = 0; h < INPUT_H; ++h)
            for (int w = 0; w < INPUT_W; ++w)
                tensor[c * INPUT_H * INPUT_W + h * INPUT_W + w] =
                    rgb.at<Vec3b>(h, w)[c] / 255.0f;
    return tensor;
}

vector<Detection> YOLOv8Infer::postprocess(const float* output, int orig_w, int orig_h,
                                            int num_anchors, int num_classes, int layout) {
    vector<Detection> detections;
    int stride = num_classes + 4;

    for (int i = 0; i < num_anchors; i++) {
        float cx, cy, w, h;
        float max_conf = 0;
        int max_id = -1;

        if (layout == 0) {
            // [1, 84, N] 布局
            cx = output[0 * num_anchors + i];
            cy = output[1 * num_anchors + i];
            w  = output[2 * num_anchors + i];
            h  = output[3 * num_anchors + i];
            for (int c = 0; c < num_classes; ++c) {
                float conf = output[(4 + c) * num_anchors + i];
                if (conf > max_conf) { max_conf = conf; max_id = c; }
            }
        } else {
            // [1, N, 84] 布局
            const float* ptr = output + i * stride;
            cx = ptr[0]; cy = ptr[1]; w = ptr[2]; h = ptr[3];
            for (int c = 0; c < num_classes; ++c) {
                float conf = ptr[4 + c];
                if (conf > max_conf) { max_conf = conf; max_id = c; }
            }
        }

        if (max_conf < CONF_THRESH || max_id < 0) continue;

        // 检查是否为目标类别
        bool is_target = false;
        for (int cls : TARGET_CLASSES) {
            if (max_id == cls) { is_target = true; break; }
        }
        if (!is_target) continue;

        // Letterbox 坐标 → 原始图像坐标
        int x1 = (int)((cx - w / 2.0f - m_pad_x) / m_scale);
        int y1 = (int)((cy - h / 2.0f - m_pad_y) / m_scale);
        int bw = (int)(w / m_scale);
        int bh = (int)(h / m_scale);

        x1 = max(0, min(x1, orig_w - 1));
        y1 = max(0, min(y1, orig_h - 1));
        bw = max(1, min(bw, orig_w - x1));
        bh = max(1, min(bh, orig_h - y1));

        if (bw * bh < MIN_OBJECT_AREA) continue;

        Detection d;
        d.class_id = max_id;
        d.confidence = max_conf;
        d.box = Rect(x1, y1, bw, bh);
        auto it = CLASS_NAMES.find(max_id);
        d.name = (it != CLASS_NAMES.end()) ? it->second : "object";
        detections.push_back(d);
    }

    // 按类别 NMS（避免不同类别互相抑制）
    vector<Detection> result;
    map<int, vector<int>> cls_map;
    for (size_t i = 0; i < detections.size(); i++) {
        cls_map[detections[i].class_id].push_back((int)i);
    }

    for (auto& kv : cls_map) {
        vector<int>& indices = kv.second;
        vector<Rect> boxes;
        vector<float> confs;
        for (int idx : indices) {
            boxes.push_back(detections[idx].box);
            confs.push_back(detections[idx].confidence);
        }
        vector<int> keep;
        dnn::NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH, keep);
        for (int k : keep) {
            result.push_back(detections[indices[k]]);
        }
    }

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

        // 动态获取输出维度
        auto& out_tensor = output_tensor;
        auto out_info = out_tensor.GetTensorTypeAndShapeInfo();
        auto out_shape = out_info.GetShape();

        int num_anchors = 2100, num_classes = 80, layout = 1;
        if (out_shape.size() == 3) {
            int dim1 = (int)out_shape[1];
            int dim2 = (int)out_shape[2];
            if (dim1 > dim2 && dim2 > 0) {
                // [1, 84, N]
                num_classes = dim1 - 4;
                num_anchors = dim2;
                layout = 0;
            } else if (dim1 > 0) {
                // [1, N, 84]
                num_anchors = dim1;
                num_classes = dim2 - 4;
                layout = 1;
            }
        }

        float* out_ptr = out_tensor.GetTensorMutableData<float>();
        return postprocess(out_ptr, frame.cols, frame.rows, num_anchors, num_classes, layout);
    } catch (...) {
        cerr << "[WARN] 推理异常" << endl;
        return {};
    }
}
