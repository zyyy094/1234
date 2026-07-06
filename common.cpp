#include "common.h"
#include <iomanip>
#include <sstream>
#include <ctime>
#include <algorithm>

std::map<int, std::string> class_names = {
    {56, "chair"}, {60, "table"}, {57, "sofa"},
    {58, "plant"}, {24, "bag"}, {28, "suitcase"}, {13, "bench"}
};

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    struct tm* tm_info = localtime(&now_time_t);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", tm_info);

    std::stringstream ss;
    ss << buffer << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

float getIoU(const cv::Rect& a, const cv::Rect& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int areaA = a.width * a.height;
    int areaB = b.width * b.height;
    if (areaA + areaB - inter == 0) return 0.f;
    return (float)inter / (areaA + areaB - inter);
}

std::vector<Detection> filterStaticDetections(const std::vector<Detection>& raw_dets,
                                             std::vector<cv::Rect>& prev_boxes) {
    std::vector<Detection> res;
    std::vector<cv::Rect> curr_boxes;
    for (auto& d : raw_dets) curr_boxes.push_back(d.box);

    if (prev_boxes.empty()) {
        prev_boxes = curr_boxes;
        return res;
    }

    for (size_t i = 0; i < raw_dets.size(); i++) {
        cv::Rect cur_box = curr_boxes[i];
        float max_iou = 0.f;
        for (auto& pre_box : prev_boxes) {
            float iou = getIoU(cur_box, pre_box);
            if (iou > max_iou) max_iou = iou;
        }
        if (max_iou > IOU_STATIC_THRESH) {
            res.push_back(raw_dets[i]);
        }
    }
    prev_boxes = curr_boxes;
    return res;
}