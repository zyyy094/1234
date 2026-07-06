#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <chrono>

// 硬件GPIO配置
#define FLAME_CHIP   2
#define FLAME_LINE   10

// YOLO参数配置
const int INPUT_W = 320;
const int INPUT_H = 320;
const float CONF_THRESH = 0.3f;
const float NMS_THRESH = 0.45f;
const std::vector<int> TARGET_CLASSES = {56, 60, 57, 58, 24, 28, 13};
extern std::map<int, std::string> class_names;

// 静止目标IoU阈值
const float IOU_STATIC_THRESH = 0.6f;

// 背景差分参数
const int MIN_OBJECT_AREA = 800;

// 状态机枚举
enum State {
    IDLE,
    WARNING,
    OCCUPIED
};

// 检测框结构体
struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
    std::string name;
};

// 工具函数声明
std::string getCurrentTimestamp();
float getIoU(const cv::Rect& a, const cv::Rect& b);
std::vector<Detection> filterStaticDetections(const std::vector<Detection>& raw_dets,
                                             std::vector<cv::Rect>& prev_boxes);

#endif