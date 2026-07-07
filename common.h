#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <chrono>

// ====================== 配置参数 ======================
#define FLAME_CHIP   2
#define FLAME_LINE   10

// YOLOv8 输入分辨率（保持 320×320 兼顾速度）
const int INPUT_W = 320;
const int INPUT_H = 320;

// 置信度阈值
const float CONF_THRESH = 0.3f;
const float NMS_THRESH = 0.45f;

// 扩充目标类别 - 涵盖消防通道常见杂物
const std::vector<int> TARGET_CLASSES = {
    56, 60, 57, 58, 59,     // chair, dining table, couch, potted plant, bed
    24, 25, 26, 28,          // backpack, umbrella, handbag, suitcase
    13,                      // bench
    39, 40, 41, 45,          // bottle, wine glass, cup, bowl
    73, 75, 77,              // book, vase, teddy bear
    32, 29, 36,              // sports ball, frisbee, skateboard
    62, 63, 67,              // tv, laptop, cell phone
};

const std::map<int, std::string> CLASS_NAMES = {
    {56, "chair"}, {60, "table"}, {57, "sofa"}, {58, "plant"}, {59, "bed"},
    {24, "backpack"}, {25, "umbrella"}, {26, "handbag"}, {28, "suitcase"},
    {13, "bench"},
    {39, "bottle"}, {40, "glass"}, {41, "cup"}, {45, "bowl"},
    {73, "book"}, {75, "vase"}, {77, "teddy"},
    {32, "ball"}, {29, "frisbee"}, {36, "skateboard"},
    {62, "tv"}, {63, "laptop"}, {67, "phone"},
};

const int WARNING_DELAY_SEC = 3;
const int ALARM_DELAY_SEC = 5;
const int NEED_CONSECUTIVE_FRAMES = 3;
const int NEED_CONSECUTIVE_CLEAR = 3;
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int MIN_OBJECT_AREA = 400;
const float IOU_STATIC_THRESH = 0.35f;

// ====================== 结构体 ======================
struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
    std::string name;
};

// ====================== 枚举 ======================
enum State { IDLE, WARNING, OCCUPIED };

// ====================== 工具函数 ======================
std::string getCurrentTimestamp();
float getIoU(const cv::Rect& a, const cv::Rect& b);

#endif
