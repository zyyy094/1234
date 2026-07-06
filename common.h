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

const int INPUT_W = 320;
const int INPUT_H = 320;
const float CONF_THRESH = 0.3f;
const float NMS_THRESH = 0.45f;

const std::vector<int> TARGET_CLASSES = {56, 60, 57, 58, 24, 28, 13};

const std::map<int, std::string> CLASS_NAMES = {
    {56, "chair"}, {60, "table"}, {57, "sofa"},
    {58, "plant"}, {24, "bag"}, {28, "suitcase"}, {13, "carton"}
};

const int WARNING_DELAY_SEC = 3;
const int ALARM_DELAY_SEC = 5;
const int NEED_CONSECUTIVE_FRAMES = 3;
const int NEED_CONSECUTIVE_CLEAR = 3;
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int MIN_OBJECT_AREA = 800;
const float IOU_STATIC_THRESH = 0.6f;

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