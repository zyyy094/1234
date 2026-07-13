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

// YOLOv8 输入分辨率（320×320 速度优先）
const int INPUT_W = 320;
const int INPUT_H = 320;

// 置信度阈值（降低以检测更多物体）
const float CONF_THRESH = 0.25f;
const float NMS_THRESH = 0.45f;

// COCO 80类完整类别名
const std::map<int, std::string> CLASS_NAMES = {
    {0, "person"}, {1, "bicycle"}, {2, "car"}, {3, "motorcycle"}, {4, "airplane"},
    {5, "bus"}, {6, "train"}, {7, "truck"}, {8, "boat"}, {9, "traffic light"},
    {10, "fire hydrant"}, {11, "stop sign"}, {12, "parking meter"}, {13, "bench"},
    {14, "bird"}, {15, "cat"}, {16, "dog"}, {17, "horse"}, {18, "sheep"}, {19, "cow"},
    {20, "elephant"}, {21, "bear"}, {22, "zebra"}, {23, "giraffe"}, {24, "backpack"},
    {25, "umbrella"}, {26, "handbag"}, {27, "tie"}, {28, "suitcase"}, {29, "frisbee"},
    {30, "skis"}, {31, "snowboard"}, {32, "sports ball"}, {33, "kite"},
    {34, "baseball bat"}, {35, "baseball glove"}, {36, "skateboard"}, {37, "surfboard"},
    {38, "tennis racket"}, {39, "bottle"}, {40, "wine glass"}, {41, "cup"}, {42, "fork"},
    {43, "knife"}, {44, "spoon"}, {45, "bowl"}, {46, "banana"}, {47, "apple"},
    {48, "sandwich"}, {49, "orange"}, {50, "broccoli"}, {51, "carrot"}, {52, "hot dog"},
    {53, "pizza"}, {54, "donut"}, {55, "cake"}, {56, "chair"}, {57, "couch"},
    {58, "potted plant"}, {59, "bed"}, {60, "dining table"}, {61, "toilet"}, {62, "tv"},
    {63, "laptop"}, {64, "mouse"}, {65, "remote"}, {66, "keyboard"}, {67, "cell phone"},
    {68, "microwave"}, {69, "oven"}, {70, "toaster"}, {71, "sink"}, {72, "refrigerator"},
    {73, "book"}, {74, "clock"}, {75, "vase"}, {76, "scissors"}, {77, "teddy bear"},
    {78, "hair drier"}, {79, "toothbrush"}
};

// 检测所有COCO类别（杂物=任何不该出现在消防通道的物体）
// 排除类别: 无 - 全部检测，由用户/状态机判断
const std::vector<int> TARGET_CLASSES = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,
    20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,
    40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,
    60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79
};

const int WARNING_DELAY_SEC = 3;
const int ALARM_DELAY_SEC = 5;
const int NEED_CONSECUTIVE_FRAMES = 2;
const int NEED_CONSECUTIVE_CLEAR = 2;
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int MIN_OBJECT_AREA = 200;
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
