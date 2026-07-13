/*
 * fire_detect_ncnn.cpp - 消防通道杂物占用检测【NCNN加速版】
 *
 * 与ONNX Runtime版相比的改动:
 *   1. 推理引擎: ONNX Runtime → NCNN (ARM NEON优化, 帧率提升3-5倍)
 *   2. 模型文件: best.onnx → best_opt.param + best_opt.bin
 *   3. 预处理: ncnn::Mat::from_pixels 替代手动CHW转换 (NEON加速)
 *   4. 推理: ncnn::Extractor 替代 Ort::Session.Run()
 *   5. 后处理: ncnn::Mat 输出格式自适应, 逻辑与ONNX版一致
 *   6. 编译: -lonnxruntime → -lncnn
 *
 * 其他所有逻辑(背景差分/状态机/RPMsg/火焰双链路/UI/截图)完全不变
 *
 * 架构: 飞腾派异构双核
 *   主核(Linux): 背景差分 + 全局单帧仅1次NCNN推理 + ROI交集过滤
 *                + 自适应IoU静态跟踪 + 双链路火焰校验 + RPMsg指令下发
 *   从核(裸机): 三色LED + 蜂鸣器声光报警驱动 + 火焰传感器上报
 */
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "ncnn/net.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <gpiod.h>
#include <csignal>
#include <iomanip>
#include <sstream>
#include <poll.h>
#include <fstream>
#include <algorithm>
#include <sys/stat.h>

using namespace cv;
using namespace std;
using namespace std::chrono;

// ====================== 硬件定义 ======================
#define FLAME_CHIP 2
#define FLAME_LINE 10

// ====================== RPMsg 协议（主→从）======================
#define CMD_STATE_IDLE      'G'
#define CMD_STATE_WARNING   'Y'
#define CMD_STATE_OCCUPIED  'R'
#define CMD_STATE_FIRE      'F'
#define CMD_EMERGENCY_STOP  'S'
#define CMD_HEARTBEAT       'H'
#define CMD_HEARTBEAT_ACK   'h'
#define CMD_SLAVE_FIRE_ALERT  'F'
#define CMD_SLAVE_FIRE_CLEAR  'f'
#define HEARTBEAT_INTERVAL_SEC   3.0
#define HEARTBEAT_TIMEOUT_SEC    10.0

#define PICTURE_DIR "pictures"

// ====================== 全局变量 ======================
int rpmsg_fd = -1;
bool running = true;

static struct gpiod_chip* flame_chip = nullptr;
static struct gpiod_line* flame_line = nullptr;

static bool slave_flame_alert = false;
static bool slave_alive = false;
static steady_clock::time_point last_heartbeat_send;
static steady_clock::time_point last_heartbeat_ack;

// NCNN预处理参数 (letterbox缩放后传给ncnn::Mat)
static float pre_scale = 1.0f;
static int pre_pad_x = 0, pre_pad_y = 0;

// NCNN模型全局实例
static ncnn::Net yolo_net;
static string g_input_name = "images";
static string g_output_name = "output0";
static char g_current_state_cmd = CMD_STATE_IDLE;

// ====================== 状态枚举 ======================
enum State {
    STATE_FREE = 0,
    STATE_WARNING,
    STATE_OCCUPIED,
    STATE_FIRE_ALERT
};

const char* getStateText(State s) {
    switch(s) {
        case STATE_FREE:      return "FREE";
        case STATE_WARNING:    return "WARNING";
        case STATE_OCCUPIED:  return "OCCUPIED";
        case STATE_FIRE_ALERT:return "FIRE ALERT!";
        default:              return "UNKNOWN";
    }
}

const char* cmd_to_state_name(char cmd) {
    switch(cmd) {
        case CMD_STATE_IDLE:       return "FREE";
        case CMD_STATE_WARNING:    return "WARNING";
        case CMD_STATE_OCCUPIED:   return "OCCUPIED";
        case CMD_STATE_FIRE:       return "FIRE ALERT!";
        case CMD_EMERGENCY_STOP:   return "EMERGENCY STOP";
        case CMD_HEARTBEAT:        return "HEARTBEAT";
        case CMD_HEARTBEAT_ACK:    return "HEARTBEAT ACK";
        default:                   return "UNKNOWN";
    }
}

Scalar getStateColor(State s) {
    switch(s) {
        case STATE_FREE:      return Scalar(0, 255, 0);
        case STATE_WARNING:    return Scalar(0, 255, 255);
        case STATE_OCCUPIED:  return Scalar(0, 0, 255);
        case STATE_FIRE_ALERT:return Scalar(0, 0, 255);
        default:              return Scalar(255, 255, 255);
    }
}

// ====================== 工具函数 ======================
bool make_dir(const string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) == 0)
        return (info.st_mode & S_IFDIR) != 0;
    if (mkdir(path.c_str(), 0755) == 0)
        return true;
    return false;
}

string getCurrentTimestamp() {
    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    struct tm* tm_info = localtime(&now_time_t);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", tm_info);
    stringstream ss;
    ss << buffer << "_" << setfill('0') << setw(3) << ms.count();
    return ss.str();
}

void save_screenshot_with_info(Mat& frame, const string& state) {
    make_dir(PICTURE_DIR);
    Mat img_with_info = frame.clone();
    string state_text = "State: " + state;
    string time_text = "Time: " + getCurrentTimestamp();
    string alert_text = (state == "OCCUPIED") ? "Alert: OCCUPIED - Channel Blocked" : "";

    int info_height = 80;
    Mat info_bar(img_with_info.rows + info_height, img_with_info.cols,
                 img_with_info.type(), Scalar(0, 0, 0));
    img_with_info.copyTo(info_bar(Rect(0, info_height, img_with_info.cols, img_with_info.rows)));

    putText(info_bar, state_text, Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
    putText(info_bar, time_text, Point(10, 55), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    if (!alert_text.empty()) {
        putText(info_bar, alert_text, Point(info_bar.cols - 320, 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    }

    string filename = string(PICTURE_DIR) + "/" + state + "_" + getCurrentTimestamp() + ".jpg";
    imwrite(filename, info_bar);
    cout << "[INFO] 截图已保存: " << filename << endl;
}

// ====================== IoU 计算 ======================
float getIoU(const Rect& a, const Rect& b) {
    int x1 = max(a.x, b.x);
    int y1 = max(a.y, b.y);
    int x2 = min(a.x + a.width,  b.x + b.width);
    int y2 = min(a.y + a.height, b.y + b.height);
    int inter = max(0, x2 - x1) * max(0, y2 - y1);
    int areaA = a.width * a.height;
    int areaB = b.width * b.height;
    if (areaA + areaB - inter == 0) return 0.f;
    return (float)inter / (areaA + areaB - inter);
}

// 自适应IoU阈值: 小物体宽松, 大物体严格
float getAdaptiveIouThresh(const Rect& box) {
    int area = box.area();
    if (area < 800)     return 0.15f;
    else if (area < 3000) return 0.25f;
    else                return 0.40f;
}

vector<Rect> lastStaticBoxes;

// ====================== 火焰传感器 ======================
bool init_flame_sensor() {
    flame_chip = gpiod_chip_open_by_number(FLAME_CHIP);
    if (!flame_chip) { cerr << "[WARN] 火焰传感器GPIO芯片打开失败" << endl; return false; }
    flame_line = gpiod_chip_get_line(flame_chip, FLAME_LINE);
    if (!flame_line) { gpiod_chip_close(flame_chip); flame_chip = nullptr; return false; }
    if (gpiod_line_request_input(flame_line, "flame_sensor") < 0) {
        gpiod_line_release(flame_line); gpiod_chip_close(flame_chip);
        flame_chip = nullptr; flame_line = nullptr; return false;
    }
    cout << "[INFO] 火焰传感器初始化成功(gpiochip" << FLAME_CHIP << " line" << FLAME_LINE << ")" << endl;
    return true;
}
void release_flame_sensor() {
    if (flame_line) { gpiod_line_release(flame_line); flame_line = nullptr; }
    if (flame_chip) { gpiod_chip_close(flame_chip); flame_chip = nullptr; }
}
bool flame_detected() {
    if (!flame_line) return false;
    return (gpiod_line_get_value(flame_line) == 0);
}

// ====================== RPMsg 控制 ======================
static char last_sent_state = 0;
void send_state(char state_cmd, bool force = false) {
    if (rpmsg_fd >= 0 && (force || state_cmd != last_sent_state)) {
        write(rpmsg_fd, &state_cmd, 1);
        usleep(1000);
        last_sent_state = state_cmd;
        g_current_state_cmd = state_cmd;
        cout << "[RPMsg] 发送状态: " << state_cmd << " (" << cmd_to_state_name(state_cmd) << ")" << endl;
    }
}
void shutdown_all_hardware() {
    if (rpmsg_fd >= 0) {
        last_sent_state = 0;
        char stop = CMD_EMERGENCY_STOP;
        write(rpmsg_fd, &stop, 1);
        usleep(100000);
    }
}

// RPMsg轮询节流: 200ms查询一次
void poll_slave_messages() {
    static steady_clock::time_point last_poll_tm;
    if (duration<double>(steady_clock::now() - last_poll_tm).count() < 0.2)
        return;
    last_poll_tm = steady_clock::now();

    if (rpmsg_fd < 0) return;
    struct pollfd pfd; pfd.fd = rpmsg_fd; pfd.events = POLLIN;
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        char buf[64]; ssize_t n = read(rpmsg_fd, buf, sizeof(buf));
        if (n <= 0) break;
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == CMD_SLAVE_FIRE_ALERT) {
                if (!slave_flame_alert) cout << "[SLAVE] 从核上报火焰警报!" << endl;
                slave_flame_alert = true;
            } else if (c == CMD_SLAVE_FIRE_CLEAR) {
                if (slave_flame_alert) cout << "[SLAVE] 从核火焰清除" << endl;
                slave_flame_alert = false;
            } else if (c == CMD_HEARTBEAT_ACK) {
                if (!slave_alive) {
                    cout << "[SLAVE] 从核恢复在线，补发当前状态" << endl;
                    slave_alive = true;
                    send_state(g_current_state_cmd, true);
                } else {
                    slave_alive = true;
                }
                last_heartbeat_ack = steady_clock::now();
            }
        }
    }
}

void send_heartbeat() {
    if (rpmsg_fd >= 0) { char hb = CMD_HEARTBEAT; write(rpmsg_fd, &hb, 1); usleep(1000); }
    last_heartbeat_send = steady_clock::now();
}
void update_heartbeat() {
    double since_ack = duration<double>(steady_clock::now() - last_heartbeat_ack).count();
    if (since_ack > HEARTBEAT_TIMEOUT_SEC) {
        if (slave_alive) { cerr << "[WARN] 从核心跳超时，判定离线" << endl; slave_alive = false; }
    }
    double since_send = duration<double>(steady_clock::now() - last_heartbeat_send).count();
    if (since_send >= HEARTBEAT_INTERVAL_SEC) send_heartbeat();
}
bool is_slave_flame() { return slave_flame_alert; }

void signal_handler(int sig) {
    (void)sig;
    running = false;
    shutdown_all_hardware();
    if (rpmsg_fd >= 0) { close(rpmsg_fd); rpmsg_fd = -1; }
    release_flame_sensor();
    destroyAllWindows();
    cout << "\n[EXIT] 捕获退出信号，资源全部释放完毕\n";
}

// ====================== YOLO 参数 ======================
// best_opt.ncnn 模型固定320x320输入
const int INPUT_W = 320;
const int INPUT_H = 320;
const float CONF_THRESH = 0.2f;
const float NMS_THRESH  = 0.45f;

const vector<int> TARGET_CLASSES = {0, 1, 2};
map<int, string> class_names = { {0, "box"}, {1, "vehicle"}, {2, "furniture"} };

struct Detection { int class_id; float confidence; Rect box; string name; };

// ====================== NCNN 预处理 ======================
// letterbox: 保持宽高比缩放到320x320, 周围填充灰色(114,114,114)
// 然后用ncnn::Mat::from_pixels完成BGR→RGB+CHW排布 (内部NEON加速)
ncnn::Mat preprocess_ncnn(const Mat& frame) {
    float scale = min((float)INPUT_W / frame.cols, (float)INPUT_H / frame.rows);
    int new_w = (int)(frame.cols * scale);
    int new_h = (int)(frame.rows * scale);
    Mat resized;
    resize(frame, resized, Size(new_w, new_h), 0, 0, INTER_LINEAR);
    Mat padded(INPUT_H, INPUT_W, CV_8UC3, Scalar(114, 114, 114));
    int pad_x = (INPUT_W - new_w) / 2;
    int pad_y = (INPUT_H - new_h) / 2;
    resized.copyTo(padded(Rect(pad_x, pad_y, new_w, new_h)));
    // 保存逆变换参数供postprocess还原坐标
    pre_scale = scale; pre_pad_x = pad_x; pre_pad_y = pad_y;

    // ncnn::Mat::from_pixels: BGR→RGB + HWC→CHW, 内部ARM NEON优化
    // PIXEL_BGR2RGB: 输入是OpenCV的BGR, 自动转RGB
    ncnn::Mat input_mat = ncnn::Mat::from_pixels(padded.data, ncnn::Mat::PIXEL_BGR2RGB, INPUT_W, INPUT_H);

    // 归一化: 0~255 → 0~1
    // substract_mean_normalize(mean, norm): output = (input - mean) * norm
    const float norm_vals[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    input_mat.substract_mean_normalize(nullptr, norm_vals);

    return input_mat;
}

// ====================== NCNN 后处理 ======================
// YOLOv8 NCNN输出: ncnn::Mat, 数据连续存储
// NCNN Mat内存布局: data[(c * h + y) * w + x]
//
// 模型输出可能是:
//   [1, 7, 2100] → ncnn::Mat: c=7, h=2100, w=1 → data[c*2100 + i] (通道优先)
//   [1, 2100, 7] → ncnn::Mat: c=1, h=2100, w=7 → data[i*7 + j] (锚点优先)
//
// 自动检测格式并按正确方式索引
vector<Detection> postprocess_ncnn(const ncnn::Mat& out, int img_w, int img_h) {
    int num_classes = 3;
    int stride = 4 + num_classes;  // 7

    // 从ncnn::Mat的维度判断输出格式
    // ncnn::Mat: c=channels, h=height, w=width
    int mc = out.c;   // 通道数
    int mh = out.h;   // 高
    int mw = out.w;   // 宽

    bool channel_first = false;  // true = [7, 2100]格式
    int num_dets = 0;

    if (mc == stride && mh * mw > stride) {
        // c=7, h*mw=2100 → 通道优先 [7, 2100]
        channel_first = true;
        num_dets = mh * mw;
    } else if (mw == stride && mc * mh > stride) {
        // w=7, c*mh=2100 → 锚点优先 [2100, 7]
        channel_first = false;
        num_dets = mc * mh;
    } else {
        // 兜底推断
        int total = mc * mh * mw;
        num_dets = total / stride;
        channel_first = (mc == stride);
        if (num_dets <= 0) num_dets = 2100;
    }

    static bool format_printed = false;
    if (!format_printed) {
        printf("[NCNN] 输出Mat: c=%d h=%d w=%d → %s, num_dets=%d, stride=%d\n",
               mc, mh, mw, channel_first ? "通道优先[7,N]" : "锚点优先[N,7]", num_dets, stride);
        format_printed = true;
    }

    const float* data = (const float*)out.data;
    vector<Detection> detections;

    for (int i = 0; i < num_dets; ++i) {
        float cx, cy, w, h;
        float max_conf = 0; int max_id = 0;

        if (channel_first) {
            // 通道优先: data[c * num_dets + i]
            cx = data[0 * num_dets + i];
            cy = data[1 * num_dets + i];
            w  = data[2 * num_dets + i];
            h  = data[3 * num_dets + i];
            for (int c = 0; c < num_classes; ++c) {
                float conf = data[(4 + c) * num_dets + i];
                if (conf > max_conf) { max_conf = conf; max_id = c; }
            }
        } else {
            // 锚点优先: data[i * stride + c]
            const float* p = data + i * stride;
            cx = p[0]; cy = p[1]; w = p[2]; h = p[3];
            for (int c = 0; c < num_classes; ++c) {
                float conf = p[4 + c];
                if (conf > max_conf) { max_conf = conf; max_id = c; }
            }
        }

        if (max_conf < CONF_THRESH) continue;
        bool is_target = false;
        for (int cls : TARGET_CLASSES) if (max_id == cls) { is_target = true; break; }
        if (!is_target) continue;

        // 坐标还原: 320x320输入空间 → 原图空间
        float scale = pre_scale;
        int px = pre_pad_x, py = pre_pad_y;
        int x = max(0, (int)((cx - w / 2 - px) / scale));
        int y = max(0, (int)((cy - h / 2 - py) / scale));
        int width  = min((int)(w / scale), img_w - x);
        int height = min((int)(h / scale), img_h - y);

        if (width > 0 && height > 0) {
            float box_area = (float)(width * height);
            float frame_area = (float)(img_w * img_h);
            if (box_area < 800.f) continue;           // 过滤过小噪声框
            if (box_area > frame_area * 0.5f) continue; // 过滤异常大框
            Detection d;
            d.class_id   = max_id;
            d.confidence = max_conf;
            d.box        = Rect(x, y, width, height);
            d.name       = class_names.count(max_id) ? class_names[max_id] : "object";
            detections.push_back(d);
        }
    }

    if (detections.empty()) return {};

    // NMS非极大值抑制
    vector<int> indices;
    vector<Rect> boxes; vector<float> confs;
    for (auto& d : detections) { boxes.push_back(d.box); confs.push_back(d.confidence); }
    dnn::NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH, indices);
    vector<Detection> result;
    for (int idx : indices) result.push_back(detections[idx]);
    return result;
}

// ====================== 背景差分 ======================
Mat background;
bool bg_ready = false;
bool detection_started = false;
int g_det_count = 0;
const int MIN_OBJECT_AREA = 300;
const int MAX_ROIS = 5;

int get_adaptive_threshold(const Mat& bg, const Mat& cur) {
    Scalar bg_mean = mean(bg);
    Scalar cur_mean = mean(cur);
    double diff = abs(bg_mean[0] - cur_mean[0]);
    int thresh = 30 + (int)(diff * 0.3);
    return max(20, min(thresh, 60));
}

vector<Rect> get_diff_rois(const Mat& frame) {
    vector<Rect> rois;
    if (!bg_ready) return rois;
    Mat gray, diff, thresh;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    Mat bg_blur, gray_blur;
    GaussianBlur(gray, gray_blur, Size(5, 5), 0);
    GaussianBlur(background, bg_blur, Size(5, 5), 0);
    absdiff(bg_blur, gray_blur, diff);

    int thresh_val = get_adaptive_threshold(bg_blur, gray_blur);
    threshold(diff, thresh, thresh_val, 255, THRESH_BINARY);

    // 形态学: 仅开运算(3x3核), 删除闭运算减少计算量
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(thresh, thresh, MORPH_OPEN, kernel);

    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<pair<double, Rect>> candidates;
    for (auto& cnt : contours) {
        Rect r = boundingRect(cnt);
        double area = r.area();
        float aspect_ratio = (float)r.width / max(1, r.height);
        if (area > MIN_OBJECT_AREA && aspect_ratio > 0.3 && aspect_ratio < 3.0) {
            candidates.push_back({area, r});
        }
    }
    sort(candidates.begin(), candidates.end(),
         [](const auto& a, const auto& b) { return a.first > b.first; });
    for (int i = 0; i < min((int)candidates.size(), MAX_ROIS); i++) {
        Rect r = candidates[i].second;
        // ROI向外扩充20像素给YOLO更多上下文
        r.x = max(0, r.x - 20);
        r.y = max(0, r.y - 20);
        r.width  = min(frame.cols - r.x, r.width + 40);
        r.height = min(frame.rows - r.y, r.height + 40);
        rois.push_back(r);
    }
    return rois;
}

// ====================== 界面绘制 ======================
void draw_overlay(Mat& frame, State s, int fps, bool emergency, double stable_seconds) {
    int right_x = frame.cols - 180;
    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "FPS: %d", fps);
    putText(frame, fps_str, Point(right_x, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

    if (s != STATE_FREE && !emergency) {
        char timer_str[64];
        snprintf(timer_str, sizeof(timer_str), "Timer: %.1f s", stable_seconds);
        putText(frame, timer_str, Point(right_x, 60), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(200, 200, 200), 1);
    }
    if (s == STATE_WARNING && !emergency) {
        char countdown_str[32];
        int remaining = max(0, 8 - (int)stable_seconds);
        snprintf(countdown_str, sizeof(countdown_str), "countdown: %ds", remaining);
        putText(frame, countdown_str, Point(right_x, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    }

    time_t now = time(nullptr);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
    putText(frame, time_str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

    const char* state_text = getStateText(s);
    Scalar state_color = getStateColor(s);
    putText(frame, state_text, Point(10, 70), FONT_HERSHEY_SIMPLEX, 1.2, state_color, 3);

    int y3 = 100;
    if (emergency || s == STATE_FIRE_ALERT) {
        putText(frame, "!!! FLAME DETECTED !!!", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        y3 += 30;
        putText(frame, "BUZZER: CONTINUOUS", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        y3 += 30;
        putText(frame, "TAKE IMMEDIATE ACTION!", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    } else if (s == STATE_WARNING) {
        putText(frame, "Object detected, waiting...", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        y3 += 30;
        putText(frame, "BUZZER: OFF", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
        y3 += 30;
        char info[64];
        snprintf(info, sizeof(info), "Will alarm in %d s", max(0, 8 - (int)stable_seconds));
        putText(frame, info, Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    } else if (s == STATE_OCCUPIED) {
        putText(frame, "!!! CHANNEL OCCUPIED !!!", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        y3 += 30;
        putText(frame, "BUZZER: INTERMITTENT", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        y3 += 30;
        putText(frame, "PLEASE CLEAR THE AREA", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    } else {
        if (!bg_ready) {
            putText(frame, "Press 's' to save background", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
            y3 += 30;
        } else if (!detection_started) {
            putText(frame, "Press 'c' to start detection", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
            y3 += 30;
        } else {
            putText(frame, "Channel is FREE", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
            y3 += 30;
            putText(frame, "BUZZER: OFF", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
        }
    }

    char slave_str[64];
    snprintf(slave_str, sizeof(slave_str), "Slave: %s", slave_alive ? "ONLINE" : "OFFLINE");
    putText(frame, slave_str, Point(right_x, 120), FONT_HERSHEY_SIMPLEX, 0.4,
           slave_alive ? Scalar(0, 200, 0) : Scalar(0, 0, 200), 1);

    char led_str[64];
    const char* led_color = "GREEN";
    if (s == STATE_FIRE_ALERT || s == STATE_OCCUPIED) led_color = "RED";
    else if (s == STATE_WARNING) led_color = "YELLOW";
    snprintf(led_str, sizeof(led_str), "LED: %s", led_color);
    putText(frame, led_str, Point(right_x, 145), FONT_HERSHEY_SIMPLEX, 0.4, getStateColor(s), 1);
}

// ====================== 主函数 ======================
int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    last_heartbeat_send = steady_clock::now();
    last_heartbeat_ack  = steady_clock::now();
    make_dir(PICTURE_DIR);

    // ---- 打开RPMsg设备 ----
    rpmsg_fd = -1;
    for (int dev = 0; dev < 8; dev++) {
        string dev_path = "/dev/rpmsg" + to_string(dev);
        rpmsg_fd = open(dev_path.c_str(), O_RDWR | O_NONBLOCK);
        if (rpmsg_fd >= 0) {
            cout << "[INFO] 成功打开RPMsg设备: " << dev_path << endl;
            break;
        }
    }
    if (rpmsg_fd < 0) {
        cerr << "[WARN] 无法打开 rpmsg 字符设备，RPMsg通信禁用" << endl;
    } else {
        slave_alive = true;
        send_state(CMD_STATE_IDLE, true);
    }

    init_flame_sensor();

    // ---- 加载NCNN模型 ----
    // best_opt.param + best_opt.bin 是 onnx2ncnn 转换并优化后的模型
    yolo_net.opt.use_vulkan_compute = false;  // 飞腾派无GPU, 使用CPU推理
    yolo_net.opt.num_threads = 4;              // 4线程并行 (飞腾派4核)
    yolo_net.opt.use_winograd_convolution = true;  // Winograd卷积加速
    yolo_net.opt.lightmode = true;             // 轻量模式, 减少内存占用

    int ret = yolo_net.load_param("best_opt.param");
    if (ret != 0) {
        cerr << "[ERROR] 加载 best_opt.param 失败 (ret=" << ret << ")" << endl;
        shutdown_all_hardware();
        if (rpmsg_fd >= 0) close(rpmsg_fd);
        release_flame_sensor();
        return -1;
    }
    ret = yolo_net.load_model("best_opt.bin");
    if (ret != 0) {
        cerr << "[ERROR] 加载 best_opt.bin 失败 (ret=" << ret << ")" << endl;
        shutdown_all_hardware();
        if (rpmsg_fd >= 0) close(rpmsg_fd);
        release_flame_sensor();
        return -1;
    }
    cout << "[INFO] NCNN 模型加载成功 (best_opt.param + best_opt.bin)" << endl;
    cout << "[INFO] 推理尺寸: " << INPUT_W << "x" << INPUT_H << endl;
    cout << "[INFO] 输入节点: " << g_input_name << " | 输出节点: " << g_output_name << endl;

    // ---- 打开摄像头 ----
    VideoCapture cap;
    for (int i = 0; i <= 4; i++) {
        cap.open(i, CAP_V4L2);
        if (cap.isOpened()) { cout << "[INFO] 打开摄像头 /dev/video" << i << endl; break; }
    }
    if (!cap.isOpened()) {
        cerr << "[ERROR] 摄像头打开失败" << endl;
        shutdown_all_hardware();
        if (rpmsg_fd >= 0) close(rpmsg_fd);
        release_flame_sensor();
        return -1;
    }
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);

    cout << "\n========== 消防通道检测系统【NCNN加速版】 ==========" << endl;
    cout << "推理引擎: NCNN (ARM NEON优化, " << yolo_net.opt.num_threads << "线程)" << endl;
    cout << "推理尺寸: " << INPUT_W << "x" << INPUT_H << endl;
    cout << "自适应IoU跟踪 | ROI最大5个 | 最小检测面积300 | 置信度阈值0.2" << endl;
    cout << "状态逻辑：静置3秒进入预警，累计8秒触发占用报警" << endl;
    cout << "按键操作：s保存背景 | c开始检测 | q退出程序" << endl;
    cout << "【自动启动】: 3秒自动保存背景，5秒自动开始检测" << endl;
    cout << "=====================================================\n" << endl;

    steady_clock::time_point last_status_write = steady_clock::now();
    steady_clock::time_point last_stream_write = steady_clock::now();
    steady_clock::time_point program_start   = steady_clock::now();
    float fps_smooth = 0.0f;
    const float FPS_ALPHA = 0.15f;

    Mat frame;
    for (int i = 0; i < 20; i++) { cap >> frame; if (frame.empty()) break; waitKey(1); }

    // 自动启动变量
    steady_clock::time_point prog_start = steady_clock::now();
    bool auto_bg_saved = false;
    bool auto_detect_started = false;

    State current_state = STATE_FREE;
    bool object_present = false;
    bool screenshot_taken = false;
    int photo_count = 0;
    bool first_frame_done = false;
    int consecutive_detection_count = 0;
    int consecutive_no_detection_count = 0;
    const int NEED_CONSECUTIVE_FRAMES = 3;
    const int NEED_CONSECUTIVE_CLEAR  = 3;
    steady_clock::time_point object_stable_start;

    int show_frame_cnt = 0;
    steady_clock::time_point last_flame_read_tm;

    // ==================== 主循环 ====================
    while (running) {
        auto start = steady_clock::now();
        cap >> frame;
        if (frame.empty()) break;

        // ---- 按键处理 ----
        char key = waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q') break;
        else if (key == 's' || key == 'S') {
            background = frame.clone();
            cvtColor(background, background, COLOR_BGR2GRAY);
            bg_ready = true;
            first_frame_done = false;
            lastStaticBoxes.clear();
            consecutive_detection_count = 0;
            consecutive_no_detection_count = 0;
            object_present = false;
            current_state = STATE_FREE;
            detection_started = false;
            auto_bg_saved = true;
            auto_detect_started = false;
            prog_start = steady_clock::now();
            send_state(CMD_STATE_IDLE, true);
            cout << "[INFO] 背景已重新保存！按 c 开始检测，或等待5秒自动开始" << endl;
        }
        else if (key == 'c' || key == 'C') {
            if (bg_ready) {
                detection_started = true;
                auto_detect_started = true;
                current_state = STATE_FREE;
                send_state(CMD_STATE_IDLE, true);
                cout << "[INFO] 手动启动检测，初始空闲状态" << endl;
            }
        }

        // ---- 自动启动 ----
        double elapsed = duration<double>(steady_clock::now() - prog_start).count();
        if (!auto_bg_saved && elapsed >= 3.0 && !bg_ready) {
            background = frame.clone();
            cvtColor(background, background, COLOR_BGR2GRAY);
            bg_ready = true;
            first_frame_done = false;
            lastStaticBoxes.clear();
            consecutive_detection_count = 0;
            consecutive_no_detection_count = 0;
            object_present = false;
            current_state = STATE_FREE;
            auto_bg_saved = true;
            send_state(CMD_STATE_IDLE, true);
            cout << "[AUTO] 3秒自动保存背景完成" << endl;
        }
        if (auto_bg_saved && !auto_detect_started && elapsed >= 5.0 && !detection_started) {
            detection_started = true;
            auto_detect_started = true;
            current_state = STATE_FREE;
            send_state(CMD_STATE_IDLE, true);
            cout << "[AUTO] 5秒自动开始检测" << endl;
        }

        // ---- 心跳 + 从核消息轮询 (200ms节流) ----
        update_heartbeat();
        poll_slave_messages();

        // ---- 火焰双链路检测 (200ms节流) ----
        bool flame_local = false;
        if (duration<double>(steady_clock::now() - last_flame_read_tm).count() >= 0.2) {
            flame_local = flame_detected();
            last_flame_read_tm = steady_clock::now();
        }
        bool flame_slave = is_slave_flame();
        bool fire = flame_local || flame_slave;

        if (fire) {
            if (current_state != STATE_FIRE_ALERT) {
                current_state = STATE_FIRE_ALERT;
                send_state(CMD_STATE_FIRE);
                cout << "[FIRE ALERT] 火焰警报触发 | ";
                if (flame_local && flame_slave) cout << "主从双传感器同时检测";
                else if (flame_slave) cout << "从机火焰上报";
                else cout << "本地GPIO火焰传感器";
                cout << endl;
                string snap = string(PICTURE_DIR) + "/fire_" + getCurrentTimestamp() + ".jpg";
                imwrite(snap, frame);
                photo_count++;
            }

            double frame_time = duration<double>(steady_clock::now() - start).count();
            int fps_value = (frame_time > 0.001) ? (int)(1.0 / frame_time) : 30;
            draw_overlay(frame, current_state, fps_value, true, 0);

            if (show_frame_cnt % 3 == 0)
                imshow("Fire Lane Detection", frame);
            show_frame_cnt++;

            if (duration<double>(steady_clock::now() - last_status_write).count() >= 1.0) {
                double uptime = duration<double>(steady_clock::now() - program_start).count();
                time_t nw = time(nullptr); char ts[64];
                strftime(ts, sizeof(ts), "%H:%M:%S", localtime(&nw));
                ofstream sf("status.json");
                sf << "{";
                sf << "\"state\":\"FIRE\",";
                sf << "\"state_color\":\"#ff1744\",";
                sf << "\"fps\":" << fps_value << ",";
                sf << "\"photo_count\":" << photo_count << ",";
                sf << "\"detections\":" << g_det_count << ",";
                sf << "\"flame_detected\":true,";
                sf << "\"slave_alive\":" << (slave_alive ? "true" : "false") << ",";
                sf << "\"uptime\":" << (int)uptime << ",";
                sf << "\"timestamp\":\"" << ts << "\"";
                sf << "}";
                sf.close();
                last_status_write = steady_clock::now();
            }
            continue;
        } else {
            if (current_state == STATE_FIRE_ALERT) {
                current_state = STATE_FREE;
                send_state(CMD_STATE_IDLE, true);
                object_present = false;
                screenshot_taken = false;
                consecutive_detection_count = 0;
                consecutive_no_detection_count = 0;
                first_frame_done = false;
                lastStaticBoxes.clear();
                cout << "[FIRE] 火焰消失，系统恢复空闲状态" << endl;
            }
        }

        // ---- 背景差分找变化ROI ----
        vector<Rect> diff_rois;
        if (detection_started && bg_ready) {
            diff_rois = get_diff_rois(frame);
        }

        // ---- NCNN 全局推理 (每帧仅1次) ----
        vector<Detection> raw_detections;
        if (detection_started && bg_ready) {
            // 1. 预处理: letterbox + ncnn::Mat::from_pixels (NEON加速)
            ncnn::Mat input_mat = preprocess_ncnn(frame);

            // 2. 推理: 创建extractor → input → extract
            ncnn::Extractor ex = yolo_net.create_extractor();
            ex.set_num_threads(4);
            ex.input(g_input_name.c_str(), input_mat);

            ncnn::Mat output_mat;
            ret = ex.extract(g_output_name.c_str(), output_mat);
            if (ret != 0) {
                cerr << "[ERROR] NCNN推理失败 (ret=" << ret << ")" << endl;
            } else {
                // 3. 后处理: 解析输出 → 置信度过滤 → 类别过滤 → NMS
                raw_detections = postprocess_ncnn(output_mat, frame.cols, frame.rows);
            }

            // 4. ROI软过滤: 检测框需与背景差分变化区域有>=5%重叠才保留
            if (!diff_rois.empty()) {
                vector<Detection> filter_dets;
                for (auto& det : raw_detections) {
                    bool intersect_ok = false;
                    for (auto& roi : diff_rois) {
                        Rect cross = det.box & roi;
                        if (cross.area() > det.box.area() * 0.05) {
                            intersect_ok = true;
                            break;
                        }
                    }
                    if (intersect_ok) filter_dets.push_back(det);
                }
                raw_detections.swap(filter_dets);
            }
            g_det_count = (int)raw_detections.size();
        }

        // ---- 自适应IoU静态跟踪 ----
        vector<Detection> final_detections;
        vector<Rect> curBoxes;
        for (auto& d : raw_detections) curBoxes.push_back(d.box);

        if (!first_frame_done) {
            final_detections = raw_detections;
            lastStaticBoxes = curBoxes;
            first_frame_done = true;
        } else {
            for (size_t i = 0; i < raw_detections.size(); i++) {
                Rect b = raw_detections[i].box;
                bool isStatic = false;
                float adaptive_th = getAdaptiveIouThresh(b);
                for (auto& lb : lastStaticBoxes) {
                    if (getIoU(b, lb) > adaptive_th) {
                        isStatic = true;
                        break;
                    }
                }
                if (isStatic) final_detections.push_back(raw_detections[i]);
            }
            lastStaticBoxes = curBoxes;
        }

        // ---- 绘制检测框: 绿色粗框 + 标签 ----
        for (auto& d : raw_detections) {
            rectangle(frame, d.box, Scalar(0, 255, 0), 4);
            char label[64];
            snprintf(label, sizeof(label), "%s %.3f", d.name.c_str(), d.confidence);
            int text_y = max(12, d.box.y - 12);
            putText(frame, label, Point(d.box.x, text_y), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        }

        // 画蓝色ROI框 (背景差分区域, 与绿色检测框区分)
        for (auto& roi : diff_rois) {
            rectangle(frame, roi, Scalar(255, 0, 0), 1, LINE_AA);
        }

        // ---- 防抖动 + 状态机 ----
        Detection best_detection;
        bool has_best = false;
        for (auto& d : final_detections) {
            if (!has_best || d.box.area() > best_detection.box.area()) {
                best_detection = d;
                has_best = true;
            }
        }

        bool current_raw = has_best;
        if (current_raw) {
            consecutive_detection_count++;
            consecutive_no_detection_count = 0;
        } else {
            consecutive_no_detection_count++;
            consecutive_detection_count = 0;
        }

        if (consecutive_detection_count >= NEED_CONSECUTIVE_FRAMES && !object_present) {
            object_present = true;
            object_stable_start = steady_clock::now();
            screenshot_taken = false;
            cout << "[INFO] 锁定静置杂物：" << best_detection.name << " 置信度：" << best_detection.confidence << endl;
        } else if (consecutive_no_detection_count >= NEED_CONSECUTIVE_CLEAR && object_present) {
            object_present = false;
            screenshot_taken = false;
            if (current_state != STATE_FREE) {
                current_state = STATE_FREE;
                send_state(CMD_STATE_IDLE, true);
                cout << "[STATE] 障碍物已移除，通道恢复空闲" << endl;
            }
        }

        double stable_seconds = 0;
        if (object_present) {
            stable_seconds = duration<double>(steady_clock::now() - object_stable_start).count();
            if (current_state == STATE_FREE && stable_seconds >= 3.0) {
                current_state = STATE_WARNING;
                send_state(CMD_STATE_WARNING);
                cout << "[STATE] 物体静置" << stable_seconds << "s，进入黄灯预警" << endl;
            } else if (current_state == STATE_WARNING && stable_seconds >= 8.0) {
                current_state = STATE_OCCUPIED;
                send_state(CMD_STATE_OCCUPIED);
                cout << "[STATE] 占用超时，触发红灯间歇蜂鸣报警" << endl;
                if (!screenshot_taken) {
                    save_screenshot_with_info(frame, "OCCUPIED");
                    photo_count++;
                    screenshot_taken = true;
                }
            }
        }

        // ---- FPS计算 ----
        auto end = steady_clock::now();
        double frame_time = duration<double>(end - start).count();
        float inst_fps = (frame_time > 0.001) ? (float)(1.0 / frame_time) : 30.0f;
        if (fps_smooth < 0.1f) fps_smooth = inst_fps;
        else fps_smooth = FPS_ALPHA * inst_fps + (1.0f - FPS_ALPHA) * fps_smooth;
        int fps_value = (int)fps_smooth;

        int warning_seconds = 0;
        if (current_state == STATE_WARNING && object_present) {
            warning_seconds = max(0, (int)(8.0 - stable_seconds));
        }

        // ---- 绘制界面 (每3帧刷新一次imshow) ----
        draw_overlay(frame, current_state, fps_value, false, stable_seconds);
        show_frame_cnt++;
        if (show_frame_cnt % 3 == 0)
            imshow("Fire Lane Detection", frame);

        // ---- 写status.json (1Hz) ----
        if (duration<double>(steady_clock::now() - last_status_write).count() >= 1.0) {
            double uptime = duration<double>(steady_clock::now() - program_start).count();
            string state_str = getStateText(current_state);
            Scalar color = getStateColor(current_state);
            char color_hex[8];
            snprintf(color_hex, sizeof(color_hex), "#%02x%02x%02x",
                     (int)color[2], (int)color[1], (int)color[0]);

            time_t nw = time(nullptr); char ts[64];
            strftime(ts, sizeof(ts), "%H:%M:%S", localtime(&nw));
            ofstream sf("status.json");
            sf << "{";
            sf << "\"state\":\"" << state_str << "\",";
            sf << "\"state_color\":\"" << color_hex << "\",";
            sf << "\"fps\":" << fps_value << ",";
            sf << "\"photo_count\":" << photo_count << ",";
            sf << "\"detections\":" << g_det_count << ",";
            sf << "\"flame_detected\":false,";
            sf << "\"slave_alive\":" << (slave_alive ? "true" : "false") << ",";
            sf << "\"timer\":" << fixed << setprecision(1) << stable_seconds << ",";
            sf << "\"countdown\":" << warning_seconds << ",";
            sf << "\"uptime\":" << (int)uptime << ",";
            sf << "\"timestamp\":\"" << ts << "\"";
            sf << "}";
            sf.close();
            last_status_write = steady_clock::now();
        }

        // ---- 写stream.jpg (0.5s = 2Hz, 降低IO压力) ----
        if (duration<double>(steady_clock::now() - last_stream_write).count() >= 0.5) {
            vector<uchar> buf;
            imencode(".jpg", frame, buf, {IMWRITE_JPEG_QUALITY, 70});
            ofstream sf("stream.jpg", ios::binary);
            sf.write((char*)buf.data(), buf.size());
            sf.close();
            last_stream_write = steady_clock::now();
        }
    }

    // ---- 清理退出 ----
    shutdown_all_hardware();
    if (rpmsg_fd >= 0) close(rpmsg_fd);
    release_flame_sensor();
    cap.release();
    destroyAllWindows();
    cout << "[INFO] 程序正常退出，所有硬件与文件资源释放完毕\n";
    return 0;
}
