/*
 * fire_detect.cpp - 消防通道杂物占用检测
 *
 * 架构: 飞腾派异构双核
 *   主核(Linux): 背景差分+YOLOv8 ROI检测 + 双链路火焰校验 + RPMsg指令下发
 *   从核(裸机): 三色LED + 蜂鸣器声光报警驱动 + 火焰传感器上报
 *
 * 操作:
 *   s - 保存背景
 *   c - 开始检测
 *   q - 退出
 *   (也支持自动启动: 3秒存背景, 5秒自动开检测)
 *
 * 逻辑:
 *   1. 背景差分法找到变化区域ROI（支持多区域, 最多3个）
 *   2. 对ROI区域跑YOLOv8识别杂物类型（box/vehicle/furniture）
 *   3. 物体静止停留3秒→WARNING黄灯
 *   4. 再停留5秒（共8秒）→OCCUPIED红灯+蜂鸣器间歇鸣响
 *   5. 主从核双链路火焰检测→FIRE红灯+蜂鸣器常鸣
 *   6. 报警截图保存到 fire_detect_project/pictures/
 *
 * ========================================
 * 本次优化清单 (共7项)
 * ========================================
 * [优化1] YOLOv8输出格式自适应: 自动检测[1,7,N]和[1,N,7]两种格式
 *         → 修复核心bug: 原代码只按[1,N,7]读取, 但Ultralytics默认导出[1,7,N]
 *         → 导致坐标和置信度全部错位, 检测框永远无法通过阈值
 * [优化2] catch块不再静默吞错: 输出ONNX运行时异常信息, 便于排查
 * [优化3] 模型输入输出名自适应: 先查实际名称, 不再硬编码"images"/"output0"
 * [优化4] 检测框样式: 绿色(0,255,0) 厚度3, ROI框改为细线浅色不干扰
 * [优化5] 阈值下调: CONF_THRESH 0.3→0.25, IOU_STATIC_THRESH 0.5→0.35,
 *         MIN_OBJECT_AREA 500→300, 提高小物体和边缘检出率
 * [优化6] 全链路调试日志: ROI数量/YOLO输出shape/检测目标/状态变化全部打印
 * [优化7] 自动启动: 3秒自动存背景, 5秒自动开检测, 无需手动按键
 */
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <gpiod.h>
#include <signal.h>
#include <iomanip>
#include <sstream>
#include <poll.h>
#include <fstream>
#include <algorithm>

using namespace cv;
using namespace std;
using namespace chrono;
using namespace Ort;

// ====================== 火焰传感器：飞腾派引脚 GPIO2_10 ======================
#define FLAME_CHIP 2
#define FLAME_LINE 10

// ====================== RPMsg 协议（主→从）======================
#define CMD_STATE_IDLE     'G'
#define CMD_STATE_WARNING  'Y'
#define CMD_STATE_OCCUPIED 'R'
#define CMD_STATE_FIRE     'F'
#define CMD_EMERGENCY_STOP 'S'
#define CMD_HEARTBEAT      'H'
#define CMD_HEARTBEAT_ACK  'h'
#define CMD_SLAVE_FIRE_ALERT  'F'
#define CMD_SLAVE_FIRE_CLEAR  'f'
#define HEARTBEAT_INTERVAL_SEC   3.0
#define HEARTBEAT_TIMEOUT_SEC    10.0

// 截图保存目录
#define PICTURE_DIR "pictures"

// 全局变量
int rpmsg_fd = -1;
bool running = true;
bool fire_triggered = false;

static struct gpiod_chip* flame_chip = nullptr;
static struct gpiod_line* flame_line = nullptr;

static bool slave_flame_alert = false;
static bool slave_alive = false;
static steady_clock::time_point last_heartbeat_send;
static steady_clock::time_point last_heartbeat_ack;

// ====================== 时间字符串 ======================
// 生成 YYYYMMDD_HHMMSS_mmm 格式时间戳，用于截图文件名
string getCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto now_time_t = chrono::system_clock::to_time_t(now);
    auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    struct tm* tm_info = localtime(&now_time_t);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", tm_info);
    stringstream ss;
    ss << buffer << "_" << setfill('0') << setw(3) << ms.count();
    return ss.str();
}

// ====================== 带信息保存截图 ======================
// 在画面底部拼接80px黑色信息栏，写入状态/时间/告警文字后保存
void save_screenshot_with_info(Mat& frame, const string& state, int warning_seconds = 0) {
    Mat img_with_info = frame.clone();
    string state_text = "State: " + state;
    string time_text = "Time: " + getCurrentTimestamp();
    string alert_text = (state == "OCCUPIED" && warning_seconds >= 0) ?
                        "Alert: OCCUPIED - Channel Blocked" : "";

    int info_height = 80;
    Mat info_bar(img_with_info.rows + info_height, img_with_info.cols,
                 img_with_info.type(), Scalar(0,0,0));
    img_with_info.copyTo(info_bar(Rect(0, info_height, img_with_info.cols, img_with_info.rows)));

    putText(info_bar, state_text, Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,255,255), 2);
    putText(info_bar, time_text, Point(10, 55), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200,200,200), 1);
    if (!alert_text.empty()) {
        putText(info_bar, alert_text, Point(info_bar.cols - 300, 25),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
    }

    // [优化] 确保pictures目录存在
    string mkdir_cmd = string("mkdir -p ") + PICTURE_DIR;
    system(mkdir_cmd.c_str());
    string filename = string(PICTURE_DIR) + "/" + state + "_" + getCurrentTimestamp() + ".jpg";
    imwrite(filename, info_bar);
    cout << "[INFO] 截图已保存: " << filename << endl;
}

// ====================== IoU 计算 ======================
// 计算两个矩形的交并比，用于静止物体过滤
float getIoU(const Rect& a, const Rect& b) {
    int x1 = max(a.x, b.x);
    int y1 = max(a.y, b.y);
    int x2 = min(a.x + a.width, b.x + b.width);
    int y2 = min(a.y + a.height, b.y + b.height);
    int inter = max(0, x2 - x1) * max(0, y2 - y1);
    int areaA = a.width * a.height;
    int areaB = b.width * b.height;
    if (areaA + areaB - inter == 0) return 0.f;
    return (float)inter / (areaA + areaB - inter);
}

vector<Rect> lastStaticBoxes;
// [优化5] IoU阈值从0.5降到0.35: 物体轻微移动也能匹配为同一静止物体
const float IOU_STATIC_THRESH = 0.35f;

// ====================== 火焰传感器 ======================
// 通过libgpiod读取GPIO2_10，低电平(0)表示检测到火焰
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
// 主核通过/dev/rpmsg0发送单字符命令给从核
// 只在状态变化时发送，避免每帧重复发送
static char last_sent_state = 0;
void send_state(char state_cmd) {
    if (rpmsg_fd >= 0 && state_cmd != last_sent_state) {
        write(rpmsg_fd, &state_cmd, 1);
        usleep(1000);
        last_sent_state = state_cmd;
        cout << "[RPMsg] 发送状态: " << state_cmd << endl;
    }
}

// 紧急停止: 发送'S'命令，从核关闭所有LED和蜂鸣器
void shutdown_all_hardware() {
    if (rpmsg_fd >= 0) {
        last_sent_state = 0;
        char stop = CMD_EMERGENCY_STOP;
        write(rpmsg_fd, &stop, 1);
        usleep(100000);
    }
}

// 轮询读取从核上报的消息: 火焰警报(F/f)和心跳回复(h)
void poll_slave_messages() {
    if (rpmsg_fd < 0) return;
    struct pollfd pfd; pfd.fd = rpmsg_fd; pfd.events = POLLIN;
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        char buf[64]; ssize_t n = read(rpmsg_fd, buf, sizeof(buf));
        if (n <= 0) break;
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == CMD_SLAVE_FIRE_ALERT) { slave_flame_alert = true; cout << "[SLAVE] 火焰警报上报!" << endl; }
            else if (c == CMD_SLAVE_FIRE_CLEAR) { slave_flame_alert = false; cout << "[SLAVE] 火焰清除" << endl; }
            else if (c == CMD_HEARTBEAT_ACK) { slave_alive = true; last_heartbeat_ack = steady_clock::now(); }
        }
    }
}

// 心跳: 每3秒发送'H'，超过10秒没收到'h'回复则判定从核离线
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

// 信号处理: Ctrl+C或kill时安全关闭硬件
void signal_handler(int sig) {
    (void)sig; running = false;
    shutdown_all_hardware();
    if (rpmsg_fd >= 0) { close(rpmsg_fd); rpmsg_fd = -1; }
    release_flame_sensor();
}

// ====================== YOLO 参数 ======================
// best.onnx 导出时 imgsz=640, 必须匹配否则推理报错
const int INPUT_W = 640;
const int INPUT_H = 640;
// [优化5] 置信度阈值从0.30降到0.25: 提高检出率，减少漏检
const float CONF_THRESH = 0.25f;
const float NMS_THRESH = 0.45f;

// best.onnx: 3类
const vector<int> TARGET_CLASSES = {0, 1, 2};
map<int, string> class_names = { {0, "box"}, {1, "vehicle"}, {2, "furniture"} };

struct Detection { int class_id; float confidence; Rect box; string name; };

// letterbox预处理产生的缩放参数，postprocess需要用来还原坐标
static float pre_scale = 1.0f;
static int pre_pad_x = 0, pre_pad_y = 0;

/*
 * letterbox预处理:
 * 1. 计算缩放比例使图像填满320x320（保持宽高比）
 * 2. 缩放后居中放置在320x320画布上，周围填充灰色(114,114,114)
 * 3. BGR→RGB，归一化到0~1，转为CHW排布的float数组
 * 保存缩放比例和偏移量供后处理还原坐标使用
 */
vector<float> preprocess(const Mat& frame) {
    float scale = min((float)INPUT_W / frame.cols, (float)INPUT_H / frame.rows);
    int new_w = (int)(frame.cols * scale);
    int new_h = (int)(frame.rows * scale);
    Mat resized;
    resize(frame, resized, Size(new_w, new_h), 0, 0, INTER_LINEAR);
    Mat padded(INPUT_H, INPUT_W, CV_8UC3, Scalar(114, 114, 114));
    int pad_x = (INPUT_W - new_w) / 2;
    int pad_y = (INPUT_H - new_h) / 2;
    resized.copyTo(padded(Rect(pad_x, pad_y, new_w, new_h)));
    // 保存给postprocess用
    pre_scale = scale; pre_pad_x = pad_x; pre_pad_y = pad_y;
    Mat rgb, float_img;
    cvtColor(padded, rgb, COLOR_BGR2RGB);
    rgb.convertTo(float_img, CV_32FC3, 1.0/255.0);
    // 转为CHW排布: [channel][height][width]
    vector<float> input_tensor(3 * INPUT_W * INPUT_H);
    for (int c=0; c<3; ++c)
        for (int h=0; h<INPUT_H; ++h)
            for (int w=0; w<INPUT_W; ++w)
                input_tensor[c*INPUT_H*INPUT_W + h*INPUT_W + w] = float_img.at<Vec3f>(h,w)[c];
    return input_tensor;
}

/*
 * [优化1] YOLOv8输出后处理 - 核心修复
 *
 * YOLOv8 ONNX输出有两种格式，必须根据实际shape正确解析:
 *
 *   格式A: [1, 7, 2100]  通道优先 (Ultralytics默认导出格式)
 *          → 第i个anchor的cx = output[0*2100 + i]
 *          → 第i个anchor的cls1 = output[4*2100 + i]
 *          → 数据排列: cx0,cx1,...,cx2099, cy0,cy1,..., h0,...
 *
 *   格式B: [1, 2100, 7]  锚点优先 (手动transpose后)
 *          → 第i个anchor的cx = output[i*7 + 0]
 *          → 第i个anchor的cls1 = output[i*7 + 4]
 *          → 数据排列: cx0,cy0,w0,h0,c00,c10,c20, cx1,cy1,...
 *
 * 原代码只按格式B读取，但best.onnx实际是格式A
 * → 所有坐标和置信度全部错位读取 → 永远无法通过阈值 → 检测失效
 *
 * 修复: 从输出tensor的shape自动判断格式，按正确方式索引
 */
vector<Detection> postprocess(float* output, int img_w, int img_h, int dim1, int dim2) {
    int num_classes = 3;  // best.onnx固定3类: box/vehicle/furniture
    int stride = 4 + num_classes;  // 每个anchor 7个值: cx,cy,w,h,cls0,cls1,cls2

    // ---- 判断输出格式 ----
    bool channel_first = false;  // true=格式A[1,7,N], false=格式B[1,N,7]
    int num_dets = 0;

    if (dim1 == stride && dim2 > stride) {
        // [1, 7, 2100] → 格式A (通道优先)
        channel_first = true;
        num_dets = dim2;
    } else if (dim2 == stride && dim1 > stride) {
        // [1, 2100, 7] → 格式B (锚点优先)
        channel_first = false;
        num_dets = dim1;
    } else {
        // 兜底: 按总元素数推断
        int total = dim1 * dim2;
        num_dets = total / stride;
        channel_first = (dim1 == stride);
        if (num_dets <= 0) num_dets = 2100;
    }

    // ---- 首次运行打印格式信息，方便确认 ----
    static bool format_printed = false;
    if (!format_printed) {
        printf("[YOLO] 输出格式: [1,%d,%d] → %s, num_dets=%d, stride=%d\n",
               dim1, dim2, channel_first ? "A(通道优先)" : "B(锚点优先)", num_dets, stride);
        format_printed = true;
    }

    vector<Detection> detections;
    int debug_count = 0;

    for (int i=0; i<num_dets; ++i) {
        float cx, cy, w, h;
        float max_conf = 0; int max_id = 0;

        if (channel_first) {
            // ---- 格式A: [1, 7, 2100] ----
            // 数据排列: [cx0..cxN, cy0..cyN, w0..wN, h0..hN, cls00..cls0N, cls10..cls1N, cls20..cls2N]
            // 第i个anchor的第c个值 = output[c * num_dets + i]
            cx = output[0 * num_dets + i];
            cy = output[1 * num_dets + i];
            w  = output[2 * num_dets + i];
            h  = output[3 * num_dets + i];
            for (int c=0; c<num_classes; ++c) {
                float conf = output[(4+c) * num_dets + i];
                if (conf > max_conf) { max_conf = conf; max_id = c; }
            }
        } else {
            // ---- 格式B: [1, 2100, 7] ----
            // 数据排列: [cx0,cy0,w0,h0,c00,c10,c20, cx1,cy1,...]
            // 第i个anchor的第c个值 = output[i * stride + c]
            float* ptr = output + i * stride;
            cx = ptr[0]; cy = ptr[1]; w = ptr[2]; h = ptr[3];
            for (int c=0; c<num_classes; ++c) {
                float conf = ptr[4+c];
                if (conf > max_conf) { max_conf = conf; max_id = c; }
            }
        }

        // ---- [优化6] 调试日志: 打印前5个高置信度anchor ----
        // 这些anchor虽然不一定过阈值，但能验证坐标值是否合理
        // 如果坐标值全在0~1范围 → 说明是归一化坐标，需要乘以输入尺寸
        // 如果坐标值在0~320范围 → 说明是像素坐标，直接使用
        if (max_conf > 0.1f && debug_count < 5) {
            printf("[DEBUG] anchor[%d] cx=%.1f cy=%.1f w=%.1f h=%.1f cls=%d conf=%.3f fmt=%s\n",
                   i, cx, cy, w, h, max_id, max_conf, channel_first ? "A" : "B");
            debug_count++;
        }

        // ---- 置信度过滤 ----
        if (max_conf < CONF_THRESH) continue;

        // ---- 目标类别过滤 ----
        bool is_target = false;
        for (int cls : TARGET_CLASSES) if (max_id == cls) { is_target = true; break; }
        if (!is_target) continue;

        // ---- 坐标还原: 从320x320输入空间映射回原图空间 ----
        // letterbox变换: 原图→缩放→填充→320x320
        // 逆变换: 320x320坐标→减去填充→除以缩放比→原图坐标
        float scale = pre_scale;
        int px = pre_pad_x, py = pre_pad_y;
        int x = max(0, (int)((cx - w/2 - px) / scale));
        int y = max(0, (int)((cy - h/2 - py) / scale));
        int width = min((int)(w / scale), img_w - x);
        int height = min((int)(h / scale), img_h - y);

        if (width > 0 && height > 0) {
            Detection d;
            d.class_id = max_id;
            d.confidence = max_conf;
            d.box = Rect(x, y, width, height);
            d.name = class_names.count(max_id) ? class_names[max_id] : "object";
            detections.push_back(d);
        }
    }

    // ---- [优化6] 检测结果汇总日志 ----
    if (!detections.empty()) {
        printf("[YOLO] 检测到 %zu 个目标 (format=%s, num_dets=%d)\n",
               detections.size(), channel_first ? "A[7,N]" : "B[N,7]", num_dets);
        for (size_t i = 0; i < detections.size() && i < 5; i++) {
            printf("[YOLO]   → %s conf=%.2f rect=[%d,%d,%d,%d]\n",
                   detections[i].name.c_str(), detections[i].confidence,
                   detections[i].box.x, detections[i].box.y,
                   detections[i].box.width, detections[i].box.height);
        }
    }

    // ---- NMS非极大值抑制: 去除重叠框 ----
    if (detections.empty()) return {};
    vector<int> indices;
    vector<Rect> boxes; vector<float> confs;
    for (auto& d : detections) { boxes.push_back(d.box); confs.push_back(d.confidence); }
    dnn::NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH, indices);
    vector<Detection> result;
    for (int idx : indices) result.push_back(detections[idx]);
    return result;
}

// ====================== 背景差分 ======================
Mat background;       // 灰度背景图
bool bg_ready = false;
bool detection_started = false;
// [优化5] 最小物体面积从500降到300: 检出更小的杂物
const int MIN_OBJECT_AREA = 300;
const int MAX_ROIS = 3;  // 最多取3个变化区域

/*
 * [优化3] 动态阈值: 根据背景与当前帧的亮度差自适应调整
 * 场景暗→阈值低(最低20), 场景亮→阈值高(最高60)
 * 这样在不同光照条件下都能有效检测变化
 */
int get_adaptive_threshold(const Mat& bg, const Mat& cur) {
    Scalar bg_mean = mean(bg);
    Scalar cur_mean = mean(cur);
    double diff = abs(bg_mean[0] - cur_mean[0]);
    int thresh = 30 + (int)(diff * 0.3);
    return max(20, min(thresh, 60));
}

/*
 * 多ROI背景差分检测:
 * 1. 当前帧和背景帧都转灰度+高斯模糊降噪
 * 2. absdiff计算差分图
 * 3. 动态阈值二值化
 * 4. 形态学操作去噪点+填孔洞
 * 5. findContours找轮廓→boundingRect得矩形ROI
 * 6. 按面积排序取前3个最大的ROI
 */
vector<Rect> get_diff_rois(const Mat& frame) {
    vector<Rect> rois;
    if (!bg_ready) return rois;

    Mat gray, diff, thresh;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // 高斯模糊降噪
    Mat bg_blur, gray_blur;
    GaussianBlur(gray, gray_blur, Size(5,5), 0);
    GaussianBlur(background, bg_blur, Size(5,5), 0);

    // 背景差分
    absdiff(bg_blur, gray_blur, diff);

    // 动态阈值二值化
    int thresh_val = get_adaptive_threshold(bg_blur, gray_blur);
    threshold(diff, thresh, thresh_val, 255, THRESH_BINARY);

    // 形态学: 先闭运算填孔洞，再开运算去噪点
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
    morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);
    morphologyEx(thresh, thresh, MORPH_OPEN, kernel);

    // 找轮廓
    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 筛选: 面积>300 且 宽高比在0.3~3.0之间（排除过细的线条）
    vector<pair<double, Rect>> candidates;
    for (auto& cnt : contours) {
        Rect r = boundingRect(cnt);
        double area = r.area();
        float aspect_ratio = (float)r.width / max(1, r.height);
        if (area > MIN_OBJECT_AREA && aspect_ratio > 0.3 && aspect_ratio < 3.0) {
            candidates.push_back({area, r});
        }
    }

    // 按面积降序排序，取前MAX_ROIS个
    sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.first > b.first;
    });
    for (int i = 0; i < min((int)candidates.size(), MAX_ROIS); i++) {
        rois.push_back(candidates[i].second);
    }
    return rois;
}

// ====================== 状态机 ======================
enum State { IDLE, WARNING, OCCUPIED };
State cur_state = IDLE;
steady_clock::time_point object_stable_start;
bool object_present = false;

// 防抖动: 连续3帧检测到才确认物体存在，连续3帧无检测才确认物体清除
int consecutive_detection_count = 0;
const int NEED_CONSECUTIVE_FRAMES = 3;
int consecutive_no_detection_count = 0;
const int NEED_CONSECUTIVE_CLEAR = 3;

bool screenshot_taken = false;
int photo_count = 0;
bool first_frame_done = false;  // 第一帧标记: 跳过IoU过滤直接放行

// ====================== 界面绘制 ======================
void draw_overlay(Mat& frame, State s, int fps, bool emergency, int warning_seconds, double stable_seconds) {
    int right_x = frame.cols - 180;
    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "FPS: %d", fps);
    putText(frame, fps_str, Point(right_x, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 2);

    if (s != IDLE && !emergency) {
        char timer_str[64];
        snprintf(timer_str, sizeof(timer_str), "Timer: %.1f s", stable_seconds);
        putText(frame, timer_str, Point(right_x, 60), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(200,200,200), 1);
    }
    if (s == WARNING && !emergency) {
        char countdown_str[32];
        snprintf(countdown_str, sizeof(countdown_str), "Alert: %d s", warning_seconds);
        putText(frame, countdown_str, Point(right_x, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255), 2);
    }

    time_t now = time(nullptr);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
    putText(frame, time_str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);

    const char* state_text = "";
    Scalar state_color;
    if (emergency) { state_text = "FIRE ALERT!"; state_color = Scalar(0,0,255); }
    else {
        switch (s) {
            case IDLE: state_text = "FREE"; state_color = Scalar(0,255,0); break;
            case WARNING: state_text = "WARNING"; state_color = Scalar(0,255,255); break;
            case OCCUPIED: state_text = "OCCUPIED"; state_color = Scalar(0,0,255); break;
        }
    }
    putText(frame, state_text, Point(10, 65), FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2);

    int y3 = 100;
    if (emergency) {
        putText(frame, "FLAME DETECTED! BUZZER ON", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
        y3 += 30;
        putText(frame, "PLEASE TAKE ACTION", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 2);
    } else if (s == WARNING) {
        putText(frame, "Object detected, stable?", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,255), 2);
        y3 += 30;
        putText(frame, "BUZZER: OFF", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255), 2);
    } else if (s == OCCUPIED) {
        putText(frame, "OCCUPIED! BUZZER INTERMITTENT", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
        y3 += 30;
        putText(frame, "PLEASE CLEAR THE AREA", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 2);
    } else {
        if (!bg_ready) {
            putText(frame, "Press 's' to save background", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,255), 2);
            y3 += 30;
        } else if (!detection_started) {
            putText(frame, "Press 'c' to start detection", Point(10, y3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,255), 2);
            y3 += 30;
        }
    }

    char slave_str[64];
    snprintf(slave_str, sizeof(slave_str), "Slave: %s", slave_alive ? "ONLINE" : "OFFLINE");
    putText(frame, slave_str, Point(right_x, 120), FONT_HERSHEY_SIMPLEX, 0.4,
            slave_alive ? Scalar(0,200,0) : Scalar(0,0,200), 1);
}

// ====================== 主函数 ======================
int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    last_heartbeat_send = steady_clock::now();
    last_heartbeat_ack = steady_clock::now();
    system("mkdir -p pictures");

    // ---- 打开RPMsg设备 ----
    // /dev/rpmsg0 是主从核通信通道，发G/Y/R/F/S命令控制LED和蜂鸣器
    rpmsg_fd = open("/dev/rpmsg0", O_RDWR);
    if (rpmsg_fd < 0) {
        for (int i = 1; i <= 3; i++) {
            string path = "/dev/rpmsg" + to_string(i);
            rpmsg_fd = open(path.c_str(), O_RDWR);
            if (rpmsg_fd >= 0) { cout << "[INFO] 打开 rpmsg: " << path << endl; break; }
        }
    }
    if (rpmsg_fd < 0) { cerr << "[WARN] 无法打开 rpmsg 设备" << endl; }
    else { slave_alive = true; send_state(CMD_STATE_IDLE); }

    // ---- 初始化火焰传感器 ----
    init_flame_sensor();

    // ---- 加载YOLO模型 ----
    Env env(ORT_LOGGING_LEVEL_WARNING, "yolo");
    SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetInterOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Session session(nullptr);
    try {
        session = Session(env, "best.onnx", opts);
        cout << "[INFO] YOLO 模型加载成功 (best.onnx - 3类: box/vehicle/furniture)" << endl;
    } catch(const Ort::Exception& e) {
        cerr << "[ERROR] 模型加载失败: " << e.what() << endl;
        cerr << "[ERROR] 请确认best.onnx在当前目录" << endl;
        shutdown_all_hardware();
        if (rpmsg_fd >= 0) close(rpmsg_fd);
        release_flame_sensor();
        return -1;
    }

    // ---- [优化3] 自动获取模型输入输出名称 ----
    // 不再硬编码"images"/"output0"，而是从模型元数据中读取实际名称
    Ort::AllocatorWithDefaultOptions allocator;
    string input_name_str, output_name_str;
    try {
        auto input_name_alloc = session.GetInputNameAllocated(0, allocator);
        input_name_str = string(input_name_alloc.get());

        auto output_name_alloc = session.GetOutputNameAllocated(0, allocator);
        output_name_str = string(output_name_alloc.get());

        cout << "[INFO] 模型输入名: " << input_name_str << endl;
        cout << "[INFO] 模型输出名: " << output_name_str << endl;
    } catch(...) {
        // 兜底: 使用常见默认名称
        input_name_str = "images";
        output_name_str = "output0";
        cout << "[WARN] 无法读取模型IO名称，使用默认: " << input_name_str << "/" << output_name_str << endl;
    }

    // ---- 打开摄像头 ----
    VideoCapture cap;
    for (int i = 0; i <= 3; i++) {
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
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);

    cout << "\n========== 消防通道检测系统 ==========" << endl;
    cout << "时间逻辑: 静止3秒预警, 再5秒报警 (真实时间)" << endl;
    cout << "1. 按 s 存背景 (或3秒后自动存)" << endl;
    cout << "2. 按 c 开始检测 (或5秒后自动开始)" << endl;
    cout << "3. 按 q 退出" << endl;
    cout << "截图保存到: pictures/" << endl;
    cout << "======================================\n" << endl;

    steady_clock::time_point last_status_write = steady_clock::now();
    steady_clock::time_point last_stream_write = steady_clock::now();
    steady_clock::time_point program_start = steady_clock::now();
    float fps_smooth = 0.0f;
    const float FPS_ALPHA = 0.15f;

    // 预热摄像头: 丢掉前20帧让曝光稳定
    Mat frame;
    for (int i = 0; i < 20; i++) { cap >> frame; if (frame.empty()) break; waitKey(1); }

    // ---- [优化7] 自动启动计时 ----
    // 程序启动3秒后自动保存背景，5秒后自动开始检测
    steady_clock::time_point prog_start = steady_clock::now();
    bool auto_bg_saved = false;
    bool auto_detect_started = false;

    // ---- 主循环 ----
    while (running) {
        auto start = steady_clock::now();
        cap >> frame;
        if (frame.empty()) break;

        // ---- 按键处理 ----
        char key = waitKey(1);
        if (key == 'q' || key == 'Q') break;
        else if (key == 's' || key == 'S') {
            if (!detection_started) {
                background = frame.clone();
                cvtColor(background, background, COLOR_BGR2GRAY);
                bg_ready = true;
                first_frame_done = false;
                lastStaticBoxes.clear();
                consecutive_detection_count = 0;
                consecutive_no_detection_count = 0;
                auto_bg_saved = true;  // 标记已手动保存
                cout << "[INFO] 背景已保存！按 c 开始检测" << endl;
            }
        }
        else if (key == 'c' || key == 'C') {
            if (bg_ready && !detection_started) {
                detection_started = true;
                auto_detect_started = true;
                cout << "[INFO] 开始检测" << endl;
            }
        }

        // ---- 自动启动逻辑 ----
        double elapsed = duration<double>(steady_clock::now() - prog_start).count();
        if (!auto_bg_saved && elapsed >= 3.0 && !bg_ready) {
            background = frame.clone();
            cvtColor(background, background, COLOR_BGR2GRAY);
            bg_ready = true;
            first_frame_done = false;
            lastStaticBoxes.clear();
            auto_bg_saved = true;
            cout << "[AUTO] 3秒自动保存背景完成" << endl;
        }
        if (auto_bg_saved && !auto_detect_started && elapsed >= 5.0 && !detection_started) {
            detection_started = true;
            auto_detect_started = true;
            cout << "[AUTO] 5秒自动开始检测" << endl;
        }

        // ---- 心跳维护 ----
        update_heartbeat();
        poll_slave_messages();

        // ---- 火焰双链路检测 ----
        // 主核直接读GPIO + 从核火焰传感器上报，任一触发即报警
        bool flame_local = flame_detected();
        bool flame_slave = is_slave_flame();
        bool fire = flame_local || flame_slave;

        if (fire) {
            if (!fire_triggered) {
                cout << "[FIRE] 火焰检测到！";
                if (flame_local && flame_slave) cout << " (双链路)";
                else if (flame_slave) cout << " (从核)";
                else cout << " (主核)";
                cout << endl;
                fire_triggered = true;
                string mkdir_cmd = string("mkdir -p ") + PICTURE_DIR;
                system(mkdir_cmd.c_str());
                string snap = string(PICTURE_DIR) + "/fire_" + getCurrentTimestamp() + ".jpg";
                imwrite(snap, frame);
                photo_count++;
            }
            send_state(CMD_STATE_FIRE);
            double frame_time = duration<double>(steady_clock::now() - start).count();
            int fps_value = (frame_time > 0.001) ? (int)(1.0 / frame_time) : 30;
            draw_overlay(frame, cur_state, fps_value, true, 0, 0);
            imshow("Fire Lane Detection", frame);
            continue;
        } else {
            if (fire_triggered) {
                cout << "[FIRE] 火焰已熄灭，关闭报警" << endl;
                fire_triggered = false;
                send_state(CMD_STATE_IDLE);
            }
        }

        // ---- 背景差分找变化ROI ----
        vector<Rect> diff_rois;
        if (detection_started && bg_ready) {
            diff_rois = get_diff_rois(frame);
        }

        // ---- [优化6] ROI调试日志 ----
        if (!diff_rois.empty()) {
            printf("[DIFF] 检测到 %zu 个变化ROI区域", diff_rois.size());
            for (size_t i = 0; i < diff_rois.size(); i++) {
                printf(" [%d,%d,%dx%d]", diff_rois[i].x, diff_rois[i].y,
                       diff_rois[i].width, diff_rois[i].height);
            }
            printf("\n");
        }

        // ---- 对每个ROI跑YOLO推理 ----
        vector<Detection> raw_detections;
        for (auto& diff_roi : diff_rois) {
            if (diff_roi.width <= 30 || diff_roi.height <= 30) continue;

            // ROI向外扩充20像素，给YOLO更多上下文
            diff_roi.x = max(0, diff_roi.x - 20);
            diff_roi.y = max(0, diff_roi.y - 20);
            diff_roi.width = min(frame.cols - diff_roi.x, diff_roi.width + 40);
            diff_roi.height = min(frame.rows - diff_roi.y, diff_roi.height + 40);

            // ROI框: 蓝色细线, 与YOLO检测框(绿色粗线)区分
            rectangle(frame, diff_roi, Scalar(255, 0, 0), 1, LINE_AA);

            // 提取ROI区域并预处理
            Mat roi = frame(diff_roi);
            vector<float> inp = preprocess(roi);
            vector<int64_t> shape = {1, 3, INPUT_H, INPUT_W};
            Value input = Value::CreateTensor<float>(
                MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
                inp.data(), inp.size(), shape.data(), shape.size());

            // [优化3] 使用从模型读取的实际IO名称
            const char* in_names[] = {input_name_str.c_str()};
            const char* out_names[] = {output_name_str.c_str()};
            Value output{nullptr};

            // [优化2] 不再静默吞错: 输出ONNX异常信息
            try {
                session.Run(RunOptions{nullptr}, in_names, &input, 1, out_names, &output, 1);
                float* out_data = output.GetTensorMutableData<float>();

                // [优化1] 获取实际输出shape，判断是[1,7,N]还是[1,N,7]
                auto out_info = output.GetTensorTypeAndShapeInfo();
                auto out_shape = out_info.GetShape();
                int dim1 = (out_shape.size() >= 2) ? (int)out_shape[1] : 7;
                int dim2 = (out_shape.size() >= 3) ? (int)out_shape[2] : 2100;

                vector<Detection> dets = postprocess(out_data, roi.cols, roi.rows, dim1, dim2);

                // ROI内坐标 → 全图坐标偏移
                for (auto& d : dets) {
                    d.box.x += diff_roi.x;
                    d.box.y += diff_roi.y;
                    raw_detections.push_back(d);
                }
            } catch (const Ort::Exception& e) {
                // [优化2] 输出错误信息而非静默忽略
                cerr << "[ERROR] YOLO推理异常: " << e.what() << endl;
            }
        }

        // ---- 静止物体过滤 ----
        // 用IoU判断当前帧检测框与上一帧是否位置重叠
        // 重叠→物体静止→纳入最终检测；不重叠→物体在移动→过滤掉
        vector<Detection> final_detections;
        vector<Rect> curBoxes;
        for (auto& d : raw_detections) curBoxes.push_back(d.box);

        if (!first_frame_done) {
            // 第一帧: 所有检测直接放行，建立初始静止框集合
            final_detections = raw_detections;
            lastStaticBoxes = curBoxes;
            first_frame_done = true;
        } else {
            for (size_t i = 0; i < raw_detections.size(); i++) {
                Rect b = raw_detections[i].box;
                bool isStatic = false;
                for (auto& lb : lastStaticBoxes) {
                    // [优化5] IoU阈值从0.5降到0.35: 轻微移动也能匹配
                    if (getIoU(b, lb) > IOU_STATIC_THRESH) { isStatic = true; break; }
                }
                if (isStatic) final_detections.push_back(raw_detections[i]);
            }
            lastStaticBoxes = curBoxes;
        }

        // ---- [优化4] 画检测框: 绿色 厚度3 带标签 ----
        Detection best_detection;
        bool has_best = false;
        for (auto& d : final_detections) {
            // 检测框: 纯绿(0,255,0), 厚度3, 抗锯齿
            rectangle(frame, d.box, Scalar(0, 255, 0), 3, LINE_AA);
            // 标签: 类名+置信度
            char label[64];
            snprintf(label, sizeof(label), "%s %.2f", d.name.c_str(), d.confidence);
            // 标签背景: 绿色半透明矩形
            int baseline = 0;
            Size text_size = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            rectangle(frame,
                      Point(d.box.x, d.box.y - text_size.height - 6),
                      Point(d.box.x + text_size.width + 4, d.box.y),
                      Scalar(0, 180, 0), -1);
            putText(frame, label, Point(d.box.x + 2, d.box.y - 4),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_AA);
            // 记录面积最大的检测框作为best
            if (!has_best || d.box.area() > best_detection.box.area()) {
                best_detection = d; has_best = true;
            }
        }

        // ---- 防抖动检测 ----
        // 连续3帧检测到物体 → 确认物体存在，开始计时
        // 连续3帧无检测 → 确认物体清除
        bool current_raw = has_best;
        if (current_raw) { consecutive_detection_count++; consecutive_no_detection_count = 0; }
        else { consecutive_no_detection_count++; consecutive_detection_count = 0; }

        if (consecutive_detection_count >= NEED_CONSECUTIVE_FRAMES && !object_present) {
            object_present = true;
            object_stable_start = steady_clock::now();
            screenshot_taken = false;
            cout << "[INFO] 检测到静止物体，开始计时: " << best_detection.name << endl;
        } else if (consecutive_no_detection_count >= NEED_CONSECUTIVE_CLEAR && object_present) {
            object_present = false;
            screenshot_taken = false;
            cout << "[INFO] 物体已清除" << endl;
        }

        double stable_seconds = 0;
        if (object_present) {
            stable_seconds = duration<double>(steady_clock::now() - object_stable_start).count();
        }

        // ---- 状态机 ----
        // IDLE(绿灯) → 物体静止3秒 → WARNING(黄灯) → 再5秒 → OCCUPIED(红灯+蜂鸣器间歇)
        if (!object_present) {
            if (cur_state != IDLE) {
                cur_state = IDLE;
                send_state(CMD_STATE_IDLE);
                screenshot_taken = false;
                cout << "[INFO] 恢复空闲状态" << endl;
            }
        } else {
            if (cur_state == IDLE) {
                if (stable_seconds >= 3.0) {
                    cur_state = WARNING;
                    send_state(CMD_STATE_WARNING);
                    cout << "[WARNING] 静止3秒，进入预警状态 (黄灯)" << endl;
                }
            } else if (cur_state == WARNING) {
                double warning_elapsed = stable_seconds - 3.0;
                if (warning_elapsed >= 5.0) {
                    cur_state = OCCUPIED;
                    send_state(CMD_STATE_OCCUPIED);
                    cout << "[ALERT] 消防通道占用！触发报警 (红灯+蜂鸣器间歇)" << endl;
                    if (!screenshot_taken) {
                        save_screenshot_with_info(frame, "OCCUPIED", 0);
                        photo_count++;
                        screenshot_taken = true;
                    }
                }
            }
        }

        int warning_seconds = 0;
        if (cur_state == WARNING && object_present) {
            double warning_elapsed = stable_seconds - 3.0;
            warning_seconds = max(0, (int)(5.0 - warning_elapsed));
        }

        // ---- FPS计算 ----
        auto end = steady_clock::now();
        double frame_time = duration<double>(end - start).count();
        float inst_fps = (frame_time > 0.001) ? (float)(1.0 / frame_time) : 30.0f;
        if (fps_smooth < 0.1f) fps_smooth = inst_fps;
        else fps_smooth = FPS_ALPHA * inst_fps + (1.0f - FPS_ALPHA) * fps_smooth;
        int fps_value = (int)fps_smooth;

        // ---- 绘制界面 ----
        draw_overlay(frame, cur_state, fps_value, false, warning_seconds, stable_seconds);
        imshow("Fire Lane Detection", frame);

        // ---- 写status.json (1Hz) ----
        if (duration<double>(steady_clock::now() - last_status_write).count() >= 1.0) {
            double uptime = duration<double>(steady_clock::now() - program_start).count();
            string state_str, state_color;
            if (fire_triggered) { state_str = "FIRE"; state_color = "#ff1744"; }
            else switch (cur_state) {
                case IDLE: state_str = "FREE"; state_color = "#2ecc71"; break;
                case WARNING: state_str = "WARNING"; state_color = "#f39c12"; break;
                case OCCUPIED: state_str = "OCCUPIED"; state_color = "#e74c3c"; break;
            }
            time_t nw = time(nullptr); char ts[64];
            strftime(ts, sizeof(ts), "%H:%M:%S", localtime(&nw));
            ofstream sf("status.json");
            sf << "{";
            sf << "\"state\":\"" << state_str << "\",";
            sf << "\"state_color\":\"" << state_color << "\",";
            sf << "\"fps\":" << fps_value << ",";
            sf << "\"photo_count\":" << photo_count << ",";
            sf << "\"flame_detected\":" << (fire_triggered ? "true" : "false") << ",";
            sf << "\"slave_alive\":" << (slave_alive ? "true" : "false") << ",";
            sf << "\"timer\":" << fixed << setprecision(1) << stable_seconds << ",";
            sf << "\"countdown\":" << warning_seconds << ",";
            sf << "\"uptime\":" << (int)uptime << ",";
            sf << "\"timestamp\":\"" << ts << "\"";
            sf << "}";
            sf.close();
            last_status_write = steady_clock::now();
        }

        // ---- 写stream.jpg (5Hz, 供Web前端预览) ----
        if (duration<double>(steady_clock::now() - last_stream_write).count() >= 0.2) {
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
    cout << "[INFO] 程序已退出" << endl;
    return 0;
}
