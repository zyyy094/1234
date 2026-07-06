#include <opencv2/opencv.hpp>
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
#include <algorithm>
#include <cmath>

using namespace cv;
using namespace std;

// ====================== 配置 ======================
#define FLAME_CHIP   2
#define FLAME_LINE   10

const int INPUT_W = 320;
const int INPUT_H = 320;
const float CONF_THRESH = 0.3f;
const float NMS_THRESH = 0.45f;

const vector<int> TARGET_CLASSES = {56, 60, 57, 58, 24, 28, 13};

const map<int, string> CLASS_NAMES = {
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
    Rect box;
    string name;
};

enum State { IDLE, WARNING, OCCUPIED };

// ====================== 全局变量 ======================
int rpmsg_fd = -1;
bool running = true;
bool fire_triggered = false;
bool bg_ready = false;
bool detection_started = false;
int photo_count = 0;

Mat background;
vector<Rect> lastStaticBoxes;

State cur_state = IDLE;
chrono::steady_clock::time_point object_stable_start;
bool object_present = false;
chrono::steady_clock::time_point last_buzzer_toggle;
bool buzzer_state = false;

int consecutive_detection_count = 0;
int consecutive_no_detection_count = 0;

static struct gpiod_chip* g_chip = nullptr;
static struct gpiod_line* g_line = nullptr;

// ====================== 工具函数 ======================
string getCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto now_time_t = chrono::system_clock::to_time_t(now);
    auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    struct tm* tm_info = localtime(&now_time_t);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", tm_info);
    stringstream ss;
    ss << buffer << "_" << setfill('0') << setw(3) << ms.count();
    return ss.str();
}

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

// ====================== GPIO 火焰传感器 ======================
bool initFlameSensor() {
    g_chip = gpiod_chip_open_by_number(FLAME_CHIP);
    if (!g_chip) {
        cerr << "[ERROR] 打开GPIO芯片失败" << endl;
        return false;
    }
    g_line = gpiod_chip_get_line(g_chip, FLAME_LINE);
    if (!g_line) {
        gpiod_chip_close(g_chip);
        g_chip = nullptr;
        cerr << "[ERROR] 获取GPIO引脚失败" << endl;
        return false;
    }
    if (gpiod_line_request_input(g_line, "flame_sensor") < 0) {
        gpiod_line_release(g_line);
        gpiod_chip_close(g_chip);
        g_chip = nullptr;
        g_line = nullptr;
        cerr << "[ERROR] GPIO输入模式申请失败" << endl;
        return false;
    }
    cout << "[INFO] 火焰传感器初始化成功" << endl;
    return true;
}

bool isFlameDetected() {
    if (!g_line) return false;
    return gpiod_line_get_value(g_line) == 0;
}

void releaseFlameSensor() {
    if (g_line) { gpiod_line_release(g_line); g_line = nullptr; }
    if (g_chip) { gpiod_chip_close(g_chip); g_chip = nullptr; }
}

// ====================== RPMsg 外设控制 ======================
bool initRpmsg() {
    rpmsg_fd = open("/dev/rpmsg0", O_RDWR);
    if (rpmsg_fd < 0) {
        for (int i = 1; i <= 3; i++) {
            string path = "/dev/rpmsg" + to_string(i);
            rpmsg_fd = open(path.c_str(), O_RDWR);
            if (rpmsg_fd >= 0) break;
        }
    }
    if (rpmsg_fd < 0) {
        cerr << "[WARN] 无法打开 rpmsg 设备" << endl;
        return false;
    }
    return true;
}

void sendCmd(char cmd) {
    if (rpmsg_fd >= 0) {
        write(rpmsg_fd, &cmd, 1);
        usleep(1000);
    }
}

void setLed(char color) {
    sendCmd('r');
    sendCmd('y');
    sendCmd('g');
    if (color == 'R') sendCmd('R');
    else if (color == 'Y') sendCmd('Y');
    else if (color == 'G') sendCmd('G');
}

void setBuzzer(bool on) {
    sendCmd(on ? 'B' : 'b');
}

void shutdownHardware() {
    setLed('r');
    setLed('y');
    setLed('g');
    setBuzzer(false);
    usleep(100000);
}

void closeRpmsg() {
    if (rpmsg_fd >= 0) {
        close(rpmsg_fd);
        rpmsg_fd = -1;
    }
}

void signalHandler(int sig) {
    cout << "\n[INFO] 退出信号捕获" << endl;
    running = false;
    shutdownHardware();
    releaseFlameSensor();
    closeRpmsg();
    exit(0);
}

// ====================== YOLO 推理 ======================
vector<float> preprocess(const Mat& frame) {
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

vector<Detection> postprocess(float* output, int img_w, int img_h) {
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

// ====================== 背景差分 ======================
bool getDiffROI(const Mat& frame, Rect& roi) {
    if (!bg_ready) return false;
    Mat gray, diff, thresh;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 0);

    Mat bg_blur;
    GaussianBlur(background, bg_blur, Size(5, 5), 0);

    absdiff(bg_blur, gray, diff);
    threshold(diff, thresh, 40, 255, THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);
    morphologyEx(thresh, thresh, MORPH_OPEN, kernel);

    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Rect max_rect;
    double max_area = 0;

    for (auto& cnt : contours) {
        Rect r = boundingRect(cnt);
        double area = r.area();
        float aspect = (float)r.width / r.height;
        if (area > max_area && area > MIN_OBJECT_AREA && aspect > 0.4 && aspect < 2.5) {
            max_area = area;
            max_rect = r;
        }
    }

    if (max_area > 0) {
        roi = max_rect;
        return true;
    }
    return false;
}

// ====================== 静态目标过滤 ======================
vector<Detection> filterStaticDetections(const vector<Detection>& dets) {
    vector<Detection> result;
    vector<Rect> curBoxes;
    for (auto& d : dets) curBoxes.push_back(d.box);

    if (lastStaticBoxes.empty()) {
        lastStaticBoxes = curBoxes;
        return result;
    }

    for (size_t i = 0; i < dets.size(); i++) {
        float max_iou = 0;
        for (auto& pre : lastStaticBoxes) {
            float iou = getIoU(curBoxes[i], pre);
            if (iou > max_iou) max_iou = iou;
        }
        if (max_iou > IOU_STATIC_THRESH) {
            result.push_back(dets[i]);
        }
    }
    lastStaticBoxes = curBoxes;
    return result;
}

// ====================== 蜂鸣器间歇控制 ======================
void updateBuzzer() {
    auto now = chrono::steady_clock::now();
    double elapsed = chrono::duration<double>(now - last_buzzer_toggle).count();
    if (elapsed >= 0.5) {
        buzzer_state = !buzzer_state;
        setBuzzer(buzzer_state);
        last_buzzer_toggle = now;
    }
}

// ====================== 界面绘制 ======================
void drawOverlay(Mat& frame, State state, int fps, bool emergency,
                 int countdown, double stable_sec) {
    int right_x = frame.cols - 180;

    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "FPS: %d", fps);
    putText(frame, fps_str, Point(right_x, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

    if (state != IDLE && !emergency) {
        char timer_str[64];
        snprintf(timer_str, sizeof(timer_str), "Timer: %.1f s", stable_sec);
        putText(frame, timer_str, Point(right_x, 60), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(200, 200, 200), 1);
    }

    if (state == WARNING && !emergency) {
        char cnt_str[32];
        snprintf(cnt_str, sizeof(cnt_str), "Alert: %d s", countdown);
        putText(frame, cnt_str, Point(right_x, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    }

    time_t now = time(nullptr);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
    putText(frame, time_str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

    const char* state_text = "";
    Scalar state_color;
    if (emergency) {
        state_text = "FIRE ALERT!";
        state_color = Scalar(0, 0, 255);
    } else {
        switch (state) {
            case IDLE:      state_text = "FREE"; state_color = Scalar(0, 255, 0); break;
            case WARNING:   state_text = "WARNING"; state_color = Scalar(0, 255, 255); break;
            case OCCUPIED:  state_text = "OCCUPIED"; state_color = Scalar(0, 0, 255); break;
        }
    }
    putText(frame, state_text, Point(10, 65), FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2);

    int y = 100;
    if (emergency) {
        putText(frame, "FLAME DETECTED! BUZZER ON", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        y += 30;
        putText(frame, "PLEASE TAKE ACTION", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    } else if (state == WARNING) {
        putText(frame, "Object detected, stable?", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        y += 30;
        putText(frame, "BUZZER: OFF", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    } else if (state == OCCUPIED) {
        putText(frame, "OCCUPIED! BUZZER INTERMITTENT", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        y += 30;
        putText(frame, "PLEASE CLEAR THE AREA", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    } else {
        if (!bg_ready) {
            putText(frame, "Press 's' to save background", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        } else if (!detection_started) {
            putText(frame, "Press 'c' to start detection", Point(10, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        }
    }

    char photo_str[64];
    snprintf(photo_str, sizeof(photo_str), "Photos: %d (Press 'p' to save)", photo_count);
    putText(frame, photo_str, Point(10, frame.rows - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 2);
}

// ====================== 主函数 ======================
int main() {
    signal(SIGINT, signalHandler);
    last_buzzer_toggle = chrono::steady_clock::now();

    system("mkdir -p /home/user/captured");

    if (!initFlameSensor()) return -1;
    initRpmsg();
    shutdownHardware();

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolo");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(2);

    // ========== 核心修复：删除空构造Ort::Session(nullptr)，直接传入模型构造 ==========
    Ort::Session session(env, "yolov8n.onnx", opts);
    // ==========================================================================

    cout << "[INFO] YOLO 模型加载成功" << endl;

    VideoCapture cap;
    bool cam_ok = false;
    for (int i = 0; i <= 3; i++) {
        if (cap.open(i, CAP_V4L2)) {
            cout << "[INFO] 打开摄像头 /dev/video" << i << endl;
            cam_ok = true;
            break;
        }
    }
    if (!cam_ok) {
        cerr << "[ERROR] 摄像头打开失败" << endl;
        shutdownHardware();
        releaseFlameSensor();
        closeRpmsg();
        return -1;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    cout << "\n===== 消防通道检测系统 =====\n";
    cout << "s: 保存背景 | c: 启动检测 | p: 拍照 | q: 退出\n";

    Mat frame;
    bool timer_started = false;
    chrono::steady_clock::time_point start_time;

    while (running) {
        auto loop_start = chrono::steady_clock::now();
        cap >> frame;
        if (frame.empty()) continue;

        char key = waitKey(1);
        if (key == 'q') break;
        else if (key == 's') {
            background = frame.clone();
            cvtColor(background, background, COLOR_BGR2GRAY);
            bg_ready = true;
            lastStaticBoxes.clear();
            timer_started = false;
            cout << "[INFO] 背景已保存" << endl;
        } else if (key == 'c' && bg_ready) {
            detection_started = true;
            cout << "[INFO] 检测启动" << endl;
        } else if (key == 'p') {
            string path = "/home/user/captured/img_" + getCurrentTimestamp() + ".jpg";
            imwrite(path, frame);
            photo_count++;
            cout << "[INFO] 截图保存: " << path << endl;
        }

        // 火焰检测
        if (isFlameDetected()) {
            if (!fire_triggered) {
                fire_triggered = true;
                cout << "[FIRE] 火焰检测到！" << endl;
            }
            setLed('R');
            setBuzzer(true);
            drawOverlay(frame, cur_state, 0, true, 0, 0);
            imshow("Fire Lane Detection", frame);
            continue;
        } else {
            if (fire_triggered) {
                fire_triggered = false;
                setBuzzer(false);
                setLed('G');
                cur_state = IDLE;
                object_present = false;
                timer_started = false;
                cout << "[INFO] 火焰已熄灭" << endl;
            }
        }

        // YOLO 检测
        vector<Detection> final_dets;
        if (detection_started && bg_ready) {
            Rect roi;
            if (getDiffROI(frame, roi) && roi.width > 30 && roi.height > 30) {
                roi.x = max(0, roi.x - 20);
                roi.y = max(0, roi.y - 20);
                roi.width = min(frame.cols - roi.x, roi.width + 40);
                roi.height = min(frame.rows - roi.y, roi.height + 40);

                Mat roi_frame = frame(roi);
                vector<float> input_data = preprocess(roi_frame);
                vector<int64_t> input_shape = {1, 3, INPUT_H, INPUT_W};

                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
                    input_data.data(), input_data.size(),
                    input_shape.data(), input_shape.size());

                const char* in_names[] = {"images"};
                const char* out_names[] = {"output0"};
                Ort::Value output_tensor{nullptr};

                try {
                    session.Run(Ort::RunOptions{}, in_names, &input_tensor, 1,
                                out_names, &output_tensor, 1);
                    float* out_ptr = output_tensor.GetTensorMutableData<float>();
                    auto dets = postprocess(out_ptr, roi_frame.cols, roi_frame.rows);
                    for (auto& d : dets) {
                        d.box.x += roi.x;
                        d.box.y += roi.y;
                    }
                    final_dets = filterStaticDetections(dets);
                } catch (...) {
                    cerr << "[WARN] 推理异常" << endl;
                }
            }
        }

        // 画框
        Detection best_det;
        bool has_target = false;
        for (auto& d : final_dets) {
            if (!has_target || d.box.area() > best_det.box.area()) {
                best_det = d;
                has_target = true;
            }
        }

        if (has_target) {
            rectangle(frame, best_det.box, Scalar(0, 255, 0), 3);
            putText(frame, best_det.name, Point(best_det.box.x, best_det.box.y - 8),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        }

        // 防抖
        if (has_target) {
            consecutive_detection_count++;
            consecutive_no_detection_count = 0;
        } else {
            consecutive_no_detection_count++;
            consecutive_detection_count = 0;
        }

        if (consecutive_detection_count >= NEED_CONSECUTIVE_FRAMES && !object_present) {
            object_present = true;
            start_time = chrono::steady_clock::now();
            timer_started = true;
            cout << "[INFO] 检测到物体" << endl;
        } else if (consecutive_no_detection_count >= NEED_CONSECUTIVE_CLEAR && object_present) {
            object_present = false;
            timer_started = false;
            cur_state = IDLE;
            setLed('G');
            setBuzzer(false);
            cout << "[INFO] 物体已移除" << endl;
        }

        // 状态机
        double stable_sec = 0;
        if (timer_started) {
            stable_sec = chrono::duration<double>(chrono::steady_clock::now() - start_time).count();
        }

        if (!object_present || !timer_started) {
            if (cur_state != IDLE) {
                cur_state = IDLE;
                setLed('G');
                setBuzzer(false);
            }
        } else {
            if (cur_state == IDLE) {
                if (stable_sec >= WARNING_DELAY_SEC) {
                    cur_state = WARNING;
                    setLed('Y');
                    cout << "[WARNING] 预警" << endl;
                }
            } else if (cur_state == WARNING) {
                double warn_elapsed = stable_sec - WARNING_DELAY_SEC;
                if (warn_elapsed >= ALARM_DELAY_SEC) {
                    cur_state = OCCUPIED;
                    setLed('R');
                    cout << "[ALERT] 报警!" << endl;
                    string snap = "occupied_" + to_string(time(nullptr)) + ".jpg";
                    imwrite(snap, frame);
                } else {
                    setLed('Y');
                }
            } else if (cur_state == OCCUPIED) {
                setLed('R');
                updateBuzzer();
            }
        }

        int countdown = 0;
        if (cur_state == WARNING && timer_started) {
            double warn_elapsed = stable_sec - WARNING_DELAY_SEC;
            countdown = max(0, (int)(ALARM_DELAY_SEC - warn_elapsed));
        }

        int fps = (int)(1.0 / chrono::duration<double>(chrono::steady_clock::now() - loop_start).count());
        drawOverlay(frame, cur_state, fps, false, countdown, stable_sec);
        imshow("Fire Lane Detection", frame);
    }

    shutdownHardware();
    releaseFlameSensor();
    closeRpmsg();
    cap.release();
    destroyAllWindows();
    cout << "[INFO] 程序正常退出" << endl;
    return 0;
}
