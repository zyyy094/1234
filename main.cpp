#include <opencv2/opencv.hpp>
#include <signal.h>
#include <iostream>
#include <chrono>

#include "common.h"
#include "hardware.h"
#include "bg_subtract.h"
#include "ui_draw.h"
#include "yolov8_infer.h"

using namespace std;
using namespace cv;

// ====================== 全局变量 ======================
bool running = true;
bool fire_triggered = false;
bool timer_started = false;

vector<Rect> lastStaticBoxes;
State cur_state = IDLE;
bool object_present = false;

chrono::steady_clock::time_point object_stable_start;
chrono::steady_clock::time_point last_buzzer_toggle;
bool buzzer_state = false;

int consecutive_detection_count = 0;
int consecutive_no_detection_count = 0;

YOLOv8Infer detector;

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

// ====================== 蜂鸣器 ======================
void updateBuzzer() {
    auto now = chrono::steady_clock::now();
    double elapsed = chrono::duration<double>(now - last_buzzer_toggle).count();
    if (elapsed >= 0.5) {
        buzzer_state = !buzzer_state;
        setBuzzer(buzzer_state);
        last_buzzer_toggle = now;
    }
}

// ====================== 信号处理 ======================
void signalHandler(int sig) {
    cout << "\n[INFO] 退出信号捕获" << endl;
    running = false;
    shutdownHardware();
    releaseFlameSensor();
    closeRpmsg();
    exit(0);
}

// ====================== 主函数 ======================
int main() {
    signal(SIGINT, signalHandler);
    last_buzzer_toggle = chrono::steady_clock::now();

    system("mkdir -p captures");

    if (!initFlameSensor()) return -1;
    initRpmsg();
    shutdownHardware();

    if (!detector.loadModel("yolov8n.onnx")) {
        shutdownHardware();
        releaseFlameSensor();
        closeRpmsg();
        return -1;
    }

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
    chrono::steady_clock::time_point start_time;

    while (running) {
        auto loop_start = chrono::steady_clock::now();
        cap >> frame;
        if (frame.empty()) continue;

        char key = waitKey(1);
        if (key == 'q') break;
        else if (key == 's') {
            saveBackground(frame);
            lastStaticBoxes.clear();
            timer_started = false;
            cout << "[INFO] 背景已保存" << endl;
        } else if (key == 'c' && bg_ready) {
            detection_started = true;
            cout << "[INFO] 检测启动" << endl;
        } else if (key == 'p') {
            string path = "captures/img_" + getCurrentTimestamp() + ".jpg";
            imwrite(path, frame);
            photo_count++;
            cout << "[INFO] 截图保存: " << path << endl;
        }

        // 火焰检测（主核 GPIO 直读 + 从核 RPMsg 上报，双重检测）
        pollSlaveMessages();  // 读取从核上报消息
        bool flame_local = isFlameDetected();
        bool flame_slave = isSlaveFlameAlert();
        bool flame_detected = flame_local || flame_slave;

        if (flame_detected) {
            if (!fire_triggered) {
                fire_triggered = true;
                cout << "[FIRE] 火焰检测到！" << endl;
                if (flame_slave && !flame_local) {
                    cout << "  (由从核上报)" << endl;
                }
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
                auto dets = detector.detect(roi_frame);
                for (auto& d : dets) {
                    d.box.x += roi.x;
                    d.box.y += roi.y;
                }
                final_dets = filterStaticDetections(dets);
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
                    string snap = "captures/occupied_" + to_string(time(nullptr)) + ".jpg";
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