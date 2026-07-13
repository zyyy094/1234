/*
 * main.cpp - 消防通道杂物占用视觉检测 + 火焰火情双判断系统
 *
 * 架构: 飞腾派异构双核
 *   主核(Linux): AI视觉推理(YOLOv8) + 业务逻辑 + Web服务
 *   从核(裸机): LED/蜂鸣器硬件驱动 + 火焰传感器采集 + 主动上报
 *
 * 通信: RPMsg 双向
 *   主→从: G/Y/R/F/S 状态指令 + H 心跳
 *   从→主: F/f 火焰上报 + h 心跳回复
 *
 * 操作:
 *   q / Ctrl+C - 退出并关闭所有声光外设
 *
 * 启动即自动全屏检测:
 *   - YOLOv8全屏检测所有物体，画绿色框+类别标签+置信度
 *   - 空闲:绿灯常亮 FREE
 *   - 杂物停留3秒:黄灯 WARNING
 *   - 杂物停留5秒:红灯+蜂鸣器间歇鸣响 OCCUPIED
 *   - 火焰:红灯+蜂鸣器持续长鸣 FIRE ALERT!
 */
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
#include <iomanip>

#include "common.h"
#include "hardware.h"
#include "fire_protocol.h"
#include "yolov8_infer.h"

using namespace std;
using namespace cv;
using namespace chrono;

// ====================== 全局变量 ======================
volatile bool running = true;
bool fire_triggered = false;
int photo_count = 0;

State cur_state = IDLE;
bool object_present = false;
steady_clock::time_point object_start_time;

int consecutive_detection_count = 0;
int consecutive_no_detection_count = 0;

YOLOv8Infer detector;

steady_clock::time_point program_start;
steady_clock::time_point last_status_write;
steady_clock::time_point last_stream_write;

// FPS平滑
float fps_smooth = 0.0f;
const float FPS_ALPHA = 0.15f;

// YOLO隔帧推理（每2帧推理一次，提升帧率）
int yolo_frame_counter = 0;
vector<Detection> cached_dets;
bool have_cached_dets = false;
const int YOLO_SKIP_FRAMES = 1;  // 每N+1帧推理一次

// ====================== 信号处理 ======================
void signalHandler(int sig) {
    (void)sig;
    cout << "\n[INFO] 退出信号，关闭所有声光外设..." << endl;
    running = false;
}

// ====================== 发送状态到从核 ======================
void sendStateToSlave(State state, bool fire) {
    if (fire) {
        sendState(CMD_STATE_FIRE);
    } else {
        switch (state) {
            case IDLE:     sendState(CMD_STATE_IDLE);     break;
            case WARNING:  sendState(CMD_STATE_WARNING);  break;
            case OCCUPIED: sendState(CMD_STATE_OCCUPIED); break;
        }
    }
}

// ====================== 叠加UI信息 ======================
void drawUI(Mat& frame, State state, int fps, bool fire, int countdown, double stable_sec) {
    int h = frame.rows;
    int w = frame.cols;

    // 顶部状态条
    Scalar bar_color;
    string state_str;
    if (fire) {
        bar_color = Scalar(0, 0, 255);
        state_str = "FIRE ALERT!";
    } else {
        switch (state) {
            case IDLE:     bar_color = Scalar(0, 200, 0);   state_str = "MONITORING"; break;
            case WARNING:  bar_color = Scalar(0, 200, 255); state_str = "WARNING";    break;
            case OCCUPIED: bar_color = Scalar(0, 0, 255);   state_str = "OCCUPIED";   break;
        }
    }
    rectangle(frame, Point(0, 0), Point(w, 40), bar_color, FILLED);

    putText(frame, state_str, Point(15, 28), FONT_HERSHEY_SIMPLEX, 0.8,
            Scalar(255,255,255), 2);

    // FPS 右上角
    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "FPS: %d", fps);
    putText(frame, fps_str, Point(w - 130, 28), FONT_HERSHEY_SIMPLEX, 0.6,
            Scalar(255,255,255), 2);

    // 计时/倒计时
    if (state == WARNING && object_present) {
        char cd[64];
        snprintf(cd, sizeof(cd), "Alarm in %ds...", countdown);
        putText(frame, cd, Point(w/2 - 80, 28), FONT_HERSHEY_SIMPLEX, 0.7,
                Scalar(0,0,0), 2);
    }
    if (state == OCCUPIED) {
        char tm[64];
        snprintf(tm, sizeof(tm), "Occupied %.1fs", stable_sec);
        putText(frame, tm, Point(w/2 - 80, 28), FONT_HERSHEY_SIMPLEX, 0.7,
                Scalar(255,255,255), 2);
    }

    // 底部提示
    putText(frame, "Press q to quit", Point(10, h - 10), FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(200,200,200), 1);
}

// ====================== 主函数 ======================
int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    program_start = steady_clock::now();
    last_status_write = program_start;
    last_stream_write = program_start;

    system("mkdir -p captures");

    // 1. 初始化RPMsg和火焰传感器
    initRpmsg();
    sendState(CMD_STATE_IDLE);
    initFlameSensor();

    // 2. 加载YOLO模型
    cout << "[INFO] 正在加载YOLOv8模型..." << endl;
    if (!detector.loadModel("yolov8n.onnx")) {
        cerr << "[ERROR] YOLO模型加载失败，请确认yolov8n.onnx在当前目录" << endl;
        sendState(CMD_EMERGENCY_STOP);
        releaseFlameSensor();
        closeRpmsg();
        return -1;
    }

    // 3. 打开摄像头
    VideoCapture cap;
    bool cam_ok = false;
    for (int i = 0; i <= 3; i++) {
        if (cap.open(i, CAP_V4L2)) { cam_ok = true; break; }
    }
    if (!cam_ok) {
        cerr << "[ERROR] 摄像头打开失败" << endl;
        sendState(CMD_EMERGENCY_STOP);
        releaseFlameSensor();
        closeRpmsg();
        return -1;
    }
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap.set(CAP_PROP_FPS, 30);

    cout << "\n===== 消防通道杂物检测系统 =====" << endl;
    cout << "架构: 异构双核 (Linux主核 + 裸机从核)" << endl;
    cout << "功能: 全屏YOLO检测 + 火焰双链路校验" << endl;
    cout << "启动后自动检测，按q退出" << endl;
    cout << "================================" << endl;

    // 等待摄像头稳定
    Mat frame;
    for (int i = 0; i < 20; i++) { cap >> frame; if (frame.empty()) break; waitKey(1); }

    State last_sent_state = (State)-1;
    bool last_sent_fire = false;

    // 4. 主循环
    while (running) {
        auto loop_start = steady_clock::now();

        cap >> frame;
        if (frame.empty()) { waitKey(1); continue; }

        // ---- 键盘操作（仅q退出）----
        int key = waitKey(1);
        if (key != -1) {
            char kc = (char)(key & 0xFF);
            if (kc == 'q' || kc == 'Q') {
                cout << "[INFO] 用户按键退出" << endl;
                break;
            }
        }

        // ---- 心跳 + 从核消息 ----
        updateHeartbeat();
        pollSlaveMessages();

        // ---- 火焰双链路检测 ----
        bool flame_local = isFlameDetected();
        bool flame_slave = isSlaveFlameAlert();
        bool flame_detected = flame_local || flame_slave;

        if (flame_detected) {
            if (!fire_triggered) {
                fire_triggered = true;
                cout << "[FIRE] 火焰检测到!";
                if (flame_local && flame_slave) cout << " (双链路)";
                else if (flame_slave) cout << " (从核)";
                else cout << " (主核)";
                cout << endl;
                string snap = "captures/fire_" + getCurrentTimestamp() + ".jpg";
                imwrite(snap, frame);
                photo_count++;
            }
        } else {
            if (fire_triggered) {
                fire_triggered = false;
                cur_state = IDLE;
                object_present = false;
                consecutive_detection_count = 0;
                consecutive_no_detection_count = 0;
                cout << "[INFO] 火焰已熄灭" << endl;
            }
        }

        int countdown = 0;
        double stable_sec = 0;

        if (fire_triggered) {
            // 火警:最高优先级
            sendStateToSlave(IDLE, true);
            last_sent_fire = true;
            // 火警时也画检测框（红色）
            vector<Detection> dets = cached_dets;
            for (auto& d : dets) {
                rectangle(frame, d.box, Scalar(0, 0, 255), 2);
                char label[64];
                snprintf(label, sizeof(label), "%s %.2f", d.name.c_str(), d.confidence);
                putText(frame, label, Point(d.box.x, d.box.y - 8),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            }
            drawUI(frame, IDLE, (int)fps_smooth, true, 0, 0);
        }
        else {
            // ---- 全屏YOLO检测（隔帧推理提升帧率）----
            vector<Detection> dets;
            yolo_frame_counter++;
            if (yolo_frame_counter > YOLO_SKIP_FRAMES || !have_cached_dets) {
                dets = detector.detect(frame);
                cached_dets = dets;
                have_cached_dets = true;
                yolo_frame_counter = 0;
            } else {
                dets = cached_dets;
            }

            bool has_target = false;
            Detection best_det;

            // 画所有检测框（绿色）
            for (auto& d : dets) {
                rectangle(frame, d.box, Scalar(0, 255, 0), 2);
                char label[64];
                snprintf(label, sizeof(label), "%s %.2f", d.name.c_str(), d.confidence);
                putText(frame, label, Point(d.box.x, d.box.y - 8),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
                if (!has_target || d.box.area() > best_det.box.area()) {
                    best_det = d;
                    has_target = true;
                }
            }

            // ---- 防抖计数 ----
            if (has_target) {
                consecutive_detection_count++;
                consecutive_no_detection_count = 0;
            } else {
                consecutive_no_detection_count++;
                consecutive_detection_count = 0;
            }

            // ---- 物体出现/消失判定 ----
            if (consecutive_detection_count >= NEED_CONSECUTIVE_FRAMES && !object_present) {
                object_present = true;
                object_start_time = steady_clock::now();
                cout << "[INFO] 检测到物体: " << best_det.name << " (conf=" << fixed << setprecision(2) << best_det.confidence << ")" << endl;
            } else if (consecutive_no_detection_count >= NEED_CONSECUTIVE_CLEAR && object_present) {
                object_present = false;
                if (cur_state != IDLE) {
                    cur_state = IDLE;
                    cout << "[INFO] 物体已移除，恢复空闲" << endl;
                }
            }

            // ---- 状态机 ----
            if (object_present) {
                stable_sec = duration<double>(steady_clock::now() - object_start_time).count();
                if (cur_state == IDLE && stable_sec >= WARNING_DELAY_SEC) {
                    cur_state = WARNING;
                    cout << "[WARNING] 物体停留 " << fixed << setprecision(1) << stable_sec << "s，预警" << endl;
                } else if (cur_state == WARNING && stable_sec >= ALARM_DELAY_SEC) {
                    cur_state = OCCUPIED;
                    cout << "[ALERT] 消防通道占用！持续 " << (int)stable_sec << "s - " << best_det.name << endl;
                    string snap = "captures/occupied_" + getCurrentTimestamp() + ".jpg";
                    imwrite(snap, frame);
                    photo_count++;
                }
            } else {
                cur_state = IDLE;
                stable_sec = 0;
            }

            if (cur_state == WARNING && object_present) {
                countdown = max(0, (int)(ALARM_DELAY_SEC - stable_sec));
            }

            // ---- 发送状态到从核 ----
            if (cur_state != last_sent_state || last_sent_fire != fire_triggered) {
                sendStateToSlave(cur_state, fire_triggered);
                last_sent_state = cur_state;
                last_sent_fire = fire_triggered;
            }

            drawUI(frame, cur_state, (int)fps_smooth, false, countdown, stable_sec);
        }

        imshow("Fire Lane Detection", frame);

        // ---- FPS ----
        double frame_time = duration<double>(steady_clock::now() - loop_start).count();
        float inst_fps = (frame_time > 0.001) ? (1.0f / (float)frame_time) : 30.0f;
        if (fps_smooth < 0.1f) fps_smooth = inst_fps;
        else fps_smooth = FPS_ALPHA * inst_fps + (1.0f - FPS_ALPHA) * fps_smooth;

        // ---- status.json ----
        if (duration<double>(steady_clock::now() - last_status_write).count() >= 1.0) {
            double uptime = duration<double>(steady_clock::now() - program_start).count();
            string state_str, state_color;
            if (fire_triggered) { state_str = "FIRE"; state_color = "#ff1744"; }
            else switch (cur_state) {
                case IDLE:     state_str = "MONITORING"; state_color = "#2ecc71"; break;
                case WARNING:  state_str = "WARNING";    state_color = "#f39c12"; break;
                case OCCUPIED: state_str = "OCCUPIED";   state_color = "#e74c3c"; break;
            }
            time_t now = time(nullptr);
            char ts[64];
            strftime(ts, sizeof(ts), "%H:%M:%S", localtime(&now));

            ofstream sf("status.json");
            sf << "{";
            sf << "\"state\":\"" << state_str << "\",";
            sf << "\"state_color\":\"" << state_color << "\",";
            sf << "\"fps\":" << (int)fps_smooth << ",";
            sf << "\"photo_count\":" << photo_count << ",";
            sf << "\"flame_detected\":" << (fire_triggered ? "true" : "false") << ",";
            sf << "\"slave_alive\":" << (isSlaveAlive() ? "true" : "false") << ",";
            sf << "\"timer\":" << fixed << setprecision(1) << stable_sec << ",";
            sf << "\"countdown\":" << countdown << ",";
            sf << "\"uptime\":" << (int)uptime << ",";
            sf << "\"timestamp\":\"" << ts << "\"";
            sf << "}";
            sf.close();
            last_status_write = steady_clock::now();
        }

        // ---- stream.jpg ----
        if (duration<double>(steady_clock::now() - last_stream_write).count() >= 0.2) {
            vector<uchar> buf;
            imencode(".jpg", frame, buf, {IMWRITE_JPEG_QUALITY, 70});
            ofstream sf("stream.jpg", ios::binary);
            sf.write((char*)buf.data(), buf.size());
            sf.close();
            last_stream_write = steady_clock::now();
        }
    }

    // ---- 退出清理 ----
    cout << "[INFO] 正在关闭..." << endl;
    sendState(CMD_EMERGENCY_STOP);
    usleep(100000);
    releaseFlameSensor();
    closeRpmsg();
    cap.release();
    destroyAllWindows();
    cout << "[INFO] 程序已退出，所有声光外设已关闭" << endl;
    return 0;
}
