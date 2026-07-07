/*
 * main.cpp - 消防通道杂物占用视觉检测 + 火焰火情双判断系统
 *
 * 架构: 飞腾派异构双核
 *   主核(Linux): AI视觉推理 + 业务逻辑 + Web服务
 *   从核(裸机): LED/蜂鸣器硬件驱动 + 火焰传感器采集 + 主动上报
 *
 * 通信: RPMsg 双向
 *   主→从: G/Y/R/F/S 状态指令 + B/b 蜂鸣器 + H 心跳
 *   从→主: F/f 火焰上报 + h 心跳回复
 */
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>

#include "common.h"
#include "hardware.h"
#include "fire_protocol.h"
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

chrono::steady_clock::time_point program_start = chrono::steady_clock::now();
chrono::steady_clock::time_point last_status_write;
chrono::steady_clock::time_point last_stream_write;
chrono::steady_clock::time_point last_heartbeat_check;

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

// ====================== 发送状态到从核 ======================
void sendStateToSlave(State state, bool fire) {
    if (fire) {
        sendState(CMD_STATE_FIRE);       // 火警:红灯+常鸣
    } else {
        switch (state) {
            case IDLE:     sendState(CMD_STATE_IDLE);     break; // 绿灯
            case WARNING:  sendState(CMD_STATE_WARNING);  break; // 黄灯
            case OCCUPIED: sendState(CMD_STATE_OCCUPIED); break; // 红灯+间歇蜂鸣
        }
    }
}

// ====================== 信号处理 ======================
void signalHandler(int sig) {
    cout << "\n[INFO] 退出信号捕获" << endl;
    running = false;
    sendState(CMD_EMERGENCY_STOP);
    releaseFlameSensor();
    closeRpmsg();
    exit(0);
}

// ====================== 主函数 ======================
int main() {
    signal(SIGINT, signalHandler);
    last_buzzer_toggle = chrono::steady_clock::now();
    last_status_write = chrono::steady_clock::now();
    last_stream_write = chrono::steady_clock::now();
    last_heartbeat_check = chrono::steady_clock::now();

    system("mkdir -p captures");

    // 1. 初始化硬件
    if (!initFlameSensor()) {
        cerr << "[WARN] 火焰传感器初始化失败，继续运行（仅视觉检测）" << endl;
    }
    initRpmsg();
    sendState(CMD_EMERGENCY_STOP);  // 初始全关
    sendState(CMD_STATE_IDLE);      // 初始绿灯

    // 2. 加载 YOLO 模型
    if (!detector.loadModel("yolov8n.onnx")) {
        sendState(CMD_EMERGENCY_STOP);
        releaseFlameSensor();
        closeRpmsg();
        return -1;
    }

    // 3. 打开摄像头
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
        sendState(CMD_EMERGENCY_STOP);
        releaseFlameSensor();
        closeRpmsg();
        return -1;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    cout << "\n===== 消防通道检测系统 =====" << endl;
    cout << "架构: 异构双核 (Linux主核 + 裸机从核)" << endl;
    cout << "功能: 视觉占用检测 + 火焰双链路校验" << endl;
    cout << "操作: s:保存背景 | c:启动检测 | p:拍照 | q:退出" << endl;
    cout << "      (3秒无操作自动启动检测)" << endl;

    // 4. 自动保存背景 + 自动启动检测
    Mat frame;
    cap >> frame;
    if (!frame.empty()) {
        saveBackground(frame);
        detection_started = true;
        cout << "[INFO] 背景已自动保存，检测已自动启动" << endl;
    }

    chrono::steady_clock::time_point start_time;
    State last_sent_state = IDLE;
    bool last_sent_fire = false;

    // 5. 主循环
    int fps = 0;
    int countdown = 0;
    double stable_sec = 0;

    while (running) {
        auto loop_start = chrono::steady_clock::now();
        cap >> frame;
        if (frame.empty()) continue;

        // 键盘操作（保留手动功能）
        char key = waitKey(1);
        if (key == 'q') break;
        else if (key == 's') {
            saveBackground(frame);
            lastStaticBoxes.clear();
            timer_started = false;
            cout << "[INFO] 背景已重新保存" << endl;
        } else if (key == 'c' && bg_ready) {
            detection_started = true;
            cout << "[INFO] 检测启动" << endl;
        } else if (key == 'p') {
            string path = "captures/img_" + getCurrentTimestamp() + ".jpg";
            imwrite(path, frame);
            photo_count++;
            cout << "[INFO] 截图保存: " << path << endl;
        }

        // ===== 心跳管理 =====
        updateHeartbeat();
        pollSlaveMessages();

        // ===== 火焰检测（主核GPIO直读 + 从核RPMsg上报，双链路校验）=====
        bool flame_local = isFlameDetected();
        bool flame_slave = isSlaveFlameAlert();
        bool flame_detected = flame_local || flame_slave;

        if (flame_detected) {
            if (!fire_triggered) {
                fire_triggered = true;
                cout << "[FIRE] 火焰检测到！";
                if (flame_local && flame_slave) {
                    cout << " (双链路确认)";
                } else if (flame_slave) {
                    cout << " (从核上报)";
                } else {
                    cout << " (主核检测)";
                }
                cout << endl;

                // 火警自动抓拍
                string snap = "captures/fire_" + getCurrentTimestamp() + ".jpg";
                imwrite(snap, frame);
                photo_count++;
                cout << "[FIRE] 火警抓拍: " << snap << endl;
            }
            // 火警: 最高优先级，红灯+常鸣蜂鸣器
            sendStateToSlave(IDLE, true);
            drawOverlay(frame, cur_state, 0, true, 0, 0);
            imshow("Fire Lane Detection", frame);
            // 跳过 YOLO 检测，直接写状态文件
        } else {
            if (fire_triggered) {
                fire_triggered = false;
                setBuzzer(false);
                cur_state = IDLE;
                object_present = false;
                timer_started = false;
                cout << "[INFO] 火焰已熄灭，恢复监测" << endl;
            }
        }

        // ===== YOLO 目标检测（火警时跳过）=====
        if (!fire_triggered)
        {
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

        // 画检测框
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
            cout << "[INFO] 检测到物体: " << best_det.name << endl;
        } else if (consecutive_no_detection_count >= NEED_CONSECUTIVE_CLEAR && object_present) {
            object_present = false;
            timer_started = false;
            if (cur_state != IDLE) {
                cur_state = IDLE;
                cout << "[INFO] 物体已移除，恢复空闲" << endl;
            }
        }

        // ===== 状态机 =====
        stable_sec = 0;
        if (timer_started) {
            stable_sec = chrono::duration<double>(chrono::steady_clock::now() - start_time).count();
        }

        if (!object_present || !timer_started) {
            cur_state = IDLE;
        } else {
            if (cur_state == IDLE && stable_sec >= WARNING_DELAY_SEC) {
                cur_state = WARNING;
                cout << "[WARNING] 物体停留 " << (int)stable_sec << "s，预警" << endl;
            } else if (cur_state == WARNING && stable_sec >= ALARM_DELAY_SEC) {
                cur_state = OCCUPIED;
                cout << "[ALERT] 消防通道占用！持续 " << (int)stable_sec << "s" << endl;
                string snap = "captures/occupied_" + getCurrentTimestamp() + ".jpg";
                imwrite(snap, frame);
                photo_count++;
            }
        }

        // 状态变化时发送指令给从核
        if (cur_state != last_sent_state || fire_triggered != last_sent_fire) {
            sendStateToSlave(cur_state, fire_triggered);
            last_sent_state = cur_state;
            last_sent_fire = fire_triggered;
        }

        // OCCUPIED 状态间歇蜂鸣
        if (cur_state == OCCUPIED) {
            updateBuzzer();
        }

        // 倒计时
        countdown = 0;
        if (cur_state == WARNING && timer_started) {
            double warn_elapsed = stable_sec - WARNING_DELAY_SEC;
            countdown = max(0, (int)(ALARM_DELAY_SEC - warn_elapsed));
        }

        fps = (int)(1.0 / chrono::duration<double>(chrono::steady_clock::now() - loop_start).count());

        drawOverlay(frame, cur_state, fps, false, countdown, stable_sec);
        imshow("Fire Lane Detection", frame);
        } // end if (!fire_triggered)

        // ===== status.json 降频写入（每秒一次）=====
        if (chrono::duration<double>(chrono::steady_clock::now() - last_status_write).count() >= 1.0) {
            double uptime = chrono::duration<double>(chrono::steady_clock::now() - program_start).count();
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
            sf << "\"fps\":" << fps << ",";
            sf << "\"photo_count\":" << photo_count << ",";
            sf << "\"flame_detected\":" << (fire_triggered ? "true" : "false") << ",";
            sf << "\"slave_alive\":" << (isSlaveAlive() ? "true" : "false") << ",";
            sf << "\"timer\":" << stable_sec << ",";
            sf << "\"countdown\":" << countdown << ",";
            sf << "\"uptime\":" << (int)uptime << ",";
            sf << "\"timestamp\":\"" << ts << "\"";
            sf << "}";
            sf.close();
            last_status_write = chrono::steady_clock::now();
        }

        // ===== MJPG 推流帧（每 200ms 一帧）=====
        if (chrono::duration<double>(chrono::steady_clock::now() - last_stream_write).count() >= 0.2) {
            vector<uchar> buf;
            imencode(".jpg", frame, buf, {IMWRITE_JPEG_QUALITY, 60});
            ofstream sf("stream.jpg", ios::binary);
            sf.write((char*)buf.data(), buf.size());
            sf.close();
            last_stream_write = chrono::steady_clock::now();
        }
    }

    // 清理
    sendState(CMD_EMERGENCY_STOP);
    releaseFlameSensor();
    closeRpmsg();
    cap.release();
    destroyAllWindows();
    cout << "[INFO] 程序正常退出" << endl;
    return 0;
}
