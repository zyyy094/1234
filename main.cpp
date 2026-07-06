#include "common.h"
#include "hardware.h"
#include "yolov8_infer.h"
#include "bg_subtract.h"
#include "ui_draw.h"
#include <iostream>

int main() {
    signal(SIGINT, signal_handler);
    last_buzzer_toggle = std::chrono::steady_clock::now();

    system("mkdir -p /home/user/captured");

    // 硬件初始化
    if (!init_flame_gpio()) {
        std::cerr << "[FATAL] 火焰传感器GPIO初始化失败，退出程序" << std::endl;
        return -1;
    }

    rpmsg_fd = open("/dev/rpmsg0", O_RDWR);
    if (rpmsg_fd < 0) {
        for (int i = 1; i <= 3; i++) {
            std::string path = "/dev/rpmsg" + std::to_string(i);
            rpmsg_fd = open(path.c_str(), O_RDWR);
            if (rpmsg_fd >= 0) {
                std::cout << "[INFO] 打开 rpmsg: " << path << std::endl;
                break;
            }
        }
    }
    if (rpmsg_fd >= 0) shutdown_all_hardware();

    // 加载YOLO模型
    YoloInfer infer("yolov8n.onnx", 2);
    std::cout << "[INFO] YOLO模型加载成功" << std::endl;

    // 打开摄像头
    cv::VideoCapture cap;
    bool cam_ok = false;
    for (int i = 0; i <= 3; i++) {
        if (cap.open(i, cv::CAP_V4L2)) {
            std::cout << "[INFO] 打开摄像头 /dev/video" << i << std::endl;
            cam_ok = true;
            break;
        }
    }
    if (!cam_ok) {
        std::cerr << "[ERROR] 摄像头打开失败" << std::endl;
        shutdown_all_hardware();
        if (rpmsg_fd >= 0) close(rpmsg_fd);
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::cout << "\n========== 消防通道检测系统 ===========" << std::endl;
    std::cout << "静止3秒进入预警，再5秒触发声光报警" << std::endl;
    std::cout << "s:保存背景  c:启动检测  p:拍照  q:退出" << std::endl;
    std::cout << "======================================\n" << std::endl;

    // 全局业务变量
    State cur_state = IDLE;
    std::chrono::steady_clock::time_point object_stable_start;
    bool object_present = false;
    int consecutive_detection_count = 0;
    const int NEED_CONSECUTIVE_FRAMES = 3;
    int consecutive_no_detection_count = 0;
    const int NEED_CONSECUTIVE_CLEAR = 3;
    std::vector<cv::Rect> lastStaticBoxes;
    int photo_count = 0;

    cv::Mat frame;
    while (running) {
        auto start = std::chrono::steady_clock::now();
        if (!cap.read(frame)) {
            std::cerr << "[WARN] 摄像头断流，尝试重连" << std::endl;
            cap.release();
            usleep(1000000);
            cap.open(0, cv::CAP_V4L2);
            continue;
        }

        char key = cv::waitKey(1);
        if (key == 'q') break;
        else if (key == 's' && !detection_started) {
            background = frame.clone();
            cvtColor(background, background, cv::COLOR_BGR2GRAY);
            bg_ready = true;
            lastStaticBoxes.clear();
            consecutive_detection_count = 0;
            consecutive_no_detection_count = 0;
            std::cout << "[INFO] 背景保存完成，请按c启动检测" << std::endl;
        }
        else if (key == 'c' && bg_ready && !detection_started) {
            detection_started = true;
            std::cout << "[INFO] 目标检测任务已启动" << std::endl;
        }
        else if (key == 'p') {
            std::string path = "/home/user/captured/img_" + getCurrentTimestamp() + ".jpg";
            cv::imwrite(path, frame);
            photo_count++;
            std::cout << "[INFO] 已保存截图：" << path << " 累计：" << photo_count << std::endl;
        }

        bool fire = flame_detected();
        if (fire) {
            if (!fire_triggered) {
                fire_triggered = true;
                std::cout << "[FIRE ALERT] 火焰传感器触发紧急报警" << std::endl;
            }
            set_led('R');
            set_buzzer(true);
            draw_overlay(frame, cur_state, 0, true, 0, 0, photo_count);
            cv::imshow("Fire Lane Detection", frame);
            continue;
        } else {
            if (fire_triggered) {
                fire_triggered = false;
                set_buzzer(false);
                set_led('G');
                object_present = false;
                cur_state = IDLE;
                consecutive_detection_count = 0;
                consecutive_no_detection_count = 0;
                std::cout << "[INFO] 火焰消失，系统恢复空闲" << std::endl;
            }
        }

        cv::Rect diff_roi;
        bool has_diff = detection_started && bg_ready && get_diff_roi(frame, diff_roi);
        std::vector<Detection> raw_dets;

        if (has_diff && diff_roi.width > 30 && diff_roi.height > 30) {
            diff_roi.x = std::max(0, diff_roi.x - 20);
            diff_roi.y = std::max(0, diff_roi.y - 20);
            diff_roi.width = std::min(frame.cols - diff_roi.x, diff_roi.width + 40);
            diff_roi.height = std::min(frame.rows - diff_roi.y, diff_roi.height + 40);

            cv::Mat roi = frame(diff_roi);
            try {
                raw_dets = infer.infer(roi);
                for (auto& d : raw_dets) {
                    d.box.x += diff_roi.x;
                    d.box.y += diff_roi.y;
                }
            } catch (...) {
                std::cerr << "[WARN] 模型推理异常，跳过当前帧" << std::endl;
            }
        }

        std::vector<Detection> static_dets = filterStaticDetections(raw_dets, lastStaticBoxes);
        Detection best_det;
        bool has_target = false;
        for (auto& d : static_dets) {
            if (!has_target || d.box.area() > best_det.box.area()) {
                best_det = d;
                has_target = true;
            }
        }

        if (has_target) {
            cv::rectangle(frame, best_det.box, cv::Scalar(0, 255, 0), 3);
            cv::putText(frame, best_det.name, cv::Point(best_det.box.x, best_det.box.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        if (has_target) {
            consecutive_detection_count++;
            consecutive_no_detection_count = 0;
        } else {
            consecutive_no_detection_count++;
            consecutive_detection_count = 0;
        }

        if (consecutive_detection_count >= NEED_CONSECUTIVE_FRAMES && !object_present) {
            object_present = true;
            object_stable_start = std::chrono::steady_clock::now();
            std::cout << "[INFO] 检测到静止障碍物，开始计时" << std::endl;
        } else if (consecutive_no_detection_count >= NEED_CONSECUTIVE_CLEAR && object_present) {
            object_present = false;
            cur_state = IDLE;
            set_led('G');
            set_buzzer(false);
            std::cout << "[INFO] 障碍物已移除，系统空闲" << std::endl;
        }

        double stable_sec = 0;
        if (object_present) {
            stable_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - object_stable_start).count();
        }

        if (!object_present) {
            cur_state = IDLE;
            set_led('G');
            set_buzzer(false);
        } else {
            if (cur_state == IDLE) {
                if (stable_sec >= 3.0) {
                    cur_state = WARNING;
                    set_led('Y');
                    std::cout << "[WARNING] 障碍物静止3秒，进入预警阶段" << std::endl;
                }
            } else if (cur_state == WARNING) {
                double warn_time = stable_sec - 3.0;
                if (warn_time >= 5.0) {
                    cur_state = OCCUPIED;
                    set_led('R');
                    std::cout << "[ALERT] 超时未清理，启动声光报警" << std::endl;
                    std::string snap = "occupied_" + std::to_string(time(nullptr)) + ".jpg";
                    cv::imwrite(snap, frame);
                    std::cout << "[INFO] 报警截图保存：" << snap << std::endl;
                } else {
                    set_led('Y');
                }
            } else if (cur_state == OCCUPIED) {
                set_led('R');
                update_buzzer_intermittent();
            }
        }

        int warn_cnt = 0;
        if (cur_state == WARNING && object_present) {
            double warn_elapse = stable_sec - 3.0;
            warn_cnt = std::max(0, static_cast<int>(5.0 - warn_elapse));
        }

        int fps = static_cast<int>(1.0 / std::duration<double>(std::chrono::steady_clock::now() - start).count());
        draw_overlay(frame, cur_state, fps, false, warn_cnt, stable_sec, photo_count);
        cv::imshow("Fire Lane Detection", frame);
    }

    shutdown_all_hardware();
    if (rpmsg_fd >= 0) close(rpmsg_fd);
    cap.release();
    cv::destroyAllWindows();
    std::cout << "[INFO] 程序正常退出" << std::endl;
    return 0;
}