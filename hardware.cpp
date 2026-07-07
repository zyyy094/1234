#include "hardware.h"
#include "common.h"
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <poll.h>
#include <chrono>

using namespace std;
using namespace chrono;

// ====================== GPIO 火焰传感器（主核直读，兼容模式） ======================
static struct gpiod_chip* g_chip = nullptr;
static struct gpiod_line* g_line = nullptr;

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

// ====================== RPMsg 通信 ======================
static int rpmsg_fd = -1;
static bool slave_flame_alert = false;
static bool slave_alive = false;
static steady_clock::time_point last_heartbeat_send;
static steady_clock::time_point last_heartbeat_ack;

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
        cerr << "[WARN] 无法打开 rpmsg 设备，从核控制不可用" << endl;
        return false;
    }
    slave_alive = true;
    last_heartbeat_send = steady_clock::now();
    last_heartbeat_ack = steady_clock::now();
    cout << "[INFO] RPMsg 通信就绪 (/dev/rpmsg0)" << endl;
    return true;
}

// 发送状态指令：G/Y/R/F/S，从核收到后自动设置 LED + 蜂鸣器模式
void sendState(char state_cmd) {
    if (rpmsg_fd >= 0) {
        write(rpmsg_fd, &state_cmd, 1);
        usleep(1000);
    }
}

// 独立蜂鸣器控制
void setBuzzer(bool on) {
    sendState(on ? CMD_BUZZER_ON : CMD_BUZZER_OFF);
}

// 紧急停止：发 S 全关
void shutdownHardware() {
    sendState(CMD_EMERGENCY_STOP);
    usleep(50000);
}

void closeRpmsg() {
    if (rpmsg_fd >= 0) {
        close(rpmsg_fd);
        rpmsg_fd = -1;
    }
}

// ====================== 从核回传数据接收 ======================
void pollSlaveMessages() {
    if (rpmsg_fd < 0) return;

    struct pollfd pfd;
    pfd.fd = rpmsg_fd;
    pfd.events = POLLIN;

    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        char buf[64];
        ssize_t n = read(rpmsg_fd, buf, sizeof(buf));
        if (n <= 0) break;

        for (ssize_t i = 0; i < n; i++) {
            char cmd = buf[i];
            switch (cmd) {
                case CMD_FIRE_ALERT:  // 'F' 从核上报火焰
                    slave_flame_alert = true;
                    cout << "[SLAVE→MASTER] 火焰警报上报!" << endl;
                    break;
                case CMD_FIRE_CLEAR:  // 'f' 从核上报火焰解除
                    slave_flame_alert = false;
                    cout << "[SLAVE→MASTER] 火焰解除" << endl;
                    break;
                case CMD_HEARTBEAT_ACK:  // 'h' 心跳回复
                    slave_alive = true;
                    last_heartbeat_ack = steady_clock::now();
                    break;
                default:
                    // 回显数据或其他，忽略
                    break;
            }
        }
    }
}

bool isSlaveFlameAlert() { return slave_flame_alert; }
bool isSlaveAlive() { return slave_alive; }

// ====================== 心跳管理 ======================
void sendHeartbeat() {
    sendState(CMD_HEARTBEAT);
    last_heartbeat_send = steady_clock::now();
}

void updateHeartbeat() {
    // 检查心跳超时
    double since_ack = duration<double>(steady_clock::now() - last_heartbeat_ack).count();
    if (since_ack > HEARTBEAT_TIMEOUT_SEC) {
        if (slave_alive) {
            cerr << "[WARN] 从核心跳超时 (" << (int)since_ack << "s)，判定离线" << endl;
            slave_alive = false;
        }
    }
    // 定期发送心跳
    double since_send = duration<double>(steady_clock::now() - last_heartbeat_send).count();
    if (since_send >= HEARTBEAT_INTERVAL_SEC) {
        sendHeartbeat();
    }
}
