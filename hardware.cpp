#include "hardware.h"
#include "common.h"
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <poll.h>

using namespace std;

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

// ====================== RPMsg ======================
static int rpmsg_fd = -1;
static bool slave_flame_alert = false;  // 从核上报的火焰状态

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

// ====================== 从核回传数据接收 ======================

// 非阻塞读取从核上报的消息
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
                case 'F':  // 从核上报: 火焰警报
                    slave_flame_alert = true;
                    cout << "[SLAVE] 火焰警报!" << endl;
                    break;
                case 'f':  // 从核上报: 火焰解除
                    slave_flame_alert = false;
                    cout << "[SLAVE] 火焰解除" << endl;
                    break;
                case 'h':  // 从核心跳回复
                    // 心跳正常，无需处理
                    break;
                default:
                    // 其他回显数据，忽略
                    break;
            }
        }
    }
}

// 获取从核上报的火焰状态
bool isSlaveFlameAlert() {
    return slave_flame_alert;
}

// 发送心跳请求给从核
void sendHeartbeat() {
    sendCmd('H');
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
