#include "hardware.h"
#include <fcntl.h>
#include <iostream>
#include <signal.h>

int rpmsg_fd = -1;
bool running = true;
bool fire_triggered = false;
std::chrono::steady_clock::time_point last_buzzer_toggle;
bool buzzer_state = false;

static struct gpiod_chip *g_flame_chip = nullptr;
static struct gpiod_line *g_flame_line = nullptr;

bool init_flame_gpio() {
    g_flame_chip = gpiod_chip_open_by_number(FLAME_CHIP);
    if (!g_flame_chip) {
        std::cerr << "[ERROR] 打开GPIO芯片失败" << std::endl;
        return false;
    }
    g_flame_line = gpiod_chip_get_line(g_flame_chip, FLAME_LINE);
    if (!g_flame_line) {
        gpiod_chip_close(g_flame_chip);
        g_flame_chip = nullptr;
        std::cerr << "[ERROR] 获取GPIO引脚失败" << std::endl;
        return false;
    }
    int ret = gpiod_line_request_input(g_flame_line, "flame_sensor");
    if (ret < 0) {
        gpiod_chip_close(g_flame_chip);
        g_flame_chip = nullptr;
        g_flame_line = nullptr;
        std::cerr << "[ERROR] GPIO输入模式申请失败" << std::endl;
        return false;
    }
    std::cout << "[INFO] 火焰传感器GPIO初始化成功" << std::endl;
    return true;
}

void release_flame_gpio() {
    if (g_flame_line) {
        gpiod_line_release(g_flame_line);
        g_flame_line = nullptr;
    }
    if (g_flame_chip) {
        gpiod_chip_close(g_flame_chip);
        g_flame_chip = nullptr;
    }
}

bool flame_detected() {
    if (!g_flame_line) return false;
    int val = gpiod_line_get_value(g_flame_line);
    return (val == 0);
}

void send_cmd(char cmd) {
    if (rpmsg_fd >= 0) {
        write(rpmsg_fd, &cmd, 1);
        usleep(1000);
    }
}

void set_led(char color) {
    send_cmd('r');
    send_cmd('y');
    send_cmd('g');
    if (color == 'R') send_cmd('R');
    else if (color == 'Y') send_cmd('Y');
    else if (color == 'G') send_cmd('G');
}

void set_buzzer(bool on) {
    send_cmd(on ? 'B' : 'b');
}

void shutdown_all_hardware() {
    std::cout << "[INFO] 关闭所有外设硬件" << std::endl;
    if (rpmsg_fd >= 0) {
        send_cmd('r');
        send_cmd('y');
        send_cmd('g');
        send_cmd('b');
        usleep(100000);
    }
    release_flame_gpio();
}

void signal_handler(int sig) {
    std::cout << "\n[INFO] 捕获退出信号，安全退出" << std::endl;
    running = false;
    shutdown_all_hardware();
    if (rpmsg_fd >= 0) {
        close(rpmsg_fd);
        rpmsg_fd = -1;
    }
    exit(EXIT_SUCCESS);
}

void update_buzzer_intermittent() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_buzzer_toggle).count();
    if (elapsed >= 0.5) {
        buzzer_state = !buzzer_state;
        set_buzzer(buzzer_state);
        last_buzzer_toggle = now;
    }
}