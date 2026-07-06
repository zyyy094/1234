#ifndef HARDWARE_H
#define HARDWARE_H

#include <gpiod.h>
#include <unistd.h>
#include "common.h"

extern int rpmsg_fd;
extern bool running;
extern bool fire_triggered;
extern std::chrono::steady_clock::time_point last_buzzer_toggle;
extern bool buzzer_state;

// GPIO初始化与释放
bool init_flame_gpio();
void release_flame_gpio();
bool flame_detected();

// RPMsg硬件控制
void send_cmd(char cmd);
void set_led(char color);
void set_buzzer(bool on);
void shutdown_all_hardware();
void signal_handler(int sig);

// 蜂鸣器间歇报警
void update_buzzer_intermittent();

#endif