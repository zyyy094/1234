/*
 * hardware_control.h - 裸机硬件控制（GPIO + PWM + 传感器）
 *
 * 控制 LED（红/黄/绿）、蜂鸣器、读取火焰传感器
 * 由从核（FTC310）直接操作寄存器
 */
#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include "ftypes.h"

/* 初始化所有硬件（GPIO + PWM + 传感器） */
int hardware_init(void);

/* LED 控制 */
void led_red_on(void);
void led_red_off(void);
void led_yellow_on(void);
void led_yellow_off(void);
void led_green_on(void);
void led_green_off(void);
void led_all_off(void);

/* 蜂鸣器控制 */
void buzzer_on(void);
void buzzer_off(void);

/* 火焰传感器（GPIO 输入）
 * 飞腾派: DO 口接引脚7 = GPIO2 PIN10
 * 传感器输出: 有火焰时低电平(0)，无火焰时高电平(1)
 * 返回: 1=检测到火焰, 0=无火焰
 */
int flame_sensor_read(void);

/* 硬件释放 */
void hardware_deinit(void);

#endif /* HARDWARE_CONTROL_H */
