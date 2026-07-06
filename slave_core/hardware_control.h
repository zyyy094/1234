/*
 * hardware_control.h - 裸机硬件控制（GPIO + PWM）
 *
 * 控制 LED（红/黄/绿）和蜂鸣器，由从核（FTC310）直接操作寄存器
 */
#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include "ftypes.h"

/* 初始化所有硬件（GPIO + PWM） */
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

/* 硬件释放 */
void hardware_deinit(void);

#endif /* HARDWARE_CONTROL_H */
