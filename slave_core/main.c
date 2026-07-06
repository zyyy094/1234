/*
 * main.c - 从核裸机程序入口
 *
 * 飞腾派异构双核架构:
 *   主核 (FTC664 x2, Linux): fire_detect 程序
 *   从核 (FTC310 x2, 裸机): 本程序
 *
 * 启动流程:
 *   1. Linux 通过 remoteproc 加载本 ELF 到从核
 *   2. 从核启动: 硬件初始化 → RPMsg 初始化 → 命令循环
 *   3. 主核 fire_detect 打开 /dev/rpmsg0 发送命令
 *   4. 从核收到命令 → 控制 LED/蜂鸣器
 *   5. 从核定时轮询火焰传感器 → 主动上报给主核
 *
 * 双向通信:
 *   主→从: 'R'/'r'/'Y'/'y'/'G'/'g'/'B'/'b'/'S'/'H'
 *   从→主: 'F' 火焰警报, 'f' 火焰解除, 'h' 心跳回复
 */

#include <stdio.h>
#include "hardware_control.h"
#include "rpmsg_handler.h"
#include "fdebug.h"
#include "fpsci.h"
#include "fcache.h"
#include "openamp/version.h"
#include "metal/version.h"

int main(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf(" Fire Detect - Slave Core (Bare Metal)\r\n");
    printf(" Phytium Pi PE2204 - FTC310\r\n");
    printf("========================================\r\n");
    printf("OpenAMP version: %s\r\n", openamp_version());
    printf("libmetal version: %s\r\n", metal_ver());
    printf("Build: %s %s\r\n", __DATE__, __TIME__);
    printf("\r\n");

    /* 1. 初始化硬件 (GPIO + PWM) */
    printf("[1/3] Initializing hardware...\r\n");
    if (hardware_init() != 0) {
        printf("[ERROR] Hardware init failed!\r\n");
        goto error;
    }
    printf("      LED: FGPIO4 pin 13/14/15\r\n");
    printf("      Buzzer: PWM2 CH0\r\n");
    printf("      Flame sensor: FGPIO2 pin 10\r\n");

    /* 2. 初始化 RPMsg 通信 */
    printf("[2/3] Initializing RPMsg...\r\n");
    if (rpmsg_handler_init() != 0) {
        printf("[ERROR] RPMsg init failed!\r\n");
        goto error_hw;
    }

    /* 3. 运行命令循环 */
    printf("[3/3] Waiting for Linux master commands...\r\n");
    printf("      Service: %s\r\n", RPMSG_SERVICE_NAME);
    printf("\r\n");

    rpmsg_handler_run();

    /* 正常退出 */
    printf("\r\nSlave core shutting down...\r\n");
    rpmsg_handler_cleanup();
    hardware_deinit();
    printf("Goodbye.\r\n");

    FPsciCpuOff();
    return 0;

error_hw:
    hardware_deinit();
error:
    printf("[FATAL] Initialization failed, halting.\r\n");
    FPsciCpuOff();
    return -1;
}
