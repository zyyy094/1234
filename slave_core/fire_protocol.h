/*
 * fire_protocol.h - 主从核通信协议定义
 *
 * 协议格式: 单字符命令（与 Linux 端 hardware.cpp 兼容）
 *
 * 命令字:
 *   'R' - 红灯亮    'r' - 红灯灭
 *   'Y' - 黄灯亮    'y' - 黄灯灭
 *   'G' - 绿灯亮    'g' - 绿灯灭
 *   'B' - 蜂鸣器开  'b' - 蜂鸣器关
 *   'S' - 全部关闭（紧急停止）
 */
#ifndef FIRE_PROTOCOL_H
#define FIRE_PROTOCOL_H

/* LED 命令 */
#define CMD_LED_RED_ON    'R'
#define CMD_LED_RED_OFF   'r'
#define CMD_LED_YEL_ON    'Y'
#define CMD_LED_YEL_OFF   'y'
#define CMD_LED_GRN_ON    'G'
#define CMD_LED_GRN_OFF   'g'

/* 蜂鸣器命令 */
#define CMD_BUZZER_ON     'B'
#define CMD_BUZZER_OFF    'b'

/* 紧急停止 */
#define CMD_ALL_OFF       'S'

/* RPMsg 服务名称（Linux 内核 rpmsg 驱动据此创建 /dev/rpmsg0） */
#define RPMSG_SERVICE_NAME "rpmsg-openamp-demo-channel"

#endif /* FIRE_PROTOCOL_H */
