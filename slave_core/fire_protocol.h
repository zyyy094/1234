/*
 * fire_protocol.h - 主从核通信协议定义
 *
 * === 主核 → 从核（单字符命令）===
 *   'R' - 红灯亮    'r' - 红灯灭
 *   'Y' - 黄灯亮    'y' - 黄灯灭
 *   'G' - 绿灯亮    'g' - 绿灯灭
 *   'B' - 蜂鸣器开  'b' - 蜂鸣器关
 *   'S' - 全部关闭（紧急停止）
 *   'H' - 心跳请求（从核回复 'h' + 传感器状态）
 *
 * === 从核 → 主核（回传数据）===
 *   'F' - 火焰警报（检测到火焰）
 *   'f' - 火焰解除（火焰消失）
 *   'h' - 心跳回复（主核据此判断从核存活）
 *   数据包格式: "F:flame=1,temp=0\n"（从核主动上报）
 */
#ifndef FIRE_PROTOCOL_H
#define FIRE_PROTOCOL_H

/* LED 命令（主→从） */
#define CMD_LED_RED_ON    'R'
#define CMD_LED_RED_OFF   'r'
#define CMD_LED_YEL_ON    'Y'
#define CMD_LED_YEL_OFF   'y'
#define CMD_LED_GRN_ON    'G'
#define CMD_LED_GRN_OFF   'g'

/* 蜂鸣器命令（主→从） */
#define CMD_BUZZER_ON     'B'
#define CMD_BUZZER_OFF    'b'

/* 紧急停止（主→从） */
#define CMD_ALL_OFF       'S'

/* 心跳请求（主→从） */
#define CMD_HEARTBEAT     'H'

/* 从核回传命令（从→主） */
#define CMD_FIRE_ALERT    'F'    /* 火焰 detected */
#define CMD_FIRE_CLEAR    'f'    /* 火焰 clear */
#define CMD_HEARTBEAT_ACK 'h'    /* 心跳回复 */

/* 传感器数据上报间隔（毫秒） */
#define SENSOR_POLL_INTERVAL_MS  200

/* RPMsg 服务名称（Linux 内核 rpmsg 驱动据此创建 /dev/rpmsg0） */
#define RPMSG_SERVICE_NAME "rpmsg-openamp-demo-channel"

#endif /* FIRE_PROTOCOL_H */
