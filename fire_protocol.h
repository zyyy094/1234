/*
 * fire_protocol.h - 主从核通信协议定义
 *
 * === 主核 → 从核（状态指令）===
 *   'G' - 状态:空闲(绿灯)
 *   'Y' - 状态:预警(黄灯)
 *   'R' - 状态:占用报警(红灯+间歇蜂鸣)
 *   'F' - 状态:火警(红灯+常鸣蜂鸣)
 *   'S' - 紧急停止(全关)
 *   'B' - 开蜂鸣器
 *   'b' - 关蜂鸣器
 *   'H' - 心跳请求
 *
 * === 从核 → 主核（回传数据）===
 *   'F' - 火焰警报（检测到火焰）
 *   'f' - 火焰解除（火焰消失）
 *   'h' - 心跳回复
 */
#ifndef FIRE_PROTOCOL_H
#define FIRE_PROTOCOL_H

/* 状态指令（主→从）：从核收到后自动设置 LED + 蜂鸣器模式 */
#define CMD_STATE_IDLE     'G'    /* 空闲:绿灯,蜂鸣器关 */
#define CMD_STATE_WARNING  'Y'    /* 预警:黄灯,蜂鸣器关 */
#define CMD_STATE_OCCUPIED 'R'    /* 占用:红灯,间歇蜂鸣 */
#define CMD_STATE_FIRE     'F'    /* 火警:红灯,常鸣蜂鸣 */
#define CMD_EMERGENCY_STOP 'S'    /* 紧急停止:全关 */

/* 独立蜂鸣器控制 */
#define CMD_BUZZER_ON      'B'
#define CMD_BUZZER_OFF     'b'

/* 心跳 */
#define CMD_HEARTBEAT      'H'
#define CMD_HEARTBEAT_ACK  'h'

/* 从核→主核 火焰上报 */
#define CMD_FIRE_ALERT     'F'    /* 注意:与状态火警同名,从核发,主核收 */
#define CMD_FIRE_CLEAR     'f'

/* 传感器轮询间隔 */
#define SENSOR_POLL_INTERVAL_MS  200

/* 心跳间隔（主核每 3 秒发一次） */
#define HEARTBEAT_INTERVAL_SEC   3.0

/* 心跳超时（超过 10 秒无回复判定从核离线） */
#define HEARTBEAT_TIMEOUT_SEC    10.0

/* RPMsg 服务名称 */
#define RPMSG_SERVICE_NAME "rpmsg-openamp-demo-channel"

#endif /* FIRE_PROTOCOL_H */
