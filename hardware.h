#ifndef HARDWARE_H
#define HARDWARE_H

#include "fire_protocol.h"

// ====================== 火焰传感器（主核GPIO直读，双链路校验） ======================
bool initFlameSensor();
bool isFlameDetected();
void releaseFlameSensor();

// ====================== RPMsg 通信（主→从：状态指令） ======================
bool initRpmsg();
// 发送状态指令（G/Y/R/F/S/B/b/H），从核自动控制LED+蜂鸣器
void sendState(char state_cmd);
// 独立蜂鸣器控制（发B/b）
void setBuzzer(bool on);
// 蜂鸣器间歇更新（保留接口，从核自动处理间歇逻辑）
void updateBuzzer();
// 紧急停止
void shutdownHardware();
void closeRpmsg();

// ====================== 从核回传数据接收（从→主） ======================
void pollSlaveMessages();
bool isSlaveFlameAlert();
bool isSlaveAlive();

// 心跳管理
void sendHeartbeat();
void updateHeartbeat();

#endif
