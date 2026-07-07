#ifndef HARDWARE_H
#define HARDWARE_H

#include "fire_protocol.h"

// ====================== 火焰传感器（主核直读，兼容模式） ======================
bool initFlameSensor();
bool isFlameDetected();
void releaseFlameSensor();

// ====================== RPMsg 通信（主→从） ======================
bool initRpmsg();
// 发送状态指令（G/Y/R/F/S），从核自动设置LED+蜂鸣器
void sendState(char state_cmd);
// 独立蜂鸣器控制
void setBuzzer(bool on);
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
