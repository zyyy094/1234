#ifndef HARDWARE_H
#define HARDWARE_H

// ====================== 火焰传感器（主核直读） ======================
bool initFlameSensor();
bool isFlameDetected();
void releaseFlameSensor();

// ====================== RPMsg 通信（主→从） ======================
bool initRpmsg();
void sendCmd(char cmd);
void setLed(char color);
void setBuzzer(bool on);
void shutdownHardware();
void closeRpmsg();

// ====================== 从核回传数据接收（从→主） ======================
// 非阻塞读取从核上报的消息（火焰警报/心跳等）
void pollSlaveMessages();
// 获取从核上报的火焰状态
bool isSlaveFlameAlert();
// 发送心跳请求给从核
void sendHeartbeat();

#endif
