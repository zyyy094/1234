#ifndef HARDWARE_H
#define HARDWARE_H

bool initFlameSensor();
bool isFlameDetected();
void releaseFlameSensor();

bool initRpmsg();
void sendCmd(char cmd);
void setLed(char color);
void setBuzzer(bool on);
void shutdownHardware();
void closeRpmsg();

#endif