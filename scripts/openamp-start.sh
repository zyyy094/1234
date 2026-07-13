#!/bin/bash
# openamp-start.sh - 启动从核固件并创建 /dev/rpmsg0
# 被 systemd 服务 openamp-init.service 调用

set -e

# 0. 预启动: 禁用 PWM2 输出，防止蜂鸣器一上电就响
# PIN_32 = PWM2_OUT = FPWM1 channel 0
if [ -d /sys/class/pwm/pwmchip0 ]; then
    echo 0 > /sys/class/pwm/pwmchip0/export 2>/dev/null || true
    echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable 2>/dev/null || true
    echo "[openamp] PWM2 已禁用"
fi

# 确保从核停在安全状态（如果之前已启动）
echo stop > /sys/class/remoteproc/remoteproc0/state 2>/dev/null || true
sleep 1

# 1. 启动从核固件
echo start > /sys/class/remoteproc/remoteproc0/state
sleep 2

# 2. 绑定 rpmsg_chrdev 驱动创建 /dev/rpmsg0
CHANNEL=$(ls /sys/bus/rpmsg/devices/ | grep "rpmsg-openamp-demo-channel" | head -1)
if [ -n "$CHANNEL" ]; then
    echo rpmsg_chrdev > /sys/bus/rpmsg/devices/$CHANNEL/driver_override 2>/dev/null || true
    echo $CHANNEL > /sys/bus/rpmsg/drivers/rpmsg_chrdev/bind 2>/dev/null || true
fi

# 3. 修改 /dev/rpmsg0 权限
chmod 666 /dev/rpmsg0 2>/dev/null || true

echo "[openamp] 从核固件已启动, /dev/rpmsg0 已创建"
