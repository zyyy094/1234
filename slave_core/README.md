# 从核裸机程序 - 异构双核架构

## 架构概述

```
┌─────────────────────────────────────────────────────┐
│                 飞腾派 PE2204                        │
│                                                     │
│  ┌──────────────────────┐  ┌──────────────────────┐ │
│  │  主核 FTC664 x2       │  │  从核 FTC310 x2      │ │
│  │  Linux 5.10           │  │  裸机 (Bare Metal)   │ │
│  │                       │  │                      │ │
│  │  fire_detect 程序     │  │  slave_core 程序     │ │
│  │  - 摄像头 (V4L2)      │  │  - LED 控制 (GPIO)   │ │
│  │  - YOLOv8 (ONNX)     │  │  - 蜂鸣器 (PWM)      │ │
│  │  - 背景减除 (OpenCV)  │  │  - 火焰传感器 (GPIO) │ │
│  │  - 业务状态机         │  │                      │ │
│  │                       │  │                      │ │
│  │  /dev/rpmsg0 ────────┼──┼──> RPMsg 端点回调    │ │
│  │  write('R')           │  │  process_command('R')│ │
│  │  write('b')           │  │  process_command('b')│ │
│  └──────────────────────┘  └──────────────────────┘ │
│           │ RPMsg Virtio (共享内存)                   │
│           └────────────────────────────────          │
└─────────────────────────────────────────────────────┘
```

## 通信协议

主核通过 `/dev/rpmsg0` 发送单字符命令，从核接收后控制硬件:

| 命令 | ASCII | 功能 |
|------|-------|------|
| `R` | 0x52 | 红灯亮 |
| `r` | 0x72 | 红灯灭 |
| `Y` | 0x59 | 黄灯亮 |
| `y` | 0x79 | 黄灯灭 |
| `G` | 0x47 | 绿灯亮 |
| `g` | 0x67 | 绿灯灭 |
| `B` | 0x42 | 蜂鸣器开 |
| `b` | 0x62 | 蜂鸣器关 |
| `S` | 0x53 | 全部关闭 |

## 文件结构

```
slave_core/
├── main.c               # 入口: 硬件初始化 → RPMsg 初始化 → 命令循环
├── fire_protocol.h      # 通信协议定义
├── hardware_control.h   # 硬件控制接口
├── hardware_control.c   # GPIO + PWM 实现 (FGPIO4 + PWM2)
├── rpmsg_handler.h      # RPMsg 通信接口
├── rpmsg_handler.c      # RPMsg 端点 + 命令回调
└── Makefile             # 构建配置
```

## 硬件引脚分配

| 外设 | 控制器 | 引脚 | 说明 |
|------|--------|------|------|
| LED 红 | FGPIO4 | PIN 13 | 高电平亮 |
| LED 黄 | FGPIO4 | PIN 14 | 高电平亮 |
| LED 绿 | FGPIO4 | PIN 15 | 高电平亮 |
| 蜂鸣器 | PWM2 (FPWM1_ID) | CH0 (引脚 32) | 2kHz 50%占空比，无源蜂鸣器 |

## 编译步骤

### 前提条件
- SDK 目录: `phytium-standalone-sdk/`
- 交叉编译工具链: `aarch64-none-elf-gcc`

### 编译

```bash
cd slave_core

# 1. 加载飞腾派 aarch64 配置
make config_phytiumpi_aarch64

# 2. 配置（可选，通常默认即可）
make menuconfig

# 3. 编译
make clean
make image

# 输出: output/openamp_core0.elf
```

## 部署到飞腾派

### 1. 复制 ELF 到板子

```bash
scp output/openamp_core0.elf user@192.168.43.231:/lib/firmware/
```

### 2. 在板子上加载从核程序

```bash
# SSH 登录板子
ssh user@192.168.43.231

# 停止已有的从核程序（如有）
echo 'user' | sudo -S bash -c 'echo stop > /sys/class/remoteproc/remoteproc0/state'

# 加载新的 ELF
echo 'user' | sudo -S bash -c 'echo openamp_core0.elf > /sys/class/remoteproc/remoteproc0/firmware'

# 启动从核
echo 'user' | sudo -S bash -c 'echo start > /sys/class/remoteproc/remoteproc0/state'

# 验证 /dev/rpmsg0 已创建
ls -la /dev/rpmsg0
```

### 3. 运行主核程序

```bash
cd /home/user/fire_detect_project
echo 'user' | sudo -S env DISPLAY=:0 LD_LIBRARY_PATH=thirdparty/onnxruntime/lib ./build/fire_detect
```

## 从核串口日志

从核程序通过 UART 输出日志（波特率 115200）:

```
========================================
 Fire Detect - Slave Core (Bare Metal)
 Phytium Pi PE2204 - FTC310
========================================
OpenAMP version: OpenAMP(v1.0)
libmetal version: 0.1.0

[1/3] Initializing hardware...
      LED: FGPIO4 pin 13/14/15
      Buzzer: PWM2 CH0
[2/3] Initializing RPMsg...
[3/3] Waiting for Linux master commands...

RPMSG: Endpoint created. Waiting for commands...
RPMSG: Linux master connected!
RPMSG: Recv cmd: 'G' (0x47)
RPMSG: Recv cmd: 'R' (0x52)
RPMSG: Recv cmd: 'B' (0x42)
```

## 故障排查

### /dev/rpmsg0 不存在

1. 检查从核是否已启动: `cat /sys/class/remoteproc/remoteproc0/state`
2. 检查固件是否加载: `cat /sys/class/remoteproc/remoteproc0/firmware`
3. 查看内核日志: `dmesg | grep rpmsg`
4. 确认 Linux 内核已启用 RPMsg 驱动: `zcat /proc/config.gz | grep RPMSG`

### 从核串口无输出

- 飞腾派从核 UART 可能是独立的串口（非主核的 ttyS0）
- 参考《飞腾嵌入式OpenAMP技术解决方案与用户操作手册》

### LED 不亮

- 确认 FGPIO4 引脚 13/14/15 的 IO 复用配置正确
- 用万用表测量引脚电平
- 检查 LED 正负极接线
