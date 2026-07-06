/*
 * rpmsg_handler.c - RPMsg 通信处理实现
 *
 * 基于 SDK openamp_for_linux 示例改造
 * 接收主核(Linux)的单字符命令，控制从核硬件
 * 同时定时轮询火焰传感器，主动上报状态给主核
 *
 * 架构:
 *   Linux 主核 --/dev/rpmsg0--> RPMsg virtio --> 从核回调 --> 硬件控制
 *   从核定时器 --> 轮询火焰传感器 --> 主动 rpmsg_send() --> 主核
 *
 * 协议:
 *   主→从: 单字符命令 ('R','r','Y','y','G','g','B','b','S','H')
 *   从→主: 'F' 火焰警报, 'f' 火焰解除, 'h' 心跳回复
 */

#include "rpmsg_handler.h"
#include "fire_protocol.h"
#include "hardware_control.h"

#include <stdio.h>
#include <openamp/open_amp.h>
#include <metal/alloc.h>
#include "platform_info.h"
#include "rpmsg_service.h"
#include <metal/sleep.h>
#include "rsc_table.h"
#include "fcache.h"
#include "fdebug.h"
#include "fpsci.h"
#include "openamp_configs.h"
#include "libmetal_configs.h"
#include "sdkconfig.h"

#define RPMSG_DEBUG_TAG "RPMSG"
#define RPMSG_DEBUG_I(format, ...) FT_DEBUG_PRINT_I(RPMSG_DEBUG_TAG, format, ##__VA_ARGS__)
#define RPMSG_DEBUG_W(format, ...) FT_DEBUG_PRINT_W(RPMSG_DEBUG_TAG, format, ##__VA_ARGS__)
#define RPMSG_DEBUG_E(format, ...) FT_DEBUG_PRINT_E(RPMSG_DEBUG_TAG, format, ##__VA_ARGS__)

/* ====================== 全局变量 ====================== */
static volatile int shutdown_req = 0;
static struct rpmsg_endpoint *g_ept = NULL;

struct remoteproc remoteproc_slave;
static struct rpmsg_device *rpdev_slave = NULL;

/* 火焰传感器状态追踪 */
static int last_flame_state = 0;
static u32 last_sensor_poll = 0;

/* ====================== 资源表（与 Linux 协商一致） ====================== */
static struct remote_resource_table __resource resources __attribute__((used)) = {
    1,                          /* Version */
    NUM_TABLE_ENTRIES,          /* Number of table entries */
    {0, 0},                     /* reserved */
    {
        offsetof(struct remote_resource_table, rpmsg_vdev),
    },
    /* Virtio device entry */
    {
        RSC_VDEV,
        VIRTIO_ID_RPMSG_,
        VDEV_NOTIFYID,
        RPMSG_IPU_C0_FEATURES,
        0, 0, 0,
        NUM_VRINGS,
        {0, 0},
    },
    /* Vring entries */
    {SLAVE00_TX_VRING_ADDR, VRING_ALIGN, SLAVE00_VRING_NUM, 1, 0},
    {SLAVE00_RX_VRING_ADDR, VRING_ALIGN, SLAVE00_VRING_NUM, 2, 0},
};

/* ====================== 共享内存（与 Linux 协商一致） ====================== */
static metal_phys_addr_t poll_phys_addr = SLAVE00_KICK_IO_ADDR;
struct metal_device kick_driver = {
    .name = SLAVE_00_KICK_DEV_NAME,
    .bus = NULL,
    .num_regions = 1,
    .regions = {{
        .virt = (void *)SLAVE00_KICK_IO_ADDR,
        .physmap = &poll_phys_addr,
        .size = 0x1000,
        .page_shift = -1UL,
        .page_mask = -1UL,
        .mem_flags = SLAVE00_SOURCE_TABLE_ATTRIBUTE,
        .ops = {NULL},
    }},
    .irq_num = 1,
    .irq_info = (void *)SLAVE_00_SGI,
};

struct remoteproc_priv slave_priv = {
    .kick_dev_name = SLAVE_00_KICK_DEV_NAME,
    .kick_dev_bus_name = KICK_BUS_NAME,
    .cpu_id = MASTER_CORE_MASK,
    .src_table_attribute = SLAVE00_SOURCE_TABLE_ATTRIBUTE,
    .share_mem_va = SLAVE00_SHARE_MEM_ADDR,
    .share_mem_pa = SLAVE00_SHARE_MEM_ADDR,
    .share_buffer_offset = SLAVE00_VRING_SIZE,
    .share_mem_size = SLAVE00_SHARE_MEM_SIZE,
    .share_mem_attribute = SLAVE00_SHARE_BUFFER_ATTRIBUTE,
};

/* ====================== 主动上报函数 ====================== */
static void send_to_master(char cmd)
{
    if (g_ept && is_rpmsg_ept_ready(g_ept)) {
        rpmsg_send(g_ept, &cmd, 1);
    }
}

/* 轮询火焰传感器，状态变化时主动上报 */
static void poll_flame_sensor(void)
{
    int cur_flame = flame_sensor_read();

    if (cur_flame != last_flame_state) {
        if (cur_flame) {
            /* 检测到火焰 - 主动上报 + 本地报警 */
            RPMSG_DEBUG_I("Flame detected! Alerting master...");
            send_to_master(CMD_FIRE_ALERT);

            /* 从核立即触发本地报警（不等主核命令） */
            led_red_on();
            buzzer_on();
        } else {
            /* 火焰消失 - 通知主核 + 本地解除 */
            RPMSG_DEBUG_I("Flame cleared. Notifying master...");
            send_to_master(CMD_FIRE_CLEAR);

            led_red_off();
            buzzer_off();
            led_green_on();  /* 恢复绿灯 */
        }
        last_flame_state = cur_flame;
    }
}

/* ====================== 命令处理 ====================== */
static void process_command(char cmd)
{
    switch (cmd) {
        /* LED 控制 */
        case CMD_LED_RED_ON:    led_red_on();    break;
        case CMD_LED_RED_OFF:   led_red_off();   break;
        case CMD_LED_YEL_ON:    led_yellow_on(); break;
        case CMD_LED_YEL_OFF:   led_yellow_off();break;
        case CMD_LED_GRN_ON:    led_green_on();  break;
        case CMD_LED_GRN_OFF:   led_green_off(); break;

        /* 蜂鸣器控制 */
        case CMD_BUZZER_ON:     buzzer_on();     break;
        case CMD_BUZZER_OFF:    buzzer_off();    break;

        /* 紧急停止 */
        case CMD_ALL_OFF:
            led_all_off();
            buzzer_off();
            break;

        /* 心跳请求: 回复心跳 + 当前传感器状态 */
        case CMD_HEARTBEAT:
            send_to_master(CMD_HEARTBEAT_ACK);
            /* 附带传感器状态 */
            if (last_flame_state) {
                send_to_master(CMD_FIRE_ALERT);
            }
            break;

        default:
            RPMSG_DEBUG_W("Unknown command: 0x%02x '%c'", cmd, cmd);
            break;
    }
}

/* ====================== RPMsg 回调 ====================== */
static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                             size_t len, uint32_t src, void *priv)
{
    (void)priv;
    (void)src;

    char *msg = (char *)data;

    if (len >= 1) {
        char cmd = msg[0];
        RPMSG_DEBUG_I("Recv cmd: '%c' (0x%02x)", cmd, cmd);
        process_command(cmd);
    }

    /* 回复 ACK（主核 read 会收到回显） */
    if (len >= 1) {
        char ack = msg[0];
        rpmsg_send(ept, &ack, 1);
    }

    return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    RPMSG_DEBUG_W("Remote endpoint destroyed.");
    g_ept = NULL;
    shutdown_req = 1;
}

/* ====================== RPMsg 应用主循环 ====================== */
static int rpmsg_app_run(struct rpmsg_device *rdev, void *priv)
{
    int ret;
    static struct rpmsg_endpoint lept = {0};

    shutdown_req = 0;
    RPMSG_DEBUG_I("Creating rpmsg endpoint...");

    ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME,
                           0, RPMSG_ADDR_ANY,
                           rpmsg_endpoint_cb, rpmsg_service_unbind);
    if (ret) {
        RPMSG_DEBUG_E("Failed to create endpoint: %d", ret);
        return -1;
    }

    g_ept = &lept;
    RPMSG_DEBUG_I("Endpoint created. Waiting for commands...");

    /* 等待 Linux 端连接 */
    while (!is_rpmsg_ept_ready(&lept)) {
        platform_poll(priv);
        fsleep_millisec(10);
    }

    RPMSG_DEBUG_I("Linux master connected!");
    RPMSG_DEBUG_I("Flame sensor polling active (interval=%dms)", SENSOR_POLL_INTERVAL_MS);

    /* 主循环：轮询 RPMsg 消息 + 定时轮询传感器 */
    while (1) {
        /* 处理 RPMsg 消息 */
        platform_poll(priv);

        /* 定时轮询火焰传感器 */
        u32 now = metal_get_timestamp();
        if (now - last_sensor_poll >= SENSOR_POLL_INTERVAL_MS) {
            poll_flame_sensor();
            last_sensor_poll = now;
        }

        if (shutdown_req || rproc_get_stop_flag()) {
            rproc_clear_stop_flag();
            break;
        }
    }

    rpmsg_destroy_ept(&lept);
    g_ept = NULL;
    RPMSG_DEBUG_I("Endpoint destroyed.");
    return 0;
}

/* ====================== 初始化 ====================== */
int rpmsg_handler_init(void)
{
    init_system();

    if (!platform_create_proc(&remoteproc_slave, &slave_priv, &kick_driver)) {
        RPMSG_DEBUG_E("Failed to create remoteproc");
        return -1;
    }

    remoteproc_slave.rsc_table = &resources;

    if (platform_setup_src_table(&remoteproc_slave, remoteproc_slave.rsc_table)) {
        RPMSG_DEBUG_E("Failed to setup resource table");
        return -1;
    }

    if (platform_setup_share_mems(&remoteproc_slave)) {
        RPMSG_DEBUG_E("Failed to setup shared memory");
        return -1;
    }

    rpdev_slave = platform_create_rpmsg_vdev(&remoteproc_slave, 0,
                                             VIRTIO_DEV_DEVICE, NULL, NULL);
    if (!rpdev_slave) {
        RPMSG_DEBUG_E("Failed to create rpmsg vdev");
        return -1;
    }

    /* 初始化传感器状态 */
    last_flame_state = flame_sensor_read();
    last_sensor_poll = metal_get_timestamp();

    RPMSG_DEBUG_I("RPMsg initialized. Flame sensor state: %s",
                  last_flame_state ? "FIRE" : "clear");
    return 0;
}

int rpmsg_handler_run(void)
{
    int ret = rpmsg_app_run(rpdev_slave, &remoteproc_slave);
    if (ret) {
        RPMSG_DEBUG_E("RPMsg app failed");
    }

    platform_release_rpmsg_vdev(rpdev_slave, &remoteproc_slave);
    platform_cleanup(&remoteproc_slave);
    return ret;
}

void rpmsg_handler_cleanup(void)
{
    platform_cleanup(&remoteproc_slave);
    RPMSG_DEBUG_I("RPMsg cleaned up.");
}
