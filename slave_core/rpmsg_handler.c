/*
 * rpmsg_handler.c - RPMsg 通信处理实现
 *
 * 基于 SDK openamp_for_linux 示例改造
 *
 * 功能:
 *   1. 接收主核状态指令 G/Y/R/F/S，自动设置 LED + 蜂鸣器模式
 *   2. 接收 B/b 独立蜂鸣器控制
 *   3. 接收 H 心跳请求，回复 h
 *   4. 定时轮询火焰传感器，主动上报 F/f
 *   5. 硬件看门狗保护，防止 IO 卡死
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

/* 看门狗计数器 */
static u32 watchdog_counter = 0;
#define WATCHDOG_TIMEOUT  15000  /* 15秒无心跳重启 */

/* 当前蜂鸣器模式 */
typedef enum {
    BUZZER_OFF,
    BUZZER_INTERMITTENT,  /* 间歇 (0.5s on/off) */
    BUZZER_CONSTANT       /* 常鸣 */
} buzzer_mode_t;
static buzzer_mode_t buzzer_mode = BUZZER_OFF;
static u32 last_buzzer_toggle = 0;
static int buzzer_on_state = 0;

/* ====================== 资源表 ====================== */
static struct remote_resource_table __resource resources __attribute__((used)) = {
    1, NUM_TABLE_ENTRIES, {0, 0},
    { offsetof(struct remote_resource_table, rpmsg_vdev), },
    { RSC_VDEV, VIRTIO_ID_RPMSG_, VDEV_NOTIFYID, RPMSG_IPU_C0_FEATURES,
      0, 0, 0, NUM_VRINGS, {0, 0}, },
    {SLAVE00_TX_VRING_ADDR, VRING_ALIGN, SLAVE00_VRING_NUM, 1, 0},
    {SLAVE00_RX_VRING_ADDR, VRING_ALIGN, SLAVE00_VRING_NUM, 2, 0},
};

static metal_phys_addr_t poll_phys_addr = SLAVE00_KICK_IO_ADDR;
struct metal_device kick_driver = {
    .name = SLAVE_00_KICK_DEV_NAME, .bus = NULL, .num_regions = 1,
    .regions = {{ .virt = (void *)SLAVE00_KICK_IO_ADDR, .physmap = &poll_phys_addr,
                  .size = 0x1000, .page_shift = -1UL, .page_mask = -1UL,
                  .mem_flags = SLAVE00_SOURCE_TABLE_ATTRIBUTE, .ops = {NULL}, }},
    .irq_num = 1, .irq_info = (void *)SLAVE_00_SGI,
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

/* ====================== 主动上报 ====================== */
static void send_to_master(char cmd) {
    if (g_ept && is_rpmsg_ept_ready(g_ept)) {
        rpmsg_send(g_ept, &cmd, 1);
    }
}

/* ====================== 蜂鸣器模式控制 ====================== */
static void update_buzzer(void) {
    u32 now = metal_get_timestamp();

    switch (buzzer_mode) {
        case BUZZER_OFF:
            if (buzzer_on_state) { buzzer_off(); buzzer_on_state = 0; }
            break;
        case BUZZER_INTERMITTENT:
            if (now - last_buzzer_toggle >= 500) {
                buzzer_on_state = !buzzer_on_state;
                if (buzzer_on_state) buzzer_on(); else buzzer_off();
                last_buzzer_toggle = now;
            }
            break;
        case BUZZER_CONSTANT:
            if (!buzzer_on_state) { buzzer_on(); buzzer_on_state = 1; }
            break;
    }
}

/* ====================== 火焰传感器轮询 ====================== */
static void poll_flame_sensor(void) {
    int cur_flame = flame_sensor_read();

    if (cur_flame != last_flame_state) {
        if (cur_flame) {
            RPMSG_DEBUG_I("Flame detected! Alerting master...");
            send_to_master(CMD_FIRE_ALERT);
            /* 从核立即触发本地报警（不等主核命令） */
            led_red_on();
            buzzer_mode = BUZZER_CONSTANT;
        } else {
            RPMSG_DEBUG_I("Flame cleared. Notifying master...");
            send_to_master(CMD_FIRE_CLEAR);
            led_red_off();
            buzzer_mode = BUZZER_OFF;
            buzzer_off();
            led_green_on();
        }
        last_flame_state = cur_flame;
    }
}

/* ====================== 命令处理 ====================== */
static void process_command(char cmd) {
    /* 收到任何命令，重置看门狗 */
    watchdog_counter = 0;

    switch (cmd) {
        /* 状态指令: 从核自动设置 LED + 蜂鸣器模式 */
        case CMD_STATE_IDLE:        /* 'G' 空闲:绿灯,蜂鸣器关 */
            led_all_off();
            led_green_on();
            buzzer_mode = BUZZER_OFF;
            break;

        case CMD_STATE_WARNING:     /* 'Y' 预警:黄灯,蜂鸣器关 */
            led_all_off();
            led_yellow_on();
            buzzer_mode = BUZZER_OFF;
            break;

        case CMD_STATE_OCCUPIED:    /* 'R' 占用:红灯,间歇蜂鸣 */
            led_all_off();
            led_red_on();
            buzzer_mode = BUZZER_INTERMITTENT;
            break;

        case CMD_STATE_FIRE:        /* 'F' 火警:红灯,常鸣 */
            led_all_off();
            led_red_on();
            buzzer_mode = BUZZER_CONSTANT;
            break;

        case CMD_EMERGENCY_STOP:    /* 'S' 紧急停止:全关 */
            led_all_off();
            buzzer_mode = BUZZER_OFF;
            buzzer_off();
            break;

        /* 独立蜂鸣器控制 */
        case CMD_BUZZER_ON:
            buzzer_mode = BUZZER_CONSTANT;
            break;
        case CMD_BUZZER_OFF:
            buzzer_mode = BUZZER_OFF;
            buzzer_off();
            break;

        /* 心跳请求 */
        case CMD_HEARTBEAT:
            send_to_master(CMD_HEARTBEAT_ACK);
            if (last_flame_state) {
                send_to_master(CMD_FIRE_ALERT);
            }
            break;

        default:
            RPMSG_DEBUG_W("Unknown cmd: 0x%02x '%c'", cmd, cmd);
            break;
    }
}

/* ====================== RPMsg 回调 ====================== */
static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                             size_t len, uint32_t src, void *priv) {
    (void)priv; (void)src;
    char *msg = (char *)data;
    if (len >= 1) {
        process_command(msg[0]);
    }
    return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept) {
    (void)ept;
    RPMSG_DEBUG_W("Remote endpoint destroyed.");
    g_ept = NULL;
    shutdown_req = 1;
}

/* ====================== RPMsg 主循环 ====================== */
static int rpmsg_app_run(struct rpmsg_device *rdev, void *priv) {
    int ret;
    static struct rpmsg_endpoint lept = {0};

    shutdown_req = 0;
    ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME,
                           0, RPMSG_ADDR_ANY, rpmsg_endpoint_cb, rpmsg_service_unbind);
    if (ret) {
        RPMSG_DEBUG_E("Failed to create endpoint: %d", ret);
        return -1;
    }
    g_ept = &lept;

    while (!is_rpmsg_ept_ready(&lept)) {
        platform_poll(priv);
        fsleep_millisec(10);
    }
    RPMSG_DEBUG_I("Master connected. Flame polling active.");

    while (1) {
        platform_poll(priv);

        u32 now = metal_get_timestamp();

        /* 定时轮询火焰传感器 */
        if (now - last_sensor_poll >= SENSOR_POLL_INTERVAL_MS) {
            poll_flame_sensor();
            last_sensor_poll = now;
        }

        /* 更新蜂鸣器模式 */
        update_buzzer();

        /* 看门狗: 超时无心跳则全关（防止蜂鸣器常响不放） */
        watchdog_counter += 1;
        if (watchdog_counter > WATCHDOG_TIMEOUT) {
            RPMSG_DEBUG_W("Watchdog timeout! Emergency stop.");
            led_all_off();
            buzzer_mode = BUZZER_OFF;
            buzzer_off();
            watchdog_counter = 0;
        }

        if (shutdown_req || rproc_get_stop_flag()) {
            rproc_clear_stop_flag();
            break;
        }
    }

    rpmsg_destroy_ept(&lept);
    g_ept = NULL;
    return 0;
}

/* ====================== 初始化 ====================== */
int rpmsg_handler_init(void) {
    init_system();
    if (!platform_create_proc(&remoteproc_slave, &slave_priv, &kick_driver)) return -1;
    remoteproc_slave.rsc_table = &resources;
    if (platform_setup_src_table(&remoteproc_slave, remoteproc_slave.rsc_table)) return -1;
    if (platform_setup_share_mems(&remoteproc_slave)) return -1;
    rpdev_slave = platform_create_rpmsg_vdev(&remoteproc_slave, 0, VIRTIO_DEV_DEVICE, NULL, NULL);
    if (!rpdev_slave) return -1;

    last_flame_state = flame_sensor_read();
    last_sensor_poll = metal_get_timestamp();
    RPMSG_DEBUG_I("Init done. Flame: %s", last_flame_state ? "FIRE" : "clear");
    return 0;
}

int rpmsg_handler_run(void) {
    int ret = rpmsg_app_run(rpdev_slave, &remoteproc_slave);
    platform_release_rpmsg_vdev(rpdev_slave, &remoteproc_slave);
    platform_cleanup(&remoteproc_slave);
    return ret;
}

void rpmsg_handler_cleanup(void) {
    platform_cleanup(&remoteproc_slave);
}
