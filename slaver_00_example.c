/*
 * slaver_00_example.c - 消防通道监测从核固件
 *
 * 基于用户验证可用的旧代码模式(FGpio高层API)重写
 * 改动: LED从1个(FGPIO1 PIN8)改为3个(FGPIO4 PIN13/14/15)
 *       蜂鸣器从FGPIO3 PIN1改为FGPIO4 PIN10
 *       RPMsg协议改为单字符(G/Y/R/F/S/H)
 *
 * 引脚映射 (飞腾派J1排针物理引脚号):
 *   LED 绿   - 引脚15 → FGPIO4 PIN15 (高电平亮)
 *   LED 黄   - 引脚18 → FGPIO4 PIN14 (高电平亮)
 *   LED 红   - 引脚22 → FGPIO4 PIN13 (高电平亮)
 *   蜂鸣器   - 引脚11 → FGPIO4 PIN10 (有源, 低电平响)
 *   火焰传感器 - 引脚7  → FGPIO2 PIN10 (低电平有效)
 *
 * 指令协议 (单字符ASCII):
 *   G → 绿灯亮, 其余关闭, 蜂鸣器关
 *   Y → 黄灯亮, 其余关闭, 蜂鸣器关
 *   R → 红灯亮 + 蜂鸣器间歇鸣响(500ms)
 *   F → 红灯亮 + 蜂鸣器持续鸣响
 *   S → 全部关闭
 *   H → 回复h (心跳应答)
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <openamp/open_amp.h>
#include <metal/alloc.h>
#include <metal/sleep.h>
#include "platform_info.h"
#include "rpmsg_service.h"
#include "rsc_table.h"
#include "fcache.h"
#include "fdebug.h"
#include "fpsci.h"
#include "helper.h"
#include "openamp_configs.h"
#include "libmetal_configs.h"
#include "slaver_00_example.h"

/* GPIO驱动头文件 - 和旧代码完全一致 */
#include "sdkconfig.h"
#include "fio.h"
#include "ftypes.h"
#include "fkernel.h"
#include "fsleep.h"
#include "fassert.h"
#include "fiopad.h"
#include "fgpio_hw.h"
#include "fgpio.h"
#include "fio_mux.h"
#include "fparameters.h"
#include "fgeneric_timer.h"

#define SLAVE_DEBUG_TAG "    SLAVE_00"
#define SLAVE_DEBUG_I(format, ...) FT_DEBUG_PRINT_I(SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define SLAVE_DEBUG_W(format, ...) FT_DEBUG_PRINT_W(SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define SLAVE_DEBUG_E(format, ...) FT_DEBUG_PRINT_E(SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)

/* === 通信协议 === */
#define CMD_STATE_IDLE     'G'
#define CMD_STATE_WARNING  'Y'
#define CMD_STATE_OCCUPIED 'R'
#define CMD_STATE_FIRE     'F'
#define CMD_EMERGENCY_STOP 'S'
#define CMD_HEARTBEAT      'H'
#define CMD_HEARTBEAT_ACK  'h'
#define CMD_FIRE_ALERT     'F'
#define CMD_FIRE_CLEAR     'f'

#define RPMSG_SERVICE_NAME "rpmsg-openamp-demo-channel"

/* === 蜂鸣器模式 === */
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_INTERMITTENT,
    BUZZER_CONSTANT
} buzzer_mode_t;

/* === 全局变量 === */
static volatile int shutdown_req = 0;
static struct rpmsg_endpoint *g_ept = NULL;
struct remoteproc remoteproc_slave_00;
static struct rpmsg_device *rpdev_slave_00 = NULL;

static buzzer_mode_t buzzer_mode = BUZZER_OFF;
static u64 last_buzzer_toggle = 0;
static int buzzer_is_on = 0;
static int last_flame_state = 0;
static u64 last_flame_poll = 0;
static u64 watchdog_counter_us = 0;

/* ===================== LED驱动 (完全照搬旧代码led20set.c模式) ===================== */
/* 3个LED: 绿(FGPIO4 PIN15), 黄(FGPIO4 PIN14), 红(FGPIO4 PIN13) */

static int led_init_flag = 0;
static FGpio led_green_gpio;
static FGpio led_yellow_gpio;
static FGpio led_red_gpio;

static void led_init(void)
{
    if (led_init_flag) return;

    /* 绿灯 FGPIO4 PIN15 */
    u32 green_id = FGPIO_ID(FGPIO_CTRL_4, FGPIO_PIN_15);
    const FGpioConfig *green_cfg = FGpioLookupConfig(green_id);
    if (green_cfg) {
        memset(&led_green_gpio, 0, sizeof(led_green_gpio));
        if (FGPIO_SUCCESS == FGpioCfgInitialize(&led_green_gpio, green_cfg)) {
            FIOPadSetGpioMux(FGPIO_CTRL_4, FGPIO_PIN_15);
            FGpioSetDirection(&led_green_gpio, FGPIO_DIR_OUTPUT);
            FGpioSetOutputValue(&led_green_gpio, 0);  /* 初始灭 */
            printf("LED green init OK (GPIO4 PIN15)\r\n");
        } else {
            printf("LED green FGpioCfgInitialize error\r\n");
        }
    } else {
        printf("LED green FGpioLookupConfig error\r\n");
    }

    /* 黄灯 FGPIO4 PIN14 */
    u32 yellow_id = FGPIO_ID(FGPIO_CTRL_4, FGPIO_PIN_14);
    const FGpioConfig *yellow_cfg = FGpioLookupConfig(yellow_id);
    if (yellow_cfg) {
        memset(&led_yellow_gpio, 0, sizeof(led_yellow_gpio));
        if (FGPIO_SUCCESS == FGpioCfgInitialize(&led_yellow_gpio, yellow_cfg)) {
            FIOPadSetGpioMux(FGPIO_CTRL_4, FGPIO_PIN_14);
            FGpioSetDirection(&led_yellow_gpio, FGPIO_DIR_OUTPUT);
            FGpioSetOutputValue(&led_yellow_gpio, 0);
            printf("LED yellow init OK (GPIO4 PIN14)\r\n");
        } else {
            printf("LED yellow FGpioCfgInitialize error\r\n");
        }
    } else {
        printf("LED yellow FGpioLookupConfig error\r\n");
    }

    /* 红灯 FGPIO4 PIN13 */
    u32 red_id = FGPIO_ID(FGPIO_CTRL_4, FGPIO_PIN_13);
    const FGpioConfig *red_cfg = FGpioLookupConfig(red_id);
    if (red_cfg) {
        memset(&led_red_gpio, 0, sizeof(led_red_gpio));
        if (FGPIO_SUCCESS == FGpioCfgInitialize(&led_red_gpio, red_cfg)) {
            FIOPadSetGpioMux(FGPIO_CTRL_4, FGPIO_PIN_13);
            FGpioSetDirection(&led_red_gpio, FGPIO_DIR_OUTPUT);
            FGpioSetOutputValue(&led_red_gpio, 0);
            printf("LED red init OK (GPIO4 PIN13)\r\n");
        } else {
            printf("LED red FGpioCfgInitialize error\r\n");
        }
    } else {
        printf("LED red FGpioLookupConfig error\r\n");
    }

    led_init_flag = 1;
}

static void led_all_off(void)
{
    led_init();
    FGpioSetOutputValue(&led_red_gpio, 0);
    FGpioSetOutputValue(&led_yellow_gpio, 0);
    FGpioSetOutputValue(&led_green_gpio, 0);
}

static void led_set_green(void)
{
    led_init();
    FGpioSetOutputValue(&led_red_gpio, 0);
    FGpioSetOutputValue(&led_yellow_gpio, 0);
    FGpioSetOutputValue(&led_green_gpio, 1);  /* 高电平亮 */
}

static void led_set_yellow(void)
{
    led_init();
    FGpioSetOutputValue(&led_red_gpio, 0);
    FGpioSetOutputValue(&led_green_gpio, 0);
    FGpioSetOutputValue(&led_yellow_gpio, 1);
}

static void led_set_red(void)
{
    led_init();
    FGpioSetOutputValue(&led_yellow_gpio, 0);
    FGpioSetOutputValue(&led_green_gpio, 0);
    FGpioSetOutputValue(&led_red_gpio, 1);
}

/* ===================== 蜂鸣器驱动 (完全照搬旧代码buzzer.c模式) ===================== */
/* 蜂鸣器: FGPIO4 PIN10 (有源, 低电平响) */

static int buzzer_init_flag = 0;
static FGpio buzzer_gpio;

static void buzzer_init(void)
{
    if (buzzer_init_flag) return;

    u32 buzzer_id = FGPIO_ID(FGPIO_CTRL_4, FGPIO_PIN_10);
    const FGpioConfig *config = FGpioLookupConfig(buzzer_id);
    if (NULL == config) {
        printf("Buzzer_Init(): FGpioLookupConfig error\r\n");
        return;
    }

    memset(&buzzer_gpio, 0, sizeof(buzzer_gpio));
    if (FGPIO_SUCCESS != FGpioCfgInitialize(&buzzer_gpio, config)) {
        printf("Buzzer_Init(): FGpioCfgInitialize error\r\n");
        return;
    }

    FIOPadSetGpioMux(FGPIO_CTRL_4, FGPIO_PIN_10);
    FGpioSetDirection(&buzzer_gpio, FGPIO_DIR_OUTPUT);
    FGpioSetOutputValue(&buzzer_gpio, 1);  /* 高电平=不响(有源低电平触发) */

    buzzer_init_flag = 1;
    printf("Buzzer init OK (GPIO4 PIN10, active low)\r\n");
}

static void buzzer_set_on(void)
{
    buzzer_init();
    FGpioSetOutputValue(&buzzer_gpio, 0);  /* 低电平=响 */
    buzzer_is_on = 1;
}

static void buzzer_set_off(void)
{
    buzzer_init();
    FGpioSetOutputValue(&buzzer_gpio, 1);  /* 高电平=不响 */
    buzzer_is_on = 0;
}

static void update_buzzer(u64 now_us)
{
    switch (buzzer_mode) {
        case BUZZER_OFF:
            if (buzzer_is_on) buzzer_set_off();
            break;
        case BUZZER_CONSTANT:
            if (!buzzer_is_on) buzzer_set_on();
            break;
        case BUZZER_INTERMITTENT:
            if (now_us - last_buzzer_toggle >= 500000) {  /* 500ms切换 */
                last_buzzer_toggle = now_us;
                if (buzzer_is_on) buzzer_set_off();
                else buzzer_set_on();
            }
            break;
    }
}

/* ===================== 火焰传感器 (FGPIO2 PIN10, 低电平有效) ===================== */
static int flame_init_flag = 0;
static FGpio flame_gpio;

static void flame_init(void)
{
    if (flame_init_flag) return;

    u32 flame_id = FGPIO_ID(FGPIO_CTRL_2, FGPIO_PIN_10);
    const FGpioConfig *config = FGpioLookupConfig(flame_id);
    if (NULL == config) {
        printf("Flame_Init(): FGpioLookupConfig error\r\n");
        return;
    }

    memset(&flame_gpio, 0, sizeof(flame_gpio));
    if (FGPIO_SUCCESS != FGpioCfgInitialize(&flame_gpio, config)) {
        printf("Flame_Init(): FGpioCfgInitialize error\r\n");
        return;
    }

    FIOPadSetGpioMux(FGPIO_CTRL_2, FGPIO_PIN_10);
    FGpioSetDirection(&flame_gpio, FGPIO_DIR_INPUT);

    flame_init_flag = 1;
    printf("Flame sensor init OK (GPIO2 PIN10, active low)\r\n");
}

static int flame_read(void)
{
    flame_init();
    FGpioVal val = FGpioGetInputValue(&flame_gpio);
    return (val == FGPIO_PIN_LOW) ? 1 : 0;  /* 低电平=检测到火焰 */
}

static void poll_flame_sensor(u64 now_us)
{
    if (now_us - last_flame_poll < 200000) return;  /* 200ms轮询 */
    last_flame_poll = now_us;

    int cur = flame_read();
    if (cur != last_flame_state) {
        if (cur) {
            printf("Flame detected!\r\n");
            if (g_ept && is_rpmsg_ept_ready(g_ept)) {
                char cmd = CMD_FIRE_ALERT;
                rpmsg_send(g_ept, &cmd, 1);
            }
        } else {
            printf("Flame cleared.\r\n");
            if (g_ept && is_rpmsg_ept_ready(g_ept)) {
                char cmd = CMD_FIRE_CLEAR;
                rpmsg_send(g_ept, &cmd, 1);
            }
        }
        last_flame_state = cur;
    }
}

/* ===================== 命令处理 ===================== */
static void process_command(char cmd)
{
    watchdog_counter_us = 0;

    switch (cmd) {
        case CMD_STATE_IDLE:       /* G */
            led_set_green();
            buzzer_mode = BUZZER_OFF;
            buzzer_set_off();
            break;
        case CMD_STATE_WARNING:    /* Y */
            led_set_yellow();
            buzzer_mode = BUZZER_OFF;
            buzzer_set_off();
            break;
        case CMD_STATE_OCCUPIED:   /* R */
            led_set_red();
            buzzer_mode = BUZZER_INTERMITTENT;
            last_buzzer_toggle = GenericTimerRead(0);
            buzzer_set_off();
            break;
        case CMD_STATE_FIRE:       /* F */
            led_set_red();
            buzzer_mode = BUZZER_CONSTANT;
            break;
        case CMD_EMERGENCY_STOP:   /* S */
            led_all_off();
            buzzer_mode = BUZZER_OFF;
            buzzer_set_off();
            break;
        case CMD_HEARTBEAT:        /* H */
            if (g_ept && is_rpmsg_ept_ready(g_ept)) {
                char ack = CMD_HEARTBEAT_ACK;
                rpmsg_send(g_ept, &ack, 1);
            }
            break;
        default:
            break;
    }
}

/* ===================== 资源表 ===================== */
static struct remote_resource_table __resource resources __attribute__((used)) = {
    1, NUM_TABLE_ENTRIES, {0, 0},
    { offsetof(struct remote_resource_table, rpmsg_vdev), },
    { RSC_VDEV, VIRTIO_ID_RPMSG_, VDEV_NOTIFYID, RPMSG_IPU_C0_FEATURES,
      0, 0, 0, NUM_VRINGS, {0, 0}, },
    {SLAVE00_TX_VRING_ADDR, VRING_ALIGN, SLAVE00_VRING_NUM, 1, 0},
    {SLAVE00_RX_VRING_ADDR, VRING_ALIGN, SLAVE00_VRING_NUM, 2, 0},
};

static metal_phys_addr_t poll_phys_addr = SLAVE00_KICK_IO_ADDR;
struct metal_device kick_driver_00 = {
    .name = SLAVE_00_KICK_DEV_NAME, .bus = NULL, .num_regions = 1,
    .regions = {{ .virt = (void *)SLAVE00_KICK_IO_ADDR, .physmap = &poll_phys_addr,
                  .size = 0x1000, .page_shift = -1UL, .page_mask = -1UL,
                  .mem_flags = SLAVE00_SOURCE_TABLE_ATTRIBUTE, .ops = {NULL}, }},
    .irq_num = 1, .irq_info = (void *)SLAVE_00_SGI,
};

struct remoteproc_priv slave_00_priv = {
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

/* ===================== RPMsg回调 ===================== */
static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
                             uint32_t src, void *priv)
{
    (void)priv;
    char *msg = (char *)data;
    ept->dest_addr = src;
    if (len >= 1) {
        process_command(msg[0]);
    }
    return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    g_ept = NULL;
    shutdown_req = 1;
}

/* ===================== 主应用 ===================== */
static int FRpmsgApp(struct rpmsg_device *rdev, void *priv)
{
    int ret = 0;
    static struct rpmsg_endpoint lept = {0};
    shutdown_req = 0;

    /* 初始化所有硬件 */
    FIOMuxInit();      /* IO复用控制器初始化 - 必须最先调用 */
    led_init();
    buzzer_init();
    flame_init();

    /* 初始状态: 绿灯亮, 蜂鸣器关 */
    led_set_green();
    buzzer_set_off();
    last_flame_state = flame_read();

    /* 创建RPMsg端点 */
    ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME, 0, RPMSG_ADDR_ANY,
                           rpmsg_endpoint_cb, rpmsg_service_unbind);
    if (ret) {
        SLAVE_DEBUG_E("Failed to create RPMsg endpoint!");
        return -1;
    }
    g_ept = &lept;

    while (!is_rpmsg_ept_ready(&lept)) {
        platform_poll(priv);
        fsleep_millisec(10);
    }
    SLAVE_DEBUG_I("RPMsg endpoint ready!");

    last_flame_poll = GenericTimerRead(0);
    last_buzzer_toggle = GenericTimerRead(0);
    watchdog_counter_us = 0;

    /* 主循环 */
    while (1) {
        platform_poll(priv);
        u64 now_us = GenericTimerRead(0);

        poll_flame_sensor(now_us);
        update_buzzer(now_us);

        /* 看门狗: 15秒无心跳全关 */
        watchdog_counter_us += 1000;
        if (watchdog_counter_us > 15000000) {
            SLAVE_DEBUG_W("Watchdog timeout! All OFF.");
            led_all_off();
            buzzer_mode = BUZZER_OFF;
            buzzer_set_off();
            watchdog_counter_us = 0;
        }

        if (shutdown_req || rproc_get_stop_flag()) {
            rproc_clear_stop_flag();
            break;
        }
        fsleep_millisec(1);
    }

    buzzer_set_off();
    led_all_off();
    rpmsg_destroy_ept(&lept);
    g_ept = NULL;
    return ret;
}

/* ===================== 从核初始化 ===================== */
int slave_init(void)
{
    init_system();
    if (!platform_create_proc(&remoteproc_slave_00, &slave_00_priv, &kick_driver_00)) return -1;
    remoteproc_slave_00.rsc_table = &resources;
    if (platform_setup_src_table(&remoteproc_slave_00, remoteproc_slave_00.rsc_table)) return -1;
    if (platform_setup_share_mems(&remoteproc_slave_00)) return -1;
    rpdev_slave_00 = platform_create_rpmsg_vdev(&remoteproc_slave_00, 0, VIRTIO_DEV_DEVICE, NULL, NULL);
    if (!rpdev_slave_00) return -1;
    return 0;
}

/* ===================== 从核入口 ===================== */
int slave00_rpmsg_echo_process(void)
{
    int ret = 0;
    SLAVE_DEBUG_I("Starting application...");
    if (!slave_init()) {
        ret = FRpmsgApp(rpdev_slave_00, &remoteproc_slave_00);
        if (ret) { platform_cleanup(&remoteproc_slave_00); return -1; }
        platform_release_rpmsg_vdev(rpdev_slave_00, &remoteproc_slave_00);
        platform_cleanup(&remoteproc_slave_00);
    } else {
        platform_cleanup(&remoteproc_slave_00);
    }
    FPsciCpuOff();
    return 0;
}
