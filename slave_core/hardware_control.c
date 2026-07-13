/*
 * hardware_control.c - 裸机硬件控制实现
 *
 * 飞腾派 PE2204 引脚分配:
 *   LED 红   - FGPIO4, PIN 13  (引脚 22)
 *   LED 黄   - FGPIO4, PIN 14  (引脚 18)
 *   LED 绿   - FGPIO4, PIN 15  (引脚 15)
 *   蜂鸣器   - FGPIO4, PIN 10  (引脚 11, J1 PIN_11, GPIO高低电平控制)
 *   火焰传感器 DO 口 - 引脚7 = FGPIO2, PIN 10 (低电平有效)
 *
 * 直接操作 GPIO 寄存器，无需 Linux 驱动
 */

#include "hardware_control.h"
#include "fire_protocol.h"
#include "fio_mux.h"
#include "fgpio_hw.h"
#include "fparameters.h"
#include "fdebug.h"
#include "fsleep.h"
#include "sdkconfig.h"
#include <stdio.h>

#define HW_DEBUG_TAG "HW_CTRL"
#define HW_DEBUG_I(format, ...) FT_DEBUG_PRINT_I(HW_DEBUG_TAG, format, ##__VA_ARGS__)
#define HW_DEBUG_E(format, ...) FT_DEBUG_PRINT_E(HW_DEBUG_TAG, format, ##__VA_ARGS__)

/* ====================== GPIO 引脚定义 ====================== */
/* LED: FGPIO4 模块, 引脚 13/14/15 */
#define LED_GPIO_BASE    FGPIO4_BASE_ADDR
#define LED_GPIO_CTRL    FGPIO_CTRL_4

#define LED_RED_PIN      FGPIO_PIN_13
#define LED_YEL_PIN      FGPIO_PIN_14
#define LED_GRN_PIN      FGPIO_PIN_15

/* 蜂鸣器: FGPIO4 模块, 引脚 10 (引脚11, J1 PIN_11) */
#define BUZZER_PIN       FGPIO_PIN_10

/* 火焰传感器: FGPIO2 模块, 引脚 10 (引脚7, DO口) */
#define FLAME_GPIO_BASE  FGPIO2_BASE_ADDR
#define FLAME_GPIO_CTRL  FGPIO_CTRL_2
#define FLAME_PIN        FGPIO_PIN_10

/* ====================== GPIO 辅助函数 ====================== */
static void gpio_set_output(u32 pin)
{
    u32 val = FGpioReadReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DDR_OFFSET);
    val |= BIT(pin);
    FGpioWriteReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DDR_OFFSET, val);
}

static void gpio_set_input(u32 base, u32 pin)
{
    u32 val = FGpioReadReg32(base, FGPIO_SWPORTA_DDR_OFFSET);
    val &= ~BIT(pin);
    FGpioWriteReg32(base, FGPIO_SWPORTA_DDR_OFFSET, val);
}

static void gpio_set_high(u32 pin)
{
    u32 val = FGpioReadReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DR_OFFSET);
    val |= BIT(pin);
    FGpioWriteReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DR_OFFSET, val);
}

static void gpio_set_low(u32 pin)
{
    u32 val = FGpioReadReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DR_OFFSET);
    val &= ~BIT(pin);
    FGpioWriteReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DR_OFFSET, val);
}

static int gpio_read_input(u32 base, u32 pin)
{
    u32 val = FGpioReadReg32(base, FGPIO_EXT_PORTA_OFFSET);
    return (val & BIT(pin)) ? 1 : 0;
}

/* ====================== LED 控制 ====================== */
void led_red_on(void)    { gpio_set_high(LED_RED_PIN); }
void led_red_off(void)   { gpio_set_low(LED_RED_PIN); }
void led_yellow_on(void) { gpio_set_high(LED_YEL_PIN); }
void led_yellow_off(void){ gpio_set_low(LED_YEL_PIN); }
void led_green_on(void)  { gpio_set_high(LED_GRN_PIN); }
void led_green_off(void) { gpio_set_low(LED_GRN_PIN); }

void led_all_off(void)
{
    gpio_set_low(LED_RED_PIN);
    gpio_set_low(LED_YEL_PIN);
    gpio_set_low(LED_GRN_PIN);
}

/* ====================== 蜂鸣器控制 (GPIO) ====================== */
/* 低电平触发: I/O=低电平 → 蜂鸣器响, I/O=高电平 → 蜂鸣器不响 */
void buzzer_on(void)
{
    gpio_set_low(BUZZER_PIN);
}

void buzzer_off(void)
{
    gpio_set_high(BUZZER_PIN);
}

/* ====================== 火焰传感器读取 ====================== */
int flame_sensor_read(void)
{
    int raw = gpio_read_input(FLAME_GPIO_BASE, FLAME_PIN);
    return (raw == 0) ? 1 : 0;
}

/* ====================== 初始化 ====================== */
int hardware_init(void)
{
    HW_DEBUG_I("Initializing hardware...");

    FIOMuxInit();

    /* 配置 LED GPIO 引脚 (输出) */
    FIOPadSetGpioMux(LED_GPIO_CTRL, LED_RED_PIN);
    FIOPadSetGpioMux(LED_GPIO_CTRL, LED_YEL_PIN);
    FIOPadSetGpioMux(LED_GPIO_CTRL, LED_GRN_PIN);

    gpio_set_output(LED_RED_PIN);
    gpio_set_output(LED_YEL_PIN);
    gpio_set_output(LED_GRN_PIN);

    led_all_off();
    HW_DEBUG_I("LED GPIO initialized (FGPIO4 pin 13/14/15)");

    /* 配置蜂鸣器 GPIO 引脚 (输出) */
    FIOPadSetGpioMux(LED_GPIO_CTRL, BUZZER_PIN);
    gpio_set_output(BUZZER_PIN);
    buzzer_off();
    HW_DEBUG_I("Buzzer GPIO initialized (FGPIO4 pin 10, PIN_11, disabled)");

    /* 配置火焰传感器 GPIO 引脚 (输入) */
    FIOPadSetGpioMux(FLAME_GPIO_CTRL, FLAME_PIN);
    gpio_set_input(FLAME_GPIO_BASE, FLAME_PIN);
    HW_DEBUG_I("Flame sensor initialized (FGPIO2 pin 10)");

    /* 开机指示: 绿灯闪一下 */
    led_green_on();
    fsleep_millisec(200);
    led_green_off();

    HW_DEBUG_I("Hardware init done.");
    return 0;
}

void hardware_deinit(void)
{
    led_all_off();
    buzzer_off();
    FIOMuxDeInit();
    HW_DEBUG_I("Hardware deinit done.");
}