/*
 * hardware_control.c - 裸机硬件控制实现
 *
 * 飞腾派 PE2204 引脚分配:
 *   LED 红   - FGPIO4, PIN 13  (与 SDK 示例一致)
 *   LED 黄   - FGPIO4, PIN 14
 *   LED 绿   - FGPIO4, PIN 15
 *   蜂鸣器   - PWM2, Channel 0 (PDF 手册 PIN32 BUZZER)
 *
 * 直接操作 GPIO 寄存器，无需 Linux 驱动
 */

#include "hardware_control.h"
#include "fire_protocol.h"
#include "fio_mux.h"
#include "fgpio_hw.h"
#include "fpwm.h"
#include "fpwm_hw.h"
#include "fparameters.h"
#include "fdebug.h"
#include "fsleep.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>

#define HW_DEBUG_TAG "HW_CTRL"
#define HW_DEBUG_I(format, ...) FT_DEBUG_PRINT_I(HW_DEBUG_TAG, format, ##__VA_ARGS__)
#define HW_DEBUG_E(format, ...) FT_DEBUG_PRINT_E(HW_DEBUG_TAG, format, ##__VA_ARGS__)

/* ====================== GPIO 引脚定义 ====================== */
/* 飞腾派: FGPIO4 模块, 引脚 13/14/15 用于 LED */
#define LED_GPIO_BASE    FGPIO4_BASE_ADDR
#define LED_GPIO_CTRL    FGPIO_CTRL_4

#define LED_RED_PIN      FGPIO_PIN_13
#define LED_YEL_PIN      FGPIO_PIN_14
#define LED_GRN_PIN      FGPIO_PIN_15

/* ====================== PWM 定义 ====================== */
#define BUZZER_PWM_ID    FPWM2_ID        /* PWM2 */
#define BUZZER_PWM_CH    0               /* Channel 0 */

static FPwmCtrl pwm_ctrl;
static FPwmConfig pwm_config;
static u32 pwm_initialized = 0;

/* ====================== GPIO 辅助函数 ====================== */
static void gpio_set_output(u32 pin)
{
    u32 val = FGpioReadReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DDR_OFFSET);
    val |= BIT(pin);  /* 1 = output */
    FGpioWriteReg32(LED_GPIO_BASE, FGPIO_SWPORTA_DDR_OFFSET, val);
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

/* ====================== 蜂鸣器控制 (PWM) ====================== */
void buzzer_on(void)
{
    if (pwm_initialized) {
        FPwmEnable(&pwm_ctrl, BUZZER_PWM_CH);
    }
}

void buzzer_off(void)
{
    if (pwm_initialized) {
        FPwmDisable(&pwm_ctrl, BUZZER_PWM_CH);
    }
}

/* ====================== 初始化 ====================== */
int hardware_init(void)
{
    HW_DEBUG_I("Initializing hardware...");

    /* 1. 初始化 IO 复用 */
    FIOMuxInit();

    /* 2. 配置 LED GPIO 引脚 */
    FIOPadSetGpioMux(LED_GPIO_CTRL, LED_RED_PIN);
    FIOPadSetGpioMux(LED_GPIO_CTRL, LED_YEL_PIN);
    FIOPadSetGpioMux(LED_GPIO_CTRL, LED_GRN_PIN);

    gpio_set_output(LED_RED_PIN);
    gpio_set_output(LED_YEL_PIN);
    gpio_set_output(LED_GRN_PIN);

    led_all_off();
    HW_DEBUG_I("LED GPIO initialized (FGPIO4 pin 13/14/15)");

    /* 3. 初始化 PWM 蜂鸣器 */
    FIOPadSetPwmMux(BUZZER_PWM_ID, BUZZER_PWM_CH);

    memset(&pwm_ctrl, 0, sizeof(pwm_ctrl));
    memset(&pwm_config, 0, sizeof(pwm_config));

    pwm_config = *FPwmLookupConfig(BUZZER_PWM_ID);
    if (FPwmCfgInitialize(&pwm_ctrl, &pwm_config) != FPWM_SUCCESS) {
        HW_DEBUG_E("PWM init failed");
        return -1;
    }

    /* 配置 PWM: 频率约 2kHz, 50% 占空比 */
    FPwmVariableConfig pwm_cfg;
    memset(&pwm_cfg, 0, sizeof(pwm_cfg));
    pwm_cfg.tim_ctrl_mode = FPWM_MODULO;
    pwm_cfg.tim_ctrl_div = 50 - 1;      /* 分频 */
    pwm_cfg.pwm_period = 20000;         /* 周期 */
    pwm_cfg.pwm_pulse = 10000;          /* 50% 占空比 */
    pwm_cfg.pwm_mode = FPWM_OUTPUT_COMPARE;
    pwm_cfg.pwm_polarity = FPWM_POLARITY_NORMAL;
    pwm_cfg.pwm_duty_source_mode = FPWM_DUTY_CCR;

    if (FPwmVariableSet(&pwm_ctrl, BUZZER_PWM_CH, &pwm_cfg) != FPWM_SUCCESS) {
        HW_DEBUG_E("PWM config failed");
        return -1;
    }

    pwm_initialized = 1;
    HW_DEBUG_I("PWM buzzer initialized (PWM2 CH0, 2kHz 50%%)");

    /* 4. 开机指示: 绿灯闪一下 */
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

    if (pwm_initialized) {
        FPwmDeInitialize(&pwm_ctrl);
        pwm_initialized = 0;
    }

    FIOMuxDeInit();
    HW_DEBUG_I("Hardware deinit done.");
}
