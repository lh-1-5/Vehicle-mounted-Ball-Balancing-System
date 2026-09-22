/*
 * encoder.c - 编码器计数 (后轮 bL/bR 直流 + 步进)
 * 详见 encoder.h. ISR (GROUP1_IRQHandler) 在本文件末尾实现.
 *
 * encoder_init 只使能 NVIC: GPIO 模块级中断 (DL_GPIO_enableInterrupt) 已由 syscfg
 * 生成的 SYSCFG_DL_init 配好, 这里补内核级使能. 三个 A 相都在 GPIOB, 共享 GROUP1.
 * bL/bR 跨端口 (A=GPIOB, B=GPIOA), B 相读方向用 per-pin _B_PORT 宏;
 * step 同端口 (两相都在 GPIOB), B 相读方向用组级 _PORT 宏.
 */
#include "ti_msp_dl_config.h"
#include "Encoder.h"

volatile int16_t encoder_bl = 0;
volatile int16_t encoder_br = 0;
volatile int16_t encoder_bl_speed = 0;
volatile int16_t encoder_br_speed = 0;
volatile int32_t encoder_step = 0;       /* 步进位置累计, 不清零 */

void encoder_init(void)
{
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/* ---- 编码器 ISR (GROUP1, GPIOB: PB0=bR, PB4=step, PB20=bL) ----
 * A 相上升沿触发, B 相电平判方向. 三个 A 相都在 GPIOB, 共享 GROUP1 中断.
 * bL/bR 跨端口: A=GPIOB (中断), B=GPIOA (读方向用 per-pin _B_PORT).
 * step 同端口:  A=PB4 (中断), B=PB19 (读方向用组级 _PORT). */
void GROUP1_IRQHandler(void)
{
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIOB,
        GPIO_ENCODER_BL_PIN_BL_A_PIN | GPIO_ENCODER_BR_PIN_BR_A_PIN
        | GPIO_ENCODER_STEP_PIN_STEP_A_PIN);

    DL_GPIO_clearInterruptStatus(GPIOB,
        GPIO_ENCODER_BL_PIN_BL_A_PIN | GPIO_ENCODER_BR_PIN_BR_A_PIN
        | GPIO_ENCODER_STEP_PIN_STEP_A_PIN);

    /* bR: A 相 PB0 上升沿, B 相 PA13 判方向. 正转(B高)递增 */
    if (gpioB & GPIO_ENCODER_BR_PIN_BR_A_PIN)
    {
        if (DL_GPIO_readPins(GPIO_ENCODER_BR_PIN_BR_B_PORT, GPIO_ENCODER_BR_PIN_BR_B_PIN))
            encoder_br++;
        else
            encoder_br--;
    }

    /* bL: A 相 PB20 上升沿, B 相 PA24 判方向. 正转(B高)递增 */
    if (gpioB & GPIO_ENCODER_BL_PIN_BL_A_PIN)
    {
        if (DL_GPIO_readPins(GPIO_ENCODER_BL_PIN_BL_B_PORT, GPIO_ENCODER_BL_PIN_BL_B_PIN))
            encoder_bl++;
        else
            encoder_bl--;
    }

    /* step: A 相 PB4 上升沿, B 相 PB19 判方向. 正转(B高)递增. 位置累计不清零 */
    if (gpioB & GPIO_ENCODER_STEP_PIN_STEP_A_PIN)
    {
        if (DL_GPIO_readPins(GPIO_ENCODER_STEP_PORT, GPIO_ENCODER_STEP_PIN_STEP_B_PIN))
            encoder_step++;
        else
            encoder_step--;
    }
}
