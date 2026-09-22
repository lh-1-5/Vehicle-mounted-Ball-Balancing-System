#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* ========================= 编码器驱动 (后轮 bL/bR + 步进) =========================
 * 3 路增量编码器, A 相 GPIO 上升沿中断计数, B 相判方向.
 * 移植自队友工程, 精简: 后轮 bL/bR (直流) + 步进电机编码器 (42步进).
 *
 * 引脚 (syscfg):
 *   GPIO_ENCODER_BL:   PIN_BL_A=PB20  (中断 RISE), PIN_BL_B=PA24 (方向) -> 左后 (跨端口)
 *   GPIO_ENCODER_BR:   PIN_BR_A=PB0   (中断 RISE), PIN_BR_B=PA13 (方向) -> 右后 (跨端口)
 *   GPIO_ENCODER_STEP: PIN_STEP_A=PB4 (中断 RISE), PIN_STEP_B=PB19(方向)-> 步进 (同端口)
 *
 * 中断: 三个 A 相都在 GPIOB, 共享 GROUP1 (GPIOB_INT_IRQn=1) 中断.
 *       ISR (GROUP1_IRQHandler) 在本模块 (Encoder.c) 实现.
 *       bL/bR 的 B 相在 GPIOA, 读方向用 per-pin _B_PORT 宏 (非组级 _PORT);
 *       step 两相都在 GPIOB, 读方向用组级 GPIO_ENCODER_STEP_PORT.
 *
 * 数据约定:
 *   encoder_bl/br      int16_t, 每 10ms 清零测速 (Control.c get_speed).
 *   encoder_step      int32_t, 位置累计不清零 (1024/圈). int32 防 16 位早溢出;
 *                     M0+ 读 32 位非原子, 闭环要严格一致时读时临时关中断.
 *
 * 用法:
 *   encoder_init();          // 使能 GPIOB NVIC (GPIO 模块级中断 syscfg 已配)
 *   // ISR 里 encoder_bl/br/step 自增/减
 *   // get_speed() 在 Control.c: 读 encoder_bl/br -> *_speed, 清零
 */

extern volatile int16_t encoder_bl;
extern volatile int16_t encoder_br;
extern volatile int16_t encoder_bl_speed;
extern volatile int16_t encoder_br_speed;
extern volatile int32_t encoder_step;   /* 步进编码器累计位置 (1024/圈, 带符号, 不清零) */

void encoder_init(void);

#endif /* __ENCODER_H */
