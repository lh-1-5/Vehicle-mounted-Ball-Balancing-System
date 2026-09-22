#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* ========================= 电机驱动 (后轮 bL/bR, 统一驱动模块) =========================
 * 2 路直流电机走同一个驱动模块 (A通道=bR, B通道=bL), 共 1 路 PWM 定时器 (TIMG8 双 CCP)
 * + 每路 2 个 GPIO 方向脚 (IN1/IN2 模式).
 * 移植自队友工程, 精简: 只保留后轮 motor_behind_control (前轮 ahind 已删).
 *
 * 引脚 (syscfg):
 *   PWM_MOTOR = TIMG8 CCP0 = PB15 (bR), CCP1 = PB16 (bL)
 *   GPIO_MOTOR_1: PIN1_AIN_1=PA14, PIN1_AIN_2=PA15  (左后 bL 方向, 驱动B通道)
 *   GPIO_MOTOR_2: PIN2_AIN_1=PA12, PIN2_AIN_2=PB5   (右后 bR 方向, 驱动A通道)
 *
 * 用法:
 *   motor_init();                       // 启动 PWM 定时器
 *   motor_behind_control(pwm_l, pwm_r); // 正负 0-1000, 正正转负反转
 */

void motor_init(void);
void motor_behind_control(int16_t pwm_l, int16_t pwm_r);

#endif /* __MOTOR_H */
