/*
 * motor.c - 后轮电机 PWM 驱动
 * 详见 motor.h. 仅后轮 (前轮 motor_ahind_control 已精简删除).
 * 两路合并到单 TIMG8 双 CCP: CCP0=bR/PB15, CCP1=bL/PB16.
 */
#include "ti_msp_dl_config.h"
#include "Motor.h"

/* 启动 PWM 定时器 (syscfg 已配外设, 这里 startCounter; 两路共一个 TIMG8). */
void motor_init(void)
{
    DL_Timer_startCounter(PWM_MOTOR_INST);
}

/* 后轮左右电机方向与转速控制.
 * 两轮同驱动模块, 方向脚逻辑一致: 正转 AIN_2=1/AIN_1=0, 反转 AIN_1=1/AIN_2=0.
 * (bL 原用 AIN_1=1 正转, 实测反转, 已对调 AIN_1/AIN_2)
 * PWM 占空比 = |pwm| (0-1000, 对应 timerCount). */
void motor_behind_control(int16_t pwm_l, int16_t pwm_r)
{
    if (pwm_l >= 0) {
        DL_GPIO_setPins(GPIO_MOTOR_1_PORT, GPIO_MOTOR_1_PIN1_AIN_2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_1_PORT, GPIO_MOTOR_1_PIN1_AIN_1_PIN);
    } else {
        DL_GPIO_setPins(GPIO_MOTOR_1_PORT, GPIO_MOTOR_1_PIN1_AIN_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_1_PORT, GPIO_MOTOR_1_PIN1_AIN_2_PIN);
        pwm_l = -pwm_l;
    }

    if (pwm_r >= 0) {
        DL_GPIO_setPins(GPIO_MOTOR_2_PIN2_AIN_2_PORT, GPIO_MOTOR_2_PIN2_AIN_2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_2_PIN2_AIN_1_PORT, GPIO_MOTOR_2_PIN2_AIN_1_PIN);
    } else {
        DL_GPIO_setPins(GPIO_MOTOR_2_PIN2_AIN_1_PORT, GPIO_MOTOR_2_PIN2_AIN_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_2_PIN2_AIN_2_PORT, GPIO_MOTOR_2_PIN2_AIN_2_PIN);
        pwm_r = -pwm_r;
    }

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)pwm_l, GPIO_PWM_MOTOR_C1_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)pwm_r, GPIO_PWM_MOTOR_C0_IDX);
}
