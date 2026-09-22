#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>

/**
 * @file    step_motor.h
 * @brief   1 路步进电机驱动 - PWM 脉冲 + 方向 GPIO
 *
 * ============================ 概述 ============================
 * 通过 TIMA1 定时器产生 PWM 脉冲驱动步进电机驱动器 (如 A4988/DRV8825),
 * 只保留两种工作模式:
 *
 *   [定步数] step_on_at_freq(N, hz)  适合精确位移控制
 *     - 正数正转 N 步, 负数反转 |N| 步, 0=立即停
 *     - 以指定频率发 N 个脉冲后自动停 (CC0_DN 中断计数到 0)
 *     - 上一拍没转完会先强制停干净再发新指令 (拍间守卫)
 *
 *   [定速]   step_drive_hz(hz)       适合连续运转 / 位置环输出
 *     - 正=正转, 负=反转, 0=停; clamp 到 [MIN, MAX] 保护电机
 *     - 直接改 PWM 周期, 无加减速 ramp, 调用即生效
 *     - 位置环 PID 周期性调用它逼近目标位置 (每拍 Hz 不同)
 *
 *   公共: Step_Init() 初始化 (不启动 PWM); step_off() 立即停 + 清状态.
 *
 *   CC0_DN 中断互斥: 定步数需 CC0_DN 计步 (本函数使能); 定速必须禁 CC0_DN
 *   否则 step1_count 恒 0 被 ISR 判"步数走完"秒停 PWM. 两者各自管中断不冲突.
 *
 * ============================ 引脚 ============================
 *   PWM 输出:  PA10 = TIMA1 CCP0  (syscfg: STEP_MOTOR_1)
 *   方向控制:  PB13 = DIR1         (syscfg: GPIO_STEP_MOTOR_DIR1)
 *   使能控制:  PB1  = EN1          (syscfg: GPIO_STEP_MOTOR_EN1)
 *
 * ============================ 使用示例 ============================
 *   // 定步数模式
 *   Step_Init();
 *   step_on_at_freq(200, 2000);   // 正转 200 步 @2000Hz, 到步数自动停
 *
 *   // 定速模式
 *   Step_Init();
 *   step_drive_hz(1000);          // 1000Hz 正转, 持续直到改值或 step_off()
 *   step_drive_hz(0);             // 停
 */

/* ---- 全局状态变量 ---- */

/** @brief 剩余脉冲数 (定步数模式: ISR 递减, 主循环只读) */
extern volatile uint32_t step1_count;

/** @brief 累计总步数, 正转+正/反转+负, 可用于里程计 */
extern int step1_sum;

/* ---- 电机参数 (加电后实测调整) ---- */

/** @brief 最低运行频率 (Hz), 低于此值电机可能无法正常转动 */
#define STEP_SPEED_MIN_HZ       200

/** @brief 最高运行频率 (Hz), 受限于电机与驱动器能力 */
#define STEP_SPEED_MAX_HZ      12000

/** @brief 编码器线数 (脉冲/圈) */
#define STEP_ENCODER_PPR        1024

/** @brief 驱动器细分 (1=整步200步/圈; 42步进+64细分=12800脉冲/圈). 可调 */
#define STEP_MICROSTEPS         64

/** @brief 电机每圈 PWM 脉冲数 = 整步数(200) × 细分 */
#define STEP_PULSES_PER_REV     (200 * STEP_MICROSTEPS)   /* 64细分 -> 12800 */

/* ---- API ---- */

void Step_Init(void);                                   /**< 模块初始化 (使能 NVIC, 不启动 PWM) */
void step_off(void);                                    /**< 立即停止电机, 清零状态 */

void step_on_at_freq(int32_t step_num, int32_t hz);     /**< [定步数] 发 N 步@hz 后自动停 (正/负/0=停) */
void step_drive_hz(int32_t hz);                         /**< [定速] 直接设频持续运转 (正/负/0=停) */

#endif /* __STEP_MOTOR_H */
