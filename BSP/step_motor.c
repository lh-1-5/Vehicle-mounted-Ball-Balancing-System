/**
 * @file    step_motor.c
 * @brief   1 路步进电机驱动 - PWM 脉冲 + 方向 GPIO, CC0_DN 中断计步
 *
 * ============================ 工作原理 ============================
 * 硬件:
 *   - PWM 输出引脚:  TIMA1 CCP0 = PA10 (通过 syscfg 配置)
 *   - 方向控制引脚:  DIR1 = PB13
 *   - 使能引脚:      EN1  = PB1
 *
 * 两种工作模式:
 *   [定步数] step_on_at_freq(N, hz) -> 装载步数, 以 hz 发脉冲,
 *              每个 CC0_DN 中断 step1_count--, 减到 0 自动 step_off() 停.
 *   [定速]   step_drive_hz(hz) -> 直接改 PWM 周期连续运转, 不计步, 不走 ramp.
 *
 * CC0_DN 中断互斥:
 *   定步数需 CC0_DN 计步 (step_on_at_freq 使能); 定速必须禁 CC0_DN, 否则
 *   step1_count 恒 0, ISR 判"步数走完"秒停 PWM (见设计记录坑2). 两者各自管中断.
 */
#include "ti_msp_dl_config.h"
#include "step_motor.h"

/* ---- 全局变量 (ISR 中修改, 主循环只读) ---- */

/** @brief 剩余脉冲数, ISR 中递减, 归零自动停机 (volatile 保证 ISR 可见性) */
volatile uint32_t step1_count = 0;

/** @brief 累计总步数 (正转+正, 反转+负), 可用于里程计/位置估算 */
int step1_sum = 0;

/**
 * @brief 根据给定频率配置 PWM 并设置方向引脚.
 *
 * 频率 -> 周期的换算:  period = TIMER_CLK / frequency
 * 占空比固定 50%:      compare_value = period / 2
 *
 * @param frequency_hz  正=正转, 负=反转, 0 则仅停止计数器 (不设方向)
 */
static void step_frequency_apply(int32_t frequency_hz)
{
    uint32_t frequency;   /* 绝对值频率 */
    uint32_t period;      /* 定时器重装载值 (决定 PWM 周期) */

    /* 先停计数器, 安全修改定时器参数 */
    DL_Timer_stopCounter(STEP_MOTOR_1_INST);

    if (frequency_hz == 0) {
        return;           /* 仅停止, 不设方向和周期 */
    }

    /* 根据符号设置方向引脚 (正转/反转) */
    if (frequency_hz < 0) {
        frequency = (uint32_t)(-frequency_hz);
        DL_GPIO_setPins(GPIO_STEP_MOTOR_DIR1_PORT, GPIO_STEP_MOTOR_DIR1_PIN);   /* DIR=高 -> 反转 */
    } else {
        frequency = (uint32_t)frequency_hz;
        DL_GPIO_clearPins(GPIO_STEP_MOTOR_DIR1_PORT, GPIO_STEP_MOTOR_DIR1_PIN); /* DIR=低 -> 正转 */
    }

    /* 计算定时器周期: 计数器时钟频率 / 目标脉冲频率 */
    period = STEP_MOTOR_1_INST_CLK_FREQ / frequency;

    /* 配置 PWM: 重装载值=period, 比较值=period/2 -> 50% 占空比 */
    DL_Timer_setLoadValue(STEP_MOTOR_1_INST, period);
    DL_Timer_setTimerCount(STEP_MOTOR_1_INST, period);
    DL_Timer_setCaptureCompareValue(
    STEP_MOTOR_1_INST, period / 2U, GPIO_STEP_MOTOR_1_C0_IDX);

    /* 重启动计数器 */
    DL_Timer_startCounter(STEP_MOTOR_1_INST);
}

/**
 * @brief 步进电机模块初始化.
 *
 * 设置 PWM 默认比较值, 使能 NVIC 中断 (CC0_DN 事件本身由各模式按需开关).
 * 注意: 此函数不启动 PWM 输出, 需调用 step_on_at_freq() 或 step_drive_hz() 才会转.
 */
void Step_Init(void)
{
    /* 设置初始 PWM 比较值 (默认 50% 占空比) */
    DL_Timer_setCaptureCompareValue(STEP_MOTOR_1_INST, 2500, GPIO_STEP_MOTOR_1_C0_IDX);

    /* 清除可能的挂起中断, 然后使能 NVIC 中断 */
    NVIC_ClearPendingIRQ(STEP_MOTOR_1_INST_INT_IRQN);
    NVIC_EnableIRQ(STEP_MOTOR_1_INST_INT_IRQN);
}

/**
 * @brief 立即停止步进电机.
 *
 * 停止 PWM 计数器, 清零剩余步数. 定步数/定速通用.
 */
void step_off(void)
{
    DL_Timer_stopCounter(STEP_MOTOR_1_INST);  /* 停止 PWM 输出 */
    step1_count = 0;        /* 清零剩余步数 */
}

/**
 * @brief [定步数] 以指定频率发 N 个脉冲后自动停止.
 *
 * 设方向 -> 使能 CC0_DN -> 装载步数到 step1_count -> 启动 PWM,
 * 随后每个 CC0_DN 中断 step1_count--, 减到 0 自动 step_off().
 *
 * 拍间守卫: 上一拍没转完(step1_count>0)先 step_off 强制停干净再发新指令,
 * 防止新指令打断旧脉冲序列导致 CC0_DN 计数错乱. 代价是上拍没转完的步数丢失,
 * 但保证每拍干净, 换向时不丢纠正.
 *
 * @param step_num  正=正转 N 步, 负=反转 |N| 步, 0=立即停止
 * @param hz        驱动频率 (取 STEP_SPEED_MIN..MAX, 符号随 step_num)
 */
void step_on_at_freq(int32_t step_num, int32_t hz)
{
    if (step_num == 0) { step_off(); return; }    /* 0 步: 立即停 */

    /* 拍间守卫: 上一拍没转完先 step_off 强制停干净, 再发新指令 */
    if (step1_count > 0) {
        step_off();
    }

    if (hz > STEP_SPEED_MAX_HZ) hz = STEP_SPEED_MAX_HZ;       /* clamp 频率 */
    else if (hz < STEP_SPEED_MIN_HZ) hz = STEP_SPEED_MIN_HZ;

    int32_t signed_hz = (step_num > 0) ? hz : -hz;   /* 方向随步数符号 */

    /* 使能 CC0_DN 计数中断 (定步数必需; step_drive_hz 会禁它, 互斥) */
    DL_Timer_clearInterruptStatus(STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_Timer_enableInterrupt(STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);

    step1_count = (uint32_t)((step_num > 0) ? step_num : -step_num);  /* 装载步数 */
    step1_sum += step_num;

    step_frequency_apply(signed_hz);    /* 设周期+方向并启动 */
}

/**
 * @brief [定速] 直接设定频率连续运转, 不计步, 不走 ramp.
 *
 * Clamp 到 [MIN_HZ, MAX_HZ] 保护电机 (太小堵转, 太高超驱动能力).
 * 必须禁用 CC0_DN 计步中断: 否则 step1_count(恒0)会被 ISR 判为"步数走完"而调
 * step_off() 把 PWM 秒停 -> 电机只发零星脉冲/抖动 (见设计记录坑2).
 * 位置环 PID 周期性调用本函数, 每拍输出不同 Hz 逼近目标位置.
 *
 * @param hz  目标频率 (Hz): 正=正转, 负=反转, 0=停
 */
void step_drive_hz(int32_t hz)
{
    /* Clamp 到允许范围 (保护电机) */
    if (hz > STEP_SPEED_MAX_HZ) {
        hz = STEP_SPEED_MAX_HZ;
    } else if (hz < -STEP_SPEED_MAX_HZ) {
        hz = -STEP_SPEED_MAX_HZ;
    } else if ((hz > 0) && (hz < STEP_SPEED_MIN_HZ)) {
        hz = STEP_SPEED_MIN_HZ;
    } else if ((hz < 0) && (hz > -STEP_SPEED_MIN_HZ)) {
        hz = -STEP_SPEED_MIN_HZ;
    }

    /* 禁 CC0_DN 防 step1_count==0 秒停 PWM */
    DL_Timer_disableInterrupt(STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_Timer_clearInterruptStatus(STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);

    step_frequency_apply(hz);    /* 直接改 PWM 周期 */
}

/**
 * @brief 定时器中断服务函数 - CC0_DN (比较匹配下降沿) 事件处理.
 *
 * 定步数模式下每个 PWM 脉冲下降沿触发一次:
 *   1. step1_count > 0 时递减计数.
 *   2. 减到 0 时自动调用 step_off() 停止 PWM.
 * 定速模式 CC0_DN 被禁用, 不进此分支.
 */
void STEP_MOTOR_1_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(STEP_MOTOR_1_INST)) {
        case DL_TIMER_IIDX_CC0_DN:
            /* 每个下降沿: 递减剩余步数 */
            if (step1_count > 0) step1_count--;
            /* 步数走完: 自动停机 */
            if (step1_count == 0) step_off();
            break;
        default:
            break;
    }
}
