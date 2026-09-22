#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* ========================= 控制核心 (小车循迹) =========================
 * 三环控制: 循迹转向环 (turn) + yaw 转向环 (yaw, 预留) + 速度环 (speed).
 * 控制周期 10ms (TIMER_0 = TIMA0, 100Hz), 在 TIMER_0_INST_IRQHandler 里跑.
 *
 * 滚球平衡控制(ball-and-beam)已拆出到 APP/ball.c (独立 TIMER_BALL, 10ms), 见 ball.h.
 *
 * 对外接口 (system.c 调用):
 *   control_init()        - 初始化 PID 参数 + 启动 100Hz 控制环 (TIMER_0)
 *   control_toggle_run()  - 切换 start_run 运行状态 (K3 长按调)
 *
 * 中断:
 *   TIMER_0_INST_IRQHandler - 控制环 10ms (turn + speed, yaw 预留)
 *   (编码器 GROUP1_IRQHandler 在 Encoder.c 实现)
 */

void control_init(void);
void control_toggle_run(void);

void get_speed(void);
void yaw_control(void);
void turn_control(void);
void speed_control(void);


void set_speed(int16_t target_speed);
void control_run(void);       /* 立即达到 basicSpeed (T4 用) */
void control_run_ramp(void);  /* 从 0 缓启到 basicSpeed (C_Run 用, 保护小球) */
void control_stop(void);

extern int16_t track_error;
extern int16_t track_angle;
extern int16_t track_turn;
extern int16_t gyro_z_speed;

extern float   yaw_error;
extern bool    start_run;
extern uint32_t isr_cnt;

extern int16_t basicSpeed;   /* 速度环目标 (脉冲数/10ms), 菜单 PARAM 可调 */

/* 循迹 PID 增益 (菜单 PARAM, AT24C02 持久化). 编辑后实时同步到 PID 结构体生效. */
extern float g_speed_kp;   /* 速度环 P (bL/bR 共用) */
extern float g_speed_ki;   /* 速度环 I (bL/bR 共用) */
extern float g_turn_kp;    /* 转向环 P */
extern float g_turn_kd;    /* 转向环 D */

/* ==================== 第四问小车状态机 (Control.c 定义, task.c 调用) ====================
 * 起跑线检测在 turn_control (ISR) 里做 -> pl_event; angle_sum 累计 yaw 跨 360°;
 * state_reset 复位所有状态. task_four_start 调 state_reset + 设 CAR_START_LINE_WAIT. */
typedef enum {
    CAR_IDLE = 0,
    CAR_START_LINE_WAIT,
    CAR_RUNNING,
    CAR_SLOW,
    CAR_BRAKING,
} car_state_t;

extern car_state_t car_state;
extern volatile bool pl_debounce;
extern volatile bool pl_event;       /* 起跑线事件 (ISR turn_control 置位, 状态机消费) */
extern int32_t      enc_at_stop_line;
extern uint32_t     run_start_tms;
extern uint32_t     run_end_tms;
extern uint32_t     total_run_ms;
extern bool         oled_shown;
extern int          full_rotations;  /* angle_sum 整圈数 */
extern uint8_t      black_cnt;       /* 黑线传感器计数 (turn_control 用) */

/* 复位第四问所有状态变量 (car_state=CAR_IDLE 等). task_four_start 调. */
void state_reset(void);
/* 累计 yaw 角度, 处理 ±180°/360° 跨越, 返回连续累计值(不限于±360). */
float angle_sum(float yaw_a);

#endif /* __CONTROL_H */
