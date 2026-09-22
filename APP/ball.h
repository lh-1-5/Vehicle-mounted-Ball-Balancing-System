#ifndef __BALL_H
#define __BALL_H

#include <stdint.h>
#include <stdbool.h>

/* ========================= 滚球平衡控制 (ball-and-beam) =========================
 * 步进电机控管子倾角, 球靠重力平衡. 独占 TIMER_BALL (TIMG6, 10ms), 与小车循迹
 * (Control.c / TIMER_0) 解耦, 不受 start_run 约束 (车不动也能控球).
 *
 * 控制模式 (编译期定, 改 BALL_LOOP_SINGLE 重烧, 无运行时切换):
 *   1=单环位置式 (默认, 当前主用): PID 出目标步位置 -> step_on_at_freq 直驱,
 *       软件累计 step_virt_pos 做角度基准, error=0 主动回正. 大扰动收敛优于双环.
 *   0=双环串级 (备选): PID 出目标倾角deg -> mechanism 换算 -> step_target_pos,
 *       内环 PID_StepPos 编码器闭环主动保持管子倾角.
 *
 * 详见 docs/步进位置闭环设计记录.md + docs/滚球模块拆分方案.md
 */

/* 控制模式: 1=单环(默认主用), 0=双环. 改后重烧. */
#ifndef BALL_LOOP_SINGLE
#define BALL_LOOP_SINGLE   1
#endif

/* 单环 D 项对比开关:
 *   1 = 摄像头新帧 + 真实 dt + 滤波球速, D 只对球位测量微分 (新方案)
 *   0 = 原 pidout() 的 error-error_prev D 项, 固定 30ms 调用 (旧方案)
 * 仅在 BALL_LOOP_SINGLE=1 时生效. */
#ifndef BALL_SINGLE_FRAME_PD
#define BALL_SINGLE_FRAME_PD   1
#endif

void ball_init(void);                   /* PID init + 启动 TIMER_BALL */
void ball_isr_10ms(void);               /* ISR入口: 新单环每拍查新帧; 旧单环/双环30ms外环 */

void ball_loop_run(void);              /* 使能外环 (菜单 B_Run) */
void ball_loop_stop(void);             /* 关外环 + 冻结 (菜单 B_Stp) */

void set_ball_position(int16_t pos);   /* 设目标 + 复位到达锁存 (task 序列) */
bool wait_ball_pos(int16_t pos);       /* 读到达锁存 (主循环非阻塞) */
void set_step_target(int input_pos);
void ball_measurement_update(int16_t position); /* 摄像头每个有效0x21帧提交一次 */

/* 菜单/调试绑定. volatile = 跨 ISR/主循环共享. */
extern volatile int16_t ball_pos;          /* 球位置 (摄像头 0x21, uart ISR 写), ±150 标尺 */
extern volatile int16_t ball_target_pos;   /* 目标球位置, 菜单 PARAM */
extern volatile int32_t step_target_pos;   /* 内环目标位置(脉冲), 菜单 PARAM (双环+step页) */
extern float           g_out;              /* 外环 PID 输出, 调试 READONLY */

/* 单环 PID 增益 (菜单 PARAM, AT24C02 持久化). 编辑后实时生效 + 掉电保存.
 * g_ball_kd = 帧-PD 模式速度增益(默认模式实际 D 项). */
extern float g_ball_kp;
extern float g_ball_ki;
extern float g_ball_kd;

/* 新方案调试量; 关闭 BALL_SINGLE_FRAME_PD 时保持为0. */
extern volatile uint32_t g_ball_frame_seq;
extern volatile uint32_t g_ball_dt_ms;
extern volatile float    g_ball_velocity_raw;
extern volatile float    g_ball_velocity_filtered;
extern volatile float    g_ball_p_out;
extern volatile float    g_ball_i_out;
extern volatile float    g_ball_d_out;

#endif /* __BALL_H */
