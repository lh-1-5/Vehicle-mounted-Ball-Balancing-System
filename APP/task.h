#ifndef TASK_H
#define TASK_H

#include <stdint.h>
#include <stdbool.h>

void task_one_start(void);
void task_three_start(void);
void task_three_stop(void);

/* 第四问: 循迹一圈并定点停车. 菜单 T4_Run/T4_Stp 触发. */
void task_four_start(void);
void task_four_stop(void);

/* 起跑线事件: 外部(摄像头/传感器)检测到起跑线时调用, 置 pl_event 推进状态机. */
void set_start_line_event(void);

void control_task(void);

/* 调试/外部查询标志.
 * 注: pl_event / car_state 等状态变量定义在 Control.c (Control.h extern),
 *     此处只导出 task.c 定义的菜单可读量. */
extern float   angle_sumer;         /* 角度累计 (T4 圈数判定, 菜单 READONLY) */
extern float   g_run_sec;           /* 减速触发时间(秒), 菜单 PARAM + AT24C02 持久化 */
extern float   g_angle_stop_deg;   /* 减速触发角度阈值(累计°), 菜单 PARAM + AT24C02 持久化 */
extern float   g_yaw_stop_min;     /* 停车 yaw 下限(°), 菜单 PARAM + AT24C02 持久化 */
extern float   g_yaw_stop_max;     /* 停车 yaw 上限(°), 菜单 PARAM + AT24C02 持久化 */
extern float   g_t3_pos;           /* T3 正目标距离(默认+50), set_ball_position 用, 菜单 PARAM + AT24C02 持久化 */
extern float   g_t3_wait;          /* T3 负目标距离(默认-45), wait_ball_pos 用, 菜单 PARAM + AT24C02 持久化 */

#endif /* TASK_H */
