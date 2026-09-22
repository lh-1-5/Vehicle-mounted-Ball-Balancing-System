#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ======================== 循迹传感器 ========================
 * 8 路红外传感器, 巡黑线: 检测到黑线 -> 1, 白线 -> 0.
 * 传感器输出低电平表示黑线, 故读取后取反.
 *
 * 引脚在 .syscfg 的 GPIO_Track 组 (Track_1..Track_8, INPUT+PULL_UP):
 *   Track_1=PB6, Track_2=PB7, Track_3=PB8, Track_4=PB9,
 *   Track_5=PB10, Track_6=PB12, Track_7=PA27, Track_8=PA28
 */

/* 传感器输出低电平=黑线, 读取后取反: 1=黑线 / 0=白线 */
#define Read_Track_1()  (!!DL_GPIO_readPins(GPIO_Track_Track_1_PORT, GPIO_Track_Track_1_PIN))
#define Read_Track_2()  (!!DL_GPIO_readPins(GPIO_Track_Track_2_PORT, GPIO_Track_Track_2_PIN))
#define Read_Track_3()  (!!DL_GPIO_readPins(GPIO_Track_Track_3_PORT, GPIO_Track_Track_3_PIN))
#define Read_Track_4()  (!!DL_GPIO_readPins(GPIO_Track_Track_4_PORT, GPIO_Track_Track_4_PIN))
#define Read_Track_5()  (!!DL_GPIO_readPins(GPIO_Track_Track_5_PORT, GPIO_Track_Track_5_PIN))
#define Read_Track_6()  (!!DL_GPIO_readPins(GPIO_Track_Track_6_PORT, GPIO_Track_Track_6_PIN))
#define Read_Track_7()  (!!DL_GPIO_readPins(GPIO_Track_Track_7_PORT, GPIO_Track_Track_7_PIN))
#define Read_Track_8()  (!!DL_GPIO_readPins(GPIO_Track_Track_8_PORT, GPIO_Track_Track_8_PIN))

/* 8 路传感器最新读数 (0=白线, 1=黑线), 由 track_update() 刷新到 track_sensors[].
 * track_error_get() 内部也调 track_update(), 故运行时自动保持新鲜;
 * 调试页面可独立调 track_update() 取实时电平. */
extern uint8_t track_sensors[8];
void track_update(void);

/* 获取循迹误差: 负值偏左, 正值偏右, 范围 [-10, 10].
 * 丢线(全白)按上次方向返回 ±10; 全黑返回 0. */
int16_t track_error_get(void);

#endif /* TRACK_H */
