#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ========================= 阻塞延时 =========================
 * 基于 delay_cycles 按周期数换算, 不依赖 SysTick, 任何上下文都能用
 * (上电初期的 IMU/OLED 初始化里也能调). CPUCLK_FREQ 由 SysConfig 给出 (本工程 80MHz).
 *
 * 注意:
 *   - 纯阻塞忙等, 延时期间不跑别的代码 (中断仍会打断, 不影响).
 *   - delay_us 小值 (几 us) 因函数调用开销会偏长一点, 粗略用; 要精确 us 级
 *     时序 (如软件 I2C) 还是直接用 delay_cycles.
 */

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif /* __DELAY_H */
