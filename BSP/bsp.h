#ifndef BSP_H
#define BSP_H

/*
 * ========================= BSP 层初始化聚合 =========================
 * bsp_init(): 一次性初始化所有 BSP 驱动 (uart/led_buzzer/oled/key/encoder/motor/step).
 *             在 SYSCFG_DL_init() 之后调. step_off() 让步进上电停转.
 */

void bsp_init(void);

#endif /* BSP_H */
