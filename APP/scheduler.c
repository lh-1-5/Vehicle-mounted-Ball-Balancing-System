/*
 * scheduler.c - 系统节拍 (SysTick 1ms)
 * g_tms 自增 + 每 KEY_SCAN_MS(10ms) 调一次 key_scan.
 * 原在 main.c, Step 3 搬到本模块, 逻辑不变.
 */
#include "scheduler.h"
#include "key.h"
#include "app_menu.h"


volatile int g_tms = 0;

void SysTick_Handler(void)
{
    g_tms++;
    /* SysTick=1ms, 每 KEY_SCAN_MS(10ms) 扫一次按键 */
    static uint8_t key_div = 0;
    if (++key_div >= KEY_SCAN_MS) {
        key_div = 0;
        key_scan();
    }
}

void software_task(void)
{

    static int last_live = 0;   /* 上次实时刷新的 g_tms */
    static int last_led  = 0;   /* 上次心跳翻转的 g_tms */

    key_process();          /* 派发按键 -> on_key -> menu_handle_event / control_toggle_run */
    
    app_menu_refresh();     /* dirty 才刷 OLED, 否则空过 (零成本) */

    /* 实时数据页 (IMU Roll/Pitch/Yaw) 周期刷新: 每 100ms 置脏一次 */
    if (g_tms - last_live >= 100) 
    {
        last_live = g_tms;
        app_menu_invalidate();
    }

    /* 心跳: 时间驱动, 不再 delay_ms 阻塞 */
    if (g_tms - last_led >= 1000) 
    {
        last_led = g_tms;
        DL_GPIO_togglePins(GPIO_CORE_LED_PORT, GPIO_CORE_LED_CORE_LED_PIN);
    }
}
