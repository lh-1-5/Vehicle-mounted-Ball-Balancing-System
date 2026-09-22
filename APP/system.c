/*
 * system.c - 系统初始化与主循环
 *
 * system_init: 按层聚合调用 bsp_init / mid_init / control_init + 应用层 init.
 * system_run:  主循环 - 按键派发 + 控制任务 + 菜单刷新 + 实时页/心跳软周期.
 * main.c 只调这两个函数.
 */
#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "mid.h"
#include "oled.h"
#include "delay.h"
#include "app_menu.h"
#include "key.h"
#include "Control.h"
#include "ball.h"
#include "scheduler.h"
#include "uart.h"
#include "task.h"


/* 按键回调: 转发菜单 + K3(BACK) 长按切换运行状态.
 * 长按不易误触; 若与菜单 BACK 长按冲突, 改为菜单项触发. */
static void on_key(uint8_t id, Key_Event event)
{
    app_menu_on_key(id, event);
}

void system_init(void)
{
    SYSCFG_DL_init();
    bsp_init();

    /* MID: at_params 加载 + attitude 初始化(含零偏标定 ~2.25s) + 启动 200Hz 采样.
     * 期间板子须静止. 失败 -> 显示错误 + 死循环闪灯 (不可继续). */
    if (mid_init() != 0) {
        OLED_Clear();
        OLED_PrintfRC(0, 0, "IMU init FAIL");
        OLED_Update();
        /*
        while (1) {
            DL_GPIO_togglePins(GPIO_CORE_LED_PORT, GPIO_CORE_LED_CORE_LED_PIN);
            delay_ms(500);
        }
        */
    }

    /* 菜单: 建页面表 + 初始化上下文 (须在 OLED_Init 之后, OLED_Init 在 bsp_init 里) */
    app_menu_init();

    /* 按键回调注册 (key_init 在 bsp_init 里) */
    for (uint8_t i = 0; i < KEY_NUM; i++)
        key_set_callback(i, on_key);

    /* 控制: PID 参数 + 启动 100Hz 控制环. start_run=false, 上电不跑 */
    control_init();
    /* 滚球: PID 参数 + 上电同步位置(内环保持原地) + 启动 TIMER_BALL(10ms). 外环默认关 */
    ball_init();
}

void system_run(void)
{
    

    while (1)
    {
        software_task();

        control_task();         /* 控制层主循环任务 */
    }
}
