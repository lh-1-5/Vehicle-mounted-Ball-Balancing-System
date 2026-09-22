#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ========================= 按键模块 =========================
 * 4 个按键, 一端接 GND, 用内部上拉, 按下 = 低电平.
 * 引脚由 SysConfig 的 GPIO_KEYS 实例配置:
 *   K0=PB21  K1=PB11  K2=PA8  K3=PA7
 *
 * 工作方式 (扫描/派发分离, 经典做法):
 *   - key_scan()   在定时器中断里调, 每 KEY_SCAN_MS 一次, 做消抖+事件检测.
 *   - key_process() 在主循环里调, 把检测到的事件派发给回调.
 *   ISR 只管检测, 主循环才做重活 (刷屏/改参数/响蜂鸣), 互不阻塞.
 *
 * 用法:
 *   key_init();
 *   key_set_callback(0, my_handler);        // 给 0 号键注册回调
 *   // SysTick(1ms) 里每 10ms 调一次 key_scan();
 *   while (1) { key_process(); ... }
 *
 * 事件:
 *   SHORT_PRESS  短按 (松开时触发)
 *   LONG_PRESS   长按到阈值 (按住时触发一次)
 *   LONG_REPEAT  长按持续 (之后每 KEY_LONG_REPEAT_MS 触发一次, 用于步进调参)
 */

#define KEY_NUM             4       /* 按键数量 */
#define KEY_SCAN_MS         10      /* key_scan() 调用间隔 (ms) */
#define KEY_DEBOUNCE_MS     20      /* 消抖时间 (ms) */
#define KEY_LONG_PRESS_MS   800     /* 长按阈值 (ms) */
#define KEY_LONG_REPEAT_MS  200     /* 长按重复间隔 (ms) */

/* 按键状态 */
typedef enum {
    KEY_STATE_RELEASED = 0,     /* 释放 */
    KEY_STATE_DEBOUNCE,         /* 消抖中 */
    KEY_STATE_PRESSED,          /* 已确认按下 */
} Key_State;

/* 按键事件 */
typedef enum {
    KEY_EVENT_NONE = 0,         /* 无 */
    KEY_EVENT_SHORT_PRESS,      /* 短按 */
    KEY_EVENT_LONG_PRESS,       /* 长按 (触发一次) */
    KEY_EVENT_LONG_REPEAT,      /* 长按持续 (重复触发) */
} Key_Event;

/* 回调函数类型: 参数为 (按键id, 事件) */
typedef void (*Key_Callback)(uint8_t id, Key_Event event);

/* 单个按键描述 */
typedef struct {
    GPIO_Regs    *port;         /* GPIO 端口 */
    uint32_t      pin;          /* GPIO 引脚位 */
    Key_State     state;        /* 当前状态 */
    Key_Event     event;        /* 待处理事件 (ISR 写, 主循环读+清) */
    uint32_t      press_time;   /* 本轮按下时长 (ms) */
    uint32_t      repeat_next;  /* 下次长按重复触发的时间阈值 */
    uint8_t       long_fired;   /* 长按已触发标志 (避免重复触发 LONG_PRESS) */
    Key_Callback  callback;     /* 事件回调, NULL=不回调 */
} Key_T;

/* 初始化状态 (GPIO 由 SysConfig 配, 这里只复位状态机) */
void key_init(void);

/* 扫描: 在定时器中断里每 KEY_SCAN_MS 调一次 */
void key_scan(void);

/* 派发: 在主循环里调, 把事件交给回调 */
void key_process(void);

/* 给指定按键注册回调 */
void key_set_callback(uint8_t id, Key_Callback cb);

#endif /* __KEY_H */
