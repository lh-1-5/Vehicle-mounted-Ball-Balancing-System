#include "key.h"

/* 按键表: 引脚宏由 SysConfig 生成 (GPIO_KEYS 实例).
 * 初始状态全为释放/无事件/无回调, 由 key_init() 和 key_set_callback() 配. */
static Key_T keys[KEY_NUM] = {
    {GPIO_KEYS_K0_PORT, GPIO_KEYS_K0_PIN, KEY_STATE_RELEASED, KEY_EVENT_NONE, 0, 0, 0, NULL},
    {GPIO_KEYS_K1_PORT, GPIO_KEYS_K1_PIN, KEY_STATE_RELEASED, KEY_EVENT_NONE, 0, 0, 0, NULL},
    {GPIO_KEYS_K2_PORT, GPIO_KEYS_K2_PIN, KEY_STATE_RELEASED, KEY_EVENT_NONE, 0, 0, 0, NULL},
    {GPIO_KEYS_K3_PORT, GPIO_KEYS_K3_PIN, KEY_STATE_RELEASED, KEY_EVENT_NONE, 0, 0, 0, NULL},
};

void key_init(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        keys[i].state        = KEY_STATE_RELEASED;
        keys[i].event        = KEY_EVENT_NONE;
        keys[i].press_time   = 0;
        keys[i].repeat_next  = 0;
        keys[i].long_fired   = 0;
        keys[i].callback     = NULL;
    }
}

void key_set_callback(uint8_t id, Key_Callback cb)
{
    if (id < KEY_NUM)
        keys[id].callback = cb;
}

/* 扫描: 每 KEY_SCAN_MS 在定时器中断里调一次.
 * 三状态机: RELEASED -> DEBOUNCE -> PRESSED
 *   按下=引脚低 (上拉+接GND). */
void key_scan(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++) 
    {
        Key_T *k = &keys[i];
        /* 读引脚: 结果 0 = 低 = 按下 */
        uint8_t pressed = (DL_GPIO_readPins(k->port, k->pin) == 0);

        switch (k->state) 
        {

        case KEY_STATE_RELEASED:
            if (pressed) {                      /* 刚按下, 进消抖 */
                k->state      = KEY_STATE_DEBOUNCE;
                k->press_time = 0;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            k->press_time += KEY_SCAN_MS;
            if (pressed && k->press_time >= KEY_DEBOUNCE_MS) {
                k->state = KEY_STATE_PRESSED;   /* 持续按住, 确认按下 */
            } else if (!pressed) {
                k->state = KEY_STATE_RELEASED;  /* 没按稳, 当抖动丢弃 */
            }
            break;

        case KEY_STATE_PRESSED:
            k->press_time += KEY_SCAN_MS;
            if (!pressed) {                     /* 松开 */
                /* 没到长按阈值 -> 短按; 到过长按则长按事件已发过, 松开不再发 */
                if (!k->long_fired && k->event == KEY_EVENT_NONE)
                    k->event = KEY_EVENT_SHORT_PRESS;
                k->state       = KEY_STATE_RELEASED;
                k->press_time  = 0;
                k->long_fired  = 0;
            } 
            else 
            {                            /* 仍按住 */
                if (!k->long_fired && k->press_time >= KEY_LONG_PRESS_MS) {
                    /* 首次达到长按阈值, 触发一次 LONG_PRESS */
                    if (k->event == KEY_EVENT_NONE) {
                        k->event       = KEY_EVENT_LONG_PRESS;
                        k->long_fired  = 1;
                        k->repeat_next = k->press_time + KEY_LONG_REPEAT_MS;
                    }
                } else if (k->long_fired && k->press_time >= k->repeat_next) {
                    /* 之后每 KEY_LONG_REPEAT_MS 触发一次 LONG_REPEAT */
                    if (k->event == KEY_EVENT_NONE) {
                        k->event        = KEY_EVENT_LONG_REPEAT;
                        k->repeat_next += KEY_LONG_REPEAT_MS;
                    }
                }
            }
            break;
        }
    }
}

/* 派发: 主循环里调, 把事件交给回调后清掉.
 * event 加了 "只在 NONE 时才写" 的保护, 所以 ISR 不会覆盖未消费的事件,
 * 这里读完清零即可, 不会丢短按/首次长按. */
void key_process(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++) 
    {
        if (keys[i].event != KEY_EVENT_NONE) 
        {
            if (keys[i].callback != NULL)
                keys[i].callback(i, keys[i].event);
            keys[i].event = KEY_EVENT_NONE;
        }
    }
}
