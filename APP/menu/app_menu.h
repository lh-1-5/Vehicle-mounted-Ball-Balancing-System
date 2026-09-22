#ifndef APP_MENU_H
#define APP_MENU_H

#include <stdint.h>
#include "key.h"   /* Key_Event */

/*
 * app_menu - 把 MenuCore (menu_core + menu_text) 接到本工程 OLED + 按键上.
 *
 * 页面表 / MenuContext / 按键->事件映射都在 app_menu.c 内, 对外只暴露四个函数.
 * MenuCore 本身与硬件解耦, 本模块是它和 oled.h / key.h / attitude.h 之间的薄胶水.
 *
 * 刷新模型 (主循环调用):
 *   app_menu_refresh()     -- dirty 才重画 + OLED_Update, 否则空过 (零成本).
 *   app_menu_invalidate()  -- 强制置 dirty, 给实时数据页 (IMU Roll/Pitch/Yaw) 周期刷新.
 *
 * 按键映射 (默认, 按物理布局可在 app_menu.c 改):
 *   K0=DOWN  K1=UP  K2=ENTER  K3=BACK
 *   UP/DOWN: 短按一步, 长按(LONG_PRESS+LONG_REPEAT)连滚.
 *   ENTER/BACK: 仅短按.
 */

/* 初始化: 建页面表 + menu_init. 须在 OLED_Init() 之后调. */
void app_menu_init(void);

/* 主循环调: dirty 才重画 + OLED_Update, 否则空过. */
void app_menu_refresh(void);

/* 强制置 dirty, 供实时数据页周期刷新用 (主循环按节拍调). */
void app_menu_invalidate(void);

/* 按键回调, 签名 = Key_Callback, 直接注册给 4 个键. */
void app_menu_on_key(uint8_t id, Key_Event event);

/* 发车计时器: task.c 在 CAR_RUNNING 进入时调 start (切到 Timer 页 + 开始计时),
 * CAR_BRAKING 进入时调 stop (冻结). 不再由按键触发. */
void app_menu_timer_start(void);
void app_menu_timer_stop(void);

#endif /* APP_MENU_H */
