#ifndef SYSTEM_H
#define SYSTEM_H

/*
 * ========================= 系统初始化与主循环 =========================
 * system_init(): 各子模块初始化 + 启动定时器 (姿态 200Hz / 控制 100Hz).
 *                IMU 初始化失败则死循环闪灯.
 * system_run():  主循环 - 按键派发 + 菜单 dirty 刷新 + 实时页/心跳软周期.
 * main.c 只调这两个函数.
 */

void system_init(void);
void system_run(void);

#endif /* SYSTEM_H */
