#ifndef SCHEDULER_H
#define SCHEDULER_H

/*
 * ========================= 系统节拍 (SysTick 1ms) =========================
 * g_tms: SysTick(1ms) 自增计数. ISR 写 / 主循环读 (差值判断软周期) -> volatile.
 * SysTick_Handler 在 scheduler.c: g_tms++ + 每 KEY_SCAN_MS(10ms) 调一次 key_scan.
 */

extern volatile int g_tms;

void software_task(void);

#endif /* SCHEDULER_H */
