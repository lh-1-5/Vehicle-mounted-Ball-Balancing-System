/**
 * @file    menu_text.h
 * @brief   菜单文本渲染模块 — 将菜单状态转换为 OLED 可显示的文本行
 *
 * 本模块是 menu_core 的渲染后端，负责将 MenuContext 中的状态格式化为
 * 适合字符型 OLED（如 SSD1306 128x64）逐行显示的文本字符串。
 *
 * 设计原则:
 *   - 与硬件完全解耦：只输出字符串，不关心如何把字符串送到屏幕
 *   - 单行渲染：每次调用只生成一行文本，由调用层循环调用
 *   - 无状态：不保存任何渲染状态，每次都从 MenuContext 重新读取
 *
 * 典型用法（在 OLED 刷新循环中）:
 *   for (uint8_t row = 0; row <= visible_rows; row++) {
 *       char line[17];  // 16 字符 + '\0'
 *       menu_text_get_line(&ctx, row, line, sizeof(line), 16);
 *       OLED_ShowString(0, row * 16, line, 16);
 *   }
 *
 * row 布局:
 *   row 0           → 页面标题（居中显示）
 *   row 1..N        → 菜单项内容
 *     - 当前高亮项前有 '>' (浏览模式) 或 '*' (编辑模式) 标记
 *     - PARAM/READONLY 显示 "name:value"
 *     - ACTION         显示 "name()"
 *     - SUBMENU        显示 "name >"
 */

#ifndef MENU_TEXT_H
#define MENU_TEXT_H

#include "menu_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 获取菜单的一行文本
 *
 * @param context      菜单上下文（只读）
 * @param row          行号：0 = 标题行，1..visible_rows = 菜单项
 * @param buffer       输出缓冲区
 * @param buffer_size  缓冲区大小（至少 width + 1）
 * @param width        显示宽度（字符数），如 128px / 8px = 16
 * @return             true = 有效行，false = row 超出范围或参数无效
 *
 * 格式说明:
 *   标题行: [    居中标题    ]  (width 个字符，空格填充)
 *   菜单行: [标记][内容......]  标记 = '>' 选中 / '*' 编辑中 / ' ' 普通
 *
 *   不同类型菜单项的内容格式:
 *     PARAM:    "name:12.50"
 *     READONLY: "name:100"
 *     ACTION:   "name()"
 *     SUBMENU:  "name >"
 */
bool menu_text_get_line(const MenuContext *context,
                        uint8_t row,
                        char *buffer,
                        size_t buffer_size,
                        uint8_t width);

#ifdef __cplusplus
}
#endif

#endif
