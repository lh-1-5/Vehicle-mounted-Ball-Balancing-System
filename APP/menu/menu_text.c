/**
 * @file    menu_text.c
 * @brief   菜单文本渲染模块实现
 *
 * 本文件将 MenuContext 状态转换为适合字符型 OLED 显示的文本行。
 * 所有函数均不持有状态，完全从 MenuContext 中读取当前状态。
 *
 * 渲染流程:
 *   1. menu_text_get_line() 被调用层循环调用 (row = 0, 1, 2, ...)
 *   2. row=0 → 居中显示页面标题
 *   3. row≥1 → 计算该行对应的菜单项下标 (top_index + row - 1)
 *   4. 判断是否为当前选中项，添加标记符 '>' 或 '*'
 *   5. 根据菜单项类型构建内容字符串，拼接后填入 buffer
 */

#include "menu_text.h"

#include <stdio.h>
#include <string.h>

/* ---------- 缓冲区大小 ---------- */
#define MENU_TEXT_VALUE_BUFFER_SIZE   48U   /**< 数值格式化缓冲区 */
#define MENU_TEXT_CONTENT_BUFFER_SIZE 96U   /**< 完整菜单项内容缓冲区 */
#define MENU_TEXT_MAX_DECIMALS        6U    /**< 浮点小数位上限 */

/* ================================================================
 *  底层文本工具
 * ================================================================ */

/** 用空格填充 buffer 前 width 个字符，末尾加 '\0' */
static void menu_text_fill_blank(char *buffer, size_t width)
{
    memset(buffer, ' ', width);
    buffer[width] = '\0';
}

/**
 * 计算有效显示宽度
 * 取 buffer 能容纳的字符数（减 1 留给 '\0'）与用户指定 width 的较小值
 */
static size_t menu_text_effective_width(size_t buffer_size, uint8_t width)
{
    size_t available;

    if (buffer_size == 0U)
    {
        return 0U;
    }

    available = buffer_size - 1U;
    return width < available ? width : available;
}

/**
 * 将文本居中拷贝到定宽 buffer
 *
 * 示例: width=16, text="Settings"
 *       结果: "    Settings    "
 *
 * 若文本超出宽度则截断。
 */
static void menu_text_copy_centered(char *buffer,
                                    size_t width,
                                    const char *text)
{
    size_t length;
    size_t start;

    menu_text_fill_blank(buffer, width);
    if (text == NULL || width == 0U)
    {
        return;
    }

    length = strlen(text);
    if (length > width)
    {
        length = width;  /* 超长截断 */
    }
    start = (width - length) / 2U;
    memcpy(&buffer[start], text, length);
}

/* ================================================================
 *  参数值读取 — 与 menu_core.c 中同名函数功能相同
 *  独立实现以避免渲染层依赖核心层的内部函数
 * ================================================================ */

/** 从参数地址读取有符号整数值 */
static int64_t menu_text_read_signed(const MenuItem *item)
{
    switch (item->data.param.value_type)
    {
        case MENU_VALUE_INT:
            return *(volatile int *)item->data.param.address;
        case MENU_VALUE_INT8:
            return *(volatile int8_t *)item->data.param.address;
        case MENU_VALUE_INT16:
            return *(volatile int16_t *)item->data.param.address;
        case MENU_VALUE_INT32:
            return *(volatile int32_t *)item->data.param.address;
        default:
            return 0;
    }
}

/** 从参数地址读取无符号整数值 */
static uint64_t menu_text_read_unsigned(const MenuItem *item)
{
    switch (item->data.param.value_type)
    {
        case MENU_VALUE_UINT8:
            return *(volatile uint8_t *)item->data.param.address;
        case MENU_VALUE_UINT16:
            return *(volatile uint16_t *)item->data.param.address;
        case MENU_VALUE_UINT32:
            return *(volatile uint32_t *)item->data.param.address;
        default:
            return 0U;
    }
}

/* ================================================================
 *  数值格式化 — 将参数值转为显示字符串
 * ================================================================ */

/**
 * 将菜单项的当前值格式化为字符串
 *
 * 格式化规则:
 *   FLOAT  → "12.50"  (小数位数由 param.decimals 控制，上限 6)
 *   INT*   → "100"     (有符号十进制)
 *   UINT*  → "255"     (无符号十进制)
 *   BOOL   → "ON" / "OFF"
 *   无效   → "N/A"
 */
static void menu_text_format_value(const MenuItem *item,
                                   char *buffer,
                                   size_t buffer_size)
{
    uint8_t decimals;

    if (buffer_size == 0U)
    {
        return;
    }
    buffer[0] = '\0';

    if (item == NULL || item->data.param.address == NULL)
    {
        (void)snprintf(buffer, buffer_size, "N/A");
        return;
    }

    switch (item->data.param.value_type)
    {
        case MENU_VALUE_FLOAT:
            decimals = item->data.param.decimals > MENU_TEXT_MAX_DECIMALS
                           ? MENU_TEXT_MAX_DECIMALS
                           : item->data.param.decimals;
            (void)snprintf(buffer,
                           buffer_size,
                           "%.*f",
                           (int)decimals,
                           (double)*(volatile float *)item->data.param.address);
            break;

        case MENU_VALUE_INT:
        case MENU_VALUE_INT8:
        case MENU_VALUE_INT16:
        case MENU_VALUE_INT32:
            (void)snprintf(buffer,
                           buffer_size,
                           "%ld",
                           (long)menu_text_read_signed(item));
            break;

        case MENU_VALUE_UINT8:
        case MENU_VALUE_UINT16:
        case MENU_VALUE_UINT32:
            (void)snprintf(buffer,
                           buffer_size,
                           "%lu",
                           (unsigned long)menu_text_read_unsigned(item));
            break;

        case MENU_VALUE_BOOL:
            (void)snprintf(buffer,
                           buffer_size,
                           "%s",
                           *(volatile bool *)item->data.param.address ? "ON" : "OFF");
            break;

        default:
            (void)snprintf(buffer, buffer_size, "N/A");
            break;
    }
}

/* ================================================================
 *  菜单项内容构建 — 根据类型生成完整显示字符串
 * ================================================================ */

/**
 * 构建菜单项的完整显示内容（不含行首标记符）
 *
 * 格式:
 *   PARAM:    "PID-Kp:12.50"
 *   READONLY: "Battery:3800"
 *   ACTION:   "Save()"
 *   SUBMENU:  "Settings >"
 */
static void menu_text_build_content(const MenuItem *item,
                                    char *buffer,
                                    size_t buffer_size)
{
    char value[MENU_TEXT_VALUE_BUFFER_SIZE];

    if (buffer_size == 0U)
    {
        return;
    }
    buffer[0] = '\0';

    if (item == NULL || item->name == NULL)
    {
        return;
    }

    switch (item->type)
    {
        case MENU_ITEM_PARAM:
        case MENU_ITEM_READONLY:
            /* 格式: "name:value" */
            menu_text_format_value(item, value, sizeof(value));
            (void)snprintf(buffer, buffer_size, "%s:%s", item->name, value);
            break;

        case MENU_ITEM_ACTION:
            /* 格式: "name()"，括号表示可执行 */
            (void)snprintf(buffer, buffer_size, "%s()", item->name);
            break;

        case MENU_ITEM_SUBMENU:
            /* 格式: "name >"，箭头表示可进入 */
            (void)snprintf(buffer, buffer_size, "%s >", item->name);
            break;

        default:
            (void)snprintf(buffer, buffer_size, "%s", item->name);
            break;
    }
}

/* ================================================================
 *  公开 API — menu_text_get_line()
 * ================================================================ */

/**
 * 获取菜单的一行文本 — 整个渲染模块的唯一入口
 *
 * 调用层（main 循环的 OLED 刷新部分）应该这样使用:
 *
 *   if (menu_is_dirty(&ctx)) {
 *       for (uint8_t r = 0; r <= menu_visible_rows(&ctx); r++) {
 *           char line[17];
 *           if (menu_text_get_line(&ctx, r, line, sizeof(line), 16)) {
 *               OLED_ShowString(0, r * 16, line);
 *           }
 *       }
 *       menu_clear_dirty(&ctx);
 *   }
 *
 * 渲染细节:
 *   - row=0: 标题行，居中显示 page->title
 *   - row≥1: 菜单项行
 *       - 行首第一个字符是标记符:
 *         '>' = 当前高亮项（浏览模式）
 *         '*' = 当前高亮项（编辑模式）
 *         ' ' = 非高亮项
 *       - 后续字符是菜单项内容
 *
 * @return true=成功获取，false=row 超出范围或参数无效
 */
bool menu_text_get_line(const MenuContext *context,
                        uint8_t row,
                        char *buffer,
                        size_t buffer_size,
                        uint8_t width)
{
    const MenuPage *page;
    const MenuItem *item;
    size_t effective_width;
    size_t content_width;
    size_t content_length;
    uint16_t item_index;
    char content[MENU_TEXT_CONTENT_BUFFER_SIZE];
    char marker;

    /* 参数校验 */
    if (context == NULL || buffer == NULL || buffer_size == 0U || width == 0U)
    {
        return false;
    }

    effective_width = menu_text_effective_width(buffer_size, width);
    if (effective_width == 0U)
    {
        buffer[0] = '\0';
        return false;
    }

    page = menu_current_page(context);
    if (page == NULL)
    {
        menu_text_fill_blank(buffer, effective_width);
        return false;
    }

    /* -------- row = 0: 居中标题行 -------- */
    if (row == 0U)
    {
        menu_text_copy_centered(buffer, effective_width, page->title);
        return true;
    }

    /* -------- row ≥ 1: 菜单项行 -------- */
    menu_text_fill_blank(buffer, effective_width);

    /* row 超出可显示范围 */
    if (row > menu_visible_rows(context))
    {
        return false;
    }

    /* 计算该行对应的菜单项在页面中的下标:
     *   item_index = top_index + (row - 1)
     *   row=1 对应屏幕第一项，row=N 对应屏幕第 N 项 */
    item_index = (uint16_t)(menu_top_index(context) +
                            (uint16_t)(row - (uint8_t)1U));
    if (item_index >= page->item_count)
    {
        return true;  /* 空白行，已填充空格 */
    }

    item = &page->items[item_index];

    /* 确定行首标记符:
     *   '>' = 当前选中项（浏览模式）
     *   '*' = 当前选中项（编辑模式）
     *   ' ' = 其他项 */
    marker = ' ';
    if (item_index == menu_selected_index(context))
    {
        marker = menu_is_editing(context) ? '*' : '>';
    }
    buffer[0] = marker;

    if (effective_width <= 1U)
    {
        return true;  /* 屏幕太窄，只能显示标记符 */
    }

    /* 构建菜单项内容并拼接到标记符之后 */
    menu_text_build_content(item, content, sizeof(content));
    content_width = effective_width - 1U;
    content_length = strlen(content);
    if (content_length > content_width)
    {
        content_length = content_width;  /* 截断超长内容 */
    }
    memcpy(&buffer[1], content, content_length);
    return true;
}
