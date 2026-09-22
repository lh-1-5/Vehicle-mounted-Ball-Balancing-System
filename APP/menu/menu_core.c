/**
 * @file    menu_core.c
 * @brief   通用多级菜单系统 — 核心状态机实现
 *
 * 本文件实现了 menu_core.h 中声明的所有 API。
 *
 * 架构概述:
 *   菜单系统是一个基于「导航栈」的状态机:
 *
 *   ┌──────────────────────────────────┐
 *   │  MenuContext                     │
 *   │  ┌────────────────────────────┐  │
 *   │  │ frames[0]  (根页面)        │   │  ← depth=0
 *   │  │ frames[1]  (一级子菜单)    │   │
 *   │  │ frames[2]  (二级子菜单)    │   │
 *   │  │ ...                        │  │
 *   │  └────────────────────────────┘  │
 *   │  depth, editing, dirty, ...      │
 *   └──────────────────────────────────┘
 *
 *   状态机有两层模式:
 *     - 外层: 浏览模式 vs 编辑模式 (context->editing)
 *     - 浏览模式: UP/DOWN 移动光标, ENTER 进入子菜单/编辑/动作, BACK 返回上级
 *     - 编辑模式: UP/DOWN 增减参数值, ENTER 提交, BACK 取消(恢复快照)
 *
 *   数据读取/写入直接通过 volatile 指针操作真实变量，无中间缓存。
 */

#include "menu_core.h"

#include <float.h>
#include <limits.h>
#include <string.h>

/* ---------- 前置声明 ---------- */
static void menu_signed_limits(MenuValueType type,
                               double *type_min,
                               double *type_max);
static double menu_unsigned_max(MenuValueType type);
static bool menu_is_signed_type(MenuValueType type);
static bool menu_is_unsigned_type(MenuValueType type);

/* ================================================================
 *  内部工具函数
 * ================================================================ */

/**
 * 验证 MenuPage 结构体的合法性
 * page 非空、title 非空、如果有菜单项则 items 数组不能为空
 */
static bool menu_page_is_valid(const MenuPage *page)
{
    if (page == NULL || page->title == NULL)
    {
        return false;
    }

    return page->item_count == 0U || page->items != NULL;
}

/**
 * 获取当前深度的栈帧（可修改版本）
 * 返回 NULL 表示 context 无效或深度越界
 */
static MenuFrame *menu_current_frame_mut(MenuContext *context)
{
    if (context == NULL || context->depth >= MENU_MAX_DEPTH)
    {
        return NULL;
    }

    return &context->frames[context->depth];
}

/**
 * 获取当前深度的栈帧（只读版本）
 */
static const MenuFrame *menu_current_frame_const(const MenuContext *context)
{
    if (context == NULL || context->depth >= MENU_MAX_DEPTH)
    {
        return NULL;
    }

    return &context->frames[context->depth];
}

/**
 * 自动滚动逻辑 — 保证当前选中项始终在可见区域内
 *
 * 当 selected_index 移出屏幕范围时，调整 top_index:
 *   - selected < top           → top 上移
 *   - selected >= top + rows   → top 下移
 *   - top > max_top            → 钳制到有效最大值
 */
static void menu_adjust_scroll(MenuContext *context)
{
    MenuFrame *frame = menu_current_frame_mut(context);
    uint16_t max_top;

    if (frame == NULL || frame->page == NULL || frame->page->item_count == 0U)
    {
        return;
    }

    if (frame->selected_index < frame->top_index)
    {
        frame->top_index = frame->selected_index;
    }
    else if (frame->selected_index >=
             (uint16_t)(frame->top_index + context->visible_rows))
    {
        frame->top_index =
            (uint16_t)(frame->selected_index -
                       (uint16_t)context->visible_rows + (uint16_t)1U);
    }

    max_top = frame->page->item_count > context->visible_rows
                  ? (uint16_t)(frame->page->item_count - context->visible_rows)
                  : 0U;

    if (frame->top_index > max_top)
    {
        frame->top_index = max_top;
    }
}

/**
 * 判断一个菜单项是否可编辑
 *
 * 条件:
 *   1. 类型必须为 MENU_ITEM_PARAM
 *   2. address 不能为 NULL
 *   3. BOOL 类型始终可编辑（直接翻转）
 *   4. 其他类型需要 step > 0 且 min ≤ max
 *   5. 限制值必须在类型能表达的范围内
 */
static bool menu_item_can_edit(const MenuItem *item)
{
    double type_min;
    double type_max;

    if (item == NULL || item->type != MENU_ITEM_PARAM ||
        item->data.param.address == NULL)
    {
        return false;
    }

    /* BOOL 类型直接翻转，不需要步长和范围检查 */
    if (item->data.param.value_type == MENU_VALUE_BOOL)
    {
        return true;
    }

    /* 基础合法性：步长必须为正，min ≤ max */
    if (item->data.param.step <= 0.0 ||
        item->data.param.min_value > item->data.param.max_value)
    {
        return false;
    }

    /* FLOAT 类型：范围需与 FLT_MAX 有交集 */
    if (item->data.param.value_type == MENU_VALUE_FLOAT)
    {
        return item->data.param.max_value >= -FLT_MAX &&
               item->data.param.min_value <= FLT_MAX;
    }

    /* 有符号整数：范围需与类型极限有交集 */
    if (menu_is_signed_type(item->data.param.value_type))
    {
        menu_signed_limits(item->data.param.value_type, &type_min, &type_max);
        return item->data.param.max_value >= type_min &&
               item->data.param.min_value <= type_max;
    }

    /* 无符号整数：范围需与 [0, UMAX] 有交集 */
    if (menu_is_unsigned_type(item->data.param.value_type))
    {
        type_max = menu_unsigned_max(item->data.param.value_type);
        return item->data.param.max_value >= 0.0 &&
               item->data.param.min_value <= type_max;
    }

    return false;
}

/* ================================================================
 *  类型化读写 — 通过 volatile 指针直接操作真实变量
 * ================================================================ */

/**
 * 从参数地址读取有符号整数值（根据类型自动选择宽度）
 * 使用 volatile 保证每次从内存重新读取
 */
static int64_t menu_read_signed(const MenuItem *item)
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
static uint64_t menu_read_unsigned(const MenuItem *item)
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

/** 将有符号整数值写入参数地址（自动截断到类型宽度） */
static void menu_write_signed(const MenuItem *item, int64_t value)
{
    switch (item->data.param.value_type)
    {
        case MENU_VALUE_INT:
            *(volatile int *)item->data.param.address = (int)value;
            break;
        case MENU_VALUE_INT8:
            *(volatile int8_t *)item->data.param.address = (int8_t)value;
            break;
        case MENU_VALUE_INT16:
            *(volatile int16_t *)item->data.param.address = (int16_t)value;
            break;
        case MENU_VALUE_INT32:
            *(volatile int32_t *)item->data.param.address = (int32_t)value;
            break;
        default:
            break;
    }
}

/** 将无符号整数值写入参数地址 */
static void menu_write_unsigned(const MenuItem *item, uint64_t value)
{
    switch (item->data.param.value_type)
    {
        case MENU_VALUE_UINT8:
            *(volatile uint8_t *)item->data.param.address = (uint8_t)value;
            break;
        case MENU_VALUE_UINT16:
            *(volatile uint16_t *)item->data.param.address = (uint16_t)value;
            break;
        case MENU_VALUE_UINT32:
            *(volatile uint32_t *)item->data.param.address = (uint32_t)value;
            break;
        default:
            break;
    }
}

/* ================================================================
 *  类型范围查询
 * ================================================================ */

/** 获取有符号整数类型的 [min, max] 范围 */
static void menu_signed_limits(MenuValueType type,
                               double *type_min,
                               double *type_max)
{
    switch (type)
    {
        case MENU_VALUE_INT8:
            *type_min = INT8_MIN;
            *type_max = INT8_MAX;
            break;
        case MENU_VALUE_INT16:
            *type_min = INT16_MIN;
            *type_max = INT16_MAX;
            break;
        case MENU_VALUE_INT32:
            *type_min = INT32_MIN;
            *type_max = INT32_MAX;
            break;
        case MENU_VALUE_INT:
        default:
            *type_min = INT_MIN;
            *type_max = INT_MAX;
            break;
    }
}

/** 获取无符号整数类型的最大值 */
static double menu_unsigned_max(MenuValueType type)
{
    switch (type)
    {
        case MENU_VALUE_UINT8:
            return UINT8_MAX;
        case MENU_VALUE_UINT16:
            return UINT16_MAX;
        case MENU_VALUE_UINT32:
        default:
            return UINT32_MAX;
    }
}

/** 将 double 值钳制到 [minimum, maximum] 区间 */
static double menu_clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/** 判断是否为有符号整数类型 */
static bool menu_is_signed_type(MenuValueType type)
{
    return type == MENU_VALUE_INT || type == MENU_VALUE_INT8 ||
           type == MENU_VALUE_INT16 || type == MENU_VALUE_INT32;
}

/** 判断是否为无符号整数类型 */
static bool menu_is_unsigned_type(MenuValueType type)
{
    return type == MENU_VALUE_UINT8 || type == MENU_VALUE_UINT16 ||
           type == MENU_VALUE_UINT32;
}

/* ================================================================
 *  编辑快照 — 进入编辑模式时保存，取消时恢复
 * ================================================================ */

/**
 * 保存编辑快照 — 在进入编辑模式前调用
 * 根据参数类型将当前值存入 context->edit_snapshot
 */
static void menu_save_edit_snapshot(MenuContext *context,
                                    const MenuItem *item)
{
    if (item->data.param.value_type == MENU_VALUE_FLOAT)
    {
        context->edit_snapshot.float_value =
            *(volatile float *)item->data.param.address;
    }
    else if (menu_is_signed_type(item->data.param.value_type))
    {
        context->edit_snapshot.signed_value = menu_read_signed(item);
    }
    else if (menu_is_unsigned_type(item->data.param.value_type))
    {
        context->edit_snapshot.unsigned_value = menu_read_unsigned(item);
    }
    else if (item->data.param.value_type == MENU_VALUE_BOOL)
    {
        context->edit_snapshot.bool_value =
            *(volatile bool *)item->data.param.address;
    }
}

/**
 * 恢复编辑快照 — 用户按 Back 取消编辑时调用
 * 将快照中的值写回真实变量，实现撤销
 */
static void menu_restore_edit_snapshot(MenuContext *context,
                                       const MenuItem *item)
{
    if (item == NULL || item->data.param.address == NULL)
    {
        return;
    }

    if (item->data.param.value_type == MENU_VALUE_FLOAT)
    {
        *(volatile float *)item->data.param.address =
            context->edit_snapshot.float_value;
    }
    else if (menu_is_signed_type(item->data.param.value_type))
    {
        menu_write_signed(item, context->edit_snapshot.signed_value);
    }
    else if (menu_is_unsigned_type(item->data.param.value_type))
    {
        menu_write_unsigned(item, context->edit_snapshot.unsigned_value);
    }
    else if (item->data.param.value_type == MENU_VALUE_BOOL)
    {
        *(volatile bool *)item->data.param.address =
            context->edit_snapshot.bool_value;
    }
}

/* ================================================================
 *  参数值修改 — 编辑模式下 UP/DOWN 按键的处理核心
 * ================================================================ */

/**
 * 按指定方向修改参数值
 *
 * @param item      要修改的参数项
 * @param direction +1 = 增加, -1 = 减少
 * @return          true = 值确实改变了, false = 已达边界或无变化
 *
 * 处理逻辑:
 *   1. BOOL: 直接翻转
 *   2. FLOAT: candidate = *value + step*direction, 钳制后写回
 *   3. 有符号整数: 同上，使用 int64_t 中间量防溢出
 *   4. 无符号整数: 同上，使用 uint64_t 中间量
 *
 * 限制值同时受用户配置的 [min_value, max_value] 和类型固有范围
 * 的双重约束，取两者的交集。
 */
static bool menu_modify_value(const MenuItem *item, int direction)
{
    double candidate;
    double minimum;
    double maximum;

    if (!menu_item_can_edit(item))
    {
        return false;
    }

    /* --- BOOL: 直接翻转 --- */
    if (item->data.param.value_type == MENU_VALUE_BOOL)
    {
        volatile bool *value = (volatile bool *)item->data.param.address;
        *value = !*value;
        return true;
    }

    /* --- FLOAT: 浮点运算 + 钳制 --- */
    if (item->data.param.value_type == MENU_VALUE_FLOAT)
    {
        volatile float *value = (volatile float *)item->data.param.address;
        minimum = item->data.param.min_value < -FLT_MAX
                      ? -FLT_MAX
                      : item->data.param.min_value;
        maximum = item->data.param.max_value > FLT_MAX
                      ? FLT_MAX
                      : item->data.param.max_value;
        candidate = (double)*value + item->data.param.step * direction;
        candidate = menu_clamp_double(candidate, minimum, maximum);

        if ((float)candidate == *value)
        {
            return false;  /* 步长太小，float 精度不足以区分 */
        }

        *value = (float)candidate;
        return true;
    }

    /* --- 有符号整数 --- */
    if (menu_is_signed_type(item->data.param.value_type))
    {
        int64_t old_value = menu_read_signed(item);
        int64_t new_value;
        double type_min;
        double type_max;

        menu_signed_limits(item->data.param.value_type, &type_min, &type_max);

        /* 取用户配置范围与类型固有范围的交集 */
        minimum = item->data.param.min_value < type_min
                      ? type_min
                      : item->data.param.min_value;
        maximum = item->data.param.max_value > type_max
                      ? type_max
                      : item->data.param.max_value;

        candidate = (double)old_value + item->data.param.step * direction;
        candidate = menu_clamp_double(candidate, minimum, maximum);
        new_value = (int64_t)candidate;

        if (new_value == old_value)
        {
            return false;
        }

        menu_write_signed(item, new_value);
        return true;
    }

    /* --- 无符号整数 --- */
    if (menu_is_unsigned_type(item->data.param.value_type))
    {
        uint64_t old_value = menu_read_unsigned(item);
        uint64_t new_value;

        minimum = item->data.param.min_value < 0.0
                      ? 0.0
                      : item->data.param.min_value;
        maximum = menu_unsigned_max(item->data.param.value_type);
        if (item->data.param.max_value < maximum)
        {
            maximum = item->data.param.max_value;
        }

        candidate = (double)old_value + item->data.param.step * direction;
        candidate = menu_clamp_double(candidate, minimum, maximum);
        new_value = (uint64_t)candidate;

        if (new_value == old_value)
        {
            return false;
        }

        menu_write_unsigned(item, new_value);
        return true;
    }

    return false;
}

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * 初始化菜单系统
 *
 * 将 context 清零，设置根页面和可见行数，并标记为脏（触发首次渲染）。
 * 所有参数校验失败时返回对应错误码，不修改 context。
 */
MenuStatus menu_init(MenuContext *context,
                     const MenuPage *root_page,
                     uint8_t visible_rows)
{
    if (context == NULL || root_page == NULL)
    {
        return MENU_STATUS_NULL_ARGUMENT;
    }
    if (!menu_page_is_valid(root_page))
    {
        return MENU_STATUS_INVALID_PAGE;
    }
    if (visible_rows == 0U)
    {
        return MENU_STATUS_INVALID_VISIBLE_ROWS;
    }

    memset(context, 0, sizeof(*context));
    context->frames[0].page = root_page;
    context->visible_rows = visible_rows;
    context->dirty = true;
    return MENU_STATUS_OK;
}

void menu_set_commit_callback(MenuContext *context,
                              MenuCommitCallback callback)
{
    if (context != NULL)
    {
        context->commit_callback = callback;
    }
}

/**
 * 菜单事件处理 — 核心状态机
 *
 * 这是整个菜单系统的「心脏」。所有用户交互都通过此函数驱动。
 *
 * 状态机分为两层:
 *
 * ┌─────────────────────────────────────────────────────┐
 * │               menu_handle_event()                   │
 * │                                                     │
 * │  if (editing) ─── 编辑模式 ───┐                     │
 * │     UP/DOWN  → menu_modify_value()                  │
 * │     ENTER    → 提交，退出编辑，回调 commit_callback │
 * │     BACK     → 恢复快照，退出编辑                    │
 * │                                                     │
 * │  else ─── 浏览模式 ───────────┤                     │
 * │     UP/DOWN  → 移动光标 + 自动滚动                   │
 * │     ENTER:                                          │
 * │       PARAM   → 进入编辑模式（保存快照）             │
 * │       ACTION  → 调用回调函数                         │
 * │       SUBMENU → push 新栈帧，进入子页面              │
 * │     BACK     → pop 栈帧，返回上级页面                │
 * └─────────────────────────────────────────────────────┘
 *
 * @return MenuResult 告知调用层发生了什么变化
 */
MenuResult menu_handle_event(MenuContext *context, MenuEvent event)
{
    MenuFrame *frame;
    const MenuItem *item;

    if (context == NULL)
    {
        return MENU_RESULT_INVALID_ITEM;
    }

    frame = menu_current_frame_mut(context);
    if (frame == NULL || !menu_page_is_valid(frame->page))
    {
        return MENU_RESULT_INVALID_ITEM;
    }

    item = menu_selected_item(context);

    /* ================================================================
     *  编辑模式: UP/DOWN 修改值，ENTER 提交，BACK 取消
     * ================================================================ */
    if (context->editing)
    {
        /* 安全防护：如果当前选中项不再是 PARAM 类型，强制退出编辑 */
        if (item == NULL || item->type != MENU_ITEM_PARAM)
        {
            context->editing = false;
            context->dirty = true;
            return MENU_RESULT_INVALID_ITEM;
        }

        switch (event)
        {
            case MENU_EVENT_UP:
                /* 编辑模式下 UP = 增加值 */
                if (menu_modify_value(item, 1))
                {
                    context->dirty = true;
                    return MENU_RESULT_VALUE_CHANGED;
                }
                return MENU_RESULT_NONE;

            case MENU_EVENT_DOWN:
                /* 编辑模式下 DOWN = 减少值 */
                if (menu_modify_value(item, -1))
                {
                    context->dirty = true;
                    return MENU_RESULT_VALUE_CHANGED;
                }
                return MENU_RESULT_NONE;

            case MENU_EVENT_ENTER:
                /* 确认修改，退出编辑，触发提交回调 */
                context->editing = false;
                context->dirty = true;
                if (context->commit_callback != NULL)
                {
                    context->commit_callback(item);
                }
                return MENU_RESULT_EDIT_COMMITTED;

            case MENU_EVENT_BACK:
                /* 放弃修改，从快照恢复原始值 */
                menu_restore_edit_snapshot(context, item);
                context->editing = false;
                context->dirty = true;
                return MENU_RESULT_EDIT_CANCELLED;

            default:
                return MENU_RESULT_NONE;
        }
    }

    /* ================================================================
     *  浏览模式: UP/DOWN 导航，ENTER 进入/触发，BACK 返回
     * ================================================================ */
    switch (event)
    {
        case MENU_EVENT_UP:
            /* 光标循环上移（到顶时跳转到最后一项） */
            if (frame->page->item_count == 0U)
            {
                return MENU_RESULT_NONE;
            }
            frame->selected_index = frame->selected_index == 0U
                                        ? (uint16_t)(frame->page->item_count - 1U)
                                        : (uint16_t)(frame->selected_index - 1U);
            menu_adjust_scroll(context);
            context->dirty = true;
            return MENU_RESULT_MOVED;

        case MENU_EVENT_DOWN:
            /* 光标循环下移（到底时跳转到第一项） */
            if (frame->page->item_count == 0U)
            {
                return MENU_RESULT_NONE;
            }
            frame->selected_index =
                (uint16_t)((frame->selected_index + 1U) % frame->page->item_count);
            menu_adjust_scroll(context);
            context->dirty = true;
            return MENU_RESULT_MOVED;

        case MENU_EVENT_ENTER:
            if (item == NULL)
            {
                return MENU_RESULT_NONE;
            }

            /* --- PARAM: 进入编辑模式 --- */
            if (item->type == MENU_ITEM_PARAM)
            {
                if (!menu_item_can_edit(item))
                {
                    return MENU_RESULT_INVALID_ITEM;
                }
                menu_save_edit_snapshot(context, item);
                context->editing = true;
                context->dirty = true;
                return MENU_RESULT_EDIT_STARTED;
            }

            /* --- ACTION: 触发回调 --- */
            if (item->type == MENU_ITEM_ACTION)
            {
                if (item->data.action == NULL)
                {
                    return MENU_RESULT_INVALID_ITEM;
                }
                item->data.action();
                context->dirty = true;
                return MENU_RESULT_ACTION_CALLED;
            }

            /* --- SUBMENU: 进入子页面 --- */
            if (item->type == MENU_ITEM_SUBMENU)
            {
                if (!menu_page_is_valid(item->data.page))
                {
                    return MENU_RESULT_INVALID_ITEM;
                }
                if ((uint8_t)(context->depth + 1U) >= MENU_MAX_DEPTH)
                {
                    return MENU_RESULT_DEPTH_LIMIT;
                }

                /* 在导航栈上压入新帧 */
                context->depth++;
                context->frames[context->depth].page = item->data.page;
                context->frames[context->depth].selected_index = 0U;
                context->frames[context->depth].top_index = 0U;
                context->dirty = true;
                return MENU_RESULT_PAGE_ENTERED;
            }

            return MENU_RESULT_NONE;

        case MENU_EVENT_BACK:
            /* 根页面无法再返回 */
            if (context->depth == 0U)
            {
                return MENU_RESULT_NONE;
            }
            /* 清除当前帧并弹出 */
            memset(&context->frames[context->depth],
                   0,
                   sizeof(context->frames[context->depth]));
            context->depth--;
            context->dirty = true;
            return MENU_RESULT_PAGE_LEFT;

        default:
            return MENU_RESULT_NONE;
    }
}

/* ================================================================
 *  脏标记管理 — 用于驱动 OLED 等显示器的增量刷新
 * ================================================================ */

void menu_invalidate(MenuContext *context)
{
    if (context != NULL)
    {
        context->dirty = true;
    }
}

bool menu_is_dirty(const MenuContext *context)
{
    return context != NULL && context->dirty;
}

void menu_clear_dirty(MenuContext *context)
{
    if (context != NULL)
    {
        context->dirty = false;
    }
}

/* ================================================================
 *  状态查询 — 供渲染模块 (menu_text) 使用
 * ================================================================ */

const MenuPage *menu_current_page(const MenuContext *context)
{
    const MenuFrame *frame = menu_current_frame_const(context);
    return frame == NULL ? NULL : frame->page;
}

const MenuItem *menu_selected_item(const MenuContext *context)
{
    const MenuFrame *frame = menu_current_frame_const(context);

    if (frame == NULL || !menu_page_is_valid(frame->page) ||
        frame->page->item_count == 0U ||
        frame->selected_index >= frame->page->item_count)
    {
        return NULL;
    }

    return &frame->page->items[frame->selected_index];
}

uint16_t menu_selected_index(const MenuContext *context)
{
    const MenuFrame *frame = menu_current_frame_const(context);
    return frame == NULL ? 0U : frame->selected_index;
}

uint16_t menu_top_index(const MenuContext *context)
{
    const MenuFrame *frame = menu_current_frame_const(context);
    return frame == NULL ? 0U : frame->top_index;
}

uint8_t menu_visible_rows(const MenuContext *context)
{
    return context == NULL ? 0U : context->visible_rows;
}

bool menu_is_editing(const MenuContext *context)
{
    return context != NULL && context->editing;
}
