/**
 * @file    menu_core.h
 * @brief   通用多级菜单系统核心模块 — 数据结构与 API 声明
 *
 * 设计思路：
 *   本模块实现一个与硬件/显示后端完全解耦的菜单状态机。
 *   菜单数据（页面 & 菜单项）由用户通过 const 数据结构静态定义，
 *   运行时状态集中存储在 MenuContext 结构体中。
 *
 *   核心概念：
 *   - MenuPage  : 一个菜单页面，包含标题 + 菜单项数组
 *   - MenuItem  : 菜单中的一行，可以是参数、只读值、动作或子菜单入口
 *   - MenuFrame : 导航栈中的一帧，记录当前页面、选中项、滚动偏移
 *   - MenuContext: 保存整个菜单系统的运行时状态（导航栈、编辑状态等）
 *
 *   支持的菜单项类型：
 *   1. PARAM    — 可编辑参数（整数/浮点/布尔），上下键修改值
 *   2. READONLY — 只读参数，仅显示不可编辑
 *   3. ACTION   — 动作项，按确认键触发回调函数
 *   4. SUBMENU  — 子菜单，按确认键进入下一级页面
 *
 *   使用流程：
 *   1. 用 const 定义 MenuPage / MenuItem 数据结构
 *   2. 静态分配一个 MenuContext
 *   3. 调用 menu_init() 初始化
 *   4. 在主循环中将按键映射为 MenuEvent，调用 menu_handle_event()
 *   5. 用 menu_text_get_line() 渲染到 OLED 等字符显示屏
 */

#ifndef MENU_CORE_H
#define MENU_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 可配置常量 ---------- */

/** 菜单导航最大深度（根页面 = 第 0 层），防止无限嵌套 */
#ifndef MENU_MAX_DEPTH
#define MENU_MAX_DEPTH 4U
#endif

/** 编译期计算静态数组元素个数 */
#define MENU_ARRAY_COUNT(array) \
    ((uint16_t)(sizeof(array) / sizeof((array)[0])))

/* ---------- 枚举类型 ---------- */

/** 菜单项的类型，决定 Enter 键的行为 */
typedef enum
{
    MENU_ITEM_PARAM    = 0,  /**< 可编辑参数：Enter 进入编辑模式 */
    MENU_ITEM_READONLY,      /**< 只读参数：仅显示数值，不可编辑 */
    MENU_ITEM_ACTION,        /**< 动作项：Enter 触发回调函数 */
    MENU_ITEM_SUBMENU        /**< 子菜单入口：Enter 进入下级页面 */
} MenuItemType;

/** 参数值的数据类型，决定内存宽度、范围限制与格式化方式 */
typedef enum
{
    MENU_VALUE_FLOAT  = 0,   /**< float (32-bit IEEE 754)，支持小数位控制 */
    MENU_VALUE_INT,          /**< int (平台相关宽度) */
    MENU_VALUE_INT8,         /**< int8_t  范围: -128 ~ 127 */
    MENU_VALUE_INT16,        /**< int16_t 范围: -32768 ~ 32767 */
    MENU_VALUE_INT32,        /**< int32_t 范围: -2^31 ~ 2^31-1 */
    MENU_VALUE_UINT8,        /**< uint8_t  范围: 0 ~ 255 */
    MENU_VALUE_UINT16,       /**< uint16_t 范围: 0 ~ 65535 */
    MENU_VALUE_UINT32,       /**< uint32_t 范围: 0 ~ 2^32-1 */
    MENU_VALUE_BOOL          /**< bool，显示为 ON/OFF，按 Enter 直接翻转 */
} MenuValueType;

/** 用户输入事件，由底层按键驱动映射为此枚举 */
typedef enum
{
    MENU_EVENT_UP    = 0,    /**< 上键：上移光标(浏览模式) / 增加参数值(编辑模式) */
    MENU_EVENT_DOWN,         /**< 下键：下移光标(浏览模式) / 减少参数值(编辑模式) */
    MENU_EVENT_ENTER,        /**< 确认键：进子菜单 / 开始编辑 / 提交编辑 / 触发动作 */
    MENU_EVENT_BACK          /**< 返回键：退出编辑(取消) / 返回上级页面 */
} MenuEvent;

/** menu_init() 的返回值，描述初始化是否成功 */
typedef enum
{
    MENU_STATUS_OK = 0,                 /**< 初始化成功 */
    MENU_STATUS_NULL_ARGUMENT,          /**< context 或 root_page 为 NULL */
    MENU_STATUS_INVALID_PAGE,           /**< root_page 数据不合法 */
    MENU_STATUS_INVALID_VISIBLE_ROWS    /**< visible_rows 为 0 */
} MenuStatus;

/**
 * menu_handle_event() 的返回值，告知调用层发生了什么，
 * 可用于驱动蜂鸣器、记录日志、更新外部状态等
 */
typedef enum
{
    MENU_RESULT_NONE = 0,           /**< 无变化（已到边界或无效操作） */
    MENU_RESULT_MOVED,              /**< 光标已移动 */
    MENU_RESULT_VALUE_CHANGED,      /**< 参数值已修改（编辑模式） */
    MENU_RESULT_EDIT_STARTED,       /**< 进入参数编辑模式 */
    MENU_RESULT_EDIT_COMMITTED,     /**< 编辑提交（Enter 确认） */
    MENU_RESULT_EDIT_CANCELLED,     /**< 编辑取消（Back 撤销） */
    MENU_RESULT_PAGE_ENTERED,       /**< 进入子菜单页面 */
    MENU_RESULT_PAGE_LEFT,          /**< 退出当前页面，返回上级 */
    MENU_RESULT_ACTION_CALLED,      /**< 动作回调已执行 */
    MENU_RESULT_DEPTH_LIMIT,        /**< 已达最大深度，无法进入更深页面 */
    MENU_RESULT_INVALID_ITEM        /**< 当前选中项无效 */
} MenuResult;

/* ---------- 前置声明 ---------- */
typedef struct MenuPage MenuPage;
typedef struct MenuItem MenuItem;

/* ---------- 回调函数类型 ---------- */

/** 动作回调：MENU_ITEM_ACTION 类型菜单项被确认时调用 */
typedef void (*MenuAction)(void);

/** 参数提交回调：编辑完成后按 Enter 提交时调用，可用于持久化等 */
typedef void (*MenuCommitCallback)(const MenuItem *item);

/* ---------- 核心数据结构 ---------- */

/**
 * 参数配置 — 描述一个可编辑参数的全部元信息
 *
 * 使用示例（静态初始化）:
 *   MenuParamConfig pid_cfg = {
 *       .address    = (volatile void *)&g_pid.kp,
 *       .value_type = MENU_VALUE_FLOAT,
 *       .step       = 0.1,
 *       .min_value  = 0.0,
 *       .max_value  = 100.0,
 *       .decimals   = 2
 *   };
 */
typedef struct
{
    volatile void *address;     /**< 指向实际变量的指针（运行时直接读写） */
    MenuValueType value_type;   /**< 变量数据类型 */
    double step;                /**< 单次按键的增减步长 */
    double min_value;           /**< 允许的最小值 */
    double max_value;           /**< 允许的最大值 */
    uint8_t decimals;           /**< 浮点数显示的小数位数（仅对 FLOAT 有效） */
} MenuParamConfig;

/**
 * 菜单项数据联合体 — 根据 MenuItemType 使用不同字段
 *
 *   PARAM / READONLY → .param   (MenuParamConfig)
 *   ACTION           → .action  (MenuAction 函数指针)
 *   SUBMENU          → .page    (指向子 MenuPage)
 */
typedef union
{
    MenuParamConfig param;      /**< 参数配置（PARAM/READONLY 使用） */
    MenuAction action;          /**< 动作回调（ACTION 使用） */
    const MenuPage *page;       /**< 子页面指针（SUBMENU 使用） */
} MenuItemData;

/**
 * 菜单项 — 菜单页面中的一行
 *
 * 静态定义模式（C99 指定初始化器）:
 *   参数行:  { "PID-Kp",  MENU_ITEM_PARAM,  {.param = {&kp, MENU_VALUE_FLOAT, 0.1, 0, 100, 2}} }
 *   动作行:  { "Save",    MENU_ITEM_ACTION, {.action = save_settings} }
 *   子菜单:  { "Settings", MENU_ITEM_SUBMENU, {.page = &page_settings} }
 */
struct MenuItem
{
    const char *name;           /**< 菜单项显示名称（字符串常量） */
    MenuItemType type;          /**< 菜单项类型 */
    MenuItemData data;          /**< 类型相关的数据 */
};

/**
 * 菜单页面 — 一个完整的菜单屏幕
 *
 * 包含标题和菜单项数组。所有数据为 const，运行时不可变。
 */
struct MenuPage
{
    const char *title;          /**< 页面标题，在 OLED 首行居中显示 */
    const MenuItem *items;      /**< 菜单项数组指针 */
    uint16_t item_count;        /**< 菜单项个数 */
};

/**
 * 导航栈帧 — 记录当前所在页面的状态
 *
 * 每一层子菜单对应一帧，保存在 MenuContext.frames[] 中。
 * depth=0 是根页面，depth=N-1 是第 N 层子页面。
 */
typedef struct
{
    const MenuPage *page;       /**< 当前所在页面 */
    uint16_t selected_index;    /**< 当前选中项的下标 */
    uint16_t top_index;         /**< 屏幕显示的第一项的页面下标（滚动用） */
} MenuFrame;

/**
 * 编辑快照 — 进入编辑模式时保存参数原始值
 *
 * 用户按 Back 取消编辑时，从此快照恢复原始值。
 */
typedef union
{
    int64_t signed_value;       /**< 有符号整数的快照 */
    uint64_t unsigned_value;    /**< 无符号整数的快照 */
    float float_value;          /**< 浮点数的快照 */
    bool bool_value;            /**< 布尔值的快照 */
} MenuValueSnapshot;

/**
 * 菜单上下文 — 整个菜单系统的运行时状态
 *
 * 【重要】必须由用户静态分配（全局变量或 static），之后仅通过 menu_* API 操作。
 * 不要在栈上分配此结构体（大小可能较大）。
 *
 * 字段说明:
 *   frames[]        导航栈，frames[0] 为根页面
 *   edit_snapshot   编辑前保存的原始值，用于取消编辑时恢复
 *   commit_callback 编辑提交时的回调（可选），可用于 EEPROM 持久化
 *   depth           当前导航深度（0 = 根页面）
 *   visible_rows    屏幕可显示的行数（不含标题行）
 *   editing         是否处于参数编辑模式
 *   dirty           脏标记，为 true 表示需要刷新显示
 */
typedef struct
{
    MenuFrame frames[MENU_MAX_DEPTH];           /**< 导航栈（静态数组，无堆分配） */
    MenuValueSnapshot edit_snapshot;            /**< 编辑前快照 */
    MenuCommitCallback commit_callback;         /**< 提交回调（可选） */
    uint8_t depth;                              /**< 当前深度 (0 ~ MENU_MAX_DEPTH-1) */
    uint8_t visible_rows;                       /**< 可见行数 */
    bool editing;                               /**< 是否在编辑模式 */
    bool dirty;                                 /**< 脏标记：需要刷新显示 */
} MenuContext;

/* ================================================================
 *  API 函数
 * ================================================================ */

/**
 * 初始化菜单系统
 *
 * @param context       用户静态分配的 MenuContext 指针
 * @param root_page     根菜单页面（顶层页面）
 * @param visible_rows  屏幕可显示行数（不含标题行），如 OLED 128x64 通常为 3~4
 * @return              初始化结果状态码
 */
MenuStatus menu_init(MenuContext *context,
                     const MenuPage *root_page,
                     uint8_t visible_rows);

/**
 * 设置编辑提交回调
 *
 * 当用户在编辑模式中按 Enter 确认修改时调用。
 * 典型用途：将修改后的参数写入 EEPROM/Flash 持久化存储。
 *
 * @param context   菜单上下文
 * @param callback  提交回调函数，传 NULL 取消回调
 */
void menu_set_commit_callback(MenuContext *context,
                              MenuCommitCallback callback);

/**
 * 处理菜单事件 — 菜单系统的核心状态机入口
 *
 * 根据当前 context 状态（浏览/编辑）和事件类型执行相应操作。
 * 调用层（main 循环）将按键映射为 MenuEvent 后调用此函数。
 *
 * @param context  菜单上下文
 * @param event    用户输入事件
 * @return         操作结果，告知调用层发生了什么
 */
MenuResult menu_handle_event(MenuContext *context, MenuEvent event);

/* ---------- 脏标记管理 ---------- */

/** 强制标记为脏（例如参数被外部修改后需刷新显示） */
void menu_invalidate(MenuContext *context);

/** 查询是否需要刷新显示 */
bool menu_is_dirty(const MenuContext *context);

/** 清除脏标记（显示刷新后调用） */
void menu_clear_dirty(MenuContext *context);

/* ---------- 状态查询 ---------- */

/** 获取当前所在页面，用于渲染模块 */
const MenuPage *menu_current_page(const MenuContext *context);

/** 获取当前高亮的菜单项 */
const MenuItem *menu_selected_item(const MenuContext *context);

/** 获取当前选中项的下标 */
uint16_t menu_selected_index(const MenuContext *context);

/** 获取当前屏幕可见区域的第一项下标（滚动偏移） */
uint16_t menu_top_index(const MenuContext *context);

/** 获取可见行数 */
uint8_t menu_visible_rows(const MenuContext *context);

/** 查询是否处于参数编辑模式（用于渲染模块显示不同光标） */
bool menu_is_editing(const MenuContext *context);

#ifdef __cplusplus
}
#endif

#endif
