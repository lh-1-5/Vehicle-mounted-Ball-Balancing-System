# MenuCore 使用说明

MenuCore 是一套静态表格式嵌入式菜单，不依赖具体单片机、GPIO、按键、
OLED 或存储芯片。正常添加菜单项时，只需要复制一条 `{ ... }` 并修改其中
的参数，不需要修改 `menu_core.c`。

## 文件说明

- `menu_core.h/.c`：菜单导航、参数编辑、页面返回、限幅和任务调用。
- `menu_text.h/.c`：把当前菜单转换成固定宽度的显示字符串。

## 一、可编辑参数

参数菜单项的格式固定为：

```c
{
    "显示名称",
    MENU_ITEM_PARAM,
    {.param = {
        &变量,
        变量类型,
        修改步长,
        最小值,
        最大值,
        小数显示位数
    }}
}
```

例如：

```c
static float kp = 1.0f;
static uint16_t target_speed = 500U;

static const MenuItem pid_items[] =
{
    {"Kp", MENU_ITEM_PARAM,
     {.param = {&kp, MENU_VALUE_FLOAT, 0.01, 0.0, 20.0, 2}}},

    {"Speed", MENU_ITEM_PARAM,
     {.param = {&target_speed, MENU_VALUE_UINT16, 10.0, 0.0, 2000.0, 0}}},
};
```

以上两行分别表示：

```text
Kp：float 类型，每次修改 0.01，范围 0~20，显示 2 位小数
Speed：uint16_t 类型，每次修改 10，范围 0~2000，不显示小数
```

整数参数的步长应填写正整数。菜单会先在更宽的临时类型中计算，再进行
限幅，最后写回实际变量，因此无符号数在零处继续减小不会回绕到最大值。

支持的数据类型：

| 菜单类型 | 实际变量类型 |
| --- | --- |
| `MENU_VALUE_FLOAT` | `float` |
| `MENU_VALUE_INT` | `int` |
| `MENU_VALUE_INT8` | `int8_t` |
| `MENU_VALUE_INT16` | `int16_t` |
| `MENU_VALUE_INT32` | `int32_t` |
| `MENU_VALUE_UINT8` | `uint8_t` |
| `MENU_VALUE_UINT16` | `uint16_t` |
| `MENU_VALUE_UINT32` | `uint32_t` |
| `MENU_VALUE_BOOL` | `bool` |

## 二、只读数据

IMU、编码器、ADC 等只需要显示而不能修改的数据，使用
`MENU_ITEM_READONLY`。步长和范围不会使用，统一填写零。

```c
static float roll;
static float pitch;

static const MenuItem imu_items[] =
{
    {"Roll", MENU_ITEM_READONLY,
     {.param = {&roll, MENU_VALUE_FLOAT, 0.0, 0.0, 0.0, 2}}},

    {"Pitch", MENU_ITEM_READONLY,
     {.param = {&pitch, MENU_VALUE_FLOAT, 0.0, 0.0, 0.0, 2}}},
};
```

生成显示行时，菜单会重新读取变量，因此可以显示实时变化的数据。

## 三、任务菜单项

任务函数使用原来菜单的简单形式：

```c
static void imu_calibrate(void)
{
    /* 执行 IMU 校准 */
}
```

任务菜单项只需要填写三项：

```c
static const MenuItem task_items[] =
{
    {"IMU Cal", MENU_ITEM_ACTION, {.action = imu_calibrate}},
};
```

选中后按 ENTER，函数只调用一次。耗时很长的工作不建议直接阻塞在任务
函数中，可以在函数里设置任务标志，由主循环执行实际工作。

## 四、子菜单和页面表

页面可以像旧菜单一样集中放在一个数组中：

```c
const MenuPage pages[];

static const MenuItem main_items[] =
{
    {"PID",   MENU_ITEM_SUBMENU, {.page = &pages[1]}},
    {"IMU",   MENU_ITEM_SUBMENU, {.page = &pages[2]}},
    {"Tasks", MENU_ITEM_SUBMENU, {.page = &pages[3]}},
};

const MenuPage pages[] =
{
    {"Main Menu", main_items, MENU_ARRAY_COUNT(main_items)},
    {"PID",       pid_items,  MENU_ARRAY_COUNT(pid_items)},
    {"IMU Data",  imu_items,  MENU_ARRAY_COUNT(imu_items)},
    {"Tasks",     task_items, MENU_ARRAY_COUNT(task_items)},
};
```

子菜单项的三个参数分别是：

```text
显示名称、MENU_ITEM_SUBMENU、目标页面地址
```

`MENU_ARRAY_COUNT()` 只负责自动计算数组中有多少项，不会隐藏菜单配置。

如果希望页面数组只在当前 `.c` 文件中可见，可以提前写出页面数量：

```c
enum { PAGE_MAIN, PAGE_PID, PAGE_IMU, PAGE_TASKS, PAGE_COUNT };
static const MenuPage pages[PAGE_COUNT];
```

然后使用 `pages[PAGE_PID]` 等名称代替数字下标。

## 五、初始化和按键输入

```c
static MenuContext menu;

void app_menu_init(void)
{
    (void)menu_init(&menu, &pages[0], 3U);
}
```

这里的 `3U` 表示标题下面显示三行菜单项。

按键模块只需要向菜单发送四种事件：

```c
menu_handle_event(&menu, MENU_EVENT_UP);
menu_handle_event(&menu, MENU_EVENT_DOWN);
menu_handle_event(&menu, MENU_EVENT_ENTER);
menu_handle_event(&menu, MENU_EVENT_BACK);
```

浏览状态：

- UP/DOWN：上下移动，首尾循环。
- ENTER：编辑参数、执行任务或进入子菜单。
- BACK：返回上一级，并恢复父页面原来的光标位置。

编辑状态：

- UP/DOWN：修改参数并限制在配置范围内。
- ENTER：确认修改。
- BACK：取消修改，恢复进入编辑前的值。

## 六、显示文字

第 0 行是居中的页面标题，后面的行是菜单项目。128×64 OLED 使用 8×16
字体时，一般为 16 列、4 行：

```c
char line[17];

for (uint8_t row = 0U; row < 4U; ++row)
{
    menu_text_get_line(&menu, row, line, sizeof(line), 16U);
    OLED_ShowString(0, row * 16U, line, OLED_FONT_16);
}
OLED_Update();
```

当前选中项以 `>` 开头，正在编辑的参数以 `*` 开头。字符串始终按照传入
的宽度安全截断，不使用不受限制的 `sprintf + strcat`。

MenuCore 本身不包含 OLED 头文件。移植到具体工程后，可以把上面的显示
循环包装成一个简单的 `app_menu_refresh()`。

## 七、参数保存

MenuCore 不直接依赖 AT24C02 或片内 Flash。如果确认参数后需要保存，注册
一个统一回调：

```c
static void parameter_committed(const MenuItem *item)
{
    /* item->data.param.address 是刚刚确认的变量地址 */
    /* 建议在这里设置待保存标志，由主循环稍后执行实际写入 */
}

menu_set_commit_callback(&menu, parameter_committed);
```

不要在按键中断中执行耗时的 EEPROM 或 Flash 写入。

## 最常用的四种写法

```c
/* 可编辑参数 */
{"Kp", MENU_ITEM_PARAM,
 {.param = {&kp, MENU_VALUE_FLOAT, 0.01, 0.0, 20.0, 2}}}

/* 只读数据 */
{"Roll", MENU_ITEM_READONLY,
 {.param = {&roll, MENU_VALUE_FLOAT, 0.0, 0.0, 0.0, 2}}}

/* 执行任务 */
{"Calibrate", MENU_ITEM_ACTION, {.action = imu_calibrate}}

/* 进入子菜单 */
{"PID", MENU_ITEM_SUBMENU, {.page = &pages[PAGE_PID]}}
```

队友通常只需要复制以上其中一行，然后修改名称、地址和参数即可。
