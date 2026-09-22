/*
 * app_menu.c - 按键菜单实现.
 *
 * 本文件定义:
 *   1. 菜单内容 (页面表 + 菜单项)
 *   2. MenuContext 实例
 *   3. 按键事件 -> MenuEvent 映射
 *   4. OLED 刷新封装 (dirty 驱动)
 *
 * 初始菜单 (验证四种 item 类型):
 *   Main Menu -> IMU Data (READONLY, 实时) / Demo Params (PARAM, 编辑) / Tasks (ACTION)
 *
 */
#include "app_menu.h"
#include "menu_core.h"
#include "menu_text.h"
#include "oled.h"
#include "attitude.h"
#include "led_buzzer.h"
#include "delay.h"
#include "at_params.h"
#include "uart.h"
#include "Encoder.h"
#include "Motor.h"
#include "step_motor.h"
#include "Control.h"
#include "pid.h"       /* PID_Speed_bL/bR/PID_AngleTurn: on_committed 同步增益 */
#include "ball.h"
#include "track.h"
#include "mechanism.h"
#include "task.h"
#include "scheduler.h"   /* g_tms: 计时器时间源 (1ms 计数) */

/* ---- 页面下标 (用枚举名代替数字, 加页面时在此追加) ----
        要添加页面, 先在此枚举里的PAGE_COUNT前面加一项,
        再在 pages[] 里定义该页面.
*/
enum { PAGE_MAIN, PAGE_IMU, PAGE_ENCODER, PAGE_PARAMS, PAGE_TASKS, PAGE_TRACK, PAGE_CAR, PAGE_TPID, PAGE_BALL, PAGE_TIMER, PAGE_COUNT };

/* 前向声明: main_items 里的 SUBMENU 项要引用 pages[], 故先声明.
 * 带 size 的 tentative 定义 (完整类型), 之后给出真实初始化. */
static const MenuPage pages[PAGE_COUNT];

/* Demo Params 变量 (g_demo_kp / g_demo_speed) 现定义在 at_params.c, 见 at_params.h.
 * 菜单 PARAM 项绑定 &g_demo_kp / &g_demo_speed; 编辑提交后由 at_params 持久化. */

/* ---- Tasks 动作函数 ---- */
static void action_buzz(void)
{
    buzzer_on();
    delay_ms(80);
    buzzer_off();
}

static void action_led_toggle(void)
{
    static uint8_t on = 0U;
    if (on) led_off(); else led_on();
    on = (uint8_t)!on;
}

/* ---- 步进位置环测试 (仅双环模式有效: 内环 step_pos_control 才跑) ----
 * 设 step_target_pos 后 P_Run 启动, 内环让步进逼近目标位置并停住.
 * P_Stp 停: 目标=当前位置即停. 单环模式(BALL_LOOP_SINGLE=1)下内环不编译, 此页不响应. */
static void action_step_move(void)
{
    if (step_target_pos == 0) {
        step_target_pos = 1024;   /* 默认 1 圈 (1024 脉冲, 加电后调) */
    }
}

static void action_step_stop(void)
{
    step_target_pos = encoder_step;   /* 目标=当前位置: 误差0, PID 输出0, 停住 */
}

/* ---- 开环 jog (单环 bring-up 工具: 找死点/标定安全弧/复测方向) ----
 * 绕过 ball 内环直驱 step_on_at_freq. 单环编译下内环不跑无抢占; jog 前 ball_loop_stop 停环.
 * jog_step=每次步数(微步), jog_pos=累计位置(微步, 标定死点边界用), J_Zero 摆水平后归零. */
#if BALL_LOOP_SINGLE
#define JOG_HZ   2000
static int32_t jog_step = 200;
static int32_t jog_pos  = 0;

static void action_jog_fwd(void)
{
    ball_loop_stop();                      /* 停控制环防抢占 */
    step_on_at_freq(jog_step, JOG_HZ);     /* 慢速正转 */
    jog_pos += jog_step;
}

static void action_jog_rev(void)
{
    ball_loop_stop();
    step_on_at_freq(-jog_step, JOG_HZ);    /* 反转 */
    jog_pos -= jog_step;
}

static void action_jog_zero(void)
{
    jog_pos = 0;                           /* 摆水平后按, 以此为标定基准 */
}
#endif

/* ---- 球位置外环测试 ----
 * B_Run: 使能外环, ISR 内 ball_pos_control 按编译期模式驱动 (单环 step_on_at_freq /
 *        双环写 step_target_pos 由内环逼近).
 * B_Stp: 关外环 + 冻结 (单环 step_off / 双环回水平零位).
 * 注意: 球环使能时别触发 M-Run(开环)/P_Run(设目标), 三者抢同一电机/目标量, 二选一. */
static void action_ball_run(void)
{
    ball_loop_run();
    app_menu_timer_start();   /* B_Run: 启动球环 + 切 Timer 页计时 */
}

/* ---- 水平零位标定 ----
 * 手动把管子摆到水平, 按 Z_cal 把当前 encoder_step 记为水平零位.
 * 双环模式下 ball_tilt_to_step_pos(0) 对应此位置; 单环模式不需要标定(以启动位置为基准). */
static void action_ball_zero_cal(void)
{
    mechanism_set_level_zero(encoder_step);
}

/* ---- 第三问序列 (0 -> +50 -> -50 保持) ----
 * T3 Run: 启动状态机 (内部使能球环 + 直接阶跃到 50). T3 Stp: 中止 + 复位 + 转回水平.
 * 前置: 已 Z_cal 标水平零位. 控制模式(单环/双环)由 ball.h 的 BALL_LOOP_SINGLE 编译期定. */
static void action_task3_run(void)
{
    task_three_start();
    app_menu_timer_start();   /* T3 Run: 启动球序列 + 切 Timer 页计时 */
}

static void action_task3_stop(void)
{
    task_three_stop();
    app_menu_timer_stop();    /* T3 Stp: 冻结计时 */
}

/* ---- 小车循迹启停 ----
 * C_Run: control_run() 置 start_run=true, ISR 跑转向环+速度环, 车开始循迹.
 * C_Stp: control_stop() 置 start_run=false + PWM 清零, 车停.
 * spd:   basicSpeed 速度环目标 (脉冲数/10ms), 运行中可实时调. */
static void action_car_run(void)
{
    control_run_ramp();   /* C_Run 缓启动, 从 0 斜坡到 basicSpeed, 保护小球不抖动 */
    app_menu_timer_start();   /* C_Run: 启动循迹 + 切 Timer 页计时 */
}

static void action_car_stop(void)
{
    control_stop();
    app_menu_timer_stop();    /* C_Stp: 冻结计时 */
}

/* ---- 第四问: 循迹一圈并定点停车 ----
 * T4_Run: task_four_start() 启动状态机 (起跑线等待 -> 循迹 -> 减速 -> 刹车).
 * T4_Stp: task_four_stop() 中止 + 停车 + 恢复菜单速度.
 * 起跑线事件: 外部检测到时调 set_start_line_event(); 超时 START_LINE_TIMEOUT_MS 自动开跑. */
static void action_task4_run(void)
{
    task_four_start();
    app_menu_timer_start();   /* T4_Run: 启动循迹+定点停车 + 切 Timer 页计时 */
}

static void action_task4_stop(void)
{
    task_four_stop();
    app_menu_timer_stop();    /* T4_Stp: 冻结计时 (task_four_stop 内部也会调, 幂等) */
}

/* ---- 任务计时器 (显示精度 0.01s) ----
 * 时间源 g_tms (SysTick 1ms 计数). 触发方式: 每个任务启动 action (B_Run/C_Run/T3 Run/
 * T4_Run) 调 app_menu_timer_start -> 切到 Timer 页 + 记起始时刻 + 全屏显示计时;
 * 停止 action (B_Stp/C_Stp/T3 Stp/T4_Stp) 调 app_menu_timer_stop 冻结.
 * T4 还在 CAR_BRAKING 状态进入时自动 stop (定点停车完成), task_four_stop 也 stop. */
static volatile bool t_running = false;
static int t_start_tms = 0;
static float g_timer_sec = 0.0f;


/* ---- 各页面菜单项 ---- */

/*
 * READONLY 项格式:
 *   { "显示名", MENU_ITEM_READONLY, {.param = {&变量, 类型, step, min, max, 小数位}} }
 */
static const MenuItem imu_items[] =
{
    /*  名称       类型                 参数: {  地址              类型          step  min  max  小数位 } */
    {"Roll",  MENU_ITEM_READONLY, {.param = {&attitude.roll,  MENU_VALUE_FLOAT,  0.0,  0.0, 0.0, 2}}},
    {"Pitch", MENU_ITEM_READONLY, {.param = {&attitude.pitch, MENU_VALUE_FLOAT,  0.0,  0.0, 0.0, 2}}},
    {"Yaw",   MENU_ITEM_READONLY, {.param = {&attitude.yaw,   MENU_VALUE_FLOAT,  0.0,  0.0, 0.0, 2}}},
};

/*
 * 编码器实时值 (READONLY).
 * spd = 每 10ms 脉冲数 (控制环运行时更新; 停止时停在上次值)
 * cnt = 原始累计 (控制环停止时不被清零, 手动转轮可看方向; 运行时每 10ms 清零会跳)
 */
static const MenuItem encoder_items[] =
{
    /*  名称       类型                 参数: {  地址                  类型             step  min  max  小数位 } */
    {"BL spd", MENU_ITEM_READONLY, {.param = {&encoder_bl_speed, MENU_VALUE_INT16, 0.0, 0.0, 0.0, 0}}},
    {"BR spd", MENU_ITEM_READONLY, {.param = {&encoder_br_speed, MENU_VALUE_INT16, 0.0, 0.0, 0.0, 0}}},
    {"BL cnt", MENU_ITEM_READONLY, {.param = {&encoder_bl,       MENU_VALUE_INT16, 0.0, 0.0, 0.0, 0}}},
    {"BR cnt", MENU_ITEM_READONLY, {.param = {&encoder_br,       MENU_VALUE_INT16, 0.0, 0.0, 0.0, 0}}},
};

/*
 * PARAM 项格式:
 *   { "显示名", MENU_ITEM_PARAM, {.param = {&变量, 类型, step, min, max, 小数位}} }
 *   提示: 添加新参数时，先定义变量 (如在 at_params.c 中)，再在此添加菜单项。
 */
static const MenuItem param_items[] =
{
    /*  名称        类型              参数: {  地址            类型          step  min   max  小数位 } */
    {"Kp",    MENU_ITEM_PARAM, {.param = {&g_demo_kp,    MENU_VALUE_FLOAT,  0.01, 0.0, 20.0,   2}}},
    {"Speed", MENU_ITEM_PARAM, {.param = {&g_demo_speed, MENU_VALUE_UINT16, 10.0, 0.0, 2000.0, 0}}},
    {"t_para", MENU_ITEM_READONLY, {.param = {&test_uart_param, MENU_VALUE_UINT8, 0.0, 0.0,0.0, 0}}},
    {"Ball_Pos", MENU_ITEM_READONLY, {.param = {&ball_pos, MENU_VALUE_INT16, 0.0, 0.0,0.0, 0}}},
};

/*
 * ACTION 项格式:
 *   { "显示名", MENU_ITEM_ACTION, {.action = 函数名} }
 *   注意: 动作函数在按键处理上下文中同步执行，不要放长时间阻塞的操作。
 *         当前 buzz 有 80ms 阻塞 (delay_ms)，可接受但不建议超过 200ms。
 */
static const MenuItem task_items[] =
{
    /* 步进位置环: p_tgt 可编辑目标位置(编码器脉冲), P_Run/P_Stp 启停, step_c 显示当前位置 */
    {"step_c", MENU_ITEM_READONLY, {.param = {&encoder_step,       MENU_VALUE_INT32, 0.0, 0.0, 0.0, 0}}},
    {"p_tgt", MENU_ITEM_PARAM, {.param = {&step_target_pos, MENU_VALUE_INT32, 5.0, -50000, 50000, 0}}},
    {"P_Run", MENU_ITEM_ACTION, {.action = action_step_move}},
    {"P_Stp", MENU_ITEM_ACTION, {.action = action_step_stop}},
#if BALL_LOOP_SINGLE
    /* 开环 jog (单环 bring-up): jog_n 步数, jog_p 累计位置, Jog+/- 转, J_Zero 归零 */
    {"jog_n",  MENU_ITEM_PARAM,    {.param = {&jog_step,   MENU_VALUE_INT32, 50.0, 10.0, 5000.0, 0}}},
    {"jog_p",  MENU_ITEM_READONLY, {.param = {&jog_pos,    MENU_VALUE_INT32, 0.0, 0.0, 0.0, 0}}},
    {"Jog+",   MENU_ITEM_ACTION,   {.action = action_jog_fwd}},
    {"Jog-",   MENU_ITEM_ACTION,   {.action = action_jog_rev}},
    {"J_Zero", MENU_ITEM_ACTION,   {.action = action_jog_zero}},
#endif
};

/*
 * 循迹 PID 调参页:
 *   s_kp PARAM 速度环 P (g_speed_kp, bL/bR 共用, AT24C02 持久化)
 *   s_ki PARAM 速度环 I (g_speed_ki, bL/bR 共用, AT24C02 持久化)
 *   t_kp PARAM 转向环 P (g_turn_kp, AT24C02 持久化)
 *   t_kd PARAM 转向环 D (g_turn_kd, AT24C02 持久化)
 * 提交后 on_committed 同步到 PID 结构体, 运行中实时生效.
 */
static const MenuItem tpid_items[] =
{
    {"s_kp", MENU_ITEM_PARAM, {.param = {&g_speed_kp, MENU_VALUE_FLOAT, 0.5, 0.0, 200.0, 2}}},
    {"s_ki", MENU_ITEM_PARAM, {.param = {&g_speed_ki, MENU_VALUE_FLOAT, 0.1, 0.0, 50.0,  2}}},
    {"t_kp", MENU_ITEM_PARAM, {.param = {&g_turn_kp,  MENU_VALUE_FLOAT, 0.1, 0.0, 20.0,  2}}},
    {"t_kd", MENU_ITEM_PARAM, {.param = {&g_turn_kd,  MENU_VALUE_FLOAT, 0.1, 0.0, 20.0,  2}}},
};

/*
 * 小车循迹页:
 *   spd    PARAM    速度环目标 (脉冲数/10ms), 运行中可实时调 (T4 运行时跟随 target_speed)
 *   err    READONLY 循迹误差 (-10..10, 负偏左正偏右)
 *   turn   READONLY 转向环输出 (叠加到左右轮速度差)
 *   C_Run  ACTION   普通循迹启停: start_run=true, ISR 跑转向+速度环
 *   C_Stp  ACTION   普通循迹停止: start_run=false + PWM 清零
 *   T4_Run ACTION   第四问: 循迹一圈定点停车 (起跑线等待->循迹->减速->刹车)
 *   T4_Stp ACTION   第四问中止: 停车 + 恢复菜单速度
 *   r_sec  PARAM    减速触发时间(秒), 从开始跑计时到此值后减速 (AT24C02 持久化)
 *   a_deg  PARAM    减速触发角度阈值(°), 从开始跑累计 yaw 达此值后减速 (AT24C02 持久化)
 *   y_min  PARAM    停车 yaw 下限(°), 减速段起跑线+yaw 范围触发刹车 (AT24C02 持久化)
 *   y_max  PARAM    停车 yaw 上限(°), 减速段起跑线+yaw 范围触发刹车 (AT24C02 持久化)
 *   a_sum  READONLY 角度累计 (T4 调试用, 不再参与判定)
 */
static const MenuItem car_items[] =
{
    {"spd",    MENU_ITEM_PARAM,    {.param = {&basicSpeed,  MENU_VALUE_INT16, 1.0, 0.0, 50.0, 0}}},
    {"err",    MENU_ITEM_READONLY, {.param = {&track_error, MENU_VALUE_INT16, 0.0, 0.0, 0.0,  0}}},
    {"turn",   MENU_ITEM_READONLY, {.param = {&track_turn,  MENU_VALUE_INT16, 0.0, 0.0, 0.0,  0}}},
    {"C_Run",  MENU_ITEM_ACTION,   {.action = action_car_run}},
    {"C_Stp",  MENU_ITEM_ACTION,   {.action = action_car_stop}},
    {"T4_Run", MENU_ITEM_ACTION,   {.action = action_task4_run}},
    {"T4_Stp", MENU_ITEM_ACTION,   {.action = action_task4_stop}},
    {"r_sec",  MENU_ITEM_PARAM,    {.param = {&g_run_sec,        MENU_VALUE_FLOAT, 0.5, 0.0, 120.0, 1}}},
    {"a_deg",  MENU_ITEM_PARAM,    {.param = {&g_angle_stop_deg, MENU_VALUE_FLOAT, 5.0, 0.0, 720.0, 1}}},
    {"y_min",  MENU_ITEM_PARAM,    {.param = {&g_yaw_stop_min,   MENU_VALUE_FLOAT, 1.0, 0.0, 180.0, 1}}},
    {"y_max",  MENU_ITEM_PARAM,    {.param = {&g_yaw_stop_max,   MENU_VALUE_FLOAT, 1.0, 0.0, 180.0, 1}}},
    {"a_sum",  MENU_ITEM_READONLY, {.param = {&angle_sumer, MENU_VALUE_FLOAT, 0.0, 0.0, 0.0, 1}}},
};

/*
 * 球位置外环页 (球杆系统):
 *   b_tgt PARAM    目标球位置 (ball_pos 标尺, ±150, 中心 0, 默认 0)
 *   b_pos READONLY 当前球位置 (摄像头 0x21)
 *   b_out READONLY 外环 PID 输出 (单环=步位置偏置 / 双环=目标倾角deg)
 *   b_P  PARAM     单环 P 增益 (g_ball_kp, AT24C02 持久化)
 *   b_I  PARAM     单环 I 增益 (g_ball_ki, AT24C02 持久化)
 *   b_D  PARAM     单环 D 增益 (g_ball_kd=帧-PD 速度增益, AT24C02 持久化)
 *   B_Run ACTION   使能环 (ISR 内 ball_pos_control)
 *   Z_cal ACTION   水平零位标定 (双环模式用, 单环不需要)
 *   t3_p  PARAM    T3 正目标距离(默认+50), set_ball_position 用, AT24C02 持久化
 *   t3_w  PARAM    T3 负目标距离(默认-45), wait_ball_pos 用, AT24C02 持久化
 *   T3 Run/Stp    第三问序列启停
 *   模式由 ball.h 的 BALL_LOOP_SINGLE 编译期定, 无运行时切换.
 */
static const MenuItem ball_items[] =
{
    {"b_tgt", MENU_ITEM_PARAM,    {.param = {&ball_target_pos, MENU_VALUE_INT16, 1.0, -150.0, 150.0, 0}}},
    {"b_pos", MENU_ITEM_READONLY, {.param = {&ball_pos,         MENU_VALUE_INT16, 0.0, 0.0, 0.0,   0}}},
    {"b_out", MENU_ITEM_READONLY, {.param = {&g_out,         MENU_VALUE_FLOAT, 0.0, 0.0, 0.0,   2}}},
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    {"b_vel", MENU_ITEM_READONLY, {.param = {&g_ball_velocity_filtered, MENU_VALUE_FLOAT,  0.0, 0.0, 0.0,   1}}},
    {"b_P",   MENU_ITEM_PARAM,    {.param = {&g_ball_kp,                MENU_VALUE_FLOAT,  0.1, 0.0, 100.0, 2}}},
    {"b_I",   MENU_ITEM_PARAM,    {.param = {&g_ball_ki,                MENU_VALUE_FLOAT,  0.01, 0.0, 10.0,  2}}},
    {"b_D",   MENU_ITEM_PARAM,    {.param = {&g_ball_kd,                MENU_VALUE_FLOAT,  0.1, 0.0, 50.0,  2}}},
    {"b_dt",  MENU_ITEM_READONLY, {.param = {&g_ball_dt_ms,             MENU_VALUE_UINT32, 0.0, 0.0, 0.0,   0}}},
#endif
    {"B_Run", MENU_ITEM_ACTION,   {.action = action_ball_run}},
    {"Z_cal", MENU_ITEM_ACTION,   {.action = action_ball_zero_cal}},
    {"t3_p",  MENU_ITEM_PARAM,    {.param = {&g_t3_pos,  MENU_VALUE_FLOAT, 1.0, 0.0,   150.0, 0}}},
    {"t3_w",  MENU_ITEM_PARAM,    {.param = {&g_t3_wait, MENU_VALUE_FLOAT, 1.0, -150.0, 0.0,  0}}},
    {"T3 Run",MENU_ITEM_ACTION,   {.action = action_task3_run}},
    {"T3 Stp",MENU_ITEM_ACTION,   {.action = action_task3_stop}},
};

/*
 * 计时器页:
 *   time   READONLY 当前计时(秒), 精确到 0.01s (2 位小数)
 *   运行中由 app_menu_refresh 全屏渲染 (居中 + 单位 S);
 *   触发由 task.c 发车事件自动调用 app_menu_timer_start, 无按键项.
 */
static const MenuItem timer_items[] =
{
    {"time",  MENU_ITEM_READONLY, {.param = {&g_timer_sec, MENU_VALUE_FLOAT, 0.0, 0.0, 0.0, 2}}},
};

/*
 * SUBMENU 项格式:
 *   { "显示名", MENU_ITEM_SUBMENU, {.page = &pages[页面下标]} }
 *   添加新子菜单的步骤:
 *     1. 在顶部 enum 中 PAGE_COUNT 前加一项 (如 PAGE_BLUETOOTH)
 *     2. 定义该页面的 MenuItem 数组 (如 bt_items[])
 *     3. 在 pages[] 表中添加该页面的定义
 *     4. 在 main_items[] 中添加 SUBMENU 入口项
 */
static const MenuItem main_items[] =
{
    /*    名称            类型                  目标页面 */
    {"IMU Data",    MENU_ITEM_SUBMENU, {.page = &pages[PAGE_IMU]}},
    {"Encoder",     MENU_ITEM_SUBMENU, {.page = &pages[PAGE_ENCODER]}},
    {"Demo Params", MENU_ITEM_SUBMENU, {.page = &pages[PAGE_PARAMS]}},
    {"Tasks",       MENU_ITEM_SUBMENU, {.page = &pages[PAGE_TASKS]}},
    {"Track",       MENU_ITEM_SUBMENU, {.page = &pages[PAGE_TRACK]}},
    {"Car",         MENU_ITEM_SUBMENU, {.page = &pages[PAGE_CAR]}},
    {"TrackPID",    MENU_ITEM_SUBMENU, {.page = &pages[PAGE_TPID]}},
    {"Ball",        MENU_ITEM_SUBMENU, {.page = &pages[PAGE_BALL]}},
    {"Timer",       MENU_ITEM_SUBMENU, {.page = &pages[PAGE_TIMER]}},
};

/* ---- 页面表 (真实定义) ---- */
static const MenuPage pages[PAGE_COUNT] =
{
    [PAGE_MAIN]    = {"Main Menu",   main_items,    MENU_ARRAY_COUNT(main_items)},
    [PAGE_IMU]     = {"IMU Data",    imu_items,     MENU_ARRAY_COUNT(imu_items)},
    [PAGE_ENCODER] = {"Encoder",     encoder_items, MENU_ARRAY_COUNT(encoder_items)},
    [PAGE_PARAMS]  = {"Demo Params", param_items,   MENU_ARRAY_COUNT(param_items)},
    [PAGE_TASKS]   = {"Tasks",       task_items,    MENU_ARRAY_COUNT(task_items)},
    [PAGE_TRACK]   = {"Track",       NULL,          0},   /* 自定义渲染, 无菜单项 */
    [PAGE_CAR]     = {"Car",         car_items,     MENU_ARRAY_COUNT(car_items)},
    [PAGE_TPID]    = {"TrackPID",    tpid_items,    MENU_ARRAY_COUNT(tpid_items)},
    [PAGE_BALL]    = {"Ball",        ball_items,    MENU_ARRAY_COUNT(ball_items)},
    [PAGE_TIMER]   = {"Timer",       timer_items,   MENU_ARRAY_COUNT(timer_items)},
};

/* ---- 菜单上下文 (静态分配) ---- */
static MenuContext g_menu;

/* ============================ API ============================ */

/* 菜单提交回调: 用户按 ENTER 确认参数编辑后触发.
 * 1. at_params_save_var: 把该变量存进 EEPROM (掉电保存).
 * 2. 循迹 PID 增益同步到 PID 结构体: PidInit 只在 control_init 调一次,
 *    菜单改 g_speed_kp 等后须手动写 PID_Speed_bL.kp 才能运行中实时生效.
 *    速度环 bL/bR 共用一组参数, 两个都要写. */
static void on_committed(const MenuItem *item)
{
    at_params_save_var(item->data.param.address);

    volatile void *addr = item->data.param.address;
    if      (addr == &g_speed_kp) { PID_Speed_bL.kp = g_speed_kp; PID_Speed_bR.kp = g_speed_kp; }
    else if (addr == &g_speed_ki) { PID_Speed_bL.ki = g_speed_ki; PID_Speed_bR.ki = g_speed_ki; }
    else if (addr == &g_turn_kp)  { PID_AngleTurn.kp = g_turn_kp; }
    else if (addr == &g_turn_kd)  { PID_AngleTurn.kd = g_turn_kd; }
}

void app_menu_init(void)
{
    /* visible_rows=3: 标题 + 3 项 = 4 行, 正好填满 128x64 / 8x16. */
    (void)menu_init(&g_menu, &pages[PAGE_MAIN], 3U);
    menu_set_commit_callback(&g_menu, on_committed);
}

void app_menu_invalidate(void)
{
    menu_invalidate(&g_menu);
}

void app_menu_timer_start(void)
{
    t_start_tms = g_tms;
    t_running = true;
    g_timer_sec = 0.0f;
    /* 切到 Timer 页 (压栈方式, BACK 可退回发车前的页面). 重复发车不重复压栈. */
    if (menu_current_page(&g_menu) != &pages[PAGE_TIMER]) {
        if ((uint8_t)(g_menu.depth + 1U) < MENU_MAX_DEPTH) {
            g_menu.depth++;
            g_menu.frames[g_menu.depth].page = &pages[PAGE_TIMER];
            g_menu.frames[g_menu.depth].selected_index = 0U;
            g_menu.frames[g_menu.depth].top_index = 0U;
        }
    }
    menu_invalidate(&g_menu);
}

void app_menu_timer_stop(void)
{
    if (!t_running) return;   /* 已停, 幂等不重复刷新 (BRAKING 每帧调) */
    g_timer_sec = (float)(g_tms - t_start_tms) * 0.001f;   /* 冻结最终值 */
    t_running = false;
    menu_invalidate(&g_menu);
}

/* ---- 循迹调试页 (自定义渲染: 8 路电平 + 误差同屏, 实时) ----
 * track 页 item_count=0, 标准菜单渲染只画标题; 此处接管整屏画 8 路电平.
 * track_error_get() 内部调 track_update(), 既刷新 track_sensors[] 又返回误差,
 * 单次读取无竞态. 调试时 start_run=false, 控制环不跑, 此处是唯一读源.
 * 布局 (8x16 字号, 16 列 × 4 行):
 *   row0: "Track" 居中
 *   row1: "1 2 3 4 5 6 7 8"   传感器编号
 *   row2: "x x x x x x x x"   实时电平 1=黑线 0=白线
 *   row3: "err: -3 n:3"        误差(-10..10) + 黑线计数 */
static void track_page_render(void)
{
    int16_t err = track_error_get();
    uint8_t cnt = (uint8_t)(track_sensors[0] + track_sensors[1] + track_sensors[2] + track_sensors[3]
                          + track_sensors[4] + track_sensors[5] + track_sensors[6] + track_sensors[7]);

    OLED_Clear();
    OLED_PrintfRC(0, 5, "Track");               /* 标题居中 (5 字符 @col5) */
    OLED_PrintfRC(1, 0, "1 2 3 4 5 6 7 8");     /* 传感器编号 */
    OLED_PrintfRC(2, 0, "%d %d %d %d %d %d %d %d",  /* 实时电平: 1=黑线 0=白线 */
        track_sensors[0], track_sensors[1], track_sensors[2], track_sensors[3],
        track_sensors[4], track_sensors[5], track_sensors[6], track_sensors[7]);
    OLED_PrintfRC(3, 0, "err:%4d n:%u", err, cnt);  /* 误差 + 黑线计数 */
    OLED_Update();
    menu_clear_dirty(&g_menu);
}

void app_menu_refresh(void)
{
    /* 计时器运行中: 实时更新显示值 + 30ms 自动置脏 (0.01s 显示步进流畅).
     * scheduler 默认 100ms 置脏太慢, 看不到 0.01 变化; 这里 timer 运行时自行提频. */
    static int last_timer_draw = 0;
    if (t_running) {
        g_timer_sec = (float)(g_tms - t_start_tms) * 0.001f;
        if (g_tms - last_timer_draw >= 30) {
            last_timer_draw = g_tms;
            menu_invalidate(&g_menu);
        }
    }

    if (!menu_is_dirty(&g_menu))
    {
        return;   /* 无变化则跳过, 避免无谓的 OLED_Update (~28ms) */
    }

    /* 循迹调试页: 自定义渲染 (8 路电平同屏), 不走标准文本行 */
    if (menu_current_page(&g_menu) == &pages[PAGE_TRACK])
    {
        track_page_render();
        return;
    }

    /* 计时器页 + 运行中: 全屏显示时间 (居中, 带单位 S) */
    if (menu_current_page(&g_menu) == &pages[PAGE_TIMER] && t_running)
    {
        OLED_Clear();
        OLED_PrintfRC(1, 2, "Timer");
        OLED_PrintfRC(2, 3, "%.2f S", g_timer_sec);
        OLED_Update();
        menu_clear_dirty(&g_menu);
        return;
    }

    OLED_Clear();
    char line[17];   /* 16 字符 + '\0' */
    uint8_t rows = menu_visible_rows(&g_menu);
    for (uint8_t row = 0U; row <= rows; row++)
    {
        if (menu_text_get_line(&g_menu, row, line, sizeof(line), 16U))
        {
            OLED_ShowString(0, (uint8_t)(row * 16U), line, OLED_FONT_16);
        }
    }
    OLED_Update();
    menu_clear_dirty(&g_menu);
}

void app_menu_on_key(uint8_t id, Key_Event event)
{
    MenuEvent me;

    switch (id)
    {
    case 0:  /* K0 = DOWN: 短按一步, 长按连滚 */
        if (event != KEY_EVENT_SHORT_PRESS && event != KEY_EVENT_LONG_PRESS &&
            event != KEY_EVENT_LONG_REPEAT)
        {
            return;
        }
        me = MENU_EVENT_DOWN;
        break;

    case 1:  /* K1 = UP: 短按一步, 长按连滚 */
        if (event != KEY_EVENT_SHORT_PRESS && event != KEY_EVENT_LONG_PRESS &&
            event != KEY_EVENT_LONG_REPEAT)
        {
            return;
        }
        me = MENU_EVENT_UP;
        break;

    case 2:  /* K2 = ENTER: 仅短按, 防误触重复 */
        if (event != KEY_EVENT_SHORT_PRESS)
        {
            return;
        }
        me = MENU_EVENT_ENTER;
        break;

    case 3:  /* K3 = BACK: 仅短按 */
        if (event != KEY_EVENT_SHORT_PRESS)
        {
            return;
        }
        me = MENU_EVENT_BACK;
        break;

    default:
        return;
    }

    menu_handle_event(&g_menu, me);
}
