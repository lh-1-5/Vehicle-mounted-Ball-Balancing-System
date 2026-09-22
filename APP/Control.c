/*
 * control.c - 小车控制核心 (转向环 + 速度环)
 *
 * 移植自队友工程, 适配 empty:
 *   - yaw 用 attitude.yaw (自写 Mahony, 见 attitude.h / imu_select.h)
 *   - 精简到后轮 bL/bR (前轮代码已删)
 *   - start_run 启用: ISR 顶部 if(!start_run) return, 上电不跑, 按键触发
 *   - 控制定时器 TIMER_0 (TIMA0, 10ms), ISR = TIMER_0_INST_IRQHandler
 *   - 编码器 ISR (GROUP1_IRQHandler) 在 Encoder.c 实现
 *
 * 滚球平衡控制(ball-and-beam)已拆出到 APP/ball.c, 独占 TIMER_BALL (TIMG6, 10ms),
 * 与本文件的小车控制彻底解耦. 球环接口见 ball.h.
 *
 * 对外接口: control_init / control_toggle_run (见 Control.h).
 */
#include "ti_msp_dl_config.h"
#include "Control.h"
#include "attitude.h"
#include "Encoder.h"
#include "Motor.h"
#include "pid.h"
#include "track.h"
#include "led_buzzer.h"
#include <math.h>        /* fabsf (angle_sum) */

int16_t basicSpeed = 5;       /* 速度环目标 (编码器脉冲数/10ms), 实测调 */

/* 循迹 PID 增益 (菜单 PARAM, AT24C02 持久化).
 * 编译初值=原 PidInit 硬编码值; 上电 at_params_init 从 EEPROM 覆盖; 菜单编辑提交时回写 + 同步 PID 结构体.
 * 速度环 bL/bR 共用同一组参数; 转向环单实例. */
float g_speed_kp = 30.0f;
float g_speed_ki = 1.0f;
float g_turn_kp   = 1.6f;
float g_turn_kd   = 1.0f;

int16_t track_error = 0;
int16_t track_angle = 0;
int16_t track_turn = 0;
int16_t gyro_z_speed = 0;

float    yaw_error = 0.0f;
bool     start_run = false;
uint32_t isr_cnt = 0;

/* ---- 第四问小车状态机变量 (Control.c 定义, task.c 通过 Control.h extern 用) ----
 * 起跑线检测在 turn_control (ISR 30ms) 里做: black_cnt>=3 且 中间 sensor3/4 黑 两端白 -> pl_event=true.
 * angle_sum 累计 yaw 跨越 360° 边界, full_rotations 记整圈数. */
car_state_t car_state           = CAR_IDLE;
volatile bool pl_debounce       = false;
volatile bool pl_event          = false;
int32_t      enc_at_stop_line   = 0;
uint32_t     run_start_tms      = 0;
uint32_t     run_end_tms        = 0;
uint32_t     total_run_ms       = 0;
bool         oled_shown         = false;
int          full_rotations     = 0;
uint8_t      black_cnt          = 0;

void state_reset(void)
{
    car_state        = CAR_IDLE;
    pl_debounce      = 0;
    pl_event         = false;
    enc_at_stop_line = 0;
    run_start_tms    = 0;
    run_end_tms      = 0;
    total_run_ms     = 0;
    oled_shown       = false;
}

float angle_sum(float yaw_a)
{
    static float angle_cur = 0;
    static float angle_pre = 0;
    static float angle_det = 0;

    angle_pre = angle_cur;

    if (yaw_a < 0)
        angle_cur = 360.0f + yaw_a;
    else
        angle_cur = yaw_a;

    angle_det = angle_cur - angle_pre;

    if (fabsf(angle_det) > 0.8f * 360.0f)
        full_rotations += (angle_det > 0) ? -1 : 1;

    return full_rotations * 360.0f + angle_cur;
}


void set_speed(int16_t target_speed)
{
    basicSpeed = target_speed;
}

/* ---- 缓启动 (仅 C_Run 普通循迹用, T4 第四问不走缓启) ----
 * ramp_enable=true 时 speed_control 走斜坡 (从 0 到 basicSpeed), 避免 C_Run 阶跃冲击小球.
 * T4 调 control_run (ramp_enable 保持 false), 立即达到 basicSpeed, 不缓启.
 * RAMP_STEP=每10ms增量(浮点). 速度环 100Hz(每10ms一拍), RAMP_STEP=0.01 → 1s 增加 1,
 * basicSpeed=6 时 6s 到 6, 柔和缓启. */
#define RAMP_STEP  0.01f
static float ramp_speed = 0.0f;
static bool   ramp_enable = false;

/* ---- 切换运行状态: start_run 翻转 ----*/
void control_toggle_run(void)
{
    start_run = !start_run;
}

void control_run(void)
{
    /* T4 用: 不缓启, 立即达到 basicSpeed (ramp_enable 保持 false, speed_control 直接跟随) */
    start_run = 1;
}

void control_run_ramp(void)
{
    /* C_Run 用: 从 0 缓启到 basicSpeed, 保护小球不抖动 */
    ramp_speed  = 0.0f;
    ramp_enable = true;
    start_run   = 1;
}

void control_stop(void)
{
    start_run   = 0;
    ramp_enable = false;
    ramp_speed  = 0.0f;
    motor_behind_control(0, 0);   /* ISR return 后上次 PWM 仍在, 须显式清零否则车不停 */
}

/* ---- 循迹转向环: track_error -> track_turn ---- */
void turn_control(void)
{
    track_error = track_error_get();
    track_turn = (int16_t)(-pidout(&PID_AngleTurn, track_error));
    track_angle = track_turn;
    black_cnt = (uint8_t)(track_sensors[0] + track_sensors[1] + track_sensors[2] + track_sensors[3]
                             + track_sensors[4] + track_sensors[5] + track_sensors[6] + track_sensors[7]);

    if (black_cnt >= 3&&track_sensors[3]&&track_sensors[4]&&(!track_sensors[0])&&(!track_sensors[7])&&(!track_sensors[6])) {
            pl_event = true;
    } 
    else {
        pl_event = false;
    }

}

/* ---- yaw 转向环 (预留, 当前未在 ISR 启用): attitude.yaw -> track_turn ---- */
void yaw_control(void)
{
    PID_YawTurn.desired = 90;
    track_turn = (int16_t)pidout(&PID_YawTurn, attitude.yaw);
    yaw_error = PID_YawTurn.error;
}

/* ---- 速度环: 编码器反馈 -> 电机 PWM ---- */
void speed_control(void)
{
    int16_t bl, br;

    get_speed();

    /* 缓启动: 仅 C_Run (ramp_enable=true) 走斜坡; T4 直接用 basicSpeed.
     * ramp_enable 时 ramp_speed 从 0 向 basicSpeed 逼近 RAMP_STEP, 保护小球不抖动. */
    int16_t spd;
    if (ramp_enable) {
        if (ramp_speed < (float)basicSpeed) {
            ramp_speed += RAMP_STEP;
            if (ramp_speed > (float)basicSpeed) ramp_speed = (float)basicSpeed;
        } else if (ramp_speed > (float)basicSpeed) {
            ramp_speed -= RAMP_STEP;
            if (ramp_speed < (float)basicSpeed) ramp_speed = (float)basicSpeed;
        }
        spd = (int16_t)ramp_speed;
    } else {
        spd = basicSpeed;
    }

    PID_Speed_bL.desired = spd - track_turn;
    PID_Speed_bR.desired = spd + track_turn;

    bl = pidout(&PID_Speed_bL, encoder_bl_speed);
    br = pidout(&PID_Speed_bR, encoder_br_speed);

    motor_behind_control(bl, br);
}

/* ---- 控制初始化: PID 参数 + 启动 100Hz 控制环 ----
 * BSP 子模块 init (encoder/motor/step) 已在 bsp_init() 里, 这里只 Control 自己的.
 * 滚球 PID (PID_StepPos / PID_BallPos) 初始化已移至 ball_init() (见 ball.c). */
void control_init(void)
{
    PidInit(&PID_Speed_bL, 0, g_speed_kp, g_speed_ki, 0, 800, -800);
    PidInit(&PID_Speed_bR, 0, g_speed_kp, g_speed_ki, 0, 800, -800);
    PidInit(&PID_AngleTurn, 0, g_turn_kp, 0, g_turn_kd, 35, -35);
    // PidInit(&PID_YawTurn, 0, 1, 0, 0, 40, -40);   /* yaw 环预留, 启用 yaw_control 时打开 */

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_Timer_startCounter(TIMER_0_INST);
}

/* 读取并清零编码器累计值 -> 每控制周期的脉冲数 (速度) */
void get_speed(void)
{
    encoder_bl_speed = -encoder_bl;      /* ISR 已调为正转递增, 无需取反 */
    encoder_br_speed = encoder_br;      /* ISR 已调为正转递增, 无需取反 */
    encoder_bl = 0;
    encoder_br = 0;
}


/* ---- 控制环 ISR (10ms) ---- */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);

        static uint8_t cnt = 0;          /* 分频计数 */
        cnt++;

        if (!start_run) return;         /* 安全开关: 上电不跑, 按键触发 start_run=true */

        if (cnt % 3 == 0) {             /* ~30ms 一次转向环 */
            turn_control();
        }
        if (cnt % 2 == 0) {             /* ~20ms 一次 yaw 环 (预留) */
            // yaw_control();
        }
        speed_control();                /* 每 10ms 速度环 */

        if (cnt >= 36) cnt = 0;
    }
}
