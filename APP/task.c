#include "task.h"
#include "Control.h"    /* control_run/stop, basicSpeed, start_run */
#include "ball.h"       /* ball_loop_run/stop, set_ball_position, wait_ball_pos */
#include "led_buzzer.h"  /* 到达完成提示 (LED/蜂鸣) */
#include "delay.h"       /* delay_ms (完成短鸣) */
#include "attitude.h"    /* attitude.yaw */
#include "scheduler.h"   /* g_tms */
#include "Motor.h"       /* motor_behind_control */
#include "app_menu.h"    /* app_menu_timer_start/stop (发车计时) */
#include <math.h>        /* fabsf */

int task_num = 0;

void task_one_start(void)
{
    task_num = 1;
}

/* ---- 第三问: 球 0 -> +50 -> -50 序列 ----
 * 状态机在 control_task (主循环) 里非阻塞推进; 球环在 TIMER_BALL ISR 跑 (见 ball.c).
 * 到达判据由 TIMER_BALL ISR 的 ball_pos_control 数稳定帧锁存, wait_ball_pos 读锁存.
 * 前置: 已 Z_cal 标水平零位. 控制模式(单环/双环)由 ball 模块默认+菜单 M_Swt 管,
 *       本序列只用 run/stop/set_position/wait 意图层 API, 不读不写模式, 彻底解耦. */
static uint8_t task_three_state = 0;   /* 文件作用域: task_three_start 可复位 */

void task_three_start(void)
{
    task_three_state = 0;
    task_num = 2;
}

void task_three_stop(void)
{
    task_num = 0;
    task_three_state = 0;
    ball_loop_stop();            /* 关外环 + 内环转回水平零位停住 */
}

static void task_three(void)
{
    switch (task_three_state)
    {
        case 0:   /* 启动: 使能球环 + 阶跃到 +g_t3_pos. 沿用 ball 模块当前模式(默认单环) */
            ball_loop_run();            /* 使能外环(+内环), 清积分/限幅 */
            set_ball_position((int16_t)g_t3_pos);      /* 目标 +g_t3_pos, 同时复位到达锁存 */
            task_three_state = 1;
            break;

        case 1:   /* 等球到 +g_t3_pos (wait_ball_pos 参数被忽略, 到达由 ISR ball_arrived 锁存) */
            if (wait_ball_pos((int16_t)g_t3_pos))
            {
                buzzer_on(); delay_ms(80); buzzer_off();   /* 到 +目标: 一声短鸣 (主循环, 不影响 ISR 控球) */
                set_ball_position((int16_t)g_t3_wait);    /* 目标 g_t3_wait(负值), 反向 */
                task_three_state = 2;
            }
            break;

        case 2:   /* 等球到 g_t3_wait(负值) -> 完成, 保持 */
            if (wait_ball_pos((int16_t)g_t3_wait))
            {
                led_on();                                       /* 到达完成: LED 常亮 */
                buzzer_on(); delay_ms(80); buzzer_off();         /* 一次短鸣 (主循环, 不影响 ISR 控球) */
                task_three_state = 3;
            }
            break;

        case 3:   /* HOLD: 环保持使能, 外环继续把球控在 -50, 不再转移 */
            break;
    }
}


/* ==================== 第四问: 循迹一圈并定点停车 ====================
 * 状态机 (control_task 主循环非阻塞推进, 小车环在 TIMER_0 ISR 跑):
 *   CAR_START_LINE_WAIT  起跑线等待: pl_event 到来 或 START_LINE_TIMEOUT_MS 超时 -> RUNNING
 *   CAR_RUNNING          循迹运行: 从开始跑计时 g_run_sec 秒 或 累计角度达 ANGLE_STOP_DEG -> SLOW
 *   CAR_SLOW             减速段: target_speed=3, pl_event 到来 -> BRAKING
 *   CAR_BRAKING          刹车: 电机清零, control_stop 关 ISR 速度环
 *
 * 减速触发 = 时间 OR 角度 (任一满足即转 SLOW).
 *   时间: run_start_tms 在进入 RUNNING 时设为 g_tms, 到 g_run_sec 秒后触发.
 *   角度: 进入 RUNNING 时记 base_angle=angle_sumer, 后续 |angle_sumer-base_angle|>=ANGLE_STOP_DEG 触发.
 *         angle_sum 内部 static 累计不 reset, 用相对基准实现"从开始跑才计数", 不影响 ISR 连续性.
 * g_run_sec 菜单 PARAM + AT24C02 持久化 (at_params.c 注册), 编辑后实时生效.
 * 状态变量/起跑线检测/angle_sum 在 Control.c 定义 (Control.h extern), task.c 只跑状态机.
 * pl_event 由 ISR turn_control 检测起跑线置位(读到即消费); angle_sumer 调 Control.c 的 angle_sum (菜单调试用).
 * target_speed 同步到 basicSpeed 控制速度环; BRAKING 后 control_stop 关速度环防 ISR 覆盖 PWM. */
#define START_LINE_TIMEOUT_MS   2000U      /* 起跑线等待超时(ms): 超时强制开跑 */
#define CAR_SLOW_SPEED          2          /* 减速段速度(脉冲数/10ms) */

float g_run_sec = 13.0f;                   /* 减速触发时间(秒), 菜单 PARAM + AT24C02 持久化 */
float g_angle_stop_deg = 250.0f;           /* 减速触发角度阈值(累计°, 一圈≈360), 菜单 PARAM + AT24C02 持久化 */
float g_yaw_stop_min = 15.0f;              /* 停车 yaw 下限(°), 菜单 PARAM + AT24C02 持久化 */
float g_yaw_stop_max = 35.0f;              /* 停车 yaw 上限(°), 菜单 PARAM + AT24C02 持久化 */
float g_t3_pos    = 50.0f;                 /* T3 正目标距离(+50), set_ball_position 用, 菜单 PARAM + AT24C02 持久化 */
float g_t3_wait   = -45.0f;                /* T3 负目标距离(-45), wait_ball_pos 用, 菜单 PARAM + AT24C02 持久化 */

float angle_sumer = 0.0f;                 /* 角度累计 (菜单 READONLY, = angle_sum 返回值) */
static int16_t target_speed = 5;
static int16_t s_saved_speed = 5;         /* 启动时保存 basicSpeed, BRAKING 后恢复 */
static float base_angle = 0.0f;           /* 进入 RUNNING 时的角度基准, 角度减速判定用相对值 */
static uint32_t pl_true_tms = 0;          /* pl_event 持续为 true 的起始时刻, SLOW 段 90ms 去抖 */

void set_start_line_event(void)
{
    pl_event = true;
}

void task_four_start(void)
{
    task_num = 3;
    state_reset();                 /* 复位 Control.c 里所有状态变量 (car_state=CAR_IDLE 等) */
    car_state = CAR_START_LINE_WAIT;
    run_start_tms = g_tms;
    s_saved_speed = basicSpeed;    /* 保存菜单当前速度, BRAKING 后恢复 */
    target_speed = basicSpeed;     /* 起始速度用菜单当前值 */
    control_run();                 /* 启动循迹: start_run=true */
}

void task_four_stop(void)
{
    task_num = 0;
    car_state = CAR_BRAKING;
    app_menu_timer_stop();         /* 手动中止: 冻结计时 */
    target_speed = 0;
    basicSpeed = s_saved_speed;    /* 恢复菜单速度 */
    control_stop();                /* 关速度环 ISR + PWM 清零 */
}

static void task_four(void)
{
    angle_sumer = angle_sum(-attitude.yaw);

    switch (car_state)
    {
    case CAR_START_LINE_WAIT:
        if (pl_event) {
            pl_event = false;
            run_start_tms = g_tms;        /* 开始跑时刻, 供 RUNNING 计时 */
            base_angle = angle_sumer;     /* 角度基准, 供 RUNNING 角度判定 (从开始跑起算) */
            car_state = CAR_RUNNING;
        }
        else if ((uint32_t)(g_tms - run_start_tms) > START_LINE_TIMEOUT_MS) {
            run_start_tms = g_tms;        /* 超时开跑: 同样记开始跑时刻 */
            base_angle = angle_sumer;
            car_state = CAR_RUNNING;
        }
        break;

    case CAR_RUNNING:
        /* 减速触发: 时间 OR 角度 (任一满足即转 SLOW).
         * 角度用相对值 angle_sumer - base_angle, 实现"从开始跑才计数". */
        if ((uint32_t)(g_tms - run_start_tms) >= (uint32_t)(g_run_sec * 1000.0f) &&
            fabsf(angle_sumer - base_angle) >= g_angle_stop_deg) {
            pl_true_tms = 0;        /* 进入 SLOW 前清零, 屏蔽 RUNNING 期间残留 */
            car_state = CAR_SLOW;
        }
        break;

    case CAR_SLOW:
        target_speed = CAR_SLOW_SPEED;
        /* 起跑线 90ms 连续去抖: pl_event 必须持续为 true 达 90ms 才算真起跑线.
         * 弯道/十字路口只会瞬时匹配一两帧(~30-60ms), 持续时间不足 90ms 不会误触发. */
        if (pl_event&&(attitude.yaw>g_yaw_stop_min&&attitude.yaw<g_yaw_stop_max)) {
            if (pl_true_tms == 0) pl_true_tms = g_tms;          /* 记录上升沿时刻 */
            if (g_tms - pl_true_tms >= 90) {
                car_state = CAR_BRAKING;
            }
        } else {
            pl_true_tms = 0;                                    /* 中途断开, 重新计时 */
        }
        break;

    case CAR_BRAKING:
        app_menu_timer_stop();        /* 停车: 冻结计时 (幂等, 每帧调安全) */
        motor_behind_control(0, 0);
        target_speed = 0;
        break;

    default:
        break;
    }

    basicSpeed = target_speed;   /* 同步到速度环目标 (菜单 spd 会跟随显示) */

    /* 刹车状态: 关速度环 ISR 防 speed_control 覆盖 motor_behind_control(0,0).
     * control_stop 幂等, 每帧调无害. */
    if (car_state == CAR_BRAKING) {
        control_stop();
    }
}


/* ---- 主循环周期任务 (非 ISR) ----
 * 在 system_run 的 while(1) 里调. 小车环在 TIMER_0 ISR, 球环在 TIMER_BALL ISR (ball.c). */
void control_task(void)
{
    switch (task_num)
    {
        case 0:
            break;
        case 1:
            control_run();
            break;
        case 2:
            task_three();
            break;
        case 3:
            task_four();
            break;
    }
}
