/*
 * ball.c - 滚球平衡控制 (ball-and-beam)
 *
 * 步进电机控管子倾角, 球靠重力平衡. 独占 TIMER_BALL (TIMG6, 10ms), 与小车循迹
 * (Control.c / TIMER_0) 解耦, 不受 start_run 约束.
 *
 * 控制模式 (编译期定, 改 BALL_LOOP_SINGLE 重烧, 无运行时切换):
 *   单环位置式 (BALL_LOOP_SINGLE=1, 默认主用): 外环 PID 出目标步位置 ->
 *       step_on_at_freq 直驱, 软件累计 step_virt_pos 做角度基准, error=0 主动回正.
 *       失步会漂(已知代价). 大扰动收敛优于双环.
 *   双环串级 (BALL_LOOP_SINGLE=0, 备选): 外环出目标倾角deg -> mechanism 换算 ->
 *       step_target_pos, 内环 PID_StepPos 编码器闭环主动保持管子倾角 (反馈不慢半拍).
 *
 * 详见 docs/步进位置闭环设计记录.md + docs/滚球模块拆分方案.md.
 */
#include "ti_msp_dl_config.h"
#include "ball.h"
#include "Encoder.h"
#include "step_motor.h"
#include "pid.h"
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
#include "scheduler.h"    /* g_tms: 摄像头帧的1ms时间戳 */
#endif
#if !BALL_LOOP_SINGLE
#include "mechanism.h"    /* 双环才需要: ball_tilt_to_step_pos / MECH_TILT_MAX_DEG */
#endif

/* 旧D方案固定30ms调用; 新D方案每10ms检查帧序号, 只消费摄像头新帧. */
#define BALL_LOOP_DIV        3
#define BALL_DIR_INVERT     (-1)     /* 球位误差->驱动 方向修正号, 实测翻号 */

/* 到达判定: |ball-target|<=容差 连续 N 拍 -> 锁存到达. ±5mm/500ms(17拍@30ms). */
#define BALL_ARRIVE_TOL       8
#define BALL_ARRIVE_FRAMES    13

/* ---- 单环增益 (输出=目标步位置偏置). 曲柄连杆减速比 r/l≈0.13 -> 同 out 的管倾角
 *      比 1:1 直连小 ~7×, 故 out 量级须比直连大 ~7× (直连 Kp=1.5 -> 连杆 Kp~10+).
 *      注: 此前 out 被 BALL_STEP_MAX=300 钳死, 升 Kp/KD 无效; 拆限位后 Kp 才起作用. ---- */
#define BALL_PID_SINGLE_KP    8.0f
#define BALL_PID_SINGLE_KI     0.12f   /* 条件积分(仅近目标累加), 消稳态偏置/死区 */
#define BALL_PID_SINGLE_KD     17.0f
#define BALL_STEP_HZ          12000    /* 单环定步数频率 (= STEP_SPEED_MAX_HZ 上限) */

/* 三层限位 (微步), 各司其职. 原 BALL_STEP_MAX=300 身兼输出上限+斜坡, 直连下 300步=8.4°
 * 管倾够用; 连杆减速后 300步=1.1°管倾太小(摆幅不足/out顶300死锁/小误差电机振) -> 拆分:
 *   BALL_OUT_MAX    PID输出上限=target范围=管倾权威. 2000步≈臂56°≈管倾7°(直连8.4°等价)
 *   BALL_DELTA_MAX  单拍增量上限=斜坡限速器(大扰动稳定关键, 限管倾变化率). 保持300
 *   BALL_VIRT_POS_* 死点硬guard(几何安全弧±3600内), 限target */
#define BALL_OUT_MAX           4000
/* 自适应斜坡(方案B): 大delta放宽让管子快倾(大阶跃加速), 小delta收紧精确定位不过冲.
 * 替代原固定 BALL_DELTA_MAX=600. -50 大阶跃时 out 顶满 2000, 放宽到 1500 让管子
 * 2拍倾到位, 球快速加速; 近目标收紧到 300 防过冲. */
#define BALL_DELTA_MAX_FAR      1500     /* |delta|>FAR_THRESH 时用(大阶跃加速段) */
#define BALL_DELTA_MAX_MID       800     /* |delta| 在 MID..FAR 之间用(过渡) */
#define BALL_DELTA_MAX_NEAR      300     /* |delta|<MID_THRESH 时用(精确定位, 防过冲) */
#define BALL_DELTA_FAR_THRESH   800     /* 进入 FAR 段的 delta 门槛(微步) */
#define BALL_DELTA_MID_THRESH   250     /* 进入 NEAR 段的 delta 门槛(微步) */
#define BALL_DELTA_DEAD           3     /* 小delta死区(微步): 抑制稳态零星脉冲抖 */

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
/* 新帧速度 D. KV=旧Kd(16)*旧周期(0.03s)=0.48, 与原D阻尼量级等效. */
#define BALL_PID_SINGLE_KV          2.60f
#define BALL_VEL_FILTER_TAU_MS     30.0f   /* 滤波加重压 D 噪声抖; dt=33ms 时 alpha≈0.53 仍跟手 */
#define BALL_FRAME_DT_MIN_MS        5U
#define BALL_FRAME_DT_MAX_MS      100U
#define BALL_FRAME_TIMEOUT_MS     100U
#define BALL_D_OUT_MAX           2200.0f   /* D 限幅配 KV 升级, 给大摆刹车余量 */
#define BALL_ARRIVE_SPEED_MAX     100.0f   /* 位置单位/s */
#define BALL_OLD_NOMINAL_DT_MS     30.0f   /* 保持原Ki每30ms累加一次的量级 */
/* 条件积分: 球-杆双积分对象加 I 易振, 故仅近目标窄带累加 + 限幅 + 饱和漏.
 * I_BAND 须 > 最坏稳态误差(如 -50 停在 -28 差 -28), 否则误差落带外被漏永远消不掉. */
#define BALL_I_BAND                 60.0f   /* 积分带(mm): |error|<此值才积分, 覆盖减速段 */
#define BALL_I_MAX                 3000.0f /* 积分上限(KI=0.12 -> i_out<=360步) */
#endif

/* 死点软限位 (几何算得 r=35/L=55.5/d=225/l=260: 死点电机臂朝支撑±102.5°=±3600微步).
 * 限 target 不限 step_virt_pos (限状态则失步卡死, 即已删 BALL_STEP_LIMIT 坑).
 * 2500(≈臂70°)在安全弧内留余量, >OUT_MAX 做 guard. */
#define BALL_VIRT_POS_MAX     2500
#define BALL_VIRT_POS_MIN    -2500

/* ---- 双环增益 (输出=目标倾角deg, 限 ±MECH_TILT_MAX_DEG) ---- */
#define BALL_PID_DUAL_KP      0.1f
#define BALL_PID_DUAL_KI       0.0f
#define BALL_PID_DUAL_KD       0.9f

volatile int32_t step_target_pos = 0;     /* 双环内环目标位置(脉冲). 双环+step页用 */
volatile int16_t ball_pos = 0;            /* 球位置(摄像头 0x21, ±150, 左正右负). uart ISR 写 */
volatile int16_t ball_target_pos = 0;     /* 目标球位置, 菜单 PARAM */

/* 单环 PID 增益 (菜单 PARAM, AT24C02 持久化).
 * 编译初值=原宏值; 上电 at_params_init 从 EEPROM 覆盖; 菜单编辑提交时回写.
 * g_ball_kd 对应帧-PD 模式的速度增益(原 BALL_PID_SINGLE_KV), 即默认模式下实际起作用的 D 项. */
float g_ball_kp = BALL_PID_SINGLE_KP;
float g_ball_ki = BALL_PID_SINGLE_KI;
float g_ball_kd = BALL_PID_SINGLE_KV;

static volatile bool     ball_loop_enable = false;   /* 环使能, 默认关. 菜单 B_Run/B_Stp */
static volatile uint16_t ball_stable_cnt  = 0;       /* 连续稳定拍数 */
static volatile bool     ball_arrived     = false;   /* 到达锁存(换目标复位) */
#if BALL_LOOP_SINGLE
static int32_t step_virt_pos = 0;        /* 单环软件累计位置(角度基准), 启动时同步编码器 */
#endif

float g_out = 0;                        /* 外环 PID 输出, 调试显示 */

volatile uint32_t g_ball_frame_seq          = 0;
volatile uint32_t g_ball_dt_ms              = 0;
volatile float    g_ball_velocity_raw       = 0.0f;
volatile float    g_ball_velocity_filtered  = 0.0f;
volatile float    g_ball_p_out              = 0.0f;
volatile float    g_ball_i_out              = 0.0f;
volatile float    g_ball_d_out              = 0.0f;

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
static volatile uint32_t s_ball_frame_time_ms = 0;
static uint32_t s_last_frame_seq               = 0;
static uint32_t s_previous_frame_time_ms       = 0;
static int16_t  s_previous_ball_pos            = 0;
static bool     s_velocity_initialized         = false;
static bool     s_frame_timeout_active         = false;
#endif

/* 双环内环前向声明 (单环编译时此函数不存在) */
#if !BALL_LOOP_SINGLE
void step_pos_control(void);
#endif
void ball_pos_control(void);   /* 外环: ball_isr_10ms 在前调用 */


/* 初始化: PID (增益/限幅按编译期模式定) + 启动 TIMER_BALL. */
void ball_init(void)
{
    PidInit(&PID_StepPos, 0, 50.0f, 0.0f, 0.0f, STEP_SPEED_MAX_HZ, -STEP_SPEED_MAX_HZ);  /* 双环内环用, 单环也 init 无害 */

#if BALL_LOOP_SINGLE
    PidInit(&PID_BallPos, 0, g_ball_kp, g_ball_ki, g_ball_kd,
            (float)BALL_OUT_MAX, -(float)BALL_OUT_MAX);
#else
    PidInit(&PID_BallPos, 0, BALL_PID_DUAL_KP, BALL_PID_DUAL_KI, BALL_PID_DUAL_KD,
            MECH_TILT_MAX_DEG, -MECH_TILT_MAX_DEG);
    step_target_pos = encoder_step;   /* 双环: 内环误差0保持原地, 不回绝对0位乱转 */
#endif

    NVIC_EnableIRQ(TIMER_BALL_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_BALL_INST);
}

/* TIMER_BALL ISR 入口 (10ms): 内环每拍(仅双环), 外环 3 分频(30ms). */
void ball_isr_10ms(void)
{
    static uint8_t ball_cnt = 0;
    ball_cnt++;

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    /* 每10ms检查一次; ball_pos_control 仅在摄像头新帧到来时更新控制量. */
    ball_pos_control();
#else
    if (ball_cnt % BALL_LOOP_DIV == 0) ball_pos_control();   /* 旧方案外环30ms */
#endif

#if !BALL_LOOP_SINGLE
    step_pos_control();    /* 双环内环: 每10ms逼近step_target_pos. 单环电机已由外环直驱, 不跑. */
#endif

    if (ball_cnt >= 36) ball_cnt = 0;
}


void set_step_target(int input_pos)
{
    step_target_pos = input_pos;
}

void ball_measurement_update(int16_t position)
{
    ball_pos = position;
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    /* 序号最后写: 控制ISR看到新序号时, 位置和时间戳已经就绪. */
    s_ball_frame_time_ms = (uint32_t)g_tms;
    g_ball_frame_seq++;
#endif
}

/* 读到达锁存(主循环非阻塞). pos 仅兼容签名, 判据用 ball_target_pos(换目标时复位). */
bool wait_ball_pos(int16_t pos)
{
    (void)pos;
    return ball_arrived;
}

void set_ball_position(int16_t pos)
{
    ball_target_pos = pos;
    ball_stable_cnt = 0;       /* 换目标复位到达锁存 */
    ball_arrived    = false;
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    PID_BallPos.integ = 0.0f;   /* 旧目标的积分对新目标方向错, 清掉防冲过头 */
#endif
}

#if !BALL_LOOP_SINGLE
/* 双环内环: encoder_step 反馈 -> PID_StepPos -> step_drive_hz. */
void step_pos_control(void)
{
    int32_t pos = encoder_step;
    PID_StepPos.desired = (float)step_target_pos;
    float out_hz = pidout(&PID_StepPos, (float)pos);
    step_drive_hz((int32_t)out_hz);
}
#endif

#if BALL_LOOP_SINGLE
static void ball_single_drive_target(float out)
{
    int32_t target = (int32_t)out;
    if (target >  BALL_VIRT_POS_MAX) target =  BALL_VIRT_POS_MAX;
    if (target <  BALL_VIRT_POS_MIN) target =  BALL_VIRT_POS_MIN;

    int32_t delta = target - step_virt_pos;

    /* 自适应斜坡(方案B): 按 |delta| 分段 clamp. 大delta放宽快倾(大阶跃加速段),
     * 小delta收紧防过冲(近目标精确定位). 替代原固定 BALL_DELTA_MAX. */
    int32_t delta_abs = (delta < 0) ? -delta : delta;
    int32_t delta_cap = BALL_DELTA_MAX_NEAR;
    if (delta_abs >= BALL_DELTA_FAR_THRESH) {
        delta_cap = BALL_DELTA_MAX_FAR;
    } else if (delta_abs >= BALL_DELTA_MID_THRESH) {
        delta_cap = BALL_DELTA_MAX_MID;
    }
    if (delta >  delta_cap) delta =  delta_cap;
    if (delta < -delta_cap) delta = -delta_cap;

    /* 小 delta 死区: |delta|<=DEAD 不发脉冲, 抑制稳态零星脉冲抖(1-3步@12kHz咔哒).
     * 残留偏置由条件积分顶过去, 不靠磨脉冲. */
    if ((delta > BALL_DELTA_DEAD) || (delta < -BALL_DELTA_DEAD)) {
        step_on_at_freq(delta, BALL_STEP_HZ);
        step_virt_pos += delta;
    } else {
        step_off();
    }
}
#endif

/* 外环: 球位误差 -> PID_BallPos -> 方向修正 -> 按模式驱动. 含到达判定. */
void ball_pos_control(void)
{
    if (!ball_loop_enable) return;

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    uint32_t seq_before;
    uint32_t seq_after;
    uint32_t frame_time_ms;
    int16_t ball;

    /* UART可能在读取期间更新测量; 前后序号一致才接受该快照. */
    do {
        seq_before = g_ball_frame_seq;
        ball = ball_pos;
        frame_time_ms = s_ball_frame_time_ms;
        seq_after = g_ball_frame_seq;
    } while (seq_before != seq_after);

    if ((seq_before == 0U) || (seq_before == s_last_frame_seq)) {
        if ((seq_before != 0U) &&
            (((uint32_t)g_tms - frame_time_ms) > BALL_FRAME_TIMEOUT_MS)) {
            if (!s_frame_timeout_active) {
                s_frame_timeout_active = true;
                s_velocity_initialized = false;
                PID_BallPos.integ = 0.0f;
                g_out = 0.0f;
                g_ball_dt_ms = 0U;
                g_ball_velocity_raw = 0.0f;
                g_ball_velocity_filtered = 0.0f;
                g_ball_p_out = 0.0f;
                g_ball_i_out = 0.0f;
                g_ball_d_out = 0.0f;
            }
            /* 丢帧后分段回水平. 等上一段走完再发下一段, 不打断定步数命令. */
            if (step1_count == 0U) ball_single_drive_target(0.0f);
        }
        return;
    }

    /* 大角度命令尚未走完时先不消费新帧; 完成后直接处理当时的最新帧. */
    if (step1_count > 0U) return;

    s_last_frame_seq = seq_before;
    s_frame_timeout_active = false;

    uint32_t dt_ms = 0U;
    float velocity_raw = 0.0f;
    float velocity_filtered = g_ball_velocity_filtered;
    float integral_scale = 1.0f;

    if (s_velocity_initialized) {
        dt_ms = frame_time_ms - s_previous_frame_time_ms;
        if ((dt_ms >= BALL_FRAME_DT_MIN_MS) && (dt_ms <= BALL_FRAME_DT_MAX_MS)) {
            int32_t delta_pos = (int32_t)ball - (int32_t)s_previous_ball_pos;
            velocity_raw = (float)delta_pos * 1000.0f / (float)dt_ms;
            float alpha = (float)dt_ms / (BALL_VEL_FILTER_TAU_MS + (float)dt_ms);
            velocity_filtered += alpha * (velocity_raw - velocity_filtered);
            integral_scale = (float)dt_ms / BALL_OLD_NOMINAL_DT_MS;
        } else {
            velocity_filtered = 0.0f;
            integral_scale = 0.0f;
        }
    } else {
        s_velocity_initialized = true;
        velocity_filtered = 0.0f;
    }

    s_previous_ball_pos = ball;
    s_previous_frame_time_ms = frame_time_ms;
    g_ball_dt_ms = dt_ms;
    g_ball_velocity_raw = velocity_raw;
    g_ball_velocity_filtered = velocity_filtered;

    float error = (float)ball_target_pos - (float)ball;

    float p_out = g_ball_kp * error;
    float d_out = -g_ball_kd * velocity_filtered;
    if (d_out >  BALL_D_OUT_MAX) d_out =  BALL_D_OUT_MAX;
    if (d_out < -BALL_D_OUT_MAX) d_out = -BALL_D_OUT_MAX;

    /* 条件积分: 近目标(I_BAND)且未饱和时累加消稳态偏置/静摩擦; out 饱和时漏 10% 防 windup;
     * 带外未饱和时不积不漏(等球减速进带). dt 异常帧(integral_scale==0)不动. 限幅 ±BALL_I_MAX.
     * I_BAND 须大到覆盖减速段: 原值太小会把逼近路当远端漏, 到近目标积分没攒够克服不了
     * 静摩擦卡死(-50 停在 -3 不动的根因). */
    if (integral_scale > 0.0f) {
        float err_abs = (error < 0.0f) ? -error : error;
        float pd_abs = ((p_out + d_out) < 0.0f) ? -(p_out + d_out) : (p_out + d_out);
        if ((err_abs < BALL_I_BAND) && (pd_abs < (float)BALL_OUT_MAX)) {
            PID_BallPos.integ += error * integral_scale;
        } else if (pd_abs >= (float)BALL_OUT_MAX) {
            PID_BallPos.integ *= 0.9f;
        }
        if (PID_BallPos.integ >  BALL_I_MAX) PID_BallPos.integ =  BALL_I_MAX;
        if (PID_BallPos.integ < -BALL_I_MAX) PID_BallPos.integ = -BALL_I_MAX;
    }
    float i_out = g_ball_ki * PID_BallPos.integ;

    float out = p_out + i_out + d_out;
    if (out >  (float)BALL_OUT_MAX) out =  (float)BALL_OUT_MAX;
    if (out < -(float)BALL_OUT_MAX) out = -(float)BALL_OUT_MAX;
    out *= BALL_DIR_INVERT;

    g_ball_p_out = p_out;
    g_ball_i_out = i_out;
    g_ball_d_out = d_out;
    g_out = out;
#else
    int16_t ball = ball_pos;
    PID_BallPos.desired = (float)ball_target_pos;
    float out = pidout(&PID_BallPos, (float)ball);
    out *= BALL_DIR_INVERT;
    g_out = out;
#endif

    /* 到达判定: |ball-target|<=容差 连续 N 拍 -> 锁存 */
    int16_t diff = ball - ball_target_pos;
    bool ball_is_stable = (diff <= BALL_ARRIVE_TOL && diff >= -BALL_ARRIVE_TOL);
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    float speed_abs = g_ball_velocity_filtered;
    if (speed_abs < 0.0f) speed_abs = -speed_abs;
    ball_is_stable = ball_is_stable && (speed_abs <= BALL_ARRIVE_SPEED_MAX);
#endif
    if (ball_is_stable) {
        if (ball_stable_cnt < 0xFFFF) ball_stable_cnt++;
        if (ball_stable_cnt >= BALL_ARRIVE_FRAMES) ball_arrived = true;
    } else {
        ball_stable_cnt = 0;
        ball_arrived    = false;
    }

#if BALL_LOOP_SINGLE
    ball_single_drive_target(out);
#else
    /* 双环串级: 倾角deg -> 几何换算 -> 内环目标 */
    step_target_pos = ball_tilt_to_step_pos(out);
#endif
}

/* 启停(菜单 B_Run/B_Stp).
 * run: 单环每次启动 step_virt_pos=0 (以当前位置为回正零点, 不读编码器).
 * stop: 双环内环每10ms重驱动, step_off会被覆盖; 冻结目标->误差0->软停. */
void ball_loop_run(void)
{
#if BALL_LOOP_SINGLE
    step_virt_pos = 0;   /* 单环相对基准(0=当前位). 不读编码器: 脉冲1024/圈与微步12800/圈单位混 */
#if BALL_SINGLE_FRAME_PD
    /* 下一拍处理当前最新帧, 但首帧速度固定为0, 避免把静止位置误当速度. */
    s_last_frame_seq = (g_ball_frame_seq == 0U) ? 0U : (g_ball_frame_seq - 1U);
    s_previous_ball_pos = ball_pos;
    s_previous_frame_time_ms = s_ball_frame_time_ms;
    s_velocity_initialized = false;
    s_frame_timeout_active = false;
    PID_BallPos.integ = 0.0f;     /* 启动清积分, 防上次残留冲过头 */
    g_ball_dt_ms = 0U;
    g_ball_velocity_raw = 0.0f;
    g_ball_velocity_filtered = 0.0f;
    g_ball_p_out = 0.0f;
    g_ball_i_out = 0.0f;
    g_ball_d_out = 0.0f;
#endif
#else
    /* 双环: 清积分防残留 (PID 积分在 stop 期间可能漂) */
    PID_BallPos.integ = 0.0f;
    PID_BallPos.preverror = 0.0f;
#endif
    ball_loop_enable = true;
}

void ball_loop_stop(void)
{
    ball_loop_enable = false;
#if BALL_LOOP_SINGLE
    step_off();                 /* 单环: 停电机 */
#if BALL_SINGLE_FRAME_PD
    s_velocity_initialized = false;
    s_frame_timeout_active = false;
    PID_BallPos.integ = 0.0f;     /* 停环清积分, 下次启动从0开始 */
    g_ball_velocity_raw = 0.0f;
    g_ball_velocity_filtered = 0.0f;
    g_ball_p_out = 0.0f;
    g_ball_i_out = 0.0f;
    g_ball_d_out = 0.0f;
#endif
#else
    step_target_pos = mechanism_get_level_zero();   /* 双环: 转回水平零位停住 */
#endif
}


/* 滚球控制环 ISR (TIMG6, 10ms). 仿 attitude.c (同为 TIMG) 写法. */
void TIMER_BALL_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(TIMER_BALL_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    ball_isr_10ms();
}
