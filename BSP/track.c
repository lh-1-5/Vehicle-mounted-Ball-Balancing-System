/* track.c - 8 路红外循迹误差计算.
 * 传感器读取见 track.h 的 Read_Track_X() 宏 (GPIO_Track 组, 低电平=黑线). */
#include "track.h"

#define TRACK_ERROR_MAX   10
#define TRACK_ERROR_MIN  (-10)

#define CLAMP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* 上一次误差, 用于丢线时继承方向 */
static int16_t last_track_error = 0;

/* 8 路传感器最新读数 (0=白线, 1=黑线), 由 track_update() 刷新.
 * 暴露给调试页面直接读取实时电平. */
uint8_t track_sensors[8];

/* 读取 8 路传感器到 track_sensors[]. track_error_get() 内部调用, 保证数组新鲜. */
void track_update(void)
{
    track_sensors[0] = Read_Track_1();
    track_sensors[1] = Read_Track_2();
    track_sensors[2] = Read_Track_3();
    track_sensors[3] = Read_Track_4();
    track_sensors[4] = Read_Track_5();
    track_sensors[5] = Read_Track_6();
    track_sensors[6] = Read_Track_7();
    track_sensors[7] = Read_Track_8();
}

int16_t track_error_get(void)
{
    track_update();

    /* 检测到黑线的传感器总数 */
    uint8_t sensor_cnt = (uint8_t)(track_sensors[0] + track_sensors[1] + track_sensors[2] + track_sensors[3]
                                 + track_sensors[4] + track_sensors[5] + track_sensors[6] + track_sensors[7]);

    /* 加权累加: 左侧负, 右侧正 */
    int16_t sum_error = 0;
    sum_error += (int16_t)(track_sensors[0] * (-7));
    sum_error += (int16_t)(track_sensors[1] * (-5));
    sum_error += (int16_t)(track_sensors[2] * (-3));
    sum_error += (int16_t)(track_sensors[3] * (-1));
    sum_error += (int16_t)(track_sensors[4] * ( 1));
    sum_error += (int16_t)(track_sensors[5] * ( 3));
    sum_error += (int16_t)(track_sensors[6] * ( 5));
    sum_error += (int16_t)(track_sensors[7] * ( 7));

    /* 丢线(全白): 按上次方向返回极值, 保持转向找回线 */
    if (sensor_cnt == 0) {
        if      (last_track_error > 0) return TRACK_ERROR_MAX;
        else if (last_track_error < 0) return TRACK_ERROR_MIN;
        else                           return 0;
    }

    /* 全黑(十字路口等): 清误差返回 0 */
    if (sensor_cnt == 8) {
        last_track_error = 0;
        return 0;
    }

    /* 归一化误差并限幅 */
    int16_t error = (int16_t)(sum_error / sensor_cnt);
    error = CLAMP(error, TRACK_ERROR_MIN, TRACK_ERROR_MAX);
    last_track_error = error;

    return error;
}
