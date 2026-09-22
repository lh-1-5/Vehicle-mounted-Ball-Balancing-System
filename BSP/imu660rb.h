#ifndef __IMU660RB_H
#define __IMU660RB_H

#include <stdint.h>

/*
 * IMU660RB 驱动 (芯片: LSM6DSR, 6 轴 = 加速度计 + 陀螺仪)
 * 接口: 硬件 I2C (syscfg 实例 I2C_IMU, SCL=PB2, SDA=PB3, 400kHz)
 *
 * 接线 (I2C 模式):
 *   SCL  -> PB2
 *   SDA  -> PB3
 *   CS   -> 3.3V   (选 I2C 模式, 必须拉高, 不可悬空)
 *   SA0  -> 3.3V   (地址 0x6B; 接 GND 则 0x6A, 需改下面的 IMU660RB_DEV_ADDR)
 *   VCC  -> 3.3V
 *   GND  -> GND
 *
 * 用法:
 *   if (imu660rb_init() != 0) { ... 失败处理 ... }   // 0=成功
 *   imu660rb_get_gyro();                              // 读陀螺仪 -> imu660rb_gyro_x/y/z
 *   imu660rb_get_acc();                               // 读加速度计 -> imu660rb_acc_x/y/z
 *   float gx = imu660rb_gyro_transition(imu660rb_gyro_x);  // 原始值 -> °/s
 *   float ax = imu660rb_acc_transition(imu660rb_acc_x);    // 原始值 -> g
 *
 * 量程: 加速度 ±8G, 陀螺仪 ±2000dps (改量程见 imu660rb.c 顶部两个 _RANGE_REG,
 *       并同步改本文件两个 transition 宏的除数).
 */

#define IMU660RB_DEV_ADDR        0x6B    /* 7 位 I2C 地址 (SA0 拉高时) */
#define IMU660RB_WHO_AM_I_VAL    0x6B    /* LSM6DSR 的 WHO_AM_I 值 */

/* 原始数据 (LSB, 有符号 16 位). 调 get_acc/get_gyro 后更新 */
extern int16_t imu660rb_acc_x,  imu660rb_acc_y,  imu660rb_acc_z;
extern int16_t imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z;

/* 初始化: 软复位 -> WHO_AM_I 自检 -> 配置量程/滤波. 返回 0=成功, 1=失败 */
uint8_t imu660rb_init(void);

/* 读加速度计, 结果存 imu660rb_acc_x/y/z */
void imu660rb_get_acc(void);

/* 读陀螺仪, 结果存 imu660rb_gyro_x/y/z */
void imu660rb_get_gyro(void);

/* 原始值 -> 物理量. acc: g(≈9.8m/s^2), gyro: °/s */
#define imu660rb_acc_transition(v)   ((float)(v) / 4096.0f)   /* ±8G, 1g≈4096 */
#define imu660rb_gyro_transition(v)  ((float)(v) / 14.3f)     /* ±2000dps */

#endif /* __IMU660RB_H */
