/*
 * mpu6050_raw.c - MPU6050 / MPU6500 6 轴 IMU 驱动, 硬件 I2C, 不用 DMP.
 *
 * 结构仿 imu660rb.c: I2C 底层 (等待空闲/写寄存器/突发读) + 对外 API.
 * 关键差异 (相对 imu660rb):
 *   1. 数据大端序: 拼接用 (d[0]<<8)|d[1] (imu660rb 是 (d[1]<<8)|d[0]).
 *   2. 寄存器地址不同: WHO_AM_I=0x75, 数据从 0x3B(ACC)/0x43(GYRO) 起.
 *   3. 量程系数不同: gyro 1°/s=16.4 LSB (imu660rb 是 14.3).
 *   4. 初始化序列: MPU6050 需先 DEVICE_RESET -> 设 CLKSEL=1(PLL) -> 配 DLPF/SMPLRT/量程.
 *      DLPF_CFG=3 使陀螺内部采样率降到 1kHz, 再 SMPLRT_DIV=4 得 200Hz 输出.
 *
 * I2C 读时序沿用 imu660rb.c 的 repeated-start 写法 (MCTR = RD_ON_TXEMPTY_ENABLE),
 * 该写法在去年 MPU6050 上已 proven.
 */
#include "ti_msp_dl_config.h"
#include "mpu6050_raw.h"
#include "delay.h"
#include <stdbool.h>

/* ---- MPU6050 寄存器地址 ---- */
#define REG_SMPLRT_DIV     0x19   /* 采样率分频: rate = 1kHz / (1 + SMPLRT_DIV) (DLPF!=0 时) */
#define REG_CONFIG         0x1A   /* DLPF_CFG (bit2-0): !=0 时陀螺内部 1kHz */
#define REG_GYRO_CONFIG    0x1B   /* FS_SEL (bit4-3): 3=±2000dps */
#define REG_ACCEL_CONFIG   0x1C   /* AFS_SEL (bit4-3): 2=±8g */
#define REG_ACCEL_CONFIG2  0x1D   /* MPU6500: 加速度计独立 DLPF; MPU6050 此地址含义不同 */
#define REG_ACCEL_XOUT_H   0x3B   /* 加速度 X 高字节 (突发读 6 字节, 大端) */
#define REG_GYRO_XOUT_H    0x43   /* 陀螺 X 高字节 (突发读 6 字节, 大端) */
#define REG_PWR_MGMT_1     0x6B   /* bit7=DEVICE_RESET, bit6=SLEEP, bit0=CLKSEL */
#define REG_WHO_AM_I       0x75   /* 器件 ID: MPU6050=0x68, MPU6500=0x70 */

/* ---- 全局数据 ---- */
int16_t mpu6050_acc_x = 0, mpu6050_acc_y = 0, mpu6050_acc_z = 0;
int16_t mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;
volatile uint8_t mpu6050_device_id = 0;

/* ---- I2C 底层 (与 imu660rb.c 同构, 超时用循环计数, 不依赖 clock) ---- */
#define I2C_TIMEOUT_LOOP  200000u

static bool mpu_i2c_wait_idle(void)
{
    uint32_t to = I2C_TIMEOUT_LOOP;
    while (!(DL_I2C_getControllerStatus(I2C_IMU_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--to == 0) return false;
    }
    return true;
}

static int mpu_write_reg(uint8_t reg, uint8_t data)
{
    DL_I2C_transmitControllerData(I2C_IMU_INST, reg);
    DL_I2C_clearInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    if (!mpu_i2c_wait_idle()) return -1;

    DL_I2C_startControllerTransfer(I2C_IMU_INST, MPU6050_DEV_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    DL_I2C_fillControllerTXFIFO(I2C_IMU_INST, &data, 1);

    uint32_t to = I2C_TIMEOUT_LOOP;
    while (!DL_I2C_getRawInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        if (--to == 0) return -1;
    }
    return 0;
}

/* 突发读 len 个寄存器 (MPU6050 地址自增). repeated-start 时序同 imu660rb. */
static int mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;

    DL_I2C_flushControllerRXFIFO(I2C_IMU_INST);
    DL_I2C_transmitControllerData(I2C_IMU_INST, reg);
    I2C_IMU_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    if (!mpu_i2c_wait_idle()) {
        DL_I2C_disableControllerReadOnTXEmpty(I2C_IMU_INST);
        return -1;
    }

    DL_I2C_startControllerTransfer(I2C_IMU_INST, MPU6050_DEV_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    uint32_t to = I2C_TIMEOUT_LOOP;
    while (!DL_I2C_getRawInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE)) {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
            uint8_t c = (uint8_t)DL_I2C_receiveControllerData(I2C_IMU_INST);
            if (i < len) buf[i++] = c;
        }
        if (--to == 0) {
            I2C_IMU_INST->MASTER.MCTR = 0;
            DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);
            return -1;
        }
    }
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
        if (i < len) buf[i++] = (uint8_t)DL_I2C_receiveControllerData(I2C_IMU_INST);
    }
    I2C_IMU_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);

    return (i == len) ? 0 : -1;
}

static int mpu_read_reg(uint8_t reg, uint8_t *value)
{
    return mpu_read_regs(reg, value, 1);
}

static bool mpu_device_id_supported(uint8_t id)
{
    return id == MPU6050_WHO_AM_I_VAL || id == MPU6500_WHO_AM_I_VAL;
}

/* ---- 对外 API ---- */
uint8_t mpu6050_init(void)
{
    uint8_t  id;
    uint16_t to;

    delay_ms(10);   /* 上电等 ~10ms */

    /* 1. 软复位: DEVICE_RESET=1. 等 100ms 让芯片稳定 (陀螺启动瞬态). */
    if (mpu_write_reg(REG_PWR_MGMT_1, 0x80) != 0) return 1;
    delay_ms(100);

    /* 2. 唤醒 + 选时钟: SLEEP=0, CLKSEL=1 (PLL with X gyro ref, 比内部 RC 稳). */
    if (mpu_write_reg(REG_PWR_MGMT_1, 0x01) != 0) return 1;
    delay_ms(10);

    /* 3. WHO_AM_I 自检: 标准 MPU6050 返回 0x68, 实测模块为 MPU6500(0x70).
     * 两者基础原始数据接口兼容, 按实际 ID 做后续差异配置. */
    mpu6050_device_id = 0;
    to = 0x00FF;
    do {
        id = 0;
        if (mpu_read_reg(REG_WHO_AM_I, &id) == 0) {
            mpu6050_device_id = id;
            if (mpu_device_id_supported(id)) break;
        }
        delay_ms(10);
    } while (--to);
    if (!mpu_device_id_supported(id)) return 1;

    /* 4. 配置:
     *    DLPF_CFG=3 -> 陀螺内部采样 1kHz, LPF ~44Hz (DLPF=0 时 8kHz, 采样率公式不同)
     *    SMPLRT_DIV=4 -> 输出 1kHz/(1+4) = 200Hz, 匹配 attitude 采样率
     *    GYRO_CONFIG FS_SEL=3  -> ±2000dps (1°/s=16.4 LSB)
     *    ACCEL_CONFIG AFS_SEL=2 -> ±8g     (1g=4096 LSB) */
    if (mpu_write_reg(REG_CONFIG,       0x03) != 0) return 1; /* gyro DLPF_CFG=3 */
    if (mpu_write_reg(REG_SMPLRT_DIV,   4)    != 0) return 1; /* 200Hz */
    if (mpu_write_reg(REG_GYRO_CONFIG,  0x18) != 0) return 1; /* ±2000dps */
    if (mpu_write_reg(REG_ACCEL_CONFIG, 0x10) != 0) return 1; /* ±8g */

    /* MPU6500 的加速度计 DLPF 独立位于 0x1D; 值 3 对应约 41Hz 带宽.
     * MPU6050 的 0x1D 是另一功能寄存器, 因此不能无条件写入. */
    if (id == MPU6500_WHO_AM_I_VAL) {
        if (mpu_write_reg(REG_ACCEL_CONFIG2, 0x03) != 0) return 1;
    }

    return 0;
}

void mpu6050_get_acc(void)
{
    uint8_t d[6];
    if (mpu_read_regs(REG_ACCEL_XOUT_H, d, 6) != 0) return;   /* 大端: d[0]=XH, d[1]=XL */
    mpu6050_acc_x = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    mpu6050_acc_y = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    mpu6050_acc_z = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
}

void mpu6050_get_gyro(void)
{
    uint8_t d[6];
    if (mpu_read_regs(REG_GYRO_XOUT_H, d, 6) != 0) return;    /* 大端: d[0]=XH, d[1]=XL */
    mpu6050_gyro_x = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    mpu6050_gyro_y = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    mpu6050_gyro_z = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
}
