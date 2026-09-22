/*
 * imu660rb.c - IMU660RB (LSM6DSR) 6 轴 IMU 驱动, 硬件 I2C.
 * 移植自逐飞 zf_device_imu660rb (TC264 的 SPI/软IIC 版), 改为 MSPM0 硬件 I2C.
 *
 * 关键移植改动:
 *   1. 底层换成 MSPM0 DL_I2C API (syscfg 实例 I2C_IMU).
 *   2. CTRL3_C = 0x44 (沿用参考值, I2C 也用这个): bit2=IF_INC(地址自增, 突发
 *      读 6 字节必需), bit6=BDU. 之前误以为 bit2 是 I2C_DISABLE 而改成 0x40,
 *      结果 IF_INC=0, 突发读返回同一字节 -> 三轴读数全是 byte*257 且相同.
 *      实测 0x40 后 I2C 仍通信正常, 证明 bit6 不是 I2C_DISABLE, 0x44 对 I2C 安全.
 *   3. 超时用循环计数 (今年还没搭 1ms 时基), 自包含, 不依赖 clock.c.
 *   4. 读寄存器用 repeated-start (DL_I2C_enableControllerReadOnTXEmpty):
 *      写完寄存器地址后不发 STOP, 直接转入读, 符合 I2C 传感器读时序.
 */
#include "ti_msp_dl_config.h"
#include "imu660rb.h"
#include "delay.h"
#include <stdbool.h>

/* ---- LSM6DSR 寄存器地址 ---- */
#define REG_FUNC_CFG_ACCESS  0x01   /* HUB 寄存器访问控制 */
#define REG_WHO_AM_I         0x0F   /* 器件 ID (期望 0x6B) */
#define REG_CTRL1_XL         0x10   /* 加速度计 ODR / 量程 */
#define REG_CTRL2_G          0x11   /* 陀螺仪 ODR / 量程 */
#define REG_CTRL3_C          0x12   /* IF_INC / BDU / I2C_DISABLE / SW_RESET */
#define REG_CTRL4_C          0x13
#define REG_CTRL5_C          0x14
#define REG_CTRL6_C          0x15
#define REG_CTRL7_G          0x16
#define REG_CTRL9_XL         0x18   /* I3C 开关 */
#define REG_OUTX_L_G         0x22   /* 陀螺仪 X 低字节 (IF_INC 自增读 6 字节) */
#define REG_OUTX_L_A         0x28   /* 加速度计 X 低字节 (IF_INC 自增读 6 字节) */

/* ---- 量程配置 (与 .h 的 transition 换算系数一一对应) ----
 * ODR=833Hz (高4位=0111): 配合姿态解算 200Hz 采样, 保证每帧读到新数据, 不重复.
 * FS 不变: ACC ±8G(1g≈4096), GYRO ±2000dps(/14.3). 换算系数未变.
 */
#define IMU660RB_ACC_RANGE_REG    0x7C   /* ±8G, ODR 833Hz (原 0x3C=104Hz) */
#define IMU660RB_GYRO_RANGE_REG   0x7C   /* ±2000dps, ODR 833Hz (原 0x5C=104Hz) */

/* ---- 全局数据 ---- */
int16_t imu660rb_acc_x = 0, imu660rb_acc_y = 0, imu660rb_acc_z = 0;
int16_t imu660rb_gyro_x = 0, imu660rb_gyro_y = 0, imu660rb_gyro_z = 0;

/* ---- I2C 底层 ---- */
#define I2C_TIMEOUT_LOOP  200000u   /* 轮询超时上限 (循环次数, 约几 ms) */

/* 等控制器回到空闲. true=OK, false=超时 */
static bool i2c_wait_idle(void)
{
    uint32_t to = I2C_TIMEOUT_LOOP;
    while (!(DL_I2C_getControllerStatus(I2C_IMU_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--to == 0) return false;
    }
    return true;
}

/* 写一个寄存器. 0=成功, -1=失败 */
static int imu_write_reg(uint8_t reg, uint8_t data)
{
    DL_I2C_transmitControllerData(I2C_IMU_INST, reg);   /* 寄存器地址先入 TX FIFO */
    DL_I2C_clearInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    if (!i2c_wait_idle()) return -1;

    /* 共发 2 字节: reg (已在 FIFO) + data */
    DL_I2C_startControllerTransfer(I2C_IMU_INST, IMU660RB_DEV_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    DL_I2C_fillControllerTXFIFO(I2C_IMU_INST, &data, 1);

    uint32_t to = I2C_TIMEOUT_LOOP;
    while (!DL_I2C_getRawInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        if (--to == 0) return -1;
    }
    return 0;
}

/*
 * 突发读 len 个寄存器 (依赖 CTRL3_C 的 IF_INC=1, 地址自增).
 * 时序: START -> addr+W -> reg -> (无 STOP) repeated-START -> addr+R -> data... -> STOP.
 * 0=成功, -1=失败.
 */
static int imu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;

    DL_I2C_flushControllerRXFIFO(I2C_IMU_INST);                 /* 清残留, 防 Z 轴卡死 */
    DL_I2C_transmitControllerData(I2C_IMU_INST, reg);            /* 寄存器地址入 TX FIFO */
    I2C_IMU_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;  /* 直接写 MCTR(非OR), 同去年 MPU6050 proven 写法 */
    DL_I2C_clearInterruptStatus(I2C_IMU_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    if (!i2c_wait_idle()) {
        DL_I2C_disableControllerReadOnTXEmpty(I2C_IMU_INST);
        return -1;
    }

    DL_I2C_startControllerTransfer(I2C_IMU_INST, IMU660RB_DEV_ADDR,
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
    /* 收尾: 把 FIFO 里剩余字节读完, 关 repeated-start, 清 TX FIFO */
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
        if (i < len) buf[i++] = (uint8_t)DL_I2C_receiveControllerData(I2C_IMU_INST);
    }
    I2C_IMU_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);

    return (i == len) ? 0 : -1;
}

/* 读一个寄存器 (读失败返回 0) */
static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t v = 0;
    imu_read_regs(reg, &v, 1);
    return v;
}

/* ---- 对外 API ---- */
uint8_t imu660rb_init(void)
{
    uint8_t  id;
    uint16_t to;

    delay_ms(10);          /* 上电等 ~10ms */

    /* 软复位: 先关 HUB 访问, 再置 SW_RESET.
     * 等 200ms (非原来的 2ms): 复位后陀螺有启动瞬态, 立即标定会采到失真零偏,
     * 导致 yaw 长期漂移. 200ms 让传感器/滤波器稳定后再标定. */
    imu_write_reg(REG_FUNC_CFG_ACCESS, 0x00);
    imu_write_reg(REG_CTRL3_C, 0x01);         /* SW_RESET */
    delay_ms(200);            /* ~200ms 等复位+稳定 */
    imu_write_reg(REG_FUNC_CFG_ACCESS, 0x00);

    /* WHO_AM_I 自检 (期望 0x6B), 最多重试 255 次 */
    to = 0x00FF;
    do {
        id = imu_read_reg(REG_WHO_AM_I);
        if (id == IMU660RB_WHO_AM_I_VAL) break;
        delay_ms(10);      /* ~10ms */
    } while (--to);
    if (id != IMU660RB_WHO_AM_I_VAL) return 1;

    /* 配置量程与滤波 (沿用参考工程的 proven 值, 仅 CTRL3_C 改为 I2C 安全值) */
    imu_write_reg(REG_CTRL1_XL, IMU660RB_ACC_RANGE_REG);   /* 加速度计 ±8G */
    imu_write_reg(REG_CTRL2_G,  IMU660RB_GYRO_RANGE_REG);  /* 陀螺仪 ±2000dps */
    imu_write_reg(REG_CTRL3_C,  0x44);   /* IF_INC(bit2)+BDU(bit6); 突发读必需, 不可用 0x40 */
    imu_write_reg(REG_CTRL4_C,  0x02);   /* 陀螺仪 LPF1 使能 */
    imu_write_reg(REG_CTRL5_C,  0x00);
    imu_write_reg(REG_CTRL6_C,  0x00);
    imu_write_reg(REG_CTRL7_G,  0x00);
    imu_write_reg(REG_CTRL9_XL, 0x01);   /* 关 I3C */
    return 0;
}

void imu660rb_get_acc(void)
{
    uint8_t d[6];
    if (imu_read_regs(REG_OUTX_L_A, d, 6) != 0) return;   /* 小端: d[0]=XL, d[1]=XH */
    imu660rb_acc_x = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
    imu660rb_acc_y = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
    imu660rb_acc_z = (int16_t)(((uint16_t)d[5] << 8) | d[4]);
}

void imu660rb_get_gyro(void)
{
    uint8_t d[6];
    if (imu_read_regs(REG_OUTX_L_G, d, 6) != 0) return;
    imu660rb_gyro_x = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
    imu660rb_gyro_y = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
    imu660rb_gyro_z = (int16_t)(((uint16_t)d[5] << 8) | d[4]);
}
