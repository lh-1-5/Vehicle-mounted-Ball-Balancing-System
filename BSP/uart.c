/* uart.c - 双串口收发 + 8 字节帧协议解析.
 * 两路 UART 收发逻辑共用, 通过 uart_channel_t 参数化差异. */
#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "ball.h"


/* 帧头 */
#define UART_FRAME_HEAD1  0xA5
#define UART_FRAME_HEAD2  0x5A

/*
 * UART STAT.BUSY 同时受 TX 和 RX 影响。摄像头 RX 线上持续有数据或低电平时，
 * DriverLib 的 transmitDataBlocking() 会一直等待 BUSY 清零并拖死主循环。
 * 发送时只等待 TX 缓冲可写，并设置超时，既避免丢字节也避免永久阻塞。
 */
#define UART_TX_WAIT_MAX  100000U

static uint8_t uart_write_byte(UART_Regs *inst, uint8_t data)
{
    uint32_t timeout = UART_TX_WAIT_MAX;

    while (DL_UART_Main_isTXFIFOFull(inst)) {
        if (--timeout == 0U)
            return 0U;
    }

    DL_UART_Main_transmitData(inst, data);
    return 1U;
}

/* ---- 通道表: 两路 UART 差异集中于此 ---- */
typedef struct {
    UART_Regs      *inst;    /* UART 实例 */
    IRQn_Type       irqn;    /* NVIC 中断号 */
    uart_rx_frame_t rx;      /* 接收状态机 */
} uart_channel_t;

static uart_channel_t g_camera      = { UART_Cam_INST,         UART_Cam_INST_INT_IRQN,         {0} };
static uart_channel_t g_communicate = { UART_Communicate_INST, UART_Communicate_INST_INT_IRQN, {0} };

volatile uint8_t test_uart_param;
volatile int g_ball_count;

/* ---- 校验和: CMD + 4 字节数据, 取低 8 位 ---- */
static uint8_t uart_checksum(uint8_t cmd, const uint8_t *data4)
{
    uint16_t sum = (uint16_t)cmd
                 + (uint16_t)data4[0] + (uint16_t)data4[1]
                 + (uint16_t)data4[2] + (uint16_t)data4[3];
    return (uint8_t)(sum & 0xFF);
}

/* ---- 发送一帧 (共用) ---- */
static void uart_send_frame(UART_Regs *inst, uint8_t cmd, const uint8_t *data4)
{
    uint8_t tx[UART_FRAME_LEN];
    tx[0] = UART_FRAME_HEAD1;
    tx[1] = UART_FRAME_HEAD2;
    tx[2] = cmd;
    tx[3] = data4[0];
    tx[4] = data4[1];
    tx[5] = data4[2];
    tx[6] = data4[3];
    tx[7] = uart_checksum(cmd, data4);

    for (uint8_t i = 0; i < UART_FRAME_LEN; i++) {
        if (!uart_write_byte(inst, tx[i]))
            break;
    }
}

void uart_camera_send_frame(uint8_t cmd, const uint8_t *data4)
{
    uart_send_frame(g_camera.inst, cmd, data4);
}

void uart_communicate_send_frame(uint8_t cmd, const uint8_t *data4)
{
    uart_send_frame(g_communicate.inst, cmd, data4);
}

/* ---- 单字节解析状态机 (共用) ---- */
static void uart_parse_byte(uart_channel_t *ch, uint8_t b)
{
    switch (ch->rx.state) {
    case 0:     /* 等帧头 0xA5 */
        if (b == UART_FRAME_HEAD1) {
            ch->rx.buf[0] = UART_FRAME_HEAD1;
            ch->rx.state = 1;
        }
        break;

    case 1:     /* 等帧头 0x5A */
        if (b == UART_FRAME_HEAD2) {
            ch->rx.buf[1] = UART_FRAME_HEAD2;
            ch->rx.state = 2;
            ch->rx.idx = 2;     /* 数据从 buf[2] 开始存 */
        } else if (b == UART_FRAME_HEAD1) {
            /* 又是 A5: 新帧头, 不死锁 */
            ch->rx.buf[0] = UART_FRAME_HEAD1;
            ch->rx.state = 1;
        } else {
            ch->rx.state = 0;
        }
        break;

    case 2:     /* 收 CMD + 数据 + 校验 */
        if (ch->rx.idx < UART_FRAME_LEN)
            ch->rx.buf[ch->rx.idx++] = b;

        if (ch->rx.idx >= UART_FRAME_LEN) {
            /* 收满一帧, 校验通过则分发 */
            uint8_t cmd = ch->rx.buf[2];
            const uint8_t *dat = &ch->rx.buf[3];
            if (uart_checksum(cmd, dat) == ch->rx.buf[7]) {
                uart_channel_id_t ch_id = (ch == &g_camera)
                    ? UART_CH_CAMERA : UART_CH_COMMUNICATE;
                uart_data_handler(ch_id, ch->rx.buf);
            }
            ch->rx.state = 0;   /* 复位, 等下一帧 */
        }
        break;

    default:
        ch->rx.state = 0;
        break;
    }
}

/* ---- 无线串口类 printf 调试输出 ----
 * 裸文本 (非协议帧), 供上位机串口助手直接查看.
 * 注意: 与协议帧共用同一串口, 混发会让对端解析乱; 调试期用 printf,
 *       比赛期用 uart_communicate_send_frame. 仅限主循环调用, 勿在中断里用. */
void uart_communicate_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    for (uint8_t i = 0; buf[i] != '\0'; i++) {
        if (!uart_write_byte(g_communicate.inst, (uint8_t)buf[i]))
            break;
    }
}

/* ---- 初始化 (共用) ---- */
static void uart_channel_init(uart_channel_t *ch)
{
    memset(&ch->rx, 0, sizeof(ch->rx));
    NVIC_ClearPendingIRQ(ch->irqn);
    NVIC_EnableIRQ(ch->irqn);
}

void uart_camera_init(void)
{
    uart_channel_init(&g_camera);
}

void uart_communicate_init(void)
{
    uart_channel_init(&g_communicate);
}

/* ---- 中断服务 ---- */
void UART_Cam_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_Cam_INST) == DL_UART_MAIN_IIDX_RX)
        uart_parse_byte(&g_camera, DL_UART_Main_receiveData(UART_Cam_INST));
}

void UART_Communicate_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_Communicate_INST) == DL_UART_MAIN_IIDX_RX)
        uart_parse_byte(&g_communicate, DL_UART_Main_receiveData(UART_Communicate_INST));
}

/* ---- 帧处理: CMD 分发 ---- */
void uart_data_handler(uart_channel_id_t ch, const uint8_t *frame)
{
    uint8_t cmd = frame[2];
    /* const uint8_t *data = &frame[3]; */   /* DATA0..3, 使用时解除注释 */
    (void)ch;   /* 暂未用, 防未用变量警告 */
    (void)cmd;

    switch (cmd) 
    {
        case 0x01:
        if (ch == UART_CH_CAMERA) 
        {
            // 摄像头发来的 0x01
            test_uart_param = frame[3];
        } 
        else 
        {
            // 无线串口发来的 0x01
            test_uart_param = frame[3];
        }
        break;
        case 0x31:
        if (ch == UART_CH_CAMERA) 
        {
            // 摄像头发来的 int 类型球计数 (DATA0..3, 小端)
            g_ball_count = (int)((uint32_t)frame[3]
                               | ((uint32_t)frame[4] << 8)
                               | ((uint32_t)frame[5] << 16)
                               | ((uint32_t)frame[6] << 24));
        }
        else 
        {
            // 无线串口发来的
            
        }
        break;
        case 0x21:
        if (ch == UART_CH_CAMERA) 
        {
            /* 摄像头球位置(int16, DATA0..1小端). ball模块在新D方案下同时
             * 记录帧序号和时间戳; 关闭实验宏时仍等价于原来的直接赋值. */
            int16_t position = (int16_t)((uint32_t)frame[3]
                                       | ((uint32_t)frame[4] << 8));
            ball_measurement_update(position);
        }
        else 
        {
            // 无线串口发来的
            
        }
        break;

    default:
        break;
    }
}
