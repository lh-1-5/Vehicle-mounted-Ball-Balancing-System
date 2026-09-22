#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ======================== 串口协议模块 ========================
 * 两路 UART 共用 8 字节定长帧协议:
 *   帧格式: A5 5A CMD D0 D1 D2 D3 SUM
 *   校验:   SUM = (CMD + D0 + D1 + D2 + D3) & 0xFF  (不含帧头)
 *   字节序: 多字节整数小端序 (低字节先发)
 *   两路串口共用帧格式, 数据含义由 CMD 字段区分
 *
 * 通道:
 *   uart_camera       -> UART_Cam         (UART1, PA17 TX / PA9 RX)   摄像头
 *   uart_communicate  -> UART_Communicate (UART2, PB17 TX / PB18 RX)  无线通信
 */

#define UART_FRAME_LEN  8   /* 帧长度: 2 帧头 + 1 CMD + 4 数据 + 1 校验 */

/* 单路接收状态机 */
typedef struct {
    uint8_t buf[UART_FRAME_LEN];   /* 接收缓存 */
    uint8_t idx;                   /* 当前接收索引 */
    uint8_t state;                 /* 0:等A5 1:等5A 2:收数据+校验 */
} uart_rx_frame_t;

/* ---- 摄像头串口 (UART_Cam) ---- */
void uart_camera_init(void);                                    /* 使能 RX 中断 */
void uart_camera_send_frame(uint8_t cmd, const uint8_t *data4); /* 发一帧 */

/* ---- 无线通信串口 (UART_Communicate) ---- */
void uart_communicate_init(void);                                    /* 使能 RX 中断 */
void uart_communicate_send_frame(uint8_t cmd, const uint8_t *data4); /* 发一帧 */
void uart_communicate_printf(const char *fmt, ...);                  /* 类printf调试输出(裸文本) */

/* ---- 通道标识 ---- */
typedef enum {
    UART_CH_CAMERA      = 0,   /* UART1, 摄像头 */
    UART_CH_COMMUNICATE = 1,   /* UART2, 无线通信 */
} uart_channel_id_t;

/* ---- 帧处理 (用户实现 CMD 分发) ----
 * 校验通过的帧由此回调处理. 在 uart.c 内有空实现, 需要时在此加 case.
 *   ch:    来源通道 (UART_CH_CAMERA / UART_CH_COMMUNICATE)
 *   frame[2] = CMD, frame[3..6] = DATA0..3, frame[7] = SUM */
void uart_data_handler(uart_channel_id_t ch, const uint8_t *frame);

/* ---- extern外部变量使用 ---- */
/* 此值由 UART RX 中断写入，主循环/菜单读取。 */
extern volatile uint8_t test_uart_param;
extern volatile int g_ball_count;   /* 摄像头 0x31 帧: 球计数 (DATA0~3, 小端) */

#endif /* UART_H */
