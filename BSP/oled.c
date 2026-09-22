/*
 * oled.c - SSD1306 128x64 driver, software I2C, framebuffer.
 * Pins from syscfg group GPIO_OLED: PIN_OLED_SCL (PA1), PIN_OLED_SDA (PA0).
 */
#include "ti_msp_dl_config.h"
#include "oled.h"
#include "delay.h"
#include "oledfont.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_PAGES    (OLED_HEIGHT / 8)
#define OLED_BUF_SZ   (OLED_WIDTH * OLED_PAGES)   /* 1024 */

#define OLED_I2C_ADDR 0x78

/* software I2C pin ops (push-pull, ACK not checked - OLED always ACKs) */
#define SCL_H()  DL_GPIO_setPins  (GPIO_OLED_PORT, GPIO_OLED_PIN_OLED_SCL_PIN)
#define SCL_L()  DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_PIN_OLED_SCL_PIN)
#define SDA_H()  DL_GPIO_setPins  (GPIO_OLED_PORT, GPIO_OLED_PIN_OLED_SDA_PIN)
#define SDA_L()  DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_PIN_OLED_SDA_PIN)

/* I2C bit-bang delay. Empty = fast (proven on this board @80MHz).
 * If the display is unstable, define e.g. delay_cycles(CPUCLK_FREQ/1000000). */
#define OLED_I2C_DLY()  delay_cycles(CPUCLK_FREQ/1000000)

static uint8_t g_fb[OLED_BUF_SZ];

/* ---------------- software I2C ---------------- */
static void I2C_Start(void)
{
    SDA_H(); OLED_I2C_DLY();
    SCL_H(); OLED_I2C_DLY();
    SDA_L(); OLED_I2C_DLY();
    SCL_L(); OLED_I2C_DLY();
}

static void I2C_Stop(void)
{
    SDA_L(); OLED_I2C_DLY();
    SCL_H(); OLED_I2C_DLY();
    SDA_H(); OLED_I2C_DLY();
}

static void I2C_WaitAck(void)   /* OLED always ACKs; not actually read */
{
    SDA_H(); OLED_I2C_DLY();
    SCL_H(); OLED_I2C_DLY();
    SCL_L(); OLED_I2C_DLY();
}

static void I2C_SendByte(uint8_t dat)
{
    for (uint8_t i = 0; i < 8; i++) {
        SCL_L(); OLED_I2C_DLY();
        if (dat & 0x80) SDA_H(); else SDA_L();
        OLED_I2C_DLY();
        SCL_H(); OLED_I2C_DLY();
        dat <<= 1;
    }
    SCL_L(); OLED_I2C_DLY();
}

/* ---------------- SSD1306 ---------------- */
static void OLED_WriteCmd(uint8_t cmd)
{
    I2C_Start();
    I2C_SendByte(OLED_I2C_ADDR); I2C_WaitAck();
    I2C_SendByte(0x00);          I2C_WaitAck();   /* D/C=0: command */
    I2C_SendByte(cmd);           I2C_WaitAck();
    I2C_Stop();
}

void OLED_Init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,           /* display off */
        0x00, 0x10,     /* column start low/high */
        0x40,           /* start line 0 */
        0x81, 0xCF,     /* contrast */
        0xA1,           /* segment remap */
        0xC8,           /* COM scan direction */
        0xA6,           /* normal display (not inverted) */
        0xA8, 0x3F,     /* multiplex 1/64 */
        0xD3, 0x00,     /* display offset 0 */
        0xD5, 0x80,     /* clock divide / osc */
        0xD9, 0xF1,     /* pre-charge period */
        0xDA, 0x12,     /* COM pins */
        0xDB, 0x40,     /* VCOMH deselect */
        0x20, 0x00,     /* addressing mode: horizontal (for full-frame flush) */
        0x8D, 0x14,     /* charge pump enable */
        0xA4,           /* display follows GDDRAM */
        0xA6,           /* normal */
        0xAF,           /* display on */
    };
    delay_ms(200);
    for (uint16_t i = 0; i < sizeof(init_cmds); i++)
        OLED_WriteCmd(init_cmds[i]);
    OLED_Clear();
    OLED_Update();
}

void OLED_Clear(void)
{
    memset(g_fb, 0, sizeof(g_fb));
}

void OLED_Update(void)
{
    /* column address range 0..127 */
    OLED_WriteCmd(0x21); OLED_WriteCmd(0x00); OLED_WriteCmd(0x7F);
    /* page address range 0..7 */
    OLED_WriteCmd(0x22); OLED_WriteCmd(0x00); OLED_WriteCmd(0x07);

    I2C_Start();
    I2C_SendByte(OLED_I2C_ADDR); I2C_WaitAck();
    I2C_SendByte(0x40);          I2C_WaitAck();   /* D/C=1: data stream */
    for (uint16_t i = 0; i < OLED_BUF_SZ; i++) {
        I2C_SendByte(g_fb[i]);
        I2C_WaitAck();
    }
    I2C_Stop();
}

void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint16_t idx = (uint16_t)(y >> 3) * OLED_WIDTH + x;
    if (on) g_fb[idx] |=  (uint8_t)(1u << (y & 7));
    else    g_fb[idx] &= (uint8_t)~(1u << (y & 7));
}

void OLED_ShowChar(uint8_t x, uint8_t y, char c, uint8_t size)
{
    if (c < ' ' || c > '~') c = ' ';
    uint8_t idx = (uint8_t)(c - ' ');

    if (size == OLED_FONT_16) {
        const unsigned char *g = asc2_1608[idx];          /* covers ' ' .. '~' */
        uint8_t page = (uint8_t)(y >> 3);
        for (uint8_t i = 0; i < 8; i++) {
            if (x + i < OLED_WIDTH) {
                g_fb[(uint16_t)page * OLED_WIDTH + x + i] = g[i];
                if (page + 1 < OLED_PAGES)
                    g_fb[(uint16_t)(page + 1) * OLED_WIDTH + x + i] = g[8 + i];
            }
        }
    } else {                                               /* OLED_FONT_8 */
        if (idx >= sizeof(asc2_0806) / sizeof(asc2_0806[0])) idx = 0;  /* '{|}~ absent */
        const unsigned char *g = asc2_0806[idx];
        uint8_t page = (uint8_t)(y >> 3);
        for (uint8_t i = 0; i < 6; i++) {
            if (x + i < OLED_WIDTH)
                g_fb[(uint16_t)page * OLED_WIDTH + x + i] = g[i];
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *s, uint8_t size)
{
    uint8_t w = (size == OLED_FONT_16) ? 8 : 6;
    while (*s && x + w <= OLED_WIDTH) {
        OLED_ShowChar(x, y, *s, size);
        x += w;
        s++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    char buf[11];
    if (len > 10) len = 10;
    for (int8_t i = (int8_t)len - 1; i >= 0; i--) {
        buf[i] = (char)('0' + (num % 10));
        num /= 10;
    }
    buf[len] = '\0';
    OLED_ShowString(x, y, buf, size);
}

void OLED_Printf(uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...)
{
    char buf[40];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    OLED_ShowString(x, y, buf, size);
}

/*
 * OLED_PrintfRC - 基于行列号的格式化打印 (OLED_Printf 的便捷封装).
 *
 * 用字符行/列号代替像素坐标, 字号固定 8x16 (OLED_FONT_16).
 * 该字号下整屏为 16 列 × 4 行:
 *   row : 0..3   (每行高 16 像素)
 *   col : 0..15  (每列宽 8 像素)
 * 超出范围的 row/col 会被夹紧到最近的有效值, 不会越界写显存.
 *
 * 参数:
 *   row - 行号 0..3
 *   col - 列号 0..15
 *   fmt - printf 风格格式串, 可变参数同 printf
 *
 * 示例:
 *   OLED_PrintfRC(0, 0, "Hello");
 *   OLED_PrintfRC(1, 0, "n=%d", n);
 *   OLED_PrintfRC(2, 4, "v=%.2f", v);
 *
 * 注意: 内部缓冲区 40 字节 (同 OLED_Printf), 超长输出会被截断.
 */
void OLED_PrintfRC(uint8_t row, uint8_t col, const char *fmt, ...)
{
    if (row > 3)  row = 3;
    if (col > 15) col = 15;

    uint8_t x = (uint8_t)(col * 8u);     /* 每列 8 像素 */
    uint8_t y = (uint8_t)(row * 16u);    /* 每行 16 像素 */

    char buf[40];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    OLED_ShowString(x, y, buf, OLED_FONT_16);
}
