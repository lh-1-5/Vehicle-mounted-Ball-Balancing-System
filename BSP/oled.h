#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/*
 * SSD1306 128x64 OLED driver, software-I2C, with framebuffer.
 *
 * Coordinate convention: x in pixels (0..127), y in pixels (0..63).
 * For text functions y MUST be a multiple of 8 (page-aligned).
 *
 * Usage: draw into the buffer with Show* / SetPixel, then call OLED_Update()
 * to push the whole buffer to the screen. OLED_Update() is blocking (~ms),
 * so call it at display rate, not every tight loop iteration.
 */

#define OLED_FONT_8   8u    /* 6x8  : 21 cols x 8 rows */
#define OLED_FONT_16  16u   /* 8x16 : 16 cols x 4 rows */

/* init / screen */
void OLED_Init(void);
void OLED_Clear(void);       /* clear framebuffer (no flush) */
void OLED_Update(void);      /* framebuffer -> screen (blocking) */

/* pixel */
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);

/* text (y must be a multiple of 8) */
void OLED_ShowChar  (uint8_t x, uint8_t y, char c, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *s, uint8_t size);
void OLED_ShowNum   (uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_Printf    (uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...);
/* OLED_Printf 的行列封装: 固定 8x16 字号, 16 列 × 4 行 */
void OLED_PrintfRC  (uint8_t row, uint8_t col, const char *fmt, ...);

#endif /* __OLED_H */
