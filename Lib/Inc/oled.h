/**
  ******************************************************************************
  * @file    oled.h
  * @brief   SSD1306 OLED display driver via software SPI (128x64)
  *          Pin mapping: PB3=DC, PB4=RES, PB5=SDA, PC14=SCL
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "app_config.h"

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

typedef enum {
    OLED_COLOR_BLACK = 0,
    OLED_COLOR_WHITE = 1
} OLED_Color;

#if APP_USE_OLED

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(OLED_Color color);
void OLED_Refresh(void);

void OLED_SetPixel(uint8_t x, uint8_t y, OLED_Color color);
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, OLED_Color color);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color color);
void OLED_DrawFilledRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color color);
void OLED_DrawCircle(uint8_t cx, uint8_t cy, uint8_t r, OLED_Color color);

void OLED_PutChar(uint8_t x, uint8_t y, char ch, OLED_Color color);
void OLED_PutString(uint8_t x, uint8_t y, const char *str, OLED_Color color);
void OLED_Printf(uint8_t x, uint8_t y, OLED_Color color, const char *fmt, ...);

#else /* APP_USE_OLED == 0: 屏幕已拆除, 所有调用退化为空操作 */

static inline void OLED_Init(void) {}
static inline void OLED_Clear(void) {}
static inline void OLED_Fill(OLED_Color color) { (void)color; }
static inline void OLED_Refresh(void) {}
static inline void OLED_SetPixel(uint8_t x, uint8_t y, OLED_Color color)
{ (void)x; (void)y; (void)color; }
static inline void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, OLED_Color color)
{ (void)x0; (void)y0; (void)x1; (void)y1; (void)color; }
static inline void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color color)
{ (void)x; (void)y; (void)w; (void)h; (void)color; }
static inline void OLED_DrawFilledRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color color)
{ (void)x; (void)y; (void)w; (void)h; (void)color; }
static inline void OLED_DrawCircle(uint8_t cx, uint8_t cy, uint8_t r, OLED_Color color)
{ (void)cx; (void)cy; (void)r; (void)color; }
static inline void OLED_PutChar(uint8_t x, uint8_t y, char ch, OLED_Color color)
{ (void)x; (void)y; (void)ch; (void)color; }
static inline void OLED_PutString(uint8_t x, uint8_t y, const char *str, OLED_Color color)
{ (void)x; (void)y; (void)str; (void)color; }
static inline void OLED_Printf(uint8_t x, uint8_t y, OLED_Color color, const char *fmt, ...)
{ (void)x; (void)y; (void)color; (void)fmt; }

#endif /* APP_USE_OLED */

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
