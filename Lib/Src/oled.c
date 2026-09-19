/**
  ******************************************************************************
  * @file    oled.c
  * @brief   SSD1306 OLED 128x64 driver via software SPI
  *          PB3=DC   PB4=RES  PB5=SDA  PC14=SCL
  ******************************************************************************
  */

#include "oled.h"
#include "app_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if APP_USE_OLED

/* ======================== 引脚宏 ======================== */
#define OLED_SCL_PORT   GPIOC
#define OLED_SCL_PIN    GPIO_PIN_14
#define OLED_SDA_PORT   GPIOB
#define OLED_SDA_PIN    GPIO_PIN_5
#define OLED_RES_PORT   GPIOB
#define OLED_RES_PIN    GPIO_PIN_4
#define OLED_DC_PORT    GPIOB
#define OLED_DC_PIN     GPIO_PIN_3

#define OLED_SCL_H()    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_L()    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SDA_H()    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_L()    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define OLED_RES_H()    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_SET)
#define OLED_RES_L()    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_RESET)
#define OLED_DC_H()     HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_DC_L()     HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)

/* ======================== 帧缓冲区 ======================== */
static uint8_t OLED_Buffer[OLED_WIDTH * OLED_PAGES];

/* ======================== 6x8 ASCII 字库 ======================== */
static const uint8_t Font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* space */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* backslash */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /*  */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x08,0x04,0x08,0x10,0x08,0x00}, /* ~ */
};

/* ======================== 底层：软件 SPI 写一个字节 ======================== */
static void OLED_SPI_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80)
            OLED_SDA_H();
        else
            OLED_SDA_L();
        OLED_SCL_H();
        byte <<= 1;
        OLED_SCL_L();
    }
}

/* ======================== 写命令 / 数据 ======================== */
static void OLED_WriteCmd(uint8_t cmd)
{
    OLED_DC_L();                    /* DC=0: 命令 */
    OLED_SPI_WriteByte(cmd);
    OLED_DC_H();                    /* 恢复 */
}

static void OLED_WriteData(uint8_t data)
{
    OLED_DC_H();                    /* DC=1: 数据 */
    OLED_SPI_WriteByte(data);
}

/* ======================== 初始化 ======================== */
void OLED_Init(void)
{
    /* 硬件复位 */
    OLED_RES_L();
    HAL_Delay(10);
    OLED_RES_H();
    HAL_Delay(10);

    /* SSD1306 初始化序列 */
    OLED_WriteCmd(0xAE); /* display off */

    OLED_WriteCmd(0xD5); /* set display clock divide */
    OLED_WriteCmd(0x80);

    OLED_WriteCmd(0xA8); /* set multiplex */
    OLED_WriteCmd(0x3F); /* 64 */

    OLED_WriteCmd(0xD3); /* set display offset */
    OLED_WriteCmd(0x00);

    OLED_WriteCmd(0x40); /* set start line */

    OLED_WriteCmd(0x8D); /* charge pump */
    OLED_WriteCmd(0x14); /* enable */

    OLED_WriteCmd(0x20); /* memory addressing mode */
    OLED_WriteCmd(0x00); /* horizontal */

    OLED_WriteCmd(0xA1); /* segment remap (col 127 = SEG0) */
    OLED_WriteCmd(0xC8); /* COM output scan direction (remapped) */

    OLED_WriteCmd(0xDA); /* COM pins hardware config */
    OLED_WriteCmd(0x12);

    OLED_WriteCmd(0x81); /* contrast */
    OLED_WriteCmd(0xCF);

    OLED_WriteCmd(0xD9); /* pre-charge period */
    OLED_WriteCmd(0xF1);

    OLED_WriteCmd(0xDB); /* VCOMH deselect level */
    OLED_WriteCmd(0x40);

    OLED_WriteCmd(0xA4); /* entire display on: follow RAM */
    OLED_WriteCmd(0xA6); /* normal (not inverted) */

    OLED_WriteCmd(0xAF); /* display on */

    OLED_Clear();
    OLED_Refresh();
}

/* ======================== 清屏 / 填充 ======================== */
void OLED_Clear(void)
{
    memset(OLED_Buffer, 0x00, sizeof(OLED_Buffer));
}

void OLED_Fill(OLED_Color color)
{
    memset(OLED_Buffer, (color == OLED_COLOR_WHITE) ? 0xFF : 0x00,
           sizeof(OLED_Buffer));
}

/* ======================== 刷新显示 ======================== */
void OLED_Refresh(void)
{
    OLED_WriteCmd(0x21); /* set column address */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x7F);

    OLED_WriteCmd(0x22); /* set page address */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x07);

    for (uint16_t i = 0; i < sizeof(OLED_Buffer); i++) {
        OLED_WriteData(OLED_Buffer[i]);
    }
}

/* ======================== 像素操作 ======================== */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_Color color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint16_t idx = x + (y / 8) * OLED_WIDTH;
    if (color == OLED_COLOR_WHITE)
        OLED_Buffer[idx] |= (1 << (y % 8));
    else
        OLED_Buffer[idx] &= ~(1 << (y % 8));
}

/* ======================== 画线 (Bresenham) ======================== */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
                   OLED_Color color)
{
    int16_t dx = (x1 > x0) ? (int16_t)(x1 - x0) : -(int16_t)(x0 - x1);
    int16_t dy = (y1 > y0) ? (int16_t)(y1 - y0) : -(int16_t)(y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (1) {
        OLED_SetPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* ======================== 画矩形 ======================== */
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_Color color)
{
    OLED_DrawLine(x, y, x + w - 1, y, color);
    OLED_DrawLine(x, y + h - 1, x + w - 1, y + h - 1, color);
    OLED_DrawLine(x, y, x, y + h - 1, color);
    OLED_DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
}

void OLED_DrawFilledRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                         OLED_Color color)
{
    for (uint8_t i = 0; i < w; i++)
        for (uint8_t j = 0; j < h; j++)
            OLED_SetPixel(x + i, y + j, color);
}

/* ======================== 画圆 (Midpoint) ======================== */
void OLED_DrawCircle(uint8_t cx, uint8_t cy, uint8_t r, OLED_Color color)
{
    int16_t f = 1 - (int16_t)r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * (int16_t)r;
    int16_t x = 0;
    int16_t y = r;

    OLED_SetPixel(cx, cy + r, color);
    OLED_SetPixel(cx, cy - r, color);
    OLED_SetPixel(cx + r, cy, color);
    OLED_SetPixel(cx - r, cy, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        OLED_SetPixel(cx + x, cy + y, color);
        OLED_SetPixel(cx - x, cy + y, color);
        OLED_SetPixel(cx + x, cy - y, color);
        OLED_SetPixel(cx - x, cy - y, color);
        OLED_SetPixel(cx + y, cy + x, color);
        OLED_SetPixel(cx - y, cy + x, color);
        OLED_SetPixel(cx + y, cy - x, color);
        OLED_SetPixel(cx - y, cy - x, color);
    }
}

/* ======================== 绘制字符 ======================== */
void OLED_PutChar(uint8_t x, uint8_t y, char ch, OLED_Color color)
{
    if (x + 6 > OLED_WIDTH || y + 8 > OLED_HEIGHT) return;
    if (ch < ' ' || ch > '~') ch = ' ';

    /* 先擦除该字符区域 */
    for (uint8_t col = 0; col < 6; col++) {
        uint16_t idx = (x + col) + (y / 8) * OLED_WIDTH;
        uint8_t mask = (uint8_t)(0xFF << (y % 8));
        OLED_Buffer[idx] &= ~mask;
        if (y % 8)
            OLED_Buffer[idx + OLED_WIDTH] &= ~(mask >> (8 - y % 8));
    }

    /* 再绘制 */
    const uint8_t *glyph = Font6x8[ch - ' '];
    for (uint8_t col = 0; col < 6; col++) {
        uint16_t idx = (x + col) + (y / 8) * OLED_WIDTH;
        OLED_Buffer[idx] |= glyph[col] << (y % 8);
        if (y % 8)
            OLED_Buffer[idx + OLED_WIDTH] |= glyph[col] >> (8 - y % 8);
    }
    (void)color;
}
/* ======================== 绘制字符串 ======================== */
void OLED_PutString(uint8_t x, uint8_t y, const char *str, OLED_Color color)
{
    while (*str) {
        OLED_PutChar(x, y, *str++, color);
        x += 6;
        if (x + 6 > OLED_WIDTH) break;
    }
}

/* ======================== 格式化输出 ======================== */
void OLED_Printf(uint8_t x, uint8_t y, OLED_Color color, const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    OLED_PutString(x, y, buf, color);
}

#endif /* APP_USE_OLED */
