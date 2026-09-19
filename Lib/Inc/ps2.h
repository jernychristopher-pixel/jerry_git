#ifndef __PS2_H
#define __PS2_H

#include "main.h"

/* PS2 手柄接线 (C10B 主板) */
#define PS2_DI_PORT     GPIOB
#define PS2_DI_PIN      GPIO_PIN_8      // PB8  ← 手柄 DO

#define PS2_DO_PORT     GPIOC
#define PS2_DO_PIN      GPIO_PIN_9      // PC9  → 手柄 DI

#define PS2_CS_PORT     GPIOC
#define PS2_CS_PIN      GPIO_PIN_4      // PC4  → 手柄 CS

#define PS2_CLK_PORT    GPIOC
#define PS2_CLK_PIN     GPIO_PIN_8      // PC8  → 手柄 CLK

/* 按键码 */
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

/* 摇杆通道索引 */
#define PSS_RX  5
#define PSS_RY  6
#define PSS_LX  7
#define PSS_LY  8

extern uint8_t  PS2_Data[9];
extern uint16_t PS2_Key;

void PS2_Init(void);
void PS2_SetInit(void);
uint8_t PS2_EnableAnalog(void);
void PS2_ReadData(void);
uint8_t PS2_DataKey(void);
uint8_t PS2_AnologData(uint8_t channel);

#endif