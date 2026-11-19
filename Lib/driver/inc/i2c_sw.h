#ifndef __I2C_SW_H
#define __I2C_SW_H

#include <stdint.h>

/* 软件I2C: PB14=SCL, PB15=SDA (与WHEELTEC C10B例程一致) */
void IIC_Init(void);
int  i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int  i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void delay_us(uint32_t us);

#endif
