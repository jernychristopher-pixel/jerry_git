#include "i2c_sw.h"
#include "stm32f1xx_hal.h"

#define IIC_SCL_PORT  GPIOB
#define IIC_SCL_PIN   GPIO_PIN_14
#define IIC_SDA_PORT  GPIOB
#define IIC_SDA_PIN   GPIO_PIN_15

/* SDA方向切换: 直接操作CRH, 与WHEELTEC例程相同的寄存器写法 */
#define SDA_IN()   do { GPIOB->CRH &= ~0xF0000000UL; GPIOB->CRH |= 0x80000000UL; } while (0)
#define SDA_OUT()  do { GPIOB->CRH &= ~0xF0000000UL; GPIOB->CRH |= 0x30000000UL; } while (0)
#define IIC_SCL_H()  (GPIOB->BSRR = (uint32_t)IIC_SCL_PIN)
#define IIC_SCL_L()  (GPIOB->BSRR = (uint32_t)IIC_SCL_PIN << 16)
#define IIC_SDA_H()  (GPIOB->BSRR = (uint32_t)IIC_SDA_PIN)
#define IIC_SDA_L()  (GPIOB->BSRR = (uint32_t)IIC_SDA_PIN << 16)
#define READ_SDA()   ((GPIOB->IDR & (uint32_t)IIC_SDA_PIN) ? 1 : 0)

static uint8_t dwt_ready = 0;

void delay_us(uint32_t us) {
    if (!dwt_ready) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dwt_ready = 1;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < cycles) {
    }
}

void IIC_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SCL_PORT, &gpio);
    IIC_SCL_H();
    IIC_SDA_H();
}

static int IIC_Start(void) {
    SDA_OUT();
    IIC_SDA_H();
    if (!READ_SDA()) return 0;
    IIC_SCL_H();
    delay_us(1);
    IIC_SDA_L();                    /* SCL为高时SDA下降沿 = 起始信号 */
    if (READ_SDA()) return 0;
    delay_us(1);
    IIC_SCL_L();                    /* 钳住总线 */
    return 1;
}

static void IIC_Stop(void) {
    SDA_OUT();
    IIC_SCL_L();
    IIC_SDA_L();
    delay_us(1);
    IIC_SCL_H();
    IIC_SDA_H();                    /* SCL为高时SDA上升沿 = 停止信号 */
    delay_us(1);
}

static int IIC_Wait_Ack(void) {
    uint8_t err = 0;
    SDA_IN();
    IIC_SDA_H();
    delay_us(1);
    IIC_SCL_H();
    delay_us(1);
    while (READ_SDA()) {
        if (++err > 50) {
            IIC_Stop();
            return 0;
        }
        delay_us(1);
    }
    IIC_SCL_L();
    return 1;
}

static void IIC_Ack(void) {
    IIC_SCL_L();
    SDA_OUT();
    IIC_SDA_L();
    delay_us(1);
    IIC_SCL_H();
    delay_us(1);
    IIC_SCL_L();
}

static void IIC_NAck(void) {
    IIC_SCL_L();
    SDA_OUT();
    IIC_SDA_H();
    delay_us(1);
    IIC_SCL_H();
    delay_us(1);
    IIC_SCL_L();
}

static void IIC_Send_Byte(uint8_t txd) {
    uint8_t t;
    SDA_OUT();
    IIC_SCL_L();
    for (t = 0; t < 8; t++) {
        if (txd & 0x80) IIC_SDA_H(); else IIC_SDA_L();
        txd <<= 1;
        delay_us(1);
        IIC_SCL_H();
        delay_us(1);
        IIC_SCL_L();
        delay_us(1);
    }
}

static uint8_t IIC_Read_Byte(uint8_t ack) {
    uint8_t i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++) {
        IIC_SCL_L();
        delay_us(2);
        IIC_SCL_H();
        receive <<= 1;
        if (READ_SDA()) receive++;
        delay_us(2);
    }
    if (ack) IIC_Ack(); else IIC_NAck();
    return receive;
}

/**
 * @brief 写器件寄存器: addr为7位地址, 返回0=成功
 */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
    uint8_t i;
    if (!IIC_Start()) return 1;
    IIC_Send_Byte((uint8_t)(addr << 1));
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    }
    IIC_Stop();
    return 0;
}

/**
 * @brief 读器件寄存器: addr为7位地址, 返回0=成功
 */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
    if (!IIC_Start()) return 1;
    IIC_Send_Byte((uint8_t)(addr << 1));
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    if (!IIC_Start()) return 1;
    IIC_Send_Byte((uint8_t)((addr << 1) | 1));
    if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    while (len) {
        if (len == 1) *buf = IIC_Read_Byte(0);
        else          *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}
