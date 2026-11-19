#include "adc.h"
#include "stm32f1xx_hal.h"

/* C10B 电池电压采样:
 *   电池经 11:1 电阻分压后送入 PC1 (ADC1_IN11)
 *   VREF = 3.3V, 12bit ADC
 *   电池电压 mV = raw * (3.3 / 4095) * 11 * 1000
 *               = raw * 36300 / 4095
 */
#define BATTERY_MV_NUM  36300U
#define BATTERY_MV_DEN  4095U

void Battery_ADC_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PC1 模拟输入 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin  = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* ADC1 时钟与分频: PCLK2/8 = 9MHz (符合 F1 ADC 时钟上限) */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV8);

    /* 单次转换, 软件触发(SWSTART), 右对齐, 12bit, 独立模式 */
    ADC1->CR1 = 0U;
    ADC1->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;   /* EXTSEL=111 -> 软件触发 */

    /* 通道11采样时间: 239.5 周期 */
    ADC1->SMPR1 = ADC_SMPR1_SMP11_0 | ADC_SMPR1_SMP11_1 | ADC_SMPR1_SMP11_2;

    /* 规则序列长度=1, 序列1=通道11 */
    ADC1->SQR1 = 0U;
    ADC1->SQR3 = 11U;

    /* 使能 ADC 并等待稳定 */
    ADC1->CR2 |= ADC_CR2_ADON;
    volatile uint32_t wait = 1000U;
    while (wait--) { __NOP(); }

    /* 校准 */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) { }
}

uint16_t Battery_Get_Mv(void)
{
    uint32_t raw;

    ADC1->SR = 0U;                       /* 清空标志 */
    ADC1->CR2 |= ADC_CR2_SWSTART;        /* 启动软件转换 */
    while ((ADC1->SR & ADC_SR_EOC) == 0U) { }   /* 等待转换完成 */

    raw = ADC1->DR & 0xFFFU;
    return (uint16_t)((raw * BATTERY_MV_NUM) / BATTERY_MV_DEN);
}
