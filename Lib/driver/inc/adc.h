#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>

/* C10B 电池电压采样: PC1 -> ADC1_IN11, 分压比 11:1
 * 参考电压 3.3V, 12bit ADC
 */
void     Battery_ADC_Init(void);
uint16_t Battery_Get_Mv(void);   /* 返回电池电压, 单位 mV */

#endif
