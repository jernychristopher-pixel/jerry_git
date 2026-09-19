#ifndef __YAW_HOLD_H
#define __YAW_HOLD_H

#include <stdint.h>

/* 航向保持: 直行时用IMU偏航角修正W, 解决左右轮速偏差导致的走不直 */
float YawHold_Process(float V, float W, float act_pulses); /* 每25ms调用; act_pulses=两侧编码器脉冲绝对值和, 用于判断真静止 */
void  YawHold_Enable(uint8_t en);
void  YawHold_SetKp(float kp);
void  YawHold_SetKi(float ki);
void  YawHold_SetKd(float kd);
float YawHold_GetError(void);              /* 当前航向误差, 度, 供OLED/VOFA显示 */
float YawHold_GetLastOut(void);           /* 最近一次W修正量, rad/s, 供VOFA诊断 */
float YawHold_GetYaw(void);                /* 当前IMU偏航, 度 */

#endif
