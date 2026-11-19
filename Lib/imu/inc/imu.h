#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

uint8_t IMU_Init(void);      /* 返回1=MPU6050+DMP初始化成功 */
uint8_t IMU_IsOk(void);      /* DMP四元数可用(仅用于显示) */
uint8_t IMU_RateOk(void);    /* 硬件连接正常(角速率控制只用这个) */
int     IMU_GetInitErr(void); /* DMP初始化失败步骤码, 0=成功 */
void    IMU_NotifyStill(void); /* 车轮静止时调用, 缓慢跟踪角速率零偏(抗温漂) */
void    IMU_Read(void);      /* 主循环轮询: 有FIFO数据就更新姿态 */
void    IMU_ResetYaw(void);  /* 把当前偏航角置零 */
float   IMU_GetYaw(void);    /* 相对上电零点的偏航角, -180~+180度 */
float   IMU_GetYawRate(void);/* 零偏补偿后的偏航角速率(rad/s, 左转为正), 主循环缓存值 */
float   IMU_GetPitch(void);
float   IMU_GetRoll(void);
int16_t IMU_GetGyroXRaw(void);  /* X轴陀螺原始LSB */
int16_t IMU_GetGyroYRaw(void);  /* Y轴陀螺原始LSB */
int16_t IMU_GetGyroZRaw(void); /* Z轴陀螺原始LSB, 串口反馈帧使用 */
int16_t IMU_GetAccelXRaw(void); /* X轴加速度原始LSB */
int16_t IMU_GetAccelYRaw(void); /* Y轴加速度原始LSB */
int16_t IMU_GetAccelZRaw(void); /* Z轴加速度原始LSB */

#endif
