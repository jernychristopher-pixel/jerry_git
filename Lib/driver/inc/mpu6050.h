#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR         0x68   /* 7位地址 */
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_USER_CTRL    0x6A
#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F

#define MPU6050_XG_OFFS_USR_H 0x13
#define MPU6050_YG_OFFS_USR_H 0x15
#define MPU6050_ZG_OFFS_USR_H 0x17

extern float MPU_Roll, MPU_Pitch, MPU_Yaw;   /* 单位: 度 */

void    MPU6050_initialize(void);
uint8_t MPU6050_testConnection(void);
int     DMP_Init(void);     /* 返回0=成功, 负数=失败步骤 */
int     Read_DMP(void);     /* 返回1=读到了新姿态 */
int16_t MPU6050_ReadGyroX(void);   /* X轴角速率原始LSB */
int16_t MPU6050_ReadGyroY(void);   /* Y轴角速率原始LSB */
int16_t MPU6050_ReadGyroZ(void);  /* 硬件偏移校正后的Z轴角速率, 原始LSB */
int16_t MPU6050_ReadAccelX(void);  /* X轴加速度原始LSB */
int16_t MPU6050_ReadAccelY(void);  /* Y轴加速度原始LSB */
int16_t MPU6050_ReadAccelZ(void);  /* Z轴加速度原始LSB */
float   MPU6050_GetGyroSens(void); /* 当前量程对应的灵敏度, 单位 LSB/dps */

#endif
