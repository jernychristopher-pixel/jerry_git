#include "imu.h"
#include "mpu6050.h"
#include "i2c_sw.h"
#include "stm32f1xx_hal.h"

static uint8_t imu_hw_ok = 0;                    /* MPU6050硬件连接正常 → 角速率控制可用 */
static uint8_t imu_ok = 0;                       /* DMP四元数可用 → YAW显示可用 */
static int     imu_init_err = 0;                 /* DMP_Init失败步骤码, 0=成功 */
static float   yaw0 = 0.0f;
static uint8_t yaw0_set = 0;

static float   yaw_rate_bias_dps = 0.0f;         /* 软件残差零偏 (dps) */
static float   gyro_sens = 65.5f;                /* 陀螺灵敏度 LSB/dps, 上电按量程读取 */
static volatile float yaw_rate_rads = 0.0f;      /* 主循环缓存, 供中断里的航向保持读取 */

/* 上电静止采0.5s角速率平均, 兜底消除硬件偏移校准后的残差 */
static void calibrate_yaw_rate_bias(void) {
    int32_t sum = 0;
    for (uint16_t i = 0; i < 50; i++) {
        sum += MPU6050_ReadGyroZ();
        HAL_Delay(10);
    }
    yaw_rate_bias_dps = (float)sum / 50.0f / gyro_sens;
}

uint8_t IMU_Init(void) {
    IIC_Init();
    MPU6050_initialize();
    imu_hw_ok = MPU6050_testConnection();
    imu_init_err = DMP_Init();                    /* DMP失败只影响YAW显示, 不影响角速率 */
    gyro_sens = MPU6050_GetGyroSens();
    if (imu_hw_ok) {
        calibrate_yaw_rate_bias();
        /* 最多等约200ms拿到第一个姿态包, 建立偏航零点 */
        for (uint8_t i = 0; i < 40; i++) {
            HAL_Delay(5);
            if (Read_DMP()) {
                yaw0 = MPU_Yaw;
                yaw0_set = 1;
                break;
            }
        }
    }
    imu_ok = imu_hw_ok && yaw0_set;
    return imu_ok;
}

uint8_t IMU_IsOk(void) {
    return imu_ok;
}

uint8_t IMU_RateOk(void) {
    return imu_hw_ok;
}
/* 车轮静止时缓慢跟踪零偏: 消除温漂等引起的残余漂移; 读数突变则忽略 */
void IMU_NotifyStill(void) {
    if (!imu_hw_ok) return;
    float dps = (float)MPU6050_ReadGyroZ() / gyro_sens;
    float delta = dps - yaw_rate_bias_dps;
    if (delta < 2.0f && delta > -2.0f)
        yaw_rate_bias_dps += 0.02f * delta;
}

int IMU_GetInitErr(void) {
    return imu_init_err;
}

void IMU_ResetYaw(void) {
    yaw0 = MPU_Yaw;
    yaw0_set = 1;
    imu_ok = imu_hw_ok;
}

/* 主循环轮询: 刷新角速率缓存; FIFO有数据就更新姿态 */
void IMU_Read(void) {
    if (!imu_hw_ok) return;
    yaw_rate_rads = ((float)MPU6050_ReadGyroZ() / gyro_sens - yaw_rate_bias_dps) * 0.01745329252f;
    if (Read_DMP() && !yaw0_set) {
        yaw0 = MPU_Yaw;
        yaw0_set = 1;
        imu_ok = 1;
    }
}

float IMU_GetYaw(void) {
    float y = MPU_Yaw - yaw0;
    while (y > 180.0f)  y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    return y;
}

float IMU_GetYawRate(void) {
    if (!imu_hw_ok) return 0.0f;
    return yaw_rate_rads;
}

float IMU_GetPitch(void) {
    return MPU_Pitch;
}

float IMU_GetRoll(void) {
    return MPU_Roll;
}

int16_t IMU_GetGyroZRaw(void) {
    return MPU6050_ReadGyroZ();
}

int16_t IMU_GetGyroXRaw(void) {
    return MPU6050_ReadGyroX();
}

int16_t IMU_GetGyroYRaw(void) {
    return MPU6050_ReadGyroY();
}

int16_t IMU_GetAccelXRaw(void) {
    return MPU6050_ReadAccelX();
}

int16_t IMU_GetAccelYRaw(void) {
    return MPU6050_ReadAccelY();
}

int16_t IMU_GetAccelZRaw(void) {
    return MPU6050_ReadAccelZ();
}
