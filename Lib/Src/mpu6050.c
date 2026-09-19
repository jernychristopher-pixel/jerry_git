#include "mpu6050.h"
#include "i2c_sw.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include <math.h>
#include "stm32f1xx_hal.h"
#include <string.h>

/* DMP输出频率: 软件I2C下100Hz即可, 降低主循环I2C占用 */
#define DEFAULT_MPU_HZ   (100)
#define q30              1073741824.0f

float MPU_Roll = 0.0f, MPU_Pitch = 0.0f, MPU_Yaw = 0.0f;

/* 板载安装姿态, 与WHEELTEC C10B例程一致; 若Yaw方向反了把Z轴对角线取反 */
static const signed char gyro_orientation[9] = {-1, 0, 0,
                                                 0,-1, 0,
                                                 0, 0, 1};

static unsigned short inv_row_2_scale(const signed char *row) {
    unsigned short b;
    if (row[0] > 0)      b = 0;
    else if (row[0] < 0) b = 4;
    else if (row[1] > 0) b = 1;
    else if (row[1] < 0) b = 5;
    else if (row[2] > 0) b = 2;
    else if (row[2] < 0) b = 6;
    else                 b = 7;   /* error */
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

static uint8_t I2C_ReadOneByte(uint8_t addr, uint8_t reg) {
    uint8_t b = 0;
    i2cRead(addr, reg, 1, &b);
    return b;
}

static uint8_t I2C_WriteOneByte(uint8_t addr, uint8_t reg, uint8_t data) {
    return i2cWrite(addr, reg, 1, &data);
}

/**
 * @brief MPU6050基础寄存器初始化 (时钟源/量程/唤醒)
 */
void MPU6050_initialize(void) {
    I2C_WriteOneByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);  /* 时钟=PLL X轴陀螺, 唤醒 */
    I2C_WriteOneByte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x08);  /* 陀螺仪 ±500dps */
    I2C_WriteOneByte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00); /* 加速度计 ±2g */
    I2C_WriteOneByte(MPU6050_ADDR, MPU6050_USER_CTRL, 0x00);   /* 关闭AUX I2C主模式 */
    I2C_WriteOneByte(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x00); /* 关闭旁路 */
}

/** @brief 采N次某陀螺轴原始值求平均 (两次采样间隔delay_ms毫秒) */
static float gyro_raw_mean(uint8_t reg, uint16_t n, uint8_t delay_ms) {
    int32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint8_t b[2] = {0, 0};
        i2cRead(MPU6050_ADDR, reg, 2, b);
        sum += (int16_t)(((uint16_t)b[0] << 8) | b[1]);
        if (delay_ms) HAL_Delay(delay_ms);
    }
    return (float)sum / (float)n;
}

/** @brief 写陀螺硬件偏移寄存器 (大端16位, 带饱和) */
static void gyro_set_offset(uint8_t reg, int32_t offs) {
    if (offs >  32767) offs =  32767;
    if (offs < -32768) offs = -32768;
    uint8_t b[2];
    b[0] = (uint8_t)(((uint16_t)(int16_t)offs >> 8) & 0xFF);
    b[1] = (uint8_t)((int16_t)offs & 0xFF);
    i2cWrite(MPU6050_ADDR, reg, 2, b);
}

/**
 * @brief 上电静止校准: 用硬件偏移寄存器(0x13/0x15/0x17)消除陀螺零偏
 * 做法: 先清零偏移采静止均值, 试写-mean并复采验证方向(残差变大则取反),
 *       再按 残差=mean+k*offs 解出k做一次精修, 把静置漂移压到最小.
 *       上电后约1.3秒内必须保持车静止不动.
 */
static void gyro_bias_calibrate(void) {
    static const uint8_t raw_reg[3]  = {MPU6050_GYRO_XOUT_H, MPU6050_GYRO_YOUT_H, MPU6050_GYRO_ZOUT_H};
    static const uint8_t offs_reg[3] = {MPU6050_XG_OFFS_USR_H, MPU6050_YG_OFFS_USR_H, MPU6050_ZG_OFFS_USR_H};
    int32_t offs[3];
    float mean[3], check[3];

    for (uint8_t a = 0; a < 3; a++) {
        offs[a] = 0;
        gyro_set_offset(offs_reg[a], 0);
    }
    HAL_Delay(20);

    for (uint8_t a = 0; a < 3; a++)
        mean[a] = gyro_raw_mean(raw_reg[a], 150, 6);

    /* 第一轮: 按"输出=原始+偏移"假设先写 -mean */
    for (uint8_t a = 0; a < 3; a++) {
        offs[a] = -(int32_t)mean[a];
        gyro_set_offset(offs_reg[a], offs[a]);
    }
    HAL_Delay(20);

    for (uint8_t a = 0; a < 3; a++)
        check[a] = gyro_raw_mean(raw_reg[a], 40, 6);

    for (uint8_t a = 0; a < 3; a++) {
        /* 方向验证: 残差反而变大说明寄存器极性相反 */
        if (fabsf(check[a]) > fabsf(mean[a])) {
            offs[a] = -offs[a];
            gyro_set_offset(offs_reg[a], offs[a]);
            HAL_Delay(20);
            check[a] = gyro_raw_mean(raw_reg[a], 40, 6);
        }
        /* 残差精修: 残差 = mean + k*offs -> k=(check-mean)/offs, 补 delta=-check/k */
        float k = (offs[a] != 0) ? ((check[a] - mean[a]) / (float)offs[a]) : 0.0f;
        if (fabsf(k) > 0.001f) {
            offs[a] += (int32_t)(-check[a] / k);
            gyro_set_offset(offs_reg[a], offs[a]);
        }
    }
}
uint8_t MPU6050_testConnection(void) {
    return I2C_ReadOneByte(MPU6050_ADDR, MPU6050_WHO_AM_I) == 0x68;
}

/* 读MPU6050一个16位大端寄存器, 返回有符号原始LSB */
static int16_t mpu_read_i16(uint8_t reg) {
    uint8_t b[2] = {0, 0};
    i2cRead(MPU6050_ADDR, reg, 2, b);
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

int16_t MPU6050_ReadGyroX(void) {
    return mpu_read_i16(MPU6050_GYRO_XOUT_H);
}

int16_t MPU6050_ReadGyroY(void) {
    return mpu_read_i16(MPU6050_GYRO_YOUT_H);
}

int16_t MPU6050_ReadGyroZ(void) {
    return mpu_read_i16(MPU6050_GYRO_ZOUT_H);
}

int16_t MPU6050_ReadAccelX(void) {
    return mpu_read_i16(MPU6050_ACCEL_XOUT_H);
}

int16_t MPU6050_ReadAccelY(void) {
    return mpu_read_i16(MPU6050_ACCEL_YOUT_H);
}

int16_t MPU6050_ReadAccelZ(void) {
    return mpu_read_i16(MPU6050_ACCEL_ZOUT_H);
}

float MPU6050_GetGyroSens(void) {
    uint8_t fs = (I2C_ReadOneByte(MPU6050_ADDR, MPU6050_GYRO_CONFIG) >> 3) & 0x03;
    if (fs == 0) return 131.0f;   /* ±250dps */
    if (fs == 1) return 65.5f;    /* ±500dps */
    if (fs == 2) return 32.8f;    /* ±1000dps */
    return 16.4f;                 /* ±2000dps */
}

/**
 * @brief 下载固件并启动内置DMP, 返回0=成功
 */
int DMP_Init(void) {
    if (!MPU6050_testConnection()) return -1;
    if (mpu_init())                                    return -2;
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) return -3;
    if (mpu_set_gyro_fsr(500))                         return -3;  /* ±500dps, 提高角速率分辨率(必须在mpu_set_sensors之后) */
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) return -4;
    if (mpu_set_sample_rate(DEFAULT_MPU_HZ))           return -5;
    if (dmp_load_motion_driver_firmware())             return -6;
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) return -7;
    /* 四元数 + DMP自带陀螺校准: MPU6050的offset寄存器在DMP模式下不生效,
       YAW的漂移只能靠GYRO_CAL消除(上电静止数秒后自动收敛) */
    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL)) return -8;
    if (dmp_set_fifo_rate(DEFAULT_MPU_HZ))             return -9;
    gyro_bias_calibrate();  /* 静止校准: 写硬件偏移寄存器消除陀螺零偏 */
    if (mpu_set_dmp_state(1))                          return -10;
    return 0;
}

/**
 * @brief 从DMP FIFO读取姿态, 返回1=拿到了新四元数并更新MPU_Roll/Pitch/Yaw
 */
int Read_DMP(void) {
    unsigned long sensor_timestamp;
    unsigned char more;
    short sensors = 0;
    short gyro_d[3], accel_d[3];
    long quat[4];

    if (dmp_read_fifo(gyro_d, accel_d, quat, &sensor_timestamp, &sensors, &more)) {
        return 0;
    }
    if (sensors & INV_WXYZ_QUAT) {
        float q0f = quat[0] / q30;
        float q1f = quat[1] / q30;
        float q2f = quat[2] / q30;
        float q3f = quat[3] / q30;
        MPU_Roll  = asinf(-2.0f * q1f * q3f + 2.0f * q0f * q2f) * 57.3f;
        MPU_Pitch = atan2f(2.0f * q2f * q3f + 2.0f * q0f * q1f,
                           -2.0f * q1f * q1f - 2.0f * q2f * q2f + 1.0f) * 57.3f;
        MPU_Yaw   = atan2f(2.0f * (q1f * q2f + q0f * q3f),
                           q0f * q0f + q1f * q1f - q2f * q2f - q3f * q3f) * 57.3f;
        return 1;
    }
    return 0;
}
