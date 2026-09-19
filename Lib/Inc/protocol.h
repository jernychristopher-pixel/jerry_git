#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* 37字节上行反馈帧 (底盘 -> 树莓派)
 *
 *  偏移 长度  字段
 *   0    1   0x7B
 *   1    1   flag_stop        0x00=电机使能, 非0=失能
 *   2    2   vx_mm_s         X轴速度 mm/s, 车头方向为正
 *   4    2   vy_mm_s         Y轴速度, 差速车恒为0
 *   6    2   wz_mrad_s       Z轴角速度 mrad/s, 逆时针(左转)为正
 *   8    2   acc_x_raw       MPU6050 原始值, 0=未接
 *  10    2   acc_y_raw
 *  12    2   acc_z_raw
 *  14    2   gyro_x_raw
 *  16    2   gyro_y_raw
 *  18    2   gyro_z_raw
 *  20    2   battery_mv      电池电压 mV, 0=未接
 *  22    2   yaw_001deg      DMP偏航角, 0.01度 (原来显示在OLED上的 YAW)
 *  24    2   yaw_rate_mrad_s 零偏补偿后偏航角速率, mrad/s (OLED 的 Ry)
 *  26    2   yaw_hold_err_001deg 航向环角度偏差, 0.01度 (OLED 的 E)
 *  28    2   yaw_hold_out_mrad_s 航向环输出修正量, mrad/s (OLED 的 Wc)
*  30    1   status          bit0=IMU可用 bit1=1:上位机(AUTO)控制中 bit2-3=挡位(0..2)
 *  31    2   vl_mm_s         左轮实测线速度 mm/s, 前进为正
 *  33    2   vr_mm_s         右轮实测线速度 mm/s, 前进为正
 *  35    1   BCC             前 35 字节异或
 *  36    1   0x7D
 *
 * 约束: vx = (vl + vr) / 2, wz = (vr - vl) / 轮距 —— 三者同源, 上位机不得自行反推左右轮速。
 * 扩展规则: 新字段一律追加在 status 之后、BCC 之前, 0..30 的既有偏移保持不变。
 */
typedef struct {
    uint8_t  flag_stop;   /* 0x00=电机使能, 非0=失能 */
    int16_t  vx_mm_s;     /* X轴速度, mm/s, 前为正 */
    int16_t  vy_mm_s;     /* Y轴速度, mm/s, 差速车恒为0 */
    int16_t  wz_mrad_s;   /* Z轴角速度, rad/s * 1000, 逆时针为正 */
    int16_t  acc_x_raw;   /* X轴加速度原始数据, 0=未接 */
    int16_t  acc_y_raw;
    int16_t  acc_z_raw;
    int16_t  gyro_x_raw;  /* X轴角速度原始数据, 0=未接 */
    int16_t  gyro_y_raw;
    int16_t  gyro_z_raw;  /* Z轴角速度原始数据(MPU6050 LSB) */
    uint16_t battery_mv;  /* 电池电压 mV, 0=未接 */
    int16_t  yaw_001deg;            /* DMP偏航角, 0.01度 */
    int16_t  yaw_rate_mrad_s;       /* 偏航角速率, mrad/s */
    int16_t  yaw_hold_err_001deg;   /* 航向环角度偏差, 0.01度 */
    int16_t  yaw_hold_out_mrad_s;   /* 航向环输出修正量, mrad/s */
    uint8_t  status;                /* 见上面帧布局说明 */
    int16_t  vl_mm_s;               /* 左轮实测线速度, mm/s, 前为正 */
    int16_t  vr_mm_s;               /* 右轮实测线速度, mm/s, 前为正 */
} Protocol_Feedback_t;

/* 把浮点物理量限幅后转 int16, 防止 IMU 异常值在组帧时溢出 */
int16_t Protocol_ClampI16(float v);

/* 11字节下行控制帧解析 (树莓派 -> 底盘) */
void Protocol_Push_Byte(uint8_t ch);

/* 二进制组帧是否处于"帧间空闲"(没收到半个帧)。
 * USART1 借此区分 '#' 文本调参命令与二进制帧头: 空闲状态下解析器本来就忽略
 * 非 0x7B 字节, 所以借用 '#' 不会打乱组帧。 */
uint8_t Protocol_Rx_Idle(void);

/* 发送33字节反馈帧 */
void Protocol_Send_Feedback(const Protocol_Feedback_t *fb);

#endif
