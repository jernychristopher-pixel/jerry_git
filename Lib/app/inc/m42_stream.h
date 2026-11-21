#ifndef __M42_STREAM_H
#define __M42_STREAM_H

#include <stdint.h>

/*
 * M4-2 采集流 (APP_MODE_M42_STREAM == 1 时启用)
 *
 * 100 Hz 从 MPU6050 一次读 14 字节, 做 -gy 角速度积分 + atan2(-ax, az)
 * 加速度参考的互补滤波, 同时用 1 s 窗口统计编码器脉冲。
 * 输出 13 列 CSV:
 *   t,count_l,count_r,rpm_l,rpm_r,ax_g,ay_g,az_g,
 *   gx_dps,gy_dps,gz_dps,angle_deg,gyro_angle_deg
 *
 * 串口发送用中断 + 2KB 队列, 不阻塞 10 ms 采样节拍。
 */

void     M42_Init(void);       /* 上电静止 2 s 估 gyro_y 零偏, 打印 # 行 */
void     M42_Step(void);       /* 每 10 ms 调一次 */
void     M42_Stop(void);       /* 切回协议模式: 中止在途发送并清空队列 */
void     M42_TxComplete(void); /* HAL_UART_TxCpltCallback 里转发 */
uint32_t M42_GetI2cErrorCount(void);
uint32_t M42_GetDropLineCount(void);

#endif
