#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/* ===== 底盘物理常量: 全工程唯一来源 =====
 * 轮距 / 轮周长 / 编码器线数只在这里定义一次, 其它模块一律引用这里的宏,
 * 不得复制数值 —— 复制出去的第二份迟早和这里对不上。
 * 注意: tools/pid_demo.py 是纯 PC 仿真, 没法 include C 头文件,
 *       它里面的同名常量必须与本处手工保持一致。
 */
#define WHEEL_TRACK      0.115f   /* 轮距 115mm (米) */
#define WHEEL_DIAMETER   0.065f   /* 轮径 65mm (米) */
#define WHEEL_PERIMETER  0.2042f  /* 轮周长 (pi * D) */
#define ENCODER_CPR      1456.0f  /* 减速比28 * 线数13 * 4倍频 */
/* ===== 机体坐标系符号标定 (实车实测, 2026-09-11) =====
 * 实测: 正命令让驱动轮朝车尾转; 而转速环内部(编码器->PID->PWM)是自洽的.
 * 这里只在"命令入口"和"反馈出口"各翻一次, 把对外坐标系摆正:
 *      对外 +V = 车头方向,  +W = 逆时针(左转), 与 ROS REP-103 一致.
 * 内部电机/编码器极性、PID 与航向环标定全部不动 —— 以后要改方向只动这两个宏.
 */
#define CHASSIS_V_SIGN   (-1.0f)
#define CHASSIS_W_SIGN   (+1.0f)

/* 纯上位机控制: 底盘没有本地手动控制源, V/W 只来自 USART1 下行帧。 */

typedef struct {
    float target_rpm_L;     /* 目标轮速 r/min: PID 给定 */
    float target_rpm_R;
    float raw_pulse_L;      /* 本周期原始脉冲: 协议上报与静止判断 */
    float raw_pulse_R;
    float actual_rpm_L;     /* 实测轮速 r/min: 3 拍滑动窗口, PID 反馈 */
    float actual_rpm_R;
    float actual_pulse_L;   /* EMA 滤波后: OLED 显示 */
    float actual_pulse_R;
} Robot_State_t;

extern Robot_State_t robot_state;

void Inverse_Kinematics(float V, float W);
void Chassis_Control_Loop(void);

/* 单位换算: 本周期脉冲数 <-> r/min */
float Pulse_To_Rpm(float pulse);
float Rpm_To_Pulse(float rpm);
float Mps_To_Rpm(float v);

/* M4-3 速度环整定: 直接以左右轮 r/min 为目标, 绕过 V/W 逆解。
 * 进入后航向保持不参与, 退出后回到上位机 V/W 控制。 */
void Chassis_Set_Wheel_Rpm(float left_rpm, float right_rpm);
void Chassis_Exit_Wheel_Rpm(void);
void Chassis_GetWheelRpm(float *left_rpm, float *right_rpm);

/* 统一速度命令入口: V(m/s), W(rad/s), 只接受上位机下发 */
void Chassis_Set_Cmd(float V, float W);

/* 供串口反馈使用: 由左右轮实际速度正解出底盘速度 */
float Chassis_GetVx_MmS(void);   /* 前进速度 mm/s */
float Chassis_GetWz_MradS(void); /* 角速度 rad/s * 1000 */
/* 左右轮实测线速度 mm/s, 前进为正。与上面两个正解同源, 恒有 vx = (vl + vr) / 2,
 * 因此上位机不需要知道轮距即可显示左右轮速。 */
void  Chassis_GetWheels_MmS(float *vl_mm_s, float *vr_mm_s);

#endif
