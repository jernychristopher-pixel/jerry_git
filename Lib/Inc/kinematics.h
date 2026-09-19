#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/* ===== 机体坐标系符号标定 (实车实测, 2026-09-11) =====
 * 实测: 正命令让驱动轮朝车尾转; 而转速环内部(编码器->PID->PWM)是自洽的.
 * 这里只在"命令入口"和"反馈出口"各翻一次, 把对外坐标系摆正:
 *      对外 +V = 车头方向,  +W = 逆时针(左转), 与 ROS REP-103 一致.
 * 内部电机/编码器极性、PID 与航向环标定全部不动 —— 以后要改方向只动这两个宏.
 */
#define CHASSIS_V_SIGN   (-1.0f)
#define CHASSIS_W_SIGN   (+1.0f)

/* 底盘控制模式: 手动(手柄) / 自动(SLAM上位机) */
typedef enum {
    CHASSIS_MODE_MANUAL = 0,
    CHASSIS_MODE_AUTO   = 1
} Chassis_Mode_t;

/* 速度命令来源, 与当前控制模式匹配才会被采纳 */
typedef enum {
    CHASSIS_SRC_MANUAL = 0,
    CHASSIS_SRC_AUTO   = 1
} Chassis_Src_t;

typedef struct {
    float target_pulse_L;
    float target_pulse_R;
    float raw_pulse_L;      /* 原始脉冲: PID 反馈 + VOFA 波形 */
    float raw_pulse_R;
    float actual_pulse_L;   /* EMA 滤波后: OLED 显示 */
    float actual_pulse_R;
} Robot_State_t;

extern Robot_State_t robot_state;
extern float brake_kp;   /* 刹车力度, VOFA #O 在线调 (默认10, 可用范围 0~20) */
extern float brake_kd;   /* 刹车微分, VOFA #P 在线调 (默认0.3, 可用范围 0~8) */

void Inverse_Kinematics(float V, float W);
void Chassis_Control_Loop(void);

/* 统一速度命令入口: V(m/s), W(rad/s), 手动与自动唯一的下发通道 */
void Chassis_Set_Cmd(float V, float W, Chassis_Src_t src);
void Chassis_Set_Mode(Chassis_Mode_t mode);
Chassis_Mode_t Chassis_Get_Mode(void);

/* 供串口反馈使用: 由左右轮实际速度正解出底盘速度 */
float Chassis_GetVx_MmS(void);   /* 前进速度 mm/s */
float Chassis_GetWz_MradS(void); /* 角速度 rad/s * 1000 */
/* 左右轮实测线速度 mm/s, 前进为正。与上面两个正解同源, 恒有 vx = (vl + vr) / 2,
 * 因此上位机不需要知道轮距即可显示左右轮速。 */
void  Chassis_GetWheels_MmS(float *vl_mm_s, float *vr_mm_s);

#endif
