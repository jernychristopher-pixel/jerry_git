#ifndef __POS_LOOP_H
#define __POS_LOOP_H

#include <stdint.h>

/* ===== 位置式 PID 位置环 =====
 *
 * 作用: 让底盘自己走到指定距离 / 转到指定角度后停住, 而不是只能"一直按速度走"。
 *       典型用途: 走固定距离做里程计标定、定角度转向、导航里的小段位姿修正。
 *
 * 与速度环 (Lib/Src/pid.c) 是级联关系, 不是替代关系:
 *     本文件  位置环: 目标 = 距离/角度    输出 = V(m/s), W(rad/s)
 *        |
 *        v
 *     pid.c   速度环: 目标 = 每拍脉冲数   输出 = PWM
 *
 * 算法就是教科书位置式 PID, 与 WHEELTEC 例程 Position_PID 同一形式:
 *     Out = Kp*e + Ki*Σe + Kd*(e - e_prev)
 * 输出直接是"绝对控制量", 每个节拍重算一次、不累加 —— 这正是位置式与增量式的分界。
 *
 * 节拍: 固定 25ms 一拍, 与 Chassis_Control_Loop() 同节拍。
 * 本模块是纯算法, 不依赖 HAL, 也不自己驱动电机; 谁调用谁负责把
 * PosLoop_GetV()/PosLoop_GetW() 下发出去。
 */

typedef struct {
    float kp;   /* 比例 */
    float ki;   /* 积分 (每个 25ms 节拍累加一次误差) */
    float kd;   /* 微分 (对误差差分, 不是对测量差分) */
} PosPID_Gains_t;

typedef struct {
    PosPID_Gains_t dist;   /* 距离环: 剩余距离(m)  -> 前进速度 V(m/s) */
    PosPID_Gains_t yaw;    /* 航向环: 航向误差(rad) -> 角速度 W(rad/s) */

    float v_max;           /* V 输出限幅, m/s */
    float w_max;           /* W 输出限幅, rad/s */

    float dist_i_max;      /* 距离环积分限幅 (防卷绕) */
    float yaw_i_max;       /* 航向环积分限幅 */

    float dist_tol_m;      /* 到位判据: 剩余距离小于此值 */
    float yaw_tol_deg;     /* 到位判据: 航向误差小于此值 */
} PosLoop_Config_t;

void PosLoop_Init(void);
void PosLoop_GetDefaultConfig(PosLoop_Config_t *cfg_out);
void PosLoop_SetConfig(const PosLoop_Config_t *cfg_in);

/* 以"当前所在位置、当前朝向"为原点下发目标; yaw_delta_deg = 0 表示保持航向直行 */
void PosLoop_Start(float dist_m, float yaw_delta_deg);
void PosLoop_Stop(void);

uint8_t PosLoop_IsActive(void);
uint8_t PosLoop_IsDone(void);

/* 每 25ms 调用一次, 传入本拍的左右轮脉冲增量。
   脉冲必须已按对外坐标系标定 (前进为正), 即 CHASSIS_V_SIGN * raw_pulse。
   返回值: 1 = 本拍判定到位 */
uint8_t PosLoop_Feed(float pulse_L, float pulse_R);

/* 输出: 本拍算出的速度命令, 交给速度环执行 */
float PosLoop_GetV(void);
float PosLoop_GetW(void);

/* 诊断量 */
float PosLoop_GetDist(void);      /* 累计里程 m */
float PosLoop_GetYawDeg(void);    /* 累计转角 deg, 逆时针为正 */
float PosLoop_GetDistErr(void);   /* 剩余距离 m */
float PosLoop_GetYawErrDeg(void); /* 航向误差 deg */

#endif /* __POS_LOOP_H */