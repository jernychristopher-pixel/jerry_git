#include "pid.h"

#define PID_INT_SEP 60.0f  /* integral separation: 三档稳态误差约37脉冲, 阈值放宽到60防止高速时积分被永久跳过 */

PID_t SpeedPID_L;
PID_t SpeedPID_R;

/**
  * 函    数：PID初始化
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
void PID_Init(PID_t *p)
{
    p->Target = 0;
    p->Actual = 0;
    p->Out = 0;
    p->Error0 = 0;
    p->Error1 = 0;
    p->ErrorInt = 0;
}

/**
  * 函    数：PID计算及结构体变量值更新（位置式）
  * 参    数：PID_t * 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update(PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /*如果Ki不为0才进行误差积分，便于调试：Ki=0时积分项强制归零*/
    if (p->Ki != 0) {
        /* integral separation: skip integration far from target to avoid arrival overshoot */
        if (p->Error0 < PID_INT_SEP && p->Error0 > -PID_INT_SEP) {
            p->ErrorInt += p->Error0;
        }
    } else {
        p->ErrorInt = 0;
    }

    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);

    /* anti-windup: back-calculate integral when output saturates */
    if (p->Out > p->OutMax) {
        p->Out = p->OutMax;
        if (p->Ki != 0) {
            p->ErrorInt = (p->OutMax - p->Kp * p->Error0 - p->Kd * (p->Error0 - p->Error1)) / p->Ki;
        }
    }
    if (p->Out < p->OutMin) {
        p->Out = p->OutMin;
        if (p->Ki != 0) {
            p->ErrorInt = (p->OutMin - p->Kp * p->Error0 - p->Kd * (p->Error0 - p->Error1)) / p->Ki;
        }
    }
}
