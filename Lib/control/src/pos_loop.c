#include "pos_loop.h"
#include "kinematics.h"

#include <math.h>

/* 物理常量 (WHEEL_TRACK / WHEEL_PERIMETER / ENCODER_CPR) 一律引用
   kinematics.h 的唯一来源, 本文件不得复制数值 */

#define PULSE_PER_M      (ENCODER_CPR / WHEEL_PERIMETER)   /* 约 7130 脉冲/m */
#define DEG2RAD          0.01745329252f
#define RAD2DEG          57.2957795f

/* 关于"到位"误判: 不需要额外的解锁门槛。
   PosLoop_Start() 已经保证目标里至少有一项超出容差, 第一拍不可能判定完成。
   (早期版本按"里程超过 5cm 才解锁"实现, 结果纯原地旋转永远不到位 ——
    原地转的里程增量恰好是 0, 这个坑留在这里做记录) */

static PosLoop_Config_t cfg;

static uint8_t active;
static uint8_t done;

static float odom_dist;   /* 累计里程 m, 前进为正 */
static float odom_yaw;    /* 累计转角 rad, 逆时针为正 */
static float tgt_dist;    /* 目标里程 m */
static float tgt_yaw;     /* 目标航向 rad (相对出发时刻) */

/* 位置式 PID 每环 3 个状态: 本拍误差 / 上拍误差 / 误差积分 */
static float d_err, d_err_prev, d_int;
static float y_err, y_err_prev, y_int;

static float out_v, out_w;

/**
  * 函    数：把角度归一化到 (-pi, pi]
  * 参    数：a 任意角度, 弧度
  * 返 回 值：归一化后的角度
  */
static float wrap_pi(float a)
{
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}

/**
  * 函    数：取默认整定参数
  * 参    数：cfg_out 指定结构体的地址
  * 返 回 值：无
  */
void PosLoop_GetDefaultConfig(PosLoop_Config_t *cfg_out)
{
    if (cfg_out == 0) return;

    /* 距离环: 剩 1m 时给 0.8 m/s, 会被限幅到 v_max; 微分项负责提前收油门防止冲过 */
    cfg_out->dist.kp = 0.80f;
    cfg_out->dist.ki = 0.15f;
    cfg_out->dist.kd = 1.60f;

    /* 航向环: 误差 0.3rad(约17度) 时给 0.6 rad/s */
    cfg_out->yaw.kp = 2.00f;
    cfg_out->yaw.ki = 0.30f;
    cfg_out->yaw.kd = 0.05f;

    cfg_out->v_max = 0.35f;
    cfg_out->w_max = 1.20f;

    cfg_out->dist_i_max = 1.5f;
    cfg_out->yaw_i_max  = 1.0f;

    cfg_out->dist_tol_m  = 0.005f;   /* 5mm */
    cfg_out->yaw_tol_deg = 1.5f;    /* 仿真实测: 收到 1.5 度只多花 0.05s, 精度翻倍 */
}

/**
  * 函    数：设置整定参数 (校验后生效, 非法参数整组丢弃)
  * 参    数：cfg_in 指定结构体的地址
  * 返 回 值：无
  */
void PosLoop_SetConfig(const PosLoop_Config_t *cfg_in)
{
    if (cfg_in == 0) return;
    if (cfg_in->v_max <= 0.0f || cfg_in->w_max <= 0.0f) return;
    if (cfg_in->dist_i_max <= 0.0f || cfg_in->yaw_i_max <= 0.0f) return;
    if (cfg_in->dist_tol_m <= 0.0f || cfg_in->yaw_tol_deg <= 0.0f) return;
    cfg = *cfg_in;
}

/**
  * 函    数：位置环初始化, 上电调用一次
  * 参    数：无
  * 返 回 值：无
  */
void PosLoop_Init(void)
{
    PosLoop_GetDefaultConfig(&cfg);
    PosLoop_Stop();
    odom_dist = 0.0f;
    odom_yaw  = 0.0f;
}

/**
  * 函    数：停止位置环并清空全部积分与差分历史
  * 参    数：无
  * 返 回 值：无
  */
void PosLoop_Stop(void)
{
    active = 0;
    done   = 0;

    d_err = d_err_prev = d_int = 0.0f;
    y_err = y_err_prev = y_int = 0.0f;

    out_v = 0.0f;
    out_w = 0.0f;
    tgt_dist = 0.0f;
    tgt_yaw  = 0.0f;
}

/**
  * 函    数：下发一段新的位置目标
  * 参    数：dist_m        目标前进距离, m, 可为负(后退)
  *           yaw_delta_deg 目标转角增量, 度, 逆时针为正, 0 = 保持航向直行
  * 返 回 值：无
  */
void PosLoop_Start(float dist_m, float yaw_delta_deg)
{
    /* 先清历史: 上一段的积分和微分状态绝不能带进新的一段 */
    PosLoop_Stop();

    odom_dist = 0.0f;
    odom_yaw  = 0.0f;

    tgt_dist = dist_m;
    tgt_yaw  = yaw_delta_deg * DEG2RAD;

    /* 目标本来就落在容差里, 直接判完成, 不启动 */
    if (fabsf(tgt_dist) < cfg.dist_tol_m &&
        fabsf(yaw_delta_deg) < cfg.yaw_tol_deg) {
        done = 1;
        return;
    }

    active = 1;
}

uint8_t PosLoop_IsActive(void) { return active; }
uint8_t PosLoop_IsDone(void)   { return done; }

float PosLoop_GetV(void)          { return out_v; }
float PosLoop_GetW(void)          { return out_w; }
float PosLoop_GetDist(void)       { return odom_dist; }
float PosLoop_GetYawDeg(void)     { return odom_yaw * RAD2DEG; }
float PosLoop_GetDistErr(void)    { return d_err; }
float PosLoop_GetYawErrDeg(void)  { return y_err * RAD2DEG; }

/**
  * 函    数：位置环主体, 每 25ms 调用一次
  * 参    数：pulse_L / pulse_R 本拍左右轮脉冲增量 (前进为正)
  * 返 回 值：1 = 本拍判定到位
  */
uint8_t PosLoop_Feed(float pulse_L, float pulse_R)
{
    if (!active) return 0;

    /* 1. 里程计: 差速底盘正解
          前进 = 两轮平均行程; 转角 = 左右轮程差 / 轮距 */
    float dL = pulse_L / PULSE_PER_M;
    float dR = pulse_R / PULSE_PER_M;
    odom_dist += (dL + dR) * 0.5f;
    odom_yaw  += (dR - dL) / WHEEL_TRACK;

    /* 2. 距离环: 位置式 PID, 输出 V */
    d_err_prev = d_err;
    d_err = tgt_dist - odom_dist;
    if (cfg.dist.ki != 0.0f) {
        d_int += d_err;
        if (d_int >  cfg.dist_i_max) d_int =  cfg.dist_i_max;
        if (d_int < -cfg.dist_i_max) d_int = -cfg.dist_i_max;
    } else {
        d_int = 0.0f;   /* Ki=0 时强制归零, 便于调试 (与 pid.c 同做法) */
    }
    out_v = cfg.dist.kp * d_err
          + cfg.dist.ki * d_int
          + cfg.dist.kd * (d_err - d_err_prev);
    if (out_v >  cfg.v_max) out_v =  cfg.v_max;
    if (out_v < -cfg.v_max) out_v = -cfg.v_max;
    /* 输出饱和且误差仍在同向推 → 本拍积分回滚, 防卷绕 (与 yaw_hold.c 同做法) */
    if (out_v >= cfg.v_max && d_err > 0.0f) d_int -= d_err;
    if (out_v <= -cfg.v_max && d_err < 0.0f) d_int -= d_err;

    /* 3. 航向环: 位置式 PID, 输出 W; 误差先归一化到 (-180, 180] 度 */
    y_err_prev = y_err;
    y_err = wrap_pi(tgt_yaw - odom_yaw);
    if (cfg.yaw.ki != 0.0f) {
        y_int += y_err;
        if (y_int >  cfg.yaw_i_max) y_int =  cfg.yaw_i_max;
        if (y_int < -cfg.yaw_i_max) y_int = -cfg.yaw_i_max;
    } else {
        y_int = 0.0f;
    }
    out_w = cfg.yaw.kp * y_err
          + cfg.yaw.ki * y_int
          + cfg.yaw.kd * (y_err - y_err_prev);
    if (out_w >  cfg.w_max) out_w =  cfg.w_max;
    if (out_w < -cfg.w_max) out_w = -cfg.w_max;
    if (out_w >= cfg.w_max && y_err > 0.0f) y_int -= y_err;
    if (out_w <= -cfg.w_max && y_err < 0.0f) y_int -= y_err;

    /* 4. 到位判定: 距离和航向都进容差才算到位 */
    if (fabsf(d_err) < cfg.dist_tol_m &&
        fabsf(y_err) < cfg.yaw_tol_deg * DEG2RAD) {
        done   = 1;
        active = 0;
        out_v  = 0.0f;
        out_w  = 0.0f;
        return 1;
    }

    return 0;
}