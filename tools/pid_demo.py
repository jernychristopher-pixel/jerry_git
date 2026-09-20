#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID 位置式 / 增量式 对比 + 位置环闭环验证。

第 1 节  用同一个电机模型跑"位置式"和"增量式"速度环。结论不是"谁更好",
         而是先证明一个精确事实: 不限幅时两者输出逐拍完全相同;
         差别只在输出饱和之后才出现。
第 2 节  把 Lib/Src/pos_loop.c 的算法原样搬到 Python 闭环跑,
         验证默认增益能否走准 1 米、转准 90 度, 上板前先在这里试参数。

只依赖标准库, python tools/pid_demo.py 即可运行。
"""

import math
import sys

DT = 0.025          # 控制周期 25ms, 与固件 TIM1 一致

# ===== 速度环被控对象: 一阶惯性 + 增益 =====
# 输入 PWM(-100~100), 输出 每25ms脉冲数。
# 稳态 脉冲/拍 = PLANT_K * PWM。取 0.6 → 目标 35 脉冲/拍时约需 PWM 58.3。
PLANT_TAU = 0.20
PLANT_K = 0.60
PWM_LIMIT = 100.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Motor(object):
    """一阶惯性: 每拍转速朝 0.6*PWM 靠拢 12.5%。"""

    def __init__(self, tau=PLANT_TAU, k=PLANT_K):
        self.tau = tau
        self.k = k
        self.v = 0.0

    def step(self, pwm):
        self.v += (DT / self.tau) * (self.k * pwm - self.v)
        return self.v


class PositionForm(object):
    """位置式: 复刻 Lib/Src/pid.c 的 PID_Update。

        Out = Kp*e + Ki*Σe + Kd*(e - e_prev)
    输出是绝对控制量, 每拍重算, 不累加。
    """

    def __init__(self, kp, ki, kd, out_max=PWM_LIMIT,
                 anti_windup=True, int_sep=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max = out_max
        self.anti_windup = anti_windup
        self.int_sep = int_sep
        self.e0 = self.e1 = 0.0
        self.ei = 0.0

    def step(self, target, actual):
        self.e1 = self.e0
        self.e0 = target - actual

        if self.ki != 0.0:
            if self.int_sep is None or -self.int_sep < self.e0 < self.int_sep:
                self.ei += self.e0
        else:
            self.ei = 0.0

        out = self.kp * self.e0 + self.ki * self.ei + self.kd * (self.e0 - self.e1)

        if out > self.out_max:
            out = self.out_max
            if self.anti_windup and self.ki != 0.0:
                # 饱和反算: 把积分拉回到"刚好不饱和"的位置 (pid.c 同做法)
                self.ei = (self.out_max - self.kp * self.e0
                           - self.kd * (self.e0 - self.e1)) / self.ki
        if out < -self.out_max:
            out = -self.out_max
            if self.anti_windup and self.ki != 0.0:
                self.ei = (-self.out_max - self.kp * self.e0
                           - self.kd * (self.e0 - self.e1)) / self.ki
        return out


class IncrementalForm(object):
    """增量式: 输出是增量, 靠累加器攒出控制量。

        du = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
        u += du
    """

    def __init__(self, kp, ki, kd, out_max=PWM_LIMIT):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max = out_max
        self.e0 = self.e1 = self.e2 = 0.0
        self.u = 0.0

    def step(self, target, actual):
        self.e2 = self.e1
        self.e1 = self.e0
        self.e0 = target - actual
        du = (self.kp * (self.e0 - self.e1)
              + self.ki * self.e0
              + self.kd * (self.e0 - 2.0 * self.e1 + self.e2))
        self.u = clamp(self.u + du, -self.out_max, self.out_max)
        return self.u


def run_speed_loop(ctrl, target, ticks):
    m = Motor()
    trace = []
    for _ in range(ticks):
        pwm = ctrl.step(target, m.v)
        trace.append((m.step(pwm), pwm))
    return trace


def metrics(trace, target):
    vs = [v for v, _ in trace]
    t90 = None
    for i, v in enumerate(vs):
        if v >= 0.9 * target:
            t90 = (i + 1) * DT
            break
    steady = sum(vs[-20:]) / 20.0
    return dict(t90=t90,
                overshoot=(max(vs) - target) / target * 100.0,
                steady=steady,
                err=(target - steady) / target * 100.0)


def ascii_plot(trace, target, width=62, height=12):
    vs = [v for v, _ in trace][:width]
    vmax = max(max(vs), target) * 1.15
    out = []
    for r in range(height, -1, -1):
        lo, hi = vmax * r / height, vmax * (r + 1) / height
        line = ''.join('#' if lo <= v < hi else ' ' for v in vs)
        out.append('|' + line + ('  <- 目标' if lo <= target < hi else ''))
    return '\n'.join(out)


def row(name, trace, target):
    m = metrics(trace, target)
    print('%-22s %8s %10.2f%% %10.3f %9.2f%%' % (
        name, '%.2fs' % m['t90'] if m['t90'] else '--',
        m['overshoot'], m['steady'], m['err']))


def section1():
    target = 35.0
    print('=' * 78)
    print('第 1 节  位置式 vs 增量式 (目标 35 脉冲/拍, PWM 限幅 +-100)')
    print('=' * 78)
    print("""
两种写法:

  位置式:  Out = Kp*e + Ki*Σe + Kd*(e - e1)
  增量式:  du  = Kp*(e - e1) + Ki*e + Kd*(e - 2*e1 + e2),  Out += du

先做一次数学对照 —— 把位置式的相邻两拍相减:
  Out_k - Out_(k-1) = Kp*(e_k - e_(k-1)) + Ki*e_k + Kd*(e_k - 2*e_(k-1) + e_(k-2))

右边恰好就是增量式的 du。所以结论是:
  增量式 = 位置式 的差分量再累加回去, 两者是同一个控制器。
""")

    KP, KI, KD = 0.60, 0.12, 0.02
    ticks = 500

    print('-' * 78)
    print('对照 1: 不加任何限幅, 两者应该逐拍完全相同')
    print('-' * 78)
    pos = PositionForm(KP, KI, KD, out_max=1e9, anti_windup=False)
    inc = IncrementalForm(KP, KI, KD, out_max=1e9)
    tp = [p for _, p in run_speed_loop(pos, target, ticks)]
    ti = [p for _, p in run_speed_loop(inc, target, ticks)]
    diff = max(abs(a - b) for a, b in zip(tp, ti))
    print('  500 拍内, 两条 PWM 曲线最大差异 = %.3e' % diff)
    print('  判定: %s' % ('完全一致, 证实前面的推导' if diff < 1e-9 else '不一致, 推导有误'))

    print()
    print('-' * 78)
    print('对照 2: 加上 PWM 限幅 +-100, 差别才真正出现')
    print('-' * 78)
    print('%-22s %8s %11s %10s %9s' % ('形式', '上升时间', '超调', '稳态值', '稳态误差'))
    row('位置式 (带抗饱和)', run_speed_loop(PositionForm(KP, KI, KD), target, ticks), target)
    row('位置式 (无抗饱和)', run_speed_loop(PositionForm(KP, KI, KD, anti_windup=False), target, ticks), target)
    row('增量式', run_speed_loop(IncrementalForm(KP, KI, KD), target, ticks), target)

    print()
    print('-' * 78)
    print('对照 3: 把 Ki 放大到 2.0 (整定过头), 位置式积分卷绕的代价')
    print('-' * 78)
    print('%-22s %8s %11s %10s %9s' % ('形式', '上升时间', '超调', '稳态值', '稳态误差'))
    bigK = 2.0
    tr_nowind = run_speed_loop(PositionForm(KP, bigK, KD, anti_windup=False), target, ticks)
    tr_wind = run_speed_loop(PositionForm(KP, bigK, KD, anti_windup=True), target, ticks)
    tr_inc = run_speed_loop(IncrementalForm(KP, bigK, KD), target, ticks)
    row('位置式 (无抗饱和)', tr_nowind, target)
    row('位置式 (带抗饱和)', tr_wind, target)
    row('增量式', tr_inc, target)

    print()
    print('位置式(无抗饱和) 曲线, 注意冲过头后要花很久才收回:')
    print(ascii_plot(tr_nowind, target))
    print()
    print('位置式(带抗饱和) 曲线:')
    print(ascii_plot(tr_wind, target))

    print("""
结论:
  * 不限幅时两者输出逐拍相同, 是同一个控制器 —— 不存在"我用错了形式"这回事。
  * 差别只在饱和之后: 位置式把误差一直累进 Σe, 输出被顶在限幅上还在累,
    等误差反号要先把这堆积分"还完"才肯回头, 这就是积分卷绕 (windup)。
  * 增量式的累加器被限幅夹住, 天然不会攒出超额积分, 但它也因此和真实输出失配,
    切模式/断电丢状态时更难恢复。
  * 所以工程上位置式必须配抗饱和。你的 pid.c 用的是"饱和反算"(back-calculation),
    yaw_hold.c 用的是"同向停止积分", 两种都是标准解法, 这里都能收敛。
""")


# ================= 第 2 节: 位置环闭环验证 =================
# 以下常量与 Lib/Src/pos_loop.c 严格一一对应, 改一边记得改另一边
WHEEL_TRACK = 0.115
WHEEL_PERIMETER = 0.2042
ENCODER_CPR = 1456.0
PULSE_PER_M = ENCODER_CPR / WHEEL_PERIMETER
DEG2RAD = 0.01745329252
RAD2DEG = 57.2957795

DEFAULT_CFG = dict(
    dist=dict(kp=0.80, ki=0.15, kd=1.60),
    yaw=dict(kp=2.00, ki=0.30, kd=0.05),
    v_max=0.35,
    w_max=1.20,
    dist_i_max=1.5,
    yaw_i_max=1.0,
    dist_tol_m=0.005,
    yaw_tol_deg=1.5,
)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PosLoop(object):
    """Lib/Src/pos_loop.c 的逐行镜像。"""

    def __init__(self, cfg=None):
        self.cfg = cfg or DEFAULT_CFG
        self.reset()

    def reset(self):
        self.active = self.done = False
        self.odom_dist = self.odom_yaw = 0.0
        self.tgt_dist = self.tgt_yaw = 0.0
        self.d_err = self.d_err_prev = self.d_int = 0.0
        self.y_err = self.y_err_prev = self.y_int = 0.0
        self.out_v = self.out_w = 0.0

    def start(self, dist_m, yaw_delta_deg):
        self.reset()
        c = self.cfg
        self.tgt_dist = dist_m
        self.tgt_yaw = yaw_delta_deg * DEG2RAD
        if abs(dist_m) < c['dist_tol_m'] and abs(yaw_delta_deg) < c['yaw_tol_deg']:
            self.done = True
            return
        self.active = True

    def feed(self, pulse_L, pulse_R):
        c = self.cfg
        if not self.active:
            return False

        dL = pulse_L / PULSE_PER_M
        dR = pulse_R / PULSE_PER_M
        self.odom_dist += (dL + dR) * 0.5
        self.odom_yaw += (dR - dL) / WHEEL_TRACK

        # 距离环 -> V
        self.d_err_prev = self.d_err
        self.d_err = self.tgt_dist - self.odom_dist
        if c['dist']['ki'] != 0.0:
            self.d_int += self.d_err
            self.d_int = clamp(self.d_int, -c['dist_i_max'], c['dist_i_max'])
        else:
            self.d_int = 0.0
        self.out_v = clamp(c['dist']['kp'] * self.d_err
                           + c['dist']['ki'] * self.d_int
                           + c['dist']['kd'] * (self.d_err - self.d_err_prev),
                           -c['v_max'], c['v_max'])
        if self.out_v >= c['v_max'] and self.d_err > 0.0:
            self.d_int -= self.d_err
        if self.out_v <= -c['v_max'] and self.d_err < 0.0:
            self.d_int -= self.d_err

        # 航向环 -> W
        self.y_err_prev = self.y_err
        self.y_err = wrap_pi(self.tgt_yaw - self.odom_yaw)
        if c['yaw']['ki'] != 0.0:
            self.y_int += self.y_err
            self.y_int = clamp(self.y_int, -c['yaw_i_max'], c['yaw_i_max'])
        else:
            self.y_int = 0.0
        self.out_w = clamp(c['yaw']['kp'] * self.y_err
                           + c['yaw']['ki'] * self.y_int
                           + c['yaw']['kd'] * (self.y_err - self.y_err_prev),
                           -c['w_max'], c['w_max'])
        if self.out_w >= c['w_max'] and self.y_err > 0.0:
            self.y_int -= self.y_err
        if self.out_w <= -c['w_max'] and self.y_err < 0.0:
            self.y_int -= self.y_err

        if (abs(self.d_err) < c['dist_tol_m']
                and abs(self.y_err) < c['yaw_tol_deg'] * DEG2RAD):
            self.done = True
            self.active = False
            self.out_v = self.out_w = 0.0
            return True
        return False


class Chassis(object):
    """底盘: 速度环一阶跟随 V/W 指令, 右轮有固定增益偏差。

    关键: 位姿必须由"两轮实际速度"积分出来, 不能直接用标称 V/W 积分,
          否则里程计和真实位姿会各说各话, 测出来的偏差没有物理意义。
    """

    def __init__(self, trim=1.0, tau_v=0.08, tau_w=0.06):
        self.trim = trim
        self.tau_v = tau_v
        self.tau_w = tau_w
        self.v = self.w = 0.0
        self.x = self.y = self.th = 0.0

    def step(self, V, W):
        self.v += (DT / self.tau_v) * (V - self.v)
        self.w += (DT / self.tau_w) * (W - self.w)

        # 右轮增益偏差作用在"右轮线速度"上
        vL = self.v - self.w * WHEEL_TRACK * 0.5
        vR = (self.v + self.w * WHEEL_TRACK * 0.5) * self.trim

        v_act = (vL + vR) * 0.5
        w_act = (vR - vL) / WHEEL_TRACK

        self.x += v_act * math.cos(self.th) * DT
        self.y += v_act * math.sin(self.th) * DT
        self.th += w_act * DT

        # 编码器读数 = 两轮实际行程, 取整模拟量化
        pL = round(vL * DT / WHEEL_PERIMETER * ENCODER_CPR)
        pR = round(vR * DT / WHEEL_PERIMETER * ENCODER_CPR)
        return pL, pR


def run_pos_loop(dist_m, yaw_deg, cfg=None, trim=1.05, max_ticks=4000):
    loop = PosLoop(cfg)
    ch = Chassis(trim=trim)
    loop.start(dist_m, yaw_deg)
    ticks = 0
    while loop.active and ticks < max_ticks:
        pL, pR = ch.step(loop.out_v, loop.out_w)
        loop.feed(pL, pR)
        ticks += 1
    return dict(ticks=ticks, time_s=ticks * DT,
                dist=loop.odom_dist, dist_err=loop.tgt_dist - loop.odom_dist,
                yaw_deg=loop.odom_yaw * RAD2DEG,
                yaw_err_deg=(loop.tgt_yaw - loop.odom_yaw) * RAD2DEG,
                lateral=abs(ch.y), timeout=(ticks >= max_ticks))


def run_straight(distance, trim, use_yaw_loop, max_ticks=4000):
    """直行固定距离, 比较"关掉航向环"和"开着航向环"走出来的轨迹。"""
    cfg = dict(DEFAULT_CFG)
    if not use_yaw_loop:
        cfg['yaw'] = dict(kp=0.0, ki=0.0, kd=0.0)
    loop = PosLoop(cfg)
    ch = Chassis(trim=trim)
    loop.start(distance, 0.0)
    for _ in range(max_ticks):
        pL, pR = ch.step(loop.out_v, loop.out_w)
        loop.feed(pL, pR)
        if abs(loop.odom_dist) >= distance:
            break
    return abs(ch.y), abs(ch.th) * RAD2DEG


def section2():
    print()
    print('=' * 78)
    print('第 2 节  位置环闭环验证 (Lib/Src/pos_loop.c 的算法)')
    print('=' * 78)
    print("""
场景: 右轮比左轮快 5% (trim=1.05), 模拟真实的左右轮速不一致。
      里程计由编码器脉冲算出, 位姿由两轮实际速度积分 —— 两者物理一致。
""")
    print('%-24s %8s %10s %11s %10s %10s' % (
        '任务', '用时', '实际距离', '距离误差', '实际转角', '转角误差'))
    print('-' * 78)

    ok = True
    for name, d, y in (('直行 1.00 m', 1.00, 0.0),
                       ('直行 0.50 m', 0.50, 0.0),
                       ('原地转 +90 度', 0.0, 90.0),
                       ('原地转 -45 度', 0.0, -45.0),
                       ('前进 0.80 m + 转 45 度', 0.80, 45.0)):
        r = run_pos_loop(d, y)
        print('%-24s %7.2fs %9.3fm %10.4fm %9.2f° %9.3f°' % (
            name, r['time_s'], r['dist'], r['dist_err'], r['yaw_deg'], r['yaw_err_deg']))
        if r['timeout']:
            print('    !! 超时未到位')
            ok = False
        if abs(r['dist_err']) > 0.02:
            print('    !! 距离误差 > 2cm')
            ok = False
        if abs(r['yaw_err_deg']) > 4.0:
            print('    !! 转角误差 > 4 度')
            ok = False

    print()
    print('直行 1.00 m 时航向环的作用 (右轮快 5%, 开环会往左画弧):')
    print('%-14s %16s %16s' % ('航向环', '终点横向偏移', '终点航向'))
    for use in (False, True):
        lat, th = run_straight(1.00, 1.05, use)
        print('%-14s %15.2f cm %15.2f°' % ('开' if use else '关', lat * 100.0, th))

    print()
    print('直行 1.00 m, 不同轮速偏差下开环的横向跑偏:')
    for trim in (1.00, 1.02, 1.05, 1.10):
        lat, th = run_straight(1.00, trim, False)
        print('  右轮偏差 %+5.1f%%  →  横向偏移 %6.2f cm, 航向偏 %5.2f°' % (
            (trim - 1.0) * 100.0, lat * 100.0, th))

    print()
    print('默认增益判据: %s' % ('全部通过' if ok else '有不达标项'))
    return ok


def main():
    section1()
    ok = section2()
    print()
    print('=' * 78)
    print('总判定: %s' % ('PASS' if ok else 'FAIL'))
    print('=' * 78)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())