#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 M4-3 的 CSV 计算阶跃指标。

用法:
    python3 analyze_step.py --input ../data/step_PID.csv --ref 60
    python3 analyze_step.py --input ../data/disturb_PID.csv --ref 60 --disturb
"""
import argparse
import csv
import math
import statistics
import sys


def load(path):
    ts, ref, act, pwm = [], [], [], []
    with open(path, encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts.append(float(row["t"]))
            ref.append(float(row["target_rpm"]))
            act.append(float(row["actual_rpm"]))
            pwm.append(float(row["pwm"]))
    return ts, ref, act, pwm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--ref", type=float, default=None,
                    help="目标值, 默认取 CSV 最后一行的 target_rpm")
    ap.add_argument("--window", type=float, default=2.0,
                    help="稳态统计窗口, 秒")
    ap.add_argument("--band", type=float, default=5.0,
                    help="调节时间判据, 目标值的百分比 (默认 ±5%%)")
    ap.add_argument("--band-min", type=float, default=6.0,
                    help="调节时间判据的绝对下限 r/min "
                         "(10ms 内 1 个脉冲约 4.12 r/min, 带太窄永远进不去)")
    ap.add_argument("--disturb", action="store_true",
                    help="抗扰实验模式: 统计捏停深度、松手后恢复时间与过冲")
    args = ap.parse_args()

    ts, ref, act, pwm = load(args.input)
    if not ts:
        sys.exit("CSV 为空")

    ref_val = args.ref if args.ref is not None else ref[-1]
    if ref_val == 0:
        sys.exit("目标值为 0, 无法计算相对指标; 请用 --ref 指定非零目标")

    # 统一投影到"正方向", 这样反转目标也能用同一套公式
    sign = 1.0 if ref_val > 0 else -1.0
    ref_abs = abs(ref_val)
    proj = [a * sign for a in act]

    band_abs = max(ref_abs * args.band / 100.0, args.band_min)
    t_end = ts[-1]
    steady = [p for t, p in zip(ts, proj) if t >= t_end - args.window]
    if not steady:
        sys.exit("稳态窗口内没有数据")

    steady_mean = statistics.fmean(steady)
    steady_err = (steady_mean - ref_abs) * sign
    steady_ripple = max(steady) - min(steady)

    print("文件      :", args.input)
    print("目标      : %.2f r/min" % ref_val)
    print("稳态误差  : %+.2f r/min" % steady_err)
    print("稳态波动  : %.2f r/min (峰峰值)" % steady_ripple)

    if args.disturb:
        # 抗扰: 找转速跌落最深的位置(近似松手时刻), 再看回到 ±band 并保持的时间
        lo = max(1, len(proj) // 10)
        hi = max(lo + 1, len(proj) * 9 // 10)
        iv = min(range(lo, hi), key=lambda k: proj[k])
        depth = ref_abs - proj[iv]
        tail = proj[iv:]
        peak = max(tail)
        overshoot = max(0.0, (peak - ref_abs) / ref_abs * 100.0)

        settle_t = None
        last_out = iv
        for k in range(iv, len(proj)):
            if abs(proj[k] - ref_abs) > band_abs:
                last_out = k
        if last_out + 1 < len(proj):
            settle_t = ts[last_out + 1] - ts[iv]

        print("捏停深度  : %.2f r/min (最低到 %.2f)" % (depth, proj[iv] * sign))
        print("恢复时间  : %s (回到 ±%.1f r/min)"
              % ("%.0f ms" % (settle_t * 1000.0) if settle_t is not None else "未恢复",
                 band_abs))
        print("松手过冲  : %.2f %%" % overshoot)
        return

    peak = max(proj)
    overshoot = max(0.0, (peak - ref_abs) / ref_abs * 100.0)

    t10 = t90 = None
    for t, p in zip(ts, proj):
        if t10 is None and p >= 0.1 * ref_abs:
            t10 = t
        if t10 is not None and p >= 0.9 * ref_abs:
            t90 = t
            break
    rise_ms = (t90 - t10) * 1000.0 if t10 is not None and t90 is not None else float("nan")

    # 调节时间: 最后一次超出 ±band 之后不再出去
    last_out = 0
    for k, p in enumerate(proj):
        if abs(p - ref_abs) > band_abs:
            last_out = k
    settle_t = ts[last_out + 1] - ts[0] if last_out + 1 < len(ts) else None

    print("超调量    : %.2f %%" % overshoot)
    if math.isnan(rise_ms):
        print("上升时间  : 未达到 (10%->90%)")
    else:
        print("上升时间  : %.0f ms (10%%->90%%)" % rise_ms)
    print("调节时间  : %s (±%.1f r/min)"
          % ("%.0f ms" % (settle_t * 1000.0) if settle_t is not None else "未进入",
             band_abs))


if __name__ == "__main__":
    main()
