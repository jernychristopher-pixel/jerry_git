#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按目标变化自动分段, 统计 M4-3 变速序列每段的跟踪质量。

用法:
    python3 tools/analyze_sequence.py --input data/seq_PID.csv
"""
import argparse
import csv
import statistics
import sys


def load(path):
    ts, ref, act = [], [], []
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            ts.append(float(row["t"]))
            ref.append(float(row["target_rpm"]))
            act.append(float(row["actual_rpm"]))
    return ts, ref, act


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--band", type=float, default=5.0,
                    help="恢复判据, 目标值的百分比")
    ap.add_argument("--band-min", type=float, default=6.0,
                    help="恢复判据的绝对下限 r/min")
    ap.add_argument("--window", type=float, default=1.0,
                    help="每段末尾稳态统计窗口, 秒")
    args = ap.parse_args()

    ts, ref, act = load(args.input)
    if not ts:
        sys.exit("CSV 为空")

    starts = [0]
    for i in range(1, len(ref)):
        if abs(ref[i] - ref[i - 1]) > 0.1:
            starts.append(i)
    starts.append(len(ref))

    print("文件:", args.input)
    print("%5s %9s %10s %10s %10s %9s %9s" %
          ("段", "目标", "稳态均值", "稳态误差", "峰/谷", "恢复(ms)", "超调%"))

    shown = 0
    for seg in range(len(starts) - 1):
        i0, i1 = starts[seg], starts[seg + 1]
        target = ref[i0]
        if seg == 0 and abs(target) < 0.1 and len(starts) > 2:
            continue
        prev = ref[i0 - 1] if i0 > 0 else 0.0
        t0, t1 = ts[i0], ts[i1 - 1]
        sw_start = max(t0, t1 - args.window)
        steady = [act[i] for i in range(i0, i1) if ts[i] >= sw_start]
        if not steady:
            continue

        mean = statistics.fmean(steady)
        band = max(abs(target) * args.band / 100.0, args.band_min)
        # 超调/下冲只看"第一次到达目标之后"的极值。
        # 否则 60 -> 30 的段会把段首的 60 当成 101% 超调。
        cross = i0
        if target > prev:
            for i in range(i0, i1):
                if act[i] >= target:
                    cross = i
                    break
            extreme = max(act[cross:i1])
        elif target < prev:
            for i in range(i0, i1):
                if act[i] <= target:
                    cross = i
                    break
            extreme = min(act[cross:i1])
        else:
            extreme = max(act[i0:i1])

        settle = None
        last_out = i0
        for i in range(i0, i1):
            if abs(act[i] - target) > band:
                last_out = i
        if last_out + 1 < i1:
            settle = (ts[last_out + 1] - t0) * 1000.0

        if abs(target) < 0.1:
            over = 0.0
        elif target > prev:
            over = max(0.0, (extreme - target) / abs(target) * 100.0)
        else:
            over = max(0.0, (target - extreme) / abs(target) * 100.0)

        shown += 1
        print("%5d %9.2f %10.2f %+10.2f %10.2f %9s %9.2f" %
              (shown, target, mean, mean - target, extreme,
               "%.0f" % settle if settle is not None else "未恢复", over))


if __name__ == "__main__":
    main()
