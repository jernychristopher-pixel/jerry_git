#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M4-3 现场变速序列采集(对应题目 2.5)。

考官现场给的目标序列(如 20 -> 60 -> 30 -> 0)直接填进 --seq,
脚本按 --hold 秒逐段下发 #T, 同时把 USART1 的 CSV 连续记录到一个文件。

用法:
    python3 m43_sequence.py --preset PID --seq 20,60,30,0 --hold 3 --out seq_PID.csv
"""
import argparse
import csv
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial: sudo apt install -y python3-serial")

PRESET_ID = {"P": 0, "PI": 1, "PID": 2}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--preset", choices=PRESET_ID.keys(), required=True)
    ap.add_argument("--seq", required=True,
                    help="逗号分隔的目标转速, 例: 20,60,30,0")
    ap.add_argument("--hold", type=float, default=3.0, help="每段保持秒数")
    ap.add_argument("--kp", type=float, help="覆盖预设 Kp (左右轮相同)")
    ap.add_argument("--ki", type=float, help="覆盖预设 Ki (左右轮相同)")
    ap.add_argument("--kd", type=float, help="覆盖预设 Kd (左右轮相同)")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="序列开始前在 0 目标稳定的时间")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    try:
        targets = [float(x) for x in args.seq.split(",") if x.strip() != ""]
    except ValueError:
        sys.exit("--seq 里必须是逗号分隔的数字")
    if not targets:
        sys.exit("--seq 不能为空")

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.2)

    def send(text):
        ser.write(text.encode("ascii"))
        ser.flush()

    def wait_csv(timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if len(line.split(",")) == 7:
                return True
        return False

    rows = 0
    try:
        send("#V 1\n")
        if not wait_csv():
            send("#V 1\n")
            if not wait_csv():
                sys.exit("串口没有进入 CSV 模式: 检查固件是否支持 #V 1")
        send("#S %d\n" % PRESET_ID[args.preset])
        time.sleep(0.2)
        if args.kp is not None:
            send("#A %.4f\n" % args.kp); send("#D %.4f\n" % args.kp)
        if args.ki is not None:
            send("#B %.4f\n" % args.ki); send("#E %.4f\n" % args.ki)
        if args.kd is not None:
            send("#C %.4f\n" % args.kd); send("#F %.4f\n" % args.kd)
        time.sleep(0.1)
        send("#T 0\n")
        time.sleep(args.settle)
        ser.reset_input_buffer()

        send("#T %.2f\n" % targets[0])
        seg_start = time.time()
        seg = 0
        first_t = None

        with open(args.out, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "target_rpm", "actual_rpm", "pwm"])
            while seg < len(targets):
                if time.time() - seg_start >= args.hold:
                    seg += 1
                    if seg >= len(targets):
                        break
                    send("#T %.2f\n" % targets[seg])
                    seg_start = time.time()
                    continue

                line = ser.readline().decode("ascii", errors="ignore").strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) != 7:
                    continue
                try:
                    vals = [float(x) for x in parts]
                except ValueError:
                    continue
                if first_t is None:
                    first_t = vals[0]
                w.writerow([
                    "%.3f" % (vals[0] - first_t),
                    "%.2f" % vals[1],
                    "%.2f" % vals[2],
                    "%.1f" % vals[3],
                ])
                rows += 1
    finally:
        try:
            send("#T 0\n")
            time.sleep(0.2)
            send("#X\n")
            send("#V 0\n")
            time.sleep(0.2)
        except Exception:
            pass
        ser.close()

    if rows == 0:
        sys.exit("写入 %s 失败: 0 行数据" % args.out)
    print("写入 %s, %d 行, 序列 %s r/min, 每段 %.1fs"
          % (args.out, rows, args.seq, args.hold))


if __name__ == "__main__":
    main()
