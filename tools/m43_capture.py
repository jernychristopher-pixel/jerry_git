#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M4-3 阶跃/抗扰数据采集。

在树莓派上运行, 通过 USART1 的 CSV 模式直接记录:
    t,target_rpm,actual_rpm,pwm

用法:
    python3 m43_capture.py --preset P   --target 60 --secs 8 --out step_P.csv
    python3 m43_capture.py --preset PI  --target 60 --secs 8 --out step_PI.csv
    python3 m43_capture.py --preset PID --target 60 --secs 8 --out step_PID.csv
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
    ap.add_argument("--target", type=float, default=60.0)
    ap.add_argument("--kp", type=float, help="覆盖预设 Kp (左右轮相同)")
    ap.add_argument("--ki", type=float, help="覆盖预设 Ki (左右轮相同)")
    ap.add_argument("--kd", type=float, help="覆盖预设 Kd (左右轮相同)")
    ap.add_argument("--secs", type=float, default=8.0, help="记录时长, 不含 settle")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="发阶跃前让电机在 0 目标稳定的时间")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.2)

    def cmd(text):
        ser.write(text.encode("ascii"))
        ser.flush()
        time.sleep(0.05)

    def wait_csv(timeout=2.0):
        """等到至少一行合法的 7 列 CSV, 确认 #V 1 真的生效。"""
        end = time.time() + timeout
        while time.time() < end:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if len(line.split(",")) == 7:
                return True
        return False

    rows = 0
    try:
        cmd("#V 1\n")          # 切 CSV 文本, 暂停二进制帧
        if not wait_csv():
            cmd("#V 1\n")
            if not wait_csv():
                sys.exit("串口没有进入 CSV 模式: 检查固件是否支持 #V 1")
        ser.reset_input_buffer()
        cmd("#S %d\n" % PRESET_ID[args.preset])
        time.sleep(0.2)
        if args.kp is not None:
            cmd("#A %.4f\n" % args.kp)
            cmd("#D %.4f\n" % args.kp)
        if args.ki is not None:
            cmd("#B %.4f\n" % args.ki)
            cmd("#E %.4f\n" % args.ki)
        if args.kd is not None:
            cmd("#C %.4f\n" % args.kd)
            cmd("#F %.4f\n" % args.kd)
        time.sleep(0.1)
        ser.reset_input_buffer()

        # 先在 0 目标下稳定, 再把 t=0 对齐到阶跃发出的时刻。
        # 早期版本把 settle 写成"发阶跃后丢 1 秒数据", 会把上升过程整段扔掉。
        time.sleep(args.settle)
        ser.reset_input_buffer()
        ser.write(("#T %.2f\n" % args.target).encode("ascii"))
        ser.flush()
        t0 = time.time()
        first_t = None
        with open(args.out, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "target_rpm", "actual_rpm", "pwm"])
            while True:
                if time.time() - t0 > args.secs:
                    break
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
            cmd("#T 0\n")
            cmd("#X\n")
            cmd("#V 0\n")
            time.sleep(0.2)
        except Exception:
            pass
        ser.close()

    if rows == 0:
        sys.exit("写入 %s 失败: 0 行数据" % args.out)
    print("写入 %s, %d 行" % (args.out, rows))


if __name__ == "__main__":
    main()
