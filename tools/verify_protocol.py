#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""协议自检 —— docs/chassis_protocol_spec.md 第 9 节的自动化版本。

用法:  python tools/verify_protocol.py

做四件事:
  1. 读固件 protocol.h / protocol.c, 取出布局表标注的帧长与 TX_LEN / RX_LEN
  2. 读三个上位机解析脚本, 取出各自的 FRAME_RX
  3. 检查数值是否一致, 且上位机侧没有第二份轮距常量 (规范 §6)
  4. 按固件字节序造帧喂给三个解析器, 校验往返一致与冗余量不变量 (规范 §5)

退出码 0 = 全通过, 1 = 有失败项。
"""
import io
import os
import re
import sys
import types
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

FIRMWARE_H = os.path.join(REPO, "Lib", "Inc", "protocol.h")
FIRMWARE_C = os.path.join(REPO, "Lib", "Src", "protocol.c")

SLAM_HMI = os.path.join(os.path.expanduser("~"), "Documents", "Codex",
                        "2026-09-11", "ni", "outputs", "slam-car-hmi")

# 能被 exec 的上位机解析脚本: (显示名, 路径, 加载时截断到哪个字符串)
# 不在本机的会被跳过而不是报错
HOST_SCRIPTS = [
    ("chassis_link_test.py", os.path.join(os.path.expanduser("~"), "Desktop", "vmware share", "chassis_link_test.py"), None),
    ("check.py",             os.path.join(os.path.expanduser("~"), "Desktop", "脚本", "check.py"), None),
    ("pi_check.py",          os.path.join(os.path.expanduser("~"), "Desktop", "脚本", "pi_check.py"), "ser = serial.Serial("),
    ("hmi/chassis_link_test.py", os.path.join(SLAM_HMI, "bridge", "chassis_link_test.py"), None),
]

# 依赖 rclpy 的 ROS2 节点, Windows 上 exec 不了, 只做静态帧长检查
STATIC_SCRIPTS = [
    ("hmi/chassis_driver.py", os.path.join(SLAM_HMI, "bridge", "chassis_driver.py")),
]

TRACK_M = 0.115          # 轮距, 只作造帧用; 上位机不得出现这个数 (规范 §6)

failures = []
def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print("  [%s] %s%s" % (tag, name, (" -- " + detail) if detail else ""))
    if not cond:
        failures.append(name)
    return cond


# ---------------------------------------------------------------- 造帧
def build_frame(vx, vy, wz, acc, gyro, batt, yaw, yr, err, wc, status, vl, vr, frame_len):
    """按 protocol.h 的字节序造一帧上行帧, BCC 自算。"""
    f = bytearray(frame_len)
    def put(i, v):
        f[i] = (v >> 8) & 0xFF
        f[i + 1] = v & 0xFF
    f[0] = 0x7B
    f[1] = 0x00
    put(2, vx & 0xFFFF); put(4, vy & 0xFFFF); put(6, wz & 0xFFFF)
    for k, a in enumerate(acc):  put(8 + 2 * k, a & 0xFFFF)
    for k, g in enumerate(gyro): put(14 + 2 * k, g & 0xFFFF)
    put(20, batt & 0xFFFF)
    put(22, yaw & 0xFFFF); put(24, yr & 0xFFFF)
    put(26, err & 0xFFFF); put(28, wc & 0xFFFF)
    f[30] = status
    put(31, vl & 0xFFFF); put(33, vr & 0xFFFF)
    bcc_at, tail_at = frame_len - 2, frame_len - 1
    x = 0
    for b in f[:bcc_at]:
        x ^= b
    f[bcc_at] = x
    f[tail_at] = 0x7D
    return bytes(f)


def load_host_script(path, stop_at=None, name="loaded"):
    """exec 上位机脚本源码以取出函数; pyserial 被打桩, 不碰真实串口。"""
    src = io.open(path, encoding="utf-8").read()
    if stop_at:
        src = src[:src.index(stop_at)]
    g = {"__name__": name}
    exec(compile(src, path, "exec"), g)
    return g


def install_serial_stub():
    m = types.ModuleType("serial")
    class _Dummy(object):
        pass
    m.Serial = _Dummy
    tools = types.ModuleType("serial.tools")
    lp = types.ModuleType("serial.tools.list_ports")
    lp.comports = lambda: []
    tools.list_ports = lp
    m.tools = tools
    sys.modules["serial"] = m
    sys.modules["serial.tools"] = tools
    sys.modules["serial.tools.list_ports"] = lp


# ---------------------------------------------------------------- 主流程
def main():
    print("== 1. 固件侧帧长 ==")
    hsrc = io.open(FIRMWARE_H, encoding="utf-8").read()
    csrc = io.open(FIRMWARE_C, encoding="utf-8").read()
    m = re.search(r"(\d+)\s*字节上行反馈帧", hsrc)
    doc_len = int(m.group(1)) if m else None
    m = re.search(r"#define\s+TX_LEN\s+(\d+)U", csrc)
    tx_len = int(m.group(1)) if m else None
    m = re.search(r"#define\s+RX_LEN\s+(\d+)U", csrc)
    rx_len = int(m.group(1)) if m else None
    print("  protocol.h 布局表帧长 = %s ; TX_LEN = %s ; RX_LEN = %s" % (doc_len, tx_len, rx_len))
    check("protocol.h 布局表帧长 == protocol.c TX_LEN", doc_len == tx_len,
          "%s vs %s" % (doc_len, tx_len))
    check("RX_LEN == 11 (下行帧长)", rx_len == 11, str(rx_len))
    frame_len = tx_len

    print("== 2. 上位机脚本 ==")
    loaded = []
    for label, path, stop_at in HOST_SCRIPTS:
        if not os.path.isfile(path):
            print("  [SKIP] %s 不在本机: %s" % (label, path))
            continue
        g = load_host_script(path, stop_at=stop_at,
                             name="host_" + re.sub(r"\W", "_", label))
        fr = g.get("FRAME_RX")
        print("  %-26s FRAME_RX = %s" % (label, fr))
        loaded.append((label, path, g, fr))
        check("%-26s FRAME_RX == 固件 TX_LEN" % label, fr == frame_len,
              "%s vs %s" % (fr, frame_len))
        src = io.open(path, encoding="utf-8").read()
        hard = re.findall(r"(\bTRACK\s*=\s*[0-9.]+|0\.115)", src)
        check("%-26s 无重复轮距常量 (规范 §6)" % label, not hard, str(hard))

    for label, path in STATIC_SCRIPTS:
        if not os.path.isfile(path):
            print("  [SKIP] %s 不在本机: %s" % (label, path))
            continue
        src = io.open(path, encoding="utf-8").read()
        m = re.search(r"RX_LEN\s*=\s*(\d+)", src)
        rx = int(m.group(1)) if m else None
        m = re.search(r"RX_BCC_IDX\s*=\s*(\d+)", src)
        bidx = int(m.group(1)) if m else None
        print("  %-26s RX_LEN = %s ; RX_BCC_IDX = %s  (静态)" % (label, rx, bidx))
        check("%-26s RX_LEN == 固件 TX_LEN" % label, rx == frame_len, "%s vs %s" % (rx, frame_len))
        check("%-26s RX_BCC_IDX == 帧长-2" % label, bidx == frame_len - 2,
              "%s vs %s" % (bidx, frame_len - 2))
        hard = re.findall(r"(\bTRACK\s*=\s*[0-9.]+|0\.115)", src)
        check("%-26s 无重复轮距常量 (规范 §6)" % label, not hard, str(hard))

    print("== 3. 往返解算与冗余量不变量 (规范 §5) ==")
    if len(loaded) == 0:
        print("  [SKIP] 本机没有可用的上位机脚本")
    else:
        label, path, g, fr = loaded[0]
        dec = g.get("decode")
        if dec is None:
            check("%s 提供 decode()" % label, False)
        else:
            cases = [("fwd", 300, 300), ("back", -300, -300), ("spin-left", -57, 57),
                     ("arc", 300, 180), ("stop", 0, 0)]
            for nm, vl, vr in cases:
                vx = int(round((vl + vr) / 2.0))
                wz = int(round((vr - vl) / TRACK_M))
                fr_bytes = build_frame(vx, 0, wz, (11, 22, 33), (44, 55, 66),
                                       12120, 1234, -567, -50, 88, 0x07, vl, vr, frame_len)
                d = dec(fr_bytes)
                good = (abs(d["vx"] - vx) <= 1 and abs(d["wz"] - wz) <= 1
                        and abs(d["vl"] * 1000.0 - vl) < 1e-6
                        and abs(d["vr"] * 1000.0 - vr) < 1e-6)
                check("往返 %-10s vl=%+5d vr=%+5d" % (nm, vl, vr), good,
                      "-> vx=%+.1f wz=%+.1f vl=%+.1f vr=%+.1f" % (d["vx"], d["wz"],
                                                                  d["vl"] * 1000.0, d["vr"] * 1000.0))
            # status 位
            gears = []
            for i in range(3):
                b = build_frame(0, 0, 0, (0, 0, 0), (0, 0, 0), 12000, 0, 0, 0, 0,
                                0x01 | 0x02 | (i << 2), 0, 0, frame_len)
                gears.append(dec(b)["gear"])
            check("status 挡位解码 = 1/2/3", gears == [1, 2, 3], str(gears))

    print("== 4. 帧头 / 帧尾 / BCC 校验 ==")
    if loaded:
        good = build_frame(300, 0, 0, (0, 0, 0), (0, 0, 0), 12000,
                           0, 0, 0, 0, 0x07, 300, 300, frame_len)
        bad = bytearray(good)
        bad[5] ^= 0xFF
        short = good[:frame_len - 1]

        def verdict(g, data):
            """喂一帧, 返回 'accept' / 'reject'。覆盖两种调用形态。"""
            Link = g.get("Link")
            if Link is not None:                       # chassis_link_test.py: 类方法
                obj = Link(None)
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    obj.buf.extend(data)
                    obj.parse()
                return "accept" if obj.frames else "reject"
            fn = g.get("parse")
            if fn is None:
                return "skip"
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                res = fn(bytes(data))
            text = str(res or "") + out.getvalue()
            if "不符" in text or "失败" in text:
                return "reject"
            return "accept" if ("stop=" in text or "vx=" in text) else "reject"

        for label, path, g, fr in loaded:
            check("%-22s 接受好帧" % label, verdict(g, good) == "accept",
                  verdict(g, good))
            check("%-22s 拒绝 BCC 篡改帧" % label, verdict(g, bytes(bad)) == "reject",
                  verdict(g, bytes(bad)))
            check("%-22s 拒绝截断帧" % label, verdict(g, short) == "reject",
                  verdict(g, short))

    print()
    if failures:
        print("RESULT: FAIL (%d 项)" % len(failures))
        for f in failures:
            print("  - " + f)
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    install_serial_stub()
    sys.exit(main())
