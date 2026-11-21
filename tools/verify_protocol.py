#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""协议自检 —— docs/chassis_protocol_spec.md 第 11 节的自动化版本。

用法:  python tools/verify_protocol.py

它不是“再跑一遍固件”，而是用一份**独立实现**去核对契约：

  1. 从 protocol.h 读消息 ID / 载荷长度，确认代码与文档是同一套数
  2. 用独立 CRC 实现复算文档里的示例帧（考官现场就是这么查的）
  3. 检查上位机脚本声明的 STATUS_PAYLOAD_LEN / 帧头是否与固件一致
  4. 如果上位机脚本能在本机 exec，就把示例帧喂给它的 decode_status() 做往返

退出码 0 = 全通过，1 = 有失败项。
"""
import io
import os
import re
import struct
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

FIRMWARE_H = os.path.join(REPO, "Lib", "comm", "inc", "protocol.h")
FIRMWARE_C = os.path.join(REPO, "Lib", "comm", "src", "protocol.c")
SPEC = os.path.join(REPO, "docs", "chassis_protocol_spec.md")

DESKTOP = os.path.join(os.path.expanduser("~"), "Desktop")
ASSESS = os.path.join(DESKTOP, "车队", "考核")

HOST_SCRIPTS = [
    ("vmware/chassis_link_test.py",
     os.path.join(DESKTOP, "vmware share", "chassis_link_test.py"), None),
    ("scripts/check.py",
     os.path.join(DESKTOP, "脚本", "check.py"), None),
    ("scripts/pi_check.py",
     os.path.join(DESKTOP, "脚本", "pi_check.py"), "ser = serial.Serial("),
    ("hmi/chassis_link_test.py",
     os.path.join(ASSESS, "M4", "slam-car-hmi", "bridge",
                  "chassis_link_test.py"), None),
]

STATIC_SCRIPTS = [
    ("hmi/chassis_driver.py",
     os.path.join(ASSESS, "M4", "slam-car-hmi", "bridge",
                  "chassis_driver.py")),
]

failures = []


def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print("  [%s] %s%s" % (tag, name, (" -- " + detail) if detail else ""))
    if not cond:
        failures.append(name)
    return cond


# ---------------------------------------------------------------- 独立实现
def crc16(data):
    """CRC-16/MODBUS: init 0xFFFF, reflected poly 0xA001, no final xor."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(msg_id, payload, seq=0):
    body = bytes([0xAA, 0x55, len(payload), msg_id, seq]) + bytes(payload)
    c = crc16(body)
    return body + bytes([c & 0xFF, (c >> 8) & 0xFF])


def build_status(**kw):
    """按协议顺序打包 24 字节状态载荷。"""
    payload = struct.pack(
        "<hhhhHHHHBBHhh",
        kw.get("left", 60),
        kw.get("right", 62),
        kw.get("pitch", 123),
        kw.get("yaw", 9050),
        kw.get("heartbeat", 1000),
        kw.get("bad", 2),
        kw.get("timeout", 1),
        kw.get("overflow", 0),
        kw.get("status", 0x07),
        0,
        kw.get("batt", 12100),
        kw.get("yaw_rate", 1500),
        kw.get("yaw_out", -250),
    )
    return build_frame(0x02, payload, kw.get("seq", 0))


def hex_text(data):
    return " ".join("%02X" % b for b in data)


def load_host_script(path, stop_at=None, name="loaded"):
    """exec 上位机脚本源码以取出纯函数；pyserial 被打桩，不碰真实串口。"""
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


def parse_define(src, name):
    m = re.search(r"#define\s+%s\s+(\d+)[uU]?" % re.escape(name), src)
    return int(m.group(1)) if m else None


def main():
    print("== 1. 固件常量 ==")
    hsrc = io.open(FIRMWARE_H, encoding="utf-8").read()
    csrc = io.open(FIRMWARE_C, encoding="utf-8").read()

    status_len = parse_define(hsrc, "PROTO_STATUS_LEN")
    tele_len = parse_define(hsrc, "PROTO_TELEMETRY_LEN")
    max_payload = parse_define(hsrc, "PROTO_MAX_PAYLOAD")
    overhead = parse_define(hsrc, "PROTO_FRAME_OVERHEAD")
    print("  PROTO_STATUS_LEN=%s TELEMETRY_LEN=%s MAX_PAYLOAD=%s OVERHEAD=%s"
          % (status_len, tele_len, max_payload, overhead))

    check("状态载荷 24 B", status_len == 24, str(status_len))
    check("遥测载荷 12 B", tele_len == 12, str(tele_len))
    check("最大载荷 32 B", max_payload == 32, str(max_payload))
    check("帧开销 7 B", overhead == 7, str(overhead))
    check("payload 上限 < 255", max_payload is not None and max_payload < 255)
    check("protocol.c 有 CRC-16/MODBUS 反射实现",
          "0xA001" in csrc and "0xFFFF" in csrc)
    check("protocol.c 按帧头..载荷算 CRC",
          "PROTO_HEAD1" in csrc and "PROTO_HEAD2" in csrc)

    print("== 2. 文档示例帧独立复算 ==")
    spec = io.open(SPEC, encoding="utf-8").read()
    spec_flat = " ".join(spec.split())

    velocity = build_frame(0x01, struct.pack("<hh", 60, 60), 0)
    status = build_status()
    telemetry = build_frame(
        0x10, struct.pack("<hhhhhh", 60, 58, 35, 60, 61, 33), 0)
    clear = build_frame(0x03, b"\x5A", 0)
    preset = build_frame(0x11, b"\x02", 0)
    gains = build_frame(
        0x12, struct.pack("<hhhhhh", 1200, 300, 50, 1200, 300, 50), 0)

    golden = [
        ("速度帧 60/60", velocity, "AA 55 04 01 00 3C 00 3C 00 10 15"),
        ("状态帧示例", status,
         "AA 55 18 02 00 3C 00 3E 00 7B 00 5A 23 E8 03 02 00 "
         "01 00 00 00 07 00 44 2F DC 05 06 FF A8 9D"),
        ("遥测帧示例", telemetry,
         "AA 55 0C 10 00 3C 00 3A 00 23 00 3C 00 3D 00 21 00 EE 08"),
        ("清计数帧", clear, "AA 55 01 03 00 5A 65 DA"),
        ("PID 预设帧", preset, "AA 55 01 11 00 02 C4 25"),
        ("PID 增益帧", gains,
         "AA 55 0C 12 00 B0 04 2C 01 32 00 B0 04 2C 01 32 00 71 4E"),
    ]
    for label, data, expect in golden:
        check("%s 自算 == 文档" % label, hex_text(data) == expect,
              hex_text(data))
        check("%s 出现在文档里" % label, expect in spec_flat)
        check("%s 帧尾 CRC 可独立复算" % label,
              crc16(data[:-2]) == (data[-2] | (data[-1] << 8)))

    check("CRC 参考向量 123456789 -> 0x4B37",
          crc16(b"123456789") == 0x4B37,
          "0x%04X" % crc16(b"123456789"))

    print("== 3. 上位机脚本声明 ==")
    loaded = []
    for label, path, stop_at in HOST_SCRIPTS:
        if not os.path.isfile(path):
            print("  [SKIP] %s 不在本机: %s" % (label, path))
            continue
        src = io.open(path, encoding="utf-8").read()
        m = re.search(r"STATUS_PAYLOAD_LEN\s*=\s*(\d+)", src)
        declared = int(m.group(1)) if m else None
        print("  %-26s STATUS_PAYLOAD_LEN = %s" % (label, declared))
        check("%-26s 状态载荷长度 == 24" % label, declared == 24, str(declared))
        check("%-26s 使用 AA55 帧头" % label,
              "0xAA" in src and "0x55" in src)
        check("%-26s 有 CRC16 实现" % label, "crc16" in src.lower())

        try:
            g = load_host_script(path, stop_at=stop_at,
                                 name="host_" + re.sub(r"\W", "_", label))
        except Exception as exc:
            check("%-26s 可加载" % label, False, repr(exc))
            continue

        dec = g.get("decode_status")
        bld = g.get("build_velocity")
        check("%-26s 提供 decode_status()" % label, callable(dec))
        check("%-26s 提供 build_velocity()" % label, callable(bld))
        if not (callable(dec) and callable(bld)):
            continue
        loaded.append((label, g))

        d = dec(status)
        check("%-26s 状态帧解回 left/right" % label,
              (d.get("left"), d.get("right")) == (60, 62), str(d))
        check("%-26s 状态帧解回电压/偏航" % label,
              d.get("batt") == 12100 and d.get("yaw", 0) == 90.5, str(d))

        # 用上位机自己的组帧器造帧，再用独立解析器验一遍
        f = bld(60, 60)
        check("%-26s build_velocity 帧头/长度正确" % label,
              f[:5] == b"\xAA\x55\x04\x01\x00", hex_text(f))
        check("%-26s build_velocity CRC 正确" % label,
              crc16(f[:-2]) == (f[-2] | (f[-1] << 8)), hex_text(f))

    for label, path in STATIC_SCRIPTS:
        if not os.path.isfile(path):
            print("  [SKIP] %s 不在本机: %s" % (label, path))
            continue
        src = io.open(path, encoding="utf-8").read()
        m = re.search(r"STATUS_PAYLOAD_LEN\s*=\s*(\d+)", src)
        declared = int(m.group(1)) if m else None
        print("  %-26s STATUS_PAYLOAD_LEN = %s  (静态)" % (label, declared))
        check("%-26s 状态载荷长度 == 24" % label, declared == 24, str(declared))
        check("%-26s 使用 AA55 帧头" % label,
              "0xAA" in src and "0x55" in src)
        check("%-26s 有 CRC16 实现/调用" % label, "crc16" in src.lower())

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
