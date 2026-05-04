#!/usr/bin/env python3
"""
选手端仿真: 把 minimal_hik_gimbal_bridge 输出到串口的 0x0310 裁判帧
→ 解析 → 提取 300B data → 包成 protobuf CustomByteBlock → 通过 MQTT publish
等价于实战中: 图传选手端读取裁判帧后转发到自定义客户端订阅的 MQTT topic.

用法:
    python3 player_end_emulator.py --serial /tmp/recv_uart --baud 921600 \
        --mqtt-host 127.0.0.1 --mqtt-port 3333 --mqtt-topic CustomByteBlock
"""
import argparse
import os
import select
import struct
import sys
import time
from collections import deque

import paho.mqtt.client as mqtt
import serial

# 协议附录一: CRC8 表 (反射, init=0xFF, poly=0x131)
CRC8_TAB = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]
# CRC16 表 (反射, init=0xFFFF, poly=0x1021 的反射形式 0x8408)
_CRC16_TAB: list[int] = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        if _c & 1:
            _c = (_c >> 1) ^ 0x8408
        else:
            _c >>= 1
    _CRC16_TAB.append(_c & 0xFFFF)


def crc8(data: bytes, init: int = 0xFF) -> int:
    c = init
    for b in data:
        c = CRC8_TAB[c ^ b]
    return c


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    c = init
    for b in data:
        c = (c >> 8) ^ _CRC16_TAB[(c ^ b) & 0xFF]
    return c & 0xFFFF


def encode_protobuf_bytes_field(data: bytes, field_number: int = 1) -> bytes:
    """Wire format: tag(varint) + length(varint) + data."""
    tag = (field_number << 3) | 2  # wire type 2 (length-delimited)
    out = bytearray()
    # Encode tag varint (small, single byte for field 1)
    out.append(tag)
    # Encode length varint
    n = len(data)
    while True:
        b = n & 0x7F
        n >>= 7
        if n == 0:
            out.append(b)
            break
        else:
            out.append(b | 0x80)
    out.extend(data)
    return bytes(out)


class FrameParser:
    """裁判系统串口帧增量解析器: SOF + len(2) + seq + crc8 + cmd_id(2) + data + crc16."""

    def __init__(self):
        self.buf = bytearray()
        self.stats = {
            "total_bytes": 0,
            "frames_seen": 0,
            "crc8_pass": 0,
            "crc8_fail": 0,
            "crc16_pass": 0,
            "crc16_fail": 0,
            "cmd_0310": 0,
            "other_cmd": 0,
            "resync": 0,
        }

    def feed(self, chunk: bytes):
        self.stats["total_bytes"] += len(chunk)
        self.buf.extend(chunk)
        # Try to parse as many frames as possible
        while True:
            f = self._try_parse_one()
            if f is None:
                break
            yield f

    def _try_parse_one(self):
        # Search for SOF
        while self.buf:
            if self.buf[0] == 0xA5:
                break
            self.buf.pop(0)
            self.stats["resync"] += 1
        if len(self.buf) < 5:
            return None

        # Validate CRC8 of header[0..4]
        header = bytes(self.buf[:4])
        c8 = crc8(header)
        if c8 != self.buf[4]:
            self.stats["crc8_fail"] += 1
            self.buf.pop(0)  # not a real SOF, slide
            self.stats["resync"] += 1
            return None
        self.stats["crc8_pass"] += 1

        data_len = self.buf[1] | (self.buf[2] << 8)
        total_len = 5 + 2 + data_len + 2
        if len(self.buf) < total_len:
            return None  # need more bytes

        cmd_id = self.buf[5] | (self.buf[6] << 8)
        data = bytes(self.buf[7 : 7 + data_len])
        crc16_recv = self.buf[7 + data_len] | (self.buf[8 + data_len] << 8)
        crc16_calc = crc16(bytes(self.buf[: 7 + data_len]))

        self.stats["frames_seen"] += 1
        if crc16_recv != crc16_calc:
            self.stats["crc16_fail"] += 1
            # Still slide forward to try resync
            self.buf.pop(0)
            self.stats["resync"] += 1
            return None
        self.stats["crc16_pass"] += 1
        if cmd_id == 0x0310:
            self.stats["cmd_0310"] += 1
        else:
            self.stats["other_cmd"] += 1

        # Successful frame, advance buffer
        del self.buf[:total_len]
        return {"cmd_id": cmd_id, "data": data, "seq": self.buf[3] if False else None}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--serial", default="/dev/ttyUSB1")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--mqtt-host", default="127.0.0.1")
    p.add_argument("--mqtt-port", type=int, default=3333)
    p.add_argument("--mqtt-topic", default="CustomByteBlock")
    p.add_argument("--mqtt-client-id", default="player-end-emulator")
    p.add_argument("--report-interval", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=0, help="seconds, 0=forever")
    args = p.parse_args()

    print(f"[player-end] opening serial {args.serial} @ {args.baud}", flush=True)
    ser = serial.Serial(args.serial, args.baud, timeout=0)

    print(f"[player-end] connecting MQTT {args.mqtt_host}:{args.mqtt_port}", flush=True)
    mq = mqtt.Client(client_id=args.mqtt_client_id)
    mq.connect(args.mqtt_host, args.mqtt_port, keepalive=10)
    mq.loop_start()

    parser = FrameParser()
    published = 0
    last_report = time.time()
    start = time.time()

    try:
        while True:
            if args.duration > 0 and (time.time() - start) > args.duration:
                break
            ready, _, _ = select.select([ser.fileno()], [], [], 0.1)
            if ready:
                try:
                    chunk = os.read(ser.fileno(), 4096)
                except BlockingIOError:
                    chunk = b""
                if chunk:
                    for frame in parser.feed(chunk):
                        if frame["cmd_id"] == 0x0310:
                            payload = encode_protobuf_bytes_field(frame["data"], 1)
                            mq.publish(args.mqtt_topic, payload, qos=1)
                            published += 1

            now = time.time()
            if now - last_report >= args.report_interval:
                s = parser.stats
                print(
                    f"[player-end] rx_bytes={s['total_bytes']} frames={s['frames_seen']} "
                    f"crc8_pass={s['crc8_pass']} crc8_fail={s['crc8_fail']} "
                    f"crc16_pass={s['crc16_pass']} crc16_fail={s['crc16_fail']} "
                    f"cmd_0310={s['cmd_0310']} other_cmd={s['other_cmd']} "
                    f"resync={s['resync']} mqtt_pub={published}",
                    flush=True,
                )
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        mq.loop_stop()
        mq.disconnect()
        ser.close()
        print("[player-end] done", flush=True)


if __name__ == "__main__":
    sys.exit(main() or 0)
