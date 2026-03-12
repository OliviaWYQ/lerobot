#!/usr/bin/env python3
"""
Receive and decode MCU state frames over serial.

Supported messages:
  0x81 STATE_BODY:
    payload <ffffH> = height_m, roll_rad, pitch_rad, yaw_rad, fault_code
  0x82 STATE_ACTUATOR:
    payload <f6f6f> = lift_pos, stewart_len[6], stewart_current[6]

Frame format:
  0xAA 0x55 | msg_type(u8) | seq(u16) | len(u16) | payload | crc16(u16)
"""

from __future__ import annotations

import argparse
import struct

import serial


FRAME_HEADER = b"\xAA\x55"
MSG_STATE_BODY = 0x81
MSG_STATE_ACTUATOR = 0x82


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MCU state receiver")
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    return p.parse_args()


def read_exact(ser: serial.Serial, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            break
        data += chunk
    return data


def sync_header(ser: serial.Serial) -> bool:
    b1 = ser.read(1)
    if not b1:
        return False
    if b1 != FRAME_HEADER[:1]:
        return False
    b2 = ser.read(1)
    return b2 == FRAME_HEADER[1:]


def decode_frame(msg_type: int, seq: int, payload: bytes) -> None:
    if msg_type == MSG_STATE_BODY:
        if len(payload) != struct.calcsize("<ffffH"):
            print(f"STATE_BODY payload length mismatch: {len(payload)}")
            return
        h, r, p, y, fault = struct.unpack("<ffffH", payload)
        print(f"[BODY] seq={seq} h={h:.3f} r={r:.3f} p={p:.3f} y={y:.3f} fault={fault}")
    elif msg_type == MSG_STATE_ACTUATOR:
        fmt = "<f6f6f"
        if len(payload) != struct.calcsize(fmt):
            print(f"STATE_ACT payload length mismatch: {len(payload)}")
            return
        vals = struct.unpack(fmt, payload)
        lift_pos = vals[0]
        lens = vals[1:7]
        currents = vals[7:13]
        print(
            f"[ACT] seq={seq} lift={lift_pos:.4f} "
            f"len={[round(v,4) for v in lens]} "
            f"cur={[round(v,3) for v in currents]}"
        )
    else:
        print(f"[RAW] msg=0x{msg_type:02X} seq={seq} payload_len={len(payload)}")


def main() -> None:
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        print(f"Listening on {args.port} @ {args.baud}")
        while True:
            if not sync_header(ser):
                continue
            head = read_exact(ser, 5)  # msg_type(u8), seq(u16), len(u16)
            if len(head) < 5:
                continue
            msg_type, seq, payload_len = struct.unpack("<BHH", head)
            payload = read_exact(ser, payload_len)
            crc_raw = read_exact(ser, 2)
            if len(payload) < payload_len or len(crc_raw) < 2:
                continue
            crc_recv = struct.unpack("<H", crc_raw)[0]
            crc_calc = crc16_ccitt_false(FRAME_HEADER + head + payload)
            if crc_recv != crc_calc:
                print(
                    f"CRC mismatch msg=0x{msg_type:02X} seq={seq} "
                    f"recv=0x{crc_recv:04X} calc=0x{crc_calc:04X}"
                )
                continue
            decode_frame(msg_type, seq, payload)


if __name__ == "__main__":
    main()
