#!/usr/bin/env python3
"""
Send body command frames from Jetson/PC to MCU over serial.

Protocol:
  Header  : 0xAA 0x55
  msg_type: u8 (0x01 = CMD_BODY)
  seq     : u16
  len     : u16
  payload : <ffffB> = height_m, roll_rad, pitch_rad, yaw_rad, flags
  crc16   : u16 (CCITT-FALSE)

Usage example:
  python3 jetson_body_cmd_sender.py \
    --port /dev/ttyACM0 --baud 115200 --hz 50 \
    --height 0.18 --roll 0.0 --pitch 0.0 --yaw 0.0
"""

from __future__ import annotations

import argparse
import math
import struct
import time

import serial


FRAME_HEADER = b"\xAA\x55"
MSG_CMD_BODY = 0x01


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


def build_cmd_body(seq: int, height: float, roll: float, pitch: float, yaw: float, estop: bool) -> bytes:
    flags = 0x01 if estop else 0x00
    payload = struct.pack("<ffffB", height, roll, pitch, yaw, flags)
    head = struct.pack("<2sBHH", FRAME_HEADER, MSG_CMD_BODY, seq & 0xFFFF, len(payload))
    crc = crc16_ccitt_false(head + payload)
    return head + payload + struct.pack("<H", crc)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Jetson body command sender")
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    p.add_argument("--hz", type=float, default=50.0, help="Publish rate")
    p.add_argument("--height", type=float, default=0.18, help="Target height in meters")
    p.add_argument("--roll", type=float, default=0.0, help="Target roll in radians")
    p.add_argument("--pitch", type=float, default=0.0, help="Target pitch in radians")
    p.add_argument("--yaw", type=float, default=0.0, help="Target yaw in radians")
    p.add_argument("--turn-radius", type=float, default=0.6, help="Current turn radius in meters")
    p.add_argument("--payload-front-kg", type=float, default=0.0, help="Approx payload on front-reaching arm")
    p.add_argument("--height-max", type=float, default=0.30, help="Configured max lift height in meters")
    p.add_argument("--tilt-threshold-deg", type=float, default=8.0, help="Tilt threshold for speed derating")
    p.add_argument("--estop", action="store_true", help="Send estop flag")
    p.add_argument("--duration", type=float, default=0.0, help="Run time in seconds; 0 means infinite")
    return p.parse_args()


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def base_vmax_from_radius(radius_m: float) -> float:
    # Conservative caps from stability review:
    # R=0.40 -> 0.60 m/s, R=0.60 -> 0.80 m/s, R=1.00 -> 1.00 m/s
    r = max(radius_m, 0.2)
    if r <= 0.4:
        return 0.6
    if r <= 0.6:
        return lerp(0.6, 0.8, (r - 0.4) / 0.2)
    if r <= 1.0:
        return lerp(0.8, 1.0, (r - 0.6) / 0.4)
    # large-radius cap
    return 1.0


def recommend_base_vmax(
    radius_m: float,
    height_m: float,
    height_max_m: float,
    roll_rad: float,
    pitch_rad: float,
    payload_front_kg: float,
    tilt_threshold_deg: float,
) -> float:
    vmax = base_vmax_from_radius(radius_m)

    tilt_deg = max(abs(math.degrees(roll_rad)), abs(math.degrees(pitch_rad)))
    height_ratio = height_m / max(height_max_m, 1e-3)

    # Derating rules:
    # 1) raised platform (>70%) => 30% reduction
    # 2) tilt beyond threshold => additional 30% reduction
    # 3) front payload >=3kg => additional 20% reduction
    if height_ratio > 0.7:
        vmax *= 0.7
    if tilt_deg > tilt_threshold_deg:
        vmax *= 0.7
    if payload_front_kg >= 3.0:
        vmax *= 0.8

    # hard floor so robot still creeps for fine positioning
    return max(vmax, 0.15)


def main() -> None:
    args = parse_args()
    period = 1.0 / max(args.hz, 1e-3)
    seq = 0

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        t0 = time.time()
        while True:
            now = time.time()
            if args.duration > 0 and (now - t0) >= args.duration:
                break

            frame = build_cmd_body(
                seq=seq,
                height=args.height,
                roll=args.roll,
                pitch=args.pitch,
                yaw=args.yaw,
                estop=args.estop,
            )
            ser.write(frame)
            if seq % 50 == 0:
                vmax = recommend_base_vmax(
                    radius_m=args.turn_radius,
                    height_m=args.height,
                    height_max_m=args.height_max,
                    roll_rad=args.roll,
                    pitch_rad=args.pitch,
                    payload_front_kg=args.payload_front_kg,
                    tilt_threshold_deg=args.tilt_threshold_deg,
                )
                print(
                    f"seq={seq} h={args.height:.3f} r={args.roll:.3f} "
                    f"p={args.pitch:.3f} y={args.yaw:.3f} estop={int(args.estop)} "
                    f"vmax_reco={vmax:.2f}m/s (R={args.turn_radius:.2f}m)"
                )
            seq = (seq + 1) & 0xFFFF
            time.sleep(period)


if __name__ == "__main__":
    main()
