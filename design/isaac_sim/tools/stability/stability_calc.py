#!/usr/bin/env python3
"""
Simple stability calculator for mobile manipulator stack:
base + lift + stewart + dual arms + payload.

Usage:
  python3 stability_calc.py --config sample_config.json
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path


G = 9.81


@dataclass
class MassItem:
    name: str
    mass_kg: float
    x_m: float
    y_m: float
    z_m: float


def weighted_com(items: list[MassItem]) -> tuple[float, float, float, float]:
    total = sum(i.mass_kg for i in items)
    if total <= 0:
        raise ValueError("Total mass must be > 0")
    x = sum(i.mass_kg * i.x_m for i in items) / total
    y = sum(i.mass_kg * i.y_m for i in items) / total
    z = sum(i.mass_kg * i.z_m for i in items) / total
    return total, x, y, z


def calc_tip_angle_deg(half_support_m: float, com_height_m: float) -> float:
    return math.degrees(math.atan2(half_support_m, com_height_m))


def calc_ay_limit_g(half_support_m: float, com_height_m: float) -> float:
    return half_support_m / com_height_m


def calc_vmax_for_turn(ay_limit_g: float, turn_radius_m: float) -> float:
    ay = ay_limit_g * G
    return math.sqrt(max(ay * turn_radius_m, 0.0))


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, required=True)
    args = p.parse_args()

    cfg = load_config(args.config)
    half_width = float(cfg["support_polygon"]["half_width_m"])
    half_length = float(cfg["support_polygon"]["half_length_m"])
    worst_axis = min(half_width, half_length)

    items = [MassItem(**x) for x in cfg["mass_items"]]
    total, cx, cy, cz = weighted_com(items)

    tip_deg = calc_tip_angle_deg(worst_axis, cz)
    ay_lim_g = calc_ay_limit_g(worst_axis, cz)

    static_margin_deg = tip_deg - float(cfg["requirements"]["max_operating_tilt_deg"])
    tilt_ok = static_margin_deg >= float(cfg["requirements"]["tilt_margin_deg_min"])

    dyn_req_g = float(cfg["requirements"]["max_expected_lateral_acc_g"])
    dyn_ok = ay_lim_g >= dyn_req_g * float(cfg["requirements"]["dynamic_margin_factor"])

    print("=== Stability Report ===")
    print(f"Total mass: {total:.2f} kg")
    print(f"COM (x,y,z): ({cx:.3f}, {cy:.3f}, {cz:.3f}) m")
    print(f"Support half-width: {half_width:.3f} m")
    print(f"Support half-length: {half_length:.3f} m")
    print(f"Worst support axis: {worst_axis:.3f} m")
    print(f"Static tip angle: {tip_deg:.2f} deg")
    print(f"Static margin to operating tilt: {static_margin_deg:.2f} deg")
    print(f"Dynamic lateral accel limit: {ay_lim_g:.3f} g")
    print(f"Static safety check: {'PASS' if tilt_ok else 'FAIL'}")
    print(f"Dynamic safety check: {'PASS' if dyn_ok else 'FAIL'}")

    print("\n--- Suggested speed caps by turn radius ---")
    for r in cfg.get("turn_radius_candidates_m", [0.4, 0.6, 1.0]):
        vmax = calc_vmax_for_turn(ay_lim_g * 0.7, float(r))  # 0.7 as control safety factor
        print(f"R={float(r):.2f} m -> vmax <= {vmax:.2f} m/s")

    if not (tilt_ok and dyn_ok):
        print("\n[Action]")
        print("- Reduce lift max height or Stewart tilt limits")
        print("- Increase support polygon (wheel track / support ring)")
        print("- Reduce allowed base speed when raised")


if __name__ == "__main__":
    main()
