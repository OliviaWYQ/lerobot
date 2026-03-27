#!/usr/bin/env python3
"""Terminal keyboard joint control for a single SO101 follower arm.

Reads keys directly from the current terminal instead of using pynput.
This is intended for a calibrated single arm with no leader device.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower


@dataclass
class ControlState:
    target_positions: dict[str, float]
    running: bool = True


JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

KEYMAP = {
    "q": ("shoulder_pan", -1),
    "a": ("shoulder_pan", 1),
    "w": ("shoulder_lift", -1),
    "s": ("shoulder_lift", 1),
    "e": ("elbow_flex", -1),
    "d": ("elbow_flex", 1),
    "r": ("wrist_flex", -1),
    "f": ("wrist_flex", 1),
    "t": ("wrist_roll", -1),
    "g": ("wrist_roll", 1),
    "y": ("gripper", -1),
    "h": ("gripper", 1),
}


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float) -> str | None:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        return sys.stdin.read(1)


def get_joint_positions(robot: SO101Follower) -> dict[str, float]:
    obs = robot.get_observation()
    return {
        key.removesuffix(".pos"): float(value)
        for key, value in obs.items()
        if key.endswith(".pos")
    }


def clamp_target(joint: str, value: float) -> float:
    if joint == "gripper":
        return max(0.0, min(100.0, value))
    if joint == "wrist_roll":
        return max(-180.0, min(180.0, value))
    return max(-120.0, min(120.0, value))


def print_help() -> None:
    print("Terminal keyboard control for SO101")
    print("q/a shoulder_pan   w/s shoulder_lift   e/d elbow_flex")
    print("r/f wrist_flex     t/g wrist_roll      y/h gripper")
    print("z print current targets")
    print("x quit")


def main() -> int:
    parser = argparse.ArgumentParser(description="Terminal keyboard joint control for SO101")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Robot serial port")
    parser.add_argument("--id", default="my_awesome_follower_arm", help="Robot calibration id")
    parser.add_argument("--step", type=float, default=5.0, help="Joint step size in degrees")
    parser.add_argument("--gripper-step", type=float, default=5.0, help="Gripper step size in percent")
    parser.add_argument("--hz", type=float, default=30.0, help="Control loop frequency")
    args = parser.parse_args()

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    robot.connect(calibrate=False)

    try:
        current_positions = get_joint_positions(robot)
        state = ControlState(target_positions=current_positions.copy())
        print_help()
        print("Connected robot:", args.id, "on", args.port)
        print("Initial positions:")
        for joint in JOINT_ORDER:
            print(f"  {joint}: {state.target_positions[joint]:.1f}")

        loop_period = 1.0 / max(args.hz, 1.0)
        with RawTerminal() as terminal:
            while state.running:
                started = time.perf_counter()
                key = terminal.read_key(loop_period)
                if key:
                    key = key.lower()
                    if key == "x":
                        state.running = False
                        continue
                    if key == "z":
                        print("Targets:")
                        for joint in JOINT_ORDER:
                            print(f"  {joint}: {state.target_positions[joint]:.1f}")
                    elif key in KEYMAP:
                        joint, direction = KEYMAP[key]
                        step = args.gripper_step if joint == "gripper" else args.step
                        new_target = clamp_target(joint, state.target_positions[joint] + direction * step)
                        state.target_positions[joint] = new_target
                        print(f"{joint} -> {new_target:.1f}")

                robot.send_action({f"{joint}.pos": value for joint, value in state.target_positions.items()})
                elapsed = time.perf_counter() - started
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
    finally:
        robot.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
