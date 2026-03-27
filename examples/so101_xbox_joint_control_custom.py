#!/usr/bin/env python3
"""Custom SO101 Xbox joint controller.

This script is separate from the official examples and controls a single
calibrated SO101 follower arm with an Xbox controller via pygame.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import pygame

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower

JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

AXIS_DEADBAND = 0.2
TRIGGER_THRESHOLD = 0.2
BUTTON_DEBOUNCE_S = 0.18
INPUT_PRINT_INTERVAL_S = 0.5


@dataclass(frozen=True)
class ControllerMapping:
    left_x: int = 0
    left_y: int = 1
    right_x: int = 3
    right_y: int = 4
    lt: int = 2
    rt: int = 5


class XboxController:
    def __init__(self, index: int = 0, mapping: ControllerMapping | None = None) -> None:
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError("No XBOX controller detected")
        if index < 0 or index >= count:
            raise RuntimeError(f"Controller index {index} out of range, found {count} controller(s)")

        self.joystick = pygame.joystick.Joystick(index)
        self.joystick.init()
        self.default_mapping = self._default_mapping()
        self.mapping = mapping or self.default_mapping

        print(f"Using controller: {self.joystick.get_name()} (index={index})")
        print(
            "Controller capabilities:",
            f"axes={self.joystick.get_numaxes()}",
            f"buttons={self.joystick.get_numbuttons()}",
            f"hats={self.joystick.get_numhats()}",
        )
        print(
            "Axis mapping:",
            f"left_x={self.mapping.left_x}",
            f"left_y={self.mapping.left_y}",
            f"right_x={self.mapping.right_x}",
            f"right_y={self.mapping.right_y}",
            f"lt={self.mapping.lt}",
            f"rt={self.mapping.rt}",
        )

    def _default_mapping(self) -> ControllerMapping:
        num_axes = self.joystick.get_numaxes()
        if num_axes >= 6:
            return ControllerMapping()
        if num_axes == 5:
            return ControllerMapping(right_x=2, right_y=3, lt=4, rt=4)
        if num_axes >= 4:
            return ControllerMapping(right_x=2, right_y=3, lt=2, rt=3)
        return ControllerMapping()

    def _axis(self, index: int) -> float:
        if 0 <= index < self.joystick.get_numaxes():
            return float(self.joystick.get_axis(index))
        return 0.0

    def _button(self, index: int) -> bool:
        if 0 <= index < self.joystick.get_numbuttons():
            return bool(self.joystick.get_button(index))
        return False

    def poll(self) -> dict[str, float | bool]:
        pygame.event.pump()
        lt = self._axis(self.mapping.lt)
        rt = self._axis(self.mapping.rt)
        if self.mapping.lt == self.mapping.rt:
            lt = max(0.0, -lt)
            rt = max(0.0, rt)
        return {
            "left_x": self._axis(self.mapping.left_x),
            "left_y": self._axis(self.mapping.left_y),
            "right_x": self._axis(self.mapping.right_x),
            "right_y": self._axis(self.mapping.right_y),
            "lt": normalize_trigger(lt),
            "rt": normalize_trigger(rt),
            "a": self._button(0),
            "back": self._button(6),
            "start": self._button(7),
            "lb": self._button(4),
            "rb": self._button(5),
        }

    def raw_state(self) -> tuple[list[float], list[int], list[tuple[int, int]]]:
        pygame.event.pump()
        axes = [round(self._axis(i), 3) for i in range(self.joystick.get_numaxes())]
        buttons = [int(self._button(i)) for i in range(self.joystick.get_numbuttons())]
        hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
        return axes, buttons, hats

    def close(self) -> None:
        self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()


def normalize_trigger(value: float) -> float:
    return max(0.0, min(1.0, (value + 1.0) / 2.0))


def apply_deadband(value: float, deadband: float = AXIS_DEADBAND) -> float:
    return 0.0 if abs(value) < deadband else value


def clamp_target(joint: str, value: float) -> float:
    if joint == "gripper":
        return max(0.0, min(100.0, value))
    if joint == "wrist_roll":
        return max(-180.0, min(180.0, value))
    return max(-120.0, min(120.0, value))


def get_joint_positions(robot: SO101Follower) -> dict[str, float]:
    obs = robot.get_observation()
    return {
        key.removesuffix(".pos"): float(value)
        for key, value in obs.items()
        if key.endswith(".pos")
    }


def print_help() -> None:
    print("Custom SO101 Xbox control")
    print("left stick: shoulder_pan / shoulder_lift")
    print("right stick: wrist_flex / elbow_flex")
    print("LB/RB: wrist_roll -/+")
    print("LT/RT: gripper close/open")
    print("A: print targets")
    print("Back: reset to startup pose")
    print("Start: quit")


def main() -> int:
    parser = argparse.ArgumentParser(description="Custom SO101 Xbox joint control")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Robot serial port")
    parser.add_argument("--id", default="my_awesome_follower_arm", help="Robot calibration id")
    parser.add_argument("--step", type=float, default=3.0, help="Joint step size")
    parser.add_argument("--gripper-step", type=float, default=6.0, help="Gripper step size")
    parser.add_argument("--hz", type=float, default=30.0, help="Control loop frequency")
    parser.add_argument("--controller-index", type=int, default=0, help="Pygame joystick index")
    parser.add_argument("--left-x-axis", type=int, default=None)
    parser.add_argument("--left-y-axis", type=int, default=None)
    parser.add_argument("--right-x-axis", type=int, default=None)
    parser.add_argument("--right-y-axis", type=int, default=None)
    parser.add_argument("--lt-axis", type=int, default=None)
    parser.add_argument("--rt-axis", type=int, default=None)
    parser.add_argument("--print-inputs", action="store_true", help="Print raw controller state")
    args = parser.parse_args()

    preview = XboxController(index=args.controller_index)
    mapping = ControllerMapping(
        left_x=preview.default_mapping.left_x if args.left_x_axis is None else args.left_x_axis,
        left_y=preview.default_mapping.left_y if args.left_y_axis is None else args.left_y_axis,
        right_x=preview.default_mapping.right_x if args.right_x_axis is None else args.right_x_axis,
        right_y=preview.default_mapping.right_y if args.right_y_axis is None else args.right_y_axis,
        lt=preview.default_mapping.lt if args.lt_axis is None else args.lt_axis,
        rt=preview.default_mapping.rt if args.rt_axis is None else args.rt_axis,
    )
    preview.close()

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    controller = XboxController(index=args.controller_index, mapping=mapping)
    robot.connect(calibrate=False)

    print_help()
    targets = get_joint_positions(robot)
    home_targets = targets.copy()
    print("Connected robot:", args.id, "on", args.port)
    for joint in JOINT_ORDER:
        print(f"  {joint}: {targets[joint]:.1f}")

    button_time = 0.0
    reset_time = 0.0
    input_time = 0.0
    loop_period = 1.0 / max(args.hz, 1.0)

    try:
        while True:
            started = time.perf_counter()
            state = controller.poll()

            if state["start"]:
                break

            if state["back"] and started - reset_time > BUTTON_DEBOUNCE_S:
                reset_time = started
                targets = home_targets.copy()
                print("Reset to startup pose")

            if args.print_inputs and started - input_time > INPUT_PRINT_INTERVAL_S:
                input_time = started
                axes, buttons, hats = controller.raw_state()
                print(f"axes={axes} buttons={buttons} hats={hats}")

            left_x = apply_deadband(float(state["left_x"]))
            left_y = apply_deadband(float(state["left_y"]))
            right_x = apply_deadband(float(state["right_x"]))
            right_y = apply_deadband(float(state["right_y"]))

            targets["shoulder_pan"] = clamp_target("shoulder_pan", targets["shoulder_pan"] + left_x * args.step)
            targets["shoulder_lift"] = clamp_target("shoulder_lift", targets["shoulder_lift"] + left_y * args.step)
            targets["elbow_flex"] = clamp_target("elbow_flex", targets["elbow_flex"] + right_y * args.step)
            targets["wrist_flex"] = clamp_target("wrist_flex", targets["wrist_flex"] + right_x * args.step)

            if state["lb"]:
                targets["wrist_roll"] = clamp_target("wrist_roll", targets["wrist_roll"] - args.step)
            if state["rb"]:
                targets["wrist_roll"] = clamp_target("wrist_roll", targets["wrist_roll"] + args.step)
            if float(state["lt"]) > TRIGGER_THRESHOLD:
                targets["gripper"] = clamp_target("gripper", targets["gripper"] - args.gripper_step)
            if float(state["rt"]) > TRIGGER_THRESHOLD:
                targets["gripper"] = clamp_target("gripper", targets["gripper"] + args.gripper_step)

            now = time.perf_counter()
            if state["a"] and now - button_time > BUTTON_DEBOUNCE_S:
                button_time = now
                print("Targets:")
                for joint in JOINT_ORDER:
                    print(f"  {joint}: {targets[joint]:.1f}")

            robot.send_action({f"{joint}.pos": value for joint, value in targets.items()})

            elapsed = time.perf_counter() - started
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
    finally:
        robot.disconnect()
        controller.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
