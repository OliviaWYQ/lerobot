#!/usr/bin/env python3
"""Interactive keyboard Z test for SO101 motor 2, 3 and 4.

Current tuning is intentionally limited to the validated low->mid Z range.
Keyboard changes shoulder_lift target; elbow_flex and wrist_flex are derived
from the recorded trajectory plus anchor-based leveling trims.

Keys:
  w : move Z up
  s : move Z down
  j/l : wrist trim -/+
  i/k : elbow trim -/+
  p : print current lift + manual trim anchor
  space : hold current pose
  r : return to startup pose
  x/q : quit
"""

import argparse
import select
import sys
import termios
import time
import traceback
import tty

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower


Z_TRAJECTORY = [
    {"label": "z0_lowest", "shoulder_lift": -100.66, "elbow_flex": 97.54, "wrist_flex": 9.54},
    {"label": "z1_low_mid", "shoulder_lift": -74.46, "elbow_flex": 75.30, "wrist_flex": 5.05},
    {"label": "z2_middle", "shoulder_lift": -58.99, "elbow_flex": 44.70, "wrist_flex": 21.85},
    {"label": "z3_high_mid", "shoulder_lift": -45.63, "elbow_flex": 10.68, "wrist_flex": 41.36},
    {"label": "z4_highest", "shoulder_lift": -19.16, "elbow_flex": -43.91, "wrist_flex": 70.73},
]

LEVEL_ANCHORS = [
    {"label": "low", "shoulder_lift": -76.04, "manual_elbow_trim": 0.00, "manual_wrist_trim": -8.00},
    {"label": "mid", "shoulder_lift": -57.23, "manual_elbow_trim": 0.00, "manual_wrist_trim": -8.00},
]

VALIDATED_LIFT_MIN = -76.04
VALIDATED_LIFT_MAX = -57.23


def clamp(value, low, high):
    return max(low, min(high, value))


def interpolate_points(points, x_key, y_keys, x_value):
    pts = sorted(points, key=lambda p: p[x_key])
    if x_value <= pts[0][x_key]:
        return tuple(pts[0][k] for k in y_keys)
    if x_value >= pts[-1][x_key]:
        return tuple(pts[-1][k] for k in y_keys)
    for p0, p1 in zip(pts, pts[1:]):
        x0 = p0[x_key]
        x1 = p1[x_key]
        if x0 <= x_value <= x1:
            t = (x_value - x0) / (x1 - x0)
            out = []
            for k in y_keys:
                out.append(p0[k] + t * (p1[k] - p0[k]))
            return tuple(out)
    return tuple(pts[-1][k] for k in y_keys)


def interpolate_from_lift(lift_value):
    return interpolate_points(Z_TRAJECTORY, 'shoulder_lift', ('elbow_flex', 'wrist_flex'), lift_value)


def interpolate_anchor_trim(lift_value):
    return interpolate_points(LEVEL_ANCHORS, 'shoulder_lift', ('manual_elbow_trim', 'manual_wrist_trim'), lift_value)


def read_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


def main():
    parser = argparse.ArgumentParser(description='SO101 motor 2/3/4 keyboard Z test for official arm')
    parser.add_argument('--port', default='/dev/ttyACM2')
    parser.add_argument('--id', default='None')
    parser.add_argument('--kp', type=float, default=0.6)
    parser.add_argument('--control-freq', type=float, default=50.0)
    parser.add_argument('--z-step-shoulder', type=float, default=2.5)
    parser.add_argument('--lift-min', type=float, default=VALIDATED_LIFT_MIN)
    parser.add_argument('--lift-max', type=float, default=VALIDATED_LIFT_MAX)
    parser.add_argument('--elbow-window', type=float, default=80.0)
    parser.add_argument('--wrist-window', type=float, default=140.0)
    parser.add_argument('--up-shoulder-sign', type=float, default=1.0)
    parser.add_argument('--trim-step-wrist', type=float, default=2.0)
    parser.add_argument('--trim-step-elbow', type=float, default=2.0)
    parser.add_argument('--log-every', type=int, default=20)
    args = parser.parse_args()

    print('=' * 60)
    print('SO101 motor 2/3/4 keyboard Z test for official arm')
    print(f'robot: {args.port}')
    print(f'kp: {args.kp}  control_freq: {args.control_freq}')
    print(f'z-step-shoulder: {args.z_step_shoulder}')
    print(f'validated lift range: {args.lift_min:.2f} .. {args.lift_max:.2f}')
    print(f'wrist window: +/-{args.wrist_window}')
    print('Trajectory: motor3 and motor4 are interpolated from recorded Z samples')
    print('Level anchors: low / low-mid / mid')
    print('w: up  s: down  j/l: wrist trim -/+  i/k: elbow trim -/+  p: print anchor')
    print('space: hold  r: return  x/q: quit')
    print('=' * 60)

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    robot.connect(calibrate=False)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        obs = robot.get_observation()
        base_lift = float(obs['shoulder_lift.pos'])
        base_elbow = float(obs['elbow_flex.pos'])
        base_wrist = float(obs['wrist_flex.pos'])
        target_lift = clamp(base_lift, args.lift_min, args.lift_max)
        base_anchor_elbow_trim, base_anchor_wrist_trim = interpolate_anchor_trim(target_lift)
        manual_elbow_trim = 0.0
        manual_wrist_trim = 0.0

        min_elbow = base_elbow - args.elbow_window
        max_elbow = base_elbow + args.elbow_window
        min_wrist = base_wrist - args.wrist_window
        max_wrist = base_wrist + args.wrist_window

        print(f'base lift={base_lift:.2f} elbow={base_elbow:.2f} wrist={base_wrist:.2f}', flush=True)

        def apply_targets():
            interp_elbow, interp_wrist = interpolate_from_lift(target_lift)
            auto_elbow_trim, auto_wrist_trim = interpolate_anchor_trim(target_lift)
            target_elbow = clamp(interp_elbow + auto_elbow_trim + manual_elbow_trim, min_elbow, max_elbow)
            target_wrist = clamp(interp_wrist + auto_wrist_trim + manual_wrist_trim, min_wrist, max_wrist)
            return target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim

        target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
        control_period = 1.0 / args.control_freq
        step_counter = 0

        while True:
            key = read_key_nonblocking()
            if key is not None:
                if key in ('q', 'x'):
                    print('exit requested', flush=True)
                    break
                if key == 'w':
                    target_lift = clamp(target_lift + args.up_shoulder_sign * args.z_step_shoulder, args.lift_min, args.lift_max)
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(
                        f'UP   target lift={target_lift:.2f} elbow={target_elbow:.2f} wrist={target_wrist:.2f} '
                        f'anchor_elbow={auto_elbow_trim:.2f} anchor_wrist={auto_wrist_trim:.2f} '
                        f'manual_elbow={manual_elbow_trim:.2f} manual_wrist={manual_wrist_trim:.2f}',
                        flush=True,
                    )
                elif key == 's':
                    target_lift = clamp(target_lift - args.up_shoulder_sign * args.z_step_shoulder, args.lift_min, args.lift_max)
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(
                        f'DOWN target lift={target_lift:.2f} elbow={target_elbow:.2f} wrist={target_wrist:.2f} '
                        f'anchor_elbow={auto_elbow_trim:.2f} anchor_wrist={auto_wrist_trim:.2f} '
                        f'manual_elbow={manual_elbow_trim:.2f} manual_wrist={manual_wrist_trim:.2f}',
                        flush=True,
                    )
                elif key == 'j':
                    manual_wrist_trim = clamp(manual_wrist_trim - args.trim_step_wrist, -args.wrist_window, args.wrist_window)
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(f'WRIST trim -> {manual_wrist_trim:.2f}  target wrist={target_wrist:.2f}', flush=True)
                elif key == 'l':
                    manual_wrist_trim = clamp(manual_wrist_trim + args.trim_step_wrist, -args.wrist_window, args.wrist_window)
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(f'WRIST trim -> {manual_wrist_trim:.2f}  target wrist={target_wrist:.2f}', flush=True)
                elif key == 'i':
                    manual_elbow_trim = clamp(manual_elbow_trim - args.trim_step_elbow, -args.elbow_window, args.elbow_window)
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(f'ELBOW trim -> {manual_elbow_trim:.2f}  target elbow={target_elbow:.2f}', flush=True)
                elif key == 'k':
                    manual_elbow_trim = clamp(manual_elbow_trim + args.trim_step_elbow, -args.elbow_window, args.elbow_window)
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(f'ELBOW trim -> {manual_elbow_trim:.2f}  target elbow={target_elbow:.2f}', flush=True)
                elif key == 'p':
                    obs = robot.get_observation()
                    cur_lift = float(obs['shoulder_lift.pos'])
                    cur_elbow = float(obs['elbow_flex.pos'])
                    cur_wrist = float(obs['wrist_flex.pos'])
                    print('LEVEL_ANCHOR ' +
                          f'{{"shoulder_lift": {cur_lift:.2f}, "elbow_flex": {cur_elbow:.2f}, "wrist_flex": {cur_wrist:.2f}, ' +
                          f'"manual_elbow_trim": {manual_elbow_trim:.2f}, "manual_wrist_trim": {manual_wrist_trim:.2f}}}',
                          flush=True)
                elif key == 'r':
                    target_lift = clamp(base_lift, args.lift_min, args.lift_max)
                    manual_elbow_trim = 0.0
                    manual_wrist_trim = 0.0
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(f'RETURN target lift={target_lift:.2f} elbow={target_elbow:.2f} wrist={target_wrist:.2f}', flush=True)
                elif key == ' ':
                    obs = robot.get_observation()
                    target_lift = clamp(float(obs['shoulder_lift.pos']), args.lift_min, args.lift_max)
                    manual_elbow_trim = 0.0
                    manual_wrist_trim = 0.0
                    target_elbow, target_wrist, auto_elbow_trim, auto_wrist_trim = apply_targets()
                    print(f'HOLD at lift={target_lift:.2f} elbow={target_elbow:.2f} wrist={target_wrist:.2f}', flush=True)

            obs = robot.get_observation()
            cur_lift = float(obs['shoulder_lift.pos'])
            cur_elbow = float(obs['elbow_flex.pos'])
            cur_wrist = float(obs['wrist_flex.pos'])
            lift_cmd = cur_lift + args.kp * (target_lift - cur_lift)
            elbow_cmd = cur_elbow + args.kp * (target_elbow - cur_elbow)
            wrist_cmd = cur_wrist + args.kp * (target_wrist - cur_wrist)

            robot.send_action({
                'shoulder_lift.pos': lift_cmd,
                'elbow_flex.pos': elbow_cmd,
                'wrist_flex.pos': wrist_cmd,
            })

            step_counter += 1
            if step_counter % args.log_every == 0:
                print(
                    f'cur lift={cur_lift:.2f} elbow={cur_elbow:.2f} wrist={cur_wrist:.2f} '
                    f'target lift={target_lift:.2f} elbow={target_elbow:.2f} wrist={target_wrist:.2f} '
                    f'trim anchor_elbow={auto_elbow_trim:.2f} anchor_wrist={auto_wrist_trim:.2f} '
                    f'manual_elbow={manual_elbow_trim:.2f} manual_wrist={manual_wrist_trim:.2f} '
                    f'cmd lift={lift_cmd:.2f} elbow={elbow_cmd:.2f} wrist={wrist_cmd:.2f}',
                    flush=True,
                )

            time.sleep(control_period)

    except KeyboardInterrupt:
        print('stopped by user', flush=True)
    except Exception as exc:
        print(f'Runtime error: {exc}', flush=True)
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        robot.disconnect()


if __name__ == '__main__':
    main()
