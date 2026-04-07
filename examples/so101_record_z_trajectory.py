#!/usr/bin/env python3
"""Record multiple SO101 Z poses for building a motor2->motor3 lookup table.

The arm stays manually movable between samples. For each point, the script briefly
connects, reads the pose, then disconnects again to release torque.
"""

import argparse
import traceback

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower

JOINTS = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

LABELS = [
    'z0_lowest',
    'z1_low_mid',
    'z2_middle',
    'z3_high_mid',
    'z4_highest',
]


def read_pose_once(port, robot_id):
    robot = SO101Follower(SO101FollowerConfig(port=port, id=robot_id))
    robot.connect(calibrate=False)
    try:
        obs = robot.get_observation()
        return {joint: float(obs[f'{joint}.pos']) for joint in JOINTS}
    finally:
        robot.disconnect()


def print_pose(label, pose):
    print(label)
    for joint in JOINTS:
        print(f'  {joint:14s} {pose[joint]:8.2f}')


def main():
    parser = argparse.ArgumentParser(description='Record SO101 Z trajectory points')
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--id', default='None')
    args = parser.parse_args()

    samples = []
    try:
        for label in LABELS:
            input(f'Place the arm at {label}, then press ENTER to sample...')
            pose = read_pose_once(args.port, args.id)
            samples.append((label, pose))
            print_pose(f'SAMPLED {label}', pose)
            print('Pose sampled and torque released again.')
            print()

        print('SUMMARY')
        for label, pose in samples:
            print(
                f"{label:12s} lift={pose['shoulder_lift']:.2f} "
                f"elbow={pose['elbow_flex']:.2f} wrist={pose['wrist_flex']:.2f}"
            )

        print('\nLOOKUP TABLE CANDIDATE')
        print('[')
        for label, pose in samples:
            print(
                '    {"label": "%s", "shoulder_lift": %.2f, "elbow_flex": %.2f, "wrist_flex": %.2f},'
                % (label, pose['shoulder_lift'], pose['elbow_flex'], pose['wrist_flex'])
            )
        print(']')

    except KeyboardInterrupt:
        print('stopped by user')
    except Exception as exc:
        print(f'Runtime error: {exc}')
        traceback.print_exc()


if __name__ == '__main__':
    main()
