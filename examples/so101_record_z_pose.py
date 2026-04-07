#!/usr/bin/env python3
"""Record SO101 joint poses for calibrating a vertical Z motion.

This version keeps the arm unlocked while you position it manually.
It only connects briefly to read the pose, then immediately releases torque again.
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
    parser = argparse.ArgumentParser(description='Record SO101 Z poses')
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--id', default='None')
    args = parser.parse_args()

    try:
        input('Place the arm at the Z START pose while unlocked, then press ENTER to sample...')
        start_pose = read_pose_once(args.port, args.id)
        print_pose('START pose', start_pose)
        print('Pose sampled and torque released again.')

        input('Move the arm to the Z END pose while unlocked, then press ENTER to sample...')
        end_pose = read_pose_once(args.port, args.id)
        print_pose('END pose', end_pose)
        print('Pose sampled and torque released again.')

        print('DELTA (end - start)')
        deltas = {}
        for joint in JOINTS:
            delta = end_pose[joint] - start_pose[joint]
            deltas[joint] = delta
            print(f'  {joint:14s} {delta:8.2f}')

        lift_delta = deltas['shoulder_lift']
        elbow_delta = deltas['elbow_flex']
        print('\nMotor 2/3 summary')
        print(f'  shoulder_lift delta: {lift_delta:.2f}')
        print(f'  elbow_flex delta:    {elbow_delta:.2f}')
        if abs(lift_delta) > 1e-6:
            print(f'  elbow/lift ratio:     {elbow_delta / lift_delta:.4f}')
        else:
            print('  elbow/lift ratio:     undefined (lift delta too small)')

    except KeyboardInterrupt:
        print('stopped by user')
    except Exception as exc:
        print(f'Runtime error: {exc}')
        traceback.print_exc()


if __name__ == '__main__':
    main()
