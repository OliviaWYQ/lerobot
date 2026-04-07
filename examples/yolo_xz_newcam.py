#!/usr/bin/env python3
"""XZ bottle follow for the new webcam.

X uses shoulder_pan to keep the bottle near the image center.
Z uses the validated low-mid to mid vertical segment only. When the bottle gets
close to the top or bottom boundary, the script adjusts shoulder_lift inside the
validated lift range and derives elbow_flex + wrist_flex from recorded lookup
points plus leveling anchors.
"""

import argparse
import traceback

import cv2
from ultralytics import YOLO

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
    {"label": "low", "shoulder_lift": -80.97, "manual_elbow_trim": 0.00, "manual_wrist_trim": -34.00},
    {"label": "low_mid", "shoulder_lift": -60.40, "manual_elbow_trim": -24.00, "manual_wrist_trim": -8.00},
    {"label": "mid", "shoulder_lift": -56.97, "manual_elbow_trim": -24.00, "manual_wrist_trim": -4.00},
]

VALIDATED_LIFT_MIN = -80.97
VALIDATED_LIFT_MAX = -56.97


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
            return tuple(p0[k] + t * (p1[k] - p0[k]) for k in y_keys)
    return tuple(pts[-1][k] for k in y_keys)


def interpolate_from_lift(lift_value):
    return interpolate_points(Z_TRAJECTORY, 'shoulder_lift', ('elbow_flex', 'wrist_flex'), lift_value)


def interpolate_anchor_trim(lift_value):
    return interpolate_points(LEVEL_ANCHORS, 'shoulder_lift', ('manual_elbow_trim', 'manual_wrist_trim'), lift_value)


def step_from_error(abs_err, deadband, base_step):
    if abs_err <= deadband:
        return 0.0
    if abs_err >= 180:
        return base_step * 3.0
    if abs_err >= 100:
        return base_step * 2.0
    return base_step


def update_streak(direction, last_direction, same_direction_frames):
    if direction != 0 and direction == last_direction:
        same_direction_frames += 1
    elif direction != 0:
        same_direction_frames = 1
    else:
        same_direction_frames = 0
    return direction, same_direction_frames


def main():
    parser = argparse.ArgumentParser(description='XZ bottle follow for new webcam')
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--id', default='None')
    parser.add_argument('--camera', default='/dev/video1')
    parser.add_argument('--model', default='/home/nvidia/Desktop/test/yolo26n.pt')
    parser.add_argument('--target', default='bottle')
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--x-step', type=float, default=2.0)
    parser.add_argument('--x-deadband', type=int, default=25)
    parser.add_argument('--x-confirm-frames', type=int, default=2)
    parser.add_argument('--x-cooldown-frames', type=int, default=2)
    parser.add_argument('--pan-limit', type=float, default=60.0)
    parser.add_argument('--stationary-pixel-threshold', type=float, default=6.0)
    parser.add_argument('--stationary-frames', type=int, default=20)
    parser.add_argument('--z-step-shoulder', type=float, default=2.5)
    parser.add_argument('--z-confirm-frames', type=int, default=3)
    parser.add_argument('--z-cooldown-frames', type=int, default=3)
    parser.add_argument('--lift-min', type=float, default=VALIDATED_LIFT_MIN)
    parser.add_argument('--lift-max', type=float, default=VALIDATED_LIFT_MAX)
    parser.add_argument('--elbow-window', type=float, default=80.0)
    parser.add_argument('--wrist-window', type=float, default=140.0)
    parser.add_argument('--z-shoulder-sign', type=float, default=-1.0)
    parser.add_argument('--top-band', type=float, default=0.18)
    parser.add_argument('--bottom-band', type=float, default=0.30)
    parser.add_argument('--edge-margin-px', type=int, default=20)
    parser.add_argument('--log-every', type=int, default=10)
    args = parser.parse_args()

    print('=' * 60)
    print('YOLO XZ follow for new webcam')
    print(f'robot: {args.port}')
    print(f'camera: {args.camera}')
    print(f'model: {args.model}')
    print(f'target: {args.target}')
    print('X: shoulder_pan center follow')
    print('Z: validated low-mid to mid segment with shoulder_lift + elbow_flex + wrist_flex')
    print(f'validated lift range: {args.lift_min:.2f} .. {args.lift_max:.2f}')
    print('Press q in the webcam window to quit.')
    print('=' * 60)

    model = YOLO(args.model)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f'Unable to open camera {args.camera}')

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    robot.connect(calibrate=False)

    frame_id = 0
    last_log = 'no detection'
    target_pan = None
    target_lift = None
    target_elbow = None
    target_wrist = None

    last_x_direction = 0
    x_same_direction_frames = 0
    x_cooldown_frames = 0

    last_z_direction = 0
    z_same_direction_frames = 0
    z_cooldown_frames = 0

    prev_obj_cx = None
    prev_obj_cy = None
    stationary_frames = 0

    startup_elbow = None
    startup_wrist = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('Camera frame read failed')
                break

            frame_id += 1
            h, w = frame.shape[:2]
            cx0 = w // 2
            cy0 = h // 2
            top_threshold = h * args.top_band + args.edge_margin_px
            bottom_threshold = h * (1.0 - args.bottom_band) - args.edge_margin_px

            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            annotated = results[0].plot()
            cv2.drawMarker(annotated, (cx0, cy0), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.line(annotated, (0, int(top_threshold)), (w - 1, int(top_threshold)), (0, 200, 255), 1)
            cv2.line(annotated, (0, int(bottom_threshold)), (w - 1, int(bottom_threshold)), (0, 200, 255), 1)

            obs = robot.get_observation()
            current_pan = float(obs['shoulder_pan.pos'])
            current_lift = float(obs['shoulder_lift.pos'])
            current_elbow = float(obs['elbow_flex.pos'])
            current_wrist = float(obs['wrist_flex.pos'])

            if target_pan is None:
                target_pan = current_pan
            if target_lift is None:
                target_lift = clamp(current_lift, args.lift_min, args.lift_max)
            if startup_elbow is None:
                startup_elbow = current_elbow
            if startup_wrist is None:
                startup_wrist = current_wrist
            if target_elbow is None or target_wrist is None:
                interp_elbow, interp_wrist = interpolate_from_lift(target_lift)
                anchor_elbow, anchor_wrist = interpolate_anchor_trim(target_lift)
                target_elbow = clamp(interp_elbow + anchor_elbow, startup_elbow - args.elbow_window, startup_elbow + args.elbow_window)
                target_wrist = clamp(interp_wrist + anchor_wrist, startup_wrist - args.wrist_window, startup_wrist + args.wrist_window)

            commanded_pan = target_pan
            commanded_lift = target_lift
            commanded_elbow = target_elbow
            commanded_wrist = target_wrist
            dx = 0.0
            dz = 0.0
            conf = 0.0
            x_step_cmd = 0.0
            z_step_lift = 0.0
            z_step_elbow = 0.0
            z_step_wrist = 0.0
            z_state = 'hold'
            anchor_elbow = 0.0
            anchor_wrist = 0.0

            best_box = None
            best_conf = -1.0
            for box in results[0].boxes:
                cls = int(box.cls[0])
                label = results[0].names[cls]
                score = float(box.conf[0])
                if label == args.target and score > best_conf:
                    best_box = box
                    best_conf = score

            if best_box is not None:
                x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
                obj_cx = (x1 + x2) / 2.0
                obj_cy = (y1 + y2) / 2.0
                dx = obj_cx - cx0
                dz = obj_cy - cy0
                conf = best_conf

                if (
                    prev_obj_cx is not None
                    and prev_obj_cy is not None
                    and abs(obj_cx - prev_obj_cx) <= args.stationary_pixel_threshold
                    and abs(obj_cy - prev_obj_cy) <= args.stationary_pixel_threshold
                ):
                    stationary_frames += 1
                else:
                    stationary_frames = 0
                prev_obj_cx = obj_cx
                prev_obj_cy = obj_cy
                is_stationary = stationary_frames >= args.stationary_frames

                x_direction = 0
                if dx > args.x_deadband:
                    x_direction = 1
                elif dx < -args.x_deadband:
                    x_direction = -1
                last_x_direction, x_same_direction_frames = update_streak(x_direction, last_x_direction, x_same_direction_frames)
                x_step_mag = step_from_error(abs(dx), args.x_deadband, args.x_step)

                if x_cooldown_frames > 0:
                    x_cooldown_frames -= 1
                elif not is_stationary and x_direction != 0 and x_same_direction_frames >= args.x_confirm_frames and x_step_mag > 0.0:
                    x_step_cmd = x_direction * x_step_mag
                    target_pan = clamp(target_pan + x_step_cmd, -args.pan_limit, args.pan_limit)
                    x_cooldown_frames = args.x_cooldown_frames
                commanded_pan = target_pan

                z_direction = 0
                z_distance = 0.0
                if obj_cy < top_threshold:
                    z_direction = -1
                    z_distance = top_threshold - obj_cy
                    z_state = 'top'
                elif obj_cy > bottom_threshold:
                    z_direction = 1
                    z_distance = obj_cy - bottom_threshold
                    z_state = 'bottom'

                last_z_direction, z_same_direction_frames = update_streak(z_direction, last_z_direction, z_same_direction_frames)
                z_step_mag = step_from_error(z_distance, 0.0, 1.0) if z_direction != 0 else 0.0

                if z_cooldown_frames > 0:
                    z_cooldown_frames -= 1
                elif not is_stationary and z_direction != 0 and z_same_direction_frames >= args.z_confirm_frames and z_step_mag > 0.0:
                    z_step_lift = z_direction * args.z_shoulder_sign * args.z_step_shoulder * z_step_mag
                    target_lift = clamp(target_lift + z_step_lift, args.lift_min, args.lift_max)
                    interp_elbow, interp_wrist = interpolate_from_lift(target_lift)
                    anchor_elbow, anchor_wrist = interpolate_anchor_trim(target_lift)
                    target_elbow = clamp(interp_elbow + anchor_elbow, startup_elbow - args.elbow_window, startup_elbow + args.elbow_window)
                    target_wrist = clamp(interp_wrist + anchor_wrist, startup_wrist - args.wrist_window, startup_wrist + args.wrist_window)
                    z_step_elbow = target_elbow - current_elbow
                    z_step_wrist = target_wrist - current_wrist
                    z_cooldown_frames = args.z_cooldown_frames
                else:
                    anchor_elbow, anchor_wrist = interpolate_anchor_trim(target_lift)

                commanded_lift = target_lift
                commanded_elbow = target_elbow
                commanded_wrist = target_wrist

                robot.send_action({
                    'shoulder_pan.pos': commanded_pan,
                    'shoulder_lift.pos': commanded_lift,
                    'elbow_flex.pos': commanded_elbow,
                    'wrist_flex.pos': commanded_wrist,
                })

                cv2.circle(annotated, (int(obj_cx), int(obj_cy)), 5, (0, 0, 255), -1)
                cv2.line(annotated, (cx0, cy0), (int(obj_cx), int(obj_cy)), (255, 255, 0), 2)
                last_log = (
                    f'bottle conf={conf:.2f} dx={dx:.0f} dz={dz:.0f} top={top_threshold:.0f} bottom={bottom_threshold:.0f} zstate={z_state} '
                    f'pan={current_pan:.2f}->{commanded_pan:.2f} '
                    f'lift={current_lift:.2f}->{commanded_lift:.2f} '
                    f'elbow={current_elbow:.2f}->{commanded_elbow:.2f} '
                    f'wrist={current_wrist:.2f}->{commanded_wrist:.2f} '
                    f'xstep={x_step_cmd:.1f} zlift={z_step_lift:.1f} zelbow={z_step_elbow:.1f} zwrist={z_step_wrist:.1f} '
                    f'anchorE={anchor_elbow:.1f} anchorW={anchor_wrist:.1f} static={is_stationary}'
                )
            else:
                robot.send_action({
                    'shoulder_pan.pos': commanded_pan,
                    'shoulder_lift.pos': commanded_lift,
                    'elbow_flex.pos': commanded_elbow,
                    'wrist_flex.pos': commanded_wrist,
                })
                last_x_direction = 0
                x_same_direction_frames = 0
                x_cooldown_frames = 0
                last_z_direction = 0
                z_same_direction_frames = 0
                z_cooldown_frames = 0
                prev_obj_cx = None
                prev_obj_cy = None
                stationary_frames = 0
                last_log = (
                    f'bottle not found pan={current_pan:.2f}->{commanded_pan:.2f} '
                    f'lift={current_lift:.2f}->{commanded_lift:.2f} '
                    f'elbow={current_elbow:.2f}->{commanded_elbow:.2f} '
                    f'wrist={current_wrist:.2f}->{commanded_wrist:.2f}'
                )

            if frame_id % args.log_every == 0:
                print(f'[frame {frame_id}] {last_log}', flush=True)

            cv2.putText(annotated, last_log, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            cv2.imshow('YOLO XZ Follow NewCam', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Runtime error: {exc}')
        traceback.print_exc()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        robot.disconnect()


if __name__ == '__main__':
    main()
