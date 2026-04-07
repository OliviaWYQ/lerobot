#!/usr/bin/env python3
"""Pan-only bottle follow for the new webcam using stable cv2 display."""

import argparse
import traceback

import cv2
from ultralytics import YOLO

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    parser = argparse.ArgumentParser(description="Pan-only bottle follow for new webcam")
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--id", default="None")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--model", default="/home/nvidia/Desktop/test/yolo26n.pt")
    parser.add_argument("--target", default="bottle")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--step", type=float, default=2.0, help="Base degrees per visual step")
    parser.add_argument("--deadband", type=int, default=25)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--confirm-frames", type=int, default=2)
    parser.add_argument("--cooldown-frames", type=int, default=2)
    parser.add_argument("--pan-limit", type=float, default=60.0)
    parser.add_argument("--stationary-pixel-threshold", type=float, default=6.0)
    parser.add_argument("--stationary-frames", type=int, default=20)
    args = parser.parse_args()

    print("=" * 60)
    print("YOLO pan follow for new webcam")
    print(f"robot: {args.port}")
    print(f"camera_index: {args.camera_index}")
    print(f"model: {args.model}")
    print(f"target: {args.target}")
    print("Only shoulder_pan is commanded.")
    print("Press q in the webcam window to quit.")
    print("=" * 60)

    model = YOLO(args.model)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index={args.camera_index}")

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    robot.connect(calibrate=False)

    frame_id = 0
    last_log = "no detection"
    target_pan = None
    last_direction = 0
    same_direction_frames = 0
    cooldown_frames = 0
    prev_obj_cx = None
    stationary_frames = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed")
                break

            frame_id += 1
            h, w = frame.shape[:2]
            cx0 = w // 2
            cy0 = h // 2

            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            annotated = results[0].plot()
            cv2.drawMarker(annotated, (cx0, cy0), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            obs = robot.get_observation()
            current_pan = float(obs["shoulder_pan.pos"])
            if target_pan is None:
                target_pan = current_pan
            commanded_pan = target_pan
            dx = 0.0
            conf = 0.0
            step_cmd = 0.0

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
                conf = best_conf

                if prev_obj_cx is not None and abs(obj_cx - prev_obj_cx) <= args.stationary_pixel_threshold:
                    stationary_frames += 1
                else:
                    stationary_frames = 0
                prev_obj_cx = obj_cx
                is_stationary = stationary_frames >= args.stationary_frames

                direction = 0
                if dx > args.deadband:
                    direction = 1
                elif dx < -args.deadband:
                    direction = -1

                if direction != 0 and direction == last_direction:
                    same_direction_frames += 1
                elif direction != 0:
                    same_direction_frames = 1
                else:
                    same_direction_frames = 0
                last_direction = direction

                step_mag = 0.0
                abs_dx = abs(dx)
                if abs_dx > args.deadband:
                    if abs_dx >= 180:
                        step_mag = args.step * 3.0
                    elif abs_dx >= 100:
                        step_mag = args.step * 2.0
                    else:
                        step_mag = args.step

                if cooldown_frames > 0:
                    cooldown_frames -= 1
                elif not is_stationary and direction != 0 and same_direction_frames >= args.confirm_frames and step_mag > 0.0:
                    step_cmd = direction * step_mag
                    target_pan = clamp(target_pan + step_cmd, -args.pan_limit, args.pan_limit)
                    cooldown_frames = args.cooldown_frames
                commanded_pan = target_pan
                robot.send_action({"shoulder_pan.pos": commanded_pan})

                cv2.circle(annotated, (int(obj_cx), int(obj_cy)), 5, (0, 0, 255), -1)
                cv2.line(annotated, (cx0, cy0), (int(obj_cx), int(obj_cy)), (255, 255, 0), 2)
                last_log = (
                    f"bottle conf={conf:.2f} dx={dx:.0f} current_pan={current_pan:.2f} target_pan={target_pan:.2f} commanded_pan={commanded_pan:.2f} step={step_cmd:.1f} streak={same_direction_frames} cooldown={cooldown_frames} static={is_stationary}"
                )
            else:
                if target_pan is not None:
                    commanded_pan = target_pan
                    robot.send_action({"shoulder_pan.pos": commanded_pan})
                    same_direction_frames = 0
                    last_direction = 0
                    cooldown_frames = 0
                    prev_obj_cx = None
                    stationary_frames = 0
                    last_log = f"bottle not found current_pan={current_pan:.2f} target_pan={target_pan:.2f}"
                else:
                    last_log = f"bottle not found current_pan={current_pan:.2f}"

            if frame_id % args.log_every == 0:
                print(f"[frame {frame_id}] {last_log}", flush=True)

            cv2.putText(annotated, last_log, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("YOLO Pan Follow NewCam", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Runtime error: {exc}")
        traceback.print_exc()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        robot.disconnect()


if __name__ == "__main__":
    main()
