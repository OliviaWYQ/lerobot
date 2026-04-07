#!/usr/bin/env python3
"""Minimal webcam YOLO test in the lerobot py312 environment."""

import argparse

import cv2
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Minimal webcam YOLO test for lerobot env")
    parser.add_argument("--model", default="/home/nvidia/Desktop/test/yolo26n.pt")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=320)
    args = parser.parse_args()

    model = YOLO(args.model)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index={args.camera_index}")

    print("Press q to quit")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed")
                break

            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            annotated = results[0].plot()

            cv2.imshow("YOLO Webcam Lerobot", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
