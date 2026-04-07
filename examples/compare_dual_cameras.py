#!/usr/bin/env python3
"""Side-by-side comparison viewer for official and new SO101 cameras."""

import argparse

import cv2


def main():
    parser = argparse.ArgumentParser(description='Compare official and new USB cameras side by side')
    parser.add_argument('--official', default='/dev/video2', help='Path for official camera stream')
    parser.add_argument('--new', dest='new_cam', default='/dev/video0', help='Path for new camera stream')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    args = parser.parse_args()

    cams = [
        ('official', args.official),
        ('new', args.new_cam),
    ]
    handles = []
    for label, path in cams:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise RuntimeError(f'Unable to open {label} camera at {path}')
        handles.append((label, path, cap))

    print('Press q in the window to quit.')
    try:
        while True:
            frames = []
            for label, path, cap in handles:
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f'Failed to read frame from {label} camera at {path}')
                frame = cv2.resize(frame, (args.width, args.height))
                cv2.putText(frame, f'{label}: {path}', (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                frames.append(frame)

            combined = cv2.hconcat(frames)
            cv2.imshow('Official vs New Camera', combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        for _, _, cap in handles:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
