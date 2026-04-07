#!/usr/bin/env python3
import cv2

CAMERA = '/dev/video0'
TITLE = 'New Camera Preview'

cap = cv2.VideoCapture(CAMERA)
if not cap.isOpened():
    raise RuntimeError(f'Unable to open {CAMERA}')

print('Press q to quit.')
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f'Failed to read frame from {CAMERA}')
        cv2.putText(frame, CAMERA, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(TITLE, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
