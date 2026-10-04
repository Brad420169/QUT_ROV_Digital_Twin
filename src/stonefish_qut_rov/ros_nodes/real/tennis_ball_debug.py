#!/usr/bin/env python3
"""Display the RTSP feed and the tracker's cleaned HSV mask; no vehicle commands."""
import argparse
import sys
import time
from pathlib import Path

# Support direct execution from the source tree as well as ros2 run.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'common'))
from camera_viewer_rtsp import FrameGrabber
import cv2
import numpy as np
from rov_config import REAL_RTSP_URL
import tennis_ball_tracker as tracker
from tennis_ball_tracker import detect_ball, estimate_distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default=REAL_RTSP_URL)
    parser.add_argument('--h-min', type=int, default=tracker.BALL_H_MIN)
    parser.add_argument('--h-max', type=int, default=tracker.BALL_H_MAX)
    parser.add_argument('--s-min', type=int, default=tracker.BALL_S_MIN)
    parser.add_argument('--v-min', type=int, default=tracker.BALL_V_MIN)
    parser.add_argument('--detection-width', type=int, default=tracker.BALL_DETECTION_WIDTH)
    args = parser.parse_args()
    if not (0 <= args.h_min <= args.h_max <= 179 and
            0 <= args.s_min <= 255 and 0 <= args.v_min <= 255 and args.detection_width > 0):
        parser.error('Invalid HSV limits or detection width')
    windows = ('Ball debug - Camera', 'Ball debug - HSV mask')
    grabber = FrameGrabber(args.url)
    print('White = colour match after cleanup; circle = selected round target. Q/Esc exits.')
    try:
        for name in windows:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 800, 450)
        while True:
            frame, stamp = grabber.latest_sample()
            if frame is None or time.monotonic() - stamp >= .25:
                frame = np.zeros((450, 800, 3), dtype=np.uint8)
                mask = np.zeros((450, 800), dtype=np.uint8)
                label = 'Waiting for fresh camera frame'
            else:
                target, mask = detect_ball(frame, (args.h_min, args.s_min, args.v_min),
                                           (args.h_max, 255, 255), args.detection_width,
                                           return_mask=True)
                height, width = frame.shape[:2]
                label = f'{width}x{height} | No qualifying ball'
                if target is not None:
                    x, y, radius = (round(v) for v in target)
                    cv2.circle(frame, (x, y), radius, (0, 255, 255), 2)
                    distance = estimate_distance(target[2], width)
                    label = f'{width}x{height} | Ball radius {target[2]:.1f}px | est. {distance:.2f} m'
            cv2.putText(frame, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        .65, (0, 255, 255), 2)
            cv2.imshow(windows[0], frame)
            cv2.imshow(windows[1], mask)
            if cv2.waitKey(30) & 0xFF in (ord('q'), 27):
                break
            if any(cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1 for name in windows):
                break
    except KeyboardInterrupt:
        pass
    finally:
        grabber.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

