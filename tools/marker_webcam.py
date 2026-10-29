#!/usr/bin/env python3
"""Try the dock-marker detector on a laptop webcam (no robot), or print the marker.

    pip install opencv-contrib-python pyyaml      (or: sudo apt install python3-opencv)
    python3 tools/marker_webcam.py --print marker.png          # 8 cm marker at 300 dpi: print at 100 %, check with a ruler
    python3 tools/marker_webcam.py                             # webcam 0, live view with the pose
    python3 tools/marker_webcam.py --camera 1 --calib my_calib.yaml --size 0.08 --id 0
    python3 tools/marker_webcam.py --headless --seconds 10     # no window: prints the readings

What to check (docs/docking.md): z_m matches a tape measure (+-1 cm at 0.3-1 m), x_m goes + when the marker
moves to the right, yaw_deg goes + when the marker is turned so its face points to the camera's right.
A laptop webcam is not the robot camera: use --calib with that webcam's calibration for real numbers
(without one the Camera Module 3 defaults are used, so distances are only roughly right).
Same detector and same output as the robot: lidar_robot.marker_pose.detect().
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'lidar_robot'))
from lidar_robot.marker_pose import MarkerDetector, load_calibration, render_marker  # noqa: E402


def print_marker(path, dictionary, marker_id, size_m, dpi=300):
    import cv2
    import numpy as np
    px = int(round(size_m / 0.0254 * dpi))
    border = int(round(px * 0.25))                   # white quiet zone around the black square
    mk = render_marker(dictionary, marker_id, px)
    img = np.full((px + 2 * border, px + 2 * border), 255, np.uint8)
    img[border:border + px, border:border + px] = mk
    cv2.putText(img, f'{dictionary} id {marker_id}: {size_m * 100:.1f} cm black square', (border, img.shape[0] - border // 3),
                cv2.FONT_HERSHEY_SIMPLEX, max(0.4, px / 900), 0, 1, cv2.LINE_AA)
    cv2.imwrite(path, img)
    print(f'saved {path}: {img.shape[1]}x{img.shape[0]} px at {dpi} dpi. Print at 100 % (no "fit to page") and '
          f'check the black square is {size_m * 100:.1f} cm. Stick it flat, centred on the dock face.')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--camera', default='0', help='webcam index or video device')
    ap.add_argument('--calib', help='camera calibration YAML (width, height, camera_matrix, dist_coeffs)')
    ap.add_argument('--dict', default='DICT_4X4_50')
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('--size', type=float, default=0.08, help='black square side (m)')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--seconds', type=float, default=0, help='stop after this long (0 = until q)')
    ap.add_argument('--print', dest='print_path', help='write a printable marker PNG and exit')
    a = ap.parse_args(argv)
    if a.print_path:
        print_marker(a.print_path, a.dict, a.id, a.size)
        return 0
    import cv2
    det = MarkerDetector(load_calibration(a.calib), a.dict, a.id, a.size)
    if det.error:
        print('!!', det.error)
        return 1
    cap = cv2.VideoCapture(int(a.camera) if a.camera.isdigit() else a.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if not cap.isOpened():
        print(f'!! cannot open camera {a.camera}')
        return 1
    print(f'calibration: {det.calib["source"]}. Press q to quit.')
    t0 = last = time.monotonic()
    while True:
        ok, frame = cap.read()
        if not ok:
            print('!! no frame from the camera')
            break
        r = det.detect(frame)
        now = time.monotonic()
        if now - last > 0.2:
            last = now
            print(f'found  x {r["x_m"]:+.3f} m  y {r["y_m"]:+.3f} m  z {r["z_m"]:.3f} m  yaw {r["yaw_deg"]:+5.1f} deg'
                  f'  ({r["side_px"]:.0f} px)' if r['found'] else 'no marker', flush=True)
        if not a.headless:
            img = frame.copy()
            if r['found']:
                cv2.circle(img, (int(r['px'][0]), int(r['px'][1])), 6, (0, 255, 0), 2)
                txt = f'z {r["z_m"]:.2f} m  x {r["x_m"]:+.2f} m  yaw {r["yaw_deg"]:+.0f} deg'
            else:
                txt = 'no marker'
            cv2.putText(img, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if r['found'] else (0, 0, 255), 2)
            cv2.imshow('dock marker (q quits)', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        if a.seconds and now - t0 > a.seconds:
            break
    cap.release()
    if not a.headless:
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
