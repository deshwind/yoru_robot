#!/usr/bin/env python3
"""Homography calibration for the real CCTV camera (dissertation Section 4.5).

Usage:
  1. Place >= 4 markers (ArUco, coloured cones, tape crosses) on the floor at
     positions you have measured in metres relative to a chosen ground origin
     (e.g. directly below the camera, X along the wall).
  2. Run:  python3 calibrate_homography.py --image cctv_frame.jpg
  3. Click the markers in the image IN THE SAME ORDER as the --ground points.
  4. The script prints the 3x3 homography (ground -> pixel) as a flat list for
     coordinate_transform_node's 'homography' parameter, plus reprojection error.

Example:
  python3 calibrate_homography.py --image frame.jpg \
      --ground 0,0 2,0 2,3 0,3
"""

import argparse

import cv2
import numpy as np

clicks = []


def on_mouse(event, x, y, _flags, _param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicks.append((x, y))
        print(f'  clicked pixel {len(clicks)}: ({x}, {y})')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, help='CCTV frame image file')
    parser.add_argument('--ground', nargs='+', required=True,
                        help='Ground points in metres as x,y (>= 4)')
    args = parser.parse_args()

    ground = np.array([[float(v) for v in p.split(',')] for p in args.ground],
                      dtype=np.float32)
    if len(ground) < 4:
        raise SystemExit('Need at least 4 ground points')

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f'Cannot read image: {args.image}')

    print(f'Click the {len(ground)} marker positions in order, then press q.')
    cv2.namedWindow('calibrate')
    cv2.setMouseCallback('calibrate', on_mouse)
    while True:
        disp = img.copy()
        for i, (x, y) in enumerate(clicks):
            cv2.circle(disp, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(disp, str(i + 1), (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
        cv2.imshow('calibrate', disp)
        if cv2.waitKey(30) & 0xFF == ord('q') or len(clicks) == len(ground):
            break
    cv2.destroyAllWindows()

    if len(clicks) != len(ground):
        raise SystemExit(f'Clicked {len(clicks)} points, need {len(ground)}')

    pixels = np.array(clicks, dtype=np.float32)
    h, mask = cv2.findHomography(ground, pixels, cv2.RANSAC, 3.0)

    reproj = cv2.perspectiveTransform(ground.reshape(-1, 1, 2), h).reshape(-1, 2)
    err = np.linalg.norm(reproj - pixels, axis=1)
    print(f'\nInliers: {int(mask.sum())}/{len(ground)}')
    print(f'Reprojection error: mean {err.mean():.2f}px, max {err.max():.2f}px')
    print('\nhomography parameter (row-major, ground->pixel):')
    print('  homography: [' + ', '.join(f'{v:.6f}' for v in h.flatten()) + ']')
    print('\nAlso set camera_yaw_2d and camera_xy_2d to the pose of your ground '
          'origin in the map frame.')


if __name__ == '__main__':
    main()
