"""
Workspace perspective calibration.

Produces the 3x3 homography that maps UNDISTORTED pixel coordinates to
real-world millimeters on the gantry plane.

Place 4 fiducial markers at known world coordinates on the workspace:
    A = (+67.5,   0.0)   # +X axis
    B = (  0.0, +67.5)   # +Y axis
    C = (-67.5,   0.0)   # -X axis
    D = (  0.0, -67.5)   # -Y axis

A printed crosshair, a dot of paint, or an ArUco tag all work; you just
need to be able to see and click the center of each.

Click the 4 fiducials IN ORDER (A, B, C, D) in the undistorted view.
's' = save, 'r' = reset, 'q' = quit.

IMPORTANT (upward-looking camera): looking up at the plane produces a
mirror image compared to looking down. The sign of one world axis may
feel "flipped". Your convention is whatever you CLICK, so pick an axis
convention that matches your Arduino gantry's (+X, +Y). After saving,
jog the gantry +X by a known amount and confirm the reported world
coordinate increases in +X too. If it decreases, swap your A and C
fiducial clicks (or B and D) and re-save.

Output: workspace_calibration.npz with keys H (3x3), K, dist, new_K.
"""

import cv2
import numpy as np
from pathlib import Path
from picamera2 import Picamera2

# ---------------- CONFIG ----------------
CAMERA_CALIB_PATH = Path("camera_calibration.npz")
OUT_PATH          = Path("workspace_calibration.npz")
FRAME_WIDTH       = 1280
FRAME_HEIGHT      = 720

# World coords (mm) of the 4 fiducials, matched to click order A,B,C,D
WORLD_POINTS = np.array([
    [ 23,   0.0],   # A: +X (long side of one of those mini breadboards)
    [  0.0,  18],   # B: +Y (short side of one of those mini breadboards)
    [-23,   0.0],   # C: -X (long side of one of those mini breadboards)
    [  0.0, -18],   # D: -Y (short side of one of those mini breadboards)
], dtype=np.float32)

clicked_points = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        wp = WORLD_POINTS[len(clicked_points) - 1]
        print(f"  click {len(clicked_points)}: pixel=({x},{y})  world=({wp[0]:+.1f},{wp[1]:+.1f}) mm")


def main():
    if not CAMERA_CALIB_PATH.exists():
        raise FileNotFoundError(
            f"Missing {CAMERA_CALIB_PATH}. Run camera_calibration.py first."
        )
    calib = np.load(CAMERA_CALIB_PATH)
    K, dist = calib["K"], calib["dist"]

    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()

    # Pre-compute undistortion maps for real-time use
    new_K, _ = cv2.getOptimalNewCameraMatrix(
        K, dist, (FRAME_WIDTH, FRAME_HEIGHT), alpha=1
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist, None, new_K, (FRAME_WIDTH, FRAME_HEIGHT), cv2.CV_16SC2
    )

    cv2.namedWindow("Workspace Calibration")
    cv2.setMouseCallback("Workspace Calibration", on_mouse)

    print("Click fiducials in order: +X, +Y, -X, -Y. 's'=save 'r'=reset 'q'=quit")

    try:
        while True:
            frame = picam.capture_array()  # BGR-ordered
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            for i, (x, y) in enumerate(clicked_points):
                cv2.circle(undistorted, (x, y), 8, (0, 255, 0), 2)
                cv2.putText(undistorted, f"{i+1}", (x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(undistorted, f"Clicks {len(clicked_points)}/4",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("Workspace Calibration", undistorted)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                clicked_points.clear()
                print("reset.")
            elif key == ord('s'):
                if len(clicked_points) != 4:
                    print("need exactly 4 clicks.")
                    continue
                img_pts = np.array(clicked_points, dtype=np.float32)
                H = cv2.getPerspectiveTransform(img_pts, WORLD_POINTS)
                print("Sanity check (clicked pixels -> world mm):")
                for ip, wp in zip(img_pts, WORLD_POINTS):
                    v = H @ np.array([ip[0], ip[1], 1.0])
                    v = v[:2] / v[2]
                    print(f"  img {tuple(ip)} -> world ({v[0]:+.2f},{v[1]:+.2f}) "
                          f"expected ({wp[0]:+.2f},{wp[1]:+.2f})")
                np.savez(
                    OUT_PATH,
                    H=H, K=K, dist=dist, new_K=new_K,
                    image_size=np.array([FRAME_WIDTH, FRAME_HEIGHT]),
                )
                print(f"Saved -> {OUT_PATH.resolve()}")
                break
    finally:
        picam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
