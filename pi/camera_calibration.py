"""
Camera intrinsic calibration for the biohybrid tracker.

Run this ONCE per physical camera setup (or after any lens/focus change).
Print a checkerboard pattern (default: 9x6 INTERIOR corners, 25 mm squares)
and glue it to a rigid flat surface — a clipboard or acrylic sheet is ideal.

Hold the board in front of the upward-looking camera at varied angles,
distances, and positions within the frame. Press SPACE to capture a frame
when the green corner overlay appears. Press 'c' to compute and save
calibration once you have >= MIN_FRAMES good captures. 'q' to quit.

Output: camera_calibration.npz with keys K, dist, image_size.

Note on wide-angle lenses: if the lens FOV exceeds ~120 deg and you see
extreme barrel distortion near the edges, switch to cv2.fisheye.calibrate
(different API). For typical 90-110 deg wide-angle lenses, the standard
cv2.calibrateCamera used here is fine.
"""

import cv2
import numpy as np
from pathlib import Path
from picamera2 import Picamera2

# ---------------- CONFIG ----------------
CHESSBOARD_SIZE = (8, 5)      # (cols, rows) of INTERIOR corners
SQUARE_SIZE_MM  = 25.0
FRAME_WIDTH     = 1280
FRAME_HEIGHT    = 720
MIN_FRAMES      = 15
OUT_PATH        = Path("camera_calibration.npz")

# 3D object points for one view of the checkerboard (Z=0 plane)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM


def main():
    picam = Picamera2()
    # Picamera2 "RGB888" format returns numpy arrays in B,G,R byte order
    # (counterintuitive but documented) — that means no color conversion is
    # needed before passing frames to OpenCV. Do NOT call cv2.cvtColor(RGB2BGR).
    config = picam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()

    objpoints = []
    imgpoints = []
    captured = 0

    print(f"Capturing frames. Need >= {MIN_FRAMES}. "
          "SPACE = capture, 'c' = calibrate, 'q' = quit.")

    try:
        while True:
            frame = picam.capture_array()  # already BGR-ordered
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(
                gray, CHESSBOARD_SIZE,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            display = frame.copy()
            corners_refined = None
            if found:
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                )
                cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners_refined, found)

            cv2.putText(display, f"Captured: {captured}/{MIN_FRAMES}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Camera Calibration", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' ') and found:
                objpoints.append(objp.copy())
                imgpoints.append(corners_refined)
                captured += 1
                print(f"  captured {captured}")
            elif key == ord('c'):
                if captured < MIN_FRAMES:
                    print(f"  need at least {MIN_FRAMES} frames (have {captured})")
                    continue
                print("Computing calibration...")
                rms, K, dist, _, _ = cv2.calibrateCamera(
                    objpoints, imgpoints, gray.shape[::-1], None, None
                )
                print(f"  RMS reprojection error: {rms:.4f} px  (<1 is good, <0.5 excellent)")
                print(f"  Camera matrix K:\n{K}")
                print(f"  Distortion coeffs: {dist.ravel()}")
                np.savez(
                    OUT_PATH,
                    K=K, dist=dist,
                    image_size=np.array([FRAME_WIDTH, FRAME_HEIGHT]),
                )
                print(f"Saved -> {OUT_PATH.resolve()}")
                break
    finally:
        picam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
