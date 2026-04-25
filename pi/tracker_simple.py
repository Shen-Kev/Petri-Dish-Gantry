"""
tracker_simple.py — color-blob tracker for the biohybrid gantry.

A simpler alternative to tracker.py that requires NO YOLO model. Use this
to validate the whole pipeline (camera -> world coords -> gantry follow)
before training a detector.

Pipeline per frame:
  1. Capture from Picamera2.
  2. Undistort with the saved camera intrinsics.
  3. Convert to HSV; mask pixels near the sampled color.
  4. Find the largest connected blob; take its centroid.
  5. Map centroid pixel -> world mm via the saved homography.
  6. Apply hardcoded (OFFSET_X_MM, OFFSET_Y_MM) to compute the gantry target.
  7. Clamp to the 135 mm circular workspace; enqueue for the serial thread.
  8. Display: live undistorted feed, mask overlay, crosshair on the tracked
     point, world coords, offset, and gantry HUD.

Workflow:
  - Click anywhere in the preview to (re)sample the color at that pixel.
    First click arms tracking; subsequent clicks retarget.
  - 'm' toggles the mask overlay (useful for tuning).
  - '[' and ']' shrink/grow the HSV color tolerance.
  - SPACE pauses/unpauses sending commands to the gantry.
  - 'q' or Ctrl+C quits cleanly.

Edit OFFSET_X_MM and OFFSET_Y_MM near the top to change the lead/follow
distance. The gantry will move to (point + offset).

Prereqs:
  - workspace_calibration.npz must exist (run camera_calibration.py and
    workspace_calibration.py first).
  - ESP32 firmware running and wired to the Pi via /dev/ttyAMA0.
"""

import queue
import signal
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import serial
from picamera2 import Picamera2

# ============================================================
# CONFIG — EDIT ME
# ============================================================
# How far ahead/aside of the tracked point the gantry should sit, in mm.
# Positive X is whatever +X means in your workspace_calibration. Same for Y.
OFFSET_X_MM = 0.0
OFFSET_Y_MM = 10.0

# HSV color-mask tolerance. Wider = more permissive (catches more pixels of
# similar shades). Tweak with [ and ] hotkeys at runtime; final value can
# be hardcoded here once you find what works.
HSV_TOL_H = 10     # Hue tolerance (OpenCV hue range is 0..179)
HSV_TOL_S = 60     # Saturation tolerance (0..255)
HSV_TOL_V = 60     # Value tolerance (0..255)

# Minimum blob area in pixels — anything smaller is ignored as noise.
MIN_BLOB_AREA_PX = 50

# ============================================================
# CONFIG — usually no need to edit
# ============================================================
WORKSPACE_CALIB_PATH = Path("workspace_calibration.npz")

FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720

WORKSPACE_DIAMETER_MM = 135.0
WORKSPACE_RADIUS_MM   = WORKSPACE_DIAMETER_MM / 2

SERIAL_PORT       = "/dev/ttyAMA0"
SERIAL_BAUD       = 115200
SERIAL_TIMEOUT_S  = 0.05
COMMAND_PERIOD_S  = 0.02   # 50 Hz cap on outbound commands

# ============================================================
# GLOBALS
# ============================================================
shutdown_event = threading.Event()
target_queue   = queue.Queue(maxsize=1)

arduino_state_lock = threading.Lock()
arduino_state = {
    "last_ack_id":  None,
    "reported_pos": None,
    "last_error":   None,
    "last_rx_time": 0.0,
}

# Sample state mutated by the mouse callback. None until first click.
sampled_hsv = None  # (h, s, v) tuple or None
last_click_px = None  # (x, y) for visual feedback


# ============================================================
# Calibration + transform
# ============================================================
def load_calibration():
    if not WORKSPACE_CALIB_PATH.exists():
        raise FileNotFoundError(
            f"Missing {WORKSPACE_CALIB_PATH}. "
            "Run camera_calibration.py then workspace_calibration.py first."
        )
    data = np.load(WORKSPACE_CALIB_PATH)
    H, K, dist, new_K = data["H"], data["K"], data["dist"], data["new_K"]
    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist, None, new_K, (FRAME_WIDTH, FRAME_HEIGHT), cv2.CV_16SC2
    )
    return H, map1, map2


def pixel_to_world(px, py, H):
    v = H @ np.array([px, py, 1.0], dtype=np.float64)
    return float(v[0] / v[2]), float(v[1] / v[2])


# ============================================================
# Color-blob detection
# ============================================================
def make_mask(hsv_frame, sample_hsv, tol_h, tol_s, tol_v):
    """Build a binary mask of pixels near sample_hsv.

    Hue wraps at 180 in OpenCV (red sits at both ~0 and ~179), so we
    handle wrap-around with two ranges OR'd together when needed.
    """
    h0, s0, v0 = sample_hsv
    s_lo = max(0, s0 - tol_s); s_hi = min(255, s0 + tol_s)
    v_lo = max(0, v0 - tol_v); v_hi = min(255, v0 + tol_v)

    h_lo = h0 - tol_h
    h_hi = h0 + tol_h

    if h_lo < 0:
        # wrap: e.g. red at h=5 with tol=10 -> [175..179] OR [0..15]
        m1 = cv2.inRange(hsv_frame, (h_lo + 180, s_lo, v_lo), (179, s_hi, v_hi))
        m2 = cv2.inRange(hsv_frame, (0, s_lo, v_lo), (h_hi, s_hi, v_hi))
        mask = cv2.bitwise_or(m1, m2)
    elif h_hi > 179:
        m1 = cv2.inRange(hsv_frame, (h_lo, s_lo, v_lo), (179, s_hi, v_hi))
        m2 = cv2.inRange(hsv_frame, (0, s_lo, v_lo), (h_hi - 180, s_hi, v_hi))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv_frame, (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))

    # Clean up: erode noise, dilate to fill small gaps inside the blob.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_blob(mask):
    """Return (cx, cy, area_px) of the largest blob, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_BLOB_AREA_PX:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy, area


# ============================================================
# Mouse callback — click to sample color
# ============================================================
def on_mouse(event, x, y, flags, param):
    global sampled_hsv, last_click_px
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    hsv_frame = param.get("hsv")
    if hsv_frame is None:
        return
    h, w = hsv_frame.shape[:2]
    if not (0 <= x < w and 0 <= y < h):
        return
    # Average a small 5x5 patch around the click for noise robustness.
    x0, x1 = max(0, x - 2), min(w, x + 3)
    y0, y1 = max(0, y - 2), min(h, y + 3)
    patch = hsv_frame[y0:y1, x0:x1].reshape(-1, 3).astype(np.float32)
    mean_hsv = patch.mean(axis=0)
    sampled_hsv = (int(round(mean_hsv[0])),
                   int(round(mean_hsv[1])),
                   int(round(mean_hsv[2])))
    last_click_px = (x, y)
    print(f"[sample] clicked ({x},{y}) -> HSV {sampled_hsv}")


# ============================================================
# Inbound message handling (from Arduino)
# ============================================================
def handle_arduino_message(payload: str) -> None:
    now = time.monotonic()
    if payload.startswith("ACK:"):
        try:
            cmd_id = int(payload[4:])
        except ValueError:
            return
        with arduino_state_lock:
            arduino_state["last_ack_id"] = cmd_id
            arduino_state["last_rx_time"] = now
    elif payload.startswith("POS:"):
        try:
            x_str, y_str = payload[4:].split(",")
            pos = (float(x_str), float(y_str))
        except ValueError:
            return
        with arduino_state_lock:
            arduino_state["reported_pos"] = pos
            arduino_state["last_rx_time"] = now
    elif payload.startswith("ERR:"):
        msg = payload[4:]
        with arduino_state_lock:
            arduino_state["last_error"] = msg
            arduino_state["last_rx_time"] = now
        print(f"[arduino] ERROR: {msg}")


# ============================================================
# Serial worker (bidirectional, non-blocking)
# ============================================================
def serial_worker(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0, write_timeout=SERIAL_TIMEOUT_S)
    except serial.SerialException as e:
        print(f"[serial] open failed on {port}: {e}")
        shutdown_event.set()
        return

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    print(f"[serial] connected {port} @ {baud}")

    rx_buf = bytearray()
    last_send = 0.0

    try:
        while not shutdown_event.is_set():
            try:
                chunk = ser.read(256)
            except serial.SerialException as e:
                print(f"[serial] read error: {e}")
                break
            if chunk:
                rx_buf.extend(chunk)
                while b"\n" in rx_buf:
                    line, _, rest = rx_buf.partition(b"\n")
                    rx_buf = bytearray(rest)
                    s = line.decode("ascii", errors="replace").strip()
                    if s.startswith("<") and s.endswith(">"):
                        handle_arduino_message(s[1:-1])
                    elif s:
                        print(f"[arduino-raw] {s!r}")

            try:
                tx, ty = target_queue.get_nowait()
            except queue.Empty:
                time.sleep(0.002)
                continue

            now = time.monotonic()
            dt = now - last_send
            if dt < COMMAND_PERIOD_S:
                time.sleep(COMMAND_PERIOD_S - dt)
            last_send = time.monotonic()

            msg = f"<{tx:.2f},{ty:.2f}>\n"
            try:
                ser.write(msg.encode("ascii"))
            except serial.SerialTimeoutException:
                print("[serial] write timeout")
            except serial.SerialException as e:
                print(f"[serial] write error: {e}")
                break
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("[serial] closed")


def enqueue_latest(item):
    try:
        target_queue.put_nowait(item)
    except queue.Full:
        try:
            target_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            target_queue.put_nowait(item)
        except queue.Full:
            pass


# ============================================================
# Main
# ============================================================
def main():
    global sampled_hsv

    signal.signal(signal.SIGINT, lambda *_: shutdown_event.set())

    print("[init] loading calibration...")
    H, map1, map2 = load_calibration()

    print("[init] starting camera...")
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.5)

    print("[init] starting serial thread...")
    t_serial = threading.Thread(
        target=serial_worker, args=(SERIAL_PORT, SERIAL_BAUD), daemon=True
    )
    t_serial.start()

    win = "Tracker (simple)"
    cv2.namedWindow(win)
    # Mouse callback closure shares a dict so we can update the HSV frame
    # reference each iteration (callback fires asynchronously).
    cb_param = {"hsv": None}
    cv2.setMouseCallback(win, on_mouse, cb_param)

    show_mask = False
    sending_enabled = True
    tol_h, tol_s, tol_v = HSV_TOL_H, HSV_TOL_S, HSV_TOL_V

    fps_t0, fps_n, fps = time.monotonic(), 0, 0.0
    print("Click on the colored marker to start tracking.")
    print("Hotkeys: m=toggle mask, [/]=shrink/grow tolerance, "
          "SPACE=pause sending, q=quit")

    try:
        while not shutdown_event.is_set():
            frame = picam.capture_array()
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
            cb_param["hsv"] = hsv  # callback can now sample from this frame

            display = undistorted.copy()
            blob = None

            if sampled_hsv is not None:
                mask = make_mask(hsv, sampled_hsv, tol_h, tol_s, tol_v)
                blob = detect_blob(mask)

                if show_mask:
                    # Tint the mask in green and blend it on top of the feed.
                    overlay = display.copy()
                    overlay[mask > 0] = (0, 255, 0)
                    display = cv2.addWeighted(display, 0.6, overlay, 0.4, 0)

                if blob is not None:
                    cx, cy, area = blob
                    wx, wy = pixel_to_world(cx, cy, H)

                    # Crosshair on the tracked point
                    cv2.drawMarker(display, (int(cx), int(cy)),
                                   (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
                    cv2.circle(display, (int(cx), int(cy)), 18, (0, 0, 255), 2)

                    # Compute and send target if inside the workspace
                    target_x = wx + OFFSET_X_MM
                    target_y = wy + OFFSET_Y_MM
                    in_bounds = (target_x ** 2 + target_y ** 2
                                 <= WORKSPACE_RADIUS_MM ** 2)

                    if in_bounds and sending_enabled:
                        enqueue_latest((target_x, target_y))

                    # Draw a small marker where the gantry is being told to go
                    # (only meaningful if we can map mm back to pixels — skip).
                    color_pt = (0, 255, 0) if in_bounds else (0, 0, 255)
                    cv2.putText(display,
                                f"point  ({wx:+6.1f}, {wy:+6.1f}) mm",
                                (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, color_pt, 2)
                    cv2.putText(display,
                                f"target ({target_x:+6.1f}, {target_y:+6.1f}) mm"
                                + ("" if in_bounds else "  OUT OF BOUNDS"),
                                (10, 110), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, color_pt, 2)
                else:
                    cv2.putText(display, "no blob (try clicking again or '[/]')",
                                (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 165, 255), 2)
            else:
                cv2.putText(display, "Click on the colored marker to begin",
                            (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)

            # HUD: FPS, sampled color, tolerances, offsets, gantry state
            fps_n += 1
            if fps_n >= 10:
                now = time.monotonic()
                fps = fps_n / (now - fps_t0)
                fps_t0, fps_n = now, 0
            cv2.putText(display, f"{fps:4.1f} FPS", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            sample_str = (f"sample HSV {sampled_hsv}  "
                          f"tol H{tol_h} S{tol_s} V{tol_v}"
                          if sampled_hsv else "sample: none yet")
            cv2.putText(display, sample_str, (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1)

            cv2.putText(display,
                        f"offset ({OFFSET_X_MM:+.1f}, {OFFSET_Y_MM:+.1f}) mm   "
                        + ("SENDING" if sending_enabled else "PAUSED"),
                        (10, FRAME_HEIGHT - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if sending_enabled else (0, 0, 255), 2)

            with arduino_state_lock:
                rep_pos = arduino_state["reported_pos"]
                last_ack = arduino_state["last_ack_id"]
                last_rx = arduino_state["last_rx_time"]
            if rep_pos is not None:
                age_ms = (time.monotonic() - last_rx) * 1000
                color = (0, 255, 255) if age_ms < 500 else (0, 0, 255)
                cv2.putText(display,
                            f"gantry ({rep_pos[0]:+6.1f}, {rep_pos[1]:+6.1f}) mm "
                            f"ack#{last_ack} age{age_ms:4.0f}ms",
                            (10, FRAME_HEIGHT - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            else:
                cv2.putText(display, "gantry: no data",
                            (10, FRAME_HEIGHT - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            # Show last click location briefly
            if last_click_px is not None:
                cv2.circle(display, last_click_px, 6, (255, 255, 0), 1)

            cv2.imshow(win, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                shutdown_event.set()
            elif key == ord('m'):
                show_mask = not show_mask
            elif key == ord('['):
                tol_h = max(1, tol_h - 2)
                tol_s = max(5, tol_s - 10)
                tol_v = max(5, tol_v - 10)
                print(f"[tol] H{tol_h} S{tol_s} V{tol_v}")
            elif key == ord(']'):
                tol_h = min(89, tol_h + 2)
                tol_s = min(255, tol_s + 10)
                tol_v = min(255, tol_v + 10)
                print(f"[tol] H{tol_h} S{tol_s} V{tol_v}")
            elif key == ord(' '):
                sending_enabled = not sending_enabled
                print(f"[send] {'ENABLED' if sending_enabled else 'PAUSED'}")
    finally:
        shutdown_event.set()
        t_serial.join(timeout=2.0)
        picam.stop()
        cv2.destroyAllWindows()
        print("[main] clean shutdown")


if __name__ == "__main__":
    main()
