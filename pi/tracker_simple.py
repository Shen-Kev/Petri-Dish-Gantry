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
  6. Apply small EMA smoothing to suppress per-frame centroid jitter.
  7. Apply hardcoded (OFFSET_X_MM, OFFSET_Y_MM) to compute the gantry target.
  8. Clamp to the 135 mm circular workspace; enqueue for the serial thread.
  9. Display: live undistorted feed, mask overlay, crosshair on the tracked
     point, world coords, offset, and gantry HUD.

Workflow:
  - Click anywhere in the preview to (re)sample the color at that pixel.
    First click arms tracking; subsequent clicks retarget.
  - 'm' toggles the mask overlay (useful for tuning).
  - '[' and ']' shrink/grow the HSV color tolerance.
  - ',' and '.' adjust focus (closer / farther). Hold to scan.
  - 'a' toggles auto-focus.
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
from libcamera import controls

# ============================================================
# CONFIG — EDIT ME
# ============================================================
# How far ahead/aside of the tracked point the gantry should sit, in mm.
OFFSET_X_MM = 0.0
OFFSET_Y_MM = 0.0

# Camera focus.
# Camera Module 3 has motorized focus. The control is a "lens position"
# in dioptres: 0.0 = infinity, larger values = closer focus.
# Useful starting values:
#   0.0  = infinity (very far)
#   2.0  = ~50 cm
#   5.0  = ~20 cm
#   10.0 = ~10 cm (macro)
# Press ',' / '.' at runtime to nudge in 0.25 dioptre steps.
USE_AUTOFOCUS = False           # True = continuous AF, False = manual fixed
MANUAL_FOCUS_DIOPTRES = 5.0     # only used if USE_AUTOFOCUS is False

# HSV color-mask tolerance. Tweak with [/] hotkeys at runtime.
HSV_TOL_H = 10
HSV_TOL_S = 60
HSV_TOL_V = 60

# Minimum blob area in pixels — anything smaller is ignored as noise.
MIN_BLOB_AREA_PX = 50

# EMA smoothing on the world-mm output. Larger = more responsive but
# jitterier; smaller = smoother but laggy. 0.5 is a good middle ground.
# Set to 1.0 to disable smoothing entirely.
WORLD_SMOOTHING_ALPHA = 0.5

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

# How much to nudge focus per ',' / '.' press, in dioptres
FOCUS_STEP = 0.25
FOCUS_MIN  = 0.0
FOCUS_MAX  = 15.0   # camera module 3 supports up to ~15 dioptres

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

sampled_hsv = None
last_click_px = None


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
    """Build a binary mask of pixels near sample_hsv, with red-wrap handling."""
    h0, s0, v0 = sample_hsv
    s_lo = max(0, s0 - tol_s); s_hi = min(255, s0 + tol_s)
    v_lo = max(0, v0 - tol_v); v_hi = min(255, v0 + tol_v)

    h_lo = h0 - tol_h
    h_hi = h0 + tol_h

    if h_lo < 0:
        m1 = cv2.inRange(hsv_frame, (h_lo + 180, s_lo, v_lo), (179, s_hi, v_hi))
        m2 = cv2.inRange(hsv_frame, (0, s_lo, v_lo), (h_hi, s_hi, v_hi))
        mask = cv2.bitwise_or(m1, m2)
    elif h_hi > 179:
        m1 = cv2.inRange(hsv_frame, (h_lo, s_lo, v_lo), (179, s_hi, v_hi))
        m2 = cv2.inRange(hsv_frame, (0, s_lo, v_lo), (h_hi - 180, s_hi, v_hi))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv_frame, (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_blob(mask):
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
# Camera focus control
# ============================================================
def set_camera_focus(picam, autofocus, dioptres):
    """Configure focus on Camera Module 3.

    Camera Module 3 has a motorized lens controlled in dioptres:
      0 = infinity, larger = closer.
    AfMode 0 = manual, 2 = continuous autofocus.
    """
    if autofocus:
        picam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        print("[focus] autofocus ON (continuous)")
    else:
        d = float(max(FOCUS_MIN, min(FOCUS_MAX, dioptres)))
        picam.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": d,
        })
        print(f"[focus] manual @ {d:.2f} dioptres")


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
#
# Note: no longer rate-throttles outbound commands. Every fresh target
# is shipped immediately. Combined with Arduino-side preemption, this
# means the gantry is always heading toward the latest known position.
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
                tx, ty = target_queue.get(timeout=0.005)
            except queue.Empty:
                continue

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

    # Apply focus settings (auto-exposure intentionally left as default = on)
    autofocus = USE_AUTOFOCUS
    focus_dioptres = MANUAL_FOCUS_DIOPTRES
    set_camera_focus(picam, autofocus, focus_dioptres)

    print("[init] starting serial thread...")
    t_serial = threading.Thread(
        target=serial_worker, args=(SERIAL_PORT, SERIAL_BAUD), daemon=True
    )
    t_serial.start()

    win = "Tracker (simple)"
    cv2.namedWindow(win)
    cb_param = {"hsv": None}
    cv2.setMouseCallback(win, on_mouse, cb_param)

    show_mask = False
    sending_enabled = True
    tol_h, tol_s, tol_v = HSV_TOL_H, HSV_TOL_S, HSV_TOL_V

    smoothed_world = None  # (wx, wy) EMA state

    fps_t0, fps_n, fps = time.monotonic(), 0, 0.0
    print("Click on the colored marker to start tracking.")
    print("Hotkeys: m=mask, [/]=tolerance, ,/.=focus -/+, "
          "a=autofocus toggle, SPACE=pause sending, q=quit")

    try:
        while not shutdown_event.is_set():
            frame = picam.capture_array()
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
            cb_param["hsv"] = hsv

            display = undistorted.copy()
            blob = None

            if sampled_hsv is not None:
                mask = make_mask(hsv, sampled_hsv, tol_h, tol_s, tol_v)
                blob = detect_blob(mask)

                if show_mask:
                    overlay = display.copy()
                    overlay[mask > 0] = (0, 255, 0)
                    display = cv2.addWeighted(display, 0.6, overlay, 0.4, 0)

                if blob is not None:
                    cx, cy, area = blob
                    wx, wy = pixel_to_world(cx, cy, H)

                    # EMA smoothing on world coordinate. Cuts per-frame
                    # centroid jitter that otherwise causes the gantry
                    # to hunt and chatter.
                    if smoothed_world is None or WORLD_SMOOTHING_ALPHA >= 1.0:
                        smoothed_world = (wx, wy)
                    else:
                        a = WORLD_SMOOTHING_ALPHA
                        smoothed_world = (
                            a * wx + (1 - a) * smoothed_world[0],
                            a * wy + (1 - a) * smoothed_world[1],
                        )
                    swx, swy = smoothed_world

                    cv2.drawMarker(display, (int(cx), int(cy)),
                                   (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
                    cv2.circle(display, (int(cx), int(cy)), 18, (0, 0, 255), 2)

                    target_x = swx + OFFSET_X_MM
                    target_y = swy + OFFSET_Y_MM
                    in_bounds = (target_x ** 2 + target_y ** 2
                                 <= WORKSPACE_RADIUS_MM ** 2)

                    if in_bounds and sending_enabled:
                        enqueue_latest((target_x, target_y))

                    color_pt = (0, 255, 0) if in_bounds else (0, 0, 255)
                    cv2.putText(display,
                                f"point  ({swx:+6.1f}, {swy:+6.1f}) mm",
                                (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, color_pt, 2)
                    cv2.putText(display,
                                f"target ({target_x:+6.1f}, {target_y:+6.1f}) mm"
                                + ("" if in_bounds else "  OUT OF BOUNDS"),
                                (10, 110), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, color_pt, 2)
                else:
                    smoothed_world = None
                    cv2.putText(display, "no blob (try clicking again or '[/]')",
                                (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 165, 255), 2)
            else:
                cv2.putText(display, "Click on the colored marker to begin",
                            (10, 85), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)

            # HUD lines
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

            focus_str = (f"focus: AUTO" if autofocus
                         else f"focus: manual {focus_dioptres:.2f} dpt")
            cv2.putText(display, focus_str,
                        (FRAME_WIDTH - 280, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)

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
            elif key == ord(','):
                # focus closer (more dioptres) — wait, convention: ',' = farther
                # Standard convention: ',' looks like '<' (back/farther)
                # and '.' looks like '>' (forward/closer). Use that.
                autofocus = False
                focus_dioptres = max(FOCUS_MIN, focus_dioptres - FOCUS_STEP)
                set_camera_focus(picam, autofocus, focus_dioptres)
            elif key == ord('.'):
                autofocus = False
                focus_dioptres = min(FOCUS_MAX, focus_dioptres + FOCUS_STEP)
                set_camera_focus(picam, autofocus, focus_dioptres)
            elif key == ord('a'):
                autofocus = not autofocus
                set_camera_focus(picam, autofocus, focus_dioptres)
    finally:
        shutdown_event.set()
        t_serial.join(timeout=2.0)
        picam.stop()
        cv2.destroyAllWindows()
        print("[main] clean shutdown")


if __name__ == "__main__":
    main()
