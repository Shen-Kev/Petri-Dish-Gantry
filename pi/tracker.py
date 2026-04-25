#LIKELY OUTDATED. BUILD FROM TRACKER_SIMPLE.




"""
Biohybrid robot tracker — main loop.

Pipeline per frame:
  1. Capture from Picamera2 (libcamera-backed, RPi 5 friendly).
  2. Undistort with saved camera intrinsics (wide-angle barrel correction).
  3. YOLO localizes the biohybrid robot (robust to cluttered background).
  4. Refine pose with cv2.minAreaRect on a thresholded ROI crop -> (cx,cy,angle).
  5. Map undistorted pixel centroid -> world mm via saved homography.
  6. Reject detections outside the 135 mm circular workspace.
  7. Placeholder control law returns gantry (target_x, target_y) in mm.
  8. Push latest target into a size-1 queue; a dedicated serial worker
     thread sends it to the Arduino as "<X,Y>\\n" AND reads framed
     replies ("<ACK:N>", "<POS:X,Y>", "<ERR:msg>") back from the firmware.
     The size-1 queue drops stale targets so the Arduino always receives
     the FRESHEST command, and the vision loop is never blocked by I/O.

Serial port note (Pi 5 specific):
  We connect to /dev/ttyAMA0 directly, NOT /dev/serial0. On the Pi 5,
  /dev/serial0 points to a separate debug UART (ttyAMA10) on a dedicated
  JST connector — NOT to GPIO pins 8/10. The UART on GPIO 14/15 is
  always ttyAMA0. This requires these entries in /boot/firmware/config.txt:
      dtparam=uart0=on
      dtoverlay=disable-bt

Run order:
  1. python camera_calibration.py      # one-time, produces camera_calibration.npz
  2. python workspace_calibration.py   # one-time, produces workspace_calibration.npz
  3. python tracker.py                 # main loop

Convention:
  World X, Y are millimeters with origin at the center of the 135 mm
  circular workspace. Camera looks upward at the workspace from below;
  axis signs are fixed by the workspace calibration step.
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
from ultralytics import YOLO

# ---------------- CONFIG ----------------
WORKSPACE_CALIB_PATH = Path("workspace_calibration.npz")
YOLO_WEIGHTS_PATH    = Path("biohybrid.pt")

FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720

WORKSPACE_DIAMETER_MM = 135.0
WORKSPACE_RADIUS_MM   = WORKSPACE_DIAMETER_MM / 2

# Pi 5 GPIO UART on pins 8 (TXD) / 10 (RXD). NOT /dev/serial0 — see
# module docstring. Change to /dev/ttyACM0 only if you switch back to USB.
SERIAL_PORT      = "/dev/ttyAMA0"
SERIAL_BAUD      = 115200
SERIAL_TIMEOUT_S = 0.05
COMMAND_PERIOD_S = 0.02   # 50 Hz soft throttle on outbound commands

YOLO_CONF_THRESH = 0.5
YOLO_IMG_SIZE    = 640
YOLO_CLASS_ID    = 0      # single-class model -> 0. Adjust if multi-class.
ROI_PAD_PX       = 10     # pad YOLO bbox before contour pose refinement

# ---------------- GLOBALS ----------------
shutdown_event = threading.Event()
target_queue   = queue.Queue(maxsize=1)

# Shared state written by the serial thread, read by the main thread.
# Protected by a lock; contents are whatever the Arduino most recently
# told us about itself.
arduino_state_lock = threading.Lock()
arduino_state = {
    "last_ack_id":  None,    # int or None
    "reported_pos": None,    # (x_mm, y_mm) or None
    "last_error":   None,    # str or None
    "last_rx_time": 0.0,     # monotonic seconds; 0 = never
}


# ============================================================
# Calibration + coordinate transforms
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
    """Map a single undistorted (px, py) pixel to (wx, wy) mm."""
    v = H @ np.array([px, py, 1.0], dtype=np.float64)
    return float(v[0] / v[2]), float(v[1] / v[2])


# ============================================================
# Control placeholder
# ============================================================
def calculate_gantry_offset(obj_x, obj_y, pose_deg):
    """
    Placeholder control law. Given the biohybrid's current world position
    (obj_x, obj_y in mm) and pose (degrees), return the gantry target
    (target_x, target_y in mm, absolute coordinates) that the Arduino
    should move to.

    The gantry DRIVES the robot (e.g., an electromagnet under the stage
    pulling the biohybrid), so the real control law will typically:
      - Compute an error vector toward a goal/waypoint,
      - Offset a small lead distance ahead of the robot in that direction,
      - Possibly bias laterally based on pose to steer the robot.

    For now: return the robot's own position so the actuator parks
    directly under it. Replace with your actual control law.
    """
    target_x = obj_x
    target_y = obj_y
    return target_x, target_y


# ============================================================
# Detection + pose extraction
# ============================================================
def detect_with_pose(frame_bgr, model):
    """
    Run YOLO -> refine pose with minAreaRect on a thresholded ROI.
    Returns (cx_px, cy_px, angle_deg, (x1,y1,x2,y2)) or None.
    """
    results = model.predict(
        frame_bgr,
        imgsz=YOLO_IMG_SIZE,
        conf=YOLO_CONF_THRESH,
        classes=[YOLO_CLASS_ID],
        verbose=False,
    )
    if not results or len(results[0].boxes) == 0:
        return None

    boxes = results[0].boxes
    best = int(np.argmax(boxes.conf.cpu().numpy()))
    x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy().astype(int)

    h, w = frame_bgr.shape[:2]
    x1p = max(0, x1 - ROI_PAD_PX); y1p = max(0, y1 - ROI_PAD_PX)
    x2p = min(w, x2 + ROI_PAD_PX); y2p = min(h, y2 + ROI_PAD_PX)
    roi = frame_bgr[y1p:y2p, x1p:x2p]
    if roi.size == 0:
        return None

    # Otsu is a safe default; if your robot has a distinctive hue,
    # swap this for an HSV mask for more robust segmentation.
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if np.mean(th) > 127:
        th = 255 - th  # ensure robot is foreground (white)

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        # Fallback: bbox center, unknown rotation
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, 0.0, (x1, y1, x2, y2)

    largest = max(contours, key=cv2.contourArea)
    (rcx, rcy), (rw, rh), angle = cv2.minAreaRect(largest)

    # cv2.minAreaRect returns angle in [-90, 0). Normalize so that
    # the reported angle is that of the rectangle's LONG axis in [0, 180).
    if rw < rh:
        angle += 90.0
    angle = angle % 180.0

    return rcx + x1p, rcy + y1p, angle, (x1, y1, x2, y2)


# ============================================================
# Inbound message handling (from Arduino)
# ============================================================
def handle_arduino_message(payload: str) -> None:
    """
    Called from the serial thread for each "<...>" frame received.
    `payload` excludes the angle brackets. Supported types:
        ACK:<id>         - acknowledgement of a command
        POS:<x>,<y>      - Arduino's reported current position (mm)
        ERR:<message>    - firmware-side error
    """
    now = time.monotonic()

    if payload.startswith("ACK:"):
        try:
            cmd_id = int(payload[4:])
        except ValueError:
            print(f"[arduino] bad ACK payload: {payload!r}")
            return
        with arduino_state_lock:
            arduino_state["last_ack_id"] = cmd_id
            arduino_state["last_rx_time"] = now

    elif payload.startswith("POS:"):
        try:
            x_str, y_str = payload[4:].split(",")
            pos = (float(x_str), float(y_str))
        except ValueError:
            print(f"[arduino] bad POS payload: {payload!r}")
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

    else:
        print(f"[arduino] unknown frame: {payload!r}")


# ============================================================
# Serial worker (bidirectional, non-blocking)
# ============================================================
def serial_worker(port, baud):
    """Bidirectional serial worker.

    - Writes the freshest (tx, ty) from target_queue as "<X,Y>\\n".
    - Reads any "<...>\\n" framed message from the Arduino without blocking.
    - Never stalls the vision loop: all I/O is on this thread.

    NOTE (Pi 5 GPIO UART): unlike USB-connected Arduinos, this link does
    NOT trigger an auto-reset when the port is opened, so there is no
    bootloader delay to wait out. The firmware is running whenever the
    ESP32 is powered, independently of whether the Pi has the port open.
    """
    try:
        ser = serial.Serial(
            port, baud,
            timeout=0,               # non-blocking reads
            write_timeout=SERIAL_TIMEOUT_S,
        )
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
            # --- drain any inbound bytes, parse complete frames ---
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
                    try:
                        s = line.decode("ascii", errors="replace").strip()
                    except Exception:
                        continue
                    if s.startswith("<") and s.endswith(">"):
                        handle_arduino_message(s[1:-1])
                    elif s:
                        # Unframed output (shouldn't happen with our firmware,
                        # but log it rather than silently dropping).
                        print(f"[arduino-raw] {s!r}")

            # --- write latest target if one is queued and rate allows ---
            try:
                tx, ty = target_queue.get_nowait()
            except queue.Empty:
                # No new command pending; sleep briefly so we don't burn CPU.
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
                print("[serial] write timeout — Arduino not draining?")
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
    """Replace whatever is in the queue with the newest value (drop-old)."""
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
    signal.signal(signal.SIGINT, lambda *_: shutdown_event.set())

    print("[init] loading calibration...")
    H, map1, map2 = load_calibration()

    print(f"[init] loading YOLO weights {YOLO_WEIGHTS_PATH}...")
    if not YOLO_WEIGHTS_PATH.exists():
        raise FileNotFoundError(
            f"{YOLO_WEIGHTS_PATH} not found — train a model and place it here."
        )
    model = YOLO(str(YOLO_WEIGHTS_PATH))

    print("[init] starting camera...")
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.5)  # let AE/AWB settle

    print("[init] starting serial thread...")
    t_serial = threading.Thread(
        target=serial_worker, args=(SERIAL_PORT, SERIAL_BAUD), daemon=True
    )
    t_serial.start()

    fps_t0, fps_n, fps = time.monotonic(), 0, 0.0
    try:
        while not shutdown_event.is_set():
            frame = picam.capture_array()  # BGR-ordered despite "RGB888" name
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            det = detect_with_pose(undistorted, model)
            if det is not None:
                cx, cy, angle, (x1, y1, x2, y2) = det
                wx, wy = pixel_to_world(cx, cy, H)

                # Only act on detections inside the circular workspace
                if wx * wx + wy * wy <= WORKSPACE_RADIUS_MM ** 2:
                    tx, ty = calculate_gantry_offset(wx, wy, angle)
                    enqueue_latest((tx, ty))

                    cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(undistorted, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                    label = f"({wx:+.1f},{wy:+.1f})mm {angle:5.1f}deg"
                    cv2.putText(undistorted, label, (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(undistorted, "OUT OF BOUNDS", (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Pull latest Arduino state for HUD overlay
            with arduino_state_lock:
                rep_pos = arduino_state["reported_pos"]
                last_ack = arduino_state["last_ack_id"]
                last_rx = arduino_state["last_rx_time"]

            # FPS counter
            fps_n += 1
            if fps_n >= 10:
                now = time.monotonic()
                fps = fps_n / (now - fps_t0)
                fps_t0, fps_n = now, 0

            cv2.putText(undistorted, f"{fps:4.1f} FPS", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if rep_pos is not None:
                age_ms = (time.monotonic() - last_rx) * 1000
                color = (0, 255, 255) if age_ms < 500 else (0, 0, 255)
                cv2.putText(undistorted,
                            f"Gantry: ({rep_pos[0]:+6.1f},{rep_pos[1]:+6.1f})mm "
                            f"ack#{last_ack} age{age_ms:4.0f}ms",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            else:
                cv2.putText(undistorted, "Gantry: no data",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            cv2.imshow("Biohybrid Tracker", undistorted)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                shutdown_event.set()
    finally:
        shutdown_event.set()
        t_serial.join(timeout=2.0)
        picam.stop()
        cv2.destroyAllWindows()
        print("[main] clean shutdown")


if __name__ == "__main__":
    main()
