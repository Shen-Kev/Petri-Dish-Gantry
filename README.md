# Petri Dish Gantry

A vision-based tracking system that drives a polar gantry to follow objects on a circular workspace. The gantry is designed to manipulate biohybrid robots — e.g., by positioning an electromagnet beneath a 135 mm petri dish — but the tracking and motion architecture is general purpose.

A Raspberry Pi 5 with a Camera Module 3 looks **up** at the workspace from beneath, identifies the target (a colored marker, or eventually a YOLO-detected biohybrid robot), transforms its pixel position into real-world millimeters, and continuously commands an Arduino Nano ESP32 over UART. The ESP32 drives a polar gantry (stepper for radial extension, servo for rotation) to track the target with a configurable offset.

---

## Table of contents

- [What this project does](#what-this-project-does)
- [System architecture](#system-architecture)
- [Repository layout](#repository-layout)
- [Bill of materials](#bill-of-materials)
- [Complete setup from a blank Raspberry Pi](#complete-setup-from-a-blank-raspberry-pi)
  - [1. Flash the OS](#1-flash-the-os)
  - [2. First boot and OS updates](#2-first-boot-and-os-updates)
  - [3. Install Python environment](#3-install-python-environment)
  - [4. Configure the Pi 5 UART](#4-configure-the-pi-5-uart)
  - [5. Clone this repository](#5-clone-this-repository)
- [Wiring](#wiring)
  - [Camera to Pi](#camera-to-pi)
  - [Pi to Arduino (UART)](#pi-to-arduino-uart)
  - [Gantry to Arduino](#gantry-to-arduino)
- [Arduino firmware](#arduino-firmware)
- [Calibration](#calibration)
  - [Camera intrinsic calibration](#camera-intrinsic-calibration)
  - [Workspace perspective calibration](#workspace-perspective-calibration)
  - [Verifying axis signs](#verifying-axis-signs)
- [Running the tracker](#running-the-tracker)
  - [Smoke-test the serial link](#smoke-test-the-serial-link)
  - [Color-blob tracker (no model required)](#color-blob-tracker-no-model-required)
  - [YOLO tracker (requires trained model)](#yolo-tracker-requires-trained-model)
- [Training a YOLO model for the biohybrid robot](#training-a-yolo-model-for-the-biohybrid-robot)
- [Communication protocol](#communication-protocol)
- [Key configuration constants](#key-configuration-constants)
- [Troubleshooting](#troubleshooting)
- [Design notes](#design-notes)

---

## What this project does

The project solves three problems at once:

1. **Vision.** Detect a small target on a flat workspace despite a wide-angle lens (significant barrel distortion) and an upward-looking camera geometry.
2. **Coordinate transform.** Map detection pixels into real-world millimeters on the gantry's plane, accounting for both lens distortion and camera-to-workspace perspective.
3. **Real-time motion.** Continuously command the gantry to follow the target. The gantry is structurally slow (small stepper, ~9 mm/s arm extension), so the architecture is designed to *never commit to a stale target* — every motion is preemptible by a fresher position update.

The result is a closed-loop tracking system where the gantry tracks a moving target with roughly 50 ms of latency from frame capture to motor reaction.

---

## System architecture

```
 ┌─────────────────────────────────┐         ┌─────────────────────────────┐
 │   Raspberry Pi 5                │  UART   │   Arduino Nano ESP32        │
 │                                 │ 115200  │                             │
 │  ┌─────────────────────────┐    │ ◄─────► │  ┌────────────────────┐     │
 │  │ Picamera2 capture       │    │         │  │ Frame parser       │     │
 │  │  → undistort (intrinsics)│   │         │  │  drop-old/latest   │     │
 │  │  → HSV / YOLO detect    │    │         │  │  workspace clamp   │     │
 │  │  → perspective (mm)     │    │         │  └─────────┬──────────┘     │
 │  │  → EMA smoothing        │    │         │            │                │
 │  │  → offset (lead/follow) │    │         │  ┌─────────▼──────────┐     │
 │  │  → enqueue target       │    │         │  │ Preemptive moveTo  │     │
 │  └────────────┬────────────┘    │         │  │  (servo + stepper) │     │
 │               │ size-1 queue    │         │  └─────────┬──────────┘     │
 │  ┌────────────▼────────────┐    │         │            │                │
 │  │ Serial worker thread    │    │         │  ┌─────────▼──────────┐     │
 │  │  (non-blocking I/O)     │────┼─────────┼──┤ Stepper (D4-D7)    │     │
 │  └─────────────────────────┘    │         │  │ Servo (D3)         │     │
 │                                 │         │  │ Limit switch (D12) │     │
 │  Camera Module 3 (looking up    │         │  └────────────────────┘     │
 │  at the workspace from below)   │         │                             │
 └─────────────────────────────────┘         └─────────────────────────────┘
```

The Pi and ESP32 each run their own loops decoupled by an asynchronous serial channel:

- **Pi side** — vision runs at full camera framerate; serial I/O is on a separate thread so it never stalls capture. A size-1 queue between threads means stale targets are dropped in favor of fresh ones.
- **Arduino side** — a `pending` single-slot target gets overwritten with every new frame. `moveTo()` checks this slot during motion and aborts to honor the freshest target rather than completing a stale path.

This "drop-old, keep-latest" pattern on both sides means the gantry always heads toward the most recent known target position, never a stale one.

---

## Repository layout

```
Petri-Dish-Gantry/
├── README.md                            (this file)
├── arduino/
│   └── tracker_firmware/
│       └── tracker_firmware.ino         polar gantry firmware + Pi link
└── pi/
    ├── camera_calibration.py            intrinsic lens calibration
    ├── workspace_calibration.py         pixel-to-mm perspective calibration
    ├── serial_smoke_test.py             quick Pi↔Arduino link verification
    ├── tracker_simple.py                color-blob tracker (works today)
    └── tracker.py                       YOLO-based tracker (needs trained model)
```

Generated calibration files (`camera_calibration.npz`, `workspace_calibration.npz`) and the trained YOLO model (`biohybrid.pt`) live alongside the Pi scripts but are excluded from version control via `.gitignore`.

---

## Bill of materials

### Compute
- Raspberry Pi 5 (4 GB or 8 GB)
- Official 27 W USB-C power supply (undervoltage will ruin frame rate)
- Active cooler or case with fan (the Pi 5 throttles quickly under sustained CV load)
- microSD card, 32 GB+, A2-rated (or NVMe HAT + SSD for serious work)

### Vision
- Raspberry Pi Camera Module 3 (standard FOV or Wide variant; this project uses Wide)
- **22-to-15 pin MIPI ribbon adapter cable** — the Pi 5 uses the narrow 22-pin connector, but Camera Module 3 ships with a 15-pin cable. Order the adapter separately from the Raspberry Pi store.

### Gantry electronics
- Arduino Nano ESP32 (3.3 V logic, multiple hardware UARTs — important)
- 28BYJ-48 stepper motor with ULN2003 driver board, or equivalent
- MG996R/MS24 270° servo (or equivalent high-torque servo)
- Limit switch (any momentary, with debounce in firmware)
- Power supply for stepper and servo (do NOT power from the Pi's 3.3 V rail)

### Connectivity
- 3 female-to-female jumper wires for Pi ↔ Arduino UART
- USB cable for flashing the ESP32 from a laptop
- 4 fiducial markers (printed crosshairs, ArUco tags, or dots of paint) for workspace calibration
- Printed 9×6 checkerboard, 25 mm squares, on a rigid flat surface

### Mechanical (not in this repo)
- Polar gantry: rotating arm + telescoping/linear extension. The firmware assumes:
  - 12-tooth, 3.016 mm pitch belt or rack on the extension axis
  - Stepper with 2048 steps/rev (full step)
  - Pivot offset of 10 mm in X and 124 mm in Y from the workspace origin
  - 48 mm minimum extension (where the limit switch triggers), 200 mm maximum
- 135 mm-diameter circular workspace (petri dish)

If your mechanical setup differs, edit the constants at the top of `tracker_firmware.ino` (`STEPS_PER_REV`, `TEETH_COUNT`, `TOOTH_PITCH_MM`, `X_OFFSET`, `Y_OFFSET`, `MIN_EXTENSION`, `MAX_EXTENSION`).

---

## Complete setup from a blank Raspberry Pi

This walkthrough assumes nothing is configured. Plan ~2 hours for first-time setup excluding YOLO training.

### 1. Flash the OS

On your laptop, install [Raspberry Pi Imager](https://www.raspberrypi.com/software/). Choose **Raspberry Pi OS (64-bit) Bookworm**, full desktop variant.

Before writing, open the gear icon and pre-configure:
- Hostname
- Username and password
- Wi-Fi credentials
- SSH enabled
- Locale

Write to the SD card.

### 2. First boot and OS updates

Insert SD card, connect the camera ribbon (contacts facing the Ethernet port on the Pi end, blue tab facing away from the board on the camera end), apply power. Log in and run:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

Verify the camera works:

```bash
rpicam-hello --timeout 5000
```

You should see a 5-second preview. If it errors with "no cameras available," the ribbon is reversed or the adapter cable is wrong.

### 3. Install Python environment

```bash
sudo apt install -y python3-picamera2 python3-opencv python3-pip python3-venv git
```

Create a virtual environment that inherits the apt-installed packages (Picamera2 and OpenCV must remain visible via system site-packages because they have C bindings):

```bash
python3 -m venv --system-site-packages ~/tracker-env
source ~/tracker-env/bin/activate
```

Make it auto-activate on shell startup:

```bash
echo 'source ~/tracker-env/bin/activate' >> ~/.bashrc
```

Install pip packages (note: `ultralytics` pulls in PyTorch, ~800 MB, takes ~10 minutes on a Pi 5):

```bash
pip install pyserial numpy ultralytics
```

### 4. Configure the Pi 5 UART

The Pi 5 has two quirks that bite everyone on first setup. Both are addressed here.

**Quirk 1: Bluetooth claims the primary UART by default.**

Edit `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Ensure the following lines are present (add them at the bottom if missing):

```
dtparam=uart0=on
dtoverlay=disable-bt
```

**Quirk 2: `/dev/serial0` does NOT point to the GPIO header UART on Pi 5.**

Unlike Pi 4 and earlier, on the Pi 5 `/dev/serial0` is a symlink to `ttyAMA10`, which is a **separate debug UART on its own JST connector** on the Pi 5 board — not connected to GPIO pins 8/10. The GPIO header UART is always `ttyAMA0`.

**Solution:** All Pi-side code in this repo opens `/dev/ttyAMA0` directly, bypassing the misleading symlink. You don't need to do anything; just be aware.

Also disable the serial login console via `raspi-config`:

```bash
sudo raspi-config
```

Navigate: **Interface Options → Serial Port**
- "Login shell over serial?" → **No**
- "Serial port hardware enabled?" → **Yes**

Reboot:

```bash
sudo reboot
```

After reboot, verify both `ttyAMA0` (GPIO UART) and `ttyAMA10` (debug UART) exist:

```bash
ls -l /dev/ttyAMA*
```

You should see both. Confirm your user is in the `dialout` group:

```bash
groups
```

If `dialout` is missing:

```bash
sudo usermod -aG dialout $USER
```

Then log out and back in.

### 5. Clone this repository

Generate an SSH key on the Pi:

```bash
ssh-keygen -t ed25519 -C "kevin-pi"
cat ~/.ssh/id_ed25519.pub
```

Paste the public key into **GitHub → Settings → SSH and GPG keys → New SSH key**.

Test the connection:

```bash
ssh -T git@github.com
```

Clone:

```bash
cd ~
git clone git@github.com:Shen-Kev/Petri-Dish-Gantry.git
cd Petri-Dish-Gantry
```

---

## Wiring

### Camera to Pi

Camera Module 3 connects to the Pi 5 via the 22-to-15 pin adapter ribbon. Orient:
- **Pi end:** contacts face the Ethernet port side of the board, blue tab faces the USB side.
- **Camera end:** blue tab faces away from the camera PCB, contacts face the PCB.

Use either CAM/DISP port; both work. Bookworm autodetects Camera Module 3 — no `raspi-config` step needed for the camera.

### Pi to Arduino (UART)

Three jumper wires. **Power off both devices before wiring.**

| Pi 5 (40-pin GPIO header) | Direction | Arduino Nano ESP32 |
|---|---|---|
| Pin 8 — GPIO 14 (TXD) | →    | **D0 (RX0)** |
| Pin 10 — GPIO 15 (RXD) | ←    | **D1 (TX0)** |
| Pin 6 — GND | —    | **GND** |

**Critical:** Pi TX connects to ESP32 RX, and Pi RX connects to ESP32 TX. The signals cross over. If you wire TX→TX you'll see no errors but no communication either — the most common first-time mistake.

**Do NOT** connect the Pi's 5 V or 3.3 V rail to the ESP32's power pins. The ESP32 should be powered from its own USB (wall adapter or laptop). Only GND and the two data lines are shared.

**Pin numbering:** hold the Pi with USB ports facing you and the GPIO header on the right. Pin 1 is at the top-left of the header. Odd pins (1, 3, 5, ...) are on the inner row, even pins (2, 4, 6, ...) are on the outer (board-edge) row. Pin 8 is the 4th pin from the top on the outer row; pin 10 is the 5th; pin 6 is the 3rd. Run `pinout` on the Pi for a labeled ASCII diagram.

### Gantry to Arduino

These pins are set in `tracker_firmware.ino`:

| Function | Arduino pin |
|---|---|
| Limit switch | D12 |
| Servo signal | D3 |
| Stepper coil 1 (ULN2003 IN1) | D7 |
| Stepper coil 2 (ULN2003 IN2) | D6 |
| Stepper coil 3 (ULN2003 IN3) | D5 |
| Stepper coil 4 (ULN2003 IN4) | D4 |
| **UART RX (from Pi TX)** | **D0** |
| **UART TX (to Pi RX)** | **D1** |

Stepper and servo are powered from their own supply, sharing GND with the ESP32.

---

## Arduino firmware

The firmware (`arduino/tracker_firmware/tracker_firmware.ino`) implements:

- **Polar kinematics** — Cartesian (x, y) → polar (r, θ) with configurable pivot offset.
- **Homing** — retract until the limit switch triggers; that position is step zero.
- **Eject** — extend half a revolution at startup, then continue until the limit switch releases. This unsticks the arm if it's homed against the switch when the board powers on.
- **Preemptive `moveTo()`** — interpolates a straight line in Cartesian space at 2 mm waypoints. Between waypoints (and inside the inner stepper loop every 32 steps) it checks if a fresher target has arrived from the Pi. If so, it returns immediately with `current.x/y` reflecting the last reached waypoint. The next loop iteration starts a new move from that position toward the new target.
- **Bidirectional framed serial** on `Serial0` (D0/D1) at 115200 baud, with a 2 KB RX buffer to absorb commands sent during long moves.
- **Workspace clamp** — rejects targets outside the 67.5 mm radius (with 10% slack) as a defense against garbled frames.
- **Manual fallback** — type `x,y` in the USB Serial Monitor for manual command testing. Non-blocking; coexists with the Pi link.

### Flashing

Use Arduino IDE 2.x on your laptop (the ESP32 toolchain is too heavy to install comfortably on the Pi).

1. **File → Preferences → Additional Boards Manager URLs**, add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager**, install **"esp32 by Espressif Systems"** (~300 MB).
3. **Tools → Board → esp32 → Arduino Nano ESP32**.
4. Open `arduino/tracker_firmware/tracker_firmware.ino`.
5. Connect the ESP32 via USB. **Tools → Port** → select the new port.
6. Click **Upload**.
7. Open **Tools → Serial Monitor** at 115200 baud. You should see the boot sequence:
   ```
   --- GANTRY ONLINE ---
   Ejecting arm...
   Homing...
   Homing Complete.
   Moving to (0,0)...
   Entering Loop...
   ```

Test a manual move: type `40,0` in Serial Monitor and press Enter. The gantry should physically move to (40, 0) mm. If yes, the firmware is healthy.

---

## Calibration

Both calibrations are one-time procedures. Re-run them if the camera physically moves, the focus changes, or the workspace is repositioned relative to the gantry.

### Camera intrinsic calibration

Corrects for the wide-angle lens's barrel distortion using a printed checkerboard.

**Prep:**
- Print a 9×6 *interior corner* checkerboard with 25 mm squares (i.e., a 10×7 squares grid). Search "OpenCV 9x6 checkerboard PDF" or generate at [calib.io](https://calib.io/pages/camera-calibration-pattern-generator).
- Glue it to a rigid flat surface (clipboard, acrylic, foamboard).

**Run:**
```bash
cd ~/Petri-Dish-Gantry/pi
python camera_calibration.py
```

Hold the checkerboard in front of the camera. When all corners are detected, green corner markers appear overlaid on the board.

**Click the OpenCV preview window first to give it focus**, then press SPACE to capture. Common issue: if SPACE does nothing, the terminal still has focus.

Capture at least 15 frames with the board at varied positions, angles, and distances. Once done, press `c` to compute.

You're looking for an **RMS reprojection error under 1.0 pixel**. Under 0.5 is excellent. If your error is above 1.5, your captures weren't diverse enough — delete `camera_calibration.npz` and recapture with more varied poses.

Output: `camera_calibration.npz` containing `K` (intrinsics) and `dist` (distortion coefficients).

### Workspace perspective calibration

Maps undistorted pixels to real-world millimeters via a 4-point homography.

**Prep:**
- Place 4 fiducial markers on your workspace at known positions:
  - **A:** (+67.5, 0)   — +X axis
  - **B:** (0, +67.5)   — +Y axis
  - **C:** (-67.5, 0)   — -X axis
  - **D:** (0, -67.5)   — -Y axis

Use whatever direction you call "+X" on the gantry. Crosshairs, dots, or ArUco tags all work.

**Run:**
```bash
python workspace_calibration.py
```

In the undistorted preview, click the fiducials **in the order A, B, C, D**. Press `s` to save. The script prints a sanity check showing each clicked pixel mapping back to the expected mm coordinate.

Output: `workspace_calibration.npz` containing the homography matrix `H` plus the camera intrinsics for convenience.

### Verifying axis signs

The camera looks **up** at the workspace, which produces a mirror image compared to a top-down view. It's easy to end up with one axis inverted. **Always verify before trusting the calibration:**

1. Place a marker at the center of the workspace.
2. With the gantry powered and the Pi-Arduino link wired, send a known command:
   ```bash
   python3 -c "import serial,time; s=serial.Serial('/dev/ttyAMA0',115200); time.sleep(0.3); s.write(b'<20.0,0.0>\n')"
   ```
3. Run `tracker_simple.py`, click a marker placed at the gantry's actual position, and confirm the on-screen world coordinate updates in the same direction the gantry physically moved.

If the X (or Y) sign is reversed, redo `workspace_calibration.py` and click the opposite-paired fiducial when prompted for that axis (e.g., click at -X first if the calibration originally clicked at +X).

---

## Running the tracker

### Smoke-test the serial link

Before running any vision code, prove the Pi can talk to the ESP32:

```bash
cd ~/Petri-Dish-Gantry/pi
python serial_smoke_test.py
```

Expected output:
```
[test] opening /dev/ttyAMA0 @ 115200
[test] sending b'<5.00,10.00>\n'
[rx] POS:0.00,0.00
[rx] POS:5.00,10.00
[rx] ACK:1
[test] OK — got ACK within timeout. Link is working.
```

The gantry should physically move to (5, 10) mm.

**If the smoke test fails:** 95% of the time it's TX/RX swapped. Swap the two data wires on one end only (not GND) and retry. If still broken, plug the ESP32 back into your laptop USB and open Serial Monitor — confirm the firmware boot sequence prints normally. That isolates whether the firmware is healthy and the issue is just the GPIO UART link.

### Color-blob tracker (no model required)

Use this as the main tracker until you have a trained YOLO model. It tracks a brightly colored marker via HSV color masking and is the easiest way to validate the full pipeline.

```bash
python tracker_simple.py
```

**Workflow:**
1. The OpenCV window opens with overlay text "Click on the colored marker to begin."
2. **Click the OpenCV window** to give it focus.
3. Click the colored marker in the live feed. The script samples a 5×5 HSV patch around the click and starts tracking pixels of that color.
4. A red crosshair appears on the tracked blob. Its world coordinates appear in the HUD. The gantry starts following.

**Hotkeys:**

| Key | Action |
|---|---|
| Left click | (Re)sample color at clicked pixel |
| `m` | Toggle mask overlay (essential for tuning) |
| `[` / `]` | Shrink / grow HSV color tolerance |
| `,` / `.` | Focus farther / closer in 0.25 dioptre steps (manual mode) |
| `a` | Toggle autofocus |
| SPACE | Pause / resume sending commands to gantry (vision continues) |
| `q` | Quit cleanly |
| Window X button | Quit cleanly |

**Editable constants at the top of `tracker_simple.py`:**

```python
OFFSET_X_MM = 0.0         # Lead/follow offset applied to target before send
OFFSET_Y_MM = 0.0

USE_AUTOFOCUS = False     # True = continuous autofocus
MANUAL_FOCUS_DIOPTRES = 5.0   # 0=infinity, larger=closer (5.0 ~ 20cm)

HSV_TOL_H = 10            # Tighter = pickier; tune with [/] hotkeys
HSV_TOL_S = 60
HSV_TOL_V = 60

WORLD_SMOOTHING_ALPHA = 0.5   # EMA smoothing on world coords; 1.0 disables
```

**Tuning the color mask:**

If "no blob" appears in the HUD when you have a clear marker:
- Press `m` to see exactly which pixels are matching (green overlay).
- Press `]` a few times to broaden tolerance, watching the mask widen.
- If the mask is bleeding into background, press `[` to tighten.

If detection is jittery:
- The default `WORLD_SMOOTHING_ALPHA = 0.5` already smooths centroid noise. Drop to 0.3 for more smoothing (more lag), raise to 0.7 for more responsiveness (more jitter).

**Tuning the focus:**

The Camera Module 3 has a motorized lens controlled in dioptres: 0.0 = infinity, larger = closer. Press `.` repeatedly to move toward macro (closer focus); press `,` to move back toward infinity. The HUD shows the current value. For a typical 20–30 cm workspace distance, 4–6 dioptres is a good range.

### YOLO tracker (requires trained model)

`tracker.py` is the production tracker. It does everything `tracker_simple.py` does but replaces HSV color detection with a YOLO model trained on your specific biohybrid robot. After YOLO localizes the robot, classical CV (Otsu threshold + `minAreaRect`) refines the pose (rotation angle) inside the YOLO bounding box.

To use it, you need a trained YOLO model file named `biohybrid.pt` in the `pi/` directory. See [Training a YOLO model](#training-a-yolo-model-for-the-biohybrid-robot) below.

```bash
python tracker.py
```

Output behavior is similar to `tracker_simple.py` but with YOLO detection instead of color masking, and pose (rotation) extraction added to the position.

---

## Training a YOLO model for the biohybrid robot

YOLO training is its own workstream, typically a 2–4 hour effort spread across data collection, labeling, and training. **Do not train on the Pi** — it will take hours and thermal-throttle.

### 1. Capture a dataset on the Pi

Roughly 300 images is a reasonable starting point. Vary lighting, robot position across the workspace, background clutter, and orientation. Quick capture snippet:

```python
# pi/capture_dataset.py
from picamera2 import Picamera2
import cv2, time, pathlib
out = pathlib.Path("dataset_raw"); out.mkdir(exist_ok=True)
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (1280, 720), "format": "RGB888"}))
cam.start(); time.sleep(0.5)
i = 0
while True:
    f = cam.capture_array()
    cv2.imshow("capture", f)
    k = cv2.waitKey(1) & 0xFF
    if k == ord(' '):
        cv2.imwrite(str(out / f"img_{i:04d}.jpg"), f); i += 1; print(i)
    elif k == ord('q'):
        break
```

### 2. Label the images

Easiest: [Roboflow](https://roboflow.com)'s free tier. Upload images, draw bounding boxes around the robot, export as "YOLOv8" format — this produces the directory structure YOLO expects, automatically.

Alternatives: [LabelImg](https://github.com/HumanSignal/labelImg) or CVAT for local labeling.

Expected layout:
```
dataset/
├── images/train/*.jpg
├── images/val/*.jpg
├── labels/train/*.txt       # one .txt per image, same basename
├── labels/val/*.txt
└── data.yaml
```

`data.yaml`:
```yaml
path: /absolute/path/to/dataset
train: images/train
val:   images/val
names:
  0: biohybrid
```

80/20 train/val split is fine. For 300 images that's 240 train / 60 val.

### 3. Train

On a laptop with a GPU, or in a free [Google Colab](https://colab.research.google.com) notebook (which has free T4 GPUs):

```bash
pip install ultralytics
yolo detect train \
    model=yolov8n.pt \
    data=/path/to/data.yaml \
    epochs=100 \
    imgsz=640 \
    batch=16 \
    patience=20
```

`yolov8n.pt` is auto-downloaded on first run (this is the COCO-pretrained nano model used as a starting point for transfer learning). Output goes to `runs/detect/train/weights/best.pt`.

On a T4 GPU, 300 images train in 10–20 minutes.

### 4. Verify

```bash
yolo detect predict model=runs/detect/train/weights/best.pt source=dataset/images/val conf=0.5
```

Look at the annotated predictions in `runs/detect/predict/`. Tight bounding boxes with 0.9+ confidence = ready to deploy. If detections are sloppy, collect more data or train longer.

### 5. Deploy

Copy `best.pt` to the Pi, rename to `biohybrid.pt`, place in `pi/`:

```bash
scp runs/detect/train/weights/best.pt kevin@petridishgantry:~/Petri-Dish-Gantry/pi/biohybrid.pt
```

Then run `python tracker.py` on the Pi.

**Performance expectations:** YOLOv8n at 640×640 on a Pi 5 CPU runs roughly 4–8 FPS. If that's too slow:
- Drop `YOLO_IMG_SIZE` to 320 in `tracker.py` — usually doubles FPS with minor accuracy loss.
- Export to NCNN format (`yolo export model=biohybrid.pt format=ncnn`) — ~2× speedup on ARM.
- For real speed, add a Hailo-8L HAT (~$70). With NPU offload, nano-class models hit 60+ FPS.

---

## Communication protocol

ASCII, framed, line-terminated. All messages on `Serial0` (Pi ↔ ESP32) at 115200 baud, 8N1.

### Pi → ESP32

| Frame | Meaning |
|---|---|
| `<X,Y>\n` | Target work coordinates in mm. X and Y as decimal floats, e.g. `<12.34,-5.67>\n` |

### ESP32 → Pi

| Frame | Meaning |
|---|---|
| `<ACK:N>\n` | Acknowledged completion (or preemption) of command N. N increments per command. |
| `<POS:X,Y>\n` | Current gantry position. Sent every 50 ms during idle, ~every 50 ms during motion. |
| `<ERR:msg>\n` | Firmware-side error. Known messages: `boot`, `bad_frame_no_comma`, `bad_frame_parse`, `out_of_bounds`, `frame_too_long`. |

### Parser semantics

Both sides use a byte-level state machine tolerant of:
- Garbage outside frames (ignored).
- Partial frames spanning loop iterations (buffered).
- Overlong frames (reset, error reported).
- Misordered bytes within frames (parsed strictly; bad payloads rejected with `ERR`).

Both sides apply "drop-old, keep-latest" semantics:
- Pi has a `queue.Queue(maxsize=1)` between vision and serial threads.
- ESP32 has a `pending` single-slot target slot.

This means a momentary slowdown anywhere in the pipeline causes stale frames to be silently dropped rather than buffered. The latest target always wins.

---

## Key configuration constants

### `pi/tracker_simple.py`

| Constant | Default | Meaning |
|---|---|---|
| `OFFSET_X_MM`, `OFFSET_Y_MM` | `0.0, 0.0` | Offset added to detected position before sending to gantry |
| `USE_AUTOFOCUS` | `False` | True = continuous AF; False = fixed manual focus |
| `MANUAL_FOCUS_DIOPTRES` | `5.0` | Initial manual focus (0=infinity, larger=closer) |
| `HSV_TOL_H/S/V` | `10, 60, 60` | Color mask tolerance (tune live with `[`/`]`) |
| `MIN_BLOB_AREA_PX` | `50` | Smaller blobs ignored as noise |
| `WORLD_SMOOTHING_ALPHA` | `0.5` | EMA on world coords; 1.0 disables |
| `FRAME_WIDTH/HEIGHT` | `1280, 720` | Camera resolution |
| `SERIAL_PORT` | `/dev/ttyAMA0` | Pi GPIO UART |
| `WORKSPACE_DIAMETER_MM` | `135.0` | Reachable circle |

### `arduino/tracker_firmware/tracker_firmware.ino`

| Constant | Default | Meaning |
|---|---|---|
| `STEPS_PER_REV` | `2048` | Stepper steps per full revolution |
| `TEETH_COUNT` | `12` | Belt/gear teeth on extension axis |
| `TOOTH_PITCH_MM` | `3.016` | Belt/gear tooth pitch |
| `X_OFFSET`, `Y_OFFSET` | `10.0, 124.0` | Pivot → work origin offsets, mm |
| `MIN_EXTENSION`, `MAX_EXTENSION` | `48.0, 200.0` | Arm extension limits, mm |
| `SERVO_ANGLE_OFFSET`, `SERVO_MOUNT_TRIM` | `45.0, 3.0` | Servo zero-point trimming, degrees |
| `STEP_DELAY_MS` | `2` | Per-step delay (smaller = faster, risks stalling) |
| `PATH_RESOLUTION` | `2.0` | mm per interpolation waypoint |
| `PREEMPT_CHECK_EVERY_N_STEPS` | `32` | Preemption check granularity inside step loop |
| `PI_WORKSPACE_RADIUS` | `67.5` | Safety clamp radius, mm |
| `POS_REPORT_PERIOD_MS` | `50` | Position report rate to Pi |

---

## Troubleshooting

### Pi UART issues

**`/dev/ttyAMA0` does not exist after reboot.**
Check `/boot/firmware/config.txt` contains `dtparam=uart0=on`. Confirm no `console=serial0` or `console=ttyAMA*` entries in `/boot/firmware/cmdline.txt`. Reboot.

**`PermissionError: [Errno 13] Permission denied: '/dev/ttyAMA0'`**
Your user isn't in the `dialout` group. Run `sudo usermod -aG dialout $USER`, log out, log back in.

**Serial reads return `b''` even with a loopback wire on pins 8 and 10.**
On Pi 5, this usually means Bluetooth is still claiming the UART (`hci_uart_bcm` shows in `dmesg`). Add `dtoverlay=disable-bt` to `/boot/firmware/config.txt` and reboot.

### Camera issues

**Loopback test fails on Camera Module 3.**
Confirm `rpicam-hello --timeout 5000` shows a preview. If not, the ribbon is reversed (very common) or you forgot the 22-to-15 pin adapter (Pi 5 has the narrow 22-pin connector; Camera Module 3 ships with a 15-pin cable).

**Image is out of focus during tracking.**
Press `,` and `.` in `tracker_simple.py` to manually adjust focus. The Pi Camera Module 3 has a motorized lens controllable in dioptres (0 = infinity, larger = closer). The HUD displays the current value. Typical workspace distances of 15–30 cm fall in the 3–7 dioptre range.

**Camera calibration's RMS error is high (>1.5 px).**
Capture more diverse poses — extreme angles, board near image edges, varied distances. Make sure the entire checkerboard is in frame for each capture.

### Serial / firmware issues

**Smoke test fails with timeout.**
Check, in order: (1) TX/RX wires swapped — try swapping them on one end; (2) ESP32 powered and the firmware boot banner visible via USB Serial Monitor; (3) GND wire firmly connected.

**ESP32 sends `<ERR:out_of_bounds>` for every command.**
The Pi is computing world coordinates outside the 67.5 mm radius. Your workspace calibration is likely scaled wrong — re-run `workspace_calibration.py` and double-check the fiducial positions.

**Gantry moves in the wrong direction.**
Axis is inverted from the upward-looking camera view. Re-run `workspace_calibration.py` and click the opposite fiducial when prompted for that axis.

**Gantry hunts / chatters on a stationary target.**
Tune `WORLD_SMOOTHING_ALPHA` in `tracker_simple.py` lower (e.g., 0.3) for more smoothing. If still jittery, your color mask may be drifting frame-to-frame — press `m` to inspect the mask and tighten tolerance with `[`.

### Quitting cleanly

The OpenCV window respects both the `q` keypress (with window focused) and the OS X button. If Ctrl+C in the terminal leaves the window stuck:

```bash
pkill -9 -f tracker_simple.py
```

### Other

**Window is stuck after Ctrl+C.**
Use `q` or the X button instead — those go through the clean shutdown path. The code includes a GUI event-pump after `destroyAllWindows()` to flush window destruction events, but Ctrl+C can still race against it.

---

## Design notes

**Why "drop-old, keep-latest" on both ends?**
The gantry is mechanically slower than the vision pipeline. The vision pipeline can produce 30+ position updates per second; the gantry can typically execute 1–5 full moves per second. Without aggressive dropping, the Arduino's UART buffer would fill with stale targets and the gantry would lag seconds behind reality. With drop-old semantics, latency stays bounded at one preemption interval (~50 ms) regardless of how slowly the gantry moves.

**Why preemption inside `moveTo()` instead of returning to `loop()` between commands?**
Each `moveTo()` can take several seconds for a long move. Without preemption, the gantry commits to that destination for the full move duration. With preemption, the gantry can reverse direction mid-move when the target jumps — essential for tracking a moving object.

**Why is the camera looking *up* at the workspace?**
The original use case is a transparent petri dish on a stage, with an electromagnet beneath. The camera goes under the dish so the gantry mechanism doesn't occlude the view. This is unusual enough to be worth flagging — it inverts one axis compared to a conventional top-down view, which the workspace calibration step accounts for via the homography.

**Why HSV for the simple tracker instead of just RGB?**
HSV separates hue (color identity) from brightness, making the mask robust to lighting changes. A red blob looks "red" under both bright and dim light, but its RGB values shift dramatically. HSV is what nearly every classical color-based tracker uses.

**Why doesn't the gantry side use real encoder feedback?**
The stepper is run open-loop. The firmware tracks "commanded steps" rather than "actual steps reached" because the small stepper used here doesn't have an encoder. The `<POS:X,Y>` reports back to the Pi reflect the commanded position; mechanical slip would not be detected. For tracking applications where small slip is tolerable, this is fine. For a more rigorous setup, add an encoder and report true position.

**Why is the Pi 5 UART setup so fiddly?**
The Pi 5 introduced a new SoC (BCM2712) with multiple PL011 UARTs and a new on-board debug connector that re-uses the historically-conventional `/dev/serial0` name. Most online documentation predates Pi 5 and assumes the Pi 4 behavior. The setup in this README is what actually works on Pi 5 / Bookworm as of 2024.
