"""
Smoke test for the Pi 5 <-> Arduino Nano ESP32 serial link.

Run this BEFORE running tracker.py to confirm:
  - /dev/ttyAMA0 is open-able (UART configuration is correct).
  - The ESP32 firmware is running and responds to framed commands.
  - Bidirectional framing works in both directions.

Prereq: flash tracker_firmware.ino to the ESP32, wire 3 jumpers
(Pi pin 8 -> D0, Pi pin 10 <- D1, Pi pin 6 -- GND), power the
ESP32 from USB.

Expected output:
  [test] opening /dev/ttyAMA0 @ 115200
  [test] sending <5.00,10.00>
  [rx] POS:0.00,0.00
  [rx] ACK:1
  [rx] POS:5.00,10.00
  [test] OK — got ACK:1 within timeout
"""

import serial
import sys
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200
TEST_X, TEST_Y = 5.0, 10.0
ACK_TIMEOUT_S  = 2.0


def main():
    print(f"[test] opening {PORT} @ {BAUD}")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)
    except serial.SerialException as e:
        print(f"[test] FAIL: could not open port: {e}")
        sys.exit(1)

    # Let the UART settle and drain any leftover bytes
    time.sleep(0.3)
    ser.reset_input_buffer()

    cmd = f"<{TEST_X:.2f},{TEST_Y:.2f}>\n".encode("ascii")
    print(f"[test] sending {cmd!r}")
    ser.write(cmd)

    got_ack = False
    rx_buf  = bytearray()
    deadline = time.monotonic() + ACK_TIMEOUT_S

    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            rx_buf.extend(chunk)
            while b"\n" in rx_buf:
                line, _, rest = rx_buf.partition(b"\n")
                rx_buf = bytearray(rest)
                s = line.decode("ascii", errors="replace").strip()
                if s.startswith("<") and s.endswith(">"):
                    payload = s[1:-1]
                    print(f"[rx] {payload}")
                    if payload.startswith("ACK:"):
                        got_ack = True
                        # Keep reading briefly so you can see the post-move POS
                elif s:
                    print(f"[rx-raw] {s!r}")
        time.sleep(0.01)

    ser.close()

    if got_ack:
        print("[test] OK — got ACK within timeout. Link is working.")
        sys.exit(0)
    else:
        print("[test] FAIL: no ACK received within 2s.")
        print("       Check:  (1) ESP32 powered on and firmware flashed")
        print("               (2) TX/RX wires not swapped")
        print("               (3) GND shared between Pi and ESP32")
        print("               (4) /boot/firmware/config.txt has:")
        print("                       dtparam=uart0=on")
        print("                       dtoverlay=disable-bt")
        sys.exit(2)


if __name__ == "__main__":
    main()
