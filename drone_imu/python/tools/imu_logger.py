#!/usr/bin/env python3
"""Headless logger for the imu_debug.ino CSV stream.

Captures the full serial stream to a timestamped CSV so you can perform a long
manoeuvre (e.g. a slow roll on a table-edge hinge) and analyse the whole thing
afterwards, instead of being limited to the dozen lines you can copy out of the
Arduino serial monitor.

Usage:
    # close the Arduino IDE serial monitor first (it holds the port), then:
    python imu_logger.py                 # log until Ctrl-C
    python imu_logger.py --seconds 15    # log for a fixed duration
    python imu_logger.py --port /dev/cu.usbserial-0001

Output: drone_imu/data/raw/imu_log_YYYYmmdd_HHMMSS.csv
"""

import argparse
import os
import sys
import time
from datetime import datetime

import serial

DEFAULT_PORT = "/dev/cu.usbserial-0001"
BAUD = 115200

# Matches the imu_debug.ino header so we can label columns and skip noise lines.
HEADER = "time_ms,roll_deg,pitch_deg,yaw_deg,magYaw_deg,mx,my,mz,gz,dt"
N_COLS = len(HEADER.split(","))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after N seconds (default: run until Ctrl-C)")
    args = ap.parse_args()

    out_dir = os.path.join(os.path.dirname(__file__), "..", "..", "data", "raw")
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"imu_log_{stamp}.csv")

    # dtr/rts False so opening the port does not reset the ESP32 mid-run.
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = BAUD
    ser.timeout = 1
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()

    print(f"Logging {args.port} -> {out_path}")
    print("Roll the board now. Press Ctrl-C to stop.\n")

    rows = 0
    stalls = 0
    t0 = time.time()
    with open(out_path, "w") as f:
        f.write(HEADER + "\n")
        try:
            while True:
                if args.seconds is not None and (time.time() - t0) >= args.seconds:
                    break
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("# sensor stall"):
                    stalls += 1
                    print(f"  [stall recovery #{stalls}]")
                    continue
                parts = line.split(",")
                if len(parts) != N_COLS:
                    continue  # header echo, boot text, status lines
                try:
                    float(parts[0])
                except ValueError:
                    continue  # the CSV header line itself
                f.write(line + "\n")
                rows += 1
                if rows % 50 == 0:
                    f.flush()
                    print(f"\r  {rows} samples  ({time.time()-t0:5.1f}s)",
                          end="", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()

    print(f"\n\nSaved {rows} samples to {out_path}")
    if stalls:
        print(f"({stalls} stall-recovery events during capture)")


if __name__ == "__main__":
    sys.exit(main())
