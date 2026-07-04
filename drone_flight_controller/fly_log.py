#!/usr/bin/env python3
"""Interactive flight console + black-box logger for drone_flight_controller.

Replaces `screen /dev/cu.DRONE_FC` for tether/tuning sessions: every keypress
is sent to the FC immediately (raw mode, no Enter), incoming telemetry is
shown live, and BOTH directions are written to a timestamped log so the
session can be analyzed afterwards (command -> drone-behavior correlation).

Usage:
    python fly_log.py [--port /dev/cu.DRONE_FC]

Log lines (flight_logs/flight_YYYYmmdd_HHMMSS.log):
    <t_seconds> TX <key>
    <t_seconds> RX <line from FC>

Quit with Ctrl-C or Ctrl-].
SAFETY: quitting closes the BT link — if the drone is ARMED, the FC's
link-loss failsafe will DISARM it (motors cut). Land and disarm first.
"""

import argparse
import datetime
import os
import select
import sys
import termios
import time
import tty

import serial

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--port", default="/dev/cu.DRONE_FC",
                help="serial port (BT: /dev/cu.DRONE_FC, USB: /dev/cu.usbserial-0001)")
args = ap.parse_args()

logdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_logs")
os.makedirs(logdir, exist_ok=True)
logpath = os.path.join(
    logdir, datetime.datetime.now().strftime("flight_%Y%m%d_%H%M%S.log"))

ser = serial.Serial(args.port, 115200, timeout=0)
log = open(logpath, "w")
t0 = time.monotonic()


def stamp(direction, payload):
    log.write(f"{time.monotonic() - t0:.3f} {direction} {payload}\n")
    log.flush()


print(f"Connected to {args.port}")
print(f"Logging to  {logpath}")
print("Keys go straight to the FC (a=arm  +/-=thr  0/space=idle  d=disarm  "
      "k=KILL  r=reset  p/P i/I x/X=gains  g=gains)")
print("Quit: Ctrl-C or Ctrl-]  (quitting while ARMED trips the link-loss "
      "disarm — land first!)\n")

fd = sys.stdin.fileno()
old_attrs = termios.tcgetattr(fd)
rxbuf = b""

try:
    tty.setcbreak(fd)
    while True:
        readable, _, _ = select.select([sys.stdin, ser], [], [], 0.05)

        if sys.stdin in readable:
            key = os.read(fd, 1)
            if key in (b"\x03", b"\x1d"):        # Ctrl-C / Ctrl-]
                break
            ser.write(key)
            shown = key.decode("ascii", "replace")
            stamp("TX", repr(shown))

        if ser in readable:
            rxbuf += ser.read(4096)
            while b"\n" in rxbuf:
                line, rxbuf = rxbuf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").rstrip("\r")
                if text:
                    print(text)
                    stamp("RX", text)
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
    log.close()
    ser.close()
    print(f"\nSession saved: {logpath}")
