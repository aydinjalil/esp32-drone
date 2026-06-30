# Bluetooth Serial Control — Design Spec

_Date: 2026-06-30_
_Target: `drone_flight_controller/drone_flight_controller.ino` (classic ESP32 DevKit / WROOM-32)_
_Branch: `firmware`_

## Context

The ESP32 is powered by the PDB (soldered 5V from the battery). Connecting USB
on top of that recreates a dual-5V-source contention that drops the USB link
(observed previously). So the powered motor-response test — the first time motors
spin under battery power — cannot rely on USB serial for arming, throttle, and
kill. We need a **wireless control + telemetry channel** so the board runs purely
on battery (no USB) while still being controllable.

The flight controller already has a serial command interface (`processCommands()`:
`a` arm, `d` disarm, `k` kill, `r` reset, `+`/`-` throttle, `0`/space idle) and a
per-loop telemetry line (`Serial.printf(...)`). This feature mirrors both onto
Bluetooth.

## Goals

- Wireless control of the existing commands (`a`/`d`/`k`/`r`/`+`/`-`/`0`/space) from a Mac over Bluetooth.
- Wireless telemetry stream (the existing line) to the Mac.
- A **link-loss failsafe**: if Bluetooth drops while armed, auto-disarm and idle the motors.
- USB serial keeps working in parallel (bench testing unaffected).

## Non-goals (YAGNI)

- BLE / iOS support (Mac client → Bluetooth Classic SPP is sufficient).
- Pairing PIN / encryption hardening.
- Binary telemetry protocol.
- Command-timeout heartbeat failsafe (disconnect-detection only, per decision).
- Altitude-hold / RC / yaw (unrelated features).

## Design

### Transport
- Bluetooth Classic SPP via the Arduino `BluetoothSerial` library.
- Global: `BluetoothSerial SerialBT;`
- `setup()`: `SerialBT.begin("DRONE_FC");` — advertises as `DRONE_FC`, pairs on macOS as `/dev/cu.DRONE_FC` (usable by any serial terminal or the project's Python tools).
- USB `Serial` remains initialized and active.

### Command handling (one path, two sources)
- Extract the body of the current `processCommands()` `if/else if` chain into a helper:
  `void handleCommand(char c)`.
- `processCommands()` then drains **both** input streams through it:
  ```c
  while (Serial.available())   handleCommand(Serial.read());
  while (SerialBT.available()) handleCommand(SerialBT.read());
  ```
- Result: every command behaves identically whether typed over USB or BT. No
  duplicated command logic.

### Telemetry (mirror to both)
- Format the existing telemetry line once into a buffer with `snprintf` (same
  fields/format as today): `h`, `v`, `state`, `thr`, roll/pitch (est/target),
  the four motor values, `tof`.
- `Serial.print(buf);` always.
- `if (SerialBT.hasClient()) SerialBT.print(buf);` — only stream over BT when a
  client is connected (avoids buffering with no listener).

### Link-loss failsafe
- Global `bool btWasConnected = false;`
- Each loop: `bool now = SerialBT.hasClient();`
  - If `btWasConnected && !now && armed`:
    `armed = false; manualThrottle = 0.0f; writeMotorsIdle();`
    and print `"BT LOST - disarmed"` (to USB `Serial`, which is the still-present channel).
  - `btWasConnected = now;`
- This reuses the same disarm+idle actions as the explicit `d`/`k` and the
  existing auto-disarm paths, so behavior is consistent.

## Safety properties

- Motors remain gated behind the unchanged `armed && !killed` interlock.
- Wireless `k` is the primary kill; the disconnect failsafe is the backup.
- All existing safety (pre-arm checks, NaN guards, MAX_ALTITUDE ceiling, IMU-stall
  auto-disarm, throttle-zero on every disarm path) is untouched.
- **Failsafe latency:** `hasClient()` reflects the BT stack's connection state —
  near-instant when the client app closes/disconnects cleanly, but up to a few
  seconds (BT supervision timeout) if the Mac simply goes out of range. Acceptable
  because the powered test is **props OFF** (no thrust, no flyaway).

## Build considerations (flagged, verified during test)

- `BluetoothSerial` links the Bluedroid stack → large flash use. If the sketch
  overflows the default partition, select a larger-app partition scheme in the
  Arduino IDE (Tools → Partition Scheme → e.g. "Huge APP (3MB No OTA)").
- Confirm the main loop still runs ~50 Hz with Bluetooth active (BT adds CPU/radio
  load); check `dt`/telemetry rate during the bench test.
- `BluetoothSerial` requires Bluetooth enabled in the ESP32 Arduino core (default
  on classic ESP32; not available on ESP32-S3 — board is classic ESP32, so OK).

## Verification

**Stage A — USB still works (props OFF, USB power):**
1. Flash; confirm boot, arming, throttle, telemetry all still work over USB exactly as before (regression check).

**Stage B — Bluetooth works (props OFF, USB power):**
2. On the Mac, pair with `DRONE_FC`; open `/dev/cu.DRONE_FC` in a serial terminal (e.g. `screen /dev/cu.DRONE_FC 115200`).
3. Confirm telemetry streams over BT.
4. Send `a` (level+still) → `ARMED`; `+`/`-` move `thr`; `0`/space idles; `k` kills — all over BT.
5. Confirm USB and BT both control the same state simultaneously.

**Stage C — Failsafe (props OFF):**
6. Arm over BT, then close the BT terminal / disconnect. Confirm `armed` drops to false and motors idle (watch via USB telemetry, which shows `state:OFF`).

**Stage D — Powered motor-response (props OFF, BATTERY power, no USB):**
7. Power from battery only; control entirely over BT. Arm, ramp throttle to ~0.2–0.35 (past the ~0.13 deadband), confirm the right motors spin and respond to tilt; `k` stops them. (This is the goal this feature unblocks.)

## Code touch points

- Globals: add `BluetoothSerial SerialBT;`, `bool btWasConnected`.
- Include: `#include "BluetoothSerial.h"`.
- `setup()`: `SerialBT.begin("DRONE_FC");`
- Refactor `processCommands()` → `handleCommand(char c)` + dual-stream drain.
- Telemetry: buffer with `snprintf`, print to `Serial` and (if client) `SerialBT`.
- `loop()`: BT-disconnect failsafe check.
