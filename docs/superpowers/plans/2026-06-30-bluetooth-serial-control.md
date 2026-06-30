# Bluetooth Serial Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Bluetooth Classic (SPP) control + telemetry channel to the flight controller so it can be armed, throttled, and killed wirelessly while running on battery power (no USB), with an auto-disarm failsafe on link loss.

**Architecture:** Mirror the existing USB serial interface onto `BluetoothSerial`. Extract command handling into `handleCommand(char c)` drained from both `Serial` and `SerialBT`; format telemetry once and print to both; a per-loop check disarms if the BT client disconnects while armed.

**Tech Stack:** Arduino C++ (ESP32 Arduino core), `BluetoothSerial` (Bluetooth Classic SPP). Hardware: classic ESP32 DevKit (WROOM-32). No `arduino-cli`/compiler and no unit-test framework here — per-task verification is **structural** (brace/paren balance + grep); behavioral verification is a final **on-hardware bench test**.

## Global Constraints

- Only file modified: `drone_flight_controller/drone_flight_controller.ino`.
- Board: classic ESP32 (WROOM-32) — Bluetooth Classic is available (it is NOT on ESP32-S3; board is classic, so OK).
- Device must advertise as exactly `DRONE_FC`.
- USB `Serial` stays active in parallel — do not remove or disable it.
- Motors stay gated behind `armed && !killed` — do not weaken the interlock.
- Failsafe reuses the existing disarm actions: `armed=false; manualThrottle=0.0f; writeMotorsIdle();`.
- Commit messages: **no `Co-Authored-By` footer, no AI/assistant references anywhere.**
- Work on the `firmware` branch.
- Bench testing is props OFF.
- **Build note for the human flashing:** `BluetoothSerial` links the Bluedroid stack and may overflow the default partition. If the Arduino IDE reports the sketch is too big, set Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)".

---

### Task 1: Bluetooth setup + command input from both USB and BT

**Files:**
- Modify: `drone_flight_controller/drone_flight_controller.ino` (include block ~line 11; globals; `setup()` after `Serial.begin` ~line 436; `processCommands()` lines 325-354)

**Interfaces:**
- Produces: global `BluetoothSerial SerialBT;`, global `bool btWasConnected = false;`, function `void handleCommand(char c)` (existing command logic), and a `processCommands()` that drains both `Serial` and `SerialBT` through `handleCommand`.
- Consumes: existing `armed`, `killed`, `manualThrottle`, `THR_STEP`, `THR_MAX`, `preArmOK()`, `takeoffGroundPressure`, `groundPressure`.

- [ ] **Step 1: Add the include**

After this line (line 11):
```c
#include <ESP32Servo.h>
```
add:
```c
#include "BluetoothSerial.h"
```

- [ ] **Step 2: Add the Bluetooth globals**

Find the sensor-objects block where `Adafruit_ICM20948 imu;` is declared. Immediately after the line `bool zeroed = false;` (in that globals area), add:
```c

// Bluetooth Classic (SPP) control channel — mirrors the USB serial interface.
BluetoothSerial SerialBT;
bool btWasConnected = false;   // for the BT link-loss failsafe
```

- [ ] **Step 3: Start Bluetooth in setup()**

Immediately after these lines in `setup()` (lines 436-437):
```c
  Serial.begin(115200);
  delay(2000);   // do NOT block on Serial — controller must boot untethered
```
add:
```c
  SerialBT.begin("DRONE_FC");   // pairs on macOS as /dev/cu.DRONE_FC
  Serial.println("Bluetooth started: DRONE_FC");
```

- [ ] **Step 4: Refactor processCommands() into handleCommand() + dual-stream drain**

Replace the entire current `processCommands()` (lines 325-354) with:
```c
// Handle one command character from any input stream (USB or Bluetooth).
void handleCommand(char c){
  if (c == 'a'){
    if (preArmOK()){
      takeoffGroundPressure = groundPressure;   // freeze altitude reference
      armed = true;
      manualThrottle = 0.0f;                    // always start at idle
      Serial.println("ARMED");
    } else {
      Serial.println("ARM REFUSED (not level/still, or killed)");
    }
  } else if (c == 'd'){
    armed = false; manualThrottle = 0.0f; Serial.println("DISARMED");
  } else if (c == 'k'){
    killed = true; armed = false; manualThrottle = 0.0f; Serial.println("KILL");
  } else if (c == 'r'){
    if (!armed){ killed = false; Serial.println("KILL RESET"); }
  } else if (c == '+' || c == '='){
    manualThrottle = constrain(manualThrottle + THR_STEP, 0.0f, THR_MAX);
    Serial.printf("THR %.2f\n", manualThrottle);
  } else if (c == '-' || c == '_'){
    manualThrottle = constrain(manualThrottle - THR_STEP, 0.0f, THR_MAX);
    Serial.printf("THR %.2f\n", manualThrottle);
  } else if (c == '0' || c == ' '){
    manualThrottle = 0.0f;
    Serial.println("THR 0 (idle)");
  }
}

void processCommands(){
  while (Serial.available())   handleCommand(Serial.read());
  while (SerialBT.available()) handleCommand(SerialBT.read());
}
```

- [ ] **Step 5: Structural verification (no compiler — this is the test)**

Run:
```bash
cd /Users/aydinjalil/dev/esp32-drone
f=drone_flight_controller/drone_flight_controller.ino
echo "braces: open=$(grep -o '{' $f | wc -l) close=$(grep -o '}' $f | wc -l)"
echo "parens: open=$(grep -o '(' $f | wc -l) close=$(grep -o ')' $f | wc -l)"
grep -nE 'BluetoothSerial.h|BluetoothSerial SerialBT|SerialBT.begin\("DRONE_FC"\)|void handleCommand|while \(SerialBT.available' $f
```
Expected: braces balanced, parens balanced; the include, the `SerialBT` global, `SerialBT.begin("DRONE_FC")`, `void handleCommand(char c)`, and the `SerialBT.available()` drain all present.

- [ ] **Step 6: Commit**

```bash
git add drone_flight_controller/drone_flight_controller.ino
git commit -m "feat(fc): add BluetoothSerial channel + accept commands over USB and BT"
```

---

### Task 2: Mirror telemetry to Bluetooth

**Files:**
- Modify: `drone_flight_controller/drone_flight_controller.ino` (telemetry `Serial.printf` at lines 646-651)

**Interfaces:**
- Consumes: `SerialBT` (from Task 1) and all existing telemetry variables (`h`, `v`, `armed`, `killed`, `safeThrottle`, `roll`, `pitch`, `target_roll`, `target_pitch`, `motorFR/FL/BL/BR`, `tofValid`, `tofAlt`).
- Produces: the same telemetry line printed to `Serial` always and to `SerialBT` when a client is connected.

- [ ] **Step 1: Replace the telemetry printf with a buffered mirror**

Replace these lines (646-651):
```c
  Serial.printf("h:%.2f v:%+.2f state:%s thr:%.3f | r:%.1f/%+.1f p:%.1f/%+.1f | mFR:%.2f mFL:%.2f mBL:%.2f mBR:%.2f | tof:%.0fmm\n",
                h, v, killed?"KILL":(armed?"ARM":"OFF"), safeThrottle,
                roll*180/PI, target_roll,
                pitch*180/PI, target_pitch,
                motorFR, motorFL, motorBL, motorBR,
                tofValid ? tofAlt*1000 : -1);
```
with:
```c
  char tbuf[200];
  snprintf(tbuf, sizeof(tbuf),
           "h:%.2f v:%+.2f state:%s thr:%.3f | r:%.1f/%+.1f p:%.1f/%+.1f | mFR:%.2f mFL:%.2f mBL:%.2f mBR:%.2f | tof:%.0fmm\n",
           h, v, killed?"KILL":(armed?"ARM":"OFF"), safeThrottle,
           roll*180/PI, target_roll,
           pitch*180/PI, target_pitch,
           motorFR, motorFL, motorBL, motorBR,
           tofValid ? tofAlt*1000 : -1);
  Serial.print(tbuf);
  if (SerialBT.hasClient()) SerialBT.print(tbuf);
```

- [ ] **Step 2: Structural verification**

Run:
```bash
cd /Users/aydinjalil/dev/esp32-drone
f=drone_flight_controller/drone_flight_controller.ino
echo "braces: open=$(grep -o '{' $f | wc -l) close=$(grep -o '}' $f | wc -l)"
echo "--- old raw printf telemetry must be GONE: ---"
grep -n 'Serial.printf("h:%.2f' $f || echo "  none (good)"
echo "--- buffered mirror present: ---"
grep -nE 'char tbuf\[200\]|snprintf\(tbuf|SerialBT.hasClient\(\) SerialBT.print\(tbuf\)' $f
```
Expected: braces balanced; the old `Serial.printf("h:...` telemetry is gone; `snprintf(tbuf...`, `Serial.print(tbuf)`, and the `SerialBT.hasClient()` mirror are present. (Note: the `Serial.printf("THR %.2f...` lines in handleCommand are unrelated and may still match a loose grep — the check above targets the `"h:` telemetry specifically.)

- [ ] **Step 3: Commit**

```bash
git add drone_flight_controller/drone_flight_controller.ino
git commit -m "feat(fc): mirror telemetry to Bluetooth when a client is connected"
```

---

### Task 3: Bluetooth link-loss failsafe

**Files:**
- Modify: `drone_flight_controller/drone_flight_controller.ino` (add `checkBtFailsafe()` near `processCommands()`; call it in `loop()` right after `processCommands();` at line 604)

**Interfaces:**
- Consumes: `SerialBT`, `btWasConnected` (Task 1), `armed`, `manualThrottle`, `writeMotorsIdle()`.
- Produces: `void checkBtFailsafe()` and its call site in `loop()`.

- [ ] **Step 1: Add the failsafe function**

Immediately AFTER the closing brace of the `processCommands()` function (the new one from Task 1), add:
```c

// BT link-loss failsafe: if the Bluetooth client disconnects while armed,
// disarm and idle the motors (same actions as 'd'/'k'). Latency = BT
// supervision timeout: ~instant on a clean disconnect, a few seconds on range loss.
void checkBtFailsafe(){
  bool nowConnected = SerialBT.hasClient();
  if (btWasConnected && !nowConnected && armed){
    armed = false; manualThrottle = 0.0f; writeMotorsIdle();
    Serial.println("BT LOST - disarmed");
  }
  btWasConnected = nowConnected;
}
```

- [ ] **Step 2: Call it in loop() after processCommands()**

Find this line in `loop()` (line 604):
```c
  processCommands();
```
and add immediately after it:
```c
  checkBtFailsafe();
```

- [ ] **Step 3: Structural verification**

Run:
```bash
cd /Users/aydinjalil/dev/esp32-drone
f=drone_flight_controller/drone_flight_controller.ino
echo "braces: open=$(grep -o '{' $f | wc -l) close=$(grep -o '}' $f | wc -l)"
echo "parens: open=$(grep -o '(' $f | wc -l) close=$(grep -o ')' $f | wc -l)"
grep -nE 'void checkBtFailsafe|checkBtFailsafe\(\);' $f
echo "--- failsafe uses the standard disarm actions: ---"
grep -nA3 'void checkBtFailsafe' $f | grep -E 'armed = false|manualThrottle = 0.0f|writeMotorsIdle'
```
Expected: braces/parens balanced; `void checkBtFailsafe` defined once and `checkBtFailsafe();` called once (in loop); the function body sets `armed=false`, `manualThrottle=0.0f`, and calls `writeMotorsIdle()`.

- [ ] **Step 4: Commit**

```bash
git add drone_flight_controller/drone_flight_controller.ino
git commit -m "feat(fc): auto-disarm on Bluetooth link loss while armed"
```

---

### Task 4: On-hardware verification (behavioral test gate)

**Files:** none. This is the human bench test. **Props OFF.**

**Build:** In the Arduino IDE, if the sketch overflows on upload, set Tools → Partition Scheme → "Huge APP (3MB No OTA)". Board "ESP32 Dev Module".

- [ ] **Step 1: Stage A — USB regression (USB power)**

Flash; open USB Serial Monitor @ 115200, Newline. Confirm everything still works over USB exactly as before: boot lines, `a` arms (level+still), `+`/`-` move `thr`, `0`/space idles, `k` kills, telemetry streams. Also confirm the new boot line `Bluetooth started: DRONE_FC` appears.

- [ ] **Step 2: Stage B — Bluetooth control (USB power)**

On the Mac: pair with `DRONE_FC` (System Settings → Bluetooth), then open the port, e.g. `screen /dev/cu.DRONE_FC 115200`. Confirm:
- Telemetry streams over BT.
- `a` (level+still) → `state:ARM` in telemetry; `+`/`-` change `thr`; `0`/space → `thr 0`; `k` → `state:KILL`.
- Commands work from BT and USB simultaneously (e.g. arm over BT, kill over USB).

- [ ] **Step 3: Stage C — Link-loss failsafe (USB power)**

Arm over BT (`a`), then disconnect BT (close `screen` with Ctrl-A K, or turn off Mac Bluetooth). Watch the USB telemetry: within a few seconds `state` must drop to `OFF` and motors idle (`mFR..mBR` = 0). Confirm the `BT LOST - disarmed` line printed on USB.

- [ ] **Step 4: Stage D — Powered motor-response (BATTERY power, no USB, props OFF)**

Secure the frame. Power from battery only (no USB). Connect over BT from the Mac. Confirm telemetry over BT. Arm (`a`), ramp `+` to ~0.20-0.35 (past the ~0.13 deadband) → the correct motors spin; tilt the frame → low-side motors spin up, high-side down; `k` stops them. Also test the failsafe live: while a motor is spinning, kill the BT link → motors must stop within a few seconds. (This is the milestone this whole feature unblocks.)

- [ ] **Step 5: Record result**

Note loop rate stayed ~50 Hz with BT active (telemetry cadence unchanged), and any deviations. If all stages pass, the feature is validated.

---

## Notes / Out of Scope (per spec)

- BLE/iOS, pairing PIN/encryption, binary telemetry, command-timeout heartbeat.
- Command acknowledgements ("ARMED", "THR x") currently print to USB `Serial` only; the BT operator sees command effects via the mirrored telemetry stream (`state`/`thr` update within one ~20 ms frame). Routing acks to both channels is a possible later enhancement, intentionally not in this scope.
