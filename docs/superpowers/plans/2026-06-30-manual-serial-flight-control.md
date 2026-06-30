# Manual Serial Flight Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the simulated bench throttle/attitude in the flight controller with safe, incremental serial manual-throttle control while the attitude PID self-levels.

**Architecture:** Add a bounded `manualThrottle` state driven by serial keys in `processCommands()`; feed it into the existing `computeSafeThrottle()` → `motorMixing()` path in `loop()`, with roll/pitch targets pinned to 0. No new files — all changes are in the single firmware sketch.

**Tech Stack:** Arduino C++ (ESP32 Arduino core), `ESP32Servo`. Hardware: classic ESP32 DevKit. No unit-test framework and no `arduino-cli` in this environment — verification is (a) **structural** (grep + brace/paren balance) per code task, and (b) a final **on-hardware bench test** over serial.

## Global Constraints

- Board: **classic ESP32 DevKit (WROOM-32)**; IMU at I²C **0x69**; baro 0x47; ToF absent (baro-only mode via `tofPresent` guard).
- Motors must stay gated behind `armed && !killed` — do **not** weaken that interlock.
- All bench testing is **props OFF, USB power only** (ESCs unpowered; validate command math, not physical spin).
- Commit messages: **no `Co-Authored-By` footer, no AI/assistant references anywhere.**
- Work happens on the **`firmware`** branch.
- Only file modified: `drone_flight_controller/drone_flight_controller.ino`.

---

### Task 1: Manual-throttle state + serial command handling

**Files:**
- Modify: `drone_flight_controller/drone_flight_controller.ino` (globals near line 128; `processCommands()` at lines 319-338)

**Interfaces:**
- Produces: global `float manualThrottle` (range `0 .. THR_MAX`); constants `THR_STEP = 0.02f`, `THR_MAX = 0.75f`. Serial keys `+`/`=` raise, `-`/`_` lower, `0`/space zero it. `a`/`d`/`k` reset it to 0.
- Consumes: existing `armed`, `killed`, `preArmOK()`, `takeoffGroundPressure`, `groundPressure`.

- [ ] **Step 1: Add the manual-throttle globals + constants**

After this existing line (line 128):
```c
float throttle = 0.0f, target_roll = 0.0f, target_pitch = 0.0f;
```
add:
```c

// Manual serial throttle (pilot-driven; no RC yet). Incremental + bounded so no
// single command (or a bad sensor zero) can cause a large throttle jump.
float manualThrottle = 0.0f;     // 0 .. THR_MAX
const float THR_STEP = 0.02f;    // 2% per keypress
const float THR_MAX  = 0.75f;    // early-test ceiling; raise after validation
```

- [ ] **Step 2: Add throttle keys + zero-on-arm/disarm/kill to `processCommands()`**

Replace the entire body of `processCommands()` (lines 319-338) with:
```c
void processCommands(){
  while (Serial.available()){
    char c = Serial.read();
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
}
```

- [ ] **Step 3: Structural verification (no compiler available — this is the test)**

Run:
```bash
cd /Users/aydinjalil/dev/esp32-drone
f=drone_flight_controller/drone_flight_controller.ino
echo "braces: open=$(grep -o '{' $f | wc -l) close=$(grep -o '}' $f | wc -l)"
echo "parens: open=$(grep -o '(' $f | wc -l) close=$(grep -o ')' $f | wc -l)"
grep -nE "manualThrottle|THR_STEP|THR_MAX" $f
```
Expected: brace open == close, paren open == close; `manualThrottle` appears in the globals and in the `+`/`-`/`0`/`a`/`d`/`k` handlers; `THR_STEP`/`THR_MAX` defined once and used in the clamps.

- [ ] **Step 4: Commit**

```bash
git add drone_flight_controller/drone_flight_controller.ino
git commit -m "feat(fc): add bounded manual serial throttle state + commands"
```

---

### Task 2: Wire manual throttle into the control loop (replace simulated inputs)

**Files:**
- Modify: `drone_flight_controller/drone_flight_controller.ino` (loop pilot-input lines 601-604)

**Interfaces:**
- Consumes: `manualThrottle` (from Task 1).
- Produces: `throttle = manualThrottle`, `target_roll = target_pitch = 0` feeding the existing `computeSafeThrottle()` → `computeAttitudePID()` → `motorMixing()` chain (unchanged).

- [ ] **Step 1: Replace the simulated pilot inputs**

Replace these four lines (601-604):
```c
  // Pilot Inputs (Simulated — replace with RC + failsafe)
  throttle = 0.55f + 0.1f * sin(now * 0.001f);    // Hover + climb
  target_roll = 5.0f * sin(now* 0.002f);          // ±5° roll command
  target_pitch = 5.0f * cos(now * 0.0015f);       // ±5° pitch command
```
with:
```c
  // Pilot inputs: manual serial throttle, self-level attitude (no RC yet)
  throttle     = manualThrottle;
  target_roll  = 0.0f;
  target_pitch = 0.0f;
```

- [ ] **Step 2: Structural verification**

Run:
```bash
cd /Users/aydinjalil/dev/esp32-drone
f=drone_flight_controller/drone_flight_controller.ino
echo "braces: open=$(grep -o '{' $f | wc -l) close=$(grep -o '}' $f | wc -l)"
echo "--- simulated inputs must be GONE: ---"
grep -nE "0.55f \+ 0.1f|5.0f \* sin|5.0f \* cos" $f || echo "  none (good)"
echo "--- manual wiring present: ---"
grep -nE "throttle *= *manualThrottle|target_roll *= *0.0f|target_pitch *= *0.0f" $f
```
Expected: braces balanced; no simulated-input lines remain; `throttle = manualThrottle` and zeroed targets present.

- [ ] **Step 3: Commit**

```bash
git add drone_flight_controller/drone_flight_controller.ino
git commit -m "feat(fc): drive throttle from manual serial input, self-level attitude"
```

---

### Task 3: On-hardware bench verification (behavioral test gate)

**Files:** none (verification only). This is the real behavioral test — the structural checks above only prove the code is well-formed.

**Setup:** Props OFF. USB power only (battery/PDB disconnected). Flash `drone_flight_controller.ino` from the Arduino IDE (board "ESP32 Dev Module", port `/dev/cu.usbserial-0001`). Open Serial Monitor @ 115200, line ending **Newline**.

- [ ] **Step 1: Boot check**

Expected on boot: `Gravity cal: ~9.x`, `Ground zero complete! ~1006 hPa`, `ToF absent — altitude from baro only`, no `IMU FAIL`, then the telemetry stream with `state:OFF thr:0.000` and `mFR/mFL/mBL/mBR: 0.00`.

- [ ] **Step 2: Arm starts at idle**

Hold the board level + still, send `a`. Expected: `ARMED`, then telemetry shows `state:ARM` with `thr:0.000` (NOT a carried-over value) and motor values near 0.

- [ ] **Step 3: Throttle ramps incrementally**

Send `+` several times. Expected: each press prints `THR 0.02`, `0.04`, … and telemetry `thr` rises in 0.02 steps; the four motor values rise together and stay balanced while the board is level.

- [ ] **Step 4: Self-level under tilt**

While throttle is up, tilt the board. Expected: motor values redistribute (front/back for pitch, left/right for roll) opposing the tilt; targets stay 0 (drone tries to return to level).

- [ ] **Step 5: Lower, quick-idle, and clamp**

Send `-` (thr drops in 0.02 steps). Send `0` or space → `THR 0 (idle)`, `thr:0.000` immediately. Send `+` ~40 times → `thr` clamps at `0.75` and stops rising.

- [ ] **Step 6: Kill + disarm zero the throttle**

Raise throttle, then send `k`. Expected: `KILL`, `state:KILL`, `thr:0.000`, motors idle. Send `d`. Expected: `DISARMED`, `state:OFF`, `thr:0.000`. Re-arm with `a` and confirm `thr` is 0 again (not carried over).

- [ ] **Step 7: Record result**

If all steps pass, the feature is validated. Note any deviation (e.g. a key not registering, throttle not clamping) for follow-up. (Powered motor-spin test is a separate milestone, blocked on USB-vs-PDB power delivery.)

---

## Notes / Out of Scope (deferred, per spec)

- Altitude-hold controller (the `h`/`v` Kalman output stays reference-only).
- RC / SBUS input.
- Yaw control (no yaw term in the mixer).
- Powered motor-spin integration test.
