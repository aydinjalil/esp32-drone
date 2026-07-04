# Next Steps — Drone Firmware Roadmap

_Last updated: 2026-07-01 (end of session)_
_Branch: `firmware`_

## Where we are (validated on hardware)

IMU (ICM-20948 @ 0x69, calibrated) · 4× motors (pins + directions) · BMP581 baro ·
altitude estimator (`h`/`v`, fused KF + gravity cal) · attitude PID + X-mixing ·
manual throttle control · **Bluetooth control + telemetry + link-loss failsafe** ·
**powered motor run over BT** (props off). Everything committed/pushed on `firmware`.

Still simulated/manual: throttle is pilot-driven (no auto altitude hold); no RC;
no yaw control; ToF not wired.

---

## Prioritized plan

### 1. ToF sensor integration — quick win (start here)
**Goal:** wire the VL53L4CX (0x29) and feed it into the altitude estimate for
accurate near-ground altitude.
**Why first:** small, low-risk, and the FC already has the plumbing — the
ToF-priority Kalman update path and the `tofPresent`/`tofValid` guards are already
in place from earlier. It just needs the sensor wired + verified. Improves the
altitude signal that altitude-hold (step 2) depends on near the ground.
**Steps:**
- Wire VL53L4CX: shared I²C (SDA 21 / SCL 22), `XSHUT → GPIO 4` (NOT GPIO2 —
  strap pin, its XSHUT pull-up can block uploads), power/GND.
- Flash `i2c_scan.ino` → expect `0x29` to join `0x69` + `0x47`.
- Quick read test (a small sketch or the FC): confirm `tof:` in telemetry shows a
  real distance and `tofPresent`/`tofValid` behave.
- Confirm the Kalman uses ToF when valid (<4 m) and falls back to baro otherwise.
**Size:** ~1 short session. No brainstorm needed (integration, not new design).

### 2. Real altitude-hold controller — the main feature
**Goal:** auto-throttle to hold altitude using the validated `h`/`v`, as a *mode*
layered on top of manual throttle (not a replacement).
**Process:** brainstorm → spec → plan → subagent build → bench test (USB/BT, props
off) → powered test.
**Design questions to settle in brainstorm:**
- Setpoint source (recommend: capture-on-arm + serial trim, matching the manual-
  throttle safety model — no fixed/absolute setpoints).
- Controller shape: cascaded (altitude error → target climb rate → throttle around
  `hoverThrottle`) vs single PID. Cascaded uses `v` explicitly and is more robust.
- **Anti-windup** on the integral (this time it's real — unlike the simulated
  throttle we mistook for windup earlier).
- Mode toggle: manual ↔ altitude-hold, and behavior on arm.
- **Revisit the vertical-channel bench hacks** in the FC: the a_z deadband
  (`|a_z| < 0.2 → 0` in `computeVerticalAcceleration()`) and the `v *= 0.90`
  damping in `stepIMU()` suppress bench drift, but they also erase genuine
  constant-velocity climbs/descents (a_z ≈ 0 while moving → `v` decays ~99%/s).
  The KF covariance now converges properly, so reduce/remove these and let the
  filter do its job — altitude hold can't work well with a `v` that lies during
  steady climbs.
**Depends on:** trustworthy `h`/`v` (done); ToF (nice-to-have, step 1).
**Size:** the meaty one — a full feature cycle.

### 3. RC receiver + production failsafe — bigger, later
**Goal:** real piloting via SBUS/ELRS receiver + a proper link-loss failsafe.
**Failsafe principle (locked in — see memory `project-bt-control-and-failsafe`):**
link-loss → controlled **hold → Return-to-Home → land**, NEVER an input-timeout or
abrupt in-air disarm. This replaces BT-manual for real flight and is the
production-grade version of what we learned this session.
**Involves:** receiver hardware, protocol parsing, throttle/roll/pitch/yaw from RC,
failsafe state machine, arming safety review.

### 4. Yaw control — needs mag/yaw fusion in the FC
**Goal:** the FC currently has NO yaw fusion or yaw mixing. Add both.
**Involves:** port the calibrated-mag + tilt-comp heading from `imu_debug.ino` into
the FC (0x69, negate-Y remap, beta fusion), add a yaw term to `motorMixing()`, and
a yaw PID. Recalibrate mag in final flight config (away from laptop — current cal
is CV 8.37%, laptop-limited).

---

## Cross-cutting (before props-on hover)
- **PID direction bench test (MANDATORY, 2 min, props off).** Two reasons:
  1. **FIXED 2026-07-03 — mixer roll column was a diagonal.** `motorMixing()` had
     roll on FR+BL vs FL+BR, which sums to ZERO net roll moment (pure yaw torque
     instead) — no roll authority at any PID sign. Corrected to left/right pairs
     (right = FR+BR gets `+rollPID`). The fix must be verified on hardware.
  2. The FC's roll/pitch sign convention is the *negation* of the hardware-
     validated `imu_debug.ino` (`roll += gx·dt` + `atan2(ay, az)` vs `roll -=
     gx·dt` + `atan2(-ay, az)`). Whether the PID corrects or amplifies tilt
     depends on physical IMU mounting — only the bench can settle it.
  Procedure (USB or BT telemetry, props OFF):
  - **Step A — estimator direction (disarmed):** tilt right side down, note the
    sign of `r:` in telemetry; tilt nose down, note the sign of `p:`. Record both.
  - **Step B — motor response (armed, small throttle, e.g. `t` a few times):**
    - tilt **right side down** → `mFR` **and** `mBR` must both **rise** (and
      `mFL`/`mBL` both fall). If the left pair rises instead → flip the roll
      sign (negate the `rollPID` column in `motorMixing()`). If FR and BR move
      in *opposite* directions, the mixer is still wrong — stop and re-check.
    - tilt **nose down** → `mFR` **and** `mFL` must both **rise** (and
      `mBL`/`mBR` both fall). If the back pair rises instead → flip the pitch
      sign (negate the `pitchPID` column).
  Do NOT fly until both axes pass with pair-wise (not diagonal) motion.

  **Results 2026-07-03 (props off, on hardware):**
  - Step A: right side down → `r` **+**; nose down → `p` **+** (estimator
    convention recorded in the `motorMixing()` comment).
  - Step B: pitch PASSED (nose down → FR+FL rise; nose up → back pair rises).
    Roll was pair-wise (mixer de-diagonalization verified) but REVERSED —
    right-down raised the left pair → flipped the `rollPID` column sign.
  - Roll re-test after the sign flip: right side down → FR+BR rise, left pair
    drops; left side down → the opposite. **PASSED.**
  - **STATUS: COMPLETE 2026-07-03 — both axes correct on hardware.** Attitude
    PID direction is verified; this item no longer blocks props-on testing.
- A safe **tethered test rig** before any props-on flight attempt.
- Battery/voltage monitoring + low-battery response.
- Re-review arming/kill safety with props on.

## Open decisions for the morning
- Confirm start order: **ToF first (recommended), then altitude-hold** — or go
  straight to altitude-hold on baro and add ToF later?
- Altitude-hold setpoint model (settle in its brainstorm).

## How we work (reminders)
- Frequent commits + push; clean logical commits; **no AI/co-author references**.
- New features: brainstorm → spec → plan → subagent-driven build. Integration/fixes:
  direct + structural verify (no compiler/test framework for `.ino`).
- Bench tests are props OFF; battery XOR USB (never both — soldered PDB contention).
