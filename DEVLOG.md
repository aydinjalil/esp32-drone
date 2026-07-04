# ESP32 Drone — Development Log

## Project Overview

Altitude-hold drone flight controller running on a classic ESP32 DevKit (WROOM-32).  
The goal is a sensor fusion stack that estimates altitude and orientation accurately enough to feed a PID flight controller.

**Hardware**
| Component | Part | Interface |
|-----------|------|-----------|
| MCU | classic ESP32 DevKit (WROOM-32) | — |
| IMU | ICM-20948 (accel + gyro + mag) | I2C (SDA=21, SCL=22) |
| Barometer | BMP581 (BMP5xx) | I2C addr 0x47 |
| TOF rangefinder | VL53L4CX | I2C addr 0x29, XSHUT=GPIO4 |
| Display (optional) | SSD1306 | I2C |

**Toolchain**
- Arduino IDE (macOS) + arduino-cli bundled inside `Arduino IDE.app`
- `esp32:esp32` core v3.1.0 (community) / `arduino:esp32` v2.0.18
- Board: "ESP32 Dev Module" (`esp32:esp32:esp32`); flash over USB `/dev/cu.usbserial-0001` (never the BT port `/dev/cu.DRONE_FC`)

---

## Issues Faced & Resolutions

### 1. Upload failing — wrong USB port

**Symptom:** esptool reported `Failed to write to target RAM (Operation timed out)` or `Requested resource not found` on every attempt.

**Root cause:** The ESP32-S3 exposes two USB endpoints. The port being used (`/dev/cu.usbmodem5A460940181`) was the native USB CDC endpoint, which doesn't respond correctly to the UART stub-loader protocol. The correct programming port was `/dev/cu.usbmodem1101` (the CH340 UART-to-USB chip).

**Resolution:** Switched to the correct port. Upload succeeded immediately.

**Lesson:** On upload failure, **change the port first** before trying any esptool flags. (Historical entry from early bring-up; on the current classic ESP32 DevKit the same class of failure recurred when the IDE port was set to the Bluetooth serial `/dev/cu.DRONE_FC` — flashing only works on the UART port `/dev/cu.usbserial-0001`.)

---

### 2. No sensor readings on Serial Monitor

**Symptom:** Code compiled and uploaded without errors. Serial Monitor at 115200 baud showed nothing.

**Root cause (A) — Pressure units:** `bmp.readPressure()` returns Pascals (~101325 Pa). The validation guard `pressure > 1200` immediately returned from `updateBarometer()` because 101325 >> 1200. `baroAlt` was never updated, so `h` stayed at 0. The ground calibration loop was also accumulating Pa values, so `groundPressure` was ~101325 Pa (meaningless as a ratio denominator since it was consistent, but the guard blocked everything).

**Resolution:** Replaced `bmp.readPressure()` with `bmp.performReading()` + `bmp.pressure`, which the Adafruit BMP5xx library already exposes in hPa. Validation bounds (300–1200) are now correct.

**Root cause (B) — Serial blocked before first print:** The very first `Serial.println` only fires after `bmp.begin()` succeeds. Any hang before that (e.g. TOF init on I2C) produces a completely blank Serial Monitor with no indication of where the code stopped. `while(!Serial) delay(10)` also blocked indefinitely if the Serial Monitor wasn't open at boot time.

**Resolution:** Replaced `while(!Serial)` with `delay(5000)` (gives time to open the monitor after a reset). TOF init now has a return-value check so a missing sensor doesn't silently lock the I2C bus.

**Root cause (C) — I2C SDA/SCL arguments swapped:** Original code called `I2Cdev.begin(SCL_PIN, SDA_PIN)` but `Wire.begin(sda, scl)` takes SDA first. Sensors happened to work because the physical wiring was adjusted to match, but the pin #defines (`SDA_PIN 16`, `SCL_PIN 17`) were misleading. Later corrected to `SDA_PIN 21`, `SCL_PIN 22` with correct argument order.

---

### 3. Magnetometer yaw drift / heading changes with roll — RESOLVED (2026-06-13)

**Symptom:** Heading (yaw) changed when the board was rolled, and drifted rather
than returning to the original heading after levelling. Worse with the board
lifted off the table.

**Root cause:** Two compounding problems, both now fixed:
1. **No magnetometer calibration** — raw readings (uncorrected hard/soft iron).
2. **Magnetometer frame mismatch** — the AK09916 mag axes are oriented
   differently from the ICM-20948 accel/gyro, and the Adafruit library reports
   the mag in its own native frame. Roll/pitch (from accel) were applied to
   mis-aligned mag axes in the tilt-comp formula, so heading moved with roll.
   The tilt-comp math itself was correct; it was being fed the wrong frame.

**Resolution:** Developed a calibration + verification toolchain in `drone_imu/`
(see `drone_imu/docs/ICM20948_STATUS.md`):
- **Full hard-iron + 3×3 soft-iron calibration** via ellipsoid fit
  (`mag_calibrate.py`), replacing the old diagonal offset+scale model. A diagonal
  model cannot correct a *tilted* ellipsoid; the 3×3 matrix can.
- **Data-derived axis remap.** The generic datasheet "swap X/Y, negate Z" remap
  was **wrong** for this board. Logging a manoeuvre and brute-forcing all 48
  signed axis permutations (scoring by how constant the horizontal field stays
  under roll) showed the correct mapping is **"negate Y only"**:
  `body = (+X, −Y, +Z)` of the native frame.
- **Verified tilt-compensation** formula `bx/by → atan2(-by, bx)`.

**Verification (heading-independent):** de-rotating the calibrated body mag by
roll/pitch should leave the horizontal field `H` constant.

| | dH/droll (µT/deg) | H CV |
|---|---|---|
| Old (swap X/Y, −Z, no cal) | 0.766 (broken) | 25.3% |
| Final (−Y only, 3×3 cal) | **0.006** | **2.9%** |

Calibration quality: total `|m|` CV **1.9%**, ellipsoid axis ratio **1.22**.
Confirmed on hardware: heading now roll/pitch-invariant; tilt-invariance test
passes. Applied to both `drone_imu/arduino/imu_debug/imu_debug.ino` and
`AdvanceSensorIntegration.ino`.

**Note:** `alpha_yaw = 0.95` (5% mag correction per step) is still in
`AdvanceSensorIntegration.ino` — much more aggressive than the `beta = 0.005`
used in `imu_debug`. Now that heading is correct, this may want softening to
reduce yaw noise (untested).

---

### 4. ICM-20948 I²C stall — magnetometer freezes during long runs — MITIGATED (2026-06-13)

**Symptom:** During capture/long runs (~2–3 min in), the magnetometer started
returning `0,0,0`, or the entire IMU froze and `getEvent()` returned stale,
byte-identical values indefinitely.

**Root cause:** The ICM-20948 reads its AK09916 magnetometer over an internal
I²C master, which can hang during sustained polling — and the stall can lock the
whole chip's I²C interface, freezing accel/gyro too. Known silicon/firmware
quirk; happens even with correct pull-ups.

**Resolution (software):** Stall detection + recovery added to the sketches:
- A live gyro is never byte-identical across reads, so 20 consecutive identical
  gyro samples (~0.4 s) ⇒ stall ⇒ re-initialise the IMU.
- The raw-capture sketch also treats sub-1 µT reads as invalid (skipped, never
  logged — zeros would poison the calibration fit).
- Logger/viewer drop invalid rows; `setMagDataRate(100 Hz)` added in init.

Reactive band-aid; if stalls become frequent, hardware suspects are I²C signal
integrity (wire length, pull-ups) and VDD decoupling. Relates to the
"No watchdog" item under Potential Future Problems.

---

### 5. On-drone bring-up — IMU @ 0x69, sensor/motor/FC validation, estimator fixes (2026-06-30)

Full hardware bring-up session. Everything below was verified on the bench
(props OFF, USB power, one power source at a time).

**Board correction:** this build runs on a **classic ESP32 DevKit (WROOM-32)**,
NOT an ESP32-S3 — confirmed because the ESCs work on GPIO18/19/23/25 and GPIO25
does not exist on the S3. (The Project Overview header and other sections have
since been corrected; SDA=21, SCL=22 were always correct.)

**IMU I²C address is 0x69, not 0x68.** SDO/AD0 is wired to VCC. All sketches
updated from `ICM20948_I2CADDR_DEFAULT` to a literal `0x69` (`imu_debug`,
`advanced_sensor_integration`, `drone_flight_controller`, `mag_capture_raw`).
Added `drone_imu/arduino/i2c_scan/i2c_scan.ino` to confirm bus addresses.

**Magnetometer recalibrated on the drone** (`mag_calibrate.py`, 11045 samples):
axis ratio 1.17, off-diagonal 4.0%, CV 8.37%. The residual is laptop proximity
during capture — recapture far from metal for flight-grade heading. New
`MAG_B`/`MAG_A` in `imu_debug.ino`; yaw nudge `beta` 0.005 → 0.01 (~1.7 s settle).

**Motors:** all four ESC directions verified individually via `motor_test.ino`
(FR=CCW, FL=CW, BL=CCW, BR=CW). Props: CCW→FR&BL, CW→FL&BR.

**Barometer:** BMP581 @ 0x47 wired (STEMMA QT chain off the IMU) and verified
reading sane pressure (~1006 hPa) with altitude tracking (`baro_test.ino`).

**Flight controller validated on hardware** (`drone_flight_controller.ino`,
IMU+baro, ToF absent): boots cleanly, serial arming `a`/`d`/`k` with pre-arm
checks works, attitude PID + X-mixing respond correctly to tilt, and altitude
holds at height when armed (takeoff reference frozen at arm). ToF init made
non-fatal (`tofPresent` guard) so the FC runs baro-only until the ToF is wired.

**Altitude estimator fixed:**
- Collapsed the double measurement update (baro was applied twice per loop) into
  a single fused `kalmanUpdate(z, R)` — ToF priority when valid (<4 m), else baro.
- Added the missing covariance reduction `P = (I − K·H)·P` — the gain now adapts
  instead of staying pinned high; `h` is stable (no jitter).
- Fixed a steady `v ≈ −0.22 m/s` bias: subtract the **measured** at-rest accel
  magnitude (`gravityMag`, calibrated in the still loop) instead of a hardcoded
  9.81. `v` now ~0 at rest and rises-then-settles on lift.
- Added `v` to the telemetry line.

**Still simulated (next task):** throttle and roll/pitch targets are bench sims
(`0.55+0.1·sin`, ±5°). No real altitude-hold controller or RC input yet.

---

## Current Architecture

```
loop()
  ├── imu.getEvent()          → ax/ay/az, gx/gy/gz (rad/s)
  ├── readTOF()               → tofAlt (m) [future use]
  ├── updateBarometer()       → baroAlt (m)
  ├── stepIMU()
  │     ├── updateOrientation()  → roll/pitch/yaw (complementary + calibrated, tilt-compensated mag)
  │     └── computeVerticalAcceleration() → a_z
  ├── kalmanPredict()         → h, v (IMU propagation; a_z = accel − gravityMag)
  └── kalmanUpdate(z, R)      → h, v + covariance (single fused update:
                                ToF if valid <4 m, else baro)
```

**Kalman filter state:** `[h, v]` — altitude and vertical velocity.  
**Process noise:** Qh=0.02, Qv=0.10  
**Measurement noise:** R_baro=2.0, R_tof=0.005  
**Covariance update:** `P = (I − K·H)·P` applied on every measurement (2026-06-30).  
**Gravity:** at-rest accel magnitude calibrated at boot (`gravityMag`), not a fixed 9.81.

**Calibration at boot (setup):**
- 5s delay for Serial Monitor to open
- BMP581 settles for 2s
- Ground pressure: 500 samples × 25ms = ~12.5s average
- Gyro bias: 2000 samples × 2ms = ~4s average

---

## What Works

- Compilation and upload via Arduino IDE, board "ESP32 Dev Module" (`esp32:esp32:esp32`), port `/dev/cu.usbserial-0001`
- BMP581 reading pressure in hPa via `bmp.performReading()` / `bmp.pressure`
- ICM-20948 accel + gyro events via `imu.getEvent()`
- ICM-20948 magnetometer via `imu.getMagnetometerSensor()->getEvent()`
- Complementary filter for roll/pitch
- **Magnetometer hard-iron + 3×3 soft-iron calibration** (`MAG_B`/`MAG_A`), data-derived axis remap, and verified tilt-compensated heading — heading is now roll/pitch-invariant (Issue #3)
- **I²C stall detection + IMU re-init recovery** (Issue #4)
- Gyro-integrated + mag-corrected yaw
- Kalman filter predict + baro update loop
- Ground pressure calibration at boot
- Gyro bias calibration at boot
- VL53L4CX TOF sensor scaffolding (`readTOF()` reads data if available)
- **Magnetometer calibration/debug toolchain** in `drone_imu/` (raw-capture sketch, ellipsoid-fit calibrator, serial logger, 3D viewer) — see `drone_imu/docs/ICM20948_STATUS.md`
- **All four ESC/motor channels** bench-verified (pins + directions) via `motor_test.ino`
- **BMP581 on-drone** wired and reading (pressure + altitude) via `baro_test.ino`
- **Flight controller bench-validated** on real hardware (IMU+baro): serial arm/disarm/kill + pre-arm checks, attitude PID + X-mixing under tilt, altitude hold at height when armed
- **Altitude estimator** with single fused baro/ToF update, covariance reduction, and boot gravity-magnitude calibration — `h` stable, `v` ~0 at rest and tracks lift

## What Doesn't Work Yet

- TOF not yet wired on the drone (Kalman has a ToF-priority update path ready; runs baro-only via the `tofPresent` guard)
- No **real** altitude-hold controller — throttle is still a simulated `0.55+0.1·sin` (next task)
- No RC input — roll/pitch targets are a bench sim (±5°)
- Powered motor-response test pending (blocked on USB-vs-PDB power delivery; logic validated unpowered)
- No display output (SSD1306 included but unused)

---

## Potential Future Problems

### Hardware
- **TOF I2C conflict:** VL53L4CX initialization is called unconditionally. If the sensor is absent or XSHUT wiring is wrong, I2C can be left in a bad state before BMP/IMU init. Should add a return-value check on `InitSensor()` and a fallback.
- **ICM-20948 magnetometer interference:** The magnetometer on the ICM-20948 shares the die with the gyro. Motor/ESC EMI will corrupt readings significantly once motors are attached. May need an external magnetometer mounted away from power electronics.
- **BMP581 airflow:** Barometer on a drone must be shielded from prop wash. Without a foam cover or pressure port, altitude readings will be noisy during flight.
- **Wrong serial port breaks uploads:** with Bluetooth paired, macOS exposes `/dev/cu.DRONE_FC` alongside the UART port `/dev/cu.usbserial-0001`. Selecting the BT port in the IDE silently breaks flashing ("No serial data received"). Always flash on the UART port.

### Software
- **`dt` clamping loses real time:** When `dt > 0.05s` (20 Hz minimum), the Kalman filter resets dt to 10ms. If the loop genuinely runs slower than 20 Hz (e.g. during I2C blocking), the filter's time model is silently wrong.
- **`updateGroundPressure()` not called:** Ground reference never updates after boot. If the drone is armed indoors and flown outdoors (or if pressure changes significantly), altitude will drift. Consider re-enabling with tighter motion guards.
- **Kalman only fuses baro:** The IMU's vertical acceleration feeds the predict step, but no update step uses it directly. With poor barometer data (e.g., near the ground in prop wash), `h` will diverge. TOF fusion would fix the near-ground case.
- **Gyro bias assumed static:** Bias is computed once at boot. Temperature-induced drift during flight is not compensated.
- **No watchdog:** A hung I2C transaction will freeze the entire loop with no recovery. Consider enabling the ESP32 hardware watchdog timer.
- **No NaN propagation guard in Kalman covariance:** If P00/P11 grow unbounded (e.g. no baro updates for a long time), the Kalman gain saturates. There is a validity check but no periodic reset mechanism.

---

## Roadmap

### Phase 1 — Sensor validation (current)
- [x] BMP581 reading in correct units (hPa)
- [x] ICM-20948 accel/gyro/mag working
- [x] Kalman filter altitude estimate from baro
- [x] Complementary filter roll/pitch
- [x] Gyro-integrated yaw with magnetometer correction
- [x] Verify altitude readings are stable at rest (h stable, v ~0 — 2026-06-30, after covariance + gravity-cal fixes)
- [x] Verify roll/pitch/yaw visually (3D viewer `drone_imu/python/imu_debug/imu_view_csv.py`) — orientation tracks correctly with new calibration
- [ ] Verify roll/pitch accuracy against a quantitative reference angle

### Phase 2 — Magnetometer calibration — DONE (2026-06-13)
- [x] Sphere-coverage capture + ellipsoid-fit calibration toolchain (`drone_imu/`)
- [x] Apply hard-iron offset + full 3×3 soft-iron matrix to mx/my/mz
- [x] Data-derived axis remap into body frame ("negate Y only")
- [x] Full tilt-compensated heading formula (verified roll-invariant)
- [ ] Re-calibrate in final flight configuration (mounted with battery/ESCs/wiring)
- [ ] Soften `alpha_yaw` in AdvanceSensorIntegration (currently 0.95 = 5%/step)

### Phase 3 — TOF fusion
- [ ] Fuse VL53L4CX altitude into Kalman filter as a second measurement update (`kalmanUpdateTOF()`) when tofAlt < 2m
- [ ] Handle TOF measurement latency (async data-ready interrupt vs polling)
- [ ] Validate TOF against baro for consistency

### Phase 4 — Ground pressure tracking
- [ ] Re-enable `updateGroundPressure()` with validated motion guards
- [ ] Test long-duration ground drift without flight

### Phase 5 — Flight controller integration
- [ ] PID controller for altitude hold using `h` and `v` from Kalman filter
- [ ] PID for roll/pitch stabilization
- [x] ESC PWM output (ESP32Servo, GPIO 18/19/23/25, 1000–1900 µs)
- [ ] RC receiver input (SBUS / PPM / ELRS)
- [ ] Arm/disarm logic with failsafe

### Phase 6 — Robustness
- [ ] Hardware watchdog for I2C hangs
- [ ] In-flight gyro bias correction (zero-velocity updates)
- [ ] Barometer foam shield / pressure port
- [ ] Move magnetometer off main PCB if motor noise degrades yaw
