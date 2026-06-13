# ESP32-S3 Drone — Development Log

## Project Overview

Altitude-hold drone flight controller running on an ESP32-S3 (QFN56, rev 0.2, 8 MB OPI PSRAM, 16 MB flash).  
The goal is a sensor fusion stack that estimates altitude and orientation accurately enough to feed a PID flight controller.

**Hardware**
| Component | Part | Interface |
|-----------|------|-----------|
| MCU | ESP32-S3 (rev 0.2, 8 MB PSRAM) | — |
| IMU | ICM-20948 (accel + gyro + mag) | I2C (SDA=21, SCL=22) |
| Barometer | BMP581 (BMP5xx) | I2C addr 0x47 |
| TOF rangefinder | VL53L4CX | I2C addr 0x29, XSHUT=GPIO4 |
| Display (optional) | SSD1306 | I2C |

**Toolchain**
- Arduino IDE (macOS) + arduino-cli bundled inside `Arduino IDE.app`
- `esp32:esp32` core v3.1.0 (community) / `arduino:esp32` v2.0.18
- FQBN used: `esp32:esp32:esp32s3`

---

## Issues Faced & Resolutions

### 1. Upload failing — wrong USB port

**Symptom:** esptool reported `Failed to write to target RAM (Operation timed out)` or `Requested resource not found` on every attempt.

**Root cause:** The ESP32-S3 exposes two USB endpoints. The port being used (`/dev/cu.usbmodem5A460940181`) was the native USB CDC endpoint, which doesn't respond correctly to the UART stub-loader protocol. The correct programming port was `/dev/cu.usbmodem1101` (the CH340 UART-to-USB chip).

**Resolution:** Switched to the correct port. Upload succeeded immediately.

**Lesson:** On ESP32-S3 boards with multiple USB ports, always verify you are using the UART/CH340 port for programming, not the native USB CDC port. On upload failure, **change the port first** before trying any esptool flags.

---

### 2. No sensor readings on Serial Monitor

**Symptom:** Code compiled and uploaded without errors. Serial Monitor at 115200 baud showed nothing.

**Root cause (A) — Pressure units:** `bmp.readPressure()` returns Pascals (~101325 Pa). The validation guard `pressure > 1200` immediately returned from `updateBarometer()` because 101325 >> 1200. `baroAlt` was never updated, so `h` stayed at 0. The ground calibration loop was also accumulating Pa values, so `groundPressure` was ~101325 Pa (meaningless as a ratio denominator since it was consistent, but the guard blocked everything).

**Resolution:** Replaced `bmp.readPressure()` with `bmp.performReading()` + `bmp.pressure`, which the Adafruit BMP5xx library already exposes in hPa. Validation bounds (300–1200) are now correct.

**Root cause (B) — Serial blocked before first print:** The very first `Serial.println` only fires after `bmp.begin()` succeeds. Any hang before that (e.g. TOF init on I2C) produces a completely blank Serial Monitor with no indication of where the code stopped. `while(!Serial) delay(10)` also blocked indefinitely if the Serial Monitor wasn't open at boot time.

**Resolution:** Replaced `while(!Serial)` with `delay(5000)` (gives time to open the monitor after a reset). TOF init now has a return-value check so a missing sensor doesn't silently lock the I2C bus.

**Root cause (C) — I2C SDA/SCL arguments swapped:** Original code called `I2Cdev.begin(SCL_PIN, SDA_PIN)` but `Wire.begin(sda, scl)` takes SDA first. Sensors happened to work because the physical wiring was adjusted to match, but the pin #defines (`SDA_PIN 16`, `SCL_PIN 17`) were misleading. Later corrected to `SDA_PIN 21`, `SCL_PIN 22` with correct argument order.

---

### 3. Magnetometer yaw drift

**Symptom:** Yaw accumulated drift with gyro-only integration.

**Status:** Partially addressed. Complementary filter blends gyro integration with magnetometer heading at `alpha_yaw = 0.95` (5% mag correction per step). The tilt-compensated heading formula is implemented but the simplified `atan2(mx, my)` is currently used instead of the full tilt-compensation to avoid compounding errors before roll/pitch are validated.

**Remaining issue:** No hard-iron / soft-iron calibration has been done on the ICM-20948 magnetometer. Readings are raw. The calibration min/max variables (`magMinX`, `magMaxX`, etc.) exist in code but the calibration routine is commented out.

---

## Current Architecture

```
loop()
  ├── imu.getEvent()          → ax/ay/az, gx/gy/gz (rad/s)
  ├── readTOF()               → tofAlt (m) [future use]
  ├── updateBarometer()       → baroAlt (m)
  ├── stepIMU()
  │     ├── updateOrientation()  → roll/pitch/yaw (complementary + mag)
  │     └── computeVerticalAcceleration() → a_z
  ├── kalmanPredict()         → h, v (IMU propagation)
  └── kalmanUpdateBaro()      → h, v (baro correction)
```

**Kalman filter state:** `[h, v]` — altitude and vertical velocity.  
**Process noise:** Qh=0.02, Qv=0.10  
**Measurement noise:** R_baro=2.0 hPa

**Calibration at boot (setup):**
- 5s delay for Serial Monitor to open
- BMP581 settles for 2s
- Ground pressure: 500 samples × 25ms = ~12.5s average
- Gyro bias: 2000 samples × 2ms = ~4s average

---

## What Works

- Compilation and upload via `arduino-cli` on `esp32:esp32:esp32s3`
- BMP581 reading pressure in hPa via `bmp.performReading()` / `bmp.pressure`
- ICM-20948 accel + gyro events via `imu.getEvent()`
- ICM-20948 magnetometer via `imu.getMagnetometerSensor()->getEvent()`
- Complementary filter for roll/pitch
- Gyro-integrated + mag-corrected yaw
- Kalman filter predict + baro update loop
- Ground pressure calibration at boot
- Gyro bias calibration at boot
- VL53L4CX TOF sensor scaffolding (`readTOF()` reads data if available)

## What Doesn't Work Yet

- Magnetometer hard/soft-iron calibration (raw values used — yaw accuracy is poor)
- TOF altitude is read but **not fused** into the Kalman filter
- `updateGroundPressure()` exists but is **not called** in the loop (was removed; ground reference is static after calibration)
- No PID or motor output — sensor fusion only
- No display output (SSD1306 included but unused)

---

## Potential Future Problems

### Hardware
- **TOF I2C conflict:** VL53L4CX initialization is called unconditionally. If the sensor is absent or XSHUT wiring is wrong, I2C can be left in a bad state before BMP/IMU init. Should add a return-value check on `InitSensor()` and a fallback.
- **ICM-20948 magnetometer interference:** The magnetometer on the ICM-20948 shares the die with the gyro. Motor/ESC EMI will corrupt readings significantly once motors are attached. May need an external magnetometer mounted away from power electronics.
- **BMP581 airflow:** Barometer on a drone must be shielded from prop wash. Without a foam cover or pressure port, altitude readings will be noisy during flight.
- **ESP32-S3 dual USB ports:** The board exposes both a CH340 UART port and a native USB CDC port. Picking the wrong one silently breaks uploads. No guard exists in the toolchain to catch this.

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
- [ ] Verify altitude readings are stable at rest (~0m drift over 60s)
- [ ] Verify roll/pitch accuracy against a reference angle

### Phase 2 — Magnetometer calibration
- [ ] Implement figure-8 / sphere calibration routine to collect min/max (code scaffolded)
- [ ] Apply hard-iron offset and soft-iron scale to mx/my/mz before yaw computation
- [ ] Use full tilt-compensated heading formula (currently simplified)

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
- [ ] ESC PWM output (likely LEDC or MCPWM on ESP32-S3)
- [ ] RC receiver input (SBUS / PPM / ELRS)
- [ ] Arm/disarm logic with failsafe

### Phase 6 — Robustness
- [ ] Hardware watchdog for I2C hangs
- [ ] In-flight gyro bias correction (zero-velocity updates)
- [ ] Barometer foam shield / pressure port
- [ ] Move magnetometer off main PCB if motor noise degrades yaw
