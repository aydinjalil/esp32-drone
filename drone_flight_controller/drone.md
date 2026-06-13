# Drone Build Background Notes

This document summarizes the drone-related portions of the conversation history, focusing on sensor selection, altitude estimation code, filtering behavior, and debugging issues. It is intended as background context for a new discussion rather than as a final design specification.

## Build context

The project evolved as a custom drone build centered on altitude estimation and onboard sensing. The discussion covered a barometric altitude path based first on a BMP280 and later on a BMP5xx sensor, an IMU path based on an MPU6050 and later an ICM20948, a VL53L0X time-of-flight sensor as a low-altitude ranging aid, OLED display output, SD card logging concerns, camera selection, and spare motor selection.

The main software goal was to build a responsive and physically reasonable altitude estimate suitable for eventual control use. The hardware goal was to assemble a practical prototype with decent sensing, logging, and video capability.

## Altitude sensing discussion

### BMP280 behavior and drift

Early testing showed that at ground level the raw altitude reading could be around -1.6 m and the filtered altitude around -1.55 m. That led to discussion about whether a simple zero-offset calibration was acceptable. The user correctly pushed back on the idea of treating a fixed offset as a real solution because the raw altitude had been near zero earlier in the day and drifted later as temperature and atmospheric conditions changed.

The key point that emerged was:

- A barometric sensor can appear to drift in altitude when ambient pressure changes.
- A fixed zero-offset can hide the symptom at one moment but does not model the cause.
- Temperature correction needs to be physically correct to be useful.

### Temperature-corrected altitude formula debate

A substantial part of the conversation focused on the correct form of the altitude equation.

Issues that came up:

- A proposed function included `currentTemp` in the signature but did not actually use it.
- A later version attempted to use `groundTemperature` and `currentTemp` with an extra term like `T0 / T_current - 1.0f`, which the user correctly identified as suspicious.
- There was discussion about whether the specific gas constant for dry air was being used correctly, and whether molar mass should appear explicitly.
- There was also discussion about whether the equation should use a power function or a logarithm.

The most useful takeaways for the next discussion are:

- If using the specific gas constant for dry air, `R_d = 287.05 J/(kg·K)` already incorporates the universal gas constant divided by the molar mass of dry air.
- If writing the equation explicitly in terms of universal constants, then use `R_u / M`.
- The user correctly challenged the appearance of the `T0 / T_current - 1.0f` factor.
- The user also correctly suggested that a logarithmic form may be more appropriate depending on the chosen derivation and assumptions.

This topic should be revisited carefully from first principles in the next discussion instead of relying on earlier tentative formulas.

### Filtering responsiveness

At one point the raw altitude became accurate but the filtered altitude was not responsive. This led to discussion of two different but related causes:

- External filtering in code, using a low-pass expression such as `filteredAlt = 0.80 * filteredAlt + 0.20 * correctedAlt`, can create a sluggish response if the old value dominates too heavily.
- Internal barometer oversampling and filtering settings can also add latency.

A bug was also suspected in the order of operations around filtered altitude updates, though the bigger long-term concern remained whether the altitude model itself was sound.

## BMP280 configuration notes

Several BMP280 settings were discussed, especially with respect to internal filtering and standby timing. A compile error occurred when using a standby enum that did not exist in the installed Adafruit library. The user then supplied the actual valid enum names from the library, showing that the correct standby option in that version was `STANDBY_MS_63` rather than `STANDBY_MS_62_5`.

Useful note:

- The installed Adafruit library used `STANDBY_MS_63` for the 62.5 ms option.

## Range sensor discussion

The VL53L0X time-of-flight sensor was discussed as a complementary sensor for low-altitude measurement.

Key conclusions from that discussion:

- A ToF sensor can be very useful for precise low-altitude ranging near the ground.
- It complements a barometer well because the ToF sensor is better near the ground while the barometer is better over a wider range.
- It is especially useful for takeoff, landing, and hover close to the ground.

This remained a promising addition to the build.

## Camera discussion

A generic AliExpress camera was evaluated and judged to be a poor fit for drone use because it likely resembled a generic camera module rather than a drone-focused recording system. The recommendation shifted toward small purpose-built recording cameras.

The strongest recommendation that emerged was:

- RunCam Thumb 2 Pro as a lightweight, drone-suitable video camera.

The reasons discussed included compact size, light weight, onboard recording, and suitability for a custom drone platform.

## Motors discussion

The user later asked about buying spare motors and clarified that the motors in question were EMAX ECO III 1900KV units.

Main conclusions from that discussion:

- 1900KV was treated as a good match for a 6S 5-inch style build.
- EMAX ECO III was viewed as a meaningful improvement over no-name generic motors.
- Buying them as spares was considered reasonable.

This should still be checked against the exact frame, props, battery, and thrust targets in the next discussion.

## SD card note

An "sd card failed" issue came up briefly. The likely causes discussed were:

- Reader or contact problems.
- File system corruption.
- Wiring or SPI/power issues if the failure happened on the ESP32-based drone electronics.

No final resolution was established in the conversation.

## Evolution to BMP5xx + ICM20948 + Kalman filter

Later in the project, the code moved away from the BMP280 path and toward a newer architecture using:

- `Adafruit_BMP5xx` for pressure sensing.
- `Adafruit_ICM20948` for IMU data.
- A simple Kalman filter with state variables:
  - `h` for altitude.
  - `v` for vertical velocity.

The code also included:

- Gyro bias calibration.
- Roll and pitch estimation with a complementary filter.
- Gravity compensation to derive vertical acceleration `a_z`.
- Barometric altitude update.
- Kalman prediction and barometer measurement update.

## Main code/debug issues encountered

### 1. `NaN` in `v` and `h`

A major issue appeared when `v` and `h` were printed as `nan`.

The most important likely causes identified were:

- `groundPressure` started at zero and could be used before being initialized.
- `baroAlt` computation could become invalid if pressure ratio calculations used a bad reference pressure.
- Once `baroAlt`, `a_z`, `h`, or `v` became `NaN`, the Kalman filter would propagate that failure through the state.

The debugging direction that emerged was:

- Validate all sensor inputs.
- Reject invalid calculations.
- Guard the Kalman update against non-finite values.
- Ensure `groundPressure` is initialized before altitude calculations rely on it.

### 2. `baroAlt` stuck at zero

After adding zero and validity guards, a later issue was that `baroAlt` remained zero and `h` was unresponsive.

The likely logic problem identified was:

- `updateBarometer()` was called before `updateGroundPressure()` in the loop.
- `updateBarometer()` returned early if `groundPressure` was not yet valid.
- `updateGroundPressure()` only updated the reference when the code believed the drone was nearly motionless, so `groundPressure` could remain zero for too long.

The recommended direction was:

- Initialize `groundPressure` during `setup()` by averaging pressure samples.
- Then use `updateGroundPressure()` only for slow adaptation afterward.
- In the loop, update the reference pressure before computing `baroAlt`.

### 3. Duplicate gyro bias calibration work

The code contained both a `calibrateGyroBias()` function and then another manual gyro-bias averaging block in `setup()`. That duplication is not necessarily harmful but suggests the initialization path is still in flux and should be cleaned up.

### 4. Filter and state tuning still immature

The Kalman filter used manually chosen values such as:

- `Qh = 0.02f`
- `Qv = 0.10f`
- `R_baro = 2.0f`

These are reasonable placeholders, but there was no final tuning process completed in the conversation. A future discussion should likely revisit:

- Proper units and expected variance for the BMP5xx altitude measurement.
- Whether acceleration should be modeled as control input noise rather than folded into `Qh` and `Qv` this way.
- Whether the current covariance update is sufficient for the intended dynamic performance.

## Current code architecture snapshot

The later code structure can be summarized as follows.

### Sensor stack

- BMP5xx barometer.
- ICM20948 IMU.
- Optional future VL53L4CX ToF sensor.
- Optional OLED.

### Main estimation path

1. Compute `dt` from `micros()`.
2. Read IMU acceleration and gyro values.
3. Remove gyro bias.
4. Estimate roll and pitch with gyro integration plus accelerometer correction.
5. Rotate acceleration into the world vertical axis.
6. Subtract gravity to estimate `a_z`.
7. Read and filter pressure.
8. Convert pressure to `baroAlt` using a ground reference pressure.
9. Run Kalman predict using `a_z`.
10. Run Kalman update using `baroAlt`.

### Robustness features already added

- `isValidFloat()` helper.
- Pressure range validation.
- Safe handling for invalid altitude ratio.
- Sanity checks for covariance and Kalman gain.
- Reset path for covariance if it becomes invalid.
- Deadband on small vertical acceleration.

## Open technical questions for the next discussion

These issues remain unresolved and should be considered the starting point for a new conversation:

1. What is the correct pressure-to-altitude equation for this application, and under what assumptions should temperature enter it?
2. Should the altitude conversion use a power-law barometric formula, a logarithmic hypsometric form, or a simplified near-ground approximation?
3. How should `groundPressure` be initialized and updated during operation to avoid both startup lockout and weather drift?
4. Is the current world-frame vertical acceleration calculation correct for the chosen body-axis convention of the IMU mounting?
5. Should the Kalman filter remain a 2-state altitude/velocity filter, or expand to include accelerometer bias or barometer bias?
6. How should process and measurement noise be tuned for responsiveness versus stability?
7. When the ToF sensor is added, how should it be fused with the barometer and IMU?

## Practical recommendations carried forward

For future work, the conversation history suggests the following practical starting assumptions:

- Initialize and verify every sensor path independently before fusing them.
- Print raw pressure, ground pressure, barometric altitude, vertical acceleration, velocity, and height together during debug.
- Establish a clean startup calibration phase for gyro bias and ground pressure.
- Do not trust a temperature-corrected altitude formula until its derivation is explicitly agreed upon.
- Treat the ToF sensor as the preferred source near the ground once integrated.
- Keep the camera and motor choices practical and secondary to stabilizing the estimator first.

## Suggested first step for the next discussion

A good next step would be to isolate the estimator into three separate debug stages:

1. Barometer-only altitude with startup pressure calibration.
2. IMU-only vertical acceleration sanity check with the drone held level and then tilted.
3. Kalman fusion only after both standalone signals are behaving sensibly.

That staged approach should make it easier to determine whether the remaining issue is in the pressure reference logic, the frame transformation, the filter tuning, or the fusion structure itself.
