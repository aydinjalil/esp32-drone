# ICM-20948 IMU — Status & Engineering Notes

_Last updated: 2026-06-13_

Attitude + heading (AHRS) subsystem for the ESP32 drone, built around an
Adafruit ICM-20948 (accel + gyro + AK09916 magnetometer) over I²C.

**Current state: working.** Accel/gyro give stable roll & pitch; the
magnetometer is calibrated (full hard- + soft-iron) and frame-aligned, so the
tilt-compensated heading is now roll-invariant. The earlier "yaw drifts/changes
when you roll the board" bug is **fixed**.

---

## 1. Hardware setup

| Item | Value |
|---|---|
| Sensor | Adafruit ICM-20948 (ICM-20948 + AK09916 mag) |
| Bus | I²C, default address (`ICM20948_I2CADDR_DEFAULT`) |
| Pins | SDA = GPIO 21, SCL = GPIO 22 |
| Clock | 100 kHz |
| Pull-ups | Onboard breakout pull-ups only (no external resistors added) |
| Accel range | ±4 g |
| Gyro range | ±500 dps |
| Mag rate | 100 Hz |

---

## 2. Firmware — `arduino/imu_debug/imu_debug.ino`

Main debug/AHRS sketch. Streams a CSV line per loop (~50–60 Hz):

```
time_ms,roll_deg,pitch_deg,yaw_deg,magYaw_deg,mx,my,mz,gz,dt
```
- `roll/pitch/yaw_deg` — fused attitude (deg)
- `magYaw_deg` — raw tilt-compensated magnetic heading (deg, 0–360), **not** fused
- `mx,my,mz` — calibrated magnetometer in **body frame** (post-remap)
- `gz` — bias-corrected gyro Z (rad/s); `dt` — loop time (s)

### Sensor fusion
- **Startup gyro bias:** averages 500 samples held still.
- **Roll/pitch:** complementary filter, `alpha = 0.98` — gyro integration
  blended with accel gravity vector (`accRoll = atan2(-ay, az)`,
  `accPitch = atan2(ax, sqrt(ay²+az²))`).
- **Yaw:** gyro integration nudged toward the tilt-compensated magnetic heading
  with a small gain, `beta = 0.005`.
- **Tilt compensation:** standard Freescale-style formula
  (`bx = mx·cp + my·sr·sp + mz·cr·sp`, `by = my·cr − mz·sr`,
  `magYaw = atan2(−by, bx)`). This math was always correct — the bug was the
  mag frame feeding it (see §3).

### Magnetometer calibration (applied in firmware)
`m_cal = MAG_A · (m_raw − MAG_B)` in the **native** sensor frame, then remapped
to body frame.

```c
const float MAG_B[3] = { -13.5353f, -31.0572f, 75.9761f };   // hard iron
const float MAG_A[3][3] = {                                   // soft iron (3x3)
  {  0.9747f, -0.0153f,  0.0226f },
  { -0.0153f,  1.0855f, -0.0771f },
  {  0.0226f, -0.0771f,  0.9614f }
};
```
Source: 100%-sphere-coverage capture (axis ratio 1.22, off-diagonal 7.7%).
Regenerate with `mag_calibrate.py` (see §5).

---

## 3. The key fix — magnetometer frame alignment

**Symptom:** heading (yaw) changed when the board was rolled, and drifted after
returning to level.

**Root cause:** the AK09916 magnetometer axes are oriented differently from the
ICM-20948 accel/gyro axes, and the Adafruit library reports the mag in its own
native frame. Roll/pitch (from accel) were being applied to mis-aligned mag
axes in the tilt-comp formula, so heading moved with roll.

**Resolution (data-driven, not datasheet lore):** the generic "swap X/Y, negate
Z" remap was **wrong** for this board's mounting. We logged a manoeuvre and
brute-forced all 48 signed axis permutations, scoring each by how constant the
horizontal field `H` stayed under roll. The winner:

```c
float mx =  mx_s;   // body X (nose)  = native X
float my = -my_s;   // body Y (right) = native Y, negated
float mz =  mz_s;   // body Z (up)    = native Z
```
i.e. **negate Y only.**

**Verification (heading-independent test):** de-rotating the body mag by
roll/pitch should leave the horizontal field `H` and vertical `V` constant.

| Mapping | H vs roll (µT/deg) | H CV |
|---|---|---|
| Old (swap X/Y, −Z) | **0.766** (broken) | 25.3% |
| Final (−Y only) | **0.006** | 2.9% |

> If a flat 360° spin ever shows heading running **backwards**, negate all three
> mag lines (the tilt-equivalent "candidate B"); tilt-invariance is unaffected.

---

## 4. I²C stall detection & recovery

The ICM-20948 reads its magnetometer over an internal I²C master, which can
**hang during long runs** — after which `getEvent()` returns stale, byte-identical
values forever (observed freezing the whole chip ~2–3 min in). Both sketches now
detect and recover:

- **`imu_debug.ino`:** a live gyro is never byte-identical across reads; 20
  consecutive identical gyro samples (~0.4 s) ⇒ stall ⇒ re-init IMU. Prints
  `# sensor stall detected, re-initialising IMU`.
- **`mag_capture_raw.ino`:** reads with magnitude `< 1 µT` are treated as
  stalled, skipped (never logged — zeros would poison the fit), and after 10 bad
  reads the IMU is re-initialised.

This is a software band-aid for a known silicon/firmware quirk. If stalls become
*frequent*, the hardware suspects are I²C signal integrity (wire length,
pull-ups) and VDD decoupling — not yet needed in practice.

---

## 5. Tooling & calibration workflow

| File | Purpose |
|---|---|
| `arduino/imu_debug/imu_debug.ino` | Main AHRS sketch (CSV stream) |
| `arduino/mag_capture_raw/mag_capture_raw.ino` | Raw mag logger (`mx,my,mz`) with stall recovery |
| `python/calibration/mag_calibration.py` | Live capture: logs `mag_log.csv`, shows XY/YZ/XZ scatter; drops invalid reads |
| `python/calibration/mag_calibrate.py` | Ellipsoid fit → hard iron + 3×3 soft iron; reports CV / axis ratio / coverage; emits C constants |
| `python/tools/imu_logger.py` | Headless logger of the full `imu_debug` CSV → `data/raw/imu_log_*.csv` (for long manoeuvres) |
| `python/imu_debug/imu_view_csv.py` | Live 3D board visualizer |

### Recalibration steps
1. Flash `mag_capture_raw.ino`.
2. Run `python/calibration/mag_calibration.py`; rotate the board through **all**
   orientations (full 360° spins at many tilts, incl. nose up/down) for ~60–120 s.
   Stay away from laptops, phones, steel desks.
3. Run `python/calibration/mag_calibrate.py mag_log.csv --plot`.
   Targets: **coverage > 90%, axis ratio < ~1.5, CV < 5%**.
4. Paste the printed `MAG_B` / `MAG_A` block into `imu_debug.ino`, re-flash.
5. Calibrate **in flight configuration** (battery/wiring) for best results — mag
   distortion is dominated by nearby electronics.

### Functional test (the real verdict)
- **Tilt-invariance:** nose on a fixed heading, roll ±30–45° → `magYaw_deg`
  stays constant. Easiest rig: hinge on a table edge (locks heading).
- **Heading-independent analysis:** capture with `imu_logger.py`, then check that
  horizontal field `H` is flat vs roll/pitch (how the §3 fix was verified).

---

## 6. Visualizer — `python/imu_debug/imu_view_csv.py`

Live 3D board with body axes, yellow nose arrow, and a title readout of
`roll / pitch / yaw / magYaw / packets`. All orientation conventions are
**display-only** named constants at the top (none affect firmware or calibration):

```python
FORWARD_SIGN = -1   # nose/tail (front-back)
ROLL_SIGN    =  1   # left/right banking
PITCH_SIGN   =  1   # nose up/down
YAW_SIGN     =  1   # heading turn direction
```
Flip any single sign if that axis appears reversed on screen. Current values are
tuned so the rendered board matches the physical board.

---

## 7. Calibration quality (current)

- Total field `|m|` CV in flight-style data: **1.9%** (excellent).
- Ellipsoid axis ratio: **1.22** (near-spherical).
- Soft-iron off-diagonal: **7.7%** of diagonal (mild tilt).
- Tilt compensation: `dH/droll ≈ 0.006 µT/deg` (roll-invariant).

> Note: the calibration *capture* reported ~8.8% CV; the lower 1.9% on later data
> confirms that residual was **environmental** (moving through a non-uniform
> field during capture), not a fit limitation.

---

## 8. Known limitations / future work

- Yaw still relies on a simple complementary nudge (`beta`), not a full EKF;
  fine for now. A Mahony/Madgwick or EKF would improve dynamic accuracy.
- I²C stall recovery is reactive; root-cause hardening (decoupling, wiring)
  untested since stalls are currently rare.
- `mag_capture.ino` is a standalone demo with a different tilt-comp variant and
  no axis remap — do **not** use it as a heading reference.
- Calibration done on the bench; redo in final flight configuration before
  trusting heading in the assembled drone.
