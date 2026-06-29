# Sensor Driver Support (ArduPilot) — Task 1 verification

Verified against the ArduPilot `master` source. Result: **two of three original
sensors keep, one swaps.**

## Findings

| Sensor | Role | ArduPilot support | Bus | Decision |
|---|---|---|---|---|
| **ICM-20948** | IMU (9-axis) | ✅ `AP_InertialSensor_Invensensev2` (probes on SPI) | **SPI1** | **KEEP** |
| **BMP581** | barometer | ✅ `AP_Baro_BMP581` — DEVTYPE 0x15, I²C 0x46/0x47, `BARO_PROBE_EXT` bit 13 | **I²C** | **KEEP** (note: I²C, not SPI) |
| **VL53L4CX** | downward ToF | ❌ no driver — only **VL53L0X** and **VL53L1X** exist | I²C | **SWAP → VL53L1X** |

### Details / sources
- **ICM-20948** — supported via the Invensense v2 backend; commonly used on SPI
  (e.g., Cube Yellow/Orange on SPI1). The internal AK09916 mag is usable in I²C
  bypass.
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp
- **BMP581** — `AP_Baro_BMP581.h` exists; selectable as an external **I²C** baro
  (addresses 0x46/0x47). So the baro belongs on the **I²C bus, not SPI** (this
  corrects the original spec, which had it tentatively on SPI).
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Baro/AP_Baro_BMP581.h
- **VL53L4CX** — **no** ArduPilot driver. Supported ST ToF parts are **VL53L0X**
  and **VL53L1X** only. The VL53L1X is the right substitute: longer range than the
  L0X, well-supported, I²C, same role.
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.cpp

## Locked sensor list for the PCB

- **IMU:** ICM-20948 on **SPI1** (Invensensev2). Alt footprint **ICM-42688-P** on
  the same SPI1/CS (native MATEKH743 line) for a future drop-in.
- **Baro:** BMP581 on **I²C** (addr 0x46).
- **ToF:** **VL53L1X** on **I²C** (was VL53L4CX) + XSHUT GPIO.
- **Compass:** AK09916 inside the ICM-20948 (I²C bypass), and/or external compass
  in the GPS unit.

## Design adjustments propagated from this finding

1. **Baro moves to I²C** (BMP581 driver is I²C). Spec sensor table + plan Task 5
   updated: BMP581 on the shared I²C bus, not SPI.
2. **ToF part change VL53L4CX → VL53L1X.** Spec + plan updated. The existing
   down-bracket (`tof_bracket.scad`, 20.1×12.4 M2 pattern) still fits the
   Adafruit VL53L1X breakout footprint family.

## PX4 note (deferred stack)
PX4 supports ICM-20948 and VL53L1X; BMP581 support is less certain in PX4 than in
ArduPilot. Since we lean ArduPilot first and BMP581 is confirmed there, this is a
non-issue for the chosen path; revisit only if switching to PX4.
