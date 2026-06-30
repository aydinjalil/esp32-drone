# Manual Serial Flight Control — Design Spec

_Date: 2026-06-30_
_Target: `drone_flight_controller/drone_flight_controller.ino` (classic ESP32 DevKit)_

## Context

The flight controller is validated on hardware: IMU (ICM-20948 @ 0x69), BMP581
baro, serial arming + pre-arm checks, attitude PID + X-mixing, and a fixed altitude
estimator (`h`/`v` from a 2-state Kalman filter) all work on the bench. But the
pilot inputs are still **simulated** for bench testing:

```c
throttle    = 0.55f + 0.1f * sin(now * 0.001f);  // simulated hover+climb
target_roll = 5.0f * sin(now * 0.002f);          // simulated ±5°
target_pitch= 5.0f * cos(now * 0.0015f);
```

To actually fly the drone, these must be replaced with real pilot input. There is
no RC receiver wired yet (the operator has a transmitter but wants serial control
first). This spec covers a **safe, incremental serial manual-flight interface**.

## Goals

- Replace the simulated throttle with **pilot-driven manual throttle** over serial.
- Hold the drone **level** (roll/pitch targets = 0) via the existing attitude PID.
- Make all throttle changes **incremental and bounded** so no single command (or a
  bad sensor zero, or a typo) can cause a sudden large throttle.

## Non-goals (deferred)

- Altitude-hold controller (the `h`/`v` Kalman output stays reference-only; hold
  becomes a follow-on toggle).
- RC / SBUS input (serial is the interim control surface).
- Yaw control (no yaw term in the mixer yet; out of scope).
- Absolute or fixed altitude setpoints (explicitly rejected: a wrong sensor zero or
  a typed value could command an aggressive climb).

## Design

### Throttle
- New global `float manualThrottle = 0.0f;` — pilot throttle, range `0 .. THR_MAX`.
- In `loop()`, replace the three simulated lines with:
  ```c
  throttle     = manualThrottle;
  target_roll  = 0.0f;
  target_pitch = 0.0f;
  ```
- `throttle` continues through the existing `computeSafeThrottle()` (keeps the
  `MAX_ALTITUDE` ceiling backstop) and `motorMixing()`.

### Attitude
- Targets fixed at 0 → PID self-levels. Yaw untouched (not mixed).

### Command set (extends existing `a`/`d`/`k`/`r` in `processCommands()`)
| Key        | Action                                        |
|------------|-----------------------------------------------|
| `+` / `=`  | `manualThrottle += THR_STEP` (clamp ≤ THR_MAX)|
| `-` / `_`  | `manualThrottle -= THR_STEP` (clamp ≥ 0)      |
| `0` / space| `manualThrottle = 0` (quick idle)             |
| `a`        | arm (existing pre-arm checks) **+ throttle→0**|
| `d` / `k`  | disarm / kill (existing) **+ throttle→0**     |
| `r`        | reset kill (existing)                         |

### Constants (tunable defaults)
```c
const float THR_STEP = 0.02f;  // 2% per keypress
const float THR_MAX  = 0.75f;  // early-test ceiling; raise after validation
```

## Safety properties

- Throttle is **0 at arm** and **0 on disarm/kill** — no surprise spin-up.
- Throttle only moves in `THR_STEP` (2%) increments and is bounded by `THR_MAX`
  (satisfies the "no sudden throttle" requirement).
- `k` (kill) forces motors idle instantly, as today; motors remain gated behind
  `armed && !killed`.
- `computeSafeThrottle()` altitude ceiling remains as an independent backstop.

## Verification (bench, props OFF, USB only)

1. Flash; boot clean (baro zero, "ToF absent", gravity cal, no IMU FAIL).
2. `a` to arm (level + still) → `state:ARM`, `thr:0.000`, motors `0.00`.
3. `+` several times → `thr` rises in 0.02 steps, motor values rise together; the
   four values stay balanced while level (self-leveling), and redistribute when the
   board is tilted (PID correcting).
4. `-` lowers `thr`; `0`/space snaps `thr` to 0 immediately.
5. `+` past 0.75 → `thr` clamps at 0.75 (THR_MAX).
6. `k` → `state:KILL`, `thr:0.000`, motors idle; `d` → `state:OFF`, throttle 0.
7. Confirm `thr` is 0 immediately after `a` (not carried over from before disarm).

(Motors won't physically spin on USB — ESCs unpowered. Powered test is a separate
milestone pending USB-vs-PDB power delivery.)

## Code touch points

- Globals: add `manualThrottle`, `THR_STEP`, `THR_MAX`.
- `processCommands()`: add `+`/`-`/`0`/space handling; zero throttle on arm/disarm/kill.
- `loop()`: replace the three simulated pilot-input lines.
- Telemetry already prints `thr` (= `safeThrottle`); no format change required.
