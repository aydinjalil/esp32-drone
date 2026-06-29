# Custom Flight Controller PCB — Design Spec

**Date:** 2026-06-29
**Status:** Design approved (technical decisions delegated to engineering)
**Phase:** Sub-project 1 of a larger autonomous-drone platform

---

## Context

The drone currently runs a hand-built prototype: a classic ESP32 DevKit on a
perfboard, three Adafruit sensor breakouts, a separate PDB, and four ESCs driven
by analog PWM. It flies (motors verified, sensors pending), but it is not a
product — it is wires and modules.

The user wants to **productize** the drone, and the longer-term goal is an
autonomous platform that performs **object detection, mapping, delivery, and
scanning**. Those perception/AI workloads do **not** run on a flight-controller
MCU — they run on a **companion computer (NVIDIA Jetson Orin Nano)** that the user
already owns. The professional architecture is therefore two brains:

- **Flight Controller (this PCB):** real-time stabilization, attitude/position
  control, motor output, sensor fusion, failsafe.
- **Companion Computer (Jetson, later phase):** the user's own autonomy software,
  commanding the FC over MAVLink.

This spec covers **only the custom Flight Controller PCB** — the first concrete
deliverable — designed so the companion-computer/autonomy phases drop in cleanly.

## Goals

- A manufacturable, productized flight controller PCB to replace the
  ESP32/perfboard prototype.
- Runs a proven open-source flight stack (**ArduPilot or PX4**) so the user can
  layer their **own autonomous tasks** on top (Lua scripting, MAVLink offboard
  control from the Jetson, or firmware forks) without writing a flight stack from
  scratch. This is how "full control of the software" is satisfied: the user owns
  the autonomy layer; the flight core is battle-tested.
- Future-ready for the Jetson companion computer (TELEM/MAVLink link present now).
- Fits the **existing custom frame** — bolts to the central bay's 30.5×30.5
  heat-set inserts already installed.

## Non-Goals (this phase)

- The companion computer, cameras, and perception/autonomy software (later phases).
- An integrated PDB or ESCs — this is **FC-only**; the existing PDB stays.
- Any onboard radio design — the RC link and telemetry radios are external modules.

---

## Architecture

```
   USB-C ──────────┤ STM32H743 (480MHz M7, 2MB flash) ├────── Timers ─→ 4× DShot ESC
                   │  ArduPilot / PX4 · native USB     │
   SPI1 ─ IMU  ICM-20948  (+ alternate footprint/site for ICM-42688-P)
   SPI  ─ BMP581 baro
   SDMMC ─ microSD (blackbox flight logging)
   I²C  ─ external compass (in GPS unit) · VL53L4CX ToF (remote port)
   UARTs ─ SBUS RX (FrSky R-XSR, HW-inverted) · GPS · TELEM→Jetson (MAVLink) · 1 spare
   ADC  ─ VBAT sense · current sense
   Power ─ PDB 5V → onboard 3.3V; USB/PDB ORed; Jetson powered separately
   Extras ─ buzzer · WS2812 status LED · boot/status LEDs
```

### MCU — STM32H743

The de-facto autonomy-capable FC MCU. 480 MHz Cortex-M7, 2 MB flash, ~1 MB RAM —
enough headroom for full ArduPilot/PX4 with scripting. Native USB (no USB-serial
chip). Abundant peripherals (multiple SPI/I²C/UART, SDMMC, timers for DShot),
removing the pin-budget pressure the ESP32 hit. 8 MHz HSE crystal; optional
32.768 kHz LSE for RTC. Internal flash holds the firmware (no external flash).

### Firmware strategy

Anchor the hardware to a **proven ArduPilot/PX4 H743 reference design**
(Matek H743 / Holybro Kakute H7 class). Matching a known board means an existing
firmware target (ArduPilot `hwdef` / PX4 board config) already fits the hardware,
massively de-risking first flight. A well-designed H743 board runs **both**
ArduPilot and PX4 — the stack choice is deferred and is not locked by hardware.
Recommendation leans PX4 for a ROS 2 / vision-autonomy future, but ArduPilot is
equally viable (and Lua scripting is the gentlest custom-autonomy on-ramp).

### Sensors

| Sensor | Role | Bus | Notes |
|---|---|---|---|
| ICM-20948 | IMU (9-axis) | SPI1 | Supported by ArduPilot/PX4; reuses existing mag-cal know-how |
| ICM-42688-P | future IMU | SPI1 | Alternate footprint/site reserved; ~5× lower gyro noise; 6-axis (needs separate mag) |
| BMP581 | barometer | SPI | **Driver support to verify**; fallback DPS310 if not native |
| VL53L4CX | downward ToF | I²C (remote) | **Driver support to verify**; fallback VL53L1X; stays on the existing down-bracket |
| compass | heading | I²C | Lives in the external GPS unit |

**Open item:** verify ArduPilot/PX4 native driver support for **BMP581** and
**VL53L4CX**. If unsupported, either select a natively-supported equivalent
(DPS310 baro, VL53L1X ToF) or write/port a driver. ICM-20948 is confirmed
supported.

### Control link

FrSky **R-XSR** receiver (owned) → **SBUS** (inverted UART, 100 kbaud 8E2) on a
dedicated UART with **hardware inversion** (no external inverter). The TX is a
RadioMaster Pocket in FrSky **D16** mode. The receiver UART is kept
protocol-flexible (also handles non-inverted **CRSF**) so a future move to ELRS is
a firmware/jumper change. ArduPilot/PX4 support SBUS natively.

### Power tree

```
PDB 5V ──[reverse-polarity P-FET]──[TVS + ferrite + bulk cap]──┬── 5V RAIL
USB-C 5V ──[ORing Schottky / ideal-diode]──────────────────────┤    (R-XSR, GPS, buzzer, WS2812)
                                                                └─→ [low-noise 3.3V LDO ~1A]
                                                                       (H743 + sensors + filtered VDDA)
VBAT (raw pack from PDB) ─[1:11 divider]─→ ADC   (6S-safe)
PDB current output ───────[RC filter]────→ ADC
```

- **USB/PDB ORing:** the board takes 5 V from USB *or* the PDB through an ORing
  diode, so they can't fight. This fixes the **port-drops-when-battery-connected**
  contention seen on the devkit — bench USB and flight battery can coexist.
- **Clean sensor rail:** low-noise LDO + separately filtered VDDA for the ADC, so
  IMU/baro noise performance isn't compromised by supply noise.
- **Input protection:** reverse-polarity FET + TVS on the 5 V input.
- **Jetson is not on this tree** — it draws its own power from the PDB via its own
  regulator; it shares only **ground + the TELEM UART** with the FC.

### Connectors & I/O

- **JST-GH (Pixhawk standard)** for GPS, TELEM(Jetson), I²C(ToF/compass) — standard
  cables, plug-and-play Jetson link.
- **4× DShot ESC** signal pads (+ 5 V / GND).
- **SBUS RX** pad/connector (+ 5 V / GND).
- **Power input** pads: 5 V + GND from PDB; VBAT + current-sense taps.
- **USB-C** for configuration/flashing (native USB).
- **Buzzer** pads, **WS2812** LED pad, status LEDs.
- A **spare UART** broken out for expansion.

### Form factor & layout

- **36 × 36 mm**, **4-layer** PCB, **30.5 × 30.5 mm M3** mounting — bolts to the
  frame's existing central heat-set inserts; PDB sits beneath on standoffs.
- 4-layer stackup: Top (signal) / GND plane / power plane / Bottom (signal).
- **IMU placed near board center / mounting holes**, away from DShot and power
  traces; **soft-mount grommets** in the mounting holes isolate the gyro (the
  on-board analogue of the prototype's TPU pad).
- **No onboard RF** (RC + telemetry radios are external), so no antenna/RF layout
  constraints — a meaningful simplification.
- High motor current does **not** pass through the FC (signal only), keeping
  currents low and the layout simpler.

---

## Firmware implications (tracked, not built here)

- New flight stack: ArduPilot or PX4 replaces the custom ESP32 `.ino`. The custom
  prototype firmware becomes a learning artifact; the IMU-calibration insight
  carries forward conceptually.
- SBUS receiver parsing handled natively by the stack.
- Custom autonomy implemented via Lua (ArduPilot) / C++ modules (PX4) / MAVLink
  offboard from the Jetson — the user's "own tasks" layer.
- DShot replaces PWM (no throttle calibration / deadband).

## Verification & next steps

1. **Confirm reference board** to anchor to (ArduPilot/PX4 supported H743 target).
2. **Verify sensor driver support** (BMP581, VL53L4CX); pick fallbacks if needed.
3. Produce the **schematic** (KiCad), then **BOM** with LCSC/JLCPCB part numbers.
4. **Layout** the 4-layer board to the constraints above; design-review before fab.
5. Order a **small batch (~5)** via JLCPCB PCBA.
6. Bring up: power rails → USB enumeration → flash ArduPilot/PX4 target → sensor
   detection → bench motor test (DShot) → first flight on the existing frame.
7. Later phases: Jetson companion + perception + autonomy applications.

## Open questions

- ArduPilot vs PX4 (deferred — hardware supports both; decide at bring-up).
- BMP581 / VL53L4CX native driver support (verify; fallbacks identified).
- Exact reference board to clone for the firmware target.
