# Custom Flight Controller PCB — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Note — this is a HARDWARE/PCB plan, not software.** There is no unit-test
> cycle. The verification gate at the end of each task is the EDA/hardware
> equivalent: **ERC** (Electrical Rules Check), **DRC** (Design Rules Check),
> **DFM** (JLCPCB manufacturability), schematic/layout **review against the
> reference board**, and finally **bench bring-up**. "Commit" means committing
> the KiCad project files / generated outputs to git.

**Goal:** Design a manufacturable STM32H743 flight-controller PCB (runs ArduPilot/PX4) to replace the ESP32 prototype, ready to order from JLCPCB PCBA.

**Architecture:** Single 36×36 mm, 4-layer FC board anchored to a proven ArduPilot/PX4 H743 reference design. Sensors on SPI/I²C, DShot motor outputs, SBUS RC in, GPS + TELEM(Jetson) UARTs, blackbox SD, powered from the existing PDB's 5 V. FC-only (existing PDB/ESCs retained).

**Tech Stack:** KiCad 8 (schematic + layout), JLCPCB (fab + PCBA), LCSC (parts), ArduPilot or PX4 (firmware target). Spec: `docs/superpowers/specs/2026-06-29-custom-flight-controller-pcb-design.md`.

## Global Constraints

- MCU: **STM32H743VIT6** (LQFP-100, 2 MB flash) — must match a real ArduPilot/PX4 board target.
- Board: **36 × 36 mm**, **4-layer**, **30.5 × 30.5 mm** M3 mounting (fits frame central bay).
- Power in: **5 V from PDB**; onboard 3.3 V; 6S-safe VBAT divider (1:11). No battery-voltage buck on-board.
- Logic level **3.3 V**; ESC outputs **DShot** (digital).
- Assembly: **JLCPCB PCBA**, prefer **LCSC "Basic"/in-stock "Extended"** parts; passives **0402** min.
- Confirmed-supported sensor: **ICM-20948**. BMP581 / VL53L4CX support **must be verified** before layout.
- No onboard RF (RC = external R-XSR; telemetry = external/Jetson).

---

### Task 1: Lock the reference board + firmware target, resolve sensor-driver support

**Files:**
- Create: `docs/superpowers/hardware/reference-target.md`
- Create: `docs/superpowers/hardware/sensor-support.md`

**Interfaces:**
- Produces: the chosen ArduPilot/PX4 board name to clone, its MCU/peripheral map, and the confirmed sensor list (with any swaps) consumed by all later schematic tasks.

- [x] **Step 1: Pick the reference board.** → **MatekH743** (`MATEKH743` hwdef). Recorded in `reference-target.md` with full peripheral map + deviations.
- [x] **Step 2: Verify sensor drivers.** ICM-20948 ✅ (Invensensev2), BMP581 ✅ (I²C), VL53L4CX ❌. Recorded in `sensor-support.md`.
- [x] **Step 3: Decide fallbacks.** VL53L4CX → **VL53L1X**; baro moved to I²C. Locked sensor list recorded.
- [x] **Step 4: Review gate.** Spec sensor table, architecture block, and open questions updated; plan Task 5 updated (baro on I²C, ToF = VL53L1X).
- [x] **Step 5: Commit.** (done — see git history)

---

### Task 2: KiCad project + library setup

**Files:**
- Create: `hardware/fc/fc.kicad_pro`, `hardware/fc/fc.kicad_sch`, `hardware/fc/fc.kicad_pcb`
- Create: `hardware/fc/README.md` (build/fab instructions)
- Create: `hardware/fc/lib/` (project symbol/footprint libs as needed)

**Interfaces:**
- Produces: an empty but configured KiCad 8 project with JLCPCB-friendly design rules, consumed by all schematic/layout tasks.

- [ ] **Step 1: Create the KiCad project** `hardware/fc/` with `fc.kicad_pro/sch/pcb`.
- [ ] **Step 2: Add design rules** for JLCPCB 4-layer (JLC04161H-7628 stackup): min trace/space 0.127 mm (5 mil) working target 0.15 mm, min via 0.3/0.6 mm, min annular ring 0.13 mm. Record in `README.md`.
- [ ] **Step 3: Add libraries** — install/link symbol+footprint libs for STM32H743VIT6, ICM-20948, BMP581/DPS310, VL53L4CX (or module connector), JST-GH, USB-C, regulators. Prefer parts with LCSC numbers.
- [ ] **Step 4: Verify** the project opens, ERC runs clean on the empty sheet, design rules load.
- [ ] **Step 5: Commit.**
```bash
git add hardware/fc/
git commit -m "hw: scaffold KiCad project with JLCPCB 4-layer design rules"
```

---

### Task 3: Schematic — MCU core (H743, clocks, USB, boot/debug)

**Files:**
- Modify: `hardware/fc/fc.kicad_sch` (MCU sheet)

**Interfaces:**
- Produces: net labels `MCU_3V3`, `GND`, `USB_DP`/`USB_DM`, `SWDIO`/`SWCLK`, `NRST`, `BOOT0`, and the SPI/I²C/UART/timer pin nets used by later tasks (named per the reference `hwdef`).

- [ ] **Step 1: Place STM32H743VIT6**; wire all VDD/VSS pairs with 100 nF each + 4.7 µF bulk; VCAP1/VCAP2 with 2.2 µF; VBAT to 3.3 V; VDDA/VREF+ to the filtered analog rail (ferrite + 1 µF/100 nF).
- [ ] **Step 2: Clocks** — 8 MHz HSE crystal with 2× load caps (per crystal datasheet) on PH0/PH1; optional 32.768 kHz LSE for RTC on PC14/PC15.
- [ ] **Step 3: USB** — PA11/PA12 to USB-C D−/D+, 22 Ω series optional, native USB (no serial chip); VBUS sense divider to a GPIO.
- [ ] **Step 4: Boot/debug** — BOOT0 with pulldown + test pad; NRST with 100 nF + button/pad; SWDIO/SWCLK to a debug header.
- [ ] **Step 5: Assign peripheral pins** exactly per the chosen `hwdef` (SPI1 IMU+baro, I²C2 compass+ToF, UARTs for SBUS/GPS/TELEM/spare, timer pins for 4× DShot, ADC pins for VBAT/current). Label nets accordingly.
- [ ] **Step 6: ERC** the MCU sheet → 0 errors (unconnected-pin warnings acceptable only for deliberately-unused pins, documented).
- [ ] **Step 7: Commit.**
```bash
git add hardware/fc/fc.kicad_sch
git commit -m "hw: schematic - STM32H743 core, clocks, USB, boot/debug"
```

---

### Task 4: Schematic — power tree

**Files:**
- Modify: `hardware/fc/fc.kicad_sch` (power sheet)

**Interfaces:**
- Consumes: `MCU_3V3`, analog rail, `GND`.
- Produces: nets `PDB_5V`, `USB_5V`, `SYS_5V`, `3V3`, `VBAT_SENSE`, `CURR_SENSE`.

- [ ] **Step 1: Input protection** — `PDB_5V` through reverse-polarity P-MOSFET (e.g., DMP3017SFG) + TVS (SMAJ5.0A) + 10 µF bulk + ferrite to `SYS_5V`.
- [ ] **Step 2: USB/PDB ORing** — `USB_5V` and `PDB_5V` ORed into `SYS_5V` via Schottky (e.g., SS14) or ideal-diode controller, so the two 5 V sources can't fight (fixes the devkit port-drop).
- [ ] **Step 3: 3.3 V LDO** — low-noise 1 A LDO (e.g., AP7361C-33 / TLV75533) `SYS_5V`→`3V3`, with 1 µF in / 10 µF out + 100 nF; derive the filtered analog rail off `3V3`.
- [ ] **Step 4: Sense dividers** — `VBAT` (raw pack tap from PDB) through 1:11 divider (e.g., 10 k / 1 k) + 100 nF → `VBAT_SENSE`; PDB current output through RC (1 k / 100 nF) → `CURR_SENSE`.
- [ ] **Step 5: ERC** the power sheet → 0 errors; confirm no rail shorts, every IC has decoupling.
- [ ] **Step 6: Commit.**
```bash
git add hardware/fc/fc.kicad_sch
git commit -m "hw: schematic - power tree (protection, ORing, 3V3, sense)"
```

---

### Task 5: Schematic — sensors (IMU SPI, baro, compass/ToF I²C)

**Files:**
- Modify: `hardware/fc/fc.kicad_sch` (sensor sheet)

**Interfaces:**
- Consumes: `3V3`, analog rail, `GND`, SPI1 nets (`SPI1_SCK/MISO/MOSI`), `IMU_CS`, `IMU_INT`, I²C2 nets (`I2C2_SCL/SDA`), `TOF_XSHUT`.
- Produces: populated sensor nets; the alternate-IMU footprint pads.

> **Task 1 result:** BMP581 is an **I²C** driver (not SPI); ToF is **VL53L1X**
> (VL53L4CX has no ArduPilot driver). IMU stays SPI1 (Invensensev2).

- [ ] **Step 1: ICM-20948** on SPI1 with `IMU_CS`, `IMU_INT` to a timer-capable GPIO, 100 nF decoupling, mode-select pins strapped for SPI.
- [ ] **Step 2: Alternate IMU site** — place an **ICM-42688-P** footprint sharing SPI1 + `IMU_CS`/`IMU_INT` (DNP now; this matches the native MATEKH743 SPI1 IMU line), so a future swap is populate-only. Document the do-not-populate set.
- [ ] **Step 3: Baro** — **BMP581 on the I²C2 bus** (addr 0x46) with 100 nF decoupling (no `BARO_CS`).
- [ ] **Step 4: Compass + ToF** — I²C2 (`SCL/SDA`) with 4.7 k pullups to the GPS-GH connector (compass) and to the ToF-GH connector for the **VL53L1X**; `TOF_XSHUT` to a GPIO.
- [ ] **Step 5: ERC** the sensor sheet → 0 errors.
- [ ] **Step 6: Commit.**
```bash
git add hardware/fc/fc.kicad_sch
git commit -m "hw: schematic - IMU(+alt), baro, compass/ToF buses"
```

---

### Task 6: Schematic — I/O, connectors, indicators

**Files:**
- Modify: `hardware/fc/fc.kicad_sch` (I/O sheet)

**Interfaces:**
- Consumes: timer DShot nets, UART nets (`SBUS_RX`, `GPS_TX/RX`, `TELEM_TX/RX/CTS/RTS`, `SPARE_*`), SDMMC nets, `3V3`, `SYS_5V`, `GND`, indicator GPIOs.
- Produces: all external connectors and their pinouts.

- [ ] **Step 1: DShot outputs** — 4× motor signal pads from the timer group (+ `SYS_5V`/`GND`), series 33 Ω optional, grouped for a clean ESC harness.
- [ ] **Step 2: RC input** — SBUS pad/GH on `SBUS_RX` (UART with HW inversion configured in firmware) + `SYS_5V`/`GND`.
- [ ] **Step 3: GPS-GH** (6-pin: `GPS_TX/RX`, `I2C2_SCL/SDA`, `SYS_5V`, `GND`) — Pixhawk-standard.
- [ ] **Step 4: TELEM-GH (Jetson)** (6-pin: `TELEM_TX/RX/CTS/RTS`, `SYS_5V`, `GND`) — MAVLink link; note Jetson is self-powered, 5 V pin optional/0 Ω.
- [ ] **Step 5: microSD** on SDMMC1 (1- or 4-bit) with pullups + 100 nF; or SPI fallback if layout demands.
- [ ] **Step 6: Indicators** — WS2812 on a GPIO (with level note), passive buzzer via NPN/MOSFET driver on a GPIO, status LED on `3V3`.
- [ ] **Step 7: Spare UART/I²C/GPIO** broken out to a small pad header.
- [ ] **Step 8: Full-design ERC** → 0 errors; assign all reference designators; annotate.
- [ ] **Step 9: Commit.**
```bash
git add hardware/fc/fc.kicad_sch
git commit -m "hw: schematic - DShot, RC, GPS, TELEM, SD, indicators, connectors"
```

---

### Task 7: Footprints, netlist, BOM (LCSC/JLCPCB)

**Files:**
- Modify: `hardware/fc/fc.kicad_sch`, `hardware/fc/fc.kicad_pcb`
- Create: `hardware/fc/bom.csv`

**Interfaces:**
- Produces: a complete netlist into the PCB and a BOM with LCSC part numbers consumed by Task 9 (fab outputs).

- [ ] **Step 1: Assign footprints** to every symbol; map each to an **LCSC part number** (prefer Basic/in-stock). For each, record JLCPCB assemblability.
- [ ] **Step 2: Update PCB from schematic** (load netlist); confirm all parts + ratsnest appear.
- [ ] **Step 3: Export BOM** to `bom.csv` with columns: Designator, Value, Footprint, LCSC#, JLC type (Basic/Extended).
- [ ] **Step 4: Verify** — 0 unassigned footprints; every line has an LCSC#; cross-check the BMP581/VL53L4CX choices against Task 1.
- [ ] **Step 5: Commit.**
```bash
git add hardware/fc/fc.kicad_sch hardware/fc/fc.kicad_pcb hardware/fc/bom.csv
git commit -m "hw: assign footprints + LCSC BOM, load netlist into PCB"
```

---

### Task 8: PCB layout — stackup, placement, routing

**Files:**
- Modify: `hardware/fc/fc.kicad_pcb`

**Interfaces:**
- Consumes: netlist + BOM.
- Produces: a fully routed 4-layer board meeting the form-factor + isolation constraints.

- [ ] **Step 1: Board outline** 36×36 mm, 4× M3 holes on 30.5×30.5, corner radius; set 4-layer stackup (Sig/GND/PWR/Sig).
- [ ] **Step 2: Placement** — H743 central; **IMU near board center / mount holes**, away from DShot + power; USB-C and connectors on edges; crystal close to MCU; decoupling caps adjacent to their pins.
- [ ] **Step 3: Power/ground** — solid GND on layer 2, power pours on layer 3; stitch vias; star the analog rail; keep `VDDA` quiet.
- [ ] **Step 4: Route** — length-reasonable SPI; USB D± as a 90 Ω differential pair; DShot signals direct; sense traces away from switching nodes.
- [ ] **Step 5: Add soft-mount grommet clearance** at the 4 mount holes; silkscreen labels for every connector + pin 1.
- [ ] **Step 6: DRC** → 0 errors against the JLCPCB ruleset.
- [ ] **Step 7: Commit.**
```bash
git add hardware/fc/fc.kicad_pcb
git commit -m "hw: 4-layer layout - placement, planes, routing, DRC clean"
```

---

### Task 9: Fab outputs + design review + order

**Files:**
- Create: `hardware/fc/fab/` (Gerbers, drill, `bom.csv`, `cpl.csv` pick-and-place)
- Create: `hardware/fc/fab/REVIEW.md`

**Interfaces:**
- Consumes: routed PCB + BOM.
- Produces: a manufacturing package + a recorded review, ready to upload to JLCPCB.

- [ ] **Step 1: Generate** Gerbers + drill (JLCPCB layer mapping), `cpl.csv` (placement), and final `bom.csv`.
- [ ] **Step 2: DFM** — run the JLCPCB Gerber viewer / DFM check; resolve any flags. Record in `REVIEW.md`.
- [ ] **Step 3: Design review** against the spec + reference board: power tree, decoupling, IMU isolation, connector pinouts, DShot grouping, USB pair. Record sign-off checklist in `REVIEW.md`.
- [ ] **Step 4: Verify** — ERC + DRC both clean; BOM has no unsourceable parts; review checklist all ticked.
- [ ] **Step 5: Commit, then order** (~5 boards, PCBA).
```bash
git add hardware/fc/fab/
git commit -m "hw: fab outputs (gerbers/cpl/bom) + design review sign-off"
```

---

### Task 10: Bring-up plan (executed when boards arrive)

**Files:**
- Create: `hardware/fc/bringup.md`

**Interfaces:**
- Consumes: the assembled board.
- Produces: a verified, flashed, flyable FC.

- [ ] **Step 1: Power-on smoke test** — bench 5 V current-limited; verify 3.3 V rail, no shorts/heat; check VBAT/current sense readings.
- [ ] **Step 2: USB enumeration** — board enumerates over USB-C (DFU/native).
- [ ] **Step 3: Flash firmware** — load the chosen ArduPilot/PX4 target; confirm boot.
- [ ] **Step 4: Sensor detection** — ground-station shows IMU, baro, compass, ToF, GPS; calibrate IMU + compass.
- [ ] **Step 5: RC + outputs** — bind R-XSR (SBUS) and verify channels; **props off**, verify all 4 DShot motors + directions (reuse the bench-test discipline from the prototype).
- [ ] **Step 6: TELEM link** — connect Jetson over the TELEM-GH port; confirm MAVLink heartbeat.
- [ ] **Step 7: First flight** — on the existing frame; record in `bringup.md`.
- [ ] **Step 8: Commit.**
```bash
git add hardware/fc/bringup.md
git commit -m "hw: bring-up results and first-flight log"
```

---

## Self-Review

**Spec coverage:** MCU (T3) · power tree incl. USB/PDB ORing (T4) · sensors incl. alt-IMU site (T5) · DShot/RC/GPS/TELEM/SD/indicators (T6) · form factor 36×36/30.5 + IMU soft-mount (T8) · ArduPilot/PX4 target + sensor-driver verification (T1) · fab/PCBA (T9) · bring-up incl. Jetson TELEM (T10). All spec sections mapped.

**Placeholder scan:** No "TBD/handle errors" placeholders; fallbacks (DPS310/VL53L1X) are explicit and conditional on Task 1's verification.

**Type/name consistency:** Net names (`SYS_5V`, `3V3`, `VBAT_SENSE`, `CURR_SENSE`, `SPI1_*`, `IMU_CS/INT`, `BARO_CS`, `I2C2_*`, `SBUS_RX`, `GPS_*`, `TELEM_*`) are defined where first produced (T3/T4) and reused consistently downstream.

**Note:** Tasks 1–9 are design work (mine + KiCad); Task 10 is physical and runs only after boards arrive. KiCad 8 is a prerequisite for Tasks 2–9.
