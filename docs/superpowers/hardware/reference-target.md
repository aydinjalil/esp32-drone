# Reference Board & Firmware Target

**Task 1 deliverable.** Locks the ArduPilot/PX4 board this PCB clones, so an
existing firmware target fits our hardware with minimal change.

## Chosen target: **MatekH743** (ArduPilot `hwdef` = `MATEKH743`)

- **MCU:** STM32H743xx (use **STM32H743VIT6**, LQFP-100, 2 MB flash), **8 MHz HSE**.
- **Why this one:** it's a mainstream, well-maintained H743 target whose peripheral
  map already matches almost everything we want — most importantly its **primary
  IMU on SPI1 is the ICM-42688** (our planned *future* IMU) and its baro is on
  **I²C**, which is exactly where our BMP581 driver lives. Huge UART count, proven
  DShot output groups. Runs on both ArduPilot and PX4.
- **hwdef source:** https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat
- **Docs:** https://ardupilot.org/copter/docs/common-matekh743-wing.html

## Reference peripheral map (verbatim from hwdef.dat)

```
MCU STM32H743xx, 8 MHz HSE

SPI1  IMU1_CS : icm42688 (Invensensev3) / mpu6000     <- PRIMARY IMU bus
SPI4  IMU2_CS : icm20602 / icm42605
SPI2  MAX7456_CS : OSD            (NOT NEEDED — drop)
SPI3  EXT_CS1 : pixartflow        (optical flow — not used)

I2C1  PB6/PB7
I2C2  PB10/PB11
BARO  MS5611 / DPS310 / BMP280  on I²C

SERIAL_ORDER OTG1 UART7 USART1 USART2 USART3 UART8 UART4 USART6 OTG2
  USART1 PA10/PA9  = Telem2     -> TELEM to Jetson (MAVLink)
  USART2 PD5/PD6   = GPS1       -> GPS
  USART6 PC7/PC6   = RC         -> SBUS (R-XSR, HW-inverted)
  UART4  PB9/PB8   = spare
  UART7  PE7/PE8   = Telem1     -> spare/2nd telem
  OTG1            = USB (native)

PWM/DShot motor outputs (use 1–4):
  PB0 TIM8_CH2N (M1)   PB1 TIM8_CH3N (M2)
  PA0 TIM5_CH1  (M3)   PA1 TIM5_CH2  (M4)
```

## Our deviations from the reference (→ requires a **custom hwdef**, forked from MATEKH743)

| Area | Reference | Ours | Change |
|---|---|---|---|
| Primary IMU | ICM-42688 (Invensensev3) on SPI1 | **ICM-20948 (Invensensev2)** on SPI1 | one `SPIDEV`+`IMU` line; **42688 alt-footprint reuses the reference's native line** for a future swap |
| Baro | MS5611/DPS310/BMP280 (I²C) | **BMP581 (I²C 0x46)** | one `BARO` line |
| Rangefinder | none onboard | **VL53L1X (I²C)** | param-configured, no hwdef change |
| OSD | MAX7456 on SPI2 | **dropped** (no analog FPV) | frees SPI2 |
| Power | 9–36 V input + BECs | **5 V from PDB** → 3.3 V | power tree only; not a firmware concern |
| TELEM | Telem2 = USART1 | **USART1 → Jetson MAVLink** | config only |

**Bring-up implication:** create a custom ArduPilot board dir (e.g.
`hwdef/DroneFC-H743/`) inheriting MATEKH743 and overriding the IMU + BARO lines and
the unused SPI2/OSD. This is standard ArduPilot custom-board practice.

## Firmware stack decision

Hardware supports **both ArduPilot and PX4** (deferred, decide at bring-up).
Lean **ArduPilot** first — the MATEKH743 target + Lua scripting is the fastest path
to a flying board with custom autonomy hooks. PX4 remains available on the same HW.
