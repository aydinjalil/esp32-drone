// Raw magnetometer logger for calibration — Bluetooth edition.
//
// Same as mag_capture_raw.ino but streams the "mx,my,mz" CSV lines over
// Bluetooth Classic (SPP) as well as USB, so the drone can be carried away
// from the laptop (and its magnetic interference) on battery power during the
// calibration sweep. The previous on-drone cal (CV 8.37%) was laptop-limited.
//
// Workflow:
//   1) Flash this sketch over USB (/dev/cu.usbserial-0001), then UNPLUG USB.
//   2) Power the drone from the battery (one power source only — never both).
//   3) On the Mac:  python mag_calibration.py --port /dev/cu.DRONE_FC
//      (advertises the same "DRONE_FC" name as the FC, so the existing macOS
//      pairing and serial port are reused).
//   4) Carry the drone a few meters from the laptop/desk and slowly rotate it
//      through ALL orientations for ~60 s: figure-8s plus full turns about
//      each axis. Glance at the scatter plots for sphere coverage gaps.
//   5) Close the logger, then:  python mag_calibrate.py mag_log.csv --plot
//
// Notes:
//   - Values are in the AK09916 native sensor frame (NOT remapped to body
//     frame). The fusion code applies the body-frame remap AFTER calibration,
//     so calibration must stay in the native frame too.
//   - Calibrate in flight configuration: battery strapped in its real (under-
//     frame) position, ESCs powered. ESCs will beep without a signal — that's
//     harmless, motors cannot spin with no PWM input.
//   - BT writes block if the link congests; acceptable here (cal tool, not
//     flight code) — worst case the stream pauses until the client drains.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>
#include "BluetoothSerial.h"

Adafruit_ICM20948 imu;
BluetoothSerial SerialBT;

// On the ICM-20948 the AK09916 magnetometer is read through the chip's internal
// I2C master into shadow registers. That aux-I2C path can stall during long
// continuous polling, after which the magnetometer reads 0,0,0 forever. We
// detect that (a run of invalid reads) and re-initialise the IMU to recover.
const float MAG_MIN_VALID_UT = 1.0f;
const int   MAX_BAD_READS    = 10;   // ~0.2 s at 50 Hz before we re-init

int badReads = 0;

bool initIMU() {
  if (!imu.begin_I2C(0x69, &Wire)) return false;   // SDO/AD0 wired HIGH -> 0x69
  imu.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  return true;
}

// Every line goes to both interfaces; BT only when a client is connected.
void emit(const char* line) {
  Serial.print(line);
  if (SerialBT.hasClient()) SerialBT.print(line);
}

void setup() {
  Serial.begin(115200);
  delay(2000);   // do not block on Serial — this runs untethered on battery

  SerialBT.begin("DRONE_FC");   // same name as the FC -> existing macOS pairing

  Wire.begin(21, 22);
  Wire.setClock(100000);

  if (!initIMU()) {
    // Keep retrying instead of halting: on battery there is no serial monitor
    // to see a one-shot failure message, but BT lets us report it live.
    while (!initIMU()) {
      emit("# IMU FAIL, retrying\n");
      delay(1000);
    }
  }
  emit("# mag capture ready\n");
}

void loop() {
  sensors_event_t mag;
  imu.getMagnetometerSensor()->getEvent(&mag);

  float mx = mag.magnetic.x;
  float my = mag.magnetic.y;
  float mz = mag.magnetic.z;
  float magnitude = sqrtf(mx * mx + my * my + mz * mz);

  if (magnitude < MAG_MIN_VALID_UT) {
    // Invalid / stalled read: never emit it (zeros poison the calibration fit).
    if (++badReads >= MAX_BAD_READS) {
      // Comment line: 1 field, so the Python logger/parsers ignore it.
      emit("# mag stall detected, re-initialising IMU\n");
      initIMU();
      badReads = 0;
    }
    delay(20);
    return;
  }

  badReads = 0;

  // Raw, uncalibrated magnetometer in the native sensor frame (uT).
  char line[48];
  snprintf(line, sizeof(line), "%.4f,%.4f,%.4f\n", mx, my, mz);
  emit(line);

  delay(20);  // ~50 Hz; plenty for a hand-rotated calibration sweep
}
