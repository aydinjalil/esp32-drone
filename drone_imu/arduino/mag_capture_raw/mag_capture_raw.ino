// Raw magnetometer logger for calibration.
//
// Streams UNCALIBRATED magnetometer readings as "mx,my,mz" CSV lines so they
// can be fitted offline by python/calibration/mag_calibrate.py.
//
// Workflow:
//   1) Flash this sketch.
//   2) Run python/calibration/mag_calibration.py (it logs mag_log.csv and shows
//      live XY/YZ/XZ scatter plots).
//   3) Slowly rotate the board through ALL orientations for ~30-60 s: figure-8
//      motions plus full turns about each axis. Aim for even coverage of the
//      sphere -- gaps in coverage produce a bad ellipsoid fit.
//   4) Close the logger, then run mag_calibrate.py on the captured CSV.
//
// Notes:
//   - Values are in the AK09916 native sensor frame (NOT remapped to body
//     frame). The fusion sketch applies the body-frame axis remap AFTER
//     calibration, so calibration must stay in the native frame too.
//   - I2C pins / clock match imu_debug.ino so the same wiring is used.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 imu;

// On the ICM-20948 the AK09916 magnetometer is read through the chip's internal
// I2C master into shadow registers. That aux-I2C path can stall during long
// continuous polling, after which the magnetometer reads 0,0,0 forever. We
// detect that (a run of invalid reads) and re-initialise the IMU to recover.
// A real read near a powered board is always well above this magnitude.
const float MAG_MIN_VALID_UT = 1.0f;
const int   MAX_BAD_READS     = 10;   // ~0.2 s at 50 Hz before we re-init

int badReads = 0;

bool initIMU() {
  if (!imu.begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire)) return false;
  imu.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  if (!initIMU()) {
    Serial.println("IMU FAIL");
    while (1) delay(10);
  }
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
      Serial.println("# mag stall detected, re-initialising IMU");
      initIMU();
      badReads = 0;
    }
    delay(20);
    return;
  }

  badReads = 0;

  // Raw, uncalibrated magnetometer in the native sensor frame (uT).
  Serial.printf("%.4f,%.4f,%.4f\n", mx, my, mz);

  delay(20);  // ~50 Hz; plenty for a hand-rotated calibration sweep
}
