// baro_test.ino — confirm the BMP581 actually READS (not just ACKs on the bus).
// Mirrors drone_flight_controller.ino: Adafruit_BMP5xx, TwoWire(0) on SDA=21/
// SCL=22, address 0x47, same oversampling/IIR/ODR config.
//
// Open Serial Monitor @ 115200. It ground-zeros pressure at boot, then streams:
//   pressure (hPa), temperature (C), relative altitude (m, 0 = where it booted).
// Lift the board ~0.5-1 m and the altitude should climb a few tenths of a meter
// (baro is noisy at this scale, but the trend should track up/down).

#include <Wire.h>
#include <math.h>
#include <Adafruit_BMP5xx.h>

#define SDA_PIN 21
#define SCL_PIN 22

TwoWire I2Cdev = TwoWire(0);
Adafruit_BMP5xx bmp;

float groundPressure = 0.0f;   // hPa, captured at boot

void setup() {
  Serial.begin(115200);
  delay(2000);                 // let USB CDC come up

  I2Cdev.begin(SDA_PIN, SCL_PIN);
  I2Cdev.setClock(100000);

  if (!bmp.begin(0x47, &I2Cdev)) {
    Serial.println("BMP FAIL — not reachable at 0x47. Check wiring/address.");
    while (1) delay(100);
  }

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  Serial.println("BMP581 OK. Settling 2s...");
  delay(2000);

  // Ground-zero: average 200 readings held still.
  Serial.println("Auto-zeroing (keep still)...");
  float total = 0; int n = 0;
  for (int i = 0; i < 200; i++) {
    if (bmp.performReading()) { total += bmp.pressure; n++; }   // hPa
    delay(10);
  }
  groundPressure = (n > 0) ? total / n : 0.0f;
  Serial.printf("Ground zero: %.2f hPa (%d samples)\n\n", groundPressure, n);
  Serial.println("pressure_hPa, temp_C, rel_alt_m");
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("read failed");
    delay(100);
    return;
  }

  float p = bmp.pressure;           // hPa
  float t = bmp.temperature;        // C
  float relAlt = 0.0f;
  if (groundPressure > 0.0f) {
    float ratio = p / groundPressure;
    relAlt = 44330.0f * (1.0f - powf(ratio, 0.1903f));   // m, 0 at ground
  }

  Serial.printf("%.2f, %.2f, %+.2f\n", p, t, relAlt);
  delay(100);                       // ~10 Hz print
}
