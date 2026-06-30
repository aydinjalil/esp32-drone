// i2c_scan.ino — bus scanner for the ESP32 drone sensor stack.
//
// Flash this, open Serial Monitor @ 115200, and read off which addresses
// actually respond. Use it to confirm IMU wiring (esp. the SDO/AD0 address
// pin) before trusting any sensor sketch.
//
// Same bus as the flight controller: SDA = GPIO21, SCL = GPIO22, 100 kHz.
//
// Expected devices on this drone:
//   0x68  ICM-20948 IMU   (SDO/AD0 = LOW / GND)
//   0x69  ICM-20948 IMU   (SDO/AD0 = HIGH / VDD)   <-- if you see THIS, the
//                                                       sketches need 0x69
//   0x47  BMP581 barometer
//   0x29  VL53L4CX ToF     (only after XSHUT is driven HIGH; see note below)

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define XSHUT_PIN 4   // ToF shutdown — held LOW keeps the ToF off the bus

const char* knownDevice(uint8_t addr) {
  switch (addr) {
    case 0x68: return "ICM-20948 IMU (SDO/AD0 LOW = 0x68)";
    case 0x69: return "ICM-20948 IMU (SDO/AD0 HIGH = 0x69)";
    case 0x47: return "BMP581 barometer";
    case 0x29: return "VL53L4CX ToF";
    default:   return "unknown device";
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);   // let USB CDC come up; do not block on while(!Serial)

  // Keep the ToF held in reset so it doesn't clutter the first scan.
  // Comment these two lines out if you WANT to see the ToF at 0x29.
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println();
  Serial.println("=== I2C scanner (SDA=21, SCL=22, 100kHz) ===");
  Serial.println("ToF held in reset (XSHUT LOW) — expect IMU + baro only.");
  Serial.println();
}

void loop() {
  uint8_t found = 0;

  Serial.println("Scanning 0x01..0x7E ...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.printf("  found 0x%02X  ->  %s\n", addr, knownDevice(addr));
      found++;
    } else if (err == 4) {
      Serial.printf("  bus error at 0x%02X\n", addr);
    }
  }

  if (found == 0) {
    Serial.println("  no devices found — check wiring, power, and pull-ups.");
  } else {
    Serial.printf("  %u device(s) responded.\n", found);
  }
  Serial.println();

  delay(3000);   // rescan every 3 s so you can wiggle wires and watch
}
