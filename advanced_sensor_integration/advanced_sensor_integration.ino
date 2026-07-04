// Core Arduino libraries
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// Sensonr libraries
#include <Adafruit_BMP5xx.h>  
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>    // (later when I have my sensor replaces MPU)
#include <vl53l4cx_class.h> // (laser distance measurer)

// ================= I2C =================
#define XSHUT_PIN 4
TwoWire I2Cdev = TwoWire(0);

// PINS
#define SDA_PIN 21
#define SCL_PIN 22

// Optional Sensor
#include <Adafruit_SSD1306.h> // Display


// ======================================================
// TIME SYSTEM
// ======================================================
unsigned long lastMicros = 0;
float dt = 0.01f;

// ======================================================
// STATE (Kalman filter core state)
// ======================================================
// h = altitude (meters)
// v = vertical velocity (m/s)

float h = 0.0f;
float v = 0.0f;

// ======================================================
// STATE COVARIANCE (uncertainty matrix)
// ======================================================

float P00 = 1.0f, P01 = 0.0f;
float P10 = 0.0f, P11 = 1.0f;

// ======================================================
// IMU RAW DATA
// ======================================================

float ax, ay, az;
float gx, gy, gz;

float bias_gx = 0;
float bias_gy = 0;
float bias_gz = 0;

// ======================================================
// ORIENTATION (needed for gravity removal later)
// ======================================================

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float mx = 0.0f;
float my = 0.0f;
float mz = 0.0f;

// ======================================================
// GRAVITY-COMPENSATED ACCELERATION
// ======================================================

float a_z = 0.0f;

// ======================================================
// BAROMETER
// ======================================================
float pressure = 0.0f;
float baroAlt = 0.0f;
float groundPressure = 0.0f;

// ======================================================
// SENSOR NOISE PARAMETERS (Kalman tuning)
// ======================================================
float Qh = 0.02f; //altitude process noise
float Qv = 0.10f; // velocity process noise
float R_baro = 2.0f; // barometer measurement noise 

// ======================================================
// SENSOR OBJECTS
// ======================================================
// Adafruit_MPU6050 mpu;
Adafruit_ICM20948 imu;
Adafruit_BMP5xx bmp;
VL53L4CX tof(&I2Cdev, XSHUT_PIN);

// ======================================================
// OPTIONAL (future)
// ======================================================
float tofAlt = 0.0f; //VL53L4CX sensor


float magMinX =  10000;
float magMaxX = -10000;

float magMinY =  10000;
float magMaxY = -10000;

float magMinZ =  10000;
float magMaxZ = -10000;

// ======================================================
// MAGNETOMETER CALIBRATION (ICM-20948 / AK09916)
// Calibrated sample: m_cal = MAG_A * (m_raw - MAG_B) in the native sensor
// frame, then remapped to body frame (see readMagBody).
// Flight-config cal 2026-07-04 (BT capture on battery, under-frame battery,
// ESCs powered) — matches imu_debug.ino; see drone_imu/docs/ICM20948_STATUS.md.
// ======================================================
const float MAG_B[3] = { -15.5383f, 20.9900f, -16.2438f };
const float MAG_A[3][3] = {
  {  1.0340f,  0.0612f, -0.0023f },
  {  0.0612f,  0.9243f,  0.0192f },
  { -0.0023f,  0.0192f,  1.0615f }
};

// ======================================================
// IMU I2C STALL DETECTION
// The ICM-20948 reads its mag over an internal I2C master that can hang during
// long runs, after which getEvent() returns stale, byte-identical values. A
// live gyro is never identical across reads, so a run of identical samples
// means the bus stalled -> re-init to recover.
// ======================================================
const int STALE_LIMIT = 20;   // ~0.4 s of frozen reads before re-init
int staleCount = 0;
float prev_gx_raw = 1e9f, prev_gy_raw = 1e9f, prev_gz_raw = 1e9f;

// ================= HELPERS =================
bool isValidFloat(float x) {
  return !(isnan(x) || isinf(x));
}

bool initIMU() {
  if (!imu.begin_I2C(0x69, &I2Cdev)) return false;   // SDO/AD0 wired HIGH -> 0x69 (confirmed by i2c_scan)
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  return true;
}

// Read magnetometer, apply hard/soft-iron calibration in the native frame, then
// remap into the accel/gyro body frame. Empirically (from logged data) the
// correct remap for this board is "negate Y only": body = (+X, -Y, +Z).
void readMagBody(float &mxb, float &myb, float &mzb) {
  sensors_event_t mag;
  imu.getMagnetometerSensor()->getEvent(&mag);
  float cx = mag.magnetic.x - MAG_B[0];
  float cy = mag.magnetic.y - MAG_B[1];
  float cz = mag.magnetic.z - MAG_B[2];
  float mx_s = MAG_A[0][0] * cx + MAG_A[0][1] * cy + MAG_A[0][2] * cz;
  float my_s = MAG_A[1][0] * cx + MAG_A[1][1] * cy + MAG_A[1][2] * cz;
  float mz_s = MAG_A[2][0] * cx + MAG_A[2][1] * cy + MAG_A[2][2] * cz;
  mxb =  mx_s;   // body X (nose)
  myb = -my_s;   // body Y (right)
  mzb =  mz_s;   // body Z (up)
}

// ================= ORIENTATION =================
void updateOrientation() {

  float gx_corr = gx - bias_gx;
  float gy_corr = gy - bias_gy;
  float gz_corr = gz - bias_gz;

  // Roll + pitch: complementary filter (gyro + accel)
  if (fabs(gx_corr) <= 20 && fabs(gy_corr) <= 20) {
    roll  -= gx_corr * dt;
    pitch -= gy_corr * dt;

    float accRoll  = atan2(-ay, az);
    float accPitch = atan2(ax, sqrt(ay * ay + az * az));

    const float alpha = 0.98f;
    roll  = alpha * roll  + (1.0f - alpha) * accRoll;
    pitch = alpha * pitch + (1.0f - alpha) * accPitch;
  }

  // Yaw: gyro integration + magnetometer tilt-compensated heading correction
  // Zero-rate threshold prevents residual gyro noise from accumulating as drift
  if (fabs(gz_corr) <= 20) {
    if (fabs(gz_corr) > 0.03f) {
      yaw += gz_corr * dt;
      yaw = atan2f(sinf(yaw), cosf(yaw));
    }

    readMagBody(mx, my, mz);   // calibrated, body-frame magnetometer

    // magMinX = min(magMinX, mx);
    // magMaxX = max(magMaxX, mx);

    // magMinY = min(magMinY, my);
    // magMaxY = max(magMaxY, my);

    // magMinZ = min(magMinZ, mz);
    // magMaxZ = max(magMaxZ, mz);

    float magMag = sqrtf(mx*mx + my*my + mz*mz);

    // Skip correction if magnetometer returns zeros or implausible field strength
    // Earth's field is ~25–65 µT; zero means sensor uninitialized or dead
    if (isValidFloat(mx) && isValidFloat(my) && isValidFloat(mz) && magMag > 10.0f) {
      // Tilt-compensated heading (verified formula; see ICM20948_STATUS.md).
      // Inputs mx/my/mz are already calibrated + body-frame from readMagBody().
      float sr = sinf(roll), cr = cosf(roll);
      float sp = sinf(pitch), cp = cosf(pitch);
      float bx = mx * cp + my * sr * sp + mz * cr * sp;
      float by = my * cr - mz * sr;
      float magYaw = atan2f(-by, bx);

      // Shortest-angle blend — avoids pulling yaw the wrong way across ±π
      float err = magYaw - yaw;
      while (err >  M_PI) err -= 2.0f * M_PI;
      while (err < -M_PI) err += 2.0f * M_PI;

      const float alpha_yaw = 0.95f;  // 5% mag correction per step (was 2%)
      static uint32_t lastPrint = 0;

      if (millis() - lastPrint > 200) {
      Serial.printf(
        "mx=%.1f my=%.1f mz=%.1f | magYaw=%.1f | yaw=%.1f\n",
        mx,
        my,
        mz,
        degrees(magYaw),
        degrees(yaw)
    );

      lastPrint = millis();
      
      }
      yaw += (1.0f - alpha_yaw) * err;
      yaw = atan2f(sinf(yaw), cosf(yaw));
    }
  }
}

// ================= ACCEL =================
void computeVerticalAcceleration() {

  if (!isValidFloat(ax) || !isValidFloat(ay) || !isValidFloat(az)) return;

  if (fabs(ax) > 50 || fabs(ay) > 50 || fabs(az) > 50) return;

  float sinR = sin(roll);
  float cosR = cos(roll);
  float sinP = sin(pitch);
  float cosP = cos(pitch);

  float acc_z_world =
      ax * sinP +
      ay * sinR * cosP +
      az * cosR * cosP;

  // subtract gravity
 float new_a_z = acc_z_world - 9.81f;

  // =========================
  // VALIDATION
  // =========================
  if(!isValidFloat(new_a_z)) return;
  if (fabs(new_a_z) < 0.2f) new_a_z = 0.0f;

  a_z = new_a_z;
}

// ================= IMU STEP =================
void stepIMU() {
  updateOrientation();
  computeVerticalAcceleration();

  if (fabs(a_z) < 0.2f) {
    v *= 0.90f;  // stronger damping
  }
}

// ================= TOF =================
void readTOF() {
  uint8_t dataReady = 0;
  tof.VL53L4CX_GetMeasurementDataReady(&dataReady);
  if (!dataReady) return;

  VL53L4CX_MultiRangingData_t data;
  tof.VL53L4CX_GetMultiRangingData(&data);

  int n = data.NumberOfObjectsFound;
  for (int i = 0; i < n; i++) {
    if (data.RangeData[i].RangeStatus == 0) {
      tofAlt = data.RangeData[i].RangeMilliMeter / 1000.0f;  // mm -> m
      break;
    }
  }
  tof.VL53L4CX_ClearInterruptAndStartMeasurement();
}

// ================= BARO =================
void updateBarometer() {

  if (!bmp.performReading()) return;
  pressure = bmp.pressure;  // already hPa

  // =========================
  // HARD VALIDATION
  // =========================
  if(!isValidFloat(pressure) || pressure < 300 || pressure > 1200) return;
  if(!isValidFloat(groundPressure) || groundPressure < 300) return;
  float ratio = pressure / groundPressure;
  if(!isValidFloat(ratio) || ratio <= 0) return;

  float alt = 44330.0f * (1.0f - pow(ratio, 0.1903f));
  if(isValidFloat(alt)) baroAlt = alt;
}

// ================= GROUND UPDATE =================
// void updateGroundPressure() {
//   static float lp = 0;
//   static bool init = false;
//   if(!init){
//     lp = pressure;
//     init = true;
//   }

//   if(fabs(v) < 0.03f && fabs(a_z) < 0.1f){
//     lp = 0.9998f * lp + 0.0002f * pressure;
//     groundPressure = lp;
//   }
// }
void updateGroundPressure() {
  static float lp = groundPressure;
  
  // Aggressive ground tracking when near zero + no motion
  if (fabs(h) < 0.5f && fabs(v) < 0.05f && fabs(a_z) < 0.2f) {
    lp = 0.995f * lp + 0.005f * pressure;  // Slow baro tracking
    groundPressure = lp;
  }
}

// ================= KALMAN =================
void kalmanPredict() {

  // =========================
  // STATE PREDICTION
  // =========================
  float h_pred = h + v * dt + 0.5f * a_z * dt * dt;
  float v_pred = v + a_z * dt;

  if(!isValidFloat(h_pred) || !isValidFloat(v_pred)) return;

  h = h_pred;
  v = v_pred;

  // =========================
  // FULL COVARIANCE UPDATE
  // =========================
  float P00_new = P00 + dt * (P10 + P01) + dt * dt * P11 + Qh;
  float P01_new = P01 + dt * P11;
  float P10_new = P10 + dt * P11;
  float P11_new = P11 + Qv;

  if(!isValidFloat(P00_new) || !isValidFloat(P11_new)) return;

  P00 = P00_new;
  P01 = P01_new;
  P10 = P10_new;
  P11 = P11_new;

  // clamp state
  v = constrain(v, -20.0f, 20.0f);
  h = constrain(h, -100.0f, 1000.0f);
}

void kalmanUpdateBaro() {

  float y = baroAlt - h;
  float S = P00 + R_baro;

  if(!isValidFloat(S) || S < 1e-6f) return;

  // Kalman gain
  float K0 = P00 / S;
  float K1 = P10 / S;

  if (!isValidFloat(K0) || !isValidFloat(K1)) return;

  // update state
  h += K0 * y;
  v += K1 * y;
}

// ================= SETUP =================
void setup() {
  
  Serial.begin(115200);
  // while(!Serial) delay(10);
  delay(5000);

  // -------------------------
  // I2C SETUP
  // -------------------------
  // Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW); 
  delay(10);

  I2Cdev.begin(SDA_PIN, SCL_PIN);
  I2Cdev.setClock(100000);
  digitalWrite(XSHUT_PIN, HIGH); 
  delay(100);
  
  // TOF
  tof.begin();
  tof.VL53L4CX_SetDeviceAddress(0x29);
  tof.InitSensor(0x29);
  tof.VL53L4CX_StartMeasurement();
  
  // // ESP32 safe stable clock
  // Serial.println("I2C Scan:");
  // for(byte addr=1; addr<127; addr++) {
  //   I2Cdev.beginTransmission(addr);
  //   if(I2Cdev.endTransmission() == 0) {
  //     Serial.printf("Found: 0x%02X\n", addr);
  //   }
  // }

  // -------------------------
  // SENSOR INIT
  // -------------------------
  if (!bmp.begin(0x47, &I2Cdev)){
    Serial.println("BMP FAIL");
    while(1);
  }

  // Configure sensor
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  Serial.println("⏳ BMP581 settling (2s)...");
  delay(2000);  // CRITICAL: Let calibration stabilize

  if (!initIMU()) {
    Serial.println("IMU FAIL");
    while(1);
  }

  // Ground calibration
  delay(3000);
  float sum = 0;
  int count = 0;
  for (int i = 0; i < 500; i++){
    if (bmp.performReading()) {
      sum += bmp.pressure;  // already hPa
      count++;
    }
    delay(25);  // 25ms > 20ms ODR period, ensures new data each iteration
  }
  groundPressure = (count > 0) ? sum / count : 0.0f;
  Serial.printf("Ground cal: %.2f hPa (%d samples)\n", groundPressure, count);

  // Gyro bias
  int N = 2000;
  sensors_event_t a, g, t;
  for (int i = 0; i < N; i++){
    imu.getEvent(&a, &g, &t);

    // ICM20948 returns rad/s
    bias_gx += g.gyro.x;
    bias_gy += g.gyro.y;
    bias_gz += g.gyro.z;
    delay(2);
  }

  bias_gx /= N;
  bias_gy /= N;
  bias_gz /= N;

  Serial.printf(
    "Gyro biases:\n"
    "gx=%.6f\n"
    "gy=%.6f\n"
    "gz=%.6f\n",
    bias_gx,
    bias_gy,
    bias_gz
);

}

void loop() {
  
  // TIME
  unsigned long now = micros();
  dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if(!isValidFloat(dt) || dt <= 0.0001f || dt > 0.05f){
    dt = 0.01f;
  }

  // IMU
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  // Stall detection: identical consecutive gyro reads => I2C hung, data frozen.
  if (gx == prev_gx_raw && gy == prev_gy_raw && gz == prev_gz_raw) {
    if (++staleCount >= STALE_LIMIT) {
      Serial.println("# sensor stall detected, re-initialising IMU");
      initIMU();
      staleCount = 0;
      lastMicros = micros();   // avoid a huge dt spike after recovery
    }
  } else {
    staleCount = 0;
  }
  prev_gx_raw = gx;
  prev_gy_raw = gy;
  prev_gz_raw = gz;

  // PIPELINE
  readTOF();
  updateBarometer();
  stepIMU();
  kalmanPredict();
  kalmanUpdateBaro();

  // Serial.printf(
  //   "roll=%6.1f pitch=%6.1f yaw=%6.1f "
  //   "mx=%7.2f my=%7.2f mz=%7.2f\n",
  //   degrees(roll), degrees(pitch), degrees(yaw),
  //   mx, my, mz
  // );
  // Serial.printf("h:%.2f baroAlt:%.2f tof:%.3f | az:%.2f v:%.2f | roll:%.1f pitch:%.1f yaw:%.1f | ax:%.2f ay:%.2f az_raw:%.2f | gx:%.3f gy:%.3f gz:%.3f\n",
  //             h, baroAlt, tofAlt,
  //             a_z, v,
  //             degrees(roll), degrees(pitch), degrees(yaw),
  //             ax, ay, az,
  //             gx - bias_gx, gy - bias_gy, gz - bias_gz);

  // Magnetometer is read, calibrated, and remapped to body frame inside
  // updateOrientation() -> readMagBody(); the mx/my/mz globals already hold the
  // calibrated body-frame field, so no separate raw read is needed here.

  // Serial.printf(
  //   "roll:%.1f pitch:%.1f yaw:%.1f\n",
  //   degrees(roll),
  //   degrees(pitch),
  //   degrees(yaw)
  // );
}
