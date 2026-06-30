// Core Arduino libraries
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// Sensonr libraries
#include <Adafruit_BMP5xx.h>  
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>
#include <vl53l4cx_class.h> // (laser distance measurer)
#include <ESP32Servo.h>

// Optional Sensor
#include <Adafruit_SSD1306.h> // Display

// ================= I2C =================
#define XSHUT_PIN 4
TwoWire I2Cdev = TwoWire(0);

// PINS
#define SDA_PIN 21
#define SCL_PIN 22

// ESC signal pins (classic ESP32 DevKit / WROOM-32).
// All four are output-safe GPIOs: not input-only (34/35/36/39), not flash
// (6-11), not boot-strapping (0/2/5/12/15), not UART (1/3/16/17). ESC ground
// must be common with the ESP32 GND. (Note: GPIO25 is invalid on the ESP32-S3 —
// these assignments are for the classic ESP32 only.)
#define ESC_FR 18 // Front Right
#define ESC_FL 19 // Front Left
#define ESC_BL 23 // Back Left
#define ESC_BR 25 // Back Right

// ======================================================
// TIME SYSTEM
// ======================================================
unsigned long lastMicros = 0;
unsigned long lastLoop = 0;
float dt = 0.01f;

// ======================================================
// STATE (Kalman filter core state)
// ======================================================
// h = altitude (meters)
// v = vertical velocity (m/s)

float h = 0.0f; // altitude (meters)
float v = 0.0f; // vertical velocity (m/s)

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

// Measured at-rest accel magnitude (should be ~9.81). Calibrated at startup and
// subtracted as "gravity" so accel scale/bias error doesn't bias vertical accel
// (and thus the velocity estimate). Replaces the hardcoded 9.81f.
float gravityMag = 9.81f;

// ======================================================
// ORIENTATION (needed for gravity removal later)
// ======================================================

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

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
float R_baro = 2.0f;   // barometer measurement noise (m^2; ~1.4m std, loose)
float R_tof  = 0.005f; // ToF measurement noise (m^2; ~7cm std, tight/accurate)

// ======================================================
// SENSOR OBJECTS
// ======================================================
// Adafruit_MPU6050 mpu;
Adafruit_ICM20948 imu;
Adafruit_BMP5xx bmp;
VL53L4CX tof(&I2Cdev, XSHUT_PIN);
float tofAlt = 0.0f; //VL53L4CX sensor
bool tofValid = false;
bool tofPresent = false;   // false until ToF inits OK; baro-only altitude if absent
bool zeroed = false;


// ========================================================
// Flight Control Variables
// ========================================================
bool armed = false;
bool killed = false;        // emergency-stop latch (motors forced idle until reset)
float takeoffGroundPressure = 0.0f;
const float MAX_ALTITUDE = 120.0f;
float hoverThrottle = 0.55f; // Tune this for motors

// Motor outputs (0.0 - 1.0 normalized)
float motorFR = 0.0f, motorFL = 0.0f, motorBL = 0.0f, motorBR = 0.0f;

// Pilot inputs (simulated - will replace with RC receiver)
float throttle = 0.0f, target_roll = 0.0f, target_pitch = 0.0f;

// Attitude PID
float Kp = 1.5f, Ki = 0.05f, Kd = 0.2f;
float errI_roll = 0, errI_pitch = 0, lastErr_roll = 0, lastErr_pitch = 0;

// ESC objects
Servo escFR, escFL, escBL, escBR;

// ESC pulse range (us)
const int ESC_MIN_US  = 1000;   // disarmed / zero throttle
const int ESC_SPAN_US = 900;    // 1000..1900

// IMU I2C stall detection (see drone_imu/docs/ICM20948_STATUS.md)
const int STALE_LIMIT = 20;     // ~0.4 s of frozen reads before re-init
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
  imu.setAccelRateDivisor(0);
  imu.setGyroRateDivisor(0);
  return true;
}

void writeMotorsIdle() {
  escFR.writeMicroseconds(ESC_MIN_US);
  escFL.writeMicroseconds(ESC_MIN_US);
  escBL.writeMicroseconds(ESC_MIN_US);
  escBR.writeMicroseconds(ESC_MIN_US);
}

// ================= ORIENTATION =================
void updateOrientation() {

  // remove bias First
  float gx_corr = gx - bias_gx;
  float gy_corr = gy - bias_gy;

  // Reject insane gyro
  if (fabs(gx_corr) > 20 || fabs(gy_corr) > 20) return;
    
  // integrate gyro (already bias corrected in loop)
  roll  += gx_corr * dt;
  pitch += gy_corr * dt;

  // accelerometer reference
  float accRoll  = atan2(ay, az);
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az));

  const float alpha = 0.98f;

  roll  = alpha * roll  + (1.0f - alpha) * accRoll;
  pitch = alpha * pitch + (1.0f - alpha) * accPitch;
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

  // subtract gravity (measured at-rest magnitude, not a nominal 9.81)
  float new_a_z = acc_z_world - gravityMag;

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

// Single 2-state KF measurement update. Measures altitude only, so H = [1 0].
// z = measurement (m), R = its noise variance (m^2). Pass baro OR ToF — one
// fused update per loop. Unlike the old code this ALSO reduces the covariance
// P = (I - K H) P, so the gain adapts down as confidence grows instead of
// staying pinned high.
void kalmanUpdate(float z, float R) {
  float y = z - h;          // innovation
  float S = P00 + R;        // innovation covariance

  if(!isValidFloat(S) || S < 1e-6f) return;

  // Kalman gain
  float K0 = P00 / S;       // altitude gain
  float K1 = P10 / S;       // velocity gain

  if (!isValidFloat(K0) || !isValidFloat(K1)) return;

  // state update
  h += K0 * y;
  v += K1 * y;

  // covariance update: P = (I - K H) P, with H = [1 0]
  float P00n = (1.0f - K0) * P00;
  float P01n = (1.0f - K0) * P01;
  float P10n = P10 - K1 * P00;
  float P11n = P11 - K1 * P01;

  if(!isValidFloat(P00n) || !isValidFloat(P11n)) return;

  P00 = P00n; P01 = P01n;
  P10 = P10n; P11 = P11n;
}

// =========================
// ARMING — explicit operator command over serial, with pre-arm checks.
// Replaces the old airborne auto-arm (which armed only AFTER takeoff — unsafe
// and logically inverted). Commands: 'a' arm, 'd' disarm, 'k' kill, 'r' reset.
// =========================
bool preArmOK(){
  bool level     = fabs(roll) < 0.17f && fabs(pitch) < 0.17f;            // ~10 deg
  bool still     = fabs(gx - bias_gx) < 0.5f && fabs(gy - bias_gy) < 0.5f;
  bool sensorsOK = isValidFloat(ax) && isValidFloat(az) && isValidFloat(roll);
  return level && still && sensorsOK && !killed;
}

void processCommands(){
  while (Serial.available()){
    char c = Serial.read();
    if (c == 'a'){
      if (preArmOK()){
        takeoffGroundPressure = groundPressure;   // freeze altitude reference
        armed = true;
        Serial.println("ARMED");
      } else {
        Serial.println("ARM REFUSED (not level/still, or killed)");
      }
    } else if (c == 'd'){
      armed = false; Serial.println("DISARMED");
    } else if (c == 'k'){
      killed = true; armed = false; Serial.println("KILL");
    } else if (c == 'r'){
      if (!armed){ killed = false; Serial.println("KILL RESET"); }
    }
  }
}

void detectDisarm(){
  if (armed && tofValid && tofAlt < 0.2f && fabs(v) < 0.05f){
    armed = false;
    Serial.println("🚁 DISARMED - Auto-zero resume");
  }
}

// =========================
// SAFETY CEILING
// =========================
float computeSafeThrottle(float  pilotThrottle){
  if(!armed || h < MAX_ALTITUDE) return pilotThrottle;
  Serial.printf("CEILING HIT: %.1fm\n", h);
  return hoverThrottle;
}

// =========================
// ATTITUDE PID
// =========================
float computeAttitudePID(float target, float current, float& integral, float& lastError){
  float error = target * PI/180.0f - current; // Degrees to radians
  integral += error * dt;
  integral = constrain(integral, -0.5f, 0.5f); //Anti-windup
  float derivative = (dt > 1e-6f) ? (error - lastError) / dt : 0.0f; // guard /0
  lastError = error;
  float out = Kp * error + Ki * integral + Kd * derivative;
  return isValidFloat(out) ? out : 0.0f;
}

// ==============================
// MOTOR MIXING (X configuration)
// ==============================
void motorMixing(float throttleInput, float rollPID, float pitchPID){
  motorFR = constrain(throttleInput + 0.25 * rollPID - 0.25f * pitchPID, 0.0f, 1.0f);
  motorFL = constrain(throttleInput - 0.25 * rollPID - 0.25f * pitchPID, 0.0f, 1.0f);
  motorBL = constrain(throttleInput + 0.25 * rollPID + 0.25f * pitchPID, 0.0f, 1.0f);
  motorBR = constrain(throttleInput - 0.25 * rollPID + 0.25f * pitchPID, 0.0f, 1.0f);
}

// =========================
// TOF + MODIFIED BAROMETER
// =========================
void updateTOF() {
  if (!tofPresent) { tofValid = false; return; }   // no ToF wired -> baro-only
  uint8_t ready = 0;
  VL53L4CX_MultiRangingData_t data;
  if (tof.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready) {
    tof.VL53L4CX_GetMultiRangingData(&data);
    if (data.RangeData[0].RangeStatus == 0) {
      tofAlt = data.RangeData[0].RangeMilliMeter / 1000.0f;
      tofValid = (tofAlt > 0.1f && tofAlt < 4.0f);
    }
    tof.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}

void updateBarometer() {

  if (!bmp.performReading()) return;
  pressure = bmp.pressure;   // hPa — readPressure() returns Pa and fails the 300-1200 guard

  // =========================
  // HARD VALIDATION
  // =========================
  if(!isValidFloat(pressure) || pressure < 300 || pressure > 1200) return;
  // if(!isValidFloat(groundPressure) || groundPressure < 300) return;

  // Use takeoff reference when armed
  if(armed && takeoffGroundPressure > 0){
    float ratio = pressure / takeoffGroundPressure;
    baroAlt = 44330.0f * (1.0f - pow(ratio, 0.1903f));
  } else{
    float ratio = pressure / groundPressure;
    baroAlt = 44330.0f * (1.0f - pow(ratio, 0.1903f));
  }
}

// ================= SETUP =================
void setup() {
  
  Serial.begin(115200);
  delay(2000);   // do NOT block on Serial — controller must boot untethered

  // -------------------------
  // I2C SETUP
  // -------------------------
  // Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW); 
  delay(10);

  I2Cdev.begin(SDA_PIN, SCL_PIN);   // begin(sda, scl) — args were swapped
  I2Cdev.setClock(100000);
  digitalWrite(XSHUT_PIN, HIGH); 
  delay(100);
  
  // TOF — non-fatal: the FC is bench-testable without it (baro-only altitude).
  // InitSensor returns a status; if the sensor is absent it errors and we just
  // leave tofPresent=false so the loop skips ToF and the Kalman uses baro.
  tof.begin();
  tof.VL53L4CX_SetDeviceAddress(0x29);
  if (tof.InitSensor(0x29) == VL53L4CX_ERROR_NONE) {
    tof.VL53L4CX_StartMeasurement();
    tofPresent = true;
    Serial.println("ToF OK");
  } else {
    tofPresent = false;
    Serial.println("ToF absent — altitude from baro only");
  }
  
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

  // Ground Calibration
  Serial.println("Auto-zeroing (keep still)...");
  float totalPressure = 0; int pCount = 0;
  for (int i = 0; i < 500; i++){
    if (bmp.performReading()) { totalPressure += bmp.pressure; pCount++; }  // hPa
    delay(10);
  }
  groundPressure = (pCount > 0) ? totalPressure / pCount : 0.0f;
  Serial.printf("\n✅ Ground zero complete! %.2f hPa (%d samples)\n", groundPressure, pCount);
  zeroed = (pCount > 0);

  if (!initIMU()) {
    Serial.println("IMU FAIL");
  }

  // Gyro bias
  int N = 500;
  sensors_event_t a, g, t;
  float gAccum = 0.0f;
  for (int i = 0; i < N; i++){
    imu.getEvent(&a, &g, &t);

    // ICM20948 returns rad/s
    bias_gx += g.gyro.x;
    bias_gy += g.gyro.y;
    bias_gz += g.gyro.z;

    // at-rest gravity magnitude (tilt-independent)
    gAccum += sqrtf(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);
    delay(5);
  }

  bias_gx /= N;
  bias_gy /= N;
  bias_gz /= N;

  // Use the measured rest magnitude as "gravity" if it's sane (8-12 m/s^2),
  // else keep the 9.81 default (e.g. if the board was moved during cal).
  float gMeas = gAccum / N;
  if (gMeas > 8.0f && gMeas < 12.0f) gravityMag = gMeas;
  Serial.printf("Gravity cal: %.3f m/s^2 (used %.3f)\n", gMeas, gravityMag);

  // Setup motors
  escFR.attach(ESC_FR, 1000, 2000);
  escFL.attach(ESC_FL, 1000, 2000);
  escBL.attach(ESC_BL, 1000, 2000);
  escBR.attach(ESC_BR, 1000, 2000);
  
  // ESC hardware init (idle ramp so the ESCs see a valid signal and initialize).
  // This is NOT flight-arming — the FC stays DISARMED until you send 'a'.
  Serial.println("Initializing ESCs (idle ramp 1000->1100us)...");
  for (int i = 1000; i <= 1100; i++){
    escFR.writeMicroseconds(i); escFL.writeMicroseconds(i);
    escBL.writeMicroseconds(i); escBR.writeMicroseconds(i);
    delay(25);
  }
  writeMotorsIdle();   // back to 1000us; motors stay idle until armed
  Serial.println("🚁 FLIGHT CONTROLLER READY — DISARMED. Send 'a' to arm (props OFF).");
}


// -------------------------
// MAIN FLIGHT LOOP
// -------------------------
void loop() {
  
  // TIME — 50 Hz gate; dt from micros, computed BEFORE updating the marker
  unsigned long now = millis();
  if(now - lastLoop < 20) return; //50Hz loop
  lastLoop = now;

  unsigned long nowUs = micros();
  dt = (nowUs - lastMicros) * 1e-6f;
  lastMicros = nowUs;
  if (!isValidFloat(dt) || dt <= 0.0001f || dt > 0.05f) dt = 0.02f;

  // IMU
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  // IMU stall detection: identical consecutive gyro reads => I2C hung (frozen data).
  if (gx == prev_gx_raw && gy == prev_gy_raw && gz == prev_gz_raw) {
    if (++staleCount >= STALE_LIMIT) {
      Serial.println("# IMU stall — disarm + idle + re-init");
      armed = false;        // degraded mode: drop arm on sensor loss
      writeMotorsIdle();
      initIMU();
      staleCount = 0;
    }
  } else {
    staleCount = 0;
  }
  prev_gx_raw = gx; prev_gy_raw = gy; prev_gz_raw = gz;

  if(!zeroed) return;

  // PIPELINE
  updateTOF();
  updateBarometer();
  stepIMU();
  updateGroundPressure();
  processCommands();
  detectDisarm();
  kalmanPredict();

  // Single fused measurement update (was applied twice: baro, then again inline).
  // ToF has priority when valid and in range (cm-accurate near ground); otherwise
  // fall back to the barometer. kalmanUpdate() now also reduces the covariance.
  if (tofValid && tofAlt < 4.0f) {
    kalmanUpdate(tofAlt, R_tof);
  } else {
    kalmanUpdate(baroAlt, R_baro);
  }

  // Pilot Inputs (Simulated — replace with RC + failsafe)
  throttle = 0.55f + 0.1f * sin(now * 0.001f);    // Hover + climb
  target_roll = 5.0f * sin(now* 0.002f);          // ±5° roll command
  target_pitch = 5.0f * cos(now * 0.0015f);       // ±5° pitch command

  // CONTROL + MOTOR OUTPUT — only when armed and not killed.
  float safeThrottle = 0.0f;
  if (armed && !killed) {
    float rollPID  = computeAttitudePID(target_roll,  roll,  errI_roll,  lastErr_roll);
    float pitchPID = computeAttitudePID(target_pitch, pitch, errI_pitch, lastErr_pitch);

    // Fault guard: never push NaN/inf into the mixer/ESCs
    if (!isValidFloat(rollPID) || !isValidFloat(pitchPID)) { rollPID = 0.0f; pitchPID = 0.0f; }

    safeThrottle = computeSafeThrottle(throttle);
    motorMixing(safeThrottle, rollPID, pitchPID);

    escFR.writeMicroseconds(ESC_MIN_US + (int)(ESC_SPAN_US * motorFR));
    escFL.writeMicroseconds(ESC_MIN_US + (int)(ESC_SPAN_US * motorFL));
    escBL.writeMicroseconds(ESC_MIN_US + (int)(ESC_SPAN_US * motorBL));
    escBR.writeMicroseconds(ESC_MIN_US + (int)(ESC_SPAN_US * motorBR));
  } else {
    // Disarmed / killed: motors idle, reset PID integrators (prevent windup)
    errI_roll = errI_pitch = 0.0f;
    lastErr_roll = lastErr_pitch = 0.0f;
    motorFR = motorFL = motorBL = motorBR = 0.0f;
    writeMotorsIdle();
  }

  Serial.printf("h:%.2f v:%+.2f state:%s thr:%.3f | r:%.1f/%+.1f p:%.1f/%+.1f | mFR:%.2f mFL:%.2f mBL:%.2f mBR:%.2f | tof:%.0fmm\n",
                h, v, killed?"KILL":(armed?"ARM":"OFF"), safeThrottle,
                roll*180/PI, target_roll,
                pitch*180/PI, target_pitch,
                motorFR, motorFL, motorBL, motorBR,
                tofValid ? tofAlt*1000 : -1);
}
