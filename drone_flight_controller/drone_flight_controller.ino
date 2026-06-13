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
#define SDA_PIN 16
#define SCL_PIN 17

// ESC Pins (ESP32-S3)
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
float R_baro = 2.0f; // barometer measurement noise 

// ======================================================
// SENSOR OBJECTS
// ======================================================
// Adafruit_MPU6050 mpu;
Adafruit_ICM20948 imu;
Adafruit_BMP5xx bmp;
VL53L4CX tof(&I2Cdev, XSHUT_PIN);
float tofAlt = 0.0f; //VL53L4CX sensor
bool tofValid = false;
bool zeroed = false;


// ========================================================
// Flight Control Variables
// ========================================================
bool armed = false;
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

// ================= HELPERS =================
bool isValidFloat(float x) {
  return !(isnan(x) || isinf(x));
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

// =========================
// ARMING DETECTION
// =========================
void detectArming(){
  static int groundLostCount = 0;
  if(tofValid && tofAlt > 0.5f){
    groundLostCount++;
    if (groundLostCount > 20 && !armed){ // 0.4s confirmation
      takeoffGroundPressure = groundPressure; // FREEZE reference!
      armed = true;
      Serial.println("🚁 ARMED - Ceiling active");
    }
  } else{
    groundLostCount = 0;
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
  float derivative = (error - lastError) / dt;
  lastError = error;
  return Kp * error + Ki * integral + Kd * derivative;
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
  
  pressure = bmp.readPressure();

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
  while(!Serial) delay(10);

  // -------------------------
  // I2C SETUP
  // -------------------------
  // Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW); 
  delay(10);

  I2Cdev.begin(SCL_PIN, SDA_PIN);
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

  // Ground Calibration 
  Serial.println("Auto-zeroing (keep still)...");
  float totalPressure = 0;
  for (int i =0; i < 500; i++){
    totalPressure += bmp.readPressure();
    delay(10);
  }
  groundPressure = totalPressure / 500.0f;
  Serial.printf("\n✅ Ground zero complete! %.2f hPa\n", groundPressure);
  zeroed = true;

  if (!imu.begin_I2C(ICM20948_I2CADDR_DEFAULT, &I2Cdev)) {
    Serial.println("IMU FAIL");
  }else{
    // ===== SENSOR CONFIG HERE =====
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRateDivisor(0);
  imu.setGyroRateDivisor(0);
  }

  // Gyro bias
  int N = 500;
  sensors_event_t a, g, t;
  for (int i = 0; i < N; i++){
    imu.getEvent(&a, &g, &t);

    // ICM20948 returns rad/s
    bias_gx += g.gyro.x;
    bias_gy += g.gyro.y;
    bias_gz += g.gyro.z;
    delay(5);
  }

  bias_gx /= N;
  bias_gy /= N;
  bias_gz /= N;

  // Setup motors
  escFR.attach(ESC_FR, 1000, 2000);
  escFL.attach(ESC_FL, 1000, 2000);
  escBL.attach(ESC_BL, 1000, 2000);
  escBR.attach(ESC_BR, 1000, 2000);
  
  Serial.println("Arming ESCs...");
  for (int i = 1000; i <= 1100; i++){
    escFR.writeMicroseconds(i); escFL.writeMicroseconds(i);
    escBL.writeMicroseconds(i); escBR.writeMicroseconds(i);
    delay(25);
  }
  Serial.println("🚁 FLIGHT CONTROLLER READY!");
}


// -------------------------
// MAIN FLIGHT LOOP
// -------------------------
void loop() {
  
  // TIME
  unsigned long now = millis();
  if(now - lastLoop < 20) return; //50Hz loop
  lastLoop = now;
  dt = (now-lastLoop) / 1000.0f;

  // IMU
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  if(!zeroed) return;

  // PIPELINE
  updateTOF();
  updateBarometer();
  stepIMU();
  updateGroundPressure();
  detectArming();
  detectDisarm();
  kalmanPredict();
  kalmanUpdateBaro();
  

  // KALMAN UPDATE(TOF Priority)
  if(tofValid && tofAlt < 4.0f){
    float y = tofAlt - h; float S = P00 + 0.005f;
    if(S > 1e-6f) {float K0 = P00/S; h+= K0 * y;}
  } else{
    float y = baroAlt - h; float S = P00 + R_baro;
    if(S > 1e-6f) { float K0 = P00/S; h+= K0 * y;}
  }

  // Pilot Inputs (Simulated-replace with RC)
  throttle = 0.55f + 0.1f * sin(now * 0.001f);    // Hover + climb
  target_roll = 5.0f * sin(now* 0.002f);          // ±5° roll command
  target_pitch = 5.0f * cos(now * 0.0015f);       // ±5° pitch command

  // Attitude PID Control
  float rollPID = computeAttitudePID(target_roll, roll, errI_roll, lastErr_roll);
  float pitchPID = computeAttitudePID(target_pitch, pitch, errI_pitch, lastErr_pitch);

  // Safety Ceiling
  float safeThrottle = computeSafeThrottle(throttle);

  // Motor Mixing
  motorMixing(safeThrottle, rollPID, pitchPID);

  // ESC OUTPUT
  escFR.writeMicroseconds(1000 + 900 * motorFR);
  escFL.writeMicroseconds(1000 + 900 * motorFL);
  escBL.writeMicroseconds(1000 + 900 * motorBL);
  escBR.writeMicroseconds(1000 + 900 * motorBR);

  Serial.printf("h:%.1f armed:%s thr:%.3f | r:%.1f/%+.1f p:%.1f/%+.1f | mFR:%.2f mFL:%.2f mBL:%.2f mBR:%.2f | tof:%.0fmm\n",
                h, armed?"YES":"NO", safeThrottle,
                roll*180/PI, target_roll,
                pitch*180/PI, target_pitch,
                motorFR, motorFL, motorBL, motorBR,
                tofValid ? tofAlt*1000 : -1);
}
