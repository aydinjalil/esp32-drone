#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20948 imu;

// ======== REPLACE with your calibration constants ========
const float MAG_OFFSET_X = -27.90f;
const float MAG_OFFSET_Y = -59.18f;
const float MAG_OFFSET_Z = 151.05f;

const float MAG_SCALE_X  = 1.215f;
const float MAG_SCALE_Y  = 1.175f;
const float MAG_SCALE_Z  = 0.754f;
// =========================================================

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  if (!imu.begin_I2C()) {
    Serial.println("IMU FAIL");
    while (1) delay(10);
  }

  Serial.println("ICM20948 tilt-compensated yaw demo");
}

void readCalibratedMag(float &mx, float &my, float &mz) {
  sensors_event_t accel, gyro, temp, mag;
  imu.getEvent(&accel, &gyro, &temp, &mag);  // single read for all sensors[web:45]

  float rawX = mag.magnetic.x;
  float rawY = mag.magnetic.y;
  float rawZ = mag.magnetic.z;

  float x = rawX - MAG_OFFSET_X;
  float y = rawY - MAG_OFFSET_Y;
  float z = rawZ - MAG_OFFSET_Z;

  mx = x * MAG_SCALE_X;
  my = y * MAG_SCALE_Y;
  mz = z * MAG_SCALE_Z;
}

void loop() {
  sensors_event_t accel, gyro, temp, mag;
  imu.getEvent(&accel, &gyro, &temp, &mag);  // accel + mag in one go[web:45]

  // Accel in m/s^2 (Adafruit Unified Sensor units)
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Normalize accel (optional but helps)
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Roll φ and pitch θ from accel (right-handed, you may swap depending on your frame)[web:54][web:62]
  float roll  = atan2(ay, az);                         // φ
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));   // θ

  // Calibrated mag
  float rawX = mag.magnetic.x;
  float rawY = mag.magnetic.y;
  float rawZ = mag.magnetic.z;

  float mx_raw = (rawX - MAG_OFFSET_X) * MAG_SCALE_X;
  float my_raw = (rawY - MAG_OFFSET_Y) * MAG_SCALE_Y;
  float mz_raw = (rawZ - MAG_OFFSET_Z) * MAG_SCALE_Z;

  // Tilt-compensate mag to horizontal frame[web:53][web:60]
  float cosRoll  = cos(roll);
  float sinRoll  = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // One common ENU-form tilt compensation variant:
  float mx_h = mx_raw * cosPitch + mz_raw * sinPitch;
  float my_h = mx_raw * sinRoll * sinPitch + my_raw * cosRoll - mz_raw * sinRoll * cosPitch;

  // Heading (yaw) in radians, range -pi..pi[web:60]
  float yaw = atan2(-my_h, mx_h);
  

  // Convert to degrees and wrap to 0..360 for readability
  float roll_deg  = roll  * 180.0f / PI;
  float pitch_deg = pitch * 180.0f / PI;
  float yaw_deg   = yaw   * 180.0f / PI;
  if (yaw_deg < 0) yaw_deg += 360.0f;

  static uint32_t t = 0;
  if (millis() - t > 200) {
    t = millis();
    Serial.printf("R=%.1f P=%.1f Y=%.1f | mx=%.1f my=%.1f mz=%.1f\n",
                  roll_deg, pitch_deg, yaw_deg,
                  mx_raw, my_raw, mz_raw);
  }

  delay(5);  // small delay to keep loop tame; you can tune this
}