#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 imu;

// State (radians)
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// Gyro bias
float bias_gx = 0.0f;
float bias_gy = 0.0f;
float bias_gz = 0.0f;

// Timing
unsigned long lastMicros = 0;
float dt = 0.01f;

// Magnetometer hard-iron offset and soft-iron matrix (native sensor frame).
// Calibrated sample:  m_cal = MAG_A * (m_raw - MAG_B)
// Generate these with python/calibration/mag_calibrate.py and paste the output
// here. Values below are from the FLIGHT-CONFIG capture (2026-07-04, 18.9k
// samples streamed over Bluetooth away from the laptop; battery strapped
// under the frame, ESCs powered; axis ratio 1.19, off-diagonal 6.1%, CV 8.3%
// on the fit set / 9.4% incl. interference bursts). The hard-iron offset
// moved a lot vs the 2026-06-30 cal — that's the battery relocation:
// recalibrate after any layout/power change. ~8% CV appears to be this
// airframe's floor with power on.
const float MAG_B[3] = { -15.5383f, 20.9900f, -16.2438f };

const float MAG_A[3][3] = {
  {  1.0340f,  0.0612f, -0.0023f },
  {  0.0612f,  0.9243f,  0.0192f },
  { -0.0023f,  0.0192f,  1.0615f }
};

// Stall detection / recovery. The ICM-20948 reads its magnetometer over an
// internal I2C master, which can hang during long runs and freeze the whole
// chip's I2C interface -- after which getEvent() returns stale, byte-identical
// values forever. A live gyro is never identical across reads, so a run of
// identical gyro samples means the bus has stalled; we then re-init to recover.
const int STALE_LIMIT = 20;   // ~0.4 s of frozen reads before re-init
int staleCount = 0;
float prev_gx_raw = 1e9f, prev_gy_raw = 1e9f, prev_gz_raw = 1e9f;

bool initIMU() {
  if (!imu.begin_I2C(0x69, &Wire)) return false;   // SDO/AD0 wired HIGH -> 0x69 (confirmed by i2c_scan)
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
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

  Serial.println("Gyro bias calib (keep still)...");
  const int N = 500;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    bias_gx += g.gyro.x;
    bias_gy += g.gyro.y;
    bias_gz += g.gyro.z;
    delay(2);
  }
  bias_gx /= N;
  bias_gy /= N;
  bias_gz /= N;

  Serial.printf("bias_gx=%.6f bias_gy=%.6f bias_gz=%.6f\n",
                bias_gx, bias_gy, bias_gz);

  lastMicros = micros();

  // CSV header
  Serial.println("time_ms,roll_deg,pitch_deg,yaw_deg,magYaw_deg,mx,my,mz,gz,dt");
}

void loop() {
  unsigned long now = micros();
  float dt_raw = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt_raw > 0.0001f && dt_raw < 0.05f) {
    dt = dt_raw;
  } else {
    dt = 0.01f;
  }

  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  sensors_event_t mag;
  imu.getMagnetometerSensor()->getEvent(&mag);

  // Stall detection: identical consecutive gyro reads => I2C hung, data frozen.
  if (gyro.gyro.x == prev_gx_raw &&
      gyro.gyro.y == prev_gy_raw &&
      gyro.gyro.z == prev_gz_raw) {
    if (++staleCount >= STALE_LIMIT) {
      Serial.println("# sensor stall detected, re-initialising IMU");
      initIMU();
      staleCount = 0;
      lastMicros = micros();   // avoid a huge dt spike after recovery
      delay(5);
      return;                  // skip this cycle; read fresh next loop
    }
  } else {
    staleCount = 0;
  }
  prev_gx_raw = gyro.gyro.x;
  prev_gy_raw = gyro.gyro.y;
  prev_gz_raw = gyro.gyro.z;

  float ax_s = accel.acceleration.x;
  float ay_s = accel.acceleration.y;
  float az_s = accel.acceleration.z;

  float gx_s = gyro.gyro.x - bias_gx;
  float gy_s = gyro.gyro.y - bias_gy;
  float gz_s = gyro.gyro.z - bias_gz;

  float mx_raw = mag.magnetic.x;
  float my_raw = mag.magnetic.y;
  float mz_raw = mag.magnetic.z;

  // Calibrated magnetometer in sensor frame: m_cal = MAG_A * (m_raw - MAG_B)
  float cx = mx_raw - MAG_B[0];
  float cy = my_raw - MAG_B[1];
  float cz = mz_raw - MAG_B[2];

  float mx_s = MAG_A[0][0] * cx + MAG_A[0][1] * cy + MAG_A[0][2] * cz;
  float my_s = MAG_A[1][0] * cx + MAG_A[1][1] * cy + MAG_A[1][2] * cz;
  float mz_s = MAG_A[2][0] * cx + MAG_A[2][1] * cy + MAG_A[2][2] * cz;

  // -------------------------------------------------
  // BODY FRAME: X = nose, Y = right, Z = up
  // Trial alignment:
  // accel/gyro body mapping
  float ax = ax_s;
  float ay = ay_s;
  float az = az_s;

  float gx = gx_s;
  float gy = gy_s;
  float gz = gz_s;

  // Reconcile the magnetometer into the accel/gyro body frame so roll/pitch
  // (derived from accel) are applied to the matching mag axes. This mapping was
  // derived empirically from logged data (python/tools/imu_logger.py + an
  // ellipsoid/tilt-invariance analysis): of all signed axis permutations, this
  // is the one that makes the horizontal field constant under roll
  // (dH/droll ~= 0.006 uT/deg, H_CV 2.9%), i.e. true tilt-invariance.
  // If a flat 360 spin shows heading running backwards, negate all three.
  float mx =  mx_s;   // body X (nose)
  float my = -my_s;   // body Y (right)
  float mz =  mz_s;   // body Z (up)
  // -------------------------------------------------

  // 1) Gyro integration
  roll  -= gx * dt;
  pitch -= gy * dt;
  yaw   += gz * dt;

  if (yaw > PI)  yaw -= 2.0f * PI;
  if (yaw < -PI) yaw += 2.0f * PI;

  // 2) Accel-only roll/pitch
  float accRoll  = atan2f(-ay, az);
  float accPitch = atan2f(ax, sqrtf(ay * ay + az * az));

  // 3) Complementary filter for roll & pitch
  const float alpha = 0.98f;
  roll  = alpha * roll  + (1.0f - alpha) * accRoll;
  pitch = alpha * pitch + (1.0f - alpha) * accPitch;

  // 4) Tilt-compensated mag heading
  float sr = sinf(roll);
  float cr = cosf(roll);
  float sp = sinf(pitch);
  float cp = cosf(pitch);

  float bx = mx * cp + my * sr * sp + mz * cr * sp;
  float by = my * cr - mz * sr;

  float magYaw_tilt = atan2f(-by, bx);
  float magMag = sqrtf(mx * mx + my * my + mz * mz);

  // Gentle yaw correction from magnetometer
  if (magMag > 5.0f) {
    float err = magYaw_tilt - yaw;
    if (err > PI)  err -= 2.0f * PI;
    if (err < -PI) err += 2.0f * PI;

    const float beta = 0.01f;   // mag->yaw nudge gain; ~1.7s settle (was 0.005 ~3.4s)
    yaw += beta * err;

    if (yaw > PI)  yaw -= 2.0f * PI;
    if (yaw < -PI) yaw += 2.0f * PI;
  }

  float roll_deg   = roll * 180.0f / PI;
  float pitch_deg  = pitch * 180.0f / PI;
  float yaw_deg    = yaw * 180.0f / PI;
  float magYaw_deg = magYaw_tilt * 180.0f / PI;
  if (magYaw_deg < 0) magYaw_deg += 360.0f;

  unsigned long t_ms = millis();

  Serial.printf(
    "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f\n",
    t_ms,
    roll_deg,
    pitch_deg,
    yaw_deg,
    magYaw_deg,
    mx, my, mz,
    gz,
    dt
  );

  delay(10);
}