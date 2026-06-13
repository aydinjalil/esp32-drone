#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <ESP32Servo.h>

unsigned long t0 = millis();
unsigned long loopInterval = 20; // 20 ms = 50 Hz update
float seaLevelPressure;

// Sensor Pins
int SDA_PIN = 16;
int SCL_PIN = 17;

// IMU values

float Ax; // Acceleration in X direction
float Ay; // Acceleration in Y direction
float Az; // Acceleration in Z direction

float Gx; // Rotation rate about the sensor's X-axis
float Gy; // Rotation rate about the sensor's Y-axis
float Gz; // Rotation rate about the sensor's Z-axis

float pitch = 0.0f; // radians
float roll = 0.0f; // roll

// PID for pitch and roll
float Kp = 1.0f; // Proportional gain: how strongly the controller reacts to the current error (e.g., how far pitch is from level).
float Ki = 0.05f; // Integral gain: how much it corrects for accumulated past error
float Kd = 0.1f; // Derivative gain: How much it  "damps" the response by reacting to the rate of change of error (reduces overshoot/oscillation)

float errP_p = 0; // current proportional term for pitch: Kp * error_p
float errI_p = 0; // running integral of pitch error: Ki * sum_of_errors
float errD_p = 0; // derivative term for pitch: Kd * (error_change/dt)
float lastErr_p = 0; // stores the previous pitch error, used to compute the derivative term

float errP_r = 0, errI_r = 0, errD_r = 0, lastErr_r = 0;

float dt = 0.01f; //loop time ~10ms

// Motor outputs (normalized [0, 1] or PWM range I need)
float motorFR = 0.05f; //front right
float motorFL = 0.05f; //front left
float motorBL = 0.05f; //rear left
float motorBR = 0.05f; //rear right

// Throttle and setpoint (stick input would go here)
float throttle = 0.5f; //0 = min, 1 = max
float setpoint_pitch = 0.0f; // target pitch (0 = level)
float setpoint_roll = 0.0f; // target roll (0 = level)

Servo escFR, escFL, escBL, escBR;  // FrontLeft, FrontRight, BackRight, BackLeft

Adafruit_MPU6050 mpu; //I2C
Adafruit_BMP280 bmp; //I2C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  Serial.begin(115200);
  
  // Attach all 4 ESCs to your GPIO pins
  escFR.attach(18, 1000, 2000);  // Front Left  - CW motor
  escFL.attach(19, 1000, 2000);  // Front Right - CCW motor
  escBL.attach(23, 1000, 2000);  // Back Right  - CW motor  
  escBR.attach(25, 1000, 2000);  // Back Left   - CCW motor
  
  Serial.println("Arming all 4 ESCs...");
  

  int baseThrottle = 1000;
  int targetThrottle = 1000 + int(900 * throttle); // e.g., throttle=0.35
  Serial.println("Arming ESCs...");
  for (int t = baseThrottle; t <= targetThrottle; t += 5) {
      escFR.writeMicroseconds(t);
      escFL.writeMicroseconds(t);
      escBL.writeMicroseconds(t);
      escBR.writeMicroseconds(t);
      delay(20);
  }
  
  Serial.println("ALL ESCs ARMED! Spinning all motors...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); //Max i2C Speed: 400kHz

  // Initialize BMP280
  bool bmpOK = bmp.begin(0x76);
  if (!bmpOK) bmpOK = bmp.begin(0x77);

  if (!bmpOK) {
    Serial.println("BMP280 not found!");
    while (1) delay(10);
  }

  // Ground reference calibration
  Serial.println("Calibrating BMP280 - keep drone on ground...");
  delay(2000); //This will let the sensor stabilize

  float pressureSum = 0;
  for (int i = 0; i < 50; i++){
    pressureSum += bmp.readPressure();
    delay(20);
  }
  seaLevelPressure = pressureSum /50 / 100.0F; //hPa
  Serial.printf("Ground ref: %.2f hPa\n", seaLevelPressure);
  
  // Initialize MPU6050
  if (!mpu.begin()){
    Serial.println("Failed to find MPU6050");
    while (1) delay(10);
  }

  // // Init OLED
  // if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
  //   Serial.println("Oled not found!");
  //   while(1) delay(10);
  // }

  
  Serial.println("All three devices ready!");

  // mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // ALL MOTORS 25% throttle (PROPS OFF!)

  unsigned long t0 = micros();
  // Get sensor data 
  
  // Accelerometer and Gyro
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);
  Ax = a.acceleration.x;
  Ay = a.acceleration.y;
  Az = a.acceleration.z;

  Gx = g.gyro.x;
  Gy = g.gyro.y;
  Gz = g.gyro.z;

  // Barometer

  float temperature = bmp.readTemperature(); //degrees Celcius
  float pressure = bmp.readPressure(); // Pa
  float altitude = bmp.readAltitude(seaLevelPressure); // meters (sea level pressure measured at 0 level of the drone)

  // Angle estimation
  float accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az));
  float accRoll = atan2(Ay, sqrt(Ax * Ax + Az * Az));

  pitch = 0.95f * (pitch + Gy * dt) + 0.05f * accPitch;
  roll = 0.95f * (roll + Gx * dt) + 0.05f * accRoll;

    // --- PID for pitch ---
  float error_p = setpoint_pitch - pitch;
  errI_p += error_p * dt;
  errD_p = (error_p - lastErr_p) / dt;
  float pid_p = Kp * error_p + Ki * errI_p + Kd * errD_p;
  lastErr_p = error_p;

  // --- PID for roll ---
  float error_r = setpoint_roll - roll;
  errI_r += error_r * dt;
  errD_r = (error_r - lastErr_r) / dt;
  float pid_r = Kp * error_r + Ki * errI_r + Kd * errD_r;
  lastErr_r = error_r;

  //  --- motor mixer ---
 float base = throttle;


  float pid_scale = 0.2f; // reduce impact
  float pid_p_scaled = pid_p * pid_scale;
  float pid_r_scaled = pid_r * pid_scale;

  motorFR = constrain(base + pid_r_scaled - pid_p_scaled, 0.25f, 1.0f);
  motorFL = constrain(base - pid_r_scaled - pid_p_scaled, 0.25f, 1.0f);
  motorBL = constrain(base + pid_r_scaled + pid_p_scaled, 0.25f, 1.0f);
  motorBR = constrain(base - pid_r_scaled + pid_p_scaled, 0.25f, 1.0f);

  //  --- send to ESCs ---
  int us1 = 1000 + 900 * motorFR; // in microseconds
  int us2 = 1000 + 900 * motorFL;
  int us3 = 1000 + 900 * motorBL;
  int us4 = 1000 + 900 * motorBR;

  // Serial.printf("M1:%d M2:%d M3:%d M4:%d\n", us1, us2, us3, us4);
  // Serial.printf("Pitch:%.2f Roll:%.2f\n", pitch, roll);
  Serial.printf("Alt: %.2f m\nTemp: %.1f C\nPres: %.0f Pa\n", altitude, temperature, pressure);
  
  // // Display on OLED
  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);

  // display.setCursor(0, 0);
  // display.printf("Pitch: %.2f Roll: %.2f\n", pitch, roll);
  // display.printf("Throttle: %.2f\n", throttle);
  // display.printf("FR: %.2f FL: %.2f\nBL: %.2f BR: %.2f", motorFR, motorFL, motorBL, motorBR);
  // // display.printf("Alt: %.2f m\nTemp: %.1f C\nPres: %.0f Pa\n", altitude, temperature, pressure);
  // display.display();
  
  unsigned long now = millis();
  if (now - t0 >= loopInterval) {
    t0 = now;

    // --- PID + motor mixing code ---
    escFR.writeMicroseconds(us1);
    escFL.writeMicroseconds(us2);
    escBL.writeMicroseconds(us3);
    escBR.writeMicroseconds(us4);
    }
  
}