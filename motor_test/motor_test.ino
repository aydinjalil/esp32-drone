// ============================================================================
//  motor_test.ino  —  standalone ESC / motor wiring test (classic ESP32 DevKit)
// ----------------------------------------------------------------------------
//  Spins ONE motor at a time, on command, at a low throttle so you can confirm
//  each ESC signal wire goes to the right motor and the right spin direction —
//  WITHOUT any sensors wired. No IMU/baro/ToF, no PID, no flight loop.
//
//  *** REMOVE ALL PROPELLERS BEFORE RUNNING. ***
//
//  Wiring (matches drone_flight_controller.ino):
//    ESC_FR = GPIO18 (Front Right)   ESC_FL = GPIO19 (Front Left)
//    ESC_BL = GPIO23 (Back Left)     ESC_BR = GPIO25 (Back Right)
//    All ESC grounds MUST be common with the ESP32 GND.
//    ESP32 powered from PDB 5V BEC -> VIN/5V pin (NOT 3V3).
//
//  Serial Monitor: 115200 baud, line ending "Newline" or "No line ending".
//  Commands (send a single character):
//    1 = Front Right   2 = Front Left   3 = Back Left   4 = Back Right
//    a = arm ESCs (run once after power-up / ESC beeps)
//    s = STOP all motors (idle)         <-- also the emergency stop
//    + = throttle up    - = throttle down   (changes the test throttle)
//    t = run all four briefly, one after another
//    ? = print this menu
// ============================================================================

#include <ESP32Servo.h>

// --- ESC signal pins (classic ESP32, output-safe GPIOs) ---
#define ESC_FR 18   // Front Right
#define ESC_FL 19   // Front Left
#define ESC_BL 23   // Back Left
#define ESC_BR 25   // Back Right

Servo escFR, escFL, escBL, escBR;

// --- ESC pulse range (us) — same as the flight controller ---
const int ESC_MIN_US  = 1000;   // disarmed / zero throttle
const int ESC_MAX_US  = 2000;
const int ESC_ARM_US  = 1100;   // end of the arm ramp

// Test throttle as a microsecond pulse. Start LOW. 1120us is just above where
// most ESCs begin to spin. Adjust with + / - (step 20us, clamped 1050..1300).
int testUs   = 1120;
const int TEST_STEP = 20;
const int TEST_MIN  = 1050;
const int TEST_MAX  = 1300;   // hard cap for bench testing — keep it gentle
const unsigned long SPIN_MS = 1500;  // how long a single-motor test runs

bool armed = false;

void allIdle() {
  escFR.writeMicroseconds(ESC_MIN_US);
  escFL.writeMicroseconds(ESC_MIN_US);
  escBL.writeMicroseconds(ESC_MIN_US);
  escBR.writeMicroseconds(ESC_MIN_US);
}

void armEscs() {
  Serial.println(F("Arming ESCs (ramp 1000 -> 1100us)..."));
  for (int i = ESC_MIN_US; i <= ESC_ARM_US; i++) {
    escFR.writeMicroseconds(i); escFL.writeMicroseconds(i);
    escBL.writeMicroseconds(i); escBR.writeMicroseconds(i);
    delay(25);
  }
  allIdle();
  armed = true;
  Serial.println(F("ESCs armed. Motors idle. Send 1-4 to spin a motor."));
}

// Spin exactly one motor at testUs for SPIN_MS, then stop it.
void spinOne(Servo &esc, const char *name) {
  if (!armed) { Serial.println(F("Not armed — send 'a' first.")); return; }
  allIdle();                       // guarantee the other three stay stopped
  Serial.printf("Spinning %s at %dus for %lums...\n", name, testUs, SPIN_MS);
  esc.writeMicroseconds(testUs);
  delay(SPIN_MS);
  esc.writeMicroseconds(ESC_MIN_US);
  Serial.printf("%s stopped.\n", name);
}

void printMenu() {
  Serial.println(F("\n=== MOTOR TEST — PROPS OFF! ==="));
  Serial.println(F(" 1=FrontRight  2=FrontLeft  3=BackLeft  4=BackRight"));
  Serial.println(F(" a=arm   s=STOP all   +=throttle up   -=throttle down"));
  Serial.println(F(" t=test all in sequence   ?=menu"));
  Serial.printf (" test throttle = %dus (range %d..%d)\n", testUs, TEST_MIN, TEST_MAX);
  Serial.println(F("================================"));
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // ESP32Servo housekeeping + 50Hz servo PWM for the ESCs
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  escFR.setPeriodHertz(50); escFL.setPeriodHertz(50);
  escBL.setPeriodHertz(50); escBR.setPeriodHertz(50);

  escFR.attach(ESC_FR, ESC_MIN_US, ESC_MAX_US);
  escFL.attach(ESC_FL, ESC_MIN_US, ESC_MAX_US);
  escBL.attach(ESC_BL, ESC_MIN_US, ESC_MAX_US);
  escBR.attach(ESC_BR, ESC_MIN_US, ESC_MAX_US);

  allIdle();   // hold low so ESCs see a valid disarm signal at boot

  Serial.println(F("\n*** REMOVE ALL PROPELLERS BEFORE PROCEEDING ***"));
  Serial.println(F("Holding all ESCs at 1000us (disarmed)."));
  Serial.println(F("Wait for the ESC startup beeps, then send 'a' to arm."));
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case '1': spinOne(escFR, "Front Right (GPIO18)"); break;
    case '2': spinOne(escFL, "Front Left  (GPIO19)"); break;
    case '3': spinOne(escBL, "Back Left   (GPIO23)"); break;
    case '4': spinOne(escBR, "Back Right  (GPIO25)"); break;
    case 'a': case 'A': armEscs(); break;
    case 's': case 'S':
      allIdle();
      Serial.println(F("STOP — all motors idle."));
      break;
    case '+': case '=':
      testUs = min(testUs + TEST_STEP, TEST_MAX);
      Serial.printf("test throttle = %dus\n", testUs);
      break;
    case '-': case '_':
      testUs = max(testUs - TEST_STEP, TEST_MIN);
      Serial.printf("test throttle = %dus\n", testUs);
      break;
    case 't': case 'T':
      spinOne(escFR, "Front Right (GPIO18)"); delay(400);
      spinOne(escFL, "Front Left  (GPIO19)"); delay(400);
      spinOne(escBL, "Back Left   (GPIO23)"); delay(400);
      spinOne(escBR, "Back Right  (GPIO25)");
      break;
    case '?': printMenu(); break;
    case '\n': case '\r': break;   // ignore line endings
    default:
      Serial.printf("Unknown command '%c' — send ? for menu.\n", c);
      break;
  }
}
