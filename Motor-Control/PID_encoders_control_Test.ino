#include <Wire.h>

// ─── MPU-6050 ────────────────────────────────────────────
#define MPU_ADDR 0x68
#define ALPHA    0.98f

// ─── Motor A pins ────────────────────────────────────────
#define AIN1 26
#define AIN2 27
#define PWMA 25

// ─── Encoder pins (Motor A) ──────────────────────────────
#define ENC_A1 34
#define ENC_B1 35

// ─── PWM config ──────────────────────────────────────────
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// ─── Encoder state ───────────────────────────────────────
volatile long encCountA = 0;
#define COUNTS_PER_REV 1980.0f
#define MAX_RPM        275.0f

// ─── IMU state ───────────────────────────────────────────
float gyroXOffset = 0;
float pitch       = 0.0f;
unsigned long lastTime = 0;

// ─── Outer PID (angle → target RPM) ─────────────────────
float setpoint     = 90.0f;
float Kp_angle     = 2.0f;
float Ki_angle     = 0.0f;
float Kd_angle     = 0.5f;
float angleError, angleLastError = 0, angleIntegral = 0;
float targetRPM    = 0;

// ─── Inner PID (velocity → PWM) ──────────────────────────
float Kp_vel       = 1.5f;
float Ki_vel       = 0.5f;
float Kd_vel       = 0.05f;
float velError, velLastError = 0, velIntegral = 0;

// ─── RPM tracking ────────────────────────────────────────
float rpmA = 0;
unsigned long lastRPMTime = 0;
long lastEncA = 0;

// ─────────────────────────────────────────────────────────
// Encoder ISR
// ─────────────────────────────────────────────────────────
void IRAM_ATTR isrA() {
  if (digitalRead(ENC_A1) == digitalRead(ENC_B1)) encCountA++;
  else encCountA--;
}

// ─────────────────────────────────────────────────────────
// MPU helpers
// ─────────────────────────────────────────────────────────
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t readWord(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

void calibrateGyro() {
  Serial.println("Calibrating gyro — hold still...");
  long sumX = 0;
  for (int i = 0; i < 1000; i++) {
    sumX += readWord(0x43);
    delay(1);
  }
  gyroXOffset = sumX / 1000.0f;
  Serial.print("Gyro offset: ");
  Serial.println(gyroXOffset);
}

// ─────────────────────────────────────────────────────────
// Motor output
// ─────────────────────────────────────────────────────────
void setMotorA(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, pwm);
  } else if (pwm < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, -pwm);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, 0);
  }
}

// ─────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1C, 0x00);
  mpuWrite(0x1B, 0x00);
  delay(100);

  calibrateGyro();

  float ay = readWord(0x3D) / 16384.0f;
  float az = readWord(0x3F) / 16384.0f;
  pitch = atan2(ay, az) * 180.0f / PI;

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);

  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), isrA, CHANGE);

  lastTime    = micros();
  lastRPMTime = micros();

  Serial.println("Ready.");
}

// ─────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  // ── 1. Read IMU ────────────────────────────────────────
  float ay = readWord(0x3D) / 16384.0f;
  float az = readWord(0x3F) / 16384.0f;
  float gx = (readWord(0x43) - gyroXOffset) / 131.0f;

  float accelAngle = atan2(ay, az) * 180.0f / PI;
  pitch = ALPHA * (pitch + gx * dt) + (1.0f - ALPHA) * accelAngle;

  // ── 2. Compute RPM (every 20ms) ────────────────────────
  unsigned long rpmNow = micros();
  float rpmDt = (rpmNow - lastRPMTime) / 1000000.0f;
  if (rpmDt >= 0.02f) {
    long deltaA = encCountA - lastEncA;
    lastEncA = encCountA;
    lastRPMTime = rpmNow;
    rpmA = (deltaA / COUNTS_PER_REV) / rpmDt * 60.0f;
  }

  // ── 3. Outer PID — angle → target RPM ─────────────────
  angleError     = pitch - setpoint;
  angleIntegral += angleError * dt;
  angleIntegral  = constrain(angleIntegral, -50, 50);
  float angleDeriv = (angleError - angleLastError) / dt;
  angleLastError = angleError;

  targetRPM = Kp_angle * angleError
            + Ki_angle * angleIntegral
            + Kd_angle * angleDeriv;
  targetRPM = constrain(targetRPM, -MAX_RPM, MAX_RPM);

  // ── 4. Inner PID — velocity → PWM ─────────────────────
  velError     = targetRPM - rpmA;
  velIntegral += velError * dt;
  velIntegral  = constrain(velIntegral, -100, 100);
  float velDeriv = (velError - velLastError) / dt;
  velLastError = velError;

  int pwmA = (int)(Kp_vel * velError
                 + Ki_vel * velIntegral
                 + Kd_vel * velDeriv);
  pwmA = constrain(pwmA, -255, 255);

  setMotorA(pwmA);

  // ── 5. Serial print every 100ms ───────────────────────
  static unsigned long lastPrint = 0;
  if (micros() - lastPrint >= 100000) {
    lastPrint = micros();
    Serial.print("Pitch: ");   Serial.print(pitch, 2);
    Serial.print("  Err: ");   Serial.print(angleError, 2);
    Serial.print("  tRPM: ");  Serial.print(targetRPM, 1);
    Serial.print("  rpmA: ");  Serial.print(rpmA, 1);
    Serial.print("  pwmA: ");  Serial.println(pwmA);
  }
}