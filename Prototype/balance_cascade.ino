#include <Wire.h>

#define MPU_ADDR 0x68
#define ALPHA 0.98f

// Motor A
#define AIN1 26
#define AIN2 27
#define PWMA 25

// Motor B
#define PWMB 16
#define BIN1 17
#define BIN2 18

// Encoders
#define ENCA_A 34
#define ENCA_B 35
#define ENCB_A 32
#define ENCB_B 33

#define COUNTS_PER_REV 1980.0f

#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// IMU
float gyroXOffset = 0;
float pitch = 0.0f;
unsigned long lastTime = 0;

// ---------------- OUTER LOOP (angle -> target RPM) ----------------
float setpoint = 82.5f;   // upright angle — adjust after first boot

float Kp_outer = 30.0f;
float Ki_outer = 0.0f;
float Kd_outer = 0.5f;
float Kc_outer = 0.05f;   // cubic term for stronger response at large lean

float angleError, lastAngleError = 0, angleIntegral = 0, angleDerivative;

#define MAX_TARGET_RPM 270.0f   // outer loop output is clamped to this

// ---------------- INNER LOOPS (RPM -> PWM), one per motor ----------------
float Kp_inner = 4.0f;
float Ki_inner = 0.5f;
float Kd_inner = 0.0f;

float rpmErrorA, lastRpmErrorA = 0, rpmIntegralA = 0, rpmDerivativeA;
float rpmErrorB, lastRpmErrorB = 0, rpmIntegralB = 0, rpmDerivativeB;

// Anti-windup clamp for the inner-loop integrators
#define INNER_INTEGRAL_LIMIT 200.0f

// ---- Encoder state ----
static const int8_t quadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

long lastCountA = 0;
long lastCountB = 0;
float rpmA = 0.0f;
float rpmB = 0.0f;

unsigned long lastPrint = 0;

void IRAM_ATTR encoderISR_A() {
  uint8_t a = digitalRead(ENCA_A);
  uint8_t b = digitalRead(ENCA_B);
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (stateA << 2) | curr;
  encoderCountA += quadTable[idx];
  stateA = curr;
}

void IRAM_ATTR encoderISR_B() {
  uint8_t a = digitalRead(ENCB_A);
  uint8_t b = digitalRead(ENCB_B);
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (stateB << 2) | curr;
  encoderCountB += quadTable[idx];
  stateB = curr;
}

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
  long sumX = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    sumX += readWord(0x43);
    delay(1);
  }
  gyroXOffset = sumX / (float)samples;
}

// Low-level: send a signed PWM value straight to a motor
void driveMotor(bool isMotorA, int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);

  if (isMotorA) {
    if (pwmVal > 0) {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    } else if (pwmVal < 0) {
      digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    }
    ledcWrite(PWMA, abs(pwmVal));
  } else {
    // Motor B direction inverted in software
    if (pwmVal > 0) {
      digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    } else if (pwmVal < 0) {
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    } else {
      digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    }
    ledcWrite(PWMB, abs(pwmVal));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1C, 0x00);
  mpuWrite(0x1B, 0x00);
  delay(100);

  calibrateGyro();

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);

  pinMode(ENCA_A, INPUT);
  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_A, INPUT);
  pinMode(ENCB_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), encoderISR_B, CHANGE);

  lastTime = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0) return;   // guard against a zero/garbage dt on the first pass

  // ---------------- Read tilt (complementary filter) ----------------
  int16_t rawAy = readWord(0x3D);
  int16_t rawAz = readWord(0x3F);
  int16_t rawGx = readWord(0x43);

  float ay = rawAy / 16384.0f;
  float az = rawAz / 16384.0f;
  float gx = (rawGx - gyroXOffset) / 131.0f;

  float accelAngle = atan2(ay, az) * 180.0f / PI;
  pitch = ALPHA * (pitch + gx * dt) + (1.0f - ALPHA) * accelAngle;

  // ---------------- Read wheel speed ----------------
  long countA = encoderCountA;
  long countB = encoderCountB;
  long deltaA = countA - lastCountA;
  long deltaB = countB - lastCountB;
  lastCountA = countA;
  lastCountB = countB;

  rpmA = (deltaA / COUNTS_PER_REV) / dt * 60.0f;
  rpmB = (deltaB / COUNTS_PER_REV) / dt * 60.0f;

  // ======================================================
  // OUTER LOOP: angle error -> target RPM (same for both wheels)
  // ======================================================
  angleError = pitch - setpoint;
  angleIntegral += angleError * dt;
  angleDerivative = (angleError - lastAngleError) / dt;
  lastAngleError = angleError;

  float targetRPM = Kp_outer * angleError
                   + Ki_outer * angleIntegral
                   + Kd_outer * angleDerivative
                   + Kc_outer * angleError * angleError * angleError;

  targetRPM = constrain(targetRPM, -MAX_TARGET_RPM, MAX_TARGET_RPM);

  // ======================================================
  // INNER LOOP A: target RPM -> PWM for Motor A
  // ======================================================
  rpmErrorA = targetRPM - rpmA;
  rpmIntegralA = constrain(rpmIntegralA + rpmErrorA * dt,
                            -INNER_INTEGRAL_LIMIT, INNER_INTEGRAL_LIMIT);
  rpmDerivativeA = (rpmErrorA - lastRpmErrorA) / dt;
  lastRpmErrorA = rpmErrorA;

  float pwmA = Kp_inner * rpmErrorA + Ki_inner * rpmIntegralA + Kd_inner * rpmDerivativeA;

  // ======================================================
  // INNER LOOP B: target RPM -> PWM for Motor B
  // ======================================================
  rpmErrorB = targetRPM - rpmB;
  rpmIntegralB = constrain(rpmIntegralB + rpmErrorB * dt,
                            -INNER_INTEGRAL_LIMIT, INNER_INTEGRAL_LIMIT);
  rpmDerivativeB = (rpmErrorB - lastRpmErrorB) / dt;
  lastRpmErrorB = rpmErrorB;

  float pwmB = Kp_inner * rpmErrorB + Ki_inner * rpmIntegralB + Kd_inner * rpmDerivativeB;

  // ---------------- Drive motors ----------------
  driveMotor(true,  (int)pwmA);
  driveMotor(false, (int)pwmB);

  // ---------------- Rate-limited telemetry ----------------
  if (millis() - lastPrint >= 50) {
    lastPrint = millis();
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print("  TargetRPM: "); Serial.print(targetRPM);
    Serial.print("  RPM_A: "); Serial.print(rpmA);
    Serial.print("  RPM_B: "); Serial.print(rpmB);
    Serial.print("  PWM_A: "); Serial.print(pwmA);
    Serial.print("  PWM_B: "); Serial.println(pwmB);
  }
}
