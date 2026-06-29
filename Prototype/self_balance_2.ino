#include <Wire.h>
#include <MPU6050.h>   // Use the "MPU6050 by Electronic Cats" or "MPU6050" library

MPU6050 mpu;

// --- Motor A pins ---
#define PWMA  25
#define AIN1  26
#define AIN2  27

// --- Motor B pins (direction inverted in software) ---
#define PWMB  16
#define BIN1  17
#define BIN2  18

// --- Tuning ---
#define SETPOINT     90.0   // Upright angle in degrees (from prototype 1)
#define DEADBAND      2.0   // Degrees of tilt before motors activate
#define MOTOR_SPEED  250     // Fixed speed 0–255 for this test

// --- Complementary filter state ---
float pitch = 0.0;
unsigned long lastTime = 0;
const float ALPHA = 0.98;   // Gyro trust weight

// --- Gyro calibration offsets (from prototype 1 — re-measure if needed) ---
float gyroX_offset = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection FAILED");
    while (true);
  }
  Serial.println("MPU6050 OK");

  // Motor pin setup
  ledcAttach(PWMA, 1000, 8);   // 1kHz, 8-bit (0–255)
  ledcAttach(PWMB, 1000, 8);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  stopMotors();

  // Gyro calibration: average 500 samples at rest
  Serial.println("Calibrating gyro — hold robot STILL...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gx;
    delay(2);
  }
  gyroX_offset = sum / 500.0;
  Serial.print("Gyro offset: ");
  Serial.println(gyroX_offset);

  // Seed pitch from accelerometer
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitch = atan2((float)ay, (float)az) * 180.0 / PI;

  lastTime = micros();
  Serial.println("Ready. Tilt the robot to test motors.");
}

void loop() {
  // --- 1. Read IMU ---
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  lastTime = now;

  float gyroRate = ((float)gx - gyroX_offset) / 131.0; // deg/s
  float accelPitch = atan2((float)ay, (float)az) * 180.0 / PI;

  pitch = ALPHA * (pitch + gyroRate * dt) + (1.0 - ALPHA) * accelPitch;

  // --- 2. Print angle ---
  Serial.println(pitch);

  // --- 3. Drive motors based on tilt ---
  float error = pitch - SETPOINT;

  if (error > DEADBAND) {
    // Tilting forward → drive forward
    driveMotors(MOTOR_SPEED, MOTOR_SPEED);
  } else if (error < -DEADBAND) {
    // Tilting backward → drive backward
    driveMotors(-MOTOR_SPEED, -MOTOR_SPEED);
  } else {
    stopMotors();
  }

  // No delay() here — loop runs as fast as possible
}

// Positive speed = forward, negative = backward
void driveMotors(int speedA, int speedB) {
  // Motor A
  if (speedA >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speedA = -speedA;
  }
  ledcWrite(PWMA, speedA);

  // Motor B — direction inverted
  if (speedB >= 0) {
    digitalWrite(BIN1, LOW);   // swapped
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH);  // swapped
    digitalWrite(BIN2, LOW);
    speedB = -speedB;
  }
  ledcWrite(PWMB, speedB);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}