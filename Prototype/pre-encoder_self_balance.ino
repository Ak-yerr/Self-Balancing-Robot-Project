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

#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// IMU
float gyroXOffset = 0;
float pitch = 0.0f;
unsigned long lastTime = 0;

// PID
float setpoint = 82.5f;  // upright angle — adjust after first boot
float Kp = 30.0f; // reaction intensity
float Ki = 0.0f; // can ignore (for now)
float Kd = 0.5f; // switching intensity
float Kc = 0.05f; // for parabolic change in motor speed

float error, lastError = 0, integral = 0, derivative;

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

void setMotors(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Forward
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, speed);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);  // B inverted
    ledcWrite(PWMB, speed);

  } else if (speed < 0) {
    // Reverse
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, -speed);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // B inverted
    ledcWrite(PWMB, -speed);

  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); ledcWrite(PWMA, 0);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); ledcWrite(PWMB, 0);
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

  lastTime = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;

  int16_t rawAx = readWord(0x3B);
  int16_t rawAy = readWord(0x3D);
  int16_t rawAz = readWord(0x3F);
  int16_t rawGx = readWord(0x43);

  float ax = rawAx / 16384.0f;
  float ay = rawAy / 16384.0f;
  float az = rawAz / 16384.0f;
  float gx = (rawGx - gyroXOffset) / 131.0f;

  float accelAngle = atan2(ay, az) * 180.0f / PI;
  pitch = ALPHA * (pitch + gx * dt) + (1.0f - ALPHA) * accelAngle;

  // PID
  error = pitch - setpoint;
  integral += error * dt;
  derivative = (error - lastError) / dt;
  lastError = error;

float output = Kp * error + Kc * error * error * error + Kd * derivative;
setMotors((int)output);

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("  Error: "); Serial.print(error);
  Serial.print("  Output: "); Serial.println(output);

  delay(3);
}