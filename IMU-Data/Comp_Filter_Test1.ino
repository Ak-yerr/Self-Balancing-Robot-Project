#include <Wire.h>
#define MPU_ADDR 0x68

float angle = 0.0; // variable to store tuned angle
unsigned long lastTime = 0; // time in microseconds since the last rotation

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  lastTime = micros();  }

void loop() {
  // read raw data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t accX  = Wire.read()<<8 | Wire.read();
  int16_t accY  = Wire.read()<<8 | Wire.read();
  int16_t accZ  = Wire.read()<<8 | Wire.read();
  int16_t temp  = Wire.read()<<8 | Wire.read();
  int16_t gyroX = Wire.read()<<8 | Wire.read();
  int16_t gyroY = Wire.read()<<8 | Wire.read();
  int16_t gyroZ = Wire.read()<<8 | Wire.read();
  // scale to deg/secs
  float ax = accX / 16384.0;
  float ay = accY / 16384.0;
  float az = accZ / 16384.0;
  float gx = gyroX / 131.0;
  // time change
  unsigned long now = micros(); // keeping track of current timing
  float dt = (now-lastTime)/1000000.0; // change in time
  lastTime = now;
  // accel angle
  float accelAngle = atan2(ay, az) * 180.0 / PI;
  // complementary filter equation 
  // 98% of our angle tilt values comes from the raw gyroscope data
  // 2% comes from the accelerometer; this dilutes noise
  // 0.98/0.02 values/percentages can be tweaked if needed
  angle = 0.98 * (angle + gx * dt) + 0.02 * accelAngle;

  Serial.print(accelAngle);
  Serial.print("\t");
  Serial.println(angle);

  delay(10);
}