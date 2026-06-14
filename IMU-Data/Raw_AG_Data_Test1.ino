#include <Wire.h>

#define MPU_ADDR 0x68

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // activate MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // power management register
  Wire.write(0);    // set to 0 to activate
  Wire.endTransmission(true);

  Serial.println("AccX\tAccY\tAccZ\tGyroX\tGyroY\tGyroZ");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register for accelerometer data
  Wire.endTransmission(false); // repeating "start" in order to loop
  Wire.requestFrom(MPU_ADDR, 14, true); // covers all 3 accel axes + temperature + all 3 gyro axes, each 2 bytes

  int16_t accX  = Wire.read() << 8 | Wire.read(); // x-axis accel
  int16_t accY  = Wire.read() << 8 | Wire.read(); // y-axis accel
  int16_t accZ  = Wire.read() << 8 | Wire.read(); // z-axis accel
  int16_t temp  = Wire.read() << 8 | Wire.read(); // skipped
  int16_t gyroX = Wire.read() << 8 | Wire.read(); // x-axis gyro
  int16_t gyroY = Wire.read() << 8 | Wire.read(); // y-axis gyro
  int16_t gyroZ = Wire.read() << 8 | Wire.read(); // z-axis gyro

  // scale to real units (degrees/sec); data from MPU6050 Datasheet
  float ax = accX / 16384.0; // ±2g range->16384 LSB/g
  float ay = accY / 16384.0;
  float az = accZ / 16384.0;
  float gx = gyroX / 131.0;  // ±250°/s range->131 LSB/°/s
  float gy = gyroY / 131.0;
  float gz = gyroZ / 131.0;

  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(100);

  // Use tools -> serial plotter to visualize accel data (values 1-3)
}
