#include <Wire.h>
void setup() {
  Wire.begin(21, 22); // SDA, SCL
  Serial.begin(115200);
  delay(1000);
  Serial.println("Scanning I2C...");
}
void loop() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found");
  delay(2000);
}