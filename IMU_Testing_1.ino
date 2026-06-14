#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial.println("Scanning...");
// begin scanning for whether the device is found. P21 -> SDA, P22 -> SCL
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
// loop to find the device at address
    if (error == 0) {
      Serial.print("Device found at 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Done.");
}

void loop() {}
