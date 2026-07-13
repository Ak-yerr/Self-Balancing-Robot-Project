# Self-Balancing Two-Wheeled Robot

**Status: Complete.** Prototype 4 is the final version of this project.

This is an inverted-pendulum balancing robot. An ESP32 reads pitch angle from an IMU, runs a PID control loop, and drives two DC gear motors with quadrature encoder feedback to stay upright. The project went through four hardware prototypes. Prototype 4 is the first to pass all three functionality tests defined for the project, and no further prototype is planned.

---

## Table of Contents

- [Hardware](#hardware)
- [Wiring](#wiring)
- [Chassis Design](#chassis-design)
- [Testing Results](#testing-results)
- [Control Architecture](#control-architecture)
- [Calibration](#calibration)
- [Software](#software)
- [Setup](#setup)

---

## Hardware

| Component | Purpose |
|---|---|
| ESP32 DevKitC (HiLetgo, on screw-terminal adapter board) | Main controller. Runs sensor fusion and PID |
| MPU-6050 IMU (pre-soldered breakout) | Accelerometer and gyro for pitch angle estimation |
| TB6612FNG motor driver (pre-soldered breakout) | Dual H-bridge driver for both drive motors |
| 2x XiaoR Geek XR25-370 gear motors with Hall encoders (280 RPM, 1:45) | Drive wheels and encoder feedback |
| AKEYSRC LM2596 buck converter | Steps LiPo voltage down to 5V for ESP32 VIN |
| 3S LiPo, 11.1V, 1400mAh, 50C, XT60 (OVONIC) | Main power source |
| XT60 connector/adapter set | Battery interconnects |
| HTRC balance charger (2S-3S compatible) | LiPo charging |
| 400-point breadboard and jumper wires | Prototyping. All signal and power interconnects |
| Zip ties (assorted) | Cable management and securing modules to the chassis |

Additional components required:

- 100nF ceramic decoupling capacitor across MPU-6050 VCC/GND
- 10µF capacitor across TB6612FNG VM/GND

Alkaline 9V batteries were tested early on. They cannot sustain motor current without voltage sag. A LiPo pack is required.

---

## Wiring

### Pin assignments

| Function | ESP32 GPIO |
|---|---|
| IMU I2C (SDA/SCL) | 21 / 22 |
| Motor A (PWM / IN1 / IN2) | 25 / 26 / 27 |
| Motor B (PWM / IN1 / IN2) | 16 / 17 / 18 |
| Encoder A (channel A/B) | 34 / 35 |
| Encoder B (channel A/B) | 32 / 33 |

Motor B was originally wired to GPIO12, an ESP32 strapping pin, which caused flash failures. It has been remapped to GPIO16/17/18 and no longer conflicts with boot mode selection.

### Power

- Raw LiPo (3S, 11.1V) goes to TB6612FNG VM for motor power.
- Raw LiPo also goes to the LM2596 buck converter (set to 5V) then to ESP32 VIN.
- ESP32 3.3V goes to TB6612FNG VCC/STBY for logic power.
- Common ground is mandatory. LiPo, ESP32, TB6612FNG, MPU-6050, and both encoders all share one ground reference.

### Decoupling

- 100nF ceramic cap directly across MPU-6050 VCC/GND
- 10µF electrolytic cap directly across TB6612FNG VM/GND

---

## Chassis Design

### Prototype 1 (retired)

A tall, vertical board layout with components stacked above the axle. This put the center of mass high above the wheels. Prototype 1 could not be stabilized and was retired.

### Prototype 2 (retired)

A flat, single-surface layout with motors, wheels, and LiPo on the bottom face and electronics on top. This lowered the center of mass close to the axle and improved on Prototype 1, but the body was too short and too flat to absorb bumps. It failed on any terrain other than a hard, even surface.

### Prototype 3 (retired)

A refinement of the flat layout. It passed on hardwood but still failed on carpet and bumpy terrain, for the same reason as Prototype 2: the body sat too low and too rigid to tolerate small changes in ground height.

### Prototype 4 (final)

Prototype 4 raises the center of mass slightly above where Prototype 2 and 3 held it. This was the key change: a body that is a little taller tolerates small terrain irregularities without oscillating out of control, while staying low enough to keep the balancing problem manageable. Combined with the PID tuning described below, this is the first version to pass sustained balancing, disturbance recovery, and terrain variation.

---

## Testing Results

Three functionality tests were defined for this project: sustained balancing, disturbance recovery, and terrain performance. Prototype 4 is the first to pass all three.

### Balancing capability

Requirement: 60 seconds of continuously sustained balancing.

Five trials were run. The first and third ran for around 100 seconds, well past every result from Prototypes 1 through 3. The second, fourth, and fifth trials did not fall at all. The robot found a balance point where the motors stopped moving because it had settled at an angle of perfect balance. Earlier prototypes oscillated continuously until they eventually fell.

### Disturbance recovery

This was the hardest of the three tests. It measures whether the robot can recover from a physical "tap" rather than just hold still. Prototype 4 was the first to show an actual corrective PID response instead of static balance alone.

Across five tap trials, it recovered on three and fell on two. The three successful recoveries each stabilized within about five seconds. Earlier prototypes either had no corrective response at all, or overcorrected and fell the opposite way. This result is a real success with a real limitation. The motors and chassis available within this project's budget cap how much further the recovery response can be pushed.

### Terrain performance

Requirement: balance on hardwood, an even wool carpet, and a bumpy table carpet.

- Prototypes 1 and 2 failed on all three surfaces.
- Prototype 3 passed only on hardwood.
- Prototype 4 passed on all three, including the bumpy carpet.

The flat, low-body design used in Prototypes 2 and 3 was too short to tolerate any oscillation caused by uneven ground. Raising the center of mass slightly in Prototype 4 was the change that fixed this.

### Summary

Prototype 4 moved from technically balancing on a flat floor to balancing under real-world variation: sustained holds, physical disturbances, and rough terrain. That result marks the end of this project.

---

## Control Architecture

The final control law is a single PID loop running directly on pitch angle, with a cubic error term added for a stronger response at larger angles:

```
error   = pitch - setpoint
output  = Kp * error + Kc * error^3 + Kd * derivative
```

`setpoint` is the robot's measured upright angle (not necessarily 0, since the IMU is never perfectly level relative to the chassis) and is calibrated after the first boot. `Kc` scales how much more aggressively the robot corrects as the tilt grows, rather than responding linearly across the whole range. `Ki` exists in the code but is left at 0 for now.

Quadrature encoders on both motors are decoded in hardware interrupts and converted to RPM every loop. This RPM is used as telemetry to verify motor behavior while tuning, rather than as a second feedback term in the final control law.

### Angle estimation

Pitch is estimated with a complementary filter:

```
pitch = ALPHA * (pitch + gx * dt) + (1 - ALPHA) * accelAngle
```

ALPHA is 0.98. This favors the gyro's short-term accuracy while using the accelerometer to correct long-term drift. Accelerometer and gyro registers are read directly over I2C rather than through a library.

The MPU-6050 is mounted low on the chassis and centered directly over the axle. Mounting it off-axis introduces centripetal and tangential acceleration into the accelerometer reading during movement, which corrupts the angle estimate independent of actual tilt.

---

## Calibration

### Gyro bias calibration (every boot)

The gyro has a small DC bias that varies between power cycles. On every boot, the firmware averages 1000 raw gyro-X samples with the robot held stationary and level, and subtracts that average as an offset. Skipping this step causes the angle estimate to drift even when the robot isn't moving.

### PID tuning

1. Start with `Kp`, `Ki`, `Kd`, and `Kc` at 0 and the robot propped upright.
2. Set `setpoint` to the pitch value printed over serial when the robot is held level and balanced by hand. This is rarely exactly 0.
3. Increase `Kp` until the robot visibly reacts to being tilted.
4. Add `Kd` to damp oscillation.
5. Add a small `Kc` if the robot needs a stronger response at larger tilt angles without over-reacting near the setpoint.
6. Leave `Ki` at 0 unless a persistent steady-state lean shows up, and keep it small if used.

### Encoder scaling

Each encoder is decoded on every A/B transition using a quadrature lookup table, giving full 4x resolution: 1980 counts per output-shaft revolution.

```
rpm = (count_delta / 1980) / dt * 60
```

---

## Software

The firmware is a single Arduino sketch with no external motor or IMU libraries. Everything from I2C register reads to quadrature decoding to PID output is written directly against the ESP32 core APIs. This keeps the loop predictable and avoids library overhead in a control loop where timing matters. The sections below walk through the sketch roughly in the order it executes: constants and pins first, then encoder decoding and IMU access that run continuously in the background, then the motor driver, then the main loop that ties all of it together.

### Pin and constant definitions

Every pin number and tunable value lives at the top of the sketch in one place, so retuning or rewiring never means hunting through the rest of the code. `setpoint` is the pitch angle the robot treats as "upright." It is rarely exactly 0, since the IMU's mounting angle is never perfectly level relative to the chassis, so this value gets measured and set per unit after the first boot. `Kp`, `Ki`, `Kd`, and `Kc` are the four PID terms used in the main loop, covered in more detail in [Control Architecture](#control-architecture):

```cpp
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

float setpoint = 82.5f;  // upright angle, adjust after first boot
float Kp = 30.0f;
float Ki = 0.0f;
float Kd = 0.5f;
float Kc = 0.05f;
```

### Encoder decoding

Each motor's encoder outputs two channels, A and B, that shift phase relative to each other depending on direction. Reading only one channel gives 1x resolution and no direction information. This firmware reads both channels on every edge and combines the previous 2-bit state with the current 2-bit state into a 4-bit index, which is looked up in `quadTable` to get a signed step of -1, 0, or +1. This gives full 4x quadrature resolution and correct direction sign in one lookup, with no branching logic inside the interrupt itself. Both ISRs run on every edge of both channels, so the count updates on every transition rather than only on rising edges of one channel:

```cpp
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
```

### IMU register access

The MPU-6050 is read directly over I2C without a library. `mpuWrite()` writes a single byte to a register, used at boot to wake the sensor and set its accelerometer and gyro ranges. `readWord()` reads a 16-bit value from a register pair, used both for the raw sensor axes in the main loop and for the calibration routine below. `calibrateGyro()` runs once at boot with the robot held level and still, averaging 1000 raw gyro-X samples to find the sensor's DC bias. That bias, `gyroXOffset`, is subtracted from every gyro reading afterward so the angle estimate doesn't slowly drift even when the robot isn't moving. The `delay(1)` inside this function is safe because it only runs once before the control loop starts:

```cpp
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
    delay(1); // fine here, runs once before the control loop starts
  }
  gyroXOffset = sumX / (float)samples;
}
```

### Motor output

`setMotors()` takes a single signed speed value from -255 to 255 and drives both motors from it. The sign sets direction by toggling each motor's IN1/IN2 pins, and the magnitude is written to the PWM channel with `ledcWrite()`. Motor B's IN1/IN2 logic is flipped relative to Motor A so that a positive `speed` drives both wheels the same physical direction, since the two motors are mounted facing opposite ways on the chassis. This keeps the direction inversion contained to one function instead of scattered through the main loop:

```cpp
void setMotors(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, speed);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);  // B inverted
    ledcWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, -speed);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  // B inverted
    ledcWrite(PWMB, -speed);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); ledcWrite(PWMA, 0);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); ledcWrite(PWMB, 0);
  }
}
```

### Main loop

`loop()` runs continuously with no `delay()` anywhere in it, since blocking the loop for even a few hundred milliseconds is long enough for the robot to fall. Each pass does five things in order: measures `dt` since the last pass, reads the IMU and updates the pitch estimate with the complementary filter, reads the encoder counts and converts them to RPM for telemetry, runs the PID calculation and sends the result to `setMotors()`, and finally prints telemetry, but only often enough to be readable rather than on every single pass. That last part is what `millis() - lastPrint >= 50` is doing: it rate-limits Serial output to about 20 times a second without ever blocking the loop the way `delay()` would:

```cpp
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

  long countA = encoderCountA;
  long countB = encoderCountB;
  long deltaA = countA - lastCountA;
  long deltaB = countB - lastCountB;
  lastCountA = countA;
  lastCountB = countB;

  if (dt > 0) {
    rpmA = (deltaA / COUNTS_PER_REV) / dt * 60.0f;
    rpmB = (deltaB / COUNTS_PER_REV) / dt * 60.0f;
  }

  error = pitch - setpoint;
  integral += error * dt;
  derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Kc * error * error * error + Kd * derivative;
  setMotors((int)output);

  if (millis() - lastPrint >= 50) {
    lastPrint = millis();
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print("  Error: "); Serial.print(error);
    Serial.print("  Output: "); Serial.print(output);
    Serial.print("  RPM_A: "); Serial.print(rpmA);
    Serial.print("  RPM_B: "); Serial.println(rpmB);
  }
}
```

---

## Setup

1. Wire the robot per [Wiring](#wiring) above. Double-check common ground across every subsystem.
2. Flash the firmware via USB with the Arduino IDE.
3. Open Serial Monitor at 115200 baud and hold the robot upright and level by hand. Read the printed pitch value and set `setpoint` to that value.
4. Re-flash with the updated `setpoint`.
5. Stand the robot up and let it balance. Tune `Kp`, `Kd`, and `Kc` as needed per [Calibration](#calibration), re-flashing between changes.

This same procedure is what carried the project from an ESP32 flashing with the wrong motor pin to a robot that balances on carpet and recovers from a shove. None of the individual steps are complicated on their own. The setpoint calibration corrects for a physical mounting quirk, the gyro calibration corrects for sensor drift, and the PID tuning corrects for everything else, chassis geometry, motor characteristics, and floor surface, that can't be known ahead of time and has to be found by testing. Getting all three right at once, on hardware built from breadboard and hobby motors rather than purpose-built parts, is what the four prototype iterations were actually for.
