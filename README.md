# Self-Balancing-Robot
Two-wheeled self-balancing robot using an ESP32 and MPU-6050 IMU. A hand-coded PID controller processes tilt angle from a complementary filter and drives two DC motors via an L298N to keep the robot upright. Features wireless Bluetooth tuning of PID gains in real time.

The robot is a classic inverted-pendulum balancing robot. An ESP32 reads pitch angle from an IMU, runs a cascade PID control loop, and drives two DC gear motors with encoder feedback to stay upright. The project is currently on Prototype 2. This is a structural redesign built after Prototype 1's tall layout proved unstable.

---

## Table of Contents

- [Hardware](#hardware)
- [Wiring](#wiring)
- [Chassis Design](#chassis-design)
- [Control Architecture](#control-architecture)
- [Calibration](#calibration)
- [Software](#software)
- [Setup](#setup)
- [Known Issues / In Progress](#known-issues--in-progress)
- [What's Next](#whats-next)

---

## Hardware

| Component | Purpose |
|---|---|
| ESP32 DevKitC (HiLetgo, on screw-terminal adapter board) | Main controller. Runs sensor fusion and cascade PID |
| MPU-6050 IMU (pre-soldered breakout) | Accelerometer and gyro for pitch angle estimation |
| TB6612FNG motor driver (pre-soldered breakout) | Dual H-bridge driver for both drive motors |
| 2x XiaoR Geek XR25-370 gear motors with Hall encoders (280 RPM, 1:45) | Drive wheels and closed-loop velocity feedback |
| AKEYSRC LM2596 buck converter | Steps LiPo voltage down to 5V for ESP32 VIN |
| 3S LiPo, 11.1V, 1400mAh, 50C, XT60 (OVONIC) | Main power source. Upgrade over the original 2S pack |
| 2S LiPo, 2200mAh, XT60 (Gens Ace) | Original prototype-1-era pack. Superseded |
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
| Motor B (PWM / IN1 / IN2) | 14 / 12 / 13 |
| Encoder A (channels 1/2) | 34 / 35 |
| Encoder B (channels 1/2) | 32 / 33 |

GPIO12 is an ESP32 strapping pin. It is currently wired to Motor B. It must be HIGH or floating at boot, or the ESP32 fails to flash. Until it is remapped to a non-strapping pin, disconnect Motor B's GPIO12 wire before flashing. Reconnect it afterward.

### Power

- Raw LiPo (3S, 11.1V) goes to TB6612FNG VM for motor power.
- Raw LiPo also goes to the LM2596 buck converter (set to 5V) then to ESP32 VIN.
- ESP32 3.3V goes to TB6612FNG VCC/STBY for logic power.
- Common ground is mandatory. LiPo, ESP32, TB6612FNG, MPU-6050, and both encoders must all share one ground reference. Flat single-surface breadboard layouts can end up with split ground rails by accident. Check this with a multimeter if anything behaves erratically.

### Decoupling

- 100nF ceramic cap directly across MPU-6050 VCC/GND
- 10µF electrolytic cap directly across TB6612FNG VM/GND

---

## Chassis Design

### Prototype 1 (retired)

Prototype 1 used a tall, vertical board layout with components stacked above the axle. This put the center of mass high above the wheels. A high center of mass makes an inverted-pendulum robot fall faster and demands more torque and control bandwidth to catch it. Prototype 1 could not be stabilized and was retired.

### Prototype 2 (current)

Prototype 2 uses a flat, single-surface layout.

- Bottom face: motors, wheels, and LiPo. All heavy mass stays low and close to the axle.
- Top face: buck converter, ESP32, and breadboard.

This layout was checked by hand for a natural static balance point before any firmware was written. One was found. That is a good sign the mass distribution favors stable dynamics.

If Prototype 2 fails to balance after full tuning, the fallback plan is a further redesign with a shorter body, a lower center of mass, and larger wheels. Larger wheels reduce the effective angular acceleration for a given tip angle and buy more reaction time.

---

## Control Architecture

The robot uses a cascade PID structure, standard for self-balancing robots.

```
                 ┌─────────────────┐
IMU angle ──────▶│  Outer loop:    │──▶ target wheel RPM (capped ~270 RPM)
                 │  Angle PID      │
                 └─────────────────┘
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
     ┌──────────────────┐   ┌──────────────────┐
     │ Inner loop:       │   │ Inner loop:       │
     │ Motor A velocity  │   │ Motor B velocity  │
     │ PID (encoder fb)  │   │ PID (encoder fb)  │
     └──────────────────┘   └──────────────────┘
              │                       │
              ▼                       ▼
         Motor A PWM             Motor B PWM
```

The outer loop takes the complementary-filtered pitch angle and outputs a target wheel RPM. Each inner loop takes that target RPM and the motor's actual encoder-derived RPM and outputs a PWM duty cycle. Motor B's direction is inverted in software by swapping IN1/IN2 logic, not by swapping motor leads.

### Angle estimation

Pitch is estimated with a complementary filter:

```
angle = ALPHA * (angle + gyroX * dt) + (1 - ALPHA) * atan2(ay, az)
```

ALPHA is 0.98. This favors the gyro's short-term accuracy while using the accelerometer to correct long-term drift. The MPU-6050 is set to +/-2g accelerometer range and +/-250 deg/s gyro range.

The MPU-6050 must be mounted low on the chassis and centered directly over the axle. Mounting it off-axis introduces centripetal and tangential acceleration into the accelerometer reading during movement. This corrupts the angle estimate independent of actual tilt.

---

## Calibration

### Gyro bias calibration (every boot)

The gyro has a small DC bias that varies between power cycles. On every boot, the firmware runs a 1000-sample calibration routine with the robot held stationary and level. This computes and subtracts the offset. Skipping this step causes the angle estimate to drift even when the robot isn't moving.

### Outer loop (angle PID) tuning

1. Prop the robot upright with motors disabled and angle PID gains near zero.
2. Increase Kp until the robot rocks or twitches around vertical.
3. Add Kd to damp overshoot.
4. Add a small Ki only if there is a persistent steady-state lean. Keep it small. Too much integral windup causes slow oscillation.

### Inner loop (velocity PID) tuning

Tune the two motor velocity loops independently first. Command a fixed target RPM directly and bypass the outer loop. Verify each motor tracks it cleanly via encoder feedback before closing the outer loop. This avoids compounding two untuned loops at once.

### Encoder scaling

11 PPR at the motor shaft, times 45:1 gearbox, times 4 for quadrature, equals 1980 counts per output-shaft revolution.

RPM = (count_delta / 1980) * (60 / dt_seconds)

---

## Software

### Staged testing approach

Firmware is validated in stages. Each stage has an explicit pass/fail check before moving to the next.

1. IMU only. Read raw accel/gyro and confirm calibration and filter output track physical tilt.
2. Motors only. Confirm both motors spin correctly in both directions at commanded PWM.
3. IMU and motors, no control loop. Confirm no interference between subsystems.
4. Full balance loop. Cascade PID closed. Robot attempts to self-balance.

This approach avoids compounded-fault diagnosis. A robot that won't balance is much harder to debug if you don't yet know whether the IMU, the motors, or the control loop is at fault.

### Pin and gain configuration

All pins and tunable gains live in one config block at the top of the sketch:

```cpp
// Motor A
#define MA_PWM 25
#define MA_IN1 26
#define MA_IN2 27

// Motor B
#define MB_PWM 14
#define MB_IN1 12   // strapping pin, disconnect before flashing
#define MB_IN2 13

// Encoders
#define ENC_A1 34
#define ENC_A2 35
#define ENC_B1 32
#define ENC_B2 33

const float ALPHA = 0.98;
const float MAX_RPM = 270.0;
const int   COUNTS_PER_REV = 1980;

// Outer loop: angle -> target RPM
float angleKp = 0, angleKi = 0, angleKd = 0;

// Inner loops: RPM -> PWM
float velKp = 0, velKi = 0, velKd = 0;
```

### Encoder counting

Each encoder channel is read on a hardware interrupt. The ISR only increments a counter. All math happens in the main loop:

```cpp
volatile long countA = 0;
volatile long countB = 0;

void IRAM_ATTR isrEncoderA() {
  int a = digitalRead(ENC_A1);
  int b = digitalRead(ENC_A2);
  countA += (a == b) ? 1 : -1;
}

void IRAM_ATTR isrEncoderB() {
  int a = digitalRead(ENC_B1);
  int b = digitalRead(ENC_B2);
  countB += (a == b) ? -1 : 1;   // inverted for Motor B orientation
}
```

### Gyro calibration

Runs once at boot with the robot held level and still:

```cpp
float gyroXoffset = 0;

void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    sum += mpu.getRotationX();
    delayMicroseconds(500); // fine here, runs once before the control loop starts
  }
  gyroXoffset = sum / 1000.0;
}
```

### Complementary filter

```cpp
float angle = 0;

float updateAngle(float ax, float ay, float az, float gyroXraw, float dt) {
  float gyroX = (gyroXraw - gyroXoffset) * (250.0 / 32768.0); // deg/s
  float accelAngle = atan2(ay, az) * 180.0 / PI;
  angle = ALPHA * (angle + gyroX * dt) + (1 - ALPHA) * accelAngle;
  return angle;
}
```

### Cascade PID

```cpp
float outerLoop(float angle, float dt) {
  static float integral = 0, lastError = 0;
  float error = 0 - angle;           // target angle is 0 (upright)
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float targetRPM = angleKp * error + angleKi * integral + angleKd * derivative;
  return constrain(targetRPM, -MAX_RPM, MAX_RPM);
}

float innerLoop(float targetRPM, float actualRPM, float dt,
                 float &integral, float &lastError) {
  float error = targetRPM - actualRPM;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float pwm = velKp * error + velKi * integral + velKd * derivative;
  return constrain(pwm, -255, 255);
}
```

### Hard rule: no delay() in the control loop

delay() blocks the entire loop, including the balance calculation. A robot that gets delay()'d for even a few hundred milliseconds falls during that window. All timing uses millis()/micros() for non-blocking rate-limiting. delay() is never used in the main loop.

```cpp
unsigned long lastLoopTime = 0;

void loop() {
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1e6;
  lastLoopTime = now;

  float angle = updateAngle(ax, ay, az, gyroXraw, dt);
  float targetRPM = outerLoop(angle, dt);

  float rpmA = (countA / (float)COUNTS_PER_REV) * (60.0 / dt);
  float rpmB = (countB / (float)COUNTS_PER_REV) * (60.0 / dt);
  countA = 0;
  countB = 0;

  float pwmA = innerLoop(targetRPM, rpmA, dt, intA, lastErrA);
  float pwmB = innerLoop(-targetRPM, rpmB, dt, intB, lastErrB); // Motor B inverted

  driveMotor(MA_PWM, MA_IN1, MA_IN2, pwmA);
  driveMotor(MB_PWM, MB_IN1, MB_IN2, pwmB);
  // no delay() here, ever
}
```

### OTA

Over-the-air updates are not in use during active development. The added complexity isn't worth it right now. OTA would also sacrifice the Serial monitor connection needed for live tuning and debugging.

---

## Setup

1. Wire the robot per [Wiring](#wiring) above. Double-check common ground across every subsystem.
2. Before first flash, disconnect Motor B's GPIO12 connection.
3. Confirm the pin block at the top of the sketch matches your wiring:

```cpp
#define MA_PWM 25
#define MA_IN1 26
#define MA_IN2 27
#define MB_PWM 14
#define MB_IN1 12   // disconnected during flashing
#define MB_IN2 13
#define ENC_A1 34
#define ENC_A2 35
#define ENC_B1 32
#define ENC_B2 33
```

4. Flash the firmware via USB with the Arduino IDE or PlatformIO.
5. Reconnect Motor B's GPIO12 wire.
6. Run the staged tests in order. Confirm pass criteria at each stage before proceeding. Start with all PID gains at zero:

```cpp
float angleKp = 0, angleKi = 0, angleKd = 0;
float velKp = 0, velKi = 0, velKd = 0;
```

7. Tune the inner velocity PID loops first, then the outer angle PID loop, adjusting the gain values above and re-flashing between tests.

---

## Known Issues / In Progress

- ESP32 flash failure. Traced to GPIO12 being used for Motor B. Current workaround is disconnecting Motor B before flashing. Long-term fix is remapping to a non-strapping GPIO.
- LM2596 buck converter heats instantly on power-up. This is a short-circuit signature, likely reversed IN/OUT terminals or a stray wire bridging pins. If a component heats instantly, disconnect power immediately. Do not continue troubleshooting live.
- TB6612FNG breakout short circuit. VCC/GND pins landed in the wrong breadboard rows and caused the ESP32 to overheat. Diagnosis is in progress. Needs full re-verification of the breadboard row mapping before repowering.
- Encoder velocity feedback is not yet integrated into the cascade PID loop.
- Full closed-loop balance tuning has not been attempted yet. Blocked on resolving the wiring faults above.

---

## What's Next

- Resolve the TB6612FNG wiring short and confirm clean power delivery
- Finish wiring encoder velocity feedback into the cascade PID loop
- Run a full self-balancing PID tuning session
- If Prototype 2 fails entirely, redesign with a shorter body, lower center of mass, and larger wheels
- Next projects in the series: an MPPT solar charge controller, and a Flappy Bird business card PCB
