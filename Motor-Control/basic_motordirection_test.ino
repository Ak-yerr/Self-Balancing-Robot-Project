// Stage 1: Basic direction test
// TB6612FNG + ESP32, Motor A (ESP32 core 3.x)

#define AIN1  26
#define AIN2  27
#define PWMA  25
#define STBY  14

#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

void motorForward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, speed);
}

void motorReverse(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, speed);
}

void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
}

void loop() {
  Serial.println("Forward");
  motorForward(128);
  delay(2000);

  motorStop();
  delay(1000);

  Serial.println("Reverse");
  motorReverse(128);
  delay(2000);

  motorStop();
  delay(1000);
}