// The following defines the ESP32 pins used to control the TB6612
// Motor A
const uint16_t PWMA = 25;  // Motor A PWM control
const uint16_t AIN2 = 17;  // Motor A input 2
const uint16_t AIN1 = 21;  // Motor A input 1

// Motor B
const uint16_t BIN1 = 22;  // Motor B input 1
const uint16_t BIN2 = 23;  // Motor B input 2
const uint16_t PWMB = 26;  // Motor B PWM control

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize motor control pins
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Function to control Motor A
void motorA(float pwmInputA) {
  int pwmIntA = round(pwmInputA);
  if (pwmIntA == 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);  // Stop motor
  } else if (pwmIntA > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, constrain(pwmIntA, 0, 255));
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, constrain(-pwmIntA, 0, 255));
  }
}

// Function to control Motor B
void motorB(float pwmInputB) {
  int pwmIntB = round(pwmInputB);
  if (pwmIntB == 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);  // Stop motor
  } else if (pwmIntB > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, constrain(pwmIntB, 0, 255));
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, constrain(-pwmIntB, 0, 255));
  }
}

void loop() {
  // Motor stops for 3 seconds
  motorA(0);
  motorB(0);
  delay(3000);

  // Motor reverses direction and turns at low speed for 3 seconds
  motorA(-64);
  motorB(-64);
  delay(3000);

  // Motor positive direction and turns at high speed for 3 seconds
  motorA(255);
  motorB(255);
  delay(3000);
}
