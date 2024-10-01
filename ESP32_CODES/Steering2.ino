#include <ESP32Servo.h>
Servo servo;

const uint16_t PWMA = 25;
const uint16_t AIN2 = 17;
const uint16_t AIN1 = 21;

const uint16_t ANALOG_WRITE_BITS = 8;

int freq = 5000;
int channel_A = 2;
int resolution = ANALOG_WRITE_BITS;

void initmotors()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  ledcAttachChannel(PWMA, freq, resolution, channel_A);
}

void forward(uint16_t pwm)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void stop(uint16_t pwm)
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void setup() 
{
  pinMode(4, OUTPUT);           //Servo motor
  servo.attach(4);
  servo.write(100);

  initmotors();
}

void loop() 
{
  forward(250);
  servo.write(180);
  delay(1000);
  forward(500);
  servo.write(20);
  delay(1000);
  forward(500);
  servo.write(100);
  delay(1000);
}
