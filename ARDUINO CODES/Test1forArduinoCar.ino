#include <Servo.h>
Servo servo;

#define ENB 5;

const int motorSpeed = 175;

void setup() 
{
  pinMode(2, OUTPUT);
  //NEGATIVE OF MOTOR
  pinMode(3, OUTPUT);
  //POSITIVE OF MOTOR
  pinMode(9, OUTPUT);
  //MOTOR SENSOR PIN
  servo.attach(9);
  servo.write(100);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  analogWrite(5, motorSpeed);

  delay(2000);

  servo.write(150);

  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);

  delay(30000);

  digitalWrite(2, LOW);
  digitalWrite(3,LOW);
}

void loop() {}
