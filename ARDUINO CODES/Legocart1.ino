#include <Servo.h>
Servo servo;

#define ENB 6;

const int motorspeed = 150;

void setup() 
{
  pinMode(9, OUTPUT);
  servo.attach(9);
  pinMode(2, OUTPUT);
//NEGATIVE OF THE MOTOR
  pinMode(3, OUTPUT);
//POSITIVE OF THE MOTOR
  analogWrite(6, motorspeed);
  servo.write(100);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
