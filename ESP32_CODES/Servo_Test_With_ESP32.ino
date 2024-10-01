#include <ESP32Servo.h>

Servo servo;
int SRX = 18;
int STX = 19;

void setup() 
{

  servo.attach(SRX);

  servo.write(100);
  delay(1000);

  servo.write(150);
  delay(1000);

  servo.write(50);
  delay(1000);
}

void loop() 
{}
