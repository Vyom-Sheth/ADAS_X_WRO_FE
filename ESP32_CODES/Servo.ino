#include <ESP32Servo.h>
Servo servo;

void setup() {
  pinMode(4, OUTPUT);
  servo.attach(4);

  servo.write(20);
}

void loop() {
  // put your main code here, to run repeatedly:

}
