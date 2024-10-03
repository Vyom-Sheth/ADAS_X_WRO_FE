#include <Servo.h>
Servo servo;

#define ENB 5;

const int mst = 175;

void setup()
{
  pinMode(2, OUTPUT);
  //NEGATIVE OF MOTOR
  pinMode(3, OUTPUT);
  //POSITIVE OF MOTOR
  pinMode(9, OUTPUT);
  //SERVO SENSOR PIN
  analogWrite(5, mst);

  servo.attach(9);
  //Attaching servo to pin 9
  servo.write(100);
  //Setting the initial position for servo

  delay(2000);

  servo.write(150);
  analogWrite(5, mst);
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);

  delay(1475);

  digitalWrite(3, LOW);
  digitalWrite(2, LOW);
  servo.write(100);

}

void loop() 
{
  analogWrite(5, mst);
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);

  delay(1000);

  digitalWrite(3, LOW);
  digitalWrite(2, LOW);

  delay(500);

  servo.write(150);
  analogWrite(5, mst);
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);

  delay(1000);

  digitalWrite(3, LOW);
  digitalWrite(2, LOW);
  servo.write(100);
}
