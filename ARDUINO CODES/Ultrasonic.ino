#include <Servo.h>
Servo servo;

#define ENB 5;

const int motorSpeed = 175;

const int trigPin = 11;

const int echoPin = 10;

float duration, distance;

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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void loop() 
{
  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = (duration * .343)/20;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance <= 15)
  {
    servo.write(50);
  }

  if (distance > 15)
  {
    servo.write(150);
  }

  delay(100);
}
