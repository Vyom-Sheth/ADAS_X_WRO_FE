#include <Servo.h>

const int ENB = 6;

const int trigr = 11;
const int echor = 10;

const int motorspeed = 140;

Servo servo;

float durationr, distancer ;

void setup() 
{
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);

  servo.attach(9);
  servo.write(100);

  pinMode(trigr, OUTPUT);
  pinMode(echor, INPUT);

  Serial.begin(9600);

  analogWrite(6, motorspeed);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
}

void loop() 
{
  measuredistancer();

  Serial.print("Distance: ");
  Serial.println(distancer);

  if (distancer < 20)
  {
    servo.write(700);
    delay(500);
  }

  else if (distancer > 30)
  {
    servo.write(130);
    delay(500);
  }

  else if (20 < distancer < 30)
  {
    servo.write(100);
    delay(500);
  }
}

void measuredistancer() {
  // Send pulse to trigger the ultrasonic sensor
  digitalWrite(trigr, LOW);
  delayMicroseconds(2);
  digitalWrite(trigr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigr, LOW);

  // Measure the echo duration to calculate distance
  durationr = pulseIn(echor, HIGH);
  distancer = durationr * 0.034 / 2;  // Convert duration to distance in cm
}