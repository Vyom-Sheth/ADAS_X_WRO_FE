#include <Servo.h>
Servo servo;

#define ENB 6;

const int motorspeed = 130;

const int trigPinr = 11;

const int echoPinr = 10;

const int echoPinf = 7;

const int trigPinf = 8;

float durationr, distancer;

float durationf, distancef;

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
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);

  pinMode(trigPinf, OUTPUT);
  pinMode(echoPinf, INPUT);
  
  analogWrite(6, motorspeed);

  Serial.begin(9600);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);

}

void loop() 
{
  
  digitalWrite(trigPinr, LOW);  
	delayMicroseconds(1);  
	digitalWrite(trigPinr, HIGH);  
	delayMicroseconds(5);  
	digitalWrite(trigPinr, LOW);

  durationr = pulseIn(echoPinr, HIGH);

  distancer = (durationr * .343)/20;

  digitalWrite(trigPinf, LOW);  
	delayMicroseconds(1);  
	digitalWrite(trigPinf, HIGH);  
	delayMicroseconds(5);  
	digitalWrite(trigPinf, LOW);

  durationf = pulseIn(echoPinr, HIGH);

  distancef = (durationf * .343)/20;

  Serial.print("Distance: ");
  Serial.println(distancer);

  distancer = map(distancer, 0 , 15 , 80 , 120);

  servo.write(distancer);


//  if (distancef > 15)
//  {

//  digitalWrite(2, LOW);
//  digitalWrite(3, HIGH);

//  }

//  else if (distancef < 15)
//  {
  //  digitalWrite(2, LOW);
    //digitalWrite(3, LOW);
  //}

  
}