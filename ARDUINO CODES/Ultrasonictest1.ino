#include <Servo.h>
Servo servo;
// INCLUDING THE LIBRARY TO MOVE THE STEERING SERVO

#define ENB 6;
// DEFINING THE PIN FOR CONTROLLING THE SPEED OF THE CAR

const int motorspeed = 130;
// DEFINING THE VARIABLE FOR THE SPEED OF THE MOTOR

const int trigr = 11;
const int echor = 10;
float durationr, distancer;
// DEFINING VARIABLES FOR DETECTION USING ULTRASONIC SENSOR ON THE RIGHT

void setup() 
{
  pinMode(2, OUTPUT);
  // NEGATIVE PIN OF THE MOTOR
  pinMode(3, OUTPUT);
  // POSITIVE PIN OF THE MOTOR
  pinMode(9, OUTPUT);
  // SERVO OF THE STEERING
  servo.attach(9);
  servo.write(100);
  // SETTING THE SERVO TO ITS DEFAULT SETTING

  pinMode(trigr, OUTPUT);
  pinMode(echor, INPUT);
  // DEFINING THE TRIGPIN AND ECHOPIN AS INPUT AND OUTPUT

  Serial.begin(9600);

  analogWrite(6, motorspeed);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  // STARTING THE MOTOR
}

void loop() 
{
   digitalWrite(trigr, LOW);  
	delayMicroseconds(1);  
	digitalWrite(trigr, HIGH);  
	delayMicroseconds(5);  
	digitalWrite(trigr, LOW);

  durationr = pulseIn(echoPinr, HIGH);

  distancer = (durationr * .343)/20;

  if ( 0<distancer<15 )
  {
    servo.write(105);
    delay(100);
    servo.write(95);
    delay(100);
    servo.write(100);
  }

  else if ( 15<distance<30 )
  {
    servo.write(95);
    delay(100);
    servo.write(105);
    delay(100);
    servo.write(100);
  }
}
