#include <Servo.h>
Servo servo;
// INCLUDING THE LIBRARY TO MOVE THE STEERING SERVO

#define ENB 6;
// DEFINING THE PIN FOR CONTROLLING THE SPEED OF THE CAR

const int motorspeed = 135;
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

  durationr = pulseIn(echor, HIGH);

  distancer = (durationr * .343)/20;

  Serial.print("Distance:");
  Serial.println(distancer);

  if ( 0<distancer<25 )
  {
    servo.write(120);
    delay(250);
    servo.write(80);
    delay(200);
    servo.write(100);
    delay(200);
  }

  else if(25<distancer<35)
  {
    servo.write(100);
    delay(250);    
  }

  else if ( 35<distancer<60 )
  {
    servo.write(80);
    delay(250);
    servo.write(120);
    delay(200);
    servo.write(100);
    delay(200);
  }
}


