#define I2C_SCL 33
#define I2C_SDA 32

#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <ESP32Servo.h>
#define TCAADDR 0x70 // I2C address of TCA9548A

int dirturn = 175;

int speed = 150;

SFEVL53L1X distanceSensor;

Servo myservo;
//int dirturn=580;

const int buttonPin = 5;

const uint16_t PWMA = 25;         
const uint16_t AIN2 = 17;        
const uint16_t AIN1 = 21;         
const uint16_t ANALOG_WRITE_BITS = 8;
int turns = 0;
int freq = 5000;
int channel_A = 5;
int resolution = ANALOG_WRITE_BITS;
uint16_t LeftS;
uint16_t RightS;

// Control parameters
uint16_t idealDistanceFromWall = 0;  // Ideal distance from the wall in mm
const int maxTurnAngle = 30;  // Maximum turning angle for the servo motor in degrees

// PID parameters
float kp = 2.0;  // Proportional gain
float ki = 0.0;  // Integral gain
float kd = 0.1;  // Derivative gain

float previousError = 0;
float integral = 0;
float dt = 0.05;  // Time step (50ms loop delay)

void tcaSelect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i); // Select the TCA channel
  Wire.endTransmission();
}

// Function to control the steering based on the distance from the wall using PID
void controlSteeringWithPID(int16_t distance) {
  if (distance == -1) return; // && distance >= 1000 && distance == 0) return;  // No valid reading, keep going straight
 
  float error = distance - idealDistanceFromWall; // Calculate error
  float Pout = kp * error; // Proportional term

  integral += error * dt; // Integral term
  float Iout = ki * integral;

  float derivative = (error - previousError) / dt; // Derivative term
  float Dout = kd * derivative;
  
  float output = Pout + Iout + Dout; // PID output
  previousError = error; // Update previous error
  //Serial.print("Left Error:");
  //Serial.println(output);

  int turnAngle = map(output, -200, 200, -maxTurnAngle, maxTurnAngle); // Map output to steering angle
  turnAngle = constrain(turnAngle, -maxTurnAngle, maxTurnAngle); // Constrain angle

  int servoAngle = 115 + turnAngle; // Set servo angle
  
  myservo.write(servoAngle); // Set steering
  delay(10);
}

void controlSteeringWithPIDR(int16_t distance) {
  if (distance == -1) return; // && distance >= 1000 && distance == 0) return;  // No valid reading, keep going straight
 
  float error = distance - idealDistanceFromWall; // Calculate error
  float Pout = kp * error; // Proportional term

  integral += error * dt; // Integral term
  float Iout = ki * integral;

  float derivative = (error - previousError) / dt; // Derivative term
  float Dout = kd * derivative;
  
  float output = Pout + Iout + Dout; // PID output
  previousError = error; // Update previous error
  //Serial.print("Right Error:");
  //Serial.println(output);

  int turnAngle = map(output, -175, 175, -maxTurnAngle, maxTurnAngle); // Map output to steering angle
  turnAngle = constrain(turnAngle, -maxTurnAngle, maxTurnAngle); // Constrain angle

  int servoAngle = 115 - turnAngle; // Set servo angle
  
  myservo.write(servoAngle); // Set steering
  delay(10);
}

void initMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  ledcAttachChannel(PWMA, freq, resolution, channel_A);
}

void forwardA(uint16_t pwm) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void stopA() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 0);
}

void setup() {
  

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect
  }
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  myservo.attach(4);



  tcaSelect(2);                               // Left side Sensor
  if (distanceSensor.begin() != 0) {
    Serial.println("Sensor 3 (channel 5) not found.");
  } else {
    Serial.println("Sensor 3 (channel 5) found and initialized.");
    distanceSensor.setDistanceModeLong();  // Set to long-range mode
    distanceSensor.setTimingBudgetInMs(50);  // Short timing budget for faster reads
    distanceSensor.startRanging();
  }
  
  delay(100);
  
  // Select TCA9548A channel 7 (sensor 2)
  tcaSelect(3);                               // Right side Sensor
  if (distanceSensor.begin() != 0) {
    Serial.println("Sensor 2 (channel 7) not found.");
  } else {
    Serial.println("Sensor 2 (channel 7) found and initialized.");
    distanceSensor.setDistanceModeLong();
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.startRanging();
  }
  
  delay(100);
  // Select and read from sensor 2 (channel 7)
    tcaSelect(2);                                       // Left Sensor
    delay(10);
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    uint16_t distance2 = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();
    Serial.print("Sensor 2 (channel 7) Distance (mm): ");
    Serial.println(distance2);
    delay(10);

    tcaSelect(3);                                         // Right Sensor 
    delay(10);
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    uint16_t distance3 = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();
    delay(10);

    LeftS = distance2; 
    RightS = distance3;     // save initial distance

    initMotors();
    myservo.write(115);
    delay(200);
    while(digitalRead(buttonPin) == HIGH)
    {
      delay(100);
    }
    forwardA(speed); // Start robot forward

}

void loop() {
//  int16_t distance = getWallDistance(); // Measure distance from the wall
// Select and read from sensor 1 (channel 6)
  while(turns<=11){

    tcaSelect(3);                         // Rigth Sensor
    delay(1);
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    uint16_t distance3 = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();
    Serial.print("Sensor 3 Distance (mm): ");
    Serial.println(distance3);
    delay(1);

    
    // Select and read from sensor 2 (channel 7)
    tcaSelect(2);                         // left sensor
    delay(1);
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    uint16_t distance2 = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();
    Serial.print("Sensor 2 Distance (mm): ");
    Serial.println(distance2);
    delay(5);

    if (turns==0)
    {
      if((distance2>LeftS+500)||(distance3>RightS+500))
      {
        if(distance2>LeftS+500)
        {
          dirturn=165;
        }
        else if(distance3>RightS+500)
        {
          dirturn=65;
        }
        
        //stopA();                // debug
        //delay(250);

        myservo.write(dirturn); // Turning left
        delay(50);
        forwardA(speed);
        delay(1350);
        stopA(); // Stop motors
        
        myservo.write(115);
        delay(1);
        forwardA(speed);
        delay(1000); // Reset steering
        stopA();                // debug

        //delay(250);            // debug
        //forwardA(speed);        // debug
        turns+=1;
      }
      else {
        idealDistanceFromWall = LeftS;
        controlSteeringWithPID(distance2); // Adjust steering based on distance using PID
        forwardA(speed); // Move the robot forward
      }
      
    }
    else{
      idealDistanceFromWall = 200;  // Ideal distance from the wall LEFTSIDE
      if(dirturn==165){
        if (distance2 != -1) {
          if (distance2 > 750 ) 
          {
            //stopA();                // debug
            //delay(250);            // debug
            
            myservo.write(dirturn); // Turning left
            delay(1);
            forwardA(speed);
            delay(1250);
            stopA();

            myservo.write(115);
            delay(1);
            forwardA(speed);
            delay(1000); // Reset steering
            //stopA();                // debug
            //delay(250);            // debug
            //forwardA(speed);        // debug
            turns+=1;
          } else {
            
            controlSteeringWithPID(distance2); // Adjust steering based on distance using PID
            forwardA(speed); // Move the robot forward
          }
        }
      }
      else{
        idealDistanceFromWall = 300;  // Ideal distance from the wall RIGHT SIDE
        if (distance3 != -1) {
          if (distance3 > 1000) {
            //stopA();                // debug
            //delay(250);            // debug

            myservo.write(dirturn);
            delay(1);
            forwardA(speed);
            delay(1450);
            stopA();

            myservo.write(115);
            delay(1);
            forwardA(speed);
            delay(900); // Reset steering
            //stopA();                // debug
            //delay(250);            // debug
            //forwardA(speed);        // debug
            turns+=1;
          } else {
            controlSteeringWithPIDR(distance3); // Adjust steering based on distance using PID
            forwardA(speed); // Move the robot forward
          }
        }
      }
    }  
  }
  while(turns==12){
    forwardA(speed);
    delay(1000);
    stopA();
    turns+=1;
  }
}