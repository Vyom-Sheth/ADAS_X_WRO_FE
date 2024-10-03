#include <Wire.h>     //Library for VL53L1X
#include <SparkFun_VL53L1X.h>   //Library for VL53L1X
#include <ESP32Servo.h> // Library for servo motor
Servo servo; // Naming the servo

const uint16_t PWMA = 25;   //Defining the PWM pin
const uint16_t AIN2 = 17;   //Defining the AIN1 pin
const uint16_t AIN1 = 21;   //Defining the AIN2 pin

#define I2C_SCL 33    //DEFINING THE SCL PIN FOR DISTANCE SENSOR
#define I2C_SDA 32    //DEFINING THE SDA PIN FOR DISTANCE SENSOR
#define SHUTDOWN_PIN 2  //DEFINING THE SHUTDOWN PIN
#define INTERRUPT_PIN 3 //DEFINING THE INTERRUPT PIN

const uint16_t ANALOG_WRITE_BITS = 8;

SFEVL53L1X distanceSensor;

int freq = 5000;
int channel_A = 2;
int resolution = ANALOG_WRITE_BITS;
int initdistance = 200;
int laps = 0;

// Control parameters
const int idealDistanceFromWall = 250;  // Ideal distance from the wall in mm
const int maxTurnAngle = 20;  // Maximum turning angle for the servo motor in degrees

// PID parameters
float kp = 1.0;  // Proportional gain
float ki = 0.0;  // Integral gain
float kd = 0.1;  // Derivative gain

float previousError = 0;
float integral = 0;
float dt = 0.05;  // Time step (50ms loop delay)

// Function to control the steering based on the distance from the wall using PID
void controlSteeringWithPID(int16_t distance) {
  if (distance == -1) return;  // No valid reading, keep going straight
 
  float error = distance - idealDistanceFromWall; // Calculate error
  float Pout = kp * error; // Proportional term

  integral += error * dt; // Integral term
  float Iout = ki * integral;

  float derivative = (error - previousError) / dt; // Derivative term
  float Dout = kd * derivative;
  
  float output = Pout + Iout + Dout; // PID output
  previousError = error; // Update previous error

  int turnAngle = map(output, -100, 100, -maxTurnAngle, maxTurnAngle); // Map output to steering angle
  turnAngle = constrain(turnAngle, -maxTurnAngle, maxTurnAngle); // Constrain angle
  int servoAngle = 100 - turnAngle; // Set servo angle
  
  servo.write(servoAngle); // Set steering
  
  // Debugging output
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" mm, Turn angle: ");
  Serial.println(turnAngle);
  Serial.println(servoAngle);
}

bool inRange(int val, int min, int max)
{
  return ((min <= val) && (val <= max));
}

void initmotors()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  ledcAttachChannel(PWMA, freq, resolution, channel_A);
}

void forward(uint16_t pwm)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void stop(uint16_t pwm)
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void setup() 
{
  pinMode(4, OUTPUT);           //Servo motor
  servo.attach(4);

  initmotors();
   
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

  servo.write(100);       // Set Steering to Centre

}

void loop() 
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  Serial.println();

  forward(150);                   // Set DC Motor Speed

  if (distance != -1) 
  {
    if (distance > 1000) {
      stop(0);
      servo.write(20);
      delay(100);
      forward(150);
      delay(2250);
      stop(0); // Stop motors
      servo.write(100);  // Reset steerin
    } 
    
    else {
      controlSteeringWithPID(distance); // Adjust steering based on distance using PID
      forward(150); // Move the robot forward
    }
  }
}