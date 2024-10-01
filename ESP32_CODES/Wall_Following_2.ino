#include <Wire.h>     //Library for VL53L1X
#include "SparkFun_VL53L1X.h"   //Library for VL53L1X
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
  servo.write(100);

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

  forward(100);

  if (distance < 300)
  {
    int mapdist = map(distance, 0, 300, 180, 100);
    
    servo.write(mapdist);
    delay(200);
  }

  else if (distance > 300)
  {
    servo.write(20);
    delay(200);
  }
}
