#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define I2C_SCL 33
#define I2C_SDA 32
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;

void setup(void) 
{
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.begin(115200);

  if (distanceSensor.begin() != 0)
  {
    Serial.println("Sensor Failed To Begin.");
    while (1)
     ;
  }
  Serial.println("Sensor online");
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

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.println();

  if (distance > 20)
  {
    //move the robot right
    delay(500);
  }

  else if (distance < 20)
  {
    //move the robot left
    delay(500);
  }
}
