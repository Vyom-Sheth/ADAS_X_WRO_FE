#include "SparkFun_VL53L1X.h"
#include<Wire.h>
#include<ESP32Servo.h>

#define I2C_SCL 33
#define I2C_SDA 32
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
Servo servo;

const uint16_t PWMA = 25;
const uint16_t AIN2 = 17;
const uint16_t AIN1 = 21;

const uint16_t ANALOG_WRITE_BITS = 8;

int freq = 5000;
int channel_A = 0;
int resolution = ANALOG_WRITE_BITS;

void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  ledcAttachChannel(PWMA, freq, resolution, channel_A);
}

void forwardA(uint16_t pwm){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void stopA(uint16_t pwm){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, pwm);
}

void setup(void) 
{
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.begin(115200);

  pinMode(4, OUTPUT);
  servo.attach(4);
  servo.write(100);
  
  if (distanceSensor.begin() != 0)
  {
    Serial.println("Sensor failed");
    while(1)
      ;
  }
  Serial.println("Sensor online");
  initMotors();
}

void loop(void) 
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

  if (distance < 300)
  {
    forwardA(1000);
    servo.write(120);
    Serial.print("Right");
    delay(100);
  }

  else if (distance < 300)
  {
    forwardA(1000);
    servo.write(80);
    Serial.print("Left");
    delay(100);
  }
}
