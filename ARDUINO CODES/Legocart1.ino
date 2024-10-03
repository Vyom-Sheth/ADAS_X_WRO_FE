#include <Wire.h>
#include <Adafruit_VL53L1X.h>

Adafruit_VL53L1X sensor;

void setup() {
  Wire.begin();
  sensor.Init();
  sensor.setMeasurementTimingBudgetMicroseconds(100000); // Set timeout to 100ms
  Serial.begin(115200); // Replace with your desired baud rate
}

void loop() {
  sensor.ranging();
  uint16_t distance = sensor.getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  delay(100); // Adjust delay as needed
}