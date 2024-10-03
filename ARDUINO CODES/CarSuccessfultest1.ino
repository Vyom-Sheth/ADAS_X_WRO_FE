#define ENB 5;

int motorspeed = 150;

void setup() 
{
  pinMode(2, OUTPUT);
  //PIN 4 ON DRIVER
  pinMode(3, OUTPUT);
  //PIN 3 ON DRIVER
  
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  analogWrite(5, motorspeed);

  delay(2000);

  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);

  delay(2000);

  digitalWrite(3, LOW);
  digitalWrite(2, HIGH);

  delay(2000);

  digitalWrite(3, LOW);
  digitalWrite(2, LOW);
}

void loop() 
{}
