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


void setup() 
{
  initMotors();
}

void loop() {
  forwardA(500);
}