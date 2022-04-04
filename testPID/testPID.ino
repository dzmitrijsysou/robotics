#define ENCA 20
#define ENCB 21
#define PWM 3
#define IN1 51
#define IN2 50

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup(){
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}
