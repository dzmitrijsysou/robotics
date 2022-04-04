// Test reading the total dispalcement of two encoders
//TEST


const int PIN_ENCOD_A_MOTOR_LEFT = 20; // A channel for left motor encoder
const int PIN_ENCOD_B_MOTOR_LEFT = 21; // B cahnnel for left motor encoder

const int PIN_ENCOD_A_MOTOR_RIGHT = 18; // A channel for right motor encoder
const int PIN_ENCOD_B_MOTOR_RIGHT = 19; // B channel for right motor encoder


volatile float pos_left = 0;
volatile float pos_right = 0;




void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);
  
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

void loop() {
  Serial.print(pos_right);
  Serial.print('\n');
  Serial.print(pos_left);
  Serial.print('\t');

}

// Right motor encoder counter
void encoderRightMotor(){
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)){
    pos_right--;
  }else{
    pos_right++;
  }
}


// Left motor encoder coutner
void encoderLeftMotor(){
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)){
    pos_left++;
  }else{
    pos_left--;
  }
}
