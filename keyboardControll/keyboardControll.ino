#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <PID_v1.h>
#include <Wire.h>


//---------------------------
//------- LOOP COUNTER ----------
//---------------------------
// Keep track of the loop counts to manage loss of communication
#define LOOPTIME 100 // Looptime in milliseconds
const byte noCommLoopMax = 5; // Number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0; // Main loop without communication counter
unsigned long lastMilli = 0;



//---------------------------
//------- ENOCDERS ----------
//---------------------------
// Left side encoder pin setup
const int PIN_ENCOD_A_MOTOR_LEFT = 20; // A channel for left motor encoder
const int PIN_ENCOD_B_MOTOR_LEFT = 21; // B cahnnel for left motor encoder

// Right side encoder pin setup
const int PIN_ENCOD_A_MOTOR_RIGHT = 18; // A channel for right motor encoder
const int PIN_ENCOD_B_MOTOR_RIGHT = 19; // B channel for right motor encoder

volatile float pos_left = 0; // Left motor encoder position
volatile float pos_right = 0; // Right motor encoder position

const double encoder_cpr = 2786; // [PPR]

//---------------------------
//------- MOTORS ------------
//---------------------------
// Left motor pin setup
int PIN_MOTOR_LEFT_1 = 51;
int PIN_MOTOR_LEFT_2 = 50;
int PIN_MOTOR_LEFT_PWM = 3;
int PWM_leftMotor = 0; // PWM command value for left motor

// Right motor pin setup
int PIN_MOTOR_RIGHT_1 = 52;
int PIN_MOTOR_RIGHT_2 = 53;
int PIN_MOTOR_RIGHT_PWM = 2;
int PWM_rightMotor = 0; // PWM command value for right motor







//-------------------------------------------
//------- ROBOT SPECIFIC CONSTANTS ----------
//-------------------------------------------
const double radius = 0.0508; // [m] Wheel radius
const double wheelbase = 0.0384; // [m] Wheelbase 

const double speed_to_pwm_ratio = 1; // Ratio to convert speed [m/s] to PWM value
const double min_speed_cmd = 1; // [N/A] min_speed_cmd/speed_to_pwm_ratio



//---------------------------
//---------- PID ------------
//---------------------------
// Overall desired robot speed
double speed_req = 0; // [m/s] Desired linear speed of the robot
double angular_speed_req = 0; // [rad/s] Desired angular speed for the robot


// Left motor speed parameters
double speed_req_left = 0; // [m/s] Desired speed for the left wheel
double speed_act_left = 0; // [m/s] Actual speed of the left wheel
double speed_cmd_left = 0; // [m/s] Command speed for the left wheel


// Right motor speed parameters
double speed_req_right  = 0; // [m/s] Desired speed for the right wheel
double speed_act_right  = 0; // [m/s] Actual speed of the right wheel
double speed_cmd_right  = 0; // [m/s] Command speed for the right wheel



// Setting max speed 
const double max_speed = 0.2; // [m/s] Max speed


// PID parameters
const double PID_left_param[] = {0, 0, 0.1}; // Respectively Kp, Ki, Kd for right motor
const double PID_right_param[] = {0, 0, 0.1}; // Respectively Kp, Ki, Kd for right motor

// PID setup
// PID(Variable that is controlled, Variable to be adjusted, Value for the input to maintain, Kp, Ki, Kd);
PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor





//------------------------------------
//-----TWIST MESSAGE SPEED DATA ------
//------------------------------------
ros::NodeHandle nh;

// handle_cmd function will be called when the subscriber received data from Twist message
void handle_cmd(const geometry_msgs::Twist& cmd_vel){
  noCommLoops = 0; // Reset the counter for the number of main loops without communication

  speed_req = cmd_vel.linear.x;

  angular_speed_req = cmd_vel.angular.z;

  speed_req_left = speed_req - angular_speed_req * (wheelbase/2); // [m/s]  Calculate the required speed for the left motor to comply with the commanded linear and angualr speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase/2); // [m/s] Calculate the required speed for the rigth motor to comply with the commanded linear and angualr speeds
}



//---------------------------------------------------
//-----SETTING UP ROS SUBSCRIBER AND PUBLISHER ------
//--------------------------------------------------
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd); // Create a subsriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg; // Create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg); // Create a publisher to ROS topic "speed" using the "speed_msg" type



void setup() {
Serial.begin(57600);

//-------ROS----------
  nh.initNode(); // Initialize ROS node
  nh.getHardware()->setBaud(57600); // Set baud for ROS serial communication
  nh.subscribe(cmd_vel); // Subscribe to ROS topic for velocity commands
  nh.advertise(speed_pub); // Prepare to publish speed in ROS topic


//--------ENCODERS----------
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);
  
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);


//-------- MOTORS ------------
  pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_2, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);

  pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);

  

//---------PID------------
PID_leftMotor.SetSampleTime(95); // [ms] Sets the frequency with which the PID calculations is performed. Default value is 100
PID_rightMotor.SetSampleTime(95); 
PID_leftMotor.SetOutputLimits(-max_speed, max_speed); // Clamps the output to a specific range
PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
PID_leftMotor.SetMode(AUTOMATIC);
PID_rightMotor.SetMode(AUTOMATIC);





  
}

void loop() {
  
  nh.spinOnce();
  if((millis() - lastMilli) >= LOOPTIME){

    lastMilli = millis();
    
    //--------LEFT WHEEL-------
    if (abs(pos_left) < 2){ // Avoids taking into account small disturbances
      speed_act_left = 0;
    }else{
      // Speed = encoder fractional position * wheel circumeference * 1/time elapsed
      speed_act_left = (pos_left/encoder_cpr)*(2)*(PI)*(1000/LOOPTIME)*(radius); // [m/s] Speed of the left wheel
    }


    pos_left = 0; // Reseting the position for left motor after the speed has been calcaulted

    //speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed); // Constrain the user input speed

    PID_leftMotor.Compute();

    PWM_leftMotor = ((speed_req_left + sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio);
    
  //  PWM_leftMotor = constrain((speed_cmd_left/speed_to_pwm_ratio), 0, 255);

    if (noCommLoops >= noCommLoopMax){ // Stop if too much time has elapsed without communication
      analogWrite(PIN_MOTOR_LEFT_PWM, 0);
      digitalWrite(PIN_MOTOR_LEFT_1, LOW);
      digitalWrite(PIN_MOTOR_LEFT_2, LOW);
      
    }else if (speed_req_left == 0){  // Stopping
      analogWrite(PIN_MOTOR_LEFT_PWM, 0);
      digitalWrite(PIN_MOTOR_LEFT_1, LOW);
      digitalWrite(PIN_MOTOR_LEFT_2, LOW);
      
    }else if (speed_req_left > 0){  // Going forward
      analogWrite(PIN_MOTOR_LEFT_PWM, PWM_leftMotor);
      digitalWrite(PIN_MOTOR_LEFT_1, HIGH);
      digitalWrite(PIN_MOTOR_LEFT_2, LOW);
      
    }else if (speed_req_left < 0){  // Going backwards
      analogWrite(PIN_MOTOR_LEFT_PWM, PWM_leftMotor);
      digitalWrite(PIN_MOTOR_LEFT_1, LOW);
      digitalWrite(PIN_MOTOR_LEFT_2, HIGH);
    }else{
      analogWrite(PIN_MOTOR_LEFT_PWM, 0);
      digitalWrite(PIN_MOTOR_LEFT_1, LOW);
      digitalWrite(PIN_MOTOR_LEFT_2, LOW);
    }


    //---------RIGHT WHEEL-------------
    if (abs(pos_right) < 2){
      speed_act_right = 0; 
    }else{
      // Speed = encoder fractional position * wheel circumeference * 1/time elapsed
      speed_act_right = (pos_right/encoder_cpr)*(2)*(PI)*(1000/LOOPTIME)*(radius); // [m/s] Speed of the right wheel
    }
    
    pos_right = 0; // Resting the position for right motor after the speed has been calcualted

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed); // Constrain the user input speed

    PID_rightMotor.Compute();

    PWM_rightMotor = ((speed_req_right + sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio);
     //PWM_rightMotor = constrain(speed_cmd_right/speed_to_pwm_ratio, 0, 255);
  
   
    if (noCommLoops >= noCommLoopMax){ // Stop if too much time has elapsed without communication
      analogWrite(PIN_MOTOR_RIGHT_PWM, 0);
      digitalWrite(PIN_MOTOR_RIGHT_1, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_2, LOW);
    }else if (speed_req_right == 0){  // Stopping
      analogWrite(PIN_MOTOR_RIGHT_PWM, 0);
      digitalWrite(PIN_MOTOR_RIGHT_1, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_2, LOW);
      
    }else if (speed_req_right > 0){  // Going forward
      analogWrite(PIN_MOTOR_RIGHT_PWM, PWM_rightMotor);
      digitalWrite(PIN_MOTOR_RIGHT_1, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_2, HIGH);
      
    }else if (speed_req_right < 0) {  // Going backwards
      analogWrite(PIN_MOTOR_RIGHT_PWM, PWM_rightMotor);
      digitalWrite(PIN_MOTOR_RIGHT_1, HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_2, LOW);
    }else{
      analogWrite(PIN_MOTOR_RIGHT_PWM, 0);
      digitalWrite(PIN_MOTOR_RIGHT_1, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_2, LOW);
    }

    if((millis() - lastMilli) >= LOOPTIME){ // Write an error if execution time of the loop is longer than specified looptime
      Serial.println("TOO LONG");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

    publishSpeed(LOOPTIME);

    
  }

  
}



//-------------------------
//-----ROS PUBLISHER ------
//-------------------------
// Fucntion that will publish the speed of the robot
void publishSpeed(double time) { 
  
  speed_msg.header.stamp = nh.now(); // Timestamp for odometry data
  speed_msg.vector.x = speed_act_left; // Left wheel speed [m/s]
  speed_msg.vector.y = speed_act_right; // Rigth wheel speed [m/s]
  speed_msg.vector.z = time/1000; // Looptime [s]
  speed_pub.publish(&speed_msg);
  nh.spinOnce();


  
  
  char PWM_rightMotor_buffer[20];
  char PWM_leftMotor_buffer[20];
  //dtostrf(value to convert, min field width, precision, buffer to store output)
  dtostrf(PWM_rightMotor, 8, 10, PWM_rightMotor_buffer);
  dtostrf(PWM_leftMotor, 8, 10, PWM_leftMotor_buffer);

  
  char pos_left_buffer[8];
  char pos_right_buffer[8];
  dtostrf(pos_left, 2, 2, pos_left_buffer);
  dtostrf(pos_right, 2, 2, pos_right_buffer);


  char speed_req_left_buffer[8];
  char speed_act_left_buffer[8];
  char speed_cmd_left_buffer[8];
  dtostrf(speed_req_left, 6, 2, speed_req_left_buffer);
  dtostrf(speed_act_left, 6, 2, speed_act_left_buffer);
  dtostrf(speed_cmd_left, 6, 2, speed_cmd_left_buffer);



  char speed_req_right_buffer[8];
  char speed_act_right_buffer[8];
  char speed_cmd_right_buffer[8];
  dtostrf(speed_req_right, 6, 2, speed_req_right_buffer);
  dtostrf(speed_act_right, 6, 2, speed_act_right_buffer);
  dtostrf(speed_cmd_right, 6, 2, speed_cmd_right_buffer);



  char speed_req_buffer[8];
  char angular_speed_req_buffer[8];
  dtostrf(speed_req, 6, 2, speed_req_buffer);
  dtostrf(angular_speed_req, 6, 2, angular_speed_req_buffer);

  
  char log_msg[200];

 //sptrintf(buffer to store output, "text %s", values to print);

  // Print the L and R motor Speeds
  //sprintf(log_msg,"Left Motor Speed [REQ, ACT, CMD] = [%s, %s, %s] Right Motor Speed = [%s, %s, %s]", speed_req_left_buffer, speed_act_left_buffer, speed_cmd_left_buffer, speed_req_right_buffer, speed_act_right_buffer, speed_cmd_right_buffer);

  // Print PWM command signlas 
 // sprintf(log_msg,"PWM = [%s, %s]", PWM_leftMotor_buffer, PWM_rightMotor_buffer);

  // Print PWM command signals with motor speeds
  sprintf(log_msg,"PWM = [%s, %s] Motor Speeds [REQ, ACT, CMD] = [%s, %s, %s] [%s, %s, %s]", PWM_leftMotor_buffer, PWM_rightMotor_buffer, speed_req_left_buffer, speed_act_left_buffer, speed_cmd_left_buffer, speed_req_right_buffer, speed_act_right_buffer, speed_cmd_right_buffer);

  
  //Print the REQ Robot Speed and REQ Motor Speeds
  //sprintf(log_msg,"Robot Req Speed [LIN, ANG] = [%s, %s] Motor Req Speeds = [%s, %s]", speed_req_buffer, angular_speed_req_buffer, speed_req_left_buffer, speed_req_right_buffer);
  nh.loginfo(log_msg);
  
}

//---------------------------------------
//-----ENCODER COUNTERS FOR MOTORS ------
//--------------------------------------

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



template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
