/*
 * Project: Grabby the warehouse robot
 * Adapted from: https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/
 * Description: ROS node that publishes the accumulated ticks for each wheel
 *              Publish:
 *                  /m1_ticks and /m2_ticks
 *              Subscribe:  
 *                  /cmd_vel 
 */
 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;
  
//-------------------- Encoder setup --------------------//
// Encoder output values 
#define enc_M1_A 2 // Yellow for encoder M1 A
#define enc_M2_A 18 // Yellow for encoder M2 A

// Encoder output to know which direction the motor spins in
#define enc_M1_B 3 // White for encoder M1 B
#define enc_M2_B 19 // White for encoder M2 B

// Booleans to know in which direction the motor spins
// True = forward / False = backward
boolean direction_M1 = true;
boolean direction_M2 = true;
  
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
  
// Keep track of the number of wheel ticks
std_msgs::Int16 m1_wheel_tick_count;
ros::Publisher m1Pub("m1_ticks", &m1_wheel_tick_count);

std_msgs::Int16 m2_wheel_tick_count;
ros::Publisher m2Pub("m2_ticks", &m2_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
  
//-------------------- Motor setup --------------------//
// Motor 1 control pins
const int IN1=39; // Pin for control signal to change motor behaviour
const int IN2=40; // Pin for control signal to change motor behaviour
const int ENA=45; // Pin for PWM

// Motor 2 control pins
// IN3 & IN4 need to be inverted compared to the actual cabling because the motors are mounted on opposit sides
const int IN3=42; // Pin for control signal to change motor behaviour
const int IN4=41; // Pin for control signal to change motor behaviour
const int ENB=44; // Pin for PWM
  
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Robot specs
const double WHEEL_RADIUS = 0.035; // in meters
const double WHEEL_BASE = 0.18; // in meters
const double WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
const double GEAR_RATIO = 98.78;
const double MOTOR_SHAFT_TICKS = 12;
const double GEAR_BOX_SHAFT_TICKS = GEAR_RATIO * MOTOR_SHAFT_TICKS;
const double TICKS_PER_REVOLUTION = GEAR_BOX_SHAFT_TICKS;
const double TICKS_PER_METER = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE; // 5393.45 ticks per meter
  
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const double K_P = 1539;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const double b = -207;

// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 80; // about 0.187 m/s
const int PWM_MAX = 100; // about 0.2 m/s

// Set linear velocity and PWM variable values for each wheel
double velM1Wheel = 0;
double velM2Wheel = 0;
double pwmM1Req = 0;
double pwmM2Req = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
  
//-------------------- Tick data --------------------// 
void m1_wheel_tick() { // Increment tick counts for m1
    int val = digitalRead(enc_M1_A);

    if (val == LOW) {
      direction_M1 = false; // Reverse
    }
    else {
      direction_M1 = true; // Forward
    }

    if (direction_M1) { 
        if (m1_wheel_tick_count.data == encoder_maximum) {
            m1_wheel_tick_count.data = encoder_minimum;
        }
        else {
            m1_wheel_tick_count.data++;  
        }    
    }
    else {
        if (m1_wheel_tick_count.data == encoder_minimum) {
            m1_wheel_tick_count.data = encoder_maximum;
        }
        else {
            m1_wheel_tick_count.data--;  
        }   
    }
}
  
void m2_wheel_tick() { // Increment tick counts for m1
    int val = digitalRead(enc_M2_A);

    if (val == LOW) {
      direction_M2 = true; // Reverse
    }
    else {
      direction_M2 = false; // Forward
    }

    if (direction_M2) {
        if (m2_wheel_tick_count.data == encoder_maximum) {
            m2_wheel_tick_count.data = encoder_minimum;
        }
        else {
            m2_wheel_tick_count.data++;  
        }  
    }
    else {
        if (m2_wheel_tick_count.data == encoder_minimum) {
            m2_wheel_tick_count.data = encoder_maximum;
        }
        else {
            m2_wheel_tick_count.data--;  
        }   
    }
}
  
//-------------------- Motor control --------------------// 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is published on the /m1_ticks topic. 
void calc_vel_m1_wheel(){
    static double prevTime = 0;

    // Variable gets created and initialized the first time a function is called.
    static int prevM1Count = 0;

    int numOfTicks = (65535 + m1_wheel_tick_count.data - prevM1Count) % 65535;

    // If we have had a big jump, it means the tick count has rolled over.
    if (numOfTicks > 10000) {
            numOfTicks = 0 - (65535 - numOfTicks);
    }

    // Calculate wheel velocity in meters per second
    velM1Wheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);

    // Keep track of the previous tick count
    prevM1Count = m1_wheel_tick_count.data;

    prevTime = (millis()/1000);
}
  
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /m2_ticks topic. 
void calc_vel_m2_wheel(){
   static double prevTime = 0;
    
   // Variable gets created and initialized the first time a function is called.
   static int prevM2Count = 0;
  
   // Manage rollover and rollunder when we get outside the 16-bit integer range 
   int numOfTicks = (65535 + m2_wheel_tick_count.data - prevM2Count) % 65535;
  
   if (numOfTicks > 10000) {
         numOfTicks = 0 - (65535 - numOfTicks);
   }
  
   // Calculate wheel velocity in meters per second
   velM2Wheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  
   prevM2Count = m2_wheel_tick_count.data;
    
   prevTime = (millis()/1000);
}
  
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis()/1000);
  
  // Calculate the PWM value given the desired velocity 
  pwmM1Req = K_P * cmdVel.linear.x + b;
  pwmM2Req = K_P * cmdVel.linear.x + b;

  // Check if we need to turn 
  if (cmdVel.angular.z != 0.0) {
    // Turn left
    if (cmdVel.angular.z > 0.0) {
      pwmM1Req = -PWM_TURN;
      pwmM2Req = PWM_TURN;
    }
    // Turn right    
    else {
      pwmM1Req = PWM_TURN;
      pwmM2Req = -PWM_TURN;
    }
  }
  // Go straight
  else {
    // Remove any differences in wheel velocities to make sure the robot goes straight
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velM1Wheel - velM2Wheel; 
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;

    // Correct PWM values of both wheels to make the vehicle go straight
    pwmM1Req -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmM2Req += (int)(avgDifference * DRIFT_MULTIPLIER);
  }

  // Handle low PWM values
  if (abs(pwmM1Req) < PWM_MIN) {
    pwmM1Req = 0;
  }
  if (abs(pwmM2Req) < PWM_MIN) {
    pwmM2Req = 0;  
  }  
}
  
void set_pwm_values() {
  // Variables for desired PWM values
  static int pwmM1Out = 0;
  static int pwmM2Out = 0;

  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmM1Req * velM1Wheel < 0 && pwmM1Out != 0) || (pwmM2Req * velM2Wheel < 0 && pwmM2Out != 0)) {
    pwmM1Req = 0;
    pwmM2Req = 0;
  }

  // Set the direction of the motors
  if (pwmM1Req > 0) { // Left wheel forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (pwmM1Req < 0) { // Left wheel reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (pwmM1Req == 0 && pwmM1Out == 0 ) { // Left wheel stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW); 
  }

  if (pwmM2Req > 0) { // Right wheel forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if(pwmM2Req < 0) { // Right wheel reverse
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (pwmM2Req == 0 && pwmM2Out == 0) { // Right wheel stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW); 
  }

  // Increase the required PWM if the robot is not moving
  if (pwmM1Req != 0 && velM1Wheel == 0) {
    pwmM1Req *= 1.5;
  }
  if (pwmM2Req != 0 && velM2Wheel == 0) {
    pwmM2Req *= 1.5;
  }

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmM1Req) > pwmM1Out) {
    pwmM1Out += PWM_INCREMENT;
  }
  else if (abs(pwmM1Req) < pwmM1Out) {
    pwmM1Out -= PWM_INCREMENT;
  }
  else{}
  
  if (abs(pwmM2Req) > pwmM2Out) {
    pwmM2Out += PWM_INCREMENT;
  }
  else if(abs(pwmM2Req) < pwmM2Out) {
    pwmM2Out -= PWM_INCREMENT;
  }
  else{}

  // Conditional operator to limit PWM output at the maximum 
  pwmM1Out = (pwmM1Out > PWM_MAX) ? PWM_MAX : pwmM1Out;
  pwmM2Out = (pwmM2Out > PWM_MAX) ? PWM_MAX : pwmM2Out;

  // PWM output cannot be less than 0
  pwmM1Out = (pwmM1Out < 0) ? 0 : pwmM1Out;
  pwmM2Out = (pwmM2Out < 0) ? 0 : pwmM2Out;

  // Set the PWM value on the pins
  analogWrite(ENA, pwmM1Out); 
  analogWrite(ENB, pwmM2Out); 
}
  
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

void setup() {
  //--------------// Encoder Setup //--------------//
  // Set pin states of the encoder
  pinMode(enc_M1_A, INPUT_PULLUP);
  pinMode(enc_M1_B, INPUT);
  pinMode(enc_M2_A, INPUT_PULLUP);
  pinMode(enc_M2_B, INPUT);

  // Pin goes HIGH = tick
  attachInterrupt(digitalPinToInterrupt(enc_M1_A), m1_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_M2_A), m2_wheel_tick, RISING);
  
  //--------------// Motor Setup //--------------//
  // Setup for motor 1 pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Setup for motor 2 pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Stop motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); 

  // Stop motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(m1Pub);
  nh.advertise(m2Pub);
  nh.subscribe(subCmdVel);
}
  
void loop() {
  nh.spinOnce();
  
  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {    
    previousMillis = currentMillis;

    // Publish tick counts to topics
    m1Pub.publish(&m1_wheel_tick_count);
    m2Pub.publish(&m2_wheel_tick_count);

    // Calculate the velocity of the right and left wheels
    calc_vel_m1_wheel();
    calc_vel_m2_wheel();
    
  }
  
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmM1Req = 0;
    pwmM2Req = 0;
  }

  set_pwm_values();
}