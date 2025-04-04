/*
* Project: Grabby Warehouse robot
* Author: Julius Ortstadt
* Date: 07.01.2025
* Description: Control the robots' motors with the arrow keys through ROS
*/


// ROS includes
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// ROS node handler
ros::NodeHandle nh;

// Motor 1 is the motor on the right
// Motor 2 is the motor on the left

// Motor 1 control pins
const int IN1=39; // Pin for control signal to change motor behaviour
const int IN2=40; // Pin for control signal to change motor behaviour
const int ENA=45; // Pin for PWM

// Motor 2 control pins
// IN3 & IN4 need to be inverted compared to the actual cabling because the motors are mounted on opposit sides
const int IN3=42; // Pin for control signal to change motor behaviour
const int IN4=41; // Pin for control signal to change motor behaviour
const int ENB=44; // Pin for PWM

// Speed
int speed = 80;

//----------------------// Directions //----------------------//
enum DIRECTIONS {FORWARD, BACKWARD, LEFT, RIGHT, FL, FR, BL, BR, STOP}; // FL (Forward-Left) / FR (Forward-Right) / BL (Backward-Left) / BR (Backward-Right)
DIRECTIONS direction = STOP;

void chooseDir(const geometry_msgs::Twist& cmdDir){
  // Select the right direction based on the message published
  auto lateral = cmdDir.linear.x; // Forward - Backward (+ -)
  auto rotation = cmdDir.angular.z; // Left - Right (- +)

  if (lateral > 0 && rotation == 0){ // Forward
    direction = FORWARD;
  }
  else if (lateral < 0 && rotation == 0){ // Backward
    direction = BACKWARD;
  }
  else if (rotation > 0 && lateral == 0){ // Right 
    direction = RIGHT;
  }
  else if (rotation < 0 && lateral == 0){ // Left
    direction = LEFT;
  }
  else if (lateral > 0 && rotation < 0){ // Forward-Left
    direction = FL;
  }
  else if (lateral > 0 && rotation > 0){ // Forward-Right 
    direction = FR;
  }
  else if (lateral < 0 && rotation < 0){ // Backward-Left
    direction = BL;
  }
  else if (lateral < 0 && rotation > 0){ // Backward-Right
    direction = BR;
  }
  else {
    direction = STOP;
  }
}

//----------------------// ROS Subscribers //----------------------//
ros::Subscriber<geometry_msgs::Twist> subDir("teleop_timeout", &chooseDir);

void setup() {
  // put your setup code here, to run once:
  // Setup for motor 1 pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Setup for motor 2 pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stop();

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subDir);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();

  // Move the robot according to the desired direction
  switch (direction){
    case STOP:
      stop();
      break;
    case FORWARD:
      forward(speed);
      break;
    case BACKWARD:
      backward(speed);
      break;
    case LEFT:
      turn_left(speed);
      break;
    case RIGHT:
      turn_right(speed);
      break;
    case FL:
      power_left(speed);
      break;
    case FR:
      power_right(speed);
      break;
    case BL:
      power_left(-speed);
      break;
    case BR:
      power_right(-speed);
      break;
  }
}


void stop(){ // Stop
  // Stop motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Stop motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void backward(int speed){ // Moves backward
  // Motor 1 forward 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Motor 2 forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void forward(int speed){ // Moves forward
  // Motor 1 backward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  // Motor 2 backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void turn_left(int speed){ // Rotates left 
  // Motor 1 backward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  // Motor 2 forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void turn_right(int speed){ // Rotates right
  // Motor 1 forward 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Motor 2 backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void power_right(int speed){ // Powered right turn
  // Motor 1 forward 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Motor 2 backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed/2);
}

void power_left(int speed){ // Powered left turn
  // Motor 1 backward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed/2);

  // Motor 2 forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}