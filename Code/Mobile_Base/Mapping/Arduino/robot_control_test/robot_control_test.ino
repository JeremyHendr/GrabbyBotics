/*
* Project: Grabby Warehouse robot
* Author: Julius Ortstadt
* Date: 03.12.2025
* Description: Test the received information by publishing a corresponding message
*/

// ROS includes
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// ROS node handler
ros::NodeHandle nh;

// Motor 1 control pins
const int IN1=4;      // Pin for control signal to change motor behaviour
const int IN2=7;      // Pin for control signal to change motor behaviour
const int ENA=5;      // Pin for PWM

// Motor 2 control pins
// IN3 & IN4 need to be inverted compared to the actual cabling because the motors are mounted on opposit sides
const int IN3=10;     // Pin for control signal to change motor behaviour
const int IN4=9;      // Pin for control signal to change motor behaviour
const int ENB=6;      // Pin for PWM

// Speed 
int speed = 100;

//----------------------// Directions //----------------------//
enum DIRECTIONS {FORWARD, BACKWARD, LEFT, RIGHT, FL, FR, BL, BR, STOP}; // FL (Forward-Left) / FR (Forward-Right) / BL (Backward-Left) / BR (Backward-Right)
DIRECTIONS direction = STOP;

void chooseDir(const geometry_msgs::Twist& cmdDir){
  // Select the right direction based on the message published
  auto lateral = cmdDir.linear.x; // Forward - Backward (+ -)
  auto rotation = cmdDir.angular.z; // Left - Right (- +)

  if (lateral == 1.0 && rotation == 0) direction = FORWARD;
  else if (lateral == -1.0 && rotation == 0) direction = BACKWARD;
  else if (lateral == 0.0 && rotation == -1.0) direction = RIGHT;
  else if (lateral == 0.0 && rotation == 1.0) direction = LEFT;
  else if (lateral == 1.0 && rotation == -1.0) direction = FR;
  else if (lateral == 1.0 && rotation == 1.0) direction = FL;
  else if (lateral == -1.0 && rotation == -1.0) direction = BR;
  else if (lateral == -1.0 && rotation == 1.0) direction = FL;
  else direction = STOP; 
}

//----------------------// ROS Subscribers //----------------------//
ros::Subscriber<geometry_msgs::Twist> subDir("cmd_serial", &chooseDir);

std_msgs::String directionStr;
ros::Publisher dirPub("Direction", &directionStr);

void setup() {
  // Setup for motor 1 pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Setup for motor 2 pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subDir);
  nh.advertise(dirPub);
}

void loop() {
  nh.spinOnce();

  // Move the robot according to the desired direction
  switch (direction){
    case STOP:
      directionStr.data = "STOP";
      break;
    case FORWARD:
      directionStr.data = "FORWARD";
      break;
    case BACKWARD:
      directionStr.data = "BACKWARD";
      break;
    case LEFT:
      directionStr.data = "LEFT";
      break;
    case RIGHT:
      directionStr.data = "RIGHT";
      break;
    case FL:
      directionStr.data = "FORWARD_LEFT";
      break;
    case FR:
      directionStr.data = "FORWARD_RIGHT";
      break;
    case BL:
      directionStr.data = "BACKWARD_LEFT";
      break;
    case BR:
      directionStr.data = "BACKWARD_RIGHT";
      break;
  }

  dirPub.publish(&directionStr);
}
