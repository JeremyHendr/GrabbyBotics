/* Project: Grabby Warehouse
 * Code: Counts the number of wheel revolutions / Used to test to count one wheel revolution
 * Julius Ortstadt
 * 10.01.2025
*/

#include <math.h>

//-----------// Motor Pins //-----------//
// Motor 1 control pins
const int IN1=39; // Pin for control signal to change motor behaviour
const int IN2=40; // Pin for control signal to change motor behaviour
const int ENA=45; // Pin for PWM

//-----------// Encoder Pins //-----------//
// WARNING: Pins need to be pins that can be interrupted (see list for Arduino)
#define enc_M1_A 2 // Yellow for encoder A
#define enc_M1_B 3 // White for encoder B

//-----------// Encoder and Motor Properties //-----------//
// Motor gear ratio
float gear_ratio = 98.78;

// Gearbox output 4741.44 counts per revolution total on all channels
float output_2channel = 4741.44;
float output_1channel_rising = output_2channel / 4;

// Motor shaft ticks per revolution for channel A rising:
float m_shaft_ticks = 12;
float g_shaft_ticks = m_shaft_ticks * gear_ratio;

//-----------// Wheel properties //-----------//
const float pi = M_PI;
const float wheel_diameter = 70 * pow(10,-3); // m
float wheel_circumference = pi * wheel_diameter; // m

long ticks_per_wheel_revolution = g_shaft_ticks; // Gearbox shaft ticks per revolution matches the gearbox output

//-----------// Tick counters //-----------//
volatile long M1_tick_count = 0;

//-----------// Motor Speed //-----------//
int speed = 255;

bool test_mode = true;
int revolution_cnt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Set the pinMode of the encoders
  pinMode(enc_M1_A, INPUT_PULLUP);

  // When the pin goes HIGH we interrupt (therefore the volatile long)
  attachInterrupt(digitalPinToInterrupt(enc_M1_A), M1_tick, RISING);

  // Setup for motor 1 pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Stop the motor
  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  forward(speed);

  // Test mode: Stop the motor after one wheel revolution 
  if (M1_tick_count >= ticks_per_wheel_revolution && test_mode){
    stop();
    Serial.println("Wheel completed one revolution.");
    Serial.print("Total ticks: ");
    Serial.println(M1_tick_count);

    // Ensure the loop doesn't restart the motor
    while (true);
  }
  // Normal mode: Count the number of revolutions of wheel 1
  else if (M1_tick_count >= ticks_per_wheel_revolution && !test_mode){
    revolution_cnt++;
    Serial.println("Wheel completed one revolution.");
    Serial.print("Total number of revolutions: ");
    Serial.println(revolution_cnt);
  }

}


void M1_tick(){
  M1_tick_count++;
}

void stop(){ // Stop
  // Stop motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void forward(int speed){ // Moves forward
  // Motor 1 forward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}
