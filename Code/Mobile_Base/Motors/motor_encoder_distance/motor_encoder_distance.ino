/* Project: Grabby Warehouse
 * Code: Travels a certain distance and displays it by using the encoder ticks
 * Julius Ortstadt
 * 09.09.2024
*/
#include <math.h>


// Motor gear ratio
float gear_ratio = 98.78;

// Gearbox output 4741.44 counts per revolution total on all channels
float output_2channel = 4741.44;
float output_1channel_rising = output_2channel / 4;

// Motor shaft ticks per revolution for channel A rising:
float m_shaft_ticks = 12;
float g_shaft_ticks = m_shaft_ticks * gear_ratio;

// g_shaft_ticks = output_1channel_rising


// Define the pins for the motor encoder
// WARNING: Pins need to be pins that can be interrupted (see list for Arduino)
#define enc_M1_A 2 // Yellow for encoder A
#define enc_M1_B 3 // White for encoder B

#define enc_M2_A 18 // Yellow for encoder A
#define enc_M2_B 19 // White for encoder B

// Motor 1 control pins
const int IN1=39; // Pin for control signal to change motor behaviour
const int IN2=40; // Pin for control signal to change motor behaviour
const int ENA=45; // Pin for PWM

// Motor 2 control pins
// IN3 & IN4 need to be inverted compared to the actual cabling because the motors are mounted on opposit sides
const int IN3=42; // Pin for control signal to change motor behaviour
const int IN4=41; // Pin for control signal to change motor behaviour
const int ENB=44; // Pin for PWM



//rpmoutgbox = (1/gear_ratio*m_shaft_ticks)*M1_ticks*60 (because 1s measureinterval)

// Tick counter
volatile long M1_tick_count = 0;
volatile long M2_tick_count = 0;

// Distance 
float M1_distance = 0;
float M2_distance = 0;

// Speed in RPM 
float RPM1_M1 = 0;
float RPM2_M1 = 0;

float RPM1_M2 = 0;
float RPM2_M2 = 0;

// Start time 
unsigned long start_time = 0;

// Motor speed
int speed = 255;

// Measurement interval & time keepers
int interval = 5000; // ms
long previousMillis = 0;
long currentMillis = 0;

// Wheel
const float pi = 3.14159;
const float wheel_diameter = 70 * pow(10,-3); // m
float wheel_circumference = pi * wheel_diameter; // m

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Set the pinMode of the encoders
  pinMode(enc_M1_A, INPUT_PULLUP);
  pinMode(enc_M2_A, INPUT_PULLUP);

  // When the pin goes HIGH we interrupt (therefore the volatile long)
  attachInterrupt(digitalPinToInterrupt(enc_M1_A), M1_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_M2_A), M2_tick, RISING);

  // Setup for motor 1 pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Setup for motor 2 pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Stop the motor
  stop();

  // Start millis()
  previousMillis = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  forward(speed);

  if (currentMillis - previousMillis > interval){
    stop();
    Serial.println("-----> Motor M1 Data <-----");
    Serial.print("M1 tick count: ");
    Serial.println(M1_tick_count);
    RPM1_M1 = (M1_tick_count * 60/(interval*pow(10,-3)))/m_shaft_ticks; // RPM of motor shaft
    RPM2_M1 = (M1_tick_count * 60/(interval*pow(10,-3)))/g_shaft_ticks; // RPM of gearbox shaft
    Serial.print("RPM Motor shaft value at ");
    Serial.print(speed);
    Serial.print(" PWM: ");
    Serial.println(RPM1_M1);
    Serial.print("RPM Gearbox shaft value at ");
    Serial.print(speed);
    Serial.print(" PWM: ");
    Serial.println(RPM2_M1);

    M1_distance = (RPM2_M1 / 60) * wheel_circumference * (interval * pow(10,-3));
    Serial.print("Distance travelled in m: ");
    Serial.println(M1_distance);


    Serial.println(" ");

    Serial.println("-----> Motor M2 Data <-----");
    Serial.print("M2 tick count: ");
    Serial.println(M2_tick_count);
    RPM1_M2 = (M2_tick_count * 60/(interval*pow(10,-3)))/m_shaft_ticks; // RPM of motor shaft
    RPM2_M2 = (M2_tick_count * 60/(interval*pow(10,-3)))/g_shaft_ticks; // RPM of gearbox shaft
    Serial.print("RPM Motor shaft value at ");
    Serial.print(speed);
    Serial.print(" PWM: ");
    Serial.println(RPM1_M2);
    Serial.print("RPM Gearbox shaft value at ");
    Serial.print(speed);
    Serial.print(" PWM: ");
    Serial.println(RPM2_M2);

    M2_distance = (RPM2_M2 / 60) * wheel_circumference * (interval * pow(10,-3));
    Serial.print("Distance travelled in m: ");
    Serial.println(M2_distance);


    Serial.println(" ");
    Serial.println(" ");

  }

}

void M1_tick(){
  M1_tick_count++;
}

void M2_tick(){
  M2_tick_count++;
}

void stop(){ // Stop
  // Stop motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Stop motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward(int speed){ // Moves forward
  // Motor 1 forward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  // Motor 2 forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

