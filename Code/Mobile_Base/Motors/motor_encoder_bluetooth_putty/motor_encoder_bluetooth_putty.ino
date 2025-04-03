/* Project: Grabby Warehouse
 * Code: Displays the calculated values through a serial connection over bluetooth (COM8) in PuTTy
 * The speed value can we adjusted through the IDE serial monitor with a wired connection
 * Chose COM8 at 9600 baudrate
 * Julius Ortstadt
 * 12.09.2024
*/

#include <SoftwareSerial.h>
#include <math.h>
#define RX 24 // = TX pin on module
#define TX 23 // = RX pin on module via a voltage divider

SoftwareSerial BlueT(RX,TX);

// Motor gear ratio
float gear_ratio = 98.78;

// Gearbox output 4741.44 counts per revolution total on all channels
//float output_2channel = 4741.44;
//float output_1channel_rising = output_2channel / 4;

// Motor shaft ticks per revolution for channel A rising:
float m_shaft_ticks = 12;
float g_shaft_ticks = m_shaft_ticks * gear_ratio;


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

// Tick counter
volatile long M1_tick_count = 0;
volatile long M2_tick_count = 0;

// Speed in RPM 
float RPM1_M1 = 0;
float RPM2_M1 = 0;

float RPM1_M2 = 0;
float RPM2_M2 = 0;

// Distance 
float M1_distance = 0;
float M2_distance = 0;

// Motor speed
int speed = 0;

// Measurement interval & time keepers
int interval = 5000; // ms
long previousMillis = 0;
long currentMillis = 0;

// Wheel
const float wheel_diameter = 70 * pow(10,-3); // m
const float wheel_circumference = M_PI * wheel_diameter; // m

// Measurement counter
int measurement = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Enter AT commands: ");

  // HC-06 Bluetooth module
  BlueT.begin(9600);

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

  // Update the speed through the serial monitor with wired connection
  if (Serial.available() > 0){
    speed = Serial.parseInt();
    if (speed < 0 || speed > 255){
      Serial.println("Not a valid number");
    }
    else {
      Serial.print("Speed updated to: ");
      Serial.println(speed);
    }
  }

  // Drive the motors forward
  forward(speed);

  // Calculate and send the results via Bluetooth
  if (currentMillis - previousMillis > interval){
    previousMillis = currentMillis;
    stop();
    
    BlueT.println(" ");

    // All the data related to motor 1
    BlueT.println("-----> Motor M1 Data <----- Measurement: " + String(measurement));
    BlueT.println("M1 tick count: " + String(M1_tick_count));
    RPM1_M1 = (M1_tick_count * 60/(interval*pow(10,-3)))/m_shaft_ticks; // RPM of motor shaft
    RPM2_M1 = (M1_tick_count * 60/(interval*pow(10,-3)))/g_shaft_ticks; // RPM of gearbox shaft
    BlueT.println("RPM Motor shaft value at " + String(speed) + " PWM: " + String(RPM1_M1));
    BlueT.println("RPM Gearbox shaft value at " + String(speed) + " PWM: " + String(RPM2_M1));
    M1_distance += (RPM2_M1 / 60) * wheel_circumference * (interval * pow(10,-3));
    BlueT.println("Distance travelled in m: " + String(M1_distance));

    BlueT.println(" ");
    BlueT.println(" ");

    // All the data related to motor 2
    BlueT.println("-----> Motor M2 Data <----- Measurement: " + String(measurement));
    BlueT.println("M2 tick count: " + String(M2_tick_count));
    RPM1_M2 = (M2_tick_count * 60/(interval*pow(10,-3)))/m_shaft_ticks; // RPM of motor shaft
    RPM2_M2 = (M2_tick_count * 60/(interval*pow(10,-3)))/g_shaft_ticks; // RPM of gearbox shaft
    BlueT.println("RPM Motor shaft value at " + String(speed) + " PWM: " + String(RPM1_M2));
    BlueT.println("RPM Gearbox shaft value at " + String(speed) + " PWM: " + String(RPM2_M2));
    M2_distance += (RPM2_M2 / 60) * wheel_circumference * (interval * pow(10,-3));
    BlueT.println("Distance travelled in m: " + String(M2_distance));

    BlueT.println(" ");

    BlueT.println("-------------------------------------------");

    measurement++;

    // Reset of the variables
    // Tick counter
    M1_tick_count = 0;
    M2_tick_count = 0;

    // Speed in RPM 
    RPM1_M1 = 0;
    RPM2_M1 = 0;

    RPM1_M2 = 0;
    RPM2_M2 = 0;
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

