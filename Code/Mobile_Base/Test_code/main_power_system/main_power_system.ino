/*
 * Project: Grabby
 * Author: Julius Ortstadt
 * Date: 26.11.2025
 * Description: Code for the display of power information on the back of the robot. 
 *              For current measurement, information was taken from: https://www.engineersgarage.com/acs712-current-sensor-with-arduino/
 */

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_RESET -1     // Display shares same reset as arduino
#define SCREEN_WIDTH 128  // In pixels
#define SCREEN_HEIGHT 64  // In pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Init the display object

// Define analog input
#define VOLT_12 1 // Voltmeter 12V Line
#define VOLT_5 0  // Voltmeter 5V Line
#define AMP_12 3  // Ampmeter 12V Line
#define AMP_5 2   // Ampmeter 5V Line

// Floats for ADC voltage & Input voltage
float adc12_voltage = 0.0;
float adc5_voltage = 0.0;

float in12_voltage = 0.0;
float in5_voltage = 0.0;
 
// Resistor values (in Ohms)
float R1 = 31006.0;
float R2 = 6019.0; 

// Reference Voltage (measured on AREF Pin)
float ref_voltage = 5.05;
 
// Initial ADC values
int adc12_value = 0;
int adc5_value = 0;

// Setup of amp meter
const int nSamples = 150;
const float adcResolution = 1023.0;
const float sensitivity = 0.100;  // 20A

void setup(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(1000); // Leave time for display to init
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Power Info");
  display.display();
}
 
void loop(){
  /*--- Current measurement ---*/
  // Init variables
  unsigned int x = 0;
  float AcsValue_1 = 0.0, Samples_1 = 0.0, AvgAcs_1 = 0.0, AcsValueF_1 = 0.0;
  float AcsValue_2 = 0.0, Samples_2 = 0.0, AvgAcs_2 = 0.0, AcsValueF_2 = 0.0;

  // Take n samples
  for (int x = 0; x < nSamples; x++){
    // Read sensor values
    AcsValue_1 = analogRead(AMP_12);
    AcsValue_2 = analogRead(AMP_5);     
    
    // Add samples
    Samples_1 = Samples_1 + AcsValue_1; 
    Samples_2 = Samples_2 + AcsValue_2; 
    
    delay (3); // ms - ADC settling time
  }

  // Take average of samples
  AvgAcs_1 = Samples_1/150.0;
  AvgAcs_2 = Samples_2/150.0; 

  // Extract the values
  // 2.5V at zero current - Arduino operates at 5.0V
  AcsValueF_1 = (2.5 - (AvgAcs_1 * (5.0 / adcResolution))) / sensitivity;
  AcsValueF_2 = (2.5 - (AvgAcs_2 * (5.0 / adcResolution))) / sensitivity;

  /*--- Voltage measurement ---*/
  // Read the Analog Input
  adc12_value = analogRead(VOLT_12);
  adc5_value = analogRead(VOLT_5);

  // Determine voltage at ADC input
  adc12_voltage  = (adc12_value * ref_voltage) / adcResolution;
  adc5_voltage  = (adc5_value * ref_voltage) / adcResolution;

  // Calculate voltage at divider input
  in12_voltage = adc12_voltage * (R1 + R2) / R2;
  in5_voltage = adc5_voltage * (R1 + R2) / R2;

  /*--- Data display ---*/
  // Update the display with the new values  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print("Power Info");

  display.setCursor(0, 16);
  display.print("12V: ");
  display.print(in12_voltage, 2);
  display.print("V");

  display.setCursor(0, 26);
  display.print("I12: ");
  display.print(AcsValueF_1, 2);
  display.print("A");

  display.setCursor(0, 42);
  display.print("5V:  ");
  display.print(in5_voltage, 2);
  display.print("V");

  display.setCursor(0, 52);
  display.print("I5:  ");
  display.print(AcsValueF_2, 2);
  display.print("A");

  display.display();

  // Delay
  delay(200);
}