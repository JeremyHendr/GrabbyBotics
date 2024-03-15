#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define XSHUT_pin2 7
#define XSHUT_pin1 6

// #define Sensor2_newAddress 0x3F
#define Sensor1_newAddress 0x31


VL53L0X Sensor1;
// VL53L0X Sensor2;

void setup() {
  pinMode(XSHUT_pin1, OUTPUT);
  // pinMode(XSHUT_pin2, OUTPUT);

  digitalWrite(XSHUT_pin1,HIGH);
  // digitalWrite(XSHUT_pin2, HIGH);
  
  Serial.begin(115200);
  
  Wire.begin();
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x64
  
  Sensor1.setAddress(Sensor1_newAddress);
  pinMode(XSHUT_pin1, LOW);
  delay(10);

  // Sensor2.setAddress(Sensor2_newAddress);
  // pinMode(XSHUT_pin2, LOW);
  // delay(10);

  Sensor1.setTimeout(500);
  Sensor1.init(true);
  Sensor1.startContinuous();

  // Sensor2.setTimeout(500);
  // Sensor2.init(true);
  // Sensor2.startContinuous();
}

void loop() {
  int range1 = Sensor1.readRangeContinuousMillimeters();
  // int range2 = Sensor2.readRangeContinuousMillimeters();
  
  // Serial.print("Sensor 2 Address: 0x");
  // Serial.print(Sensor2.getAddress(), HEX);
  // Serial.print(", Range: ");
  // Serial.print(range2);
  // Serial.println(" mm");

  Serial.print("Sensor 1 Address: 0x");
  Serial.print(Sensor1.getAddress(), HEX);
  Serial.print(", Range: ");
  Serial.print(range1-50); //50 is a correction
  Serial.println(" mm");
  
  // Clear the display
  display.clearDisplay();

  // Set text size and color
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Print sensor readings on the OLED display
  display.setCursor(0, 0);
  display.print("Sensor 1: ");
  display.print(range1);
  display.println(" mm");

  // display.setCursor(0, 10);
  // display.print("Sensor 2: ");
  // display.print(range2);
  // display.println(" mm");

  display.display();
  delay(500);
}