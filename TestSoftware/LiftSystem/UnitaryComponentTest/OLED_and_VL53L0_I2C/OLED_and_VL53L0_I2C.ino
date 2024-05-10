#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define XSHUT_pin1 6

// #define Sensor2_newAddress 0x3F
#define Sensor1_newAddress 0x31


VL53L0X Sensor1;
// VL53L0X Sensor2;

void setup() {
  Serial.begin(115200);

  //TOF sensor
  pinMode(XSHUT_pin1, OUTPUT);
  digitalWrite(XSHUT_pin1,HIGH);

  Sensor1.setAddress(Sensor1_newAddress);
  pinMode(XSHUT_pin1, LOW);
  delay(10);
  Sensor1.setTimeout(500);
  Sensor1.init(true);
  Sensor1.startContinuous();

  //display
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x64
  display.clearDisplay();
  display.invertDisplay(1);
  delay(2000);
  display.invertDisplay(0);
  display.display();
}

void loop() {
  int range1 = Sensor1.readRangeContinuousMillimeters();

  Serial.print("Sensor 1 Address: 0x");
  Serial.print(Sensor1.getAddress(), HEX);
  Serial.print(", Range: ");
  Serial.print(range1-50); //50 is a correction
  Serial.println(" mm");
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Sensor 1: ");
  display.print(range1);
  display.println(" mm");
  display.display();

  delay(500);
}