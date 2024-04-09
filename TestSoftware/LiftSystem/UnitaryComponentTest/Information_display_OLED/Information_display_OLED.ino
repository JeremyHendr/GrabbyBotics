#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>

#define screen_w 128
#define screen_h 32

#define screen_reset -1
Adafruit_SSD1306 screen(screen_w, screen_h, &Wire, screen_reset);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  screen.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x64
}

void loop() {
  // Clear the display
  screen.clearDisplay();

  //yellow info
  screen.setTextSize(1);
  screen.setTextColor(SSD1306_WHITE);
  screen.setCursor(0, 0);
  screen.println("H:xxxx  D:xxx  p:x");

  //blue info
  screen.println("Picking up package");
  screen.println();
  screen.println("Holding at XXXmm");
  screen.display();
  delay(500);
}

void updateScreen(){

}

//--------------Yellow info
//====================//  space on one line
//H:xxxx  D:xxx  p:x
//H: heigth platform in mm
//D: distance slider in mm
//p: boolean if package or not 


//--------------Blue changing messages--------------
//====================//
//Picking up package
//Package picked up!
//Dropping package
//Package dropped!
//Holding package
//No package 


//Going up to XXXmm
//Going down to XXXmm
//Holding at H=XXXmm