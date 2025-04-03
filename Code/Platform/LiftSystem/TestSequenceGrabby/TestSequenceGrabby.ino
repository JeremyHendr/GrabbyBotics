//Nema17Stepper
#define slider_dir 34
#define slider_cmd 35

//led
#define led_R 5
#define led_G 6
#define led_B 7

//LS
#define ls_slider_back 32
#define ls_slider_front 33
#define ls_lift_bottom 49
#define ls_lift_top 48 //29

//relay
#define pump 31 //pin is 50
#define valve 30

//DC motor
#define lift_dir 47
#define lift_cmd 46


//variable
bool lsf_last=1; //to know if the limit switch slider front was the last pressed
bool lslt_last=0;

//Bande LED
// #include "Adafruit_NeoPixel.h"
// #define led_strip_pin  4
// #define led_strip_pixel_quant 120
// Adafruit_NeoPixel strip(led_strip_pixel_quant, led_strip_pin, NEO_GRB + NEO_KHZ800);
#include "LedStrip.hpp"

//OLED screen
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#define screen_w 128 //
#define screen_h 32 //64 works but small, 32 better
Adafruit_SSD1306 screen(screen_w, screen_h, &Wire, -1);


//TOF sensor
#include <VL53L0X.h>
#include <movingAvg.h>
VL53L0X tof;
movingAvg tof_avg(15);

// //PID
#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp=20, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//----------------------------------SETUP---------------------------------------------
void setup() {
  Serial.begin(9600);
  //stepper
  pinMode(slider_dir, OUTPUT);  // DIR Pin 
  pinMode(slider_cmd, OUTPUT);  // STEP Pin
  digitalWrite(slider_cmd, 0);
  digitalWrite(slider_dir, 0);
  //led
  pinMode(led_R, OUTPUT);
  pinMode(led_G, OUTPUT);
  pinMode(led_B, OUTPUT);
  //LS
  pinMode(ls_slider_front, INPUT_PULLUP);
  pinMode(ls_slider_back, INPUT_PULLUP);
  pinMode(ls_lift_bottom, INPUT_PULLUP);
  pinMode(ls_lift_top, INPUT_PULLUP);
  //relays
  pinMode(pump, OUTPUT);
  pinMode(valve, OUTPUT);
  //DC motor
  pinMode(lift_dir, OUTPUT);
  pinMode(lift_cmd, OUTPUT);
  digitalWrite(lift_cmd, 0);
  digitalWrite(lift_dir, 0);

  //SETUP LED
  strip.begin();
  strip.setBrightness(50);  // luminosité de la LED (maximum 255)
  for (int x=0; x <= led_strip_pixel_quant; x++) {
    strip.setPixelColor(x, strip.Color(100, 200, 0));
    strip.show();
    delay(10);
    strip.clear();
  }

  //OLED screen
  screen.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  screen.clearDisplay();
  screen.invertDisplay(0);
  screen.setTextSize(1);
  screen.setTextColor(SSD1306_WHITE);
  screen.setCursor(40, 0);
  screen.println("Welcome to");
  screen.setCursor(35, 10);
  screen.println("GrabbyBotics");
  screen.display();


  //TOF sensor
  tof.setAddress(0x31);
  tof.setTimeout(500);
  tof.init(true);
  tof.startContinuous();
  tof_avg.begin();

  digitalWrite(led_R,1);
  delay(500);
  digitalWrite(led_R,0);
  digitalWrite(led_G,1);
  delay(500);
  digitalWrite(led_G,0);
  digitalWrite(led_B,1);
  delay(500);
  digitalWrite(led_B,0);
  Serial.println("Setup done");
  //setup pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
  Setpoint = 180;
}


//----------------------------------LOOP---------------------------------------------
void loop() {
  // TEST SEQUENCE:
  lsTest(); //green light is on when ls pressed
  // relayTest(); //activate relays in turns
  // stepperTest2(); //one turn cw one cc
  // sliderTest(); //slider moves back to front
  // sliderTest2(); //slider moves front to back and ativate pump and valve
  // DCmotorTest(); //3sec cc and 3sec cw
  // liftTest(); //Lift goes all the way up then all the way down
  // globalTest(); //platform goes up -> grip sequence -> platform down -> release sequence

  // lcdScreen();
  // TOF();
  // lcdAndTOF();
  //platformAndTOF();
  //  platformPID(); //85mm to 277mm
  // globalGripAndReleaseSequence();

}

//----------------------------------FUNCTIONS----------------------------------------
void platformPID(){
  int dst = tof.readRangeContinuousMillimeters();
  int dst_avg = tof_avg.reading(dst);
  Serial.print(dst_avg);
  Input = dst_avg;
  myPID.Compute();
  if(Output <  0){
    digitalWrite(lift_dir, HIGH);
  }
  else{
    digitalWrite(lift_dir, LOW);
  }
  Serial.print(" output:");
  Serial.println(Output);
  if (abs(Output)>20) {
    analogWrite(lift_cmd, abs(Output));
  }
  else {
    analogWrite(lift_cmd, 0);
  }

  screen.clearDisplay();
  screen.setTextSize(1);
  screen.setTextColor(SSD1306_WHITE);
  screen.setCursor(0, 0);
  screen.println("Height: "+String(dst_avg)+" mm");
  screen.display();
}

void lcdAndTOF(){
  int dst = tof.readRangeContinuousMillimeters();
  int dst_avg = tof_avg.reading(dst);

  screen.clearDisplay();
  screen.setTextSize(1);
  screen.setTextColor(SSD1306_WHITE);
  screen.setCursor(0, 0);
  screen.println("Height: "+String(dst_avg)+" mm");
  screen.display();
}

void TOF(){
  int dst = tof.readRangeContinuousMillimeters();
  int dst_avg = tof_avg.reading(dst);
  Serial.println("distance: "+String(dst_avg));
  delay(50);
}

void lcdScreen(){
  screen.clearDisplay();

  //yellow info
  screen.setTextSize(1);
  screen.setTextColor(SSD1306_WHITE);
  screen.setCursor(0, 0);
  screen.println("H:xxx  D:xxx  p:x");
  //blue info
  screen.println("Picking up package");
  screen.println();
  screen.println("Holding at XXXmm");
  screen.display();
  delay(500);
}

//-----------------------------Test Sequence Functions-------------------------------------
void globalTest(){
  static bool ls_bottom_last = 1;
  static int step_delay = 200;
  bool ls_bottom_pressed = !digitalRead(ls_lift_bottom);
  bool ls_top_pressed = !digitalRead(ls_lift_top);

  //Platform arrives at the bottom
  if (ls_bottom_pressed && !ls_bottom_last) {
      analogWrite(lift_cmd, 0);
      strip.fill(strip.Color(0,165,255));
      strip.show();
      
      //Pump starts to grab the package and slider push it
      bool slider_ls_front_pressed = !digitalRead(ls_slider_front);
      digitalWrite(slider_dir, 0);
      digitalWrite(pump, 1);
      while (!slider_ls_front_pressed) {
        digitalWrite(slider_cmd,1);
        delayMicroseconds(step_delay);
        digitalWrite(slider_cmd,0);
        delayMicroseconds(step_delay);
        slider_ls_front_pressed = !digitalRead(ls_slider_front);
      }
      digitalWrite(pump, 0);

      //Set the package free
      digitalWrite(valve, 1);
      delay(2000);
      digitalWrite(valve, 0);

      //slider goes back to origin
      bool slider_ls_back_pressed = !digitalRead(ls_slider_back);
      digitalWrite(slider_dir, 1);
      while (!slider_ls_back_pressed) {
        digitalWrite(slider_cmd,1);
        delayMicroseconds(step_delay);
        digitalWrite(slider_cmd,0);
        delayMicroseconds(step_delay);
        slider_ls_back_pressed = !digitalRead(ls_slider_back);
      }

      //Platform starts going up
      digitalWrite(lift_dir, 0);
      analogWrite(lift_cmd, 255);
      digitalWrite(led_G, 0);
      digitalWrite(led_R, 1);
      ls_bottom_last = 1;
  }

  //Platform arrives to the top
  else if (ls_top_pressed && ls_bottom_last) {
      analogWrite(lift_cmd, 0);
      strip.fill(strip.Color(165,0,255));
      strip.show();
     
      
      
      //Slider goes forward to go in contact with the package
      bool slider_ls_front_pressed = !digitalRead(ls_slider_front);
      digitalWrite(slider_dir, 0);
      while (!slider_ls_front_pressed) {
        digitalWrite(slider_cmd,1);
        delayMicroseconds(step_delay);
        digitalWrite(slider_cmd,0);
        delayMicroseconds(step_delay);
        slider_ls_front_pressed = !digitalRead(ls_slider_front);
      }

      //Sucks the package
      digitalWrite(pump, 1);
      delay(1000);
      
      //Slider pulls the package on the platform
       bool slider_ls_back_pressed = !digitalRead(ls_slider_back);
      digitalWrite(slider_dir, 1);
      while (!slider_ls_back_pressed) {
        digitalWrite(slider_cmd,1);
        delayMicroseconds(step_delay);
        digitalWrite(slider_cmd,0);
        delayMicroseconds(step_delay);
        slider_ls_back_pressed = !digitalRead(ls_slider_back);
      }
      digitalWrite(pump,0);

      //Platform starts going down
      digitalWrite(lift_dir, 1);
      analogWrite(lift_cmd, 255);
      digitalWrite(led_G, 1);
      digitalWrite(led_R, 0);
      ls_bottom_last = 0;
  }

  if (ls_bottom_last) {
    ledLiftUp();
  }
  else {
    ledLiftDown();
  }
}

void liftTest(){
  bool lsb = digitalRead(ls_lift_bottom);
  bool lst = digitalRead(ls_lift_top);
  if (!lsb && lslt_last) {
    digitalWrite(lift_dir, 1);
    digitalWrite(lift_cmd, 1);
    digitalWrite(led_G, 0);
    digitalWrite(led_R, 1);
    lslt_last = 0;
  }
  else if (!lst && !lslt_last) {
    digitalWrite(lift_dir, 0);
    digitalWrite(lift_cmd, 1);
    digitalWrite(led_G, 1);
    digitalWrite(led_R, 0);
    lslt_last = 1;
  }
  if (lslt_last) {
    ledLiftUp();
  }
  else {
    ledLiftDown();
  }
}

void DCmotorTest() {
  //3sec one cw 3sec cc
  digitalWrite(lift_dir, 1);
  digitalWrite(led_G, 0);
  digitalWrite(led_R, 1);
  digitalWrite(lift_cmd, 1);
  delay(3000);
  digitalWrite(lift_cmd,0);
  delay(500);
  digitalWrite(lift_dir, 0);
  digitalWrite(led_G, 1);
  digitalWrite(led_R, 0);
  digitalWrite(lift_cmd, 1);
  delay(3000);
  digitalWrite(lift_cmd,0);
  delay(500);
}

void sliderTest2() {
  //forward (1) when cw, backward (0) when ccw
  bool lsf = digitalRead(ls_slider_front);
  bool lsb = digitalRead(ls_slider_back);
  if (!lsf && !lsf_last) { //front ls closed
    digitalWrite(slider_dir, 1);
    digitalWrite(pump,1);
    digitalWrite(led_G,0);
    digitalWrite(led_R,0);
    delay(2000);
    digitalWrite(pump,0);
    digitalWrite(led_G,1);
    digitalWrite(led_R,0);
    lsf_last = 1;
  }
  else if (!lsb && lsf_last) { //back ls closed
    digitalWrite(slider_dir, 0);
    digitalWrite(valve,1);
    digitalWrite(led_G,0);
    digitalWrite(led_R,0);
    delay(2000);
    digitalWrite(valve,0);
    digitalWrite(led_G,0);
    digitalWrite(led_R,1);
    lsf_last = 0;
  }
  int dl = 300;
  digitalWrite(slider_cmd,1);
  delayMicroseconds(dl);
  digitalWrite(slider_cmd,0);
  delayMicroseconds(dl);
}

void sliderTest() {
  //forward (1) when cw, backward (0) when ccw
  bool lsf = digitalRead(ls_slider_front);
  bool lsb = digitalRead(ls_slider_back);
  if (!lsf) { //front ls closed
    digitalWrite(slider_dir, 1);
    digitalWrite(led_G,1);
    digitalWrite(led_R,0);
  }
  else if (!lsb) { //back ls closed
    digitalWrite(slider_dir, 0);
    digitalWrite(led_G,0);
    digitalWrite(led_R,1);
  }
  int dl = 200 ;
  digitalWrite(slider_cmd,1);
  delayMicroseconds(dl);
  digitalWrite(slider_cmd,0);
  delayMicroseconds(dl);
}

void stepperTest2() { //stepper does one turn clockwise and one cc
   int dl = 200;
  digitalWrite(slider_dir,1); 
  digitalWrite(led_G, 1);
  for (int x=0; x<2048; x++) {
    digitalWrite(slider_cmd,1);
    delayMicroseconds(dl);
    digitalWrite(slider_cmd,0);
    delayMicroseconds(dl);
  }
  digitalWrite(led_G, 0);
  delay(2000);
  digitalWrite(led_R, 1);
  digitalWrite(slider_dir,0);  
  for (int x=0; x<2048; x++) {
    digitalWrite(slider_cmd,1);
    delayMicroseconds(dl);
    digitalWrite(slider_cmd,0);
    delayMicroseconds(dl);
  }
  digitalWrite(led_R, 0);
  delay(2000);
}

void relayTest() { //activates in turns both pump and valve relay
  digitalWrite(pump, 1);
  digitalWrite(led_G, 1);
  delay(1000);
  digitalWrite(led_G, 0);
  digitalWrite(pump, 0);
  delay(1000);
  digitalWrite(valve, 1);
  digitalWrite(led_R, 1);
  delay(1000);
  digitalWrite(led_R, 0);
  digitalWrite(valve, 0);
  delay(2000);
}

void lsTest(){ //green light on if one switch is closed
  bool a = digitalRead(ls_slider_front);
  bool b = digitalRead(ls_slider_back);
  bool c = digitalRead(ls_lift_bottom);
  bool d = digitalRead(ls_lift_top);
  //a || b || c || d
  if (!a || !b || !c || !d) {
    digitalWrite(led_G,1);
  }
  else {
    digitalWrite(led_G,0);
  }
}

//-----------------------------------ARCHIVES-------------------------------------------
/*
void DEMO(){
  bool lsb = digitalRead(ls_lift_bottom);
  bool lst = digitalRead(ls_lift_top);
  if (!lsb && lslt_last) {
    grippSequence();
    digitalWrite(lift_dir, 1);
    lslt_last = 0;
  }
  else if (!lst && !lslt_last) {
    releaseSequence();
    digitalWrite(lift_dir, 0);
    lslt_last = 1;
  }
  else {
    digitalWrite(lift_cmd, 1);
  }

  digitalWrite(led_G, 0);
  digitalWrite(led_R, 1);
}
void grippSequence() {
  bool lsf = digitalRead(ls_slider_front);
  int dl = 1300;
  digitalWrite(slider_dir, 1);
  while (!lsf) {
      digitalWrite(slider_cmd,1);
      delayMicroseconds(1300);
      digitalWrite(slider_cmd,0);
      delayMicroseconds(1300);
      lsf = digitalRead(ls_slider_front);
  }
  digitalWrite(pump,1);
  digitalWrite(led_G,0);
  digitalWrite(led_R,0);
  delay(2000);
  digitalWrite(pump,0);
  digitalWrite(led_G,1);
  digitalWrite(led_R,0);
  bool lsb = digitalRead(ls_slider_back);
  digitalWrite(slider_dir, 0);
  while (!lsb) {
    digitalWrite(slider_cmd,1);
    delayMicroseconds(1300);
    digitalWrite(slider_cmd,0);
    delayMicroseconds(1300);
    lsb = digitalRead(ls_slider_back);
  }
  digitalWrite(valve,1);
  digitalWrite(led_G,0);
  digitalWrite(led_R,0);
  delay(2000);
  digitalWrite(valve,0);
  digitalWrite(led_G,0);
  digitalWrite(led_R,1);

}

void releaseSequence(){
  //pass
}

void vacuumTest(){
  //if front ls if closed, start pump for 2s, if back ls is closed open valve for 2s
  bool lsf = digitalRead(ls_slider_front);
  bool lsb = digitalRead(ls_slider_back);
  if (!lsf) {
    digitalWrite(pump, 1);
    digitalWrite(led_G, 0);
    delay(2000);
  }
  else {
    digitalWrite(pump, 0);
    digitalWrite(led_G, 1);
  }
  if (!lsb) {
    digitalWrite(valve, 1);
    digitalWrite(led_R, 0);
    delay(2000);
  }
  else {
    digitalWrite(led_R, 1);
    digitalWrite(valve, 0);
  }
}

void stepperTest(){ //makes the stepper turn
  digitalWrite(slider_dir,0); // set the direction ofd the rotation (1 is clockwise)
  //turn on an off the pin delay max 
  digitalWrite(slider_cmd,1); 
  delayMicroseconds(400); 
  digitalWrite(slider_cmd,0);
  delayMicroseconds(400); 
}
bool slider(int direction=0, int speed=50; int duration=1, long int start_time=millis()) {
  //direction (1 is clockwise, 0 counterClockwise)
  //speed (0-100) ~ (400-1500) converted to delay in ns
  //durations time in ms
  int delay =1500-11*speed; // ((100-speed)/100)*1100 + 400
  digitalWrite(slider_dir, direction);
  if (millis()-start_time < duration) {
    digitalWrite(slider_cmd,1)
    delayMicroseconds(delay);
    digitalWrite(slider_cmd,0);
    delayMicroseconds(delay)
    return 1;
  }
  return 0;
}
void sliderTest(){
  start_time = 
  while ( slider(1,70,1000))
}
*/

/*
void stepper28BYJTest(int steps, int direction, int delay){
  //delay is in ms (min 2);
  //direction (1 is cloxkmise)
  for (int x=0; x<steps; x++) {
    if (!direction) {
      Stepper28BYJCounterClockwise(delay);
    }
    else {
      Stepper28BYJClockwise(delay);
    }
  }
}
void Stepper28BYJClockwise(int dl){
  digitalWrite(in1, HIGH);digitalWrite(in2, LOW);digitalWrite(in3, LOW);digitalWrite(in4, HIGH);delay(dl);
  digitalWrite(in1, HIGH);digitalWrite(in2, HIGH);digitalWrite(in3, LOW);digitalWrite(in4, LOW);delay(dl);
  digitalWrite(in1, LOW);digitalWrite(in2, HIGH);digitalWrite(in3, HIGH);digitalWrite(in4, LOW);delay(dl);
  digitalWrite(in1, LOW);digitalWrite(in2, LOW);digitalWrite(in3, HIGH);digitalWrite(in4, HIGH);delay(dl);
}
void Stepper28BYJCounterClockwise(int dl){
  digitalWrite(in1, LOW);digitalWrite(in2, LOW);digitalWrite(in3, HIGH);digitalWrite(in4, HIGH);delay(dl);
  digitalWrite(in1, LOW);digitalWrite(in2, HIGH);digitalWrite(in3, HIGH);digitalWrite(in4, LOW);delay(dl);
  digitalWrite(in1, HIGH);digitalWrite(in2, HIGH);digitalWrite(in3, LOW);digitalWrite(in4, LOW);delay(dl);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(in3, LOW); digitalWrite(in4, HIGH);delay(dl);
}
*/