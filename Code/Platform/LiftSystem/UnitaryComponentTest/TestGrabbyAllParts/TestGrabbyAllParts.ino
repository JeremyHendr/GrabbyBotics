//Nema17Stepper
#define dir_pin 52
#define stepper_pin 53

//led
#define led_R 48
#define led_G 49

//LS
#define ls_slider_back 42
#define ls_slider_front 43
#define ls_bottom 50
#define ls_top 51

//relay
#define pump 22
#define valve 23


//----------------------------------SETUP---------------------------------------------
void setup() {
  Serial.begin(9600);
  //stepper
  pinMode(dir_pin, OUTPUT);  // DIR Pin 
  pinMode(stepper_pin, OUTPUT);  // STEP Pin
  //led
  pinMode(led_R, OUTPUT);
  pinMode(led_G, OUTPUT);
  //LS
  pinMode(ls_slider_front, INPUT_PULLUP);
  pinMode(ls_slider_back, INPUT_PULLUP);
  pinMode(ls_bottom, INPUT_PULLUP);
  pinMode(ls_top, INPUT_PULLUP);
  //realys
  pinMode(pump, OUTPUT);
  pinMode(valve, OUTPUT);
  //
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(led_R,0);
  digitalWrite(led_G,0);
  delay(1000);
  digitalWrite(led_R,1);
  digitalWrite(led_G,1);
}
//----------------------------------LOOP---------------------------------------------
void loop() {

}

//----------------------------------FUNCTIONS----------------------------------------
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

void relayTest() { //activates in turns both pump and valve relay
  digitalWrite(pump, 1);
  digitalWrite(led_G, 0);
  delay(1000);
  digitalWrite(led_G, 1);
  digitalWrite(pump, 0);
  delay(1000);
  digitalWrite(valve, 1);
  digitalWrite(led_R, 0);
  delay(1000);
  digitalWrite(led_R, 1);
  digitalWrite(valve, 0);
  delay(5000);
}

void lsTest(){ //green light on if one switch is closed
  bool a = digitalRead(ls_slider_front);
  bool b = digitalRead(ls_slider_back);
  bool c = digitalRead(ls_bottom);
  bool d = digitalRead(ls_top);
  //a || b || c || d
  if (!a || !b || !c || !d) {
    digitalWrite(led_G,0);
  }
  else {
    digitalWrite(led_G,1);
  }
}

void stepperTest(){ //makes the stepper turn
  digitalWrite(dir_pin,0); // set the direction ofd the rotation (1 is clockwise)
  //turn on an off the pin delay max 
  digitalWrite(stepper_pin,1); 
  delayMicroseconds(400); 
  digitalWrite(stepper_pin,0);
  delayMicroseconds(400); 
}

void stepperTest2() { //stepper does one turn clockwise and one cc
  digitalWrite(dir_pin,1);  
  for (int x=0; x<2048; x++) {
    digitalWrite(stepper_pin,1);
    delayMicroseconds(500);
    digitalWrite(stepper_pin,0);
    delayMicroseconds(500);
  }
  delay(2000);
  digitalWrite(dir_pin,0);  
  for (int x=0; x<2048; x++) {
    digitalWrite(stepper_pin,1);
    delayMicroseconds(500);
    digitalWrite(stepper_pin,0);
    delayMicroseconds(500);
  }
  delay(2000)
}

void sliderTest() {
  //forward (1) when cw, backward (0) when ccw
  bool lsf = digitalRead(ls_slider_front);
  bool lsb = digitalRead(ls_slider_back);
  if (!lsf) { //front ls closed
    digitalWrite(dir_pin, 0);
    digitalWrite(led_G,1);
    digitalWrite(led_R,0);
  }
  else if (!lsb) { //back ls closed
    digitalWrite(dir_pin, 1);
    digitalWrite(led_G,0);
    digitalWrite(led_R,1);
  }
  digitalWrite(stepper_pin,1);
  delayMicroseconds(500);
  digitalWrite(stepper_pin,0);
  delayMicroseconds(500);
}

//-----------------------------------ARCHIVES-------------------------------------------
/*
bool slider(int direction=0, int speed=50; int duration=1, long int start_time=millis()) {
  //direction (1 is clockwise, 0 counterClockwise)
  //speed (0-100) ~ (400-1500) converted to delay in ns
  //durations time in ms
  int delay =1500-11*speed; // ((100-speed)/100)*1100 + 400
  digitalWrite(dir_pin, direction);
  if (millis()-start_time < duration) {
    digitalWrite(stepper_pin,1)
    delayMicroseconds(delay);
    digitalWrite(stepper_pin,0);
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