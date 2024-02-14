//Nema17Stepper
#define dir_pin 52
#define stepper_pin 53

//led
#define led_R 4
#define led_G 5
#define led_B 6

//LS
#define ls_slider_back 23
#define ls_slider_front 22
#define ls_bottom 24
#define ls_top 25

//relay
#define pump 50
#define valve 51

//DC motor
#define lift_dir_pin 3
#define lift_pin 2

//variable
bool lsf_last=1; //to know is the limit switch slider front was the last pressed
bool lslt_last=0; 

//----------------------------------SETUP---------------------------------------------
void setup() {
  Serial.begin(9600);
  //stepper
  pinMode(dir_pin, OUTPUT);  // DIR Pin 
  pinMode(stepper_pin, OUTPUT);  // STEP Pin
  digitalWrite(stepper_pin, 0);
  digitalWrite(dir_pin, 0);
  digitalWrite(stepper_pin, 0);
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
  //DC motor
  pinMode(lift_dir_pin, OUTPUT);
  pinMode(lift_pin, OUTPUT);

  digitalWrite(led_R,1);
  delay(1000);
  digitalWrite(led_R,0);
  digitalWrite(led_G,1);
  delay(1000);
  digitalWrite(led_G,0);
  digitalWrite(led_B,1);
  delay(1000);
  digitalWrite(led_B,0);


}
//----------------------------------LOOP---------------------------------------------
void loop() {
  //TEST SEQUENCE: led,ls,relay,stepper,lift
  // lsTest(); //green light is ls pressed
  // relayTest(); //activate relays in turns
  stepperTest2(); //one turn cw one cc
  // sliderTest(); //slider moves back to front
  // sliderTest2(); //slider moves front to back and ativate pump and valve
  // DCmotorTest(); //3sec cc and 3sec cw
  // liftTest(); //

}

//----------------------------------FUNCTIONS----------------------------------------



//-----------------------------Test Sequence Functions-------------------------------------
void liftTest(){
  bool lsb = digitalRead(ls_bottom);
  bool lst = digitalRead(ls_top);
  if (!lsb && lslt_last) {
    digitalWrite(lift_dir_pin, 1);
    digitalWrite(lift_pin, 1);
    digitalWrite(led_G, 0);
    digitalWrite(led_R, 1);
    lslt_last = 0;
  }
  else if (!lst && !lslt_last) {
    digitalWrite(lift_dir_pin, 0);
    digitalWrite(lift_pin, 1);
    digitalWrite(led_G, 1);
    digitalWrite(led_R, 0);
    lslt_last = 1;
  }
}

void DCmotorTest() {
  //3sec one cw 3sec cc
  digitalWrite(lift_dir_pin, 1);
  digitalWrite(led_G, 0);
  digitalWrite(led_R, 1);
  digitalWrite(lift_pin, 1);
  delay(3000);
  digitalWrite(lift_pin,0);
  delay(500);
  digitalWrite(lift_dir_pin, 0);
  digitalWrite(led_G, 1);
  digitalWrite(led_R, 0);
  digitalWrite(lift_pin, 1);
  delay(3000);
  digitalWrite(lift_pin,0);
  delay(500);
}

void sliderTest2() {
  //forward (1) when cw, backward (0) when ccw
  bool lsf = digitalRead(ls_slider_front);
  bool lsb = digitalRead(ls_slider_back);
  if (!lsf && !lsf_last) { //front ls closed
    digitalWrite(dir_pin, 1);
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
    digitalWrite(dir_pin, 0);
    digitalWrite(valve,1);
    digitalWrite(led_G,0);
    digitalWrite(led_R,0);
    delay(2000);
    digitalWrite(valve,0);
    digitalWrite(led_G,0);
    digitalWrite(led_R,1);
    lsf_last = 0;
  }
  int dl = 1300;
  digitalWrite(stepper_pin,1);
  delayMicroseconds(1300);
  digitalWrite(stepper_pin,0);
  delayMicroseconds(1300);
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
  int dl = 1300;
  digitalWrite(stepper_pin,1);
  delayMicroseconds(1300);
  digitalWrite(stepper_pin,0);
  delayMicroseconds(1300);
}

void stepperTest2() { //stepper does one turn clockwise and one cc
   int dl = 800;
  digitalWrite(dir_pin,1);  
  for (int x=0; x<2048; x++) {
    digitalWrite(stepper_pin,1);
    delayMicroseconds(dl);
    digitalWrite(stepper_pin,0);
    delayMicroseconds(dl);
  }
  delay(2000);
  digitalWrite(dir_pin,0);  
  for (int x=0; x<2048; x++) {
    digitalWrite(stepper_pin,1);
    delayMicroseconds(dl);
    digitalWrite(stepper_pin,0);
    delayMicroseconds(dl);
  }
  delay(2000);
}

void relayTest() { //activates in turns both pump and valve relay
  digitalWrite(pump, 1);
  digitalWrite(led_G, 1);
  delay(1000);
  digitalWrite(led_G, 0);
  digitalWrite(pump, 0);
  delay(2000);
  digitalWrite(valve, 1);
  digitalWrite(led_R, 1);
  delay(1000);
  digitalWrite(led_R, 0);
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
    digitalWrite(led_G,1);
  }
  else {
    digitalWrite(led_G,0);
  }
}


//-----------------------------------ARCHIVES-------------------------------------------
/*
void DEMO(){
  bool lsb = digitalRead(ls_bottom);
  bool lst = digitalRead(ls_top);
  if (!lsb && lslt_last) {
    grippSequence();
    digitalWrite(lift_dir_pin, 1);
    lslt_last = 0;
  }
  else if (!lst && !lslt_last) {
    releaseSequence();
    digitalWrite(lift_dir_pin, 0);
    lslt_last = 1;
  }
  else {
    digitalWrite(lift_pin, 1);
  }

  digitalWrite(led_G, 0);
  digitalWrite(led_R, 1);
}
void grippSequence() {
  bool lsf = digitalRead(ls_slider_front);
  int dl = 1300;
  digitalWrite(dir_pin, 1);
  while (!lsf) {
      digitalWrite(stepper_pin,1);
      delayMicroseconds(1300);
      digitalWrite(stepper_pin,0);
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
  digitalWrite(dir_pin, 0);
  while (!lsb) {
    digitalWrite(stepper_pin,1);
    delayMicroseconds(1300);
    digitalWrite(stepper_pin,0);
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
  digitalWrite(dir_pin,0); // set the direction ofd the rotation (1 is clockwise)
  //turn on an off the pin delay max 
  digitalWrite(stepper_pin,1); 
  delayMicroseconds(400); 
  digitalWrite(stepper_pin,0);
  delayMicroseconds(400); 
}
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