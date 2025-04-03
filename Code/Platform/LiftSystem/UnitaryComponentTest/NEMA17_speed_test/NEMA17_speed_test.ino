#define dir_pin 52
#define stepper_pin 53
#define led_R 48
#define led_G 49
#define limit_switch_down 50
#define limit_switch_up 51

const bool stepper_down = 0;
const bool stepper_up = !stepper_down;
bool stepper_direction;
int speed_stepper = 0;
int delay_stepper = 0;


void setup() {
  Serial.begin(9600);
  pinMode(dir_pin, OUTPUT);  // DIR Pin 
  pinMode(stepper_pin, OUTPUT);  // STEP Pin
  pinMode(led_R, OUTPUT);
  pinMode(led_G, OUTPUT);
  pinMode(limit_switch_down, INPUT_PULLUP);
  pinMode(limit_switch_up, INPUT_PULLUP);
 
  digitalWrite(led_R,0);
  digitalWrite(led_G,0);
  delay(1000);
  digitalWrite(led_R,1);
  digitalWrite(led_G,1);
  stepper_direction = stepper_up;
  delay_stepper = 0;
  
}


void loop() {
  

  if (!digitalRead(limit_switch_down)) {
    delay_stepper = delay_stepper+100;
    digitalWrite(led_G,0);
    Serial.println(delay_stepper);
    delay(200);
    digitalWrite(led_G,1);
  }
  if(!digitalRead(limit_switch_up)){
    delay_stepper = delay_stepper-100;
    digitalWrite(led_R,0);
    Serial.println(delay_stepper);
    delay(200);
    digitalWrite(led_R,1);
  }
  
  digitalWrite(dir_pin,stepper_direction); // DIR sens1  
  digitalWrite(stepper_pin,1); 
  delayMicroseconds(delay_stepper);
  digitalWrite(stepper_pin,0);
  delayMicroseconds(delay_stepper);
  
 
}
