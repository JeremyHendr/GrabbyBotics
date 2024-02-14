#define valve 13
#define motor 12
#define button 4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(valve, OUTPUT);
  pinMode(motor, OUTPUT);
  pinMode(button, INPUT);
 
  digitalWrite(valve,1);
  delay(1000);
  // digitalWrite(motor,1);
  // delay(1000);
  // digitalWrite(motor,0);
  // delay(50);
  digitalWrite(valve,0);
  // digitalWrite(valve, 1);
  // delay(5000);
  // digitalWrite(valve, 0);
  // Serial.println("done");
}

void loop() {
  //we suck air threw the cup, we create vacuum
  Serial.println(digitalRead(button));
  if (!digitalRead(button)) {
    digitalWrite(motor,1);
    delay(1500);
    digitalWrite(motor,0); 
    delay(10000);
    //we release vacuum by letting air go out
    digitalWrite(valve,1);
    delay(1000);
    digitalWrite(valve,0);
    delay(5000);
  }
}
