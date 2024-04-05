void setup() {
  // put your setup code here, to run once:
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  digitalWrite(52, 1);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(53, 1);
  delayMicroseconds(700);
  digitalWrite(53, 0);
  delayMicroseconds(700);

}
