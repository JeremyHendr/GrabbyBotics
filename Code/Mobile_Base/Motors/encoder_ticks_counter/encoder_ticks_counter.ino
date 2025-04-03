/* Project: Grabby Warehouse
 * Code: Counts the ticks of the encoder per revolution of the motor shaft and gearbox
 * Julius Ortstadt
 * 06.04.2024
 * Help: https://automaticaddison.com/calculate-pulses-per-revolution-for-a-dc-motor-with-encoder/
*/

// Define the pins for the motor encoder
// WARNING: Pins need to be pins that can be interrupted (see list for Arduino)
#define enc_M1_A 2 // Yellow for encoder A
#define enc_M1_B 3 // White for encoder B

#define enc_M2_A 18 // Yellow for encoder A
#define enc_M2_B 19 // White for encoder B

// Tick counter
volatile long M1_tick_count = 0;
volatile long M2_tick_count = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Set the pinMode of the encoders
  pinMode(enc_M1_A, INPUT_PULLUP);
  //pinMode(enc_M2_A, INPUT_PULLUP);

  // When the pin goes HIGH we interrupt (therefore the volatile long)
  attachInterrupt(digitalPinToInterrupt(enc_M1_A), M1_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_M2_A), M2_tick, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Tick M1: ");
  Serial.println(M1_tick_count);

  Serial.print("Tick M2: ");
  Serial.println(M2_tick_count);
}


void M1_tick(){
  M1_tick_count++;
}

void M2_tick(){
  M2_tick_count++;
}