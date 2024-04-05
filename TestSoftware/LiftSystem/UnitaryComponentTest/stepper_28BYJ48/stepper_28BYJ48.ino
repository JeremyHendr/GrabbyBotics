// Inclut la bibliothèque Arduino Stepper.h:
#include <Stepper.h>

// Définit le nombre de pas par rotation:
const int stepsPerRevolution = 2048;

// Câblage:
// Broche 8 à IN1 sur le pilote ULN2003
// Broche 9 à IN2 sur le pilote ULN2003
// Broche 10 à IN3 sur le pilote ULN2003
// Broche 11 à IN4 sur le pilote ULN2003

// Créez un objet stepper appelé 'myStepper', notez l'ordre des broches:
Stepper myStepper = Stepper ( stepsPerRevolution, 44, 45, 46, 47 ) ;

void setup() {
  // Réglez la vitesse sur 5 tr / min:
  myStepper.setSpeed(5); //20 un peu trop
  // pinMode(44, OUTPUT);
  // pinMode(45, OUTPUT);
  // pinMode(46, OUTPUT);
  // pinMode(47, OUTPUT);
  // digitalWrite(44,1);
  // delay(500);
  // digitalWrite(45,1);
  // delay(500);
  // digitalWrite(46,1);
  // delay(500);
  // digitalWrite(47,1);
  // delay(500);
  // digitalWrite(44,0);
  // digitalWrite(45,0);
  // digitalWrite(46,0);
  // digitalWrite(47,0);
  // delay(500);

// Commencez la communication série à une vitesse de transmission de 9600:
}

void loop() {
  // Étape d'une révolution dans une direction:
  myStepper.step(stepsPerRevolution);
  delay(5000);
  // myStepper.step(-stepsPerRevolution);
  // Étape d'une révolution dans l'autre sens:
  // Serial.println("counterclockwise");
  // myStepper.step(-stepsPerRevolution);
  // delay(500);
}