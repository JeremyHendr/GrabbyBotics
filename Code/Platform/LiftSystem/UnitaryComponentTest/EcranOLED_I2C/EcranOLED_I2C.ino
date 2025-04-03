#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 ecranOLED(128, 256, &Wire, -1);

#define black 0             // Draw 'off' pixels
#define SSD1306_WHITE 1             // Draw 'on' pixels
#define SSD1306_INVERSE 2           // Invert pixels



void setup() {
  Serial.begin(115200);
  ecranOLED.begin(SSD1306_SWITCHCAPVCC, 0x3C); //0x3C , 0x3D , 0x78  , 0x7A


  // int16_t posX = 40;
  // int16_t posY = 20;
  // int16_t couleur = SSD1306_WHITE;
  // ecranOLED.drawPixel(posX, posY, couleur);

  ecranOLED.clearDisplay();
  ecranOLED.invertDisplay(0);
  ecranOLED.fillScreen(0);
  ecranOLED.display();
  Serial.println("cleared");
  delay(2000);

  for(byte numeroLigne=0 ; numeroLigne < 128 ; numeroLigne++) {
    for(byte numeroColonne=0 ; numeroColonne < 64 ; numeroColonne++) {
      ecranOLED.drawPixel(numeroColonne, numeroLigne, 2);          // On demande l'allumage de chaque pixel, un à un, avec la commande : drawPixel(posX,posY,couleur);
      ecranOLED.display();
      delay(5);
    }
  }
  ecranOLED.display();
  delay(5000);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("invert");
  ecranOLED.invertDisplay(1);
  delay(2000);
  ecranOLED.invertDisplay(0);
  delay(2000);

  // ecranOLED.clearDisplay();
  // ecranOLED.display();
  // delay(5000);

  // for(byte numeroLigne=0 ; numeroLigne < 64 ; numeroLigne++) {
  //   for(byte numeroColonne=0 ; numeroColonne < 128 ; numeroColonne++) {
  //     ecranOLED.drawPixel(numeroColonne, numeroLigne, SSD1306_WHITE);
  //     ecranOLED.display();          // On demande l'allumage de chaque pixel, un à un, avec la commande : drawPixel(posX,posY,couleur);
  //   }
  // }

  // delay(5000);
  

}
