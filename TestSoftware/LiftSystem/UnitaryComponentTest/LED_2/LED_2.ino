#include "Adafruit_NeoPixel.h"
#define PIN  5
#define NUMPIXELS 120
Adafruit_NeoPixel strip (NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  strip.begin();
  strip.setBrightness(50);  // luminosité de la LED (maximum 255)
  strip.clear();
  Serial.begin(9600);


  // strip.setPixelColor(4, strip.Color(255, 0, 0));
  // strip.setPixelColor(5, strip.Color(255, 0, 0));
  // strip.setPixelColor(34, strip.Color(255, 0, 0));
  // strip.setPixelColor(35, strip.Color(255, 0, 0));
  // strip.setPixelColor(64, strip.Color(255, 0, 0));
  // strip.setPixelColor(65, strip.Color(255, 0, 0));
  // strip.setPixelColor(94, strip.Color(255, 0, 0));
  // strip.setPixelColor(95, strip.Color(255, 0, 0));
  // strip.show();

  

}



void loop() {
//  Circling(5, 1, strip.Color(255,165,0));
//  headLight();
audi();
}

void ledLiftUp() {
  Circling(10, 1, strip.Color(0,255,0));
}

void ledLiftDown(){
  Circling(10, -1, strip.Color(255,0,0));
}


void Circling(int led_quant, bool clockwise, uint32_t color) {
  static int pos;
  static int spacing = NUMPIXELS / led_quant;
  int x;

  if (clockwise) {
    for (int i = 0; i < led_quant; i++) {
      x = (pos + i * spacing) % NUMPIXELS;
      strip.setPixelColor(x, strip.Color(0,0,0));
      if (strip.getPixelColor(x)==color) {
        strip.setPixelColor(x, strip.Color(0,0,0));
      }
    }
    for (int i = 0; i < led_quant; i++) {
      x = (pos -1 + i * spacing) % NUMPIXELS;
      if (strip.getPixelColor(x)==strip.Color(0,0,0)) {
        strip.setPixelColor(x, color);
      }
    }
    pos = (pos + 1) % NUMPIXELS;
  }
  else {
    for (int i = NUMPIXELS; i >= 0; i--) {
      x = (pos + i * spacing) % NUMPIXELS;
      if (strip.getPixelColor(x)==color) {
        
      }
      strip.setPixelColor(x, strip.Color(0,0,0));
    }
    for (int i = NUMPIXELS; i >= 0; i--) {
      x = (pos + 1 + i * spacing) % NUMPIXELS;
      if (strip.getPixelColor(x)==strip.Color(0,0,0)) {
        strip.setPixelColor(x, color);
      }
      
      
    }
    pos = (pos - 1) % NUMPIXELS;
  }

  strip.show(); // Update the LED strip
  delay(30); // Adjust the delay as needed to control the speed of the animation
}



void headLight() {
  strip.setPixelColor(54,strip.Color(255,255,255));
  strip.setPixelColor(55,strip.Color(255,255,255));
  strip.setPixelColor(56,strip.Color(255,255,255));
  strip.setPixelColor(73,strip.Color(255,255,255));
  strip.setPixelColor(74,strip.Color(255,255,255));
  strip.setPixelColor(75,strip.Color(255,255,255));
  strip.show();
}

void audi(){
  static int pos_l = 64;
  static int pos_r = 65;
  static int wave_nb = 0;
  static int delay_wave = 8000;
  if (wave_nb < 59) {
    if (pos_l==5){
      pos_l=64;
      pos_r=65;
      delay_wave=delay_wave/1.5;
      wave_nb++;
    }

    //éteint les les leds de la vague d'avant
    strip.setPixelColor(pos_r-1,strip.Color(0,0,0));
    strip.setPixelColor(pos_l+1,strip.Color(0,0,0));

    //allume les leds de la nouvelle vague
    strip.setPixelColor(pos_r,strip.Color(255,255,255));
    strip.setPixelColor(pos_l,strip.Color(255,255,255));

    strip.show();
    delayMicroseconds(delay_wave);

    for (int x=0; x<wave_nb; x++) {
      strip.setPixelColor(5+x,strip.Color(255,255,255));
      strip.setPixelColor(124-x,strip.Color(255,255,255));
    }

    pos_r++;
    pos_l--;
  }
}
