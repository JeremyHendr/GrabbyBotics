#ifndef LEDSTRIP_HPP
#define LEDSTRIP_HPP

#include "Adafruit_NeoPixel.h"
#define led_strip_pin  5
#define led_strip_pixel_quant 120
Adafruit_NeoPixel strip (led_strip_pixel_quant, led_strip_pin, NEO_GRB + NEO_KHZ800);



void Circling(int led_quant, bool clockwise, uint32_t color) {
  static int pos;
  static int spacing = led_strip_pixel_quant / led_quant;
  int x;

  if (clockwise) {
    for (int i = 0; i < led_quant; i++) {
      x = (pos + i * spacing) % led_strip_pixel_quant;
      strip.setPixelColor(x, strip.Color(0,0,0));
      if (strip.getPixelColor(x)==color) {
        strip.setPixelColor(x, strip.Color(0,0,0));
      }
    }
    for (int i = 0; i < led_quant; i++) {
      x = (pos -1 + i * spacing) % led_strip_pixel_quant;
      if (strip.getPixelColor(x)==strip.Color(0,0,0)) {
        strip.setPixelColor(x, color);
      }
    }
    pos = (pos + 1) % led_strip_pixel_quant;
  }
  else {
    for (int i = led_strip_pixel_quant; i >= 0; i--) {
      x = (pos + i * spacing) % led_strip_pixel_quant;
      if (strip.getPixelColor(x)==color) {
        
      }
      strip.setPixelColor(x, strip.Color(0,0,0));
    }
    for (int i = led_strip_pixel_quant; i >= 0; i--) {
      x = (pos + 1 + i * spacing) % led_strip_pixel_quant;
      if (strip.getPixelColor(x)==strip.Color(0,0,0)) {
        strip.setPixelColor(x, color);
      }
      
      
    }
    pos = (pos - 1) % led_strip_pixel_quant;
  }

  strip.show(); // Update the LED strip
  delay(30); // Adjust the delay as needed to control the speed of the animation
}

void ledLiftUp() {
  Circling(10, 1, strip.Color(0,255,0));
}

void ledLiftDown(){
  Circling(10, 0, strip.Color(255,0,0));
}

#endif LEDSTRIP_HPP