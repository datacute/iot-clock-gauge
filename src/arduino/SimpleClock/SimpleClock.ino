#include <Wire.h>
#include <DS1307.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//store the current time data
int rtc[7];

#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  DDRC |= _BV(2) | _BV(3); // POWER:Vcc Gnd
  PORTC |= _BV(3); // VCC PINC3
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(16);
}

void loop() {
  uint16_t i;
  //get current time
  RTC.get(rtc, true);
  int hours = rtc[2];
  if (hours >= 12) hours -= 12;
  int minutes = rtc[1];
  int seconds = rtc[0];
  int hoursPixel = hours * 5 + minutes/12;
  strip.clear();
  strip.setPixelColor(mod60(hoursPixel-2), 0, 0, 16); // hours in blue
  strip.setPixelColor(mod60(hoursPixel-1), 0, 0, 32); // hours in blue
  strip.setPixelColor(mod60(hoursPixel+1), 0, 0, 32); // hours in blue
  strip.setPixelColor(mod60(hoursPixel+2), 0, 0, 16); // hours in blue
  strip.setPixelColor(mod60(minutes-1), 0, 32, 0); // minutes in green
  strip.setPixelColor(mod60(minutes+1), 0, 32, 0); // minutes in green
  strip.setPixelColor(0, 127, 127, 127);
  strip.setPixelColor(5, 32, 32, 32);
  strip.setPixelColor(10, 32, 32, 32);
  strip.setPixelColor(15, 127, 127, 127);
  strip.setPixelColor(20, 32, 32, 32);
  strip.setPixelColor(25, 32, 32, 32);
  strip.setPixelColor(30, 127, 127, 127);
  strip.setPixelColor(35, 32, 32, 32);
  strip.setPixelColor(40, 32, 32, 32);
  strip.setPixelColor(45, 127, 127, 127);
  strip.setPixelColor(50, 32, 32, 32);
  strip.setPixelColor(55, 32, 32, 32);
  strip.setPixelColor(hoursPixel, 0, 0, 127); // hours in blue
  strip.setPixelColor(minutes, 0, 127, 0); // minutes in green
  strip.setPixelColor(seconds, 127, 0, 0); // seconds in red
  strip.show();
  delay(100);
}

uint8_t mod60(int8_t v)
{
  while (v < 0) v += 60;
  while (v >= 60) v -= 60;
  return v;
}
