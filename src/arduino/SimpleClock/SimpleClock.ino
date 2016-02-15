#include <DS1302RTC.h>
#include <Time.h>
#include <TimeLib.h>

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN D4

DS1302RTC RTC(D3, D2, D1);

enum PaletteColour { QUARTER_TICK, FIVE_MINUTE_TICK, HOUR, HOUR1, HOUR2, MINUTE, MINUTE1, SECOND};

uint32_t palette0[] = {0x808080,0x202020,0x800000,0x200000,0x100000,0x008000,0x002000,0x000080};

uint32_t* palettes[]={
  &palette0[0]
};

int currentPalette = 0;

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
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(32);
}

void loop() {
  tmElements_t tm;
  //get current time
  if (RTC.read(tm) == 0) {
    int hours = tm.Hour;
    while (hours >= 12) hours -= 12;
    int minutes = tm.Minute;
    int seconds = tm.Second;
    int hoursPixel = hours * 5 + minutes/12;
    
    uint32_t* p = palettes[currentPalette];
    
    strip.clear();
    strip.setPixelColor(mod60(hoursPixel-2), p[HOUR2]); // hours in blue
    strip.setPixelColor(mod60(hoursPixel-1), p[HOUR1]); // hours in blue
    strip.setPixelColor(mod60(hoursPixel+1), p[HOUR1]); // hours in blue
    strip.setPixelColor(mod60(hoursPixel+2), p[HOUR2]); // hours in blue
    strip.setPixelColor(mod60(minutes-1), p[MINUTE1]); // minutes in green
    strip.setPixelColor(mod60(minutes+1), p[MINUTE1]); // minutes in green
    strip.setPixelColor(0, p[QUARTER_TICK]);
    strip.setPixelColor(5, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(10, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(15, p[QUARTER_TICK]);
    strip.setPixelColor(20, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(25, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(30, p[QUARTER_TICK]);
    strip.setPixelColor(35, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(40, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(45, p[QUARTER_TICK]);
    strip.setPixelColor(50, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(55, p[FIVE_MINUTE_TICK]);
    strip.setPixelColor(hoursPixel, p[HOUR]);
    strip.setPixelColor(minutes, p[MINUTE]);
    strip.setPixelColor(seconds, p[SECOND]);
    strip.show();
  }
  delay(100);
}

uint8_t mod60(int8_t v)
{
  while (v < 0) v += 60;
  while (v >= 60) v -= 60;
  return v;
}
