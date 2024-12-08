

#include <FastLED_NeoPixel.h>

// Which pin on the Arduino is connected to the LEDs?
#define DATA_PIN 6

// How many LEDs are attached to the Arduino?
#define NUM_LEDS 10

// LED brightness, 0 (min) to 255 (max)
//#define BRIGHTNESS 50

// Amount of time for each half-blink, in milliseconds
#define BLINK_TIME 1000

/* Declare the NeoPixel strip object:
*     * Argument 1 = Number of LEDs in the LED strip
*     * Argument 2 = Arduino pin number
*     * Argument 3 = LED strip color order
* 
* The FastLED_NeoPixel version uses template arguments instead of function
* arguments. Note the use of '<>' brackets!
* 
* You can switch between libraries by commenting out one of these two objects.
* In this example they should behave identically.
*/
// Adafruit_NeoPixel strip(NUM_LEDS, DATA_PIN, NEO_GRB);  // <- Adafruit NeoPixel version
FastLED_NeoPixel<NUM_LEDS, DATA_PIN, NEO_GRB> neo_pixels1;      // <- FastLED NeoPixel version


void setup() {
	neo_pixels1.begin();  // initialize strip (required!)
	//strip.setBrightness(BRIGHTNESS);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
}

void loop() {

  int bits = digitalRead(10) + (2 * digitalRead(11)) + (4 * digitalRead(12));
  //int bits = 1;

  switch(bits)
  {
    case 0:
      for(int i = 0; i < NUM_LEDS; i ++){
        neo_pixels1.setPixelColor(i, neo_pixels1.Color(0, 0, 0));
      }
      neo_pixels1.show();
      break;
    case 1:
      for(int i = 0; i < NUM_LEDS; i ++){
        neo_pixels1.setPixelColor(i, neo_pixels1.Color(0, 10, 75));
      }
      neo_pixels1.show();
      break;
    case 2:
      for(int i = 0; i < NUM_LEDS; i ++){
        neo_pixels1.setPixelColor(i, neo_pixels1.Color(75, 20, 0));
      }
      neo_pixels1.show();
      break;
    case 3:
      for(int x = 0; x < 3; x++){
        for(int i = 0; i < NUM_LEDS; i ++){
          neo_pixels1.setPixelColor(i, neo_pixels1.Color(85, 10, 0));
        }
        neo_pixels1.show();
        delay(100);
        for(int i = 0; i < NUM_LEDS; i ++){
          neo_pixels1.setPixelColor(i, neo_pixels1.Color(0, 0, 0));
        }
        neo_pixels1.show();
        delay(100);
      }
      delay(1000);        
      break;
  }
  
}
