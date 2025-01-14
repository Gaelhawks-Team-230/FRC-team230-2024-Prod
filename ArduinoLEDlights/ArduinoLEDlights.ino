set #include <FastLED.h>

#define NUM_LEDS 12
#define DATA_PIN 6
CRGB leds[NUM_LEDS];

void setup() {
  // put your setup code here, to run once:
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(120);
  FastLED.clear();

  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}


void loop() {
  int bits = 0;
  int bit0 = digitalRead(12);
  int c = digitalRead(11);
  int bit1 = c << 1;
  c = digitalRead(10);
  int bit2 = c << 2;
  bits = bit0 | bit1 | bit2;

  switch (bits) {
    case 0:
      DisplayBlank();  // 000
      break;
    case 1:
      DisplayGamepieceWanted();
      break;
    case 2:
      DisplayConnected();  // 010
      break;
    case 3:
      DisplayHunting();  // 110

      break;
    case 4:
      DisplayDisconnected();  // 001
      break;
    case 5:
      DisplayHasGamepiece();  // 101
      break;
    case 6:
      DisplayAmplify();  // 011

      break;
    case 7:
      // Default State - display code disconnected
      DisplayCodeDisconnected();  // 111
      break;
  }
}

// Displays blank (black/off)
void DisplayBlank() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(300);
}


// Displays flashing red for Code running but no DS
void DisplayDisconnected() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(250);

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(250);
}

// scorlling animation green
void DisplayConnected() {
  cylonAnimation(2, CRGB::Green, 100);
}

// Displays flashing blue to signal amplify for the human player
void DisplayAmplify() {
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(150);

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(150);
}

// Flashes orange to signal gamepiece
void DisplayGamepieceWanted() {
  fill_solid(leds, NUM_LEDS, CRGB(255,51,51));
  FastLED.show();
  delay(150);

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(150);
}

// Flashes green for 2 seconds then stops
void DisplayHasGamepiece() {
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(150);

  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(150);
}

// Displays fire animation
void DisplayHunting() {
  Fire(50, 100, 10);
}

void DisplayCodeDisconnected() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(100);
}

// * Animations
void cylonAnimation(int eyeSize, CRGB color, int delayTime) {
  int startPosition = 0;
  int endPosition = NUM_LEDS;

  // Move the eye from one end to the other
  for (int i = startPosition; i <= endPosition - eyeSize; i++) {
    // Turn off all LEDs
    fill_solid(leds, NUM_LEDS, CRGB::Black);

    // Turn on the LEDs for the eye
    for (int j = i; j < i + eyeSize; j++) {
      leds[j] = color;
    }

    FastLED.show();
    delay(delayTime);
  }

  // Move the eye back from the other end to the starting position
  for (int i = endPosition - eyeSize; i >= startPosition; i--) {
    // Turn off all LEDs
    fill_solid(leds, NUM_LEDS, CRGB::Black);

    // Turn on the LEDs for the eye
    for (int j = i; j < i + eyeSize; j++) {
      leds[j] = color;
    }

    FastLED.show();
    delay(delayTime);
  }
}


// FlameHeight - Use larger value for shorter flames, default=50.
// Sparks - Use larger value for more ignitions and a more active fire (between 0 to 255), default=100.
// DelayDuration - Use larger value for slower flame speed, default=10.

void Fire(int FlameHeight, int Sparks, int DelayDuration) {
  static byte heat[NUM_LEDS];
  int cooldown;

  // Cool down each cell a little
  for (int i = 0; i < NUM_LEDS; i++) {
    cooldown = random(0, ((FlameHeight * 10) / NUM_LEDS) + 2);

    if (cooldown > heat[i]) {
      heat[i] = 0;
    }
    else {
      heat[i] = heat[i] - cooldown;
    }
  }

  // Heat from each cell drifts up and diffuses slightly
  for (int k = (NUM_LEDS - 1); k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Randomly ignite new Sparks near bottom of the flame
  if (random(255) < Sparks) {
    int y = random(7);
    heat[y] = heat[y] + random(160, 255);
  }

  // Convert heat to LED colors
  for (int j = 0; j < NUM_LEDS; j++) {
    setPixelHeatColor(j, heat[j]);
  }

  FastLED.show();
  delay(DelayDuration);
}

void setPixelHeatColor(int Pixel, byte temperature) {
  // Rescale heat from 0-255 to 0-191
  byte t192 = round((temperature / 255.0) * 191);

  // Calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0...63
  heatramp <<= 2; // scale up to 0...252

  // Figure out which third of the spectrum we're in:
  if (t192 > 0x80) {                   // hottest
    leds[Pixel].setRGB(255, 255, heatramp);
  }
  else if (t192 > 0x40) {              // middle
    leds[Pixel].setRGB(255, heatramp, 0);
  }
  else {                               // coolest
    leds[Pixel].setRGB(heatramp, 0, 0);
  }
}