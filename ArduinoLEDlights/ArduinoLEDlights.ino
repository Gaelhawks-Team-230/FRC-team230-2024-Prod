// Adafruit FastLED_NeoPixel Library

#include <FastLED_NeoPixel.h>

// Number of individual LEDs on each of the strips
#define NUM_LEDSstrip 10

//'Setup for the LED strips
FastLED_NeoPixel<NUM_LEDSstrip, 6, NEO_GRB> neo_pixels1;
FastLED_NeoPixel<NUM_LEDSstrip, 10, NEO_GRB> neo_pixels2;

void setup() {
  neo_pixels1.begin();  // Start LED strip
  neo_pixels2.begin();
  //Inputs
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
}

void loop() {
  //"reset" the LEDs
  DisplayBlank();
  //Values can range from 0-7, this determines which case will run in the loop
  int bits = digitalRead(10) + (2 * digitalRead(11)) + (4 * digitalRead(12));
  bits = 7;
  switch (bits) {
    //DisplayBlank() | Display Nothing
    case 0:
      DisplayBlank();
      break;
    //DisplayDisconnected() | Solid Red
    case 1:
      DisplayDisconnected();
      break;
    //DisplayConnected() | Blinking Red
    case 2:
      DisplayConnected();
      break;
    //DisplayFMSConnected() | Green Animation
    case 3:
      DisplayFMSConnected();
      break;
    //DisplayAmplify() | Flashing Blue
    case 4:
      DisplayAmplify();
      break;
    //DisplayFeed() | Flashing Orange
    case 5:
      DisplayFeed();
      break;
    //DisplayHasNote() | Double Flash Green
    case 6:
      DisplayHasNote();
      break;
    //DisplayHunting() | *Cool Animation*
    case 7:
      DisplayHunting();
      break;
  }
}
//Function to make entire LED strip a color
void fillLEDs(int R, int G, int B){
  for (int i = 0; i < NUM_LEDSstrip; i++) {
    neo_pixels1.setPixelColor(i, neo_pixels1.Color(R, G, B));
    neo_pixels2.setPixelColor(i, neo_pixels2.Color(R, G, B));
  }
  neo_pixels1.show();
  neo_pixels2.show();
}
//Function to make LEDs blank
void DisplayBlank(){
  fillLEDs(0, 0, 0);
}
void DisplayDisconnected(){
  fillLEDs(150, 0, 0);
}
void DisplayConnected(){
  fillLEDs(150, 0, 0);
      delay(250);
      DisplayBlank();
      delay(250);
}
void DisplayFMSConnected(){
    for (int i = 1; i < NUM_LEDSstrip; i++){
    neo_pixels1.setPixelColor(i, neo_pixels1.Color(0, 150, 0));
    neo_pixels2.setPixelColor(i, neo_pixels2.Color(0, 150, 0));
    neo_pixels1.setPixelColor(i - 1, neo_pixels1.Color(0, 100, 0));
    neo_pixels2.setPixelColor(i - 1, neo_pixels2.Color(0, 100, 0));
    neo_pixels1.setPixelColor(i - 2, neo_pixels1.Color(0, 50, 0));
    neo_pixels2.setPixelColor(i - 2, neo_pixels2.Color(0, 50, 0));
    neo_pixels1.show();
    neo_pixels2.show();
    delay(1000 / NUM_LEDSstrip);
    DisplayBlank();
  }
  for (int i = NUM_LEDSstrip - 2; i > 0; i = i - 1){
    neo_pixels1.setPixelColor(i, neo_pixels1.Color(0, 150, 0));
    neo_pixels2.setPixelColor(i, neo_pixels2.Color(0, 150, 0));
    neo_pixels1.setPixelColor(i + 1, neo_pixels1.Color(0, 100, 0));
    neo_pixels2.setPixelColor(i + 1, neo_pixels2.Color(0, 100, 0));
    neo_pixels1.setPixelColor(i + 2, neo_pixels1.Color(0, 50, 0));
    neo_pixels2.setPixelColor(i + 2, neo_pixels2.Color(0, 50, 0));
    neo_pixels1.show();
    neo_pixels2.show();
    delay(1000 / NUM_LEDSstrip);
    DisplayBlank();
  }
}
void DisplayAmplify(){
  fillLEDs(0, 0, 150);
  delay(250);
  DisplayBlank();
  delay(100);
  fillLEDs(0, 0, 150);
  delay(150);
  DisplayBlank();
  delay(500);
}
void DisplayFeed(){
  fillLEDs(150, 15, 0);
  delay(250);
  DisplayBlank();
  delay(100);
  fillLEDs(150, 15, 0);
  delay(150);
  DisplayBlank();
  delay(500);
}
void DisplayHasNote(){
  fillLEDs(0, 150, 0);
  delay(250);
  DisplayBlank();
  delay(250);
  fillLEDs(0, 150, 0);
  delay(500);
}
void DisplayHunting(){
  ledColor = 1 
  /*
  for (int i = 1; i < NUM_LEDSstrip; i++) {
      neo_pixels1.setPixelColor(i, neo_pixels1.Color(150, 15, 0));
      neo_pixels2.setPixelColor(i, neo_pixels2.Color(150, 15, 0));
      neo_pixels1.show();
      neo_pixels2.show();
      delay(150);
      DisplayBlank();
      neo_pixels1.setPixelColor(i + 2, neo_pixels1.Color(150, 15, 0));
      neo_pixels2.setPixelColor(i + 2, neo_pixels2.Color(150, 15, 0));
      neo_pixels1.show();
      neo_pixels2.show();
      delay(150);
      DisplayBlank();
    }
  */

}