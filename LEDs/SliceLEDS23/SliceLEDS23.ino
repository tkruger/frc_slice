#include <Adafruit_NeoPixel.h>  

#define ledPIN 8 
#define rioPin 2

#define NUMPIXELS 39 

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, ledPIN, NEO_GRB + NEO_KHZ800);  
int delayval = 30; // timing delay in milliseconds  
int redColor = 0;  
int greenColor = 0;  
int blueColor = 0;  

void setup() {  
  // Initialize the NeoPixel library.  
  pixels.begin();  
}  

void loop() {  
  setColor();  
  
  for (int i=0; i < NUMPIXELS; i++) {  
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255  
    pixels.setPixelColor(i, pixels.Color(redColor, greenColor, blueColor));  
    // This sends the updated pixel color to the hardware.  
    pixels.show();  
    // Delay for a period of time (in milliseconds).  
    delay(delayval);  
  }  
}  
// setColor()  
// picks random values to set for RGB  
void setColor(){  
  redColor = random(0, 255);  
  greenColor = random(0,255);  
  blueColor = random(0, 255);  
}  