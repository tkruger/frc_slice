#include <Adafruit_NeoPixel.h>  

#define ledPIN 8 
#define rioPin 2

#define NUMLOWPIXELS 39 
#define NUMHIGHPIXELS 38
#define NUMPIXELS NUMLOWPIXELS + NUMHIGHPIXELS

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, ledPIN, NEO_GRB + NEO_KHZ800);  
int delayval = 30; // timing delay in milliseconds 
 
int redColor = 0;  
int greenColor = 0;  
int blueColor = 0;  

int redAltColor = 0;  
int greenAltColor = 0;  
int blueAltColor = 0; 

void setup() {  
  // Initialize the NeoPixel library.  
  pixels.begin();  
}  

void loop() {  
  setColor();  
  setAltColor();
  
  for (int i=0; i < NUMLOWPIXELS; i++) {  
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255  
    pixels.setPixelColor(i, pixels.Color(redColor, greenColor, blueColor));  
    
    if(i < NUMHIGHPIXELS) {
      pixels.setPixelColor(i + NUMLOWPIXELS, pixels.Color(redAltColor, greenAltColor, blueAltColor));  
    } 

    // This sends the updated pixel color to the hardware.  
    pixels.show();  
    // Delay for a period of time (in milliseconds).  
    delay(delayval);
  }  
}  

// picks random values to set for RGB  
void setColor(){  
  redColor = random(0, 255);  
  greenColor = random(0,255);  
  blueColor = random(0, 255);  
}  

void setAltColor(){  
  redAltColor = random(0, 255);  
  greenAltColor = random(0,255);  
  blueAltColor = random(0, 255);  
}  