#include "Adafruit_WS2801.h"
#include <elapsedMillis.h>
#include "SPI.h" // Comment out this line if using Trinket or Gemma
#ifdef __AVR_ATtiny85__
 #include <avr/power.h>
#endif
#include "RGBdriver.h"
#define NUMPIXELS 25
#include <HSBColor.h>
#include <Ultrasonic.h>


/*****************************************************************************
Example sketch for driving Adafruit WS2801 pixels!
Developed October, 2015

  Designed specifically to work with the Adafruit RGB Pixels!
  12mm Bullet shape ----> https://www.adafruit.com/products/322
  12mm Flat shape   ----> https://www.adafruit.com/products/738
  36mm Square shape ----> https://www.adafruit.com/products/683

  These pixels use SPI to transmit the color data, and have built in
  high speed PWM drivers for 24 bit color per pixel
  2 pins are required to interface

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution

*****************************************************************************/

elapsedMillis em;

#define IR_PROXIMITY_SENSOR A2 // Analog input pin that  is attached to the sensor
#define ADC_REF 5//reference voltage of ADC is 5v.If the Vcc switch on the Seeeduino
				 //board switches to 3V3, the ADC_REF should be 3.3

// Choose which 2 pins you will use for output.
// Can be any valid output pins.
// The colors of the wires may be totally different so
// BE SURE TO CHECK YOUR PIXELS TO SEE WHICH WIRES TO USE!
uint8_t dataPin  = 5;    // Yellow wire on Adafruit Pixels
uint8_t clockPin = 4;    // Green wire on Adafruit Pixels

// Don't forget to connect the ground wire to Arduino ground,
// and the +5V wire to a +5V supply

// Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
Adafruit_WS2801 strip = Adafruit_WS2801(50, 3,2);

// Optional: leave off pin numbers to use hardware SPI
// (pinout is then specific to each board and can't be changed)
//Adafruit_WS2801 strip = Adafruit_WS2801(25);

// For 36mm LED pixels: these pixels internally represent color in a
// different format.  Either of the above constructors can accept an
// optional extra parameter: WS2801_RGB is 'conventional' RGB order
// WS2801_GRB is the GRB order required by the 36mm pixels.  Other
// than this parameter, your code does not need to do anything different;
// the library will handle the format change.  Examples:
// Adafruit_WS2801 strip = Adafruit_WS2801(25, dataPin, clockPin, WS2801_GRB);
// Adafruit_WS2801 strip = Adafruit_WS2801(25, WS2801_GRB);

Ultrasonic ultrasonic(5);
int ap=0;
#define THRESHOLD 80

void setup() {
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    
    //Serial.begin(115200); while (!Serial);
    
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
    clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

    strip.begin();
    // Update LED contents, to start they are all 'off'
    for (int i=0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, Color(255, 0, 0));
    }
    strip.show();
    delay(500);
    for (int i=0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, Color(0, 255, 0));
    }
    strip.show();
    delay(500);
    for (int i=0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, Color(0, 0, 255));
    }
    strip.show();
    delay(500);
    for (int i=0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, Color(0, 0, 0));
    }
    strip.show();
    delay(500);
    
    strip.show();
    //Serial.print("Size: values[");Serial.print(ITEMS, DEC);Serial.println("]");
}

float hue = 0;

float mapf(long x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int   numpixels;
float dimmer = 1.0;
float brightness = 1.0;


int getVoltageOld() {
    int values[10];
#define ITEMS (sizeof(values) / sizeof(int))
    int range = analogRead(IR_PROXIMITY_SENSOR);
    values[ap++] = range;
    //Serial.print("values[");Serial.print(ap, DEC);Serial.print("] = ");Serial.print(range, DEC);Serial.print(".  Average=");
    if (ap >= ITEMS) {
      ap=0;
    }
    range=0; 
    for (int x = 0; x < ITEMS; x++) range += values[x];
    range = range / ITEMS;
    return range;
}

int getVoltage() {
	int sensor_value;
	int sum;  
	// read the analog in value:
	for (int i = 0;i < 20;i ++) { //Continuous sampling 20 times
		sensor_value = analogRead(IR_PROXIMITY_SENSOR); 
		sum += sensor_value;
	}
	sensor_value = sum / 20;
	float voltage;
	voltage = (float)sensor_value*ADC_REF/1024;
	return int(voltage * 100);
} 

void loop() {
    int range;
    int aval  = analogRead(A0); // slider
    brightness = mapf(aval,1024,0,1.0,0.0);
    // range = getVoltage();
    range = THRESHOLD;
    // Serial.println(range, DEC);
    if ( range >= THRESHOLD ) {
      if (em > 50) {
          em = 0;
          dimmer += 0.1;
          if (dimmer > 1.0) dimmer = 1.0;
          numpixels = min(numpixels + 1, strip.numPixels());
      }
    } else {
        if (em > 50) {
            em = 0;
            dimmer -= 0.01;
            if (dimmer < 0.1) dimmer = 0.1;
            numpixels = max(numpixels - 1, 10);
        }
    }
    HSBrainbow(brightness * dimmer, numpixels, 150);
}

HSBColor cur_color = HSBColor(1,1,1);
void HSBrainbow(float bright, int num, uint8_t wait){
      hue += 0.001;
      if ( hue >=1 ) hue = 0;
      float sat = 1.0;
      float val = 0.4;
      cur_color.convert_hcl_to_rgb(hue,sat,bright);
      for (int i=0; i < num; i++) {
            strip.setPixelColor(i, Color(cur_color.red, cur_color.green, cur_color.blue));
      }
      for (int i=num; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Color(0, 0, 0));
      }
      strip.show();
      delay(wait);
}


void rainbowWhole(uint8_t wait) {
   for (int j=0; j < 255; j++) {     // 3 cycles of all 256 colors in the wheel
      int c = Wheel(j % 256);

      for (int i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
      }
      strip.show();
      delay(wait);
    }
}

void rainbow(uint8_t wait) {  
    for (int j=0; j < 256; j++) {     // 3 cycles of all 256 colors in the wheel
        for (int i=0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel( (i + j) % 255));
        }  
        strip.show();   // write all the pixels out
        delay(wait);
    }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(uint8_t wait) { 
  for (int j=0; j < 256 * 5; j++) {     // 5 cycles of all 25 colors in the wheel
    for (int i=0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 256 / strip.numPixels()) + j) % 256) );
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint32_t c, uint8_t wait) {
  for (int i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
