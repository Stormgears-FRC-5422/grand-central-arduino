#include <Adafruit_NeoPixel.h>

#define PIN 6
#define PIXELSPERGROUP 6
#define NUMGROUPS 4

Adafruit_NeoPixel strip= Adafruit_NeoPixel(NUMGROUPS * PIXELSPERGROUP, PIN, NEO_GRBW + NEO_KHZ800);

/* **********
 *  0t - turn on precision mode
 *  0f - turn off precision mode
 *  1t - white (error?)
 *  1r - vision mode for rocket
 *  1c - vision mode for cargo ship
 *  1f - turn off vision
 *  2t - vacuum insufficient
 *  2f - turn off vacuum light
 *  3t - intake vacuum armed
 *  3f - intake vacuum unarmed
 */


String str;
char grp;

uint32_t off =    strip.Color(  0,   0,   0,   0);
uint32_t white =  strip.Color(  0,   0,   0, 255);
uint32_t green =  strip.Color(  0, 255,   0,   0);
uint32_t red =    strip.Color(255,   0,   0,   0);
uint32_t blue =   strip.Color(0,     0, 255,   0);
uint32_t orange = strip.Color(255, 165,   0,   0);
uint32_t purple = strip.Color(128,   0, 128,   0);

uint32_t colors[NUMGROUPS] = { green, white, red, purple}; 

void setup() {
  Serial.begin(115200);
  strip.begin();
  stripShow(0, white);
  stripShow(1, white);
  stripShow(2, white);
  stripShow(3, white);
}

void loop()
{
  if (Serial.available() > 0)
  {
    str = Serial.readStringUntil('\n');
    grp = str[0] - '0';  // Turn the character into an offset from the number 0
    switch (str[1]) {
      case 't':  // most modes are just on / off
        stripShow(grp, colors[grp]);
        break;
      case 'c':  // vision mode for cargo ship
        stripShow(grp, blue);
        break;
      case 'r':  // vision mode for rocket
        stripShow(grp, orange);
        break;
      case 'f':
      default:
        stripShow(grp, off);
    }
    if (str[1] == 't') {
      stripShow(grp, colors[grp]);
    } else {
    }
  }
}

void stripShow(char group, uint32_t color) {
  if (group < NUMGROUPS) {
    for (int i = 0; i < PIXELSPERGROUP; i++) {
      strip.setPixelColor(group * PIXELSPERGROUP + i, color);
      strip.show();
    }      
  } else {
    // This makes no sense. Turn the whole thing white
    for (int i = 0; i < NUMGROUPS; i++) {
      stripShow(i, white);   
    }
  }  
}
