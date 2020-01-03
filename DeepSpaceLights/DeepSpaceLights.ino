#include <Adafruit_NeoPixel.h>

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

#define NEOPIXELPIN 12

#define PRECISIONMODEPIN 2
#define VISIONROCKETPIN 4
#define VISIONCARGOPIN 5
#define VACARMEDPIN 6
#define VACINSUFFICIENTPIN 8

#define PIXELSPERGROUP 6
#define NUMGROUPS 4

Adafruit_NeoPixel strip= Adafruit_NeoPixel(NUMGROUPS * PIXELSPERGROUP, NEOPIXELPIN, NEO_GRBW + NEO_KHZ800);

uint32_t off =    strip.Color(  0,   0,   0,   0);
uint32_t white =  strip.Color(  0,   0,   0, 127);
uint32_t red =    strip.Color(255,   0,   0,   0);
uint32_t green =  strip.Color(  0, 255,   0,   0);
uint32_t blue =   strip.Color(0,     0, 255,   0);
uint32_t yellow = strip.Color(255, 255,   0,   0);
uint32_t magenta= strip.Color(255,   0, 255,   0);
uint32_t cyan   = strip.Color(  0, 255, 255,   0);
uint32_t orange = strip.Color(255, 165,   0,   0);
uint32_t purple = strip.Color(128,   0, 128,   0);


uint32_t colors[NUMGROUPS] = { green, white, red, purple}; 
String str;
char grp;

boolean precisionModeLight = false;
boolean visionRocketLight = false;
boolean visionCargoLight = false;
boolean vacArmedLight = false;
boolean vacInsufficientLight = false;

void setup() {
  pinMode(PRECISIONMODEPIN, INPUT_PULLUP);
  pinMode(VISIONROCKETPIN, INPUT_PULLUP);
  pinMode(VISIONCARGOPIN, INPUT_PULLUP);
  pinMode(VACARMEDPIN, INPUT_PULLUP);
  pinMode(VACINSUFFICIENTPIN, INPUT_PULLUP);
  
  strip.begin();
  stripShow(0, white);
  stripShow(1, white);
  stripShow(2, white);
  stripShow(3, white);

  delay(5000);
}

void loop()
{
  boolean precisionModeNow   = digitalRead(PRECISIONMODEPIN);
  boolean visionRocketNow    = digitalRead(VISIONROCKETPIN);
  boolean visionCargoNow     = digitalRead(VISIONCARGOPIN);
  boolean vacArmedNow        = digitalRead(VACARMEDPIN);
  boolean vacInsufficientNow = digitalRead(VACINSUFFICIENTPIN);

  stripShow(0, precisionModeNow ? green : off);
  stripShow(2, vacArmedNow ? red : off);
  stripShow(3, vacInsufficientNow ? purple : off);
  
  if (!visionRocketNow && ! visionCargoNow) {
    stripShow(1, off);
  } else {
    stripShow(1, visionRocketNow ? yellow : blue);
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

/* **********
//  if (precisionModeNow != precisionModeLight) {
//    stripShow(0, precisionModeNow ? green : off);
//    precisionModeLight = precisionModeNow;
//  }

//  if (vacArmedNow != vacArmedLight) {
//    stripShow(2, vacArmedNow ? red : off);
//    vacArmedLight = vacArmedNow;
//  }

//  if (vacInsufficientNow != vacInsufficientLight) {
//    stripShow(3, vacInsufficientNow ? purple : off);
//    vacInsufficientLight = vacInsufficientNow;
//  }
  
  // since these two settings control the same light they need to be treated together
//  if (visionRocketNow != visionRocketLight || visionCargoNow != visionCargoLight) {
//    if (!visionRocketNow && !visionCargoNow) {
//      stripShow(1, off);
//    } else if (visionRocketNow && visionCargoNow) {
//      stripShow(1, yellow);   // shouldn't really happen, but technically possible - must be transitioning and hit a rare timing situation
//      return;                 // don't save state - pick it up on the next loop iteration
//    } else {
//      stripShow(1, visionRocketNow ? orange : blue);
//    }
    
//      stripShow(1, visionRocketNow ? orange : blue);

//    visionRocketLight = visionRocketNow;
//    visionCargoLight = visionCargoNow;
//  }

//  if (Serial.available() > 0)
//  {
//    str = Serial.readStringUntil('\n');
//    grp = str[0] - '0';  // Turn the character into an offset from the number 0
//    switch (str[1]) {
//      case 't':  // most modes are just on / off
//        stripShow(grp, colors[grp]);
//        break;
//      case 'c':  // vision mode for cargo ship
//        stripShow(grp, blue);
//        break;
//      case 'r':  // vision mode for rocket
//        stripShow(grp, orange);
//        break;
//      case 'f':
//      default:
//        stripShow(grp, off);
//    }
//    if (str[1] == 't') {
//      stripShow(grp, colors[grp]);
//    } else {
//    }
//  }
//}

********** */
