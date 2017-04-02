#include "StormNetCommon.h"
#include <Adafruit_NeoPixel.h>


// TODO choose a unique address
const char I2C_ADDRESS = 13;    // each device needs its own 7 bit address

// Command modes
const char MODE_LIGHT = 6;        // your mode here
// TODO: add more modes

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;

#define NUMLIGHTS 39
#define NUMLIGHTSTRINGS 3
int lightStrings[NUMLIGHTSTRINGS][2] = {{0, 4}, {5, 21}, {22, 38}};
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMLIGHTS, 6, NEO_RGBW); //first number is total count, ,second number is pin# - probably not right

//colors
uint32_t off = strip.Color (0, 0, 0, 0);
uint32_t white = strip.Color(0, 0, 0, 255);
uint32_t yellow = strip.Color(128, 255, 0, 0);
uint32_t red = strip.Color(0, 255, 0, 0);
uint32_t teal = strip.Color(120, 1, 67, 2);
uint32_t blue = strip.Color(0, 0, 255, 0);
uint32_t brown = strip.Color(32, 255, 0, 0);

//light arrays
int modes[NUMLIGHTSTRINGS][3] = {{6, 5, 5}, {6, 5, 5}, {6, 5, 5}};

//things modes need to remember   ==================
//These arrays needs to be AT LEAST as long as NUMLIGHTSTRINGS. They can be longer
//seizure
long lastSeizureTime[5] = {0, 0, 0, 0, 0};
int seizureStatus[5] = {0, 0, 0, 0, 0};
//warp
long warpValues[5][2] = {(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)};
//underglow
long underglowValues[5][3] = {(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)};
//fire
long fireValues[5][2] = {(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)};
//rainbowCycle
long rainbowValues[5][2] = {(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)};    //row 1 = last time, row 2 = counter

//==================================================

const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

void setup() {
  g_i2cAddress = I2C_ADDRESS;
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output
  Wire.begin(I2C_ADDRESS);          // join i2c bus
  Wire.onRequest(requestEvent);     // register event
  Wire.onReceive(receiveEvent);     // register event

  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(9600);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");

  strip.begin();

}

void loop() {

  //main user command loop
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  //  serialMode = Serial.available();

  //========== flash heartbeat (etc) LED =============
  currentMillis = millis();
  // the interrupts could change the value of g_blinkInterval which can mess with this logic
  noInterrupts();
  // Blink superfast if we haven't heard from the master in a while
  if ( (currentMillis - previousI2C) > i2cHeartbeatTimeout)  // stale
    g_blinkInterval = 100;
  else
    g_blinkInterval = g_blinkInterval == 100 ? 1000 : g_blinkInterval;

  boolean flipNow = ( (currentMillis - previousBlink) >= g_blinkInterval);
  interrupts();

  if (flipNow) {
    previousBlink = currentMillis;    // save the last time you blinked the LED
    if (ledState == LOW) {            // if the LED is off turn it on and vice-versa
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);   // set the LED with the ledState of the variable
  }

   // Check for serial input.  Note that i2c input happens through interrupts, not here.
    if (serialMode && Serial.available() > 0) { //diagnostic menu system starts here
     receiveEvent(5); // call the interrupt handler directly.  We may or may not read this many bytes
     requestEvent();
    }
  

  strip.setBrightness(96);    //this brightness change needs to be in the final code - if the lights "flake out" change this number

  lightLoop();
}

void lightLoop() {
  for (int i = 0; i < NUMLIGHTSTRINGS; i++) {
    switch (modes[i][0]) {
      case 0:
        // ringMode(i, 0);
        break;
      case 1:
        ringMode(i, modes[i][1]);
        break;
      case 2:
        seizure(i, modes[i][1], modes[i][2]);
        break;
      case 3:
        warp(i, modes[i][1]);
        break;
      case 4:
        underglow(i, modes[i][1]);
        break;
      case 5:
        fire(i);
        break;
      case 6:
        rainbowCycle(i, modes[i][1]);
        break;
    }
  }
}

void ringMode(int iD, int color) {
  for (int i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
    switch (color) {
      case 0:
        strip.setPixelColor(i, off);
        break;
      case 1:
        strip.setPixelColor(i, white);
        break;
      case 2:
        strip.setPixelColor(i, yellow);
        break;
      case 3:
        strip.setPixelColor(i, red);
        break;
      case 4:
        strip.setPixelColor(i, teal);
        break;
      case 5:
        strip.setPixelColor(i, blue);
        break;
      case 6:
        strip.setPixelColor(i, brown);
        break;
      default:
        strip.setPixelColor(i, off);
    }
  }
  strip.show();
}

void seizure(int iD, int color, int delayVal) {
  if (millis() - lastSeizureTime[iD] >= delayVal) {
    if (seizureStatus[iD] == 0) {
      for (int i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
        switch (color) {
          case 0:
            strip.setPixelColor(i, off);
            break;
          case 1:
            strip.setPixelColor(i, white);
            break;
          case 2:
            strip.setPixelColor(i, yellow);
            break;
          case 3:
            strip.setPixelColor(i, red);
            break;
          case 4:
            strip.setPixelColor(i, teal);
            break;
          case 5:
            strip.setPixelColor(i, blue);
            break;
          case 6:
            strip.setPixelColor(i, brown);
            break;
          default:
            strip.setPixelColor(i, off);
        }
      }
      seizureStatus[iD] = 1; //last status was colorful
      lastSeizureTime[iD] = millis();
    } else {
      for (int i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
        strip.setPixelColor(i, off);
      }
      seizureStatus[iD] = 0;  //last status was off
      lastSeizureTime[iD] = millis();
    }
  }
  strip.show();
}

void warp(int iD, int color) {
  int red, green, blue, white;
  switch (color) {
    case 1:   //white
      red = 0;
      blue = 0;
      green = 0;
      white = 255;
      break;
    case 2:   //yellow
      red = 255;
      blue = 0;
      green = 128;
      white = 0;
      break;
    case 3:   //red
      red = 255;
      blue = 0;
      green = 0;
      white = 0;
      break;
    case 4:   //teal
      red = 1;
      blue = 67;
      green = 120;
      white = 2;
      break;
    case 5:   //blue
      red = 0;
      blue = 255;
      green = 0;
      white = 0;
      break;
    case 6: //brown
      red = 255;
      blue = 0;
      green = 32;
      white = 0;
      break;
  }



  if ((millis() - warpValues[iD][0]) >= 5) {
    if (warpValues[iD][1] >= (lightStrings[iD][1] - lightStrings[iD][0]) * 2) {
      // warpValues[iD][1] = 0;      //comment this out if it fixes the little 'blip' thing
    }
    warpValues[iD][1]++;
    for (int i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
      strip.setPixelColor(i, ((sin(i + warpValues[iD][1]) * 127 + 128) / 255) * green,
                          ((sin(i + warpValues[iD][1]) * 127 + 128) / 255) * red,
                          ((sin(i + warpValues[iD][1]) * 127 + 128) / 255) * blue,
                          ((sin(i + warpValues[iD][1]) * 127 + 128) / 255) * white);
    }
    strip.show();
    warpValues[iD][0] = millis();
  }
}

void underglow(int iD, int color) {
  float r, g, b, w;
  int red, blue, green, white;
  switch (color) {
    case 1:   //white
      red = 0;
      blue = 0;
      green = 0;
      white = 255;
      break;
    case 2:   //yellow
      red = 255;
      blue = 0;
      green = 128;
      white = 0;
      break;
    case 3:   //red
      red = 255;
      blue = 0;
      green = 0;
      white = 0;
      break;
    case 4:   //teal
      red = 1;
      blue = 67;
      green = 120;
      white = 2;
      break;
    case 5:   //blue
      red = 0;
      blue = 255;
      green = 0;
      white = 0;
      break;
    case 6: //brown
      red = 255;
      blue = 0;
      green = 32;
      white = 0;
      break;
  }
  if ((millis() - underglowValues[iD][2]) >= 5) {
    if (underglowValues[iD][1] == 0) {
      r = (underglowValues[iD][0] / 256.0) * red;
      g = (underglowValues[iD][0] / 256.0) * green;
      b = (underglowValues[iD][0] / 256.0) * blue;
      w = (underglowValues[iD][0] / 256.0) * white;
      for (int q = lightStrings[iD][0]; q <= lightStrings[iD][1]; q++) {
        strip.setPixelColor(q, g, r, b, w);
      }
      underglowValues[iD][0] += 8;
      if (underglowValues[iD][0] >= 256) {
        underglowValues[iD][0] = 255;
        underglowValues[iD][1] = 1;
      }
      strip.show();
    }

    if (underglowValues[iD][1] == 1) {
      r = (underglowValues[iD][0] / 256.0) * red;
      g = (underglowValues[iD][0] / 256.0) * green;
      b = (underglowValues[iD][0] / 256.0) * blue;
      w = (underglowValues[iD][0] / 256.0) * white;
      for (int q = lightStrings[iD][0]; q <= lightStrings[iD][1]; q++) {
        strip.setPixelColor(q, g, r, b, w);
      }
      underglowValues[iD][0] = underglowValues[iD][0] - 8;
      if (underglowValues[iD][0] <= 0) {
        underglowValues[iD][0] = 0;
        underglowValues[iD][1] = 0;
      }
      strip.show();
    }
    underglowValues[iD][2] = millis();
  }
}

void fire(int iD) {
  int r = 255;
  int g = r - 110;
  int b = 40;
  if ((millis() - fireValues[iD][0]) >= fireValues[iD][1]) {
    for (int i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
      int flicker = random(30, 150);
      int r1 = r - flicker;
      int g1 = g - flicker;
      int b1 = b - flicker;
      if (g1 < 0) g1 = 0;
      if (r1 < 0) r1 = 0;
      if (b1 < 0) b1 = 0;
      strip.setPixelColor(i, g1, r1, b1, 0);
    }
    fireValues[iD][0] = millis();
    fireValues[iD][1] = random(50, 150);
    strip.show();

  }
}

void rainbowCycle(int iD, int speedDelay) {
  byte *c;
  speedDelay = 0;
  uint16_t i;
  if ((millis() - rainbowValues[iD][0]) >= speedDelay) {
    for (i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
      c = Wheel((((i - lightStrings[iD][0]) * 256 / (lightStrings[iD][1] - lightStrings[iD][0])) + rainbowValues[iD][1]) & 255);
      strip.setPixelColor(i, *c, *(c + 1), *(c + 2), 0);
    }
    if (rainbowValues[iD][1] >= 256 * 5) {
      rainbowValues[iD][1] = 0;
    }
    rainbowValues[iD][1]++;
    strip.show();
    rainbowValues[iD][0] = millis();
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];

  if (WheelPos < 85) {
    c[0] = WheelPos * 3;
    c[1] = 255 - WheelPos * 3;
    c[2] = 0;
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    c[0] = 255 - WheelPos * 3;
    c[1] = 0;
    c[2] = WheelPos * 3;
  } else {
    WheelPos -= 170;
    c[0] = 0;
    c[1] = WheelPos * 3;
    c[2] = 255 - WheelPos * 3;
  }

  return c;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
// this function can also be called at other times (say via events coming through Serial)
void requestEvent() {
  previousI2C = currentMillis;      // reset the comm timeout clock
  switch (g_commandMode) {
    // case MODE_X:
    // handleXRequest();
    // break;
    case MODE_HELP:
      handleHelpRequest();
      break;
    case MODE_LIGHT:
    default:
      requestBuiltIn();
  }
}

//================================
void receiveEvent(int howMany) { // handles i2c write event from master
  char c;
  // reset the comm timeout clock
  previousI2C = currentMillis;

  if (howMany > 0) c = readByte();
  switch (c) {
    case 'L':
      g_commandMode = MODE_LIGHT;
      handleLightReceive();
      break;
    case '?':
      g_commandMode = MODE_HELP;
      break;
    default:
      receiveBuiltIn(c);
  }

  while (readAvailable()) c = readByte(); // in case there is other stuff sent that needs to be collected - don't expect this
}

//================================
void handleHelpRequest() {
  if (serialMode) {
    Serial.println();
    Serial.println("===== Stormgears I2C Device Help =====");
    Serial.println("    P:  Ping - read I2C Address");
    Serial.println("    F:  Change LED to FAST flash. Read 'FAST'");
    Serial.println("    S:  Change LED to SLOW flash. Read 'SLOW'");
    Serial.println("    B:  Change LED flash rate directly - pass another long to say how fast (in milliseconds)");
    // TODO - add menu items
    Serial.println("   \\0:  (or anything unhandled) Read unsingned int counter");
    Serial.println("    ?:  Show this help (otherwise act like \\0)");
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

//================================
void handleLightReceive() {
  //put rest of values into mode array
  byte id;
  byte mode;
  byte arg1;
  byte arg2;

  readBytes( &id , 1);
  readBytes( &mode , 1);
  readBytes( &arg1, 1);
  readBytes( &arg2, 1);

  // for testing
  // id = 1;
  // mode = 1;
  // arg1 = random(0,3);
  // arg2 = 0;

  modes[id][0] = mode;
  modes[id][1] = arg1;
  modes[id][2] = arg2;


}
// TODO - write handlers - see StormNetCommon.h for examples
