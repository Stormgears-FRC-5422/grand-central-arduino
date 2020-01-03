#include <Adafruit_NeoPixel.h>
#include "StormNetCommon.h"

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

uint32_t off =    strip.Color(  0,   0,   0,   0);
uint32_t white =  strip.Color(  0,   0,   0, 255);
uint32_t green =  strip.Color(  0, 255,   0,   0);
uint32_t red =    strip.Color(255,   0,   0,   0);
uint32_t blue =   strip.Color(0,     0, 255,   0);
uint32_t orange = strip.Color(255, 165,   0,   0);
uint32_t purple = strip.Color(128,   0, 128,   0);

uint32_t colors[NUMGROUPS] = { green, white, red, purple}; 

// TODO choose a unique address
const char I2C_ADDRESS = 8;    // each device needs its own 7 bit address

// Command modes
// const char MODE_X = 6;        // your mode here
// TODO: add more modes
const char MODE_SET_STATUS_LIGHTS = 7;

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;

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

  showStatusLights(0, white);
  showStatusLights(1, white);
  showStatusLights(2, white);
  showStatusLights(3, white);

  Serial.begin(115200);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");
}


void loop() { //main user command loop
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  if (Serial.available()) {
    g_talkMode = serialMode;
  }
  else {
    g_talkMode = I2CMode;
  }

  if (g_talkMode == serialMode) {
    Wire.onRequest(NULL);     // It is problematic to get wire interrupts during serial mode
    Wire.onReceive(NULL);
  }
  else {
    Wire.onRequest(requestEvent);     // register event
    Wire.onReceive(receiveEvent);     // register event
  }

  //========== flash heartbeat (etc) LED =============
  currentMillis = millis();

  // the interrupts could change the value of g_blinkInterval which can mess with this logic
  noInterrupts();
    // Blink superfast if we haven't heard from the master in a while
    if ( (currentMillis - previousI2C) > i2cHeartbeatTimeout)  // stale
      g_blinkInterval = 100;
    else
      g_blinkInterval = g_blinkInterval==100 ? 1000 : g_blinkInterval;

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
  if (g_talkMode == serialMode && Serial.available() > 0) { //diagnostic menu system starts here
    receiveEvent(5); // call the interrupt handler directly.  We may or may not read this many bytes
    requestEvent();
  }

}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
// this function can also be called at other times (say via events coming through Serial)
void requestEvent() {
  previousI2C = currentMillis;      // reset the comm timeout clock
  switch(g_commandMode) {
// TODO - new modes
//    case MODE_X:
//      handleXRequest();
//      break;
    case MODE_SET_STATUS_LIGHTS:
      handleSetStatusLightsRequest();
      break;
    case MODE_HELP:
      handleHelpRequest();
      break;
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
// TODO - new modes
//    case 'X':
//      g_commandMode = MODE_X;
//      break;
    case 'D':
      g_commandMode = MODE_SET_STATUS_LIGHTS;
      handleSetStatusLightsReceive();
      break;
    case '?':
      g_commandMode = MODE_HELP;
      break;
    default:
      receiveBuiltIn(c);
  }

  while (readAvailable()) c=readByte(); // in case there is other stuff sent that needs to be collected - don't expect this
}

//================================
void handleHelpRequest() {
  if (g_talkMode == serialMode) {
    printBuiltInHelp();
    // TODO - add menu items
    Serial.println("    D:  Set Status lights [0t / 0f / 1t / 1c / 1r / 2t / 2f / 3t / 3f");
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}


void showStatusLights(char group, uint32_t color) {
  if (group < NUMGROUPS) {
    for (int i = 0; i < PIXELSPERGROUP; i++) {
      strip.setPixelColor(group * PIXELSPERGROUP + i, color);
      strip.show();
    }      
  } else {
    // This makes no sense. Turn the whole thing white
    for (int i = 0; i < NUMGROUPS; i++) {
      showStatusLights(i, white);   
    }
  }  
}

void handleSetStatusLightsReceive() {
  byte str[3]; // 1t\n
  byte grp;
  readBytes(str, 3, byteType);

  grp = str[0] - '0';  // Turn the character into an offset from the number 0
  switch (str[1]) {
    case 't':  // most modes are just on / off
      showStatusLights(grp, colors[grp]);
      break;
    case 'c':  // vision mode for cargo ship
      showStatusLights(grp, blue);
      break;
    case 'r':  // vision mode for rocket
      showStatusLights(grp, orange);
      break;
    case 'f':
    default:
      showStatusLights(grp, off);
  }
}

void handleSetStatusLightsRequest() {
  handleDefaultRequest();
}
