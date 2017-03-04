
#include "StormNetCommon.h"
#include <Adafruit_NeoPixel.h>


// TODO choose a unique address
const char I2C_ADDRESS = 5;    // each device needs its own 7 bit address

// Command modes
const char MODE_LIGHT = 6;        // your mode here
// TODO: add more modes

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;

const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

const int TOTAL_LIGHTS = 45;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(TOTAL_LIGHTS, 6,  NEO_GRBW + NEO_KHZ800); //second number is pin#

//colors
uint32_t off   = strip.Color(0, 0, 0, 0);
uint32_t white = strip.Color(0, 0, 0, 255);
uint32_t green = strip.Color(0, 255, 0, 0);
uint32_t red   = strip.Color(255, 0, 0, 0);
uint32_t blue  = strip.Color(0, 0, 255, 0);


//light arrays
int modes[2][3] = {{1, 1, 255}, {1, 2, 255}}; // {mode, param1, param2}
// dividing up the light strip into segments
int lightStrings[2][2] = {{0, 4}, {5, 44}}; // onboard, gear ring light, shooter ring light

// For onboard light
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 6, NEO_RGBW); //second number is pin# - probably not right
//int modes[2][3] = {{1, 2, 0}, {0, 0, 0}};
//int lightStrings[2][2] = {{0, 0}, {0, 0}};


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

void loop() { //main user command loop
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  serialMode = Serial.available();
  if (serialMode) {
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

  lightLoop();
}

void lightLoop() {
  for (int i = 0; i < 2; i++) {
    switch (modes[i][0]) {
      case 0:
       // ringMode(i, 0);
        break;
      case 1:
        ringMode(i, modes[i][1], modes[i][2]);
        break;
    }
  }
}

void ringMode(int iD, int color, byte brightness) {
  for (int i = lightStrings[iD][0]; i <= lightStrings[iD][1]; i++) {
    switch (color) {
      case 0:
        strip.setPixelColor(i, off);
        break;
      case 1:
        strip.setPixelColor(i, white);
        break;
      case 2:
        strip.setPixelColor(i, green);
        break;
      case 3:
        strip.setPixelColor(i, red);
        break;
      default:
        strip.setPixelColor(i, off);
    }
  }

  strip.setBrightness(brightness);
  strip.show();
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
    printBuiltInHelp();
    // TODO - add menu items
    Serial.println("    L:  Light control [id, mode, arg1, arg2]");
    
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

//================================
void handleLightReceive() {
  byte buffer[4];  
  byte id;

  //[id, mode, arg1, arg2]
  readBytes( buffer, 4, byteType);

  id = buffer[0]; 
  //put rest of values into mode array [mode, arg1, arg2]
  modes[id][0] = buffer[1];
  modes[id][1] = buffer[2];
  modes[id][2] = buffer[3];
}
// TODO - write handlers - see StormNetCommon.h for examples
