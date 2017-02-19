
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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(30, 6, NEO_RGBW); //second number is pin#

//colors
uint32_t off   = strip.Color(0, 0, 0, 0);
uint32_t white = strip.Color(0, 0, 0, 255);
uint32_t green = strip.Color(255, 0, 0, 0);
uint32_t red   = strip.Color(0, 255, 0, 0);


//light arrays
int modes[2][3] = {{1, 1, 0}, {1, 1, 0}}; // {mode, param1, param2}
int lightStrings[2][2] = {{0, 4}, {5, 29}}; // dividing up the light strip into segments

// For onboard light
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 6, NEO_RGBW); //second number is pin# - probably not right
//int modes[2][3] = {{1, 2, 0}, {0, 0, 0}};
//int lightStrings[2][2] = {{0, 0}, {0, 0}};


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

void loop() { //main user command loop
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  serialMode = Serial.available();

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
        ringMode(i, modes[i][1]);
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
        strip.setPixelColor(i, green);
        break;
      default:
        strip.setPixelColor(i, off);
    }
  }
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
