#include "StormNetCommon.h"

// TODO choose a unique address
const char I2C_ADDRESS = 11;    // each device needs its own 7 bit address

// Command modes
// const char MODE_X = 6;        // your mode here
// TODO: add more modes
const char MODE_COLOR = 6;

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;

const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

#define NUM_COLORS 3 // Red, Blue, Gray (Carpet) and White
byte lineSensorColors[NUM_COLORS] = {0,0,0}; // Order of this is R/B, W, G (Carpet)

short analogValues[2] ={0,0};
int analogPin_zero = A0;
int analogPin_one = A1;

void setup() {
  g_i2cAddress = I2C_ADDRESS;
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output
  Wire.begin(I2C_ADDRESS);          // join i2c bus
  Wire.onRequest(requestEvent);     // register event
  Wire.onReceive(receiveEvent);     // register event

  pinMode(analogPin_zero, INPUT);
  pinMode(analogPin_one, INPUT);

  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(9600);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");
}

void loop() { //main user command loop
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  if (Serial.available()) {
    g_talkMode = serialMode;   
    Wire.onRequest(NULL);     // It is problematic to get wire interrupts during serial mode
    Wire.onReceive(NULL);
  }
  else {
    g_talkMode = I2CMode;
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
  IRLoop();
}

void IRLoop() {
 noInterrupts();
   analogValues[0] = (short)analogRead(analogPin_zero);
   analogValues[1] = (short)analogRead(analogPin_one);
 interrupts();
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
     case MODE_COLOR:
      handleGetColorRequest();
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
    case 'C':
      g_commandMode = MODE_COLOR;
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
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

// TODO - write handlers - see StormNetCommon.h for examples
void handleGetColorRequest() { 
  if(serialMode) { 
    Serial.println("In serial mode!!");
  }
   Serial.print("SENSOR VALUE");
   Serial.println(analogValues[0]);
   Serial.println(analogValues[1]);
   
   writeShorts(analogValues, 2, g_talkMode); 
}

