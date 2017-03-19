#include <Adafruit_NeoPixel.h>

#include "StormNetCommon.h"
#include <SoftwareSerial.h>


// TODO choose a unique address
const char I2C_ADDRESS = 8;    // each device needs its own 7 bit address

// Command modes
// const char MODE_X = 6;        // your mode here
// TODO: add more modes
const char MODE_US = 6;
const char MODE_GEAR_RING_OFF = 7;
const char MODE_GEAR_RING_ON = 8;
const char MODE_SHOOTER_RING_OFF = 9;
const char MODE_SHOOTER_RING_ON = 10;

#define NUMSENSORS 4
int usEN[NUMSENSORS] = {11,9,7,4};
int usRX[NUMSENSORS] = {12,10,8,5};
// volatile?
byte ranges[NUMSENSORS] = { 0, 0, 0, 0};

SoftwareSerial US[NUMSENSORS] = {SoftwareSerial(12, 13, true),
                                 SoftwareSerial(10, 13, true),
                                 SoftwareSerial(8, 13, true),
                                 SoftwareSerial(5, 13, true),
                                 };

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;

const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated


// neopixel support
#define NUMLIGHTS 80
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMLIGHTS, 2, NEO_RGBW); //first number is total count, ,second number is pin#

//colors
uint32_t off = strip.Color (0, 0, 0, 0);
uint32_t white = strip.Color(0, 0, 0, 255);
uint32_t green = strip.Color(255, 0, 0, 0);
uint32_t red = strip.Color(0, 255, 0, 0);
uint32_t teal = strip.Color(120, 1, 67, 2);
uint32_t blue = strip.Color(0, 0, 255, 0);

int lightStrings[2][2] = {{0, 39}, {40, NUMLIGHTS}};

char next_Gear_Ring_State=1;
char next_Shooter_Ring_State=1;
char Gear_Ring_State=0;
char Shooter_Ring_State=0;

void setup() {
  g_i2cAddress = I2C_ADDRESS;
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output
  Wire.begin(I2C_ADDRESS);          // join i2c bus
  Wire.onRequest(requestEvent);     // register event
  Wire.onReceive(receiveEvent);     // register event

  Serial.begin(9600);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");

  for(int i = 0; i < NUMSENSORS; i++) {
     pinMode(usEN[i], OUTPUT);
     pinMode(usRX[i], INPUT);
     digitalWrite(usEN[i], LOW);
     US[i].begin(9600);
  }
   strip.begin();
   setRingLights();
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
  if (serialMode && Serial.available() > 0) { //diagnostic menu system starts here
    receiveEvent(5); // call the interrupt handler directly.  We may or may not read this many bytes
    requestEvent();
  }

  usLoop();
  // Only bother setting the lights if the state has changed.
  if ( (next_Gear_Ring_State != Gear_Ring_State) || (next_Shooter_Ring_State != Shooter_Ring_State) ) {
    setRingLights();
  }
}

void usLoop() {
  for(int i = 0; i < NUMSENSORS; i++) {
    ranges[i] = stormGetRange(US[i], usEN[i]);
  }
}

int stormGetRange(SoftwareSerial US, int usEN) {
  char buffer[5];
  unsigned long startTime = 0;
  unsigned long currentTime = 0;

  digitalWrite(usEN, HIGH);
  US.listen();
  US.flush();
  //while (!US1.available() || US1.read() != 'R');
  startTime = millis();
  do {
  while ((!US.available() && (currentTime = (millis() - startTime)) < 49)); //wait for character or time out
  if (currentTime < 49) {
    buffer[0] = US.read();
  } else {
    digitalWrite(usEN, LOW);
    return 0;
  }
  }  while (buffer[0]!='R');  //wait until you get the first character of the next reading

  startTime = millis();
  while (!US.available() && (currentTime = (millis() - startTime)) < 49); //wait for character or time out
  if (currentTime < 49) {
    buffer[1] = US.read();
  } else {
    digitalWrite(usEN, LOW);
    return 0;
  }
  startTime = millis();
  while (!US.available() && (currentTime = (millis() - startTime)) < 49); //wait for character or time out
  if (currentTime < 49) {
    buffer[2] = US.read();
  } else {
    digitalWrite(usEN, LOW);
    return 0;
  }
  startTime = millis();
  while (!US.available() && (currentTime = (millis() - startTime)) < 49); //wait for character or time out
  if (currentTime < 49) {
    buffer[3] = US.read();
  } else {
    digitalWrite(usEN, LOW);
    return 0;
  }
  //Serial.println(atoi(&buffer[1]));
  digitalWrite(usEN, LOW);
  return atoi(&buffer[1]);
}

void setRingLights() {
   // Update the "current" status to reflect the new normal
   Gear_Ring_State = next_Gear_Ring_State;
   Shooter_Ring_State = next_Shooter_Ring_State;

   if (Gear_Ring_State==0) for (int i=0; i<=39; i++) strip.setPixelColor(i,off);
   if (Gear_Ring_State==1) for (int i=0; i<=39; i++) strip.setPixelColor(i,green);
   if (Shooter_Ring_State==0) for (int i=40; i<=NUMLIGHTS; i++) strip.setPixelColor(i,off);
   if (Shooter_Ring_State==1) for (int i=40; i<=NUMLIGHTS; i++) strip.setPixelColor(i,green);

   strip.show();
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
    case MODE_US:
      handleUSRequest();
      break;
    case MODE_GEAR_RING_ON:
    case MODE_GEAR_RING_OFF:
    case MODE_SHOOTER_RING_ON:
    case MODE_SHOOTER_RING_OFF:
      handleRingLightRequest();
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
    case 'U':
      g_commandMode = MODE_US;
      break;
    case '1':
      g_commandMode = MODE_GEAR_RING_ON;
      next_Gear_Ring_State = 1;
      break;
    case '2':
      g_commandMode = MODE_GEAR_RING_OFF;
      next_Gear_Ring_State = 0;
      break;
    case '3':
      g_commandMode = MODE_SHOOTER_RING_ON;
      next_Shooter_Ring_State = 1;
      break;
    case '4':
      g_commandMode = MODE_SHOOTER_RING_OFF;
      next_Shooter_Ring_State = 0;
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
  if (serialMode) {
    printBuiltInHelp();
    // TODO - add menu items
    Serial.println("    U:  Print ultrasonic values");
    Serial.println("    1:  Gear ring light ON");
    Serial.println("    2:  Gear ring light OFF");
    Serial.println("    3:  Shooter ring light ON");
    Serial.println("    4:  Shooter ring light OFF");
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

void handleRingLightRequest() {
  // Nothing much to say - just return the counter
  handleDefaultRequest();
}

void handleUSRequest() {
  if (serialMode)
    Serial.println("we are in US");

  writeBytes(ranges, NUMSENSORS, byteType, serialMode);
}
// TODO - write handlers
