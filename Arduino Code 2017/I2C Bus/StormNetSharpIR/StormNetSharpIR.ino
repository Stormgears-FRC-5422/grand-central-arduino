#include "StormNetCommon.h"

// TODO choose a unique address
const char I2C_ADDRESS = 11;    // each device needs its own 7 bit address

// Command modes
// const char MODE_X = 6;        // your mode here
const char MODE_GetIR = 6;        // your mode here
const char MODE_GetGearState = 7;
const char MODE_Diagnostic = 8;   // report the beam and proximity status
// TODO: add more modes

//IR constants
#define NUMSENSORS 6
#define IR1_ANL 0  //pin assignment
#define IR2_ANL 1  //pin assignment
#define IR3_ANL 2  //pin assignment
#define IR4_ANL 3  //pin assignment
#define IR5_ANL 6  //pin assignment
#define IR6_ANL 7  //pin assignment
#define GP2Y0A51SK0F 1 // 2-15 centimeter range
#define GP2Y0A21YK0F 2 // 10-80 centimeter range
#define GP2Y0A60SZLF 3 // 20-150 centimeter range
// volatile?
short int IR_distances[] = { 0, 0, 0, 0, 0, 0}; // in centimeters
short int IR_config[NUMSENSORS][2] = { //define what Sharp IR sensors are plugged in where
   {GP2Y0A51SK0F, IR1_ANL},
   {GP2Y0A51SK0F, IR2_ANL},
   {GP2Y0A21YK0F, IR3_ANL},
   {GP2Y0A21YK0F, IR4_ANL},
   {GP2Y0A60SZLF, IR5_ANL},
   {GP2Y0A60SZLF, IR6_ANL}
};

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;

const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

//TODO: have to add the gear dropping and gear exiting states afterwards 
enum gear_states {
  EMPTY_BIN = 0, // Bin is empty 
  GEAR_LIFTING, // Gear is leaving, no longer present but has not broken through the beam sensor 
  FULL_BIN, // Gear is in the bin and is sensed by the line sensor 
  GEAR_EXITING, // Gear is breaking beam on the way out 
  UNKNOWN_STATE // Undefined states (Need more sensor data)  
};

#define NUM_LINE_PINS 5 // Number of pins for the line sensor 
byte lineSensorPins[NUM_LINE_PINS] = {12,11,10,9,8};
byte lineSensorVal[NUM_LINE_PINS] = {0,0,0,0,0};
int analogVal = 0;
int analogPin = 0;

// Booleans to determine the gear states 
boolean beamBroken = false;
boolean gearPresent = false;

// Holds our next state 
short currentState = EMPTY_BIN;

void setup() {
  analogReference(DEFAULT);
  g_i2cAddress = I2C_ADDRESS;
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output
  Wire.begin(I2C_ADDRESS);          // join i2c bus
  Wire.onRequest(requestEvent);     // register event
  Wire.onReceive(receiveEvent);     // register event

  for(int i = 0; i < NUM_LINE_PINS; i++) {
    pinMode(lineSensorPins[i], INPUT);
  }

  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(9600);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");
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
  IRLoop();
  StateLoop();
}

void IRLoop() {
  short IR_distance;

  for (int i=0; i<NUMSENSORS; i++) {
    IR_distance = GetSharpIR(IR_config[i][0],IR_config[i][1]);

    // need to protect the assignment to the global array
    // since the assignment could be interrupted by an I2C request halfway through.
    // This is true since the arduino is an 8 bit processor, and the
    // target is 16 bits - it takes two cycles to make the assignment
    noInterrupts();
      IR_distances[i] = IR_distance;
    interrupts();
  }

}

void StateLoop() {
   analogVal = analogRead(0);
  if(analogVal > 500) {
    beamBroken = true; // Gear was detected (Something entered the gear beam ) 
  }
  else if (analogVal < 500) {
   beamBroken = false;
  }
  
  for(int i = 0; i < NUM_LINE_PINS; i++) {
    lineSensorVal[i] = digitalRead(lineSensorPins[i]);
  }

  for(int i = 0; i < NUM_LINE_PINS; i++) {
    if(lineSensorVal[i] == 0) {
      gearPresent = true;
      break;
    }
    else 
     gearPresent = false;
  }

  determineNextState();
}

short int GetSharpIR(short int sensortype, short int pin) {
  short int distance;
  switch (sensortype) {
    case 1: // GP2Y0A51SK0F 2-15 centimeter range
      distance = 970/(analogRead(pin)-13);
      if(distance > 15) return 16;
      else if(distance < 2) return 1;
      else return distance;
      break;
    case 2: // GP2Y0A21YK0F 10-80 centimeter range
      distance = 4800/(analogRead(pin)-20);
      if(distance > 80) return 81;
      else if(distance < 10) return 9;
      else return distance;
      break;
    case 3: // GP2Y0A60SZLF 20-150 centimeter range
      distance = 9462/(analogRead(pin)-17);
      if(distance > 150) return 151;
      else if(distance < 20) return 19;
      else return distance;
      break;
    default:
      return -1;
    }

}

void determineNextState() { 
  short nextState; 
  switch(currentState) { 
    case FULL_BIN:case GEAR_LIFTING:
      if(gearPresent && !beamBroken) 
        nextState = FULL_BIN;
      else if(gearPresent && beamBroken) 
        nextState = UNKNOWN_STATE;
      else if(!gearPresent && !beamBroken) 
        nextState = GEAR_LIFTING;
      else if(!gearPresent && beamBroken) 
        nextState = GEAR_EXITING;
      break;
    case EMPTY_BIN:
       if(gearPresent && !beamBroken) 
        nextState = FULL_BIN;
      else if(gearPresent && beamBroken) 
        nextState = UNKNOWN_STATE;
      else if(!gearPresent && !beamBroken) 
        nextState = EMPTY_BIN;
      else if(!gearPresent && beamBroken) 
        nextState = UNKNOWN_STATE;
      break;
    case GEAR_EXITING:
      if(gearPresent && !beamBroken) 
        nextState = FULL_BIN;
      else if(gearPresent && beamBroken) 
        nextState = UNKNOWN_STATE;
      else if(!gearPresent && !beamBroken) 
        nextState = EMPTY_BIN;
      else if(!gearPresent && beamBroken) 
        nextState = GEAR_EXITING;
      break; 
    case UNKNOWN_STATE:
      if(gearPresent && !beamBroken) 
        nextState = FULL_BIN;
      else
        nextState = UNKNOWN_STATE;
      break;
    default:
     nextState = EMPTY_BIN;
  }
    // need to protect the assignment to the global array
    // since the assignment could be interrupted by an I2C request halfway through.
    // This is true since the arduino is an 8 bit processor, and the
    // target is 16 bits - it takes two cycles to make the assignment
  noInterrupts();
    currentState = nextState;
  interrupts();
}

void printEnum(int i) {
    Serial.print("Current State: ");
    switch(i) {
    case FULL_BIN:
      Serial.println("FULL_BIN");    
      break;   
    case GEAR_LIFTING:
      Serial.println("GEAR_LIFTING");       
      break;   
    case EMPTY_BIN:
      Serial.println("EMPTY_BIN");
      break;   
    case GEAR_EXITING:
      Serial.println("GEAR_EXITING");
      break;   
    case UNKNOWN_STATE:
      Serial.println("UNKNOWN_STATE");
      break;   
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
    case MODE_GetIR:
      handleGetIRRequest();
      break;
    case MODE_GetGearState:
      handleGetGearStateRequest();
      break;
    case MODE_Diagnostic:
      handleGetDiagnosticRequest();
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
    case 'I':
      g_commandMode = MODE_GetIR;
      break;
    case 'G':
      g_commandMode = MODE_GetGearState;
      break;
    case 'D':
      g_commandMode = MODE_Diagnostic;
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
    Serial.println("    I:  Display IR Value");
    Serial.println("    G:  Display Current Gear State");
    Serial.println("    D:  Display beam and proximity details");
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

void handleGetIRRequest() {
  if (serialMode) {
    Serial.println("In IR");
  }

  writeShorts(IR_distances, NUMSENSORS, serialMode);
}

void handleGetGearStateRequest() {
  if (serialMode) {
    Serial.println("Getting gear states");
  }
  writeShorts(&currentState, 1, serialMode);
}

void handleGetDiagnosticRequest() {
  if (serialMode) {
    Serial.println("Getting diagnostics");
    // write the beam status
    Serial.print("[ ");
    Serial.print(beamBroken ? "beam broken" : "beam NOT broken");
    Serial.println(" ]");    
  } else {
    writeBytes(&beamBroken, 1, byteType, serialMode);
  }
  
  // write the proximity status
  writeBytes(lineSensorVal, NUM_LINE_PINS, byteType, serialMode);  
}

// TODO - write handlers - see StormNetCommon.h for examples
