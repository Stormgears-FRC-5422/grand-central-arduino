#include "StormNetCommon.h"

// TODO choose a unique address
const char I2C_ADDRESS = 11;    // each device needs its own 7 bit address

// Command modes
// const char MODE_X = 6;        // your mode here
const char MODE_GetIR = 6;        // your mode here
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

void setup() {
  analogReference(DEFAULT);
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
}

void loop() { //main user command loop
  // Originally intialized as false.  This is a one-way switch
  if (Serial.available())
    serialMode = true;

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
    Serial.println();
    Serial.println("===== Stormgears I2C Device Help =====");
    Serial.println("    P:  Ping - read I2C Address");
    Serial.println("    F:  Change LED to FAST flash. Read 'FAST'");
    Serial.println("    S:  Change LED to SLOW flash. Read 'SLOW'");
    Serial.println("    B:  Change LED flash rate directly - pass another long to say how fast (in milliseconds)");
    Serial.println("    I:  Display IR Value");
// TODO - add menu items
    Serial.println("   \\0:  (or anything unhandled) Read unsingned int counter");
    Serial.println("    ?:  Show this help (otherwise act like \\0)");
    g_commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}



void handleGetIRRequest() {
  if (serialMode) {
    Serial.println("we are in IR");
  }

  writeShorts(IR_distances, NUMSENSORS, shortType);
}

// TODO - write handlers - see StormNetCommon.h for examples
