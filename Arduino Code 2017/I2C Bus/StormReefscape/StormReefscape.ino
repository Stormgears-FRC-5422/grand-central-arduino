#include <Ethernet.h>
#include <EthernetUdp.h>
#include <VL53L0X.h>  // Pololu variety
#include "StormNetCommon.h"
#include "PCA9633.h"

// For 2025 Reefscape

// Command modes
const char MODE_TIMER = 6; // Give me an up-to-date timer value (for diagnostics)
const char MODE_LIDAR = 7;        // your mode here
const char MODE_I2C_MASTER = 8;   // unpacks a stormnet i2c command to pass downstream and back
const char MODE_I2C_ADDRESSES = 9;

// TODO: add more modes

// These are a bit aggressive to save memory.  Keep an eye on them
byte g_i2cCommandBuffer[32]; 
byte g_i2cRequestBuffer[64]; 
byte g_i2cRequestSize;


// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;
unsigned long previousTransmit = 0;
unsigned long transmitInterval = 10; // milliseconds - not sure what the right number is here.
unsigned long timerMillis = 0;
unsigned long timerResults[3] = {0,0,0};

// TODO - what does ethernet hearbeat mean, if anything?
const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

// TODO - move this into the StormNetCommon code (do this when we make it a library)
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
bool useNetwork = false;
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(10, 54, 22, 177);
IPAddress roborioIP(10, 54, 22, 2);
const int udpPort=5423;  

#define NUM_LIDARS 2  // Total number of installed LiDar sensors (maximum - fewer is OK, more can take time transmitting)
#define LIDAR_TIMING_BUDGET 100000

// 0x29 is the default id. This should either not be used or be the last one in the list
int lidarAddresses[NUM_LIDARS] = {0x30, 0x31};
int lidarXshutPins[NUM_LIDARS] = {D3, D4};    // <-- Need to put correct pins here


VL53L0X *sensors[NUM_LIDARS] = {NULL, NULL};
unsigned short lidarReadings[NUM_LIDARS * 2] = { 0, 0};  //  An unsigned short array for storing sensor readings

void setup()
{ 
  int index = 0;
  int i = 0;

  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output

  Wire.begin();  // I2C for lidar sensors

  // This might have something to do with the full StormNet board. Not relevant with a bare Mega.
  // // Enable voltage regulator for Mega connectors
  // pinMode(9, INPUT);
  
  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(115200);             // start serial port and wait for port to open
  Serial.println();
  Serial.println("Stormgears StormNet Diagnostic System");
  Serial.println("Hit '?' for Help");

  if (useNetwork) {
    // start the Ethernet connection and the server:
    Ethernet.begin(mac, ip);
    g_udpClient.begin(udpPort);
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("Network is not used. Set useNetwork to true to enable");
  }
    
  // This disables the lidar devices. They will be enabled again to set their address during the initialize step
  for (int i=0; i < NUM_LIDARS; i++) {
    pinMode(lidarXshutPins[i], OUTPUT);
    digitalWrite(lidarXshutPins[i], LOW);
  }  

  for (int i=0; i < NUM_LIDARS; i++) {
    initializeLidarNode(i);
  }  

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED back on. Eventually the comm blink will take over
  delay(3000);  // hold so we can see the steady LED indicating A-OK
}


void loop()
{   
  currentMillis = millis();
    
  // Get some reading and note if we are in range
  for (int i=0 ; i< NUM_LIDARS; i++) {
    // This patterns comes from the Pololu implementation of readRangeContinuousMillimeters 
    // (source code https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp)
    // It checks the interrupt flags directly to avoid blocking, then resets. 
    // TODO consider timeouts
    if (sensors[i]->readReg(PCA_RESULT_INTERRUPT_STATUS) & 0x07) { // Interrupt flag set
      lidarReadings[2*i] = sensors[i]->readReg16Bit(PCA_RESULT_RANGE_STATUS);  // Read range value
      lidarReadings[2*i + 1] = 1; // TODO figure out the right way to get status code
      sensors[i]->writeReg(PCA_SYSTEM_INTERRUPT_CLEAR, 0x01);  // reset interrupt
    }
  }
   
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  if (Serial.available()) {
    g_talkMode = serialMode;
  }
  else {
    if (useNetwork) {
      g_talkMode = udpMode;
    } else {
      g_talkMode = serialMode;
    }
  }

  //========== flash heartbeat (etc) LED =============
  // the interrupts could change the value of g_blinkInterval which can mess with this logicL
  //noInterrupts(); no interrupts (they don't happen) in ethernet mode
  // Blink superfast if we haven't heard from the master in a while
  if ( (currentMillis - previousI2C) > i2cHeartbeatTimeout)  // stale
    g_blinkInterval = 100;
  else
    g_blinkInterval = g_blinkInterval==100 ? 1000 : g_blinkInterval;

  boolean flipNow = ( (currentMillis - previousBlink) >= g_blinkInterval);
  //interrupts();

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

  // The trouble with mixed-mode communication is that you don't know whether the other side is going to try
  // to read the *prior* message that was sent as a response to its request. So skip this until a better protocol can be worked out

  if ( (currentMillis - previousTransmit > transmitInterval) ) {
    if (useNetwork) {
      udpSendLoop();
    } else {
      delay(1000);
      dataLoop();
    }
    previousTransmit = currentMillis;  // reset
  }
}

// UDP Mode command loop
void udpSendLoop() {
  // It seems to take a while for the client connection to drop once it has been disconnected from the robot side
  // Lets try UDP instead

  if (g_udpClient.beginPacket(roborioIP, udpPort) ) {
    dataLoop();    
    if (!g_udpClient.endPacket()) {
      Serial.write("Error ending packet!");
    }
  }
  else {
    Serial.write("Error beginning packet!");  
  }
}

void dataLoop() {
    // This must match what is expected on the server side
    handlePingRequest();  // "P", 1 * 1 bytes;  offset 0 + 1 = 1
    handleFastRequest();  // "F", 1 * 4 bytes;  offset 1 + 4 = 5
    handleSlowRequest();  // "S", 1 * 4 bytes;  offset 5 + 4 = 9
    handleBlinkRequest(); // "B", 1 * 4 bytes;  offset 9 + 4 = 13
    handleLidarRequest(); // "L", 1 * 4 bytes;  offset 13 + 4 = 17
    handleTimerRequest(); // ":", 3 * 4 bytes;  offset 17 + 12 = 29
//    // length 29
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
    case MODE_TIMER:
      handleTimerRequest();
      break;
    case MODE_LIDAR:
      handleLidarRequest();
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
    case ':' :
        g_commandMode = MODE_TIMER;
        break;
    case 'L':
        g_commandMode = MODE_LIDAR;
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
    Serial.println("    ::  (colon) Return time details");
    Serial.println("    L:  Print lidar values");

    g_commandMode = MODE_IDLE;  // This only makes sense in g_talkMode == serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}


void handleTimerRequest() {
  timerResults[0] = timerMillis;   // The last time we got a time report
  timerResults[1] = currentMillis; // The beginning of the current loop
  timerResults[2] = millis();      // right now (which is hopefully close to the current loop time(

  timerMillis = timerResults[2];
  writeLongs((long*)timerResults, 3, g_talkMode);
}

void handleLidarRequest() {
  if (g_talkMode == serialMode) {
    Serial.println("we are in lidar request");
  }
  // first short is distance, second is quality (1 is good, 0 is bad)
  writeShorts((short*)lidarReadings, NUM_LIDARS * 2, g_talkMode);
}


// Presumes lidar 
void initializeLidarNode(int index) {
  int addr = lidarAddresses[index];
  VL53L0X *sensor = sensors[index];

  // XSHUT	This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. 
  // Driving this pin low puts the sensor into hardware standby. This input is not level-shifted and it is not 5V-tolerant.
  // So we cannot drive it high. We need to just take the LOW away by flipping to INPUT.
  pinMode(lidarXshutPins[i], INPUT);
  delay(50); // wait for the device to wake up

  // The sensor class is pretty touchy to reentrant calls.  Let's just avoid that problem by dumping and
  // recreating them in this function
  // There is a slim chance that this code could leave an untouched sensor class floating around, but
  // we won't hit it if the g_nodeCount is correct (this can happen if the count drops for some reason)
  if (sensor) {
    delete sensor;
  }

  sensor = new VL53L0X();
  sensors[index] = sensor;
      
  // In fact setting up multiple nodes is touchy. Here we really only have one, and it is at the default location
  sensor->setAddress(addr);
  sensor->init();
  sensor->setTimeout(500);
  sensor->setSignalRateLimit(0.1);
  sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor->setMeasurementTimingBudget(LIDAR_TIMING_BUDGET);
  
  sensor->startContinuous();
}
