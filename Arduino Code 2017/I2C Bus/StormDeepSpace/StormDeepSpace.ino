#include <Adafruit_NeoPixel.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <VL53L0X.h>
#include "StormNetCommon.h"
#include "PCA9633.h"


// Based on 2018 StormTCPVL53L0X.ino
// adds line sensors (Cytron LSS05) to enable tape following in Deep Space competition.

// Command modes
const char MODE_TIMER = 6; // Give me an up-to-date timer value (for diagnostics)
const char MODE_LIDAR = 7;        // your mode here
const char MODE_I2C_MASTER = 8;   // unpacks a stormnet i2c command to pass downstream and back
const char MODE_I2C_ADDRESSES = 9;
const char MODE_LIDAR_PAIR = 10;  // Give me pair 0, etc.  0 means index 0, 1 N means index 2*N, 2*N+1
const char MODE_LIDAR_PAIR_THRESHOLD = 11; // Set the distance threshold to indicate same distance for a pair of lidars (all pairs share same threshold)

const char MODE_LINE_VALUE = 12; // Give me a number between -1 and 1 to indicate how far along the set of sensors I am
const char MODE_LINE_VALUE_LIST = 13;

const char MODE_RING_OFF = 14;
const char MODE_RING_ON = 15;
const char MODE_RING_COLOR = 16;

// TODO: add more modes

// for line sensors. These are analog pins that need to be used in digital mode
#define NUM_LINE_PINS 15 // Number of pins for the line sensor
byte lineCalibrationPin = 0; 
byte lineSensorPins[NUM_LINE_PINS] = {31, 29, 27, 25, 23,   30, 28, 26, 24, 22,    41, 39, 37, 35, 33};  // next batch is 32, 34, 36, 38, 40
byte calibrationPin = 42;
byte lineSensorValues[NUM_LINE_PINS] =  {0, 0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0, 0};
float lineSensorLocations[NUM_LINE_PINS] = {-8.75, -7.75, -6.75, -5.75, -4.75,   -2, -1, 0, 1, 2,   5, 6, 7, 8, 9};  // this might be slowish - could use shorts in mm??
float lineSensorWidth = 17.75;  // from above

// These are a bit aggressive to save memory.  Keep an eye on them
byte g_i2cCommandBuffer[32]; 
byte g_i2cRequestBuffer[64]; 
byte g_i2cRequestSize;


// blink control
const int ledPin =  13;             // the number of the LED pin
const int enSensors = 9;
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
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):

IPAddress ip(10, 54, 22, 177);
IPAddress roborioIP(10, 54, 22, 2);
const int ipPort=5422;  // use as local listener and remote listener as well
const int udpPort=5423;  
EthernetClient client;
EthernetServer server(ipPort);


#define NUM_LIDARS 2  // Total number of installed LiDar sensors (maximum - fewer is OK, more can take time transmitting)
// The base below cannot overlap with the above mask when you add NUM_LIDARS to the base
// The idea is that we will create the lidar i2c address by adding the bitmask to the base address of the node
#define LIDAR_ADDRESS_MASK 0x20
#define LIDAR_NODE_BASE_ADDRESS 0x04  
#define LIDAR_RANGE_THRESHOLD 2000
#define LIDAR_TIMING_BUDGET 100000
#define LIDAR_PAIR_MAX_THRESHOLD 40

// 16
//VL53L0X *sensors[NUM_LIDARS] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
//short lidarReadings[NUM_LIDARS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //  An integer array for storing sensor readings
//byte nodeAddress[NUM_LIDARS]    = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// 2
VL53L0X *sensors[NUM_LIDARS] = {NULL, NULL};
unsigned short lidarReadings[NUM_LIDARS] = { 0, 0};  //  An integer array for storing sensor readings
byte nodeAddress[NUM_LIDARS]    = { 0, 0};
int g_nodeCount = 0;  // how many do we actually find?
volatile long g_lidarPair = 0;
volatile long g_lidarPairThreshold = 5;

boolean g_showLidarActivity = true;

// neopixel support
#define NUMLIGHTS 40
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMLIGHTS, 2, NEO_RGBW); //first number is total count, ,second number is pin#

//colors
// TODO - need to make the brightness scalable or check the api.
uint32_t off = strip.Color(0, 0, 0, 0);
uint32_t white = strip.Color(0, 0, 0, 255);
uint32_t green = strip.Color(255, 0, 0, 0);  // DIM
uint32_t red = strip.Color(0, 255, 0, 0);
uint32_t teal = strip.Color(120, 1, 67, 2);
uint32_t blue = strip.Color(0, 0, 255, 0);

uint32_t lastColor = green;

char next_Ring_State=1;
char Ring_State=0;



void setup()
{ 
  int index = 0;
  int i = 0;
  int addr;
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output
  pinMode(enSensors, OUTPUT);

  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(115200);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  g_udpClient.begin(udpPort);
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  
  // for line sensor
  pinMode(lineCalibrationPin, OUTPUT);
  digitalWrite(lineCalibrationPin, HIGH);  // not calibrating - set LOW to calibrate 
  for(int i = 0; i < NUM_LINE_PINS; i++) {
    pinMode(lineSensorPins[i], INPUT);
  } 

  digitalWrite(enSensors, LOW);
  delay(1000);
  digitalWrite(enSensors, HIGH);
  delay(500);
  
  Wire.begin();  // I2C for lidar sensors
  Wire.setClock(WIRE_CLOCK);

  // Look for all devices. The ones between ids 16 - 31 are special - assume they are lidar nodes
  //delay(5000);
  Serial.println("Initial I2C scan...");
  I2CScan(true);  // Uncomment this to see addresses for all I2C devices on the bus  

  initializeAllNodes();
    
  // Test scan to make sure everything was properly configured above.
  Serial.println("Secondary I2C scan...");
  I2CScan(true);  // Uncomment this to see addresses for all I2C devices on the bus  

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED back on. Eventually the comm blink will take over
  delay(3000);  // hold so we can see the steady LED indicating A-OK

  strip.begin();
  setRingLights();

  // Finally set to blue to indicate setup has completed
  for (int i=0; i < NUM_LIDARS; i+=2) {
    LEDOUT(nodeAddress[i], BLUE, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR); 
    LEDOUT(nodeAddress[i+1], BLUE, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR); 
  }  
}


void loop()
{   
  currentMillis = millis();
  
  // Only bother setting the lights if the state has changed.
  if ( (next_Ring_State != Ring_State) ) {  // TODO - add others if there are more lights!
    setRingLights();
  }

  // Right now, all of this happens in one loop.
  // TODO - look (or at least look out) for the timing here - across lidars especially
  for(int i = 0; i < NUM_LINE_PINS; i++) {
    lineSensorValues[i] = digitalRead(lineSensorPins[i]);
  }
  
  // Get some reading and note if we are in range
  for (int i=0 ; i< g_nodeCount; i++) {
    // lidarReadings[i] = sensors[i]->readRangeContinuousMillimeters();
    // This patterns comes from the Pololu implementation of readRangeContinuousMillimeters 
    // (source code https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp)
    // It checks the interrupt flags directly to avoid blocking, then resets. 
    // TODO consider timeouts
    if (sensors[i]->readReg(PCA_RESULT_INTERRUPT_STATUS) & 0x07) { // Interrupt flag set
      lidarReadings[i] = sensors[i]->readReg16Bit(PCA_RESULT_RANGE_STATUS);  // Read range value
      sensors[i]->writeReg(PCA_SYSTEM_INTERRUPT_CLEAR, 0x01);  // reset interrupt
    }
  }
   
  // Lidar status indicators
  // These LEDOUT calls (in this loop with NUM_LIDARS = 16) take a total of about 
  // 11 ms as measured by the timer. This needs to be optimized or eliminated
//  if (g_showLidarActivity) {
//    for (int i=0; i < NUM_LIDARS; i+=2) {
//      if (lidarReadings[i] > 0 && lidarReadings[i] < LIDAR_RANGE_THRESHOLD &&
//          lidarReadings[i+1] > 0 && lidarReadings[i+1] < LIDAR_RANGE_THRESHOLD &&
//          abs(lidarReadings[i] - lidarReadings[i+1]) < g_lidarPairThreshold) { 
//        LEDOUT(nodeAddress[i], GREEN, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR); 
//        LEDOUT(nodeAddress[i+1], GREEN, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR); 
//      } else {
//        LEDOUT(nodeAddress[i], BLUE, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR); 
//        LEDOUT(nodeAddress[i+1], BLUE, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR); 
//      }
//    }
//  }

  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  if (Serial.available()) {
    g_talkMode = serialMode;
  }
  else {
    //g_talkMode = ethernetMode;
    g_talkMode = udpMode;
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
//  //interrupts();

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

  // TODO - need a decent way to debug with this, right now just add or delete g_ethernetClient && to the condition below
  // to allow this to work or not without that connection
  //socket_loop(); // the connection is actually made in here

  if ( (currentMillis - previousTransmit > transmitInterval) ) {
    send_loop();
    previousTransmit = currentMillis;  // reset
  }
}

// UDP Mode command loop
void send_loop() {
  // It seems to take a while for the client connection to drop once it has been disconnected from the robot side
  // Lets try UDP instead

  if (g_udpClient.beginPacket(roborioIP, udpPort) ) {
    // This must match what is expected on the server side
    handlePingRequest();  // "P", 1 * 1 bytes;  offset 0
    handleFastRequest();  // "F", 1 * 4 bytes;  offset 1
    handleSlowRequest();  // "S", 1 * 4 bytes;  offset 5
    handleBlinkRequest(); // "B", 1 * 4 bytes;  offset 9
    handleLidarRequest(); // "L", 2 * 2 bytes;  offset 13
    handleLineValueRequest(); // "V", 2 * 4 bytes;  offset 17
    handleTimerRequest(); // ":", 3 * 4 bytes;  offset 25
    // length 37
    
    if (!g_udpClient.endPacket()) {
      Serial.write("Error ending packet!");
    }
  }
  else {
    Serial.write("Error beginning packet!");  
  }
}

// Used with ethernetMode not udpMode
void socket_loop() {
  // listen for incoming clients
  // if an incoming client connects, there will be bytes available to read:
  client = server.available();

  if (client) {
    g_ethernetClient = client;
    if (client.available()) {
      receiveEvent(1);
      requestEvent();
    }
  }
}

void setRingLights() {
   // Update the "current" status to reflect the new normal
   Ring_State = next_Ring_State;

   if (Ring_State==0) for (int i=0; i<NUMLIGHTS; i++) strip.setPixelColor(i,off);
   if (Ring_State==1) for (int i=0; i<NUMLIGHTS; i++) strip.setPixelColor(i,lastColor);

   strip.show();
}

void setRingLightColor(byte R, byte G, byte B, byte W) {
   lastColor = strip.Color(G,R,B,W);
   setRingLights();
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
    case MODE_LINE_VALUE:
      handleLineValueRequest();
      break;
    case MODE_LINE_VALUE_LIST:
      handleLineValueListRequest();
      break;
    case MODE_LIDAR:
      handleLidarRequest();
      break;
    case MODE_LIDAR_PAIR:
      handleLidarPairRequest();
      break;

    case MODE_RING_ON:
    case MODE_RING_OFF:
      handleRingLightRequest();
      break;
    case MODE_RING_COLOR:
      handleRingLightColorRequest();
      break;
    case MODE_LIDAR_PAIR_THRESHOLD:
      handleLidarPairThresholdRequest();
      break;
    case MODE_HELP:
      handleHelpRequest();
      break;
    case MODE_I2C_ADDRESSES:
      handleI2CAddressesRequest();
      break;
    case MODE_I2C_MASTER:
      handleI2CMasterRequest();
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
    case 'V':
        g_commandMode = MODE_LINE_VALUE;
        break;
    case 'v': 
        g_commandMode = MODE_LINE_VALUE_LIST;
        break;
    case 'L':
        g_commandMode = MODE_LIDAR;
        break;
    case 'R':
        g_commandMode = MODE_LIDAR_PAIR;
        handleLidarPairReceive();
        break;
    case 'T':
        g_commandMode = MODE_LIDAR_PAIR_THRESHOLD;
        handleLidarPairThresholdReceive();
        break;
    case '?':
      g_commandMode = MODE_HELP;
      break;
    case 'A':
      g_commandMode = MODE_I2C_ADDRESSES;
      handleI2CAddressesReceive();
      break;
    case '@':
      g_commandMode = MODE_I2C_MASTER;
      handleI2CMasterReceive();
      break;
    case '!':
      g_showLidarActivity = !g_showLidarActivity;
      // fall through to default handling  
    case '1':
      g_commandMode = MODE_RING_ON;
      next_Ring_State = 1;
      break;
    case '2':
      g_commandMode = MODE_RING_OFF;
      next_Ring_State = 0;
      break;
    case 'C':
      g_commandMode = MODE_RING_COLOR;
      handleRingLightColorReceive();
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
    Serial.println("    V:  Return line sensor position value");
    Serial.println("    v:  Return line sensor position list");
    Serial.println("    A:  Reset I2C addresses");
    Serial.println("    L:  Print lidar values");
    Serial.println("    R:  Report lidar pair [value 0, 1 2]");
    Serial.println("    T:  Change range threshold [value in mm]");
    Serial.println("    !:  Show lidar activity (on LEDs)");
    Serial.println("    1:  Ring light ON");
    Serial.println("    2:  Ring light OFF");
    Serial.println("    C:  Ring light color [R G B W ]");

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

// TODO - change this to something that does processing for PID - that is, return -1 to 1.
// details pending
void handleLineValueRequest() {
  float result[2] = {0,0};
  float sum=0;
  short count=0;
  for (int i=0; i< NUM_LINE_PINS; i++) {
    sum += lineSensorValues[i] * lineSensorLocations[i];
    count += lineSensorValues[i];
    }

  if (count) {
    result[0] = sum / (float) count;
    // Uncomment this to normalize to -1..1
    // result[0] /= (lineSensorWidth / 2.0);
    result[1] = count;
  }
    
  writeFloats(result, 2, g_talkMode);
}

void handleLineValueListRequest() {
  if (g_talkMode == serialMode) {
    Serial.println("we are in Line Value request");
  }
  writeBytes((void*)lineSensorValues, NUM_LINE_PINS, byteType, g_talkMode);
}

void handleLidarRequest() {
  if (g_talkMode == serialMode) {
    Serial.println("we are in lidar request");
  }
   
  writeShorts((short*)lidarReadings, NUM_LIDARS, g_talkMode);
}

// No need for handleLidarReceive()

void handleLidarPairRequest() {
  if (g_talkMode == serialMode) {
    Serial.println("we are in lidar pair request");
  }
   
  writeShorts((short*)lidarReadings + 2*g_lidarPair, 2, g_talkMode); // two shorts per pair
}

void handleLidarPairReceive() {
  long pair;

  readLongs(&pair, 1);

  // just for status. Doesn't get sent back to master device
  switch (g_talkMode) {
    case serialMode:
      Serial.print("pair now ");
      Serial.println(pair);
      break;
    case ethernetMode:
    default:
      break;
  }

  if (pair < 0 || pair > NUM_LIDARS / 2)
    pair = 0;
    
  g_lidarPair = pair;
}

void handleLidarPairThresholdRequest() {
  if (g_talkMode == serialMode) {
    Serial.println("we are in lidar pair threshold request");
  }
   
  writeBytes((void*)&g_lidarPairThreshold, 4, longType, g_talkMode);
}

void handleLidarPairThresholdReceive() {
  long t;

  readLongs(&t, 1);

  // just for status. Doesn't get sent back to master device
  switch (g_talkMode) {
    case serialMode:
      Serial.print("threshold now ");
      Serial.println(t);
      break;
    case ethernetMode:
    default:
      break;
  }

  if (t < 0 || t > LIDAR_PAIR_MAX_THRESHOLD)
    t = LIDAR_PAIR_MAX_THRESHOLD;

  g_lidarPairThreshold = t;
}

void handleI2CAddressesReceive() {
  ; // nothing to see here. The work has already been done at startup.
}

void handleI2CAddressesRequest() {
  initializeAllNodes();
  I2CScan(false);  // Update; skip printout  
  writeBytes((void*)g_i2cAddresses, MAX_I2C_ADDRESSES, byteType, g_talkMode);
}


void handleI2CMasterReceive() {
  byte buffer[3];
  byte receiveSize;

  readBytes(buffer, 3, byteType);
  g_i2cAddress = buffer[0];
  receiveSize = buffer[1];
  g_i2cRequestSize = buffer[2];

  if (g_talkMode == serialMode) {
    while (!Serial.available()) {;}  // wait for the character to show up
  }
  
  readBytes(g_i2cCommandBuffer, receiveSize, byteType);  //buffer now contains the full i2c command string
  
  Wire.beginTransmission(g_i2cAddress);
    Wire.write(g_i2cCommandBuffer, receiveSize);
  Wire.endTransmission(g_i2cAddress);

}

void handleI2CMasterRequest() {
  byte b;
  g_i2cRequestBuffer[0] = 0; // start pessimistic - assume fail

  Wire.requestFrom((int)g_i2cAddress, (int)g_i2cRequestSize);
  for (int i=0; i< g_i2cRequestSize; i++) {
    if (Wire.available()) {
      g_i2cRequestBuffer[0] = 1; // succeeding, at least

      b = Wire.read();    // receive a byte
      g_i2cRequestBuffer[i+1] = b;
    }
  }
  
  writeBytes((void*)g_i2cRequestBuffer, g_i2cRequestSize + 1, byteType, g_talkMode);
}

// Presumes lidar 
void initializeLidarNode(int index) {
  int addr = nodeAddress[index];
  VL53L0X *sensor = sensors[index];

  // The sensor class is pretty touchy to reentrant calls.  Let's just avoid that problem by dumping and
  // recreating them in this function
  // There is a slim chance that this code could leave an untouched sensor class floating around, but
  // we won't hit it if the g_nodeCount is correct (this can happen if the count drops for some reason)
  if (sensor) {
    delete sensor;
  }

  sensor = new VL53L0X();
  sensors[index] = sensor;
      
  LEDOUT(addr, CYAN, LEDOUT_XSHUT_ON, PWM_ON_WITH_LIDAR);
  delay(XSHUT_ON_WAIT); // give it a moment  

  sensor->setAddress(addr + LIDAR_ADDRESS_MASK); // that is, set the lidar i2c address to MASK + node address. Nice and simple - just add a bit
  sensor->init();
  sensor->setTimeout(500);
  sensor->setSignalRateLimit(0.1);
  sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor->setMeasurementTimingBudget(LIDAR_TIMING_BUDGET);
  
  sensor->startContinuous();
}

void initializeAllNodes() 
{
  int addr, i, index;

  Serial.println("In initializeAllNodes");
  I2CScan(true);
  for (i = 0, index = 0; i < MAX_I2C_ADDRESSES ; i++) {
    if (g_i2cAddresses[i] >= LIDAR_NODE_BASE_ADDRESS && g_i2cAddresses[i] < LIDAR_ADDRESS_MASK) {
      Serial.print("Found node ");
      Serial.print(i);
      Serial.print(" at i2c address ");
      Serial.println(g_i2cAddresses[i]);
      addr = g_i2cAddresses[i];
      nodeAddress[index++] = addr;

      PCA9633_WriteRegister(addr, PCA9633_MODE1, B00000001);  // Start basic node 0 Keep the all call address running
      PCA9633_WriteRegister(addr, PCA9633_MODE2, B00000000);  // Set OUTDRV(bit 2) to open drain. Keep INVRT (bit 4) at 0
      LEDOUT(addr, YELLOW, LEDOUT_XSHUT_OFF, PWM_ON_WITH_LIDAR); // Start by turning off all of the lidar devices
    }
    g_nodeCount = index;
  }
  delay(500);

  for (i = 0; i < g_nodeCount ; i++) {
    initializeLidarNode(i);
  }
}

void handleRingLightRequest() {
  // Nothing much to say - just return the counter
  handleDefaultRequest();
}

void handleRingLightColorReceive() {
  byte buf[4];

  readBytes(buf, 4, byteType);
  
  // just for status. Doesn't get sent back to master device
  switch (g_talkMode) {
    case serialMode:
      Serial.print("Color now ");
      Serial.print(buf[0], HEX);
      Serial.print(",");
      Serial.print(buf[1], HEX);
      Serial.print(",");
      Serial.print(buf[2], HEX);
      Serial.print(",");
      Serial.println(buf[3], HEX);
      break;
    case ethernetMode:
    case udpMode:
    default:
      break;
  }

  setRingLightColor(buf[0], buf[1], buf[2], buf[3]);
}


void handleRingLightColorRequest() {
  handleDefaultRequest();
}
