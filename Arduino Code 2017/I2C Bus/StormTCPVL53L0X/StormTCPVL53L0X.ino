#include <Wire.h>
#include <Ethernet.h>
#include <VL53L0X.h>

#include "StormNetCommon.h"
#include "PCA9633.h"

// Command modes
const char MODE_LIDAR = 6;        // your mode here
const char MODE_I2C_MASTER = 7;   // unpacks a stormnet i2c command to pass downstream and back
const char MODE_I2C_ADDRESSES = 8;
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

// TODO - what does ethernet hearbeat mean, if anything?
const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(10, 54, 22, 177);
const int IPPort=5422;

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(IPPort);
EthernetClient ethernetClient;

#define NUM_LIDARS 8  // Total number of installed LiDar sensors
VL53L0X *sensors[NUM_LIDARS];
short lidarReadings[NUM_LIDARS] = { 0, 0, 0, 0, 0, 0, 0, 0};  //  An integer array for storing sensor readings
byte nodeAddress[NUM_LIDARS]    = { 0, 0, 0, 0, 0, 0, 0, 0};

// The base below cannot overlap with the above mask when you add NUM_LIDARS to the base
// The idea is that we will create the lidar i2c address by adding the bitmask to the base address of the node
#define LIDAR_ADDRESS_MASK 0x20
#define LIDAR_NODE_BASE_ADDRESS 0x04  
#define LIDAR_RANGE_THRESHOLD 2000

int g_nodeCount = 0;

void setup()
{ 
  int index = 0;
  int i = 0;
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output

  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(115200);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  Wire.begin();  // I2C for lidar sensors
  Wire.setClock(WIRE_CLOCK);

  // Look for all devices. The ones between ids 16 - 31 are special - assume they are lidar nodes
  I2CScan(true);  // Uncomment this to see addresses for all I2C devices on the bus  

  for (i = 0; i < MAX_I2C_ADDRESSES ; i++) {
    if (g_i2cAddresses[i] >= LIDAR_NODE_BASE_ADDRESS && g_i2cAddresses[i] < LIDAR_ADDRESS_MASK) {
      Serial.print("Found at i = ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(index);
      Serial.print(" ");
      Serial.println(g_i2cAddresses[i]);
      nodeAddress[index] = g_i2cAddresses[i];
      initializeLidarNode(index);
      delay(1000);
      index++;
    }

    g_nodeCount = index;
  }

  // Test scan to make sure everything was properly configured above.
  Serial.println("About to run I2CScan...");
  I2CScan(true);  // Uncomment this to see addresses for all I2C devices on the bus  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED back on. Eventually the comm blink will take over
  delay(5000);  // hold so we can see the steady LED indicating A-OK
}


void loop()
{   
  int level = 64;
  int wait = 100;
  
  // Get some reading and note if we are in range
  for (int i=0 ; i< g_nodeCount; i++) {
    lidarReadings[i] = sensors[i]->readRangeContinuousMillimeters();
//      handleLidarRequest();
//      writeBytes(nodeAddress, NUM_LIDARS, byteType, g_talkMode);
//      writeBytes(g_i2cAddresses, NUM_LIDARS, byteType, g_talkMode);
//     LEDOUT(nodeAddress[i], level, 0, 0);
      
     Serial.print(" sensor address is ");
     Serial.print(sensors[i]->getAddress());
     Serial.print(" and value is ");
     Serial.println(lidarReadings[i]);
      
    if (lidarReadings[i] > 0 && lidarReadings[i] < LIDAR_RANGE_THRESHOLD) { 
      LEDOUT(nodeAddress[i], level, level, level); 
    } else if (lidarReadings[i] == -1) {      
      LEDOUT(nodeAddress[i], level, level, 0); 
    } else {
      LEDOUT(nodeAddress[i], 0, 0, 0); 
    }
  }

delay(wait);

//  for (int i=0 ; i< g_nodeCount; i++) {
//      LEDOUT(nodeAddress[i], 0, level, 0);
//  }
//  delay(wait);
//
//  for (int i=0 ; i< g_nodeCount; i++) {
//      LEDOUT(nodeAddress[i], 0, 0, level);
//  }
//  delay(wait);
//
//  for (int i=0 ; i< g_nodeCount; i++) {
//      LEDOUT(nodeAddress[i], level, level, level);
//  }
//  delay(wait);
  
  // Flip to serial mode if there is anything to be read. Otherwise back to I2C mode
  if (Serial.available()) {
    g_talkMode = serialMode;
  }
  else {
    g_talkMode = ethernetMode;
  }

  //========== flash heartbeat (etc) LED =============
  currentMillis = millis();
  // the interrupts could change the value of g_blinkInterval which can mess with this logic
  //noInterrupts(); no interrupts in ethernet mode
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

  socket_loop();
}

void initializeLidarNode(int index) {
  int addr = nodeAddress[index];
  VL53L0X *sensor = new VL53L0X();
  sensors[index] = sensor;
  
//      Serial.print("Initializing index = ");
//      Serial.print(index);
//      Serial.print(" addr = ");
//      Serial.print(addr);
//      Serial.print(" sensor is ");
//      Serial.println((long)sensor);

  // RESET of MODE1 register to 0 and turn PCA9633 on
  PCA9633_WriteRegister(addr, PCA9633_TURN_ON, 0x00);  // Keep the all call address running
  LEDOUT(addr, 64, 64, 0);
//  XSHUT(addr, false);
  delay(10); // give it a moment
  
//  XSHUT(addr, true);  
//  sensor->setAddress(addr | LIDAR_ADDRESS_MASK); // that is, set the lidar i2c address to MASK + node address. Nice and simple
//  sensor->setAddress(0x29); // that is, set the lidar i2c address to MASK + node address. Nice and simple
  delay(10); // give it a moment  
  sensor->init();
  sensor->setTimeout(500);
  sensor->startContinuous(10);

  Serial.print(" sensor address is "); 
  Serial.println(sensor->getAddress());


//  delay(5000);
}

void socket_loop() {
  // listen for incoming clients
  // if an incoming client connects, there will be bytes available to read:
  EthernetClient client = server.available();

  if (client) {
    g_ethernetClient = client;
    if (client.available()) {
      receiveEvent(1);
      requestEvent();
    }
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
    case MODE_LIDAR:
      handleLidarRequest();
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
    case 'L':
        g_commandMode = MODE_LIDAR;
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
    default:
      receiveBuiltIn(c);
  }

  while (readAvailable()) c=readByte(); // in case there is other stuff sent that needs to be collected - don't expect this
}

//================================
void handleHelpRequest() {
  if (g_talkMode == serialMode) {
    printBuiltInHelp();
    Serial.println("    L:  Print lidar values");
    g_commandMode = MODE_IDLE;  // This only makes sense in g_talkMode == serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

void handleLidarRequest() {
  if (g_talkMode == serialMode) {
    Serial.println("we are in lidar request");
  }
   
  writeShorts(lidarReadings, NUM_LIDARS, g_talkMode);
}

void handleI2CAddressesReceive() {
  ; // nothing to see here. The work has already been done at startup.
}

void handleI2CAddressesRequest() {
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

