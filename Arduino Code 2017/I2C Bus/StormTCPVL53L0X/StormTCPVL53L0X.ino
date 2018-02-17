#include <Wire.h>
#include <Ethernet.h>
#include <VL53L0X.h>

#include "StormNetCommon.h"

// Command modes
const char MODE_LIDAR = 6;        // your mode here
const char MODE_I2C_MASTER = 7;   // unpacks a stormnet i2c command to pass downstream and back
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

#define XSHUT_pin0 22 //not really required for address change
#define XSHUT_pin1 23
#define XSHUT_pin2 24
#define XSHUT_pin3 25

//ADDRESS_DEFAULT 0b0101001 or 41
#define Sensor0_newAddress 41 //not required address change
#define Sensor1_newAddress 42
#define Sensor2_newAddress 43
#define Sensor3_newAddress 44

VL53L0X Sensor0;
VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;

#define NUM_LIDARS 4  // Total number of installed LiDar sensors
short lidarReadings[NUM_LIDARS] = { 0, 0, 0, 0};  //  An integer array for storing sensor readings

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

void setup()
{ 
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output

  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(9600);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears I2C Slave Device Diagnostic System");
  Serial.println("Hit '?' for Help");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  Wire.begin();  // I2C for lidar sensors
  
  /*WARNING*/
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NOT 5V TOLERANT. 5V will fry them
  pinMode(XSHUT_pin0, OUTPUT);
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);

  //Set address of sensor and power up next one
  //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  pinMode(XSHUT_pin3, INPUT);
  delay(10);
  Sensor3.setAddress(Sensor3_newAddress);
  Serial.println("Sensor3 address set");

  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  Serial.println("Sensor2 address set");

  pinMode(XSHUT_pin1, INPUT);
  delay(10);
  Serial.println("Sensor1 address set");
  Sensor1.setAddress(Sensor1_newAddress);

  pinMode(XSHUT_pin0, INPUT);
  delay(10);
  Serial.println("Sensor0 address set");
  Sensor0.setAddress(Sensor0_newAddress);

  Serial.println("Initializing...");
  Sensor0.init();
  Sensor1.init();
  Sensor2.init();
  Sensor3.init();

  Serial.println("Setting timeouts...");
  Sensor0.setTimeout(500);
  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Serial.println("Starting continuous scans...");
  Sensor0.startContinuous();
  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor3.startContinuous();

  // Test scan to make sure everything was properly configured above.
  Serial.println("About to run I2CScan...");
  I2CScan();  // Uncomment this to see addresses for all I2C devices on the bus  
}

void loop()
{
  lidarReadings[0] = Sensor0.readRangeContinuousMillimeters();
  lidarReadings[1] = Sensor1.readRangeContinuousMillimeters();
  lidarReadings[2] = Sensor2.readRangeContinuousMillimeters();
  lidarReadings[3] = Sensor3.readRangeContinuousMillimeters();
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


void socket_loop() {
  // listen for incoming clients
  // if an incoming client connects, there will be bytes available to read:
  EthernetClient client = server.available();

//  // simple echo client
//  if (client) {
//    Serial.println("new client");
//    while (client.connected()) {
//      if (client.available()) {
//        char c = client.read();
//        Serial.write(c);
//        // if you've gotten to the end of the line
//        if (c == '\n') break;
//      }
//    }
//  }
  
  if (client) {
    g_ethernetClient = client;
    if (client.available()) {
      receiveEvent(1);
      requestEvent();
    }
  }
}

// Original test loop for web server access. Can be any port...
void http_loop() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 1");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin
          //          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
          //            int sensorReading = analogRead(analogChannel);
          //            int sensorReading = Sensor1.readRangeContinuousMillimeters();
          for (int lidarSensors = 0; lidarSensors < NUM_LIDARS; lidarSensors++) {
            client.print("Sensor ");
            client.print(lidarSensors + 1);
            client.print(": ");
            if (lidarReadings[lidarSensors] > 8180)
              client.print("Out of range");
            else
              client.print(lidarReadings[lidarSensors]);
            //client.print(" is ");
            //client.print(sensorReading);
            client.println("<br />");
          }
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
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

