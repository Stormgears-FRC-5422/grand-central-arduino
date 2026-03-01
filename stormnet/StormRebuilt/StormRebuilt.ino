#include <Ethernet.h>
#include <EthernetUdp.h>
// #include <avr/wdt.h>

#include "DFRobot_MatrixLidar.h"

#include "StormNetCommon.h"

// For 2026 Rebuilt

// Command modes
const char MODE_TIMER = 6; // Give me an up-to-date timer value (for diagnostics)
const char MODE_MATRIX_LIDAR = 7;        // your mode here

// TODO: add more modes

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
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

bool useNetwork = false;
IPAddress ip(10, 54, 22, 177);
IPAddress roborioIP(10, 54, 22, 2);
const int udpPort=5423;  

#define MATRIX_LIDAR_ADDRESS 0x33
#define MATRIX_LIDAR_BUFFER_SIZE 64
#define MATRIX_LIDAR_BAD_VALUE 4000
#define MATRIX_LIDAR_NUM_RESULTS 1

DFRobot_MatrixLidar_I2C tof(MATRIX_LIDAR_ADDRESS);
uint16_t matrixLidarBuffer[MATRIX_LIDAR_BUFFER_SIZE];
long matrixLidarResults[MATRIX_LIDAR_NUM_RESULTS] = {0};


void setup()
{ 
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output

  // Enable voltage regulator for Mega connectors
  // pinMode(9, INPUT);
  
  // Apparently there isn't a way to tell whether the Serial usb is connected or not, but this should be harmless if not.
  // note that Serial resets when the usb cable is connected, so we can be sure that setup will be called at that time
  Serial.begin(115200);             // start serial port at 115200 bps and wait for port to open
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
    Serial.println("Not using network. Set useNetwork to true to enable");
  }
    
  // Matrix sensor
  initMatrixLidar();

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED back on. Eventually the comm blink will take over
  Serial.print("Ready");
  // delay(3000);  // hold so we can see the steady LED indicating A-OK
}


void loop()
{   
  currentMillis = millis();
    
  getMatrixLidarData();
  matrixLidarResults[0] = getMatrixLidarAverage();

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
      send_loop();
    } else {
      data_loop();
      delay(1000);
    }
    previousTransmit = currentMillis;  // reset
  }

}


// UDP Mode command loop
void send_loop() {
  // It seems to take a while for the client connection to drop once it has been disconnected from the robot side
  // Lets try UDP instead

  if (g_udpClient.beginPacket(roborioIP, udpPort) ) {
    data_loop();
    
    if (!g_udpClient.endPacket()) {
      Serial.write("Error ending packet!");
    }
  }
  else {
    Serial.write("Error beginning packet!");  
  }
}


void data_loop() {
    // This must match what is expected on the server side
    handlePingRequest();  // "P", 1 * 1 bytes;  offset 0 + 1 = 1
    handleFastRequest();  // "F", 1 * 4 bytes;  offset 1 + 4 = 5
    handleSlowRequest();  // "S", 1 * 4 bytes;  offset 5 + 4 = 9
    handleBlinkRequest(); // "B", 1 * 4 bytes;  offset 9 + 4 = 13
    handleMatrixLidarRequest(); // "M", 1 * 4 bytes;  offset 13 + 4 = 17
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
    case MODE_MATRIX_LIDAR:
      handleMatrixLidarRequest();
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
  char c = 0;
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
    case 'M':
        g_commandMode = MODE_MATRIX_LIDAR;
        break;
    case '?':
      g_commandMode = MODE_HELP;
      break;
    default:
      receiveBuiltIn(c);
  }

  while (readAvailable()) c=readByte(); // in case there is other stuff sent that needs to be collected - don't expect this
}

// Use this if the sensors end up locking up the device. 
// void hardReset() {
//   wdt_enable(WDTO_15MS);
//   while(1) {}
// }

//COMMAND PROCESSORS HERE

//================================
void handleHelpRequest() {
  if (g_talkMode == serialMode) {
    printBuiltInHelp();
    Serial.println("    ::  (colon) Return time details");
    Serial.println("    M:  Print matrix lidar values");

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
  if (g_talkMode == serialMode) {
    Serial.print("timer request: ");
  }
  writeLongs((long*)timerResults, 3, g_talkMode);
}

void handleMatrixLidarRequest() {
  if (g_talkMode == serialMode) {
    Serial.print("matrix lidar request: ");
  }

  writeLongs((long*)matrixLidarResults, MATRIX_LIDAR_NUM_RESULTS, g_talkMode);
}

void initMatrixLidar() {
  while(tof.begin() != 0){
    Serial.println("begin error !!!!!");
  }
  Serial.println("begin success");
  //config matrix mode
  while(tof.setRangingMode(eMatrix_8X8) != 0){
    Serial.println("init error !!!!!");
    delay(1000);
  }
  Serial.println("init success");
}

void getMatrixLidarData() {
  tof.getAllData(matrixLidarBuffer);
}

long getMatrixLidarAverage() {
  int count = 0;
  long sum = 0;
  long value = 0;

  for(uint8_t i = 0; i < 8; i++){
    for(uint8_t j = 0; j < 8; j++){
      value = matrixLidarBuffer[i * 8 + j];
      
      if (value != MATRIX_LIDAR_BAD_VALUE) {
        sum += value;
        count++;
      }
    }
  }

  if (count == 0) return -1;
  else return sum / count;
}