#include <Wire.h>

// TODO choose a unique address
const char I2C_ADDRESS = 5;    // each device needs its own 7 bit address

// TODO: flip to false if not testing with USB
const boolean serialMode = false;             // false means I2C

// Command modes
// You can add mode modes here, but please don't remove these
const char MODE_IDLE = -1;        // Only for serial loop
const char MODE_DEFAULT = 0;      // send back a cycle of numbers
const char MODE_HELP = 1;         // Serial only (!) Display help
const char MODE_SLOW = 2;         // blink the LED slow.  Request reply with word SLOW
const char MODE_FAST = 3;         // blink the LED fast.  Request reply with word FAST
const char MODE_PING = 4;         // send back the word PING on request
const char MODE_BLINK = 5;        // you tell me how fast to blink. Expects milliseconds as a long
// TODO: add more modes

volatile char commandMode = 0;
volatile unsigned int counter = 0;           // global counter for default handler

int serialRead = 1;

// To help with serial display
enum dataType {
  textType,
  rawType,
  byteType,
  shortType,
  longType,
  floatType,
  doubleType
};

// blink control
const int ledPin =  13;             // the number of the LED pin
int ledState = LOW;                 // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;
volatile long blinkInterval = 100;           // interval at which to blink (milliseconds)

const unsigned long int i2cHeartbeatTimeout = 15000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
volatile unsigned long previousI2C = 0;   // will store last time LED was updated

void setup() {
  previousI2C = millis();           // start the timer now
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);          // set the digital pin as output
  Wire.begin(I2C_ADDRESS);          // join i2c bus
  Wire.onRequest(requestEvent);     // register event
  Wire.onReceive(receiveEvent);     // register event
  if (serialMode) {
    Serial.begin(9600);             // start serial port at 9600 bps and wait for port to open
    Serial.println();
    Serial.println("Stormgears I2C Slave Device Diagnostic System");
    Serial.println("Hit '?' for Help");
  }
}

void loop() { //main user command loop
  //========== flash heartbeat (etc) LED =============
  currentMillis = millis();

  boolean staleI2C  = ( (currentMillis - previousI2C) > i2cHeartbeatTimeout);

  noInterrupts();  // the interrupts could change the value of blinkInterval which can mess with this logic
    // Blink superfast if we haven't heard from the master in a while
    if (staleI2C)
      blinkInterval = 100;
    else
      blinkInterval = blinkInterval==100 ? 1000 : blinkInterval;

    boolean flipNow = ( (currentMillis - previousBlink) >= blinkInterval);
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
    Serial.println("Looping");
    receiveEvent(5); // call the interrupt handler directly.  We may or may not read this many bytes
    requestEvent();
  }
//  if (serialMode && flipNow) {    // run repeatedly in serial mode, but not too quickly (just follow the blink)
//    if (commandMode != MODE_IDLE) {
//      requestEvent();
//    }
//  }
}


void printData(byte* array, uint8_t array_size, dataType dType = rawType) {
  if (dType == textType) {
    Serial.print((char*)array);
  }
  else {
    Serial.print("[");
    for (int i = 0; i < array_size;) {
      switch (dType) {
        case shortType:
          i+=sizeof(short);
          Serial.print(*((short*)(array + i)));
          break;
        case longType:
          Serial.print(*((long*)(array + i)));
          i+=sizeof(long);
          break;
        case floatType:
          Serial.print(*((float*)(array + i)));
          i+=sizeof(float);
          break;
        case doubleType:
          Serial.print(*((double*)(array + i)));
          i+=sizeof(double);
          break;
        case rawType:
        case byteType:
        default:
          i+=sizeof(char);
          break;
      }
      Serial.println(" , ");
    }
    Serial.println("]");
  }
}

// Helper to write to either the I2C Wire or Serial
void writeBytes(void* buffer, byte count, dataType dType = rawType) {
  if (serialMode) {
    printData( (byte*)buffer, count, dType );
  } else {
    Wire.write((byte*)buffer, count);
  }
}

boolean readAvailable() {
  if (serialMode) return Serial.available();
  else return Wire.available();
}

// ultimately dealing with single byte reads.
// this could be optimized a bit, but this is pretty clear.
byte readByte() {
  if (serialMode) {
    byte c = Serial.read();
    Serial.print("Received ");
    Serial.println(c, HEX);
    return c;
  }
  else return (byte)Wire.read();
}

// Fill a buffer with "count" bytes read from the bus
void readBytes(byte* buffer, int count) {
  for (int i=0; i<count; i++)
    buffer[i] = readByte();
}

// Fill a buffer with "count" shorts read from the bus
void readShorts(short* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(short));
}

// Fill a buffer with "count" shorts read from the bus
void writeShorts(short* buffer, int count) {
  writeBytes((byte*) buffer, count * sizeof(short), shortType);
}

// Fill a buffer with "count" longs read from the bus
void readLongs(long* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(long));
}

// Fill a buffer with "count" longs read from the bus
void writeLongs(long* buffer, int count) {
  writeBytes((byte*) buffer, count * sizeof(long), longType);
}

// Fill a buffer with "count" floats read from the bus
void readFloats(float* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(float));
}

// Fill a buffer with "count" floats read from the bus
void writeFloats(float* buffer, int count) {
  writeBytes((byte*) buffer, count * sizeof(float), floatType);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
// this function can also be called at other times (say via events coming through Serial)
void requestEvent() {
  previousI2C = currentMillis;      // reset the comm timeout clock
  switch(commandMode) {
    case MODE_IDLE:
      break;
    case MODE_HELP:
      handleHelpRequest();
      break;
    case MODE_PING:
      handlePingRequest();
      break;
    case MODE_SLOW:
      handleSlowRequest();
      break;
    case MODE_FAST:
      handleFastRequest();
      break;
    case MODE_BLINK:
      handleBlinkRequest();
      break;
    // TODO - add mode handlers
    case MODE_DEFAULT:
    default:
      handleDefaultRequest();
  }
}

//================================
void receiveEvent(int howMany) { // handles i2c write event from master
  int bytesLeft = howMany;
  char c;

  previousI2C = currentMillis;      // reset the comm timeout clock
  if (bytesLeft > 0) {
    c = readByte();
    bytesLeft--;
  }

  switch (c) {
    case '?':
      commandMode = MODE_HELP;
      break;
    case 'P':
      commandMode = MODE_PING;
      break;
    case 'S':
      commandMode = MODE_SLOW;
      break;
    case 'F':
      commandMode = MODE_FAST;
      break;
    case 'B':
      commandMode = MODE_BLINK;
      handleBlinkReceive(bytesLeft);
      bytesLeft-=4;
      break;
    // TODO - recognize additional modes
    case '\0':
    default:
      commandMode = MODE_DEFAULT;
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
// TODO - add menu items
    Serial.println("   \\0:  (or anything unhandled) Read unsingned int counter");
    Serial.println("    ?:  Show this help (otherwise act like \\0)");
    commandMode = MODE_IDLE;  // This only makes sense in serialMode
  }
  else // move on - nothing to see here
    handleDefaultRequest();
}

void handleDefaultRequest() {
  writeBytes((void*)&counter, sizeof(unsigned int), shortType);
  counter++;
}

void handlePingRequest() {
  writeBytes((void*)I2C_ADDRESS, 1, byteType);
}

void handleSlowRequest() {
  blinkInterval = 1500;
  writeBytes((void*)"SLOW", 4, textType);
}

void handleFastRequest() {
  blinkInterval = 250;
  writeBytes((void*)"FAST", 4, textType);
}

void handleBlinkRequest() {
  writeBytes((void*)blinkInterval, 4, textType);
}

void handleBlinkReceive(int howMany) {
  long interval;

  if (howMany == 4) {
    readLongs(&interval, 1);
  } else {
    // shouldn't get here
    interval = 0;
  }

  if(serialMode) {
    Serial.print("Interval now ");
    Serial.println(interval);
  }

  if (interval < 0)
    interval = 2000;
  blinkInterval = interval;
}

// TODO - write handlers
