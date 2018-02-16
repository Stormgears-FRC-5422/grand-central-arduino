#include <Wire.h>

// Command modes
// You can add mode modes here, but please don't remove these
const char MODE_IDLE = -1;        // Only for serial loop
const char MODE_DEFAULT = 0;      // send back a cycle of numbers
const char MODE_HELP = 1;         // Serial only (!) Display help
const char MODE_SLOW = 2;         // blink the LED slow.  Request reply with word SLOW
const char MODE_FAST = 3;         // blink the LED fast.  Request reply with word FAST
const char MODE_PING = 4;         // send back the word PING on request
const char MODE_BLINK = 5;        // you tell me how fast to blink. Expects milliseconds as a long

char g_i2cAddress = 0;
volatile char g_commandMode = 0;
volatile unsigned int g_counter = 0;           // global counter for default handler
volatile long g_blinkInterval = 100;           // interval at which to blink (milliseconds)

// false means I2C - start with false by default
// this should flip to true in loop() if serial.available()
// that's the best way I can figure out how to tell if the usb is attached
boolean serialMode = false;

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

void printBuiltInHelp() {
    Serial.println();
    Serial.println("===== Stormgears I2C Device Help =====");
    Serial.println("Pass arguments in [].  For example, B[100]");
    Serial.println("    ?:  Show this help (otherwise act like \\0)");
    Serial.println("   \\0:  (or anything unhandled) Read unsignned int counter");
    Serial.println("    P:  Ping - read I2C Address");
    Serial.println("    F:  Change LED to FAST flash. Read 'FAST'");
    Serial.println("    S:  Change LED to SLOW flash. Read 'SLOW'");
    Serial.println("    B:  Change LED flash rate directly - pass another long to say how fast (in milliseconds)");
}

void printData(byte* array, uint8_t array_size, dataType dType = rawType) {
  if (dType == textType) {
    Serial.println((char*)array);
  }
  else {
    Serial.print("[ ");
    for (int i = 0; i < array_size;) {
      switch (dType) {
        case shortType:
          Serial.print(*((short*)(array + i)));
          i+=sizeof(short);
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
          Serial.print(*(array + i));
          i+=sizeof(char);
          break;
      }
      if (i < array_size)
        Serial.print(" , ");
    }
    Serial.println(" ]");
  }
}

// Read data values from the serial port. This respects the pattern followed by printData() above -
// That is, it will read
// [ value, value, value ]
// into a buffer of three values. It tries to be forgiving of format difference.
// We'll use 0 for missing or otherwise invalid cases.
void scanData(byte* array, uint8_t array_size, dataType dType = rawType) {
  for (int i = 0; i < array_size;) {
    switch (dType) {
      case shortType:
        *((short*)(array + i)) = (short)Serial.parseInt();
        i+=sizeof(short);
        break;
      case longType:
        *((long*)(array + i)) = (long)Serial.parseInt();
        i+=sizeof(long);
        break;
      case floatType:
        *((float*)(array + i)) = (float)Serial.parseFloat();
        i+=sizeof(float);
        break;
      case doubleType:
        *((double*)(array + i)) = (double)Serial.parseFloat();
        i+=sizeof(double);
        break;
      case byteType:
        *((byte*)(array + i)) = (byte)Serial.parseInt();
        i+=sizeof(byte);
        break;
      case rawType:
      default:
        *((byte*)(array + i)) = (byte)Serial.read();
        i+=sizeof(char);
        break;
    }
  }
  // for good measure:
  Serial.print("read ");
  printData(array, array_size, dType);
  Serial.readString(); // take care of any straggling character. ']', etc.
}
// Helper to write to either the I2C Wire or Serial
void writeBytes(void* buffer, byte count, dataType dType = rawType, boolean serialMode = false) {
  if (serialMode) {
    printData( (byte*)buffer, count, dType );
  }
  Wire.write((byte*)buffer, count);
}

// TODO Not sure what to do with this if both I2C and Serial are on
boolean readAvailable() {
  if (serialMode) return Serial.available();
  else return Wire.available();
}

// ultimately dealing with single byte reads.
// this could be optimized a bit, but this is pretty clear.
byte readByte() {
  if (serialMode) {
    while (!Serial.available()) {;}  // wait for the character to show up
    byte c = Serial.read();
    Serial.print("Received ");
    Serial.println(c, HEX);
    return c;
  }
  else return (byte)Wire.read();
}

// Fill a buffer with "count" bytes read from the bus
void readBytes(byte* buffer, int count, dataType dType = rawType) {
  if (serialMode) {
    scanData(buffer, count, dType);
  }
  else {
    for (int i=0; i<count; i++)
      buffer[i] = readByte();
  }
}

// Fill a buffer with "count" shorts read from the bus
void readShorts(short* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(short), shortType);
}

// Fill a buffer with "count" shorts read from the bus
void writeShorts(short* buffer, int count, boolean serialMode = false) {
  writeBytes((byte*) buffer, count * sizeof(short), shortType, serialMode);
}

// Fill a buffer with "count" longs read from the bus
void readLongs(long* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(long), longType);
}

// Fill a buffer with "count" longs read from the bus
void writeLongs(long* buffer, int count, boolean serialMode = false) {
  writeBytes((byte*) buffer, count * sizeof(long), longType, serialMode);
}

// Fill a buffer with "count" floats read from the bus
void readFloats(float* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(float), floatType);
}

// Fill a buffer with "count" floats read from the bus
void writeFloats(float* buffer, int count, boolean serialMode = false) {
  writeBytes((byte*) buffer, count * sizeof(float), floatType, serialMode);
}

// Fill a buffer with "count" doubles read from the bus
void readDouble(double* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(double), doubleType);
}

// Fill a buffer with "count" doubles read from the bus
void writeDoubles(double* buffer, int count, boolean serialMode = false) {
  writeBytes((byte*) buffer, count * sizeof(double), doubleType, serialMode);
}

// Handlers
void handleDefaultRequest() {
  writeBytes((void*)&g_counter, sizeof(unsigned int), shortType, serialMode);
  g_counter++;
}

void handlePingRequest() {
  writeBytes((void*)&g_i2cAddress, 1, byteType, serialMode);
}

void handleSlowRequest() {
  g_blinkInterval = 1500;
  writeBytes((void*)"SLOW", 4, textType, serialMode);
}

void handleFastRequest() {
  g_blinkInterval = 250;
  writeBytes((void*)"FAST", 4, textType, serialMode);
}

void handleBlinkRequest() {
  writeBytes((void*)&g_blinkInterval, 4, longType, serialMode);
}

void handleBlinkReceive() {
  long interval;

  readLongs(&interval, 1);

  if(serialMode) {
    Serial.print("Interval now ");
    Serial.println(interval);
  }

  if (interval < 0)
    interval = 2000;
  g_blinkInterval = interval;
}

void receiveBuiltIn(char c) {
  switch(c) {
    case 'P':
      g_commandMode = MODE_PING;
      break;
    case 'S':
      g_commandMode = MODE_SLOW;
      break;
    case 'F':
      g_commandMode = MODE_FAST;
      break;
    case 'B':
      g_commandMode = MODE_BLINK;
      handleBlinkReceive();
      break;
    // TODO - recognize additional modes
    case '\0':
    default:
      g_commandMode = MODE_DEFAULT;
  }
}

void requestBuiltIn() {
  switch(g_commandMode) {
    case MODE_IDLE:
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
