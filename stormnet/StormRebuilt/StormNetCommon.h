#include <Wire.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ===== Built-in command modes =====
// You can add more modes in your sensor file, but please don't remove these
const char MODE_IDLE = -1;        // Only for serial loop
const char MODE_DEFAULT = 0;      // send back a cycle of numbers
const char MODE_HELP = 1;         // Serial only (!) Display help
const char MODE_SLOW = 2;         // blink the LED slow.  Request reply with word SLOW
const char MODE_FAST = 3;         // blink the LED fast.  Request reply with word FAST
const char MODE_PING = 4;         // send back the word PING on request
const char MODE_BLINK = 5;        // you tell me how fast to blink. Expects milliseconds as a long
const char MODE_TIMER = 6;        // return loop timing diagnostics
#define FIRST_CUSTOM_MODE 7       // sensor files should start add-on modes here

byte g_i2cAddress = 0;
// TODO this could be more dynamic
#define MAX_I2C_ADDRESSES 24
byte g_i2cAddresses[MAX_I2C_ADDRESSES] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#define WIRE_CLOCK 100000

volatile char g_commandMode = 0;
volatile unsigned int g_counter = 0;           // global counter for default handler
volatile long g_blinkInterval = 100;           // interval at which to blink (milliseconds)

EthernetClient g_ethernetClient;
EthernetUDP g_udpClient;

// ===== Talk mode =====
// default to I2C
// this should flip to serialMode in loop() if serial.available()
enum wireMode {
  I2CMode,
  ethernetMode,
  udpMode,
  serialMode
};

wireMode g_talkMode = I2CMode;

// ===== Packet field descriptor =====
// Returned by write*Data functions to self-describe the data they produce.
// data_loop() collects these into g_packetFields; printPacketLayout() displays them.
struct PacketField {
  const char* label;    // protocol letter, e.g. "P"
  const char* name;     // human-readable name, e.g. "Ping"
  int byteCount;        // bytes written to the channel
};

#define MAX_PACKET_FIELDS 16
PacketField g_packetFields[MAX_PACKET_FIELDS];
int g_packetFieldCount = 0;

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

// Only call from i2c master
void I2CScan(boolean printOut = true) {
  byte error, address;
  int i, nDevices;

  if (printOut)
    Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
//    if (address == 41) continue;
    
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      if (printOut) {
        if (nDevices == 0) Serial.print("I2C device found at address ");
//        if (address < 16)
//          Serial.print("0");
        Serial.print(address);
        Serial.print(", ");
      }

      g_i2cAddresses[nDevices] = address;
      nDevices++;
    }
    else if (printOut && error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (printOut) {
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("\ndone\n");    
  }

  for (int i = nDevices ; i < MAX_I2C_ADDRESSES ; i++) {
    g_i2cAddresses[i] = 0;
  }

}

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
    Serial.println("    ::  (colon) Return time details");
}

void printPacketLayout() {
  Serial.println("Packet layout:");
  int offset = 0;
  for (int i = 0; i < g_packetFieldCount; i++) {
    Serial.print("  ");
    Serial.print(g_packetFields[i].label);
    Serial.print("  ");
    Serial.print(g_packetFields[i].name);
    Serial.print("  ");
    Serial.print(g_packetFields[i].byteCount);
    Serial.print(" bytes  offset ");
    Serial.println(offset);
    offset += g_packetFields[i].byteCount;
  }
  Serial.print("Total: ");
  Serial.print(offset);
  Serial.println(" bytes");
}

// print friendly representation of data stream to serial port
// this is for debugging and display 
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
void writeBytes(void* buffer, byte count, dataType dType = rawType, wireMode mode = I2CMode) {
  switch(mode) {
    case serialMode:
      printData( (byte*)buffer, count, dType );
      break;
    case ethernetMode:
      g_ethernetClient.write((byte*)buffer, count);
      break;
    case udpMode:
      g_udpClient.write((byte*)buffer, count);
      break;
    default:
      Wire.write((byte*)buffer, count);
  }
}

// TODO Not sure what to do with this if both I2C and Serial are on
boolean readAvailable() {
  switch(g_talkMode) {
    case serialMode:
      return Serial.available();
      break;
    case ethernetMode:
      return g_ethernetClient.available();
      break;
    case udpMode :
      return g_udpClient.available();
      break;
    default:
      return Wire.available();
  }
}

// ultimately dealing with single byte reads.
// this could be optimized a bit, but this is pretty clear.
byte readByte() {
  byte c;
  switch(g_talkMode) {
    case serialMode:
      while (!Serial.available()) {;}  // wait for the character to show up
      c = Serial.read();
      Serial.print("Received ");
      Serial.println(c, HEX);
      return c;
    case ethernetMode:
      while (!g_ethernetClient.available()) {;}  // wait for the character to show up
      c = g_ethernetClient.read();
      // TODO remove debugging
      Serial.print("Received ");
      Serial.println(c, HEX);
      return c;
    case udpMode:
      // TODO - for this to really work, there is some setup around ParsePacket, etc. that needs to be worked out. Currently we are only writing, so no big deal
      // BUT THIS WONT WORK until this setup has happened
      while (!g_udpClient.available()) {;}  // wait for the character to show up
      c = g_udpClient.read();
      // TODO remove debugging
      Serial.print("Received ");
      Serial.println(c, HEX);
      return c;
    default:
      c = (byte)Wire.read();      
  }

  return c;
}

// Fill a buffer with "count" bytes read from the bus
void readBytes(byte* buffer, int count, dataType dType = rawType) {
  switch(g_talkMode) {
    case serialMode:
      scanData(buffer, count, dType);
      break;
    case ethernetMode: // fall through
    case udpMode:
    default:
      for (int i=0; i<count; i++)
        buffer[i] = readByte();
  }

}

// Fill a buffer with "count" shorts read from the bus
void readShorts(short* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(short), shortType);
}

// Fill a buffer with "count" shorts read from the bus
void writeShorts(short* buffer, int count, wireMode mode = I2CMode) {
    writeBytes((byte*) buffer, count * sizeof(short), shortType, mode);
}

// Fill a buffer with "count" longs read from the bus
void readLongs(long* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(long), longType);
}

// Fill a buffer with "count" longs read from the bus
void writeLongs(long* buffer, int count, wireMode mode = I2CMode) {
  writeBytes((byte*) buffer, count * sizeof(long), longType, mode);
}

// Fill a buffer with "count" floats read from the bus
void readFloats(float* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(float), floatType);
}

// Fill a buffer with "count" floats read from the bus
void writeFloats(float* buffer, int count, wireMode mode = I2CMode) {
  writeBytes((byte*) buffer, count * sizeof(float), floatType, mode);
}

// Fill a buffer with "count" doubles read from the bus
void readDoubles(double* buffer, int count) {
  readBytes((byte*) buffer, count * sizeof(double), doubleType);
}

// Fill a buffer with "count" doubles read from the bus
void writeDoubles(double* buffer, int count, wireMode mode = I2CMode) {
  writeBytes((byte*) buffer, count * sizeof(double), doubleType, mode);
}

// ===== Write functions =====
// Write current state to the specified channel. No state changes happen here —
// state is updated in the receive path (receiveBuiltIn). Each returns a PacketField
// descriptor; callers that don't need it can ignore the return value.

PacketField writeDefaultData(wireMode mode) {
  writeBytes((void*)&g_counter, sizeof(unsigned int), shortType, mode);
  g_counter++;
  return {"-", "Counter", (int)sizeof(unsigned int)};
}

PacketField writePingData(wireMode mode) {
  writeBytes((void*)&g_i2cAddress, 1, byteType, mode);
  return {"P", "Ping", 1};
}

PacketField writeFastData(wireMode mode) {
  writeBytes((void*)"FAST", 4, textType, mode);
  return {"F", "Fast", 4};
}

PacketField writeSlowData(wireMode mode) {
  writeBytes((void*)"SLOW", 4, textType, mode);
  return {"S", "Slow", 4};
}

PacketField writeBlinkData(wireMode mode) {
  writeBytes((void*)&g_blinkInterval, 4, longType, mode);
  return {"B", "Blink", 4};
}

// ===== Receive functions =====
// Process incoming commands and update Arduino state. No output.

bool receiveBuiltIn(char c) {
  switch(c) {
    case 'P':
      g_commandMode = MODE_PING;
      return true;
    case 'S':
      g_commandMode = MODE_SLOW;
      g_blinkInterval = 1500;
      return true;
    case 'F':
      g_commandMode = MODE_FAST;
      g_blinkInterval = 250;
      return true;
    case 'B': {
      g_commandMode = MODE_BLINK;
      long interval;
      readLongs(&interval, 1);
      if (g_talkMode == serialMode) {
        Serial.print("Interval now ");
        Serial.println(interval);
      }
      if (interval < 0) interval = 2000;
      g_blinkInterval = interval;
      return true;
    }
    case ':':
      g_commandMode = MODE_TIMER;
      return true;
    case '?':
      g_commandMode = MODE_HELP;
      return true;
    case '\0':
    default:
      g_commandMode = MODE_DEFAULT;
      return false;
  }
}

// ===== Timer (loop timing diagnostics) =====
// Tracks how long each main loop cycle takes. Useful as a built-in diagnostic.
unsigned long g_timerMillis = 0;
long g_timerResults[3] = {0, 0, 0};

PacketField writeTimerData(wireMode mode) {
  unsigned long now = millis();
  g_timerResults[0] = g_timerMillis;
  g_timerResults[1] = now;
  g_timerResults[2] = millis();
  g_timerMillis = g_timerResults[2];
  if (mode == serialMode) {
    Serial.print("timer: ");
  }
  writeLongs(g_timerResults, 3, mode);
  return {":", "Timer", 3 * (int)sizeof(long)};
}

// ===== User hooks =====
// Define all of these in your .ino's Device customization section.
void setup_custom();
void loop_custom();
bool receive_custom(char c);
bool request_custom();
void data_loop_custom(wireMode mode);
void printCustomHelp();

// ===== Built-in command dispatch =====
// Responds to the current commandMode. Called from requestEvent() in request-response mode.

void handleBuiltInCommand(wireMode mode) {
  switch(g_commandMode) {
    case MODE_IDLE:
      break;
    case MODE_HELP:
      if (mode == serialMode) {
        printBuiltInHelp();
        printCustomHelp();
        Serial.println();
        printPacketLayout();
        g_commandMode = MODE_IDLE;
      } else {
        writeDefaultData(mode);
      }
      break;
    case MODE_PING:
      writePingData(mode);
      break;
    case MODE_SLOW:
      writeSlowData(mode);
      break;
    case MODE_FAST:
      writeFastData(mode);
      break;
    case MODE_BLINK:
      writeBlinkData(mode);
      break;
    case MODE_TIMER:
      writeTimerData(mode);
      break;
    case MODE_DEFAULT:
    default:
      writeDefaultData(mode);
  }
}

// ===== Built-in data bundle =====
// Writes all built-in fields for a push packet. Call from data_loop() before sensor fields.
// Accumulates into g_packetFields[]; caller must reset g_packetFieldCount first.

void data_loop_builtin(wireMode mode) {
  g_packetFields[g_packetFieldCount++] = writePingData(mode);
  g_packetFields[g_packetFieldCount++] = writeBlinkData(mode);
  g_packetFields[g_packetFieldCount++] = writeTimerData(mode);
}
