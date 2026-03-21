// StormRebuilt.ino — 2026 season sensor client
//
// This file is the thin glue layer between StormNet infrastructure and the
// attached sensors. To add a sensor:
//   1. Create MySensor.h with setupMySensor(), loopMySensor(), writeMySensorData(wireMode)
//   2. #include it below
//   3. Replace the Device customization section at the bottom of this file

#include <Ethernet.h>
#include <EthernetUdp.h>

#include "StormNetCommon.h"
#include "MatrixLidar.h"

// ===== Network config =====
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
bool useNetwork = false;
IPAddress ip(10, 54, 22, 177);
IPAddress roborioIP(10, 54, 22, 2);
const int udpPort = 5423;

// ===== Timing =====
const int ledPin = 13;
int ledState = LOW;
unsigned long currentMillis = 0;
unsigned long previousBlink = 0;
unsigned long previousTransmit = 0;
unsigned long previousSerialDebug = 0;
const unsigned long transmitInterval = 10;        // ms between UDP packets
const unsigned long serialDebugInterval = 1000;   // ms between serial packet dumps in UDP mode
const unsigned long i2cHeartbeatTimeout = 15000;  // ms before LED goes to fast pulse
volatile unsigned long previousI2C = 0;


void setup() {
  previousI2C = millis();
  previousBlink = previousI2C;
  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Stormgears StormNet Diagnostic System");

  if (useNetwork) {
    Ethernet.begin(mac, ip);
    g_udpClient.begin(udpPort);
    Serial.print("Network:  enabled  local ");
    Serial.print(Ethernet.localIP());
    Serial.print("  target ");
    Serial.print(roborioIP);
    Serial.print(":");
    Serial.println(udpPort);
  } else {
    Serial.println("Network:  disabled (set useNetwork = true to enable)");
  }

  Serial.println("Hit '?' for help");

  setup_custom();

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Ready");
}


void loop() {
  currentMillis = millis();

  // --- Sensor updates ---
  loop_custom();

  // --- Talk mode ---
  if (Serial.available()) {
    g_talkMode = serialMode;
  } else {
    g_talkMode = useNetwork ? udpMode : serialMode;
  }

  // --- Heartbeat LED ---
  if ((currentMillis - previousI2C) > i2cHeartbeatTimeout)
    g_blinkInterval = 100;
  else
    g_blinkInterval = (g_blinkInterval == 100) ? 1000 : g_blinkInterval;

  boolean flipNow = ((currentMillis - previousBlink) >= g_blinkInterval);
  if (flipNow) {
    previousBlink = currentMillis;
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(ledPin, ledState);
  }

  // --- Serial input ---
  if (g_talkMode == serialMode && Serial.available() > 0) {
    receiveEvent(5);
    requestEvent();
  }

  // --- Transmit UDP ---
  if (useNetwork && (currentMillis - previousTransmit) > transmitInterval) {
    send_loop();
    previousTransmit = currentMillis;
  }

  // --- Serial output ---
  if ((currentMillis - previousSerialDebug) >= serialDebugInterval) {
    data_loop(serialMode);
    previousSerialDebug = currentMillis;
  }
}


// Wraps data_loop() in a UDP packet
void send_loop() {
  if (g_udpClient.beginPacket(roborioIP, udpPort)) {
    data_loop(udpMode);
    if (!g_udpClient.endPacket()) {
      Serial.write("Error ending packet!");
    }
  } else {
    Serial.write("Error beginning packet!");
  }
}


// Defines the packet layout — must match server-side expectations.
// Call printPacketLayout() (via '?' command) to see the current layout.
void data_loop(wireMode mode) {
  g_packetFieldCount = 0;
  data_loop_builtin(mode);
  data_loop_custom(mode);
}


void receiveEvent(int howMany) {
  char c = 0;
  previousI2C = currentMillis;

  if (howMany > 0) c = readByte();
  if (!receiveBuiltIn(c)) receive_custom(c);

  while (readAvailable()) c = readByte();
}


void requestEvent() {
  previousI2C = currentMillis;

  if (!request_custom()) handleBuiltInCommand(g_talkMode);
}


// ===== Device customization =====
// Replace this section to target a different sensor.
// Do not modify the infrastructure functions above.

const char MODE_MATRIX_LIDAR = FIRST_CUSTOM_MODE;

void setup_custom() {
  setupMatrixLidar();
}

void loop_custom() {
  loopMatrixLidar(); 
}

bool receive_custom(char c) {
  if (c == 'M') { g_commandMode = MODE_MATRIX_LIDAR; return true; }
  return false;
}

bool request_custom() {
  if (g_commandMode == MODE_MATRIX_LIDAR) {
    handleMatrixLidar();
    return true;
  }
  return false;
}

void data_loop_custom(wireMode mode) {
  g_packetFields[g_packetFieldCount++] = writeMatrixLidarData(mode);
}

void printCustomHelp() {
  Serial.println("    M:  Print matrix lidar values");
}
