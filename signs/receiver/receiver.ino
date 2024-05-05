#include <RH_ASK.h>
#include "signs.h"

#define RADIO_DELAY_MS 5000

#define LED_COUNT 5
#define LED_PIN 6

uint8_t buflen;
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
String str_out;

bool received = false;
bool toggle = false;
int i = 0;

//Only one pin is used - we don't want the other to confuse the library, so make it different
#define RX_PIN 12
#define TX_PIN 11

RH_ASK driver(2000, RX_PIN, TX_PIN);
#define RADIO_RETRY_DELAY_MS 250

// set to false when actually trying to receive
#define DEBUG false

void setup() {
  Serial.begin(9600); // debugging
  Serial.println("Hello, World! I'm a receiver");

  pinMode(LED_ADDON_RED_PIN, OUTPUT);
  pinMode(LED_ADDON_BLUE_PIN, OUTPUT);
  setup_led_write_pins();

  if (!driver.init()) {
    Serial.println("init failed");
  } else {
    Serial.println("initialized");
  }
}

void loop() {
  digitalWrite(LED_ADDON_RED_PIN, toggle ? HIGH : LOW);
  digitalWrite(LED_ADDON_BLUE_PIN, toggle ? LOW : HIGH);

  // Have to reset this on each iteration - this is an input / output parameter
  // at input time this is the size of the buffer we are willing to fill, then it is 
  // reset to the size of the data block actually returned
  buflen = sizeof(buf);
  received = false;

  Serial.print(i++);
  Serial.print(": looping...");
  if (DEBUG) {
    // pretend we are receiving at the transmit rate
    delay(RADIO_DELAY_MS);
    received = true;
    buf[0] = '0' + (i % 4);
    buf[1] = 0;
    buflen = strlen(buf);
    str_out = String((char*)buf);
  }
  else if (driver.recv(buf, &buflen)) {
    // 0 byte messages are possible, but we want to ignore them
    if (buflen > 0) {
      received = true;
      // make sure buffer is null-terminated
      buf[buflen] = 0;
      str_out = String((char*)buf);
    } else {
      Serial.print(" empty");
    }
  } else {
   delay(RADIO_RETRY_DELAY_MS);
  }

  // What to do here needs to be in sync with what gets sent
  if (received) {
    Serial.print(buflen);
    Serial.print(" message bytes: ");
    Serial.print(str_out);
    if (buflen > 0) {
      Serial.print(", ");
      Serial.print(str_out[0] - '0');
      write_led_bits(str_out[0] - '0');
    }
    toggle = !toggle;
  }
 Serial.println("");
}
