#include <RH_ASK.h>
#include <SPI.h>
#include "signs.h"

#define RADIO_DELAY_MS 5000

//Only one pin is used - we don't want the other to confuse the library, so make it different
#define RX_PIN 10
#define TX_PIN 12

// default Tx pin is 12
RH_ASK driver(2000, RX_PIN, TX_PIN);

// set to false when actually trying to transmit
#define DEBUG false

char msg[8] = {'\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'};
int iteration = 0;
bool toggle = false;

void setup() {
  Serial.begin(9600); // debugging
  Serial.println("Hello, World! I'm a transmitter");

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

  msg[0] = '0' + (iteration++ % 4);
  msg[1] = '\x00';
  
  driver.send((uint8_t*)msg, strlen(msg));
  driver.waitPacketSent();
  Serial.print("Sent ");
  Serial.print(strlen(msg));
  Serial.print(" byte(s): ");
  Serial.println(msg);

  // Simple strategy here - use the first byte of msg to set the bits for the LEDs locally
  write_led_bits(msg[0]);

  // wait a bit to send again
  delay(RADIO_DELAY_MS);
  toggle = !toggle;
}
