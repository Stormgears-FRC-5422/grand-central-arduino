#include <Adafruit_NeoPixel.h>

// For now this needs to be defined before signs.h is included
#define LED_COUNT 10
#define LED_STRING_PIN 13
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_STRING_PIN, NEO_GRBW + NEO_KHZ800);

#include "signs.h"

int i = 0;
int oldValue = -1;
bool toggle = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Hello, World! I'm an LED Driver");

  pinMode(LED_ADDON_RED_PIN, OUTPUT);
  pinMode(LED_ADDON_BLUE_PIN, OUTPUT);

  setup_led_read_pins();
  strip.begin();
}

void loop () {
  int value = read_led_bits();

  digitalWrite(LED_ADDON_BLUE_PIN, toggle ? HIGH : LOW);
  digitalWrite(LED_ADDON_RED_PIN, toggle ? LOW : HIGH);

  if (value != oldValue) {
    // new pattern. Does it matter?
    Serial.print("Value read from bits = ");
    Serial.println(value);
    
    oldValue = value;
    toggle = !toggle;
  }

  int c = 128;
  colorWipe(BIT0 & value ? c : 0, BIT1 & value ? c : 0, value == 0 ? c : 0, 0);

  // switch(value % 4) {
  //   case 0:
  //     colorWipe(250, 0, 0, 0);
  //     break;
  //   case 1:
  //     colorWipe(0, 250, 0, 0);
  //     break;
  //   case 2:
  //     colorWipe(0, 0, 250, 0);
  //     break;
  //   case 3:
  //     colorWipe(250, 0, 250, 0);
  //     break;
  //   default:
  //     colorWipe(250, 250, 250, 0);
  // }
  
  strip.show();  
}
