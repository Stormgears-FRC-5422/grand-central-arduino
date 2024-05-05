// Should make a library here, but use these across all four files)
#define LED_ADDON_RED_PIN 7
#define LED_ADDON_BLUE_PIN 8
#define LED_STRING_PIN 13

#define NUMBER_OF_BITS 4

#define BIT0_LED_PIN 5
#define BIT1_LED_PIN 4
#define BIT2_LED_PIN 3
#define BIT3_LED_PIN 2

#define BIT0_RADIO_PIN 2
#define BIT1_RADIO_PIN 3
#define BIT2_RADIO_PIN 4
#define BIT3_RADIO_PIN 5

#define BIT0 0b0001
#define BIT1 0b0010
#define BIT2 0b0100
#define BIT3 0b1000

setup_led_read_pins() {
  pinMode(BIT0_LED_PIN, INPUT);
  pinMode(BIT1_LED_PIN, INPUT);
  pinMode(BIT2_LED_PIN, INPUT);
  pinMode(BIT3_LED_PIN, INPUT);
}

setup_led_write_pins() {
  pinMode(BIT0_RADIO_PIN, OUTPUT);
  pinMode(BIT1_RADIO_PIN, OUTPUT);
  pinMode(BIT2_RADIO_PIN, OUTPUT);
  pinMode(BIT3_RADIO_PIN, OUTPUT);
}

void write_led_bits(int value) {

  digitalWrite(BIT0_RADIO_PIN, (value & BIT0) ? HIGH : LOW);
  digitalWrite(BIT1_RADIO_PIN, (value & BIT1) ? HIGH : LOW);
  digitalWrite(BIT2_RADIO_PIN, (value & BIT2) ? HIGH : LOW);
  digitalWrite(BIT3_RADIO_PIN, (value & BIT3) ? HIGH : LOW);
}

int read_led_bits() {
  int value = 0;

  // Add one bit at a time
  value += digitalRead(BIT0_LED_PIN) << 0;
  value += digitalRead(BIT1_LED_PIN) << 1;
  value += digitalRead(BIT2_LED_PIN) << 2;
  value += digitalRead(BIT3_LED_PIN) << 3;

  return value;
}

#ifdef ADAFRUIT_NEOPIXEL_H

void colorWipe(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void chase(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t wait);
void chaseRainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos);


void colorWipe(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, r, g, b, w);
  }
}

void chase(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t wait) {
  for (int j = 0; j < 10; j++) {
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, r, g, b, w);
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);
      }
    }
  }
}

void chaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel((i + j) % 255));
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);
      }
    }
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

#endif