#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN 18

#define NUM_LEDS 3

#define BRIGHTNESS 50

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);

byte neopix_gamma[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

//PCA9633 led layout
// PWM Register LEDOUT Address 08H
//     3  2  1  0
//    00 01 00 01 = Red
//    00 00 01 01 = Green
//    00 01 01 00 = Blue
//
//LED driver LEDOUT state
//    10 10 10 10 = 0xAA
#include <Wire.h>

//Register address values for projector
#define PCA9633_TURN_ON 0x00  // Configure PCA9633 for Mode 1
#define PCA9633_LIDAR 0x02
#define PCA9633_RED 0x03
#define PCA9633_GREEN 0x04
#define PCA9633_BLUE 0x05
#define PCA9633_PWM_ON 0x08

byte g_i2cAddress = 0;
// TODO this could be more dynamic
#define MAX_I2C_ADDRESSES 24
byte g_i2cAddresses[MAX_I2C_ADDRESSES] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Only call from i2c master
void I2CScan(boolean printOut = true) {
  byte error, address;
  int i, nDevices;

  if (printOut)
    Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    if (address == 41) continue;
    
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

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif

  Serial.begin(115200);
  
  // End of trinket special code
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  //  PCA9633
  Wire.begin();
  I2CScan();
  
  // RESET of MODE1 register to 0 and turn PCA9633 on
  PCA9633_WriteRegisterAll(PCA9633_TURN_ON, 0x00);
  LEDOUT_ALL(0, 0, 255); //All Off
  delay(2000);
}

void loop() {
  // Some example procedures showing how to display to the pixels:
  Serial.println("ColorWipes");
  colorWipe(strip.Color(255, 0, 0), 50); // Red
  LEDOUT_ALL(255, 0, 0);
  delay(500);
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  LEDOUT_ALL(0, 255, 0);
  delay(500);
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  LEDOUT_ALL(0, 0, 255);
  delay(500);
  colorWipe(strip.Color(0, 0, 0, 255), 50); // White
  LEDOUT_ALL(255, 255, 255);
  delay(500);

  Serial.println("WhiteOverRainbow");
  whiteOverRainbow(20, 75, 5);

  pulseWhite(5);

  // fullWhite();
  // delay(2000);

  Serial.println("RainbowFade2White");
  rainbowFade2White(3, 3, 1);

  Serial.println("Done");
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void pulseWhite(uint8_t wait) {
  for (int j = 0; j < 256 ; j++) {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 0, neopix_gamma[j] ) );
      LEDOUT_ALL(1 * j, 1 * j, 1 * j);
    }
    delay(wait);
    strip.show();
  }

  for (int j = 255; j >= 0 ; j--) {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 0, neopix_gamma[j] ) );
      LEDOUT_ALL(1 * j, 1 * j, 1 * j);
    }
    delay(wait);
    strip.show();
  }
}


void rainbowFade2White(uint8_t wait, int rainbowLoops, int whiteLoops) {
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;

  for (int k = 0 ; k < rainbowLoops ; k ++) {

    for (int j = 0; j < 256; j++) { // 5 cycles of all colors on wheel

      for (int i = 0; i < strip.numPixels(); i++) {

        wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);

        redVal = red(wheelVal) * float(fadeVal / fadeMax);
        greenVal = green(wheelVal) * float(fadeVal / fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal / fadeMax);

        strip.setPixelColor( i, strip.Color( redVal, greenVal, blueVal ) );
        LEDOUT_ALL(redVal, greenVal, blueVal);

      }

      //First loop, fade in!
      if (k == 0 && fadeVal < fadeMax - 1) {
        fadeVal++;
      }

      //Last loop, fade out!
      else if (k == rainbowLoops - 1 && j > 255 - fadeMax ) {
        fadeVal--;
      }

      strip.show();
      delay(wait);
    }

  }



  delay(500);


  for (int k = 0 ; k < whiteLoops ; k ++) {

    for (int j = 0; j < 256 ; j++) {

      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, neopix_gamma[j] ) );
        LEDOUT_ALL(1 * j, 1 * j, 1 * j);
      }
      strip.show();
    }

    delay(2000);
    for (int j = 255; j >= 0 ; j--) {

      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, neopix_gamma[j] ) );
        LEDOUT_ALL(1 * j, 1 * j, 1 * j);
      }
      strip.show();
    }
  }

  delay(500);


}

void whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) {

  if (whiteLength >= strip.numPixels()) whiteLength = strip.numPixels() - 1;

  int head = whiteLength - 1;
  int tail = 0;

  int loops = 3;
  int loopNum = 0;

  static unsigned long lastTime = 0;


  while (true) {
    for (int j = 0; j < 256; j++) {
      for (uint16_t i = 0; i < strip.numPixels(); i++) {
        if ((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) ) {
          strip.setPixelColor(i, strip.Color(0, 0, 0, 255 ) );
          LEDOUT_ALL(255, 255, 255);
        }
        else {
          strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }

      }

      if (millis() - lastTime > whiteSpeed) {
        head++;
        tail++;
        if (head == strip.numPixels()) {
          loopNum++;
        }
        lastTime = millis();
      }

      if (loopNum == loops) return;

      head %= strip.numPixels();
      tail %= strip.numPixels();
      strip.show();
      delay(wait);
    }
  }

}
void fullWhite() {

  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 255 ) );
  }
  LEDOUT_ALL(255, 255, 255);
  strip.show();
}


// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}

uint8_t red(uint32_t c) {
  return (c >> 16);
}
uint8_t green(uint32_t c) {
  return (c >> 8);
}
uint8_t blue(uint32_t c) {
  return (c);
}

void LEDOUT_ALL(int r, int g, int b) {
    for (int i = 0, index = 0; i < MAX_I2C_ADDRESSES ; i++) {
      if (g_i2cAddresses[i] != 0) LEDOUT(g_i2cAddresses[i], r, g, b);
    }
}

//Configure PCA9633 LED registers and output values
//usage. (PCA9633 address, RGB value)
void LEDOUT(int addr, int r, int g, int b)
{
  //  int r, g, b;

  // Set LDRx's of LEDOUT Reg. to 10b for RGB LED, it enables PWM
  // Set LDRx of LEDOUT Reg. to 11b for D2 LED, it enables PWM and GRPPWM control
  int ledout = 0xAA;

  //Split RGB value into r, g, b element value
  //  b = read_nibble(hexval, 0);
  //  g = read_nibble(hexval, 1);
  //  r = read_nibble(hexval, 2);

  //Invert RGB values for MBS Projector configuration
  //  r = r ^ 0xFF;
  //  g = g ^ 0xFF;
  //  b = b ^ 0xFF;

//  //XOR LEDOUT register address for selected color red
//  if (r == 255)
//    ledout = ledout ^ 0x0C;
//
//  //XOR LEDOUT register address for selected color green
//  if (g == 255)
//    ledout = ledout ^ 0x30;
//
//  //XOR LEDOUT register address for selected color blue
//  if (b == 255)
//    ledout = ledout ^ 0xC0;

  //Set PWM register for each color
  PCA9633_WriteRegister(addr, PCA9633_RED, r);
//  Serial.print("Red = ");
//  Serial.println(r, HEX);
  PCA9633_WriteRegister(addr, PCA9633_GREEN, g);
//  Serial.print("Green = ");
//  Serial.println(g, HEX);
  PCA9633_WriteRegister(addr, PCA9633_BLUE, b);
//  Serial.print("Blue = ");
//  Serial.println(b, HEX);

  //Set LED driver output state
  PCA9633_WriteRegister(addr, PCA9633_PWM_ON, ledout);
}

//Read hex element value
uint8_t read_nibble(uint32_t data, uint8_t ix)
{
  return ((data >> (ix * 8)) & 0xff);
}

//Convert a hex ascii string to a hex number.
long htol(char *p)
{
  /*
     Look for 'x' as second character as in '0x' format
  */
  if ((p[1] == 'x')  || (p[1] == 'X'))
    return (strtol(&p[2], (char **)0, 16));
  else
    return (strtol(p, (char **)0, 16));
}

void PCA9633_WriteRegisterAll(byte reg, byte data) {
    for (int i = 0, index = 0; i < MAX_I2C_ADDRESSES ; i++) {
      if (g_i2cAddresses[i] != 0) PCA9633_WriteRegister(g_i2cAddresses[i], reg, data);
    }
}

//Write data at selected address and register
void PCA9633_WriteRegister(byte addr, byte reg, byte data)
{
  //Begin all transmission at address
  Wire.beginTransmission(addr);

  //Write at register data
  Wire.write(reg);
  Wire.write(data);

  //Stop all transmission
  Wire.endTransmission();
}

