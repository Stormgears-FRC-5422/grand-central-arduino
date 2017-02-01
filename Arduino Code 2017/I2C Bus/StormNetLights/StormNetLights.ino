//Motor Slave for Stormgears SmartCart

#include <Wire.h>
#define LIGHT_SLAVE 10  // light controller I2C Bus ID 10
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6
// Number of lights, address
Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, PIN, NEO_GRBW);

const int ledPin =  13;      // the number of the LED pin
const int enablePin = 7;  // Enable pin for ultraSonic sensor
int ledState = LOW;             // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;        // will store last time LED was updated
long blinkinterval = 100;           // interval at which to blink (milliseconds)
const unsigned long int i2cHeartbeatTimeout = 5000; // master must talk tho slave within this number of milliseconds or LED will revert to fast pulse
unsigned long int currenti2c = 0;
unsigned long int previousi2c = 0;
float usINPUT1;
float usINPUT2;
volatile byte* INPUT1FloatPtr;
volatile byte* INPUT2FloatPtr;

unsigned int CmdByte = 0;         // incoming serial byte
//unsigned int Left_Pot,Left_PWM,Right_Pot,Right_PWM;

void setup() {
  strip.begin();
  strip.setBrightness(16);
  
  strip.show(); // Initialize all pixels to 'off'
  pinMode(ledPin, OUTPUT); // set the digital pin as output
  Wire.begin(LIGHT_SLAVE);        // join i2c bus
  Wire.onRequest(requestEvent);   // register event
  Wire.onReceive(receiveEvent);   // register event
  Serial.begin(9600); // start serial port at 9600 bps and wait for port to open

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.println("Light Slave Controller Diagnostic System");
  Serial.println("Hit '?' for Help");
  Serial.print("Light Slave ==> ");
}

void loop() { //main user command loop

  //code to flash heartbeat LED =============
  currenti2c = currentMillis = millis();
  if (currenti2c - previousi2c >= i2cHeartbeatTimeout) blinkinterval = 100;
  if (currentMillis - previousMillis >= blinkinterval) {
    previousMillis = currentMillis;    // save the last time you blinked the LED
    if (ledState == LOW) {// if the LED is off turn it on and vice-versa
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);  // set the LED with the ledState of the variable
  }

  //main light control code here ===============

 //light mode loops=============================
// if (seizureON()) {
//  seizure(25);
// }

 

 //serial display code==========================
  if (Serial.available() > 0) { //diagnostic menu system starts here
    CmdByte = Serial.read(); // get user command
    Serial.println(char(CmdByte));
    switch (CmdByte) { //act on command
      case 's':
        Serial.println();
        Serial.println("   Light Controller Slave Command Display. Hit any key to exit...");
        Serial.println();
        Serial.print("   I'm Seizing   ");
        while (Serial.available() == 0) {
         seizure(25);
          Serial.print(".");
        }
        Serial.read(); //clear the char entered to break out of display mode
        Serial.println();
        break;
      case 'c': 
        Serial.println();
        Serial.println("   Light Controller Slave Command Display. Hit any key to exit...");
        Serial.println();
        Serial.print("   I'm Chasing    ");
        while (Serial.available() == 0) {
          chase();
          Serial.print(".");
        }
        break;
      case '?':  display_light_slave_help(); break; //help
      default:  Serial.println("   Not a recognized command."); break;
    }

    Serial.println();
    Serial.print("Light Slave ==> ");
  }
}


void printArray(float* array, uint8_t array_size) {
  Serial.print("[");
  for (int i = 0; i < array_size; i++) {
    Serial.print(array[i]);
    if (i != array_size - 1) {
      Serial.print(", ");
    }
  }
  Serial.print("]");
}

//================================
void display_light_slave_help() {
  Serial.println();
  Serial.println("===== Light Slave Help =====");
  Serial.println("    c:  Chase");
  Serial.println("    s:  Seizure");
  Serial.println("    ?:  Help");
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  byte* Data;
  INPUT1FloatPtr = (byte*) &usINPUT1;
  INPUT2FloatPtr = (byte*) &usINPUT2;
  Data[0] = INPUT1FloatPtr[0];
  Data[1] = INPUT1FloatPtr[1];
  Data[2] = INPUT1FloatPtr[2];
  Data[3] = INPUT1FloatPtr[3];
  Data[4] = INPUT2FloatPtr[0];
  Data[5] = INPUT2FloatPtr[1];
  Data[6] = INPUT2FloatPtr[2];
  Data[7] = INPUT2FloatPtr[3];
  Wire.write(Data, 8);
}

//================================
void oldrequestEvent() { // handle i2c request from master
  blinkinterval = 1000;
  previousi2c = currenti2c;

  float usSensorVal;

  //Wire.write((usSensorCM >> 8) & 0x0000ffff); // Distance high byte
  //Wire.write((usSensorCM) & 0x0000ffff); // Distance low byte
  //Wire.write((Left_Pot>>8)&0x0000ffff);  // left pot high byte
  //Wire.write((Left_Pot)&0x0000ffff);  // left pot low byte
  //Wire.write((Left_PWM)&0x0000ffff);  // left pwm byte
  //Wire.write((Right_Pot>>8)&0x0000ffff);  // right pot high byte
  //Wire.write((Right_Pot)&0x0000ffff);  // right pot low byte
  //Wire.write((Right_PWM)&0x0000ffff);  // right pwm byte
}
//================================
void receiveEvent(int howMany) { // handles i2c write event from master
  blinkinterval = 1000;
  previousi2c = currenti2c;

  while (Wire.available()) { // loop until no more bytes available
    unsigned int lightID = Wire.read();
    char command = Wire.read(); // receive byte as a character
    char c;
    unsigned int param1 = Wire.read();
    unsigned int param2 = Wire.read();
    
    switch (command) { //act on command
      case 'F': //flash heartbeat LED fast
        blinkinterval = 250;
        while (Wire.available()) c = Wire.read(); //clean out i2c command buffer
        break;
      case 'S': //flash heartbeat LED slow
        blinkinterval = 1000;
        while (Wire.available()) c = Wire.read(); //clean out i2c command buffer
        break;
      default:
        while (Wire.available()) c = Wire.read(); //clean out i2c command buffer
        break;
    }
  }
}

bool wait(int waitDelay) {
  int waitStart = millis();
  int waitNow = millis();
  while (((waitNow - waitStart) < waitDelay)) {
    //    modeCheck();
    waitNow = millis();
  }
  //  return(modeChanged);
}
//===============================
void chase() {
  int i = 0;
  uint32_t off = strip.Color(0, 0, 0, 0);
  uint32_t white = strip.Color(0, 0, 0, 255);

  for(i=0; i<10; i++) {
    if(i==0) {
      strip.setPixelColor(i, off);
      strip.setPixelColor(9, white);
      strip.show();
    }
    else {
      strip.setPixelColor(i, off);
      strip.setPixelColor(i-1, white);
      strip.show();
    }
    delay(50);
  }
  
}

//===============================

void seizure(int extremity) {
  uint32_t off = strip.Color(0, 0, 0, 0);
  uint32_t white = strip.Color(0, 0, 0, 255);
  uint32_t magenta = strip.Color(255, 0, 255);
  
  for(int i = 0; i < 10; i++) {
    strip.setPixelColor(i, off);
  } 
  strip.show();
  delay (extremity);
  for(int j = 0; j < 10; j++) {
    strip.setPixelColor(j, magenta);
  }
  strip.show();
  delay (extremity);
  
}

