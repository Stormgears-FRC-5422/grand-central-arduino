#include "storm_ota.h"
#include "storm_blink.h"
#include <ESP32Servo.h>
#include <driver/adc.h>

/*
  Analog Input


  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.


*/

// Use the ESP32C3 Dev Module
#define SERIAL_BAUDRATE 115200

// Sign
#define SUPER_FAST 250
#define FAST_BLINK 750
#define SLOW_BLINK 1500
#define FORWARD 1
#define REVERSE -1

// INPUT PINS
#define PotPin 4   // select the input pin for the analog potentiometer
#define IRDPin 6   // Digital output of IR sensor
#define IRAPin A5  // Analog output of IR sensor
// int pirPin = 1;    // Goes HIGH for 8 seconds when motion is detected
// int neoPixelPin = 3;  // Control NeoPixels on this pin

//  OUTPUT PINS
#define statusPin BLINK_PIN  // Output pin to display status
#define pwmPin 2             // Output pin to control motor speed and direction to Talon SR

#define NEUTRAL 95
// System variables
uint32_t currentMillis = millis();
// int irDVal = 0;
// int irAVal = 0;
// int IRState = 1;
// int LastIRState = 1;  // Remmber what the prior IR sensor state was
// int IRCounter = 0;
int Direction = 1;
// int lastPirState = 1;
// int pirState = 1;
// int stopGears = 0;
int lastStopGears = currentMillis;
int stopGearsDuration = 5000;
double sensorValue = 0;  // variable to store the value coming from the POT
double talonSpeed = 90;
double maxSpeed = 25;

// Global Variables
int gesture_isr_flag = 0;

uint32_t lastTick = currentMillis;  // Variable used to debounce the microswitch input
int debounceDelay = 500;            // 100 millisecond debounce delay
double changeDirectionDelay;
double lastChange = 0;

int direction = 1;

Servo gearTalon;  // create servo object to control a servo

int lastCounter = 0;

// Data table
typedef struct {
  double input_signal;
  double time_for_8_rotations;
} DataPoint;

// Table of measured data
DataPoint table[] = {
  {  95, 120.0 },
  { 100, 90.0 },
  { 105, 30.0 },
  { 110, 20.0 },
  { 115, 15.0 },
  { 120, 10.0 }
};

#define TABLE_SIZE (sizeof(table) / sizeof(table[0]))

// Linear interpolation function
double interpolate(double input_signal) {
  // Handle out-of-range cases
  if (input_signal <= table[0].input_signal) {
    return table[0].time_for_8_rotations;
  }
  if (input_signal >= table[TABLE_SIZE - 1].input_signal) {
    return table[TABLE_SIZE - 1].time_for_8_rotations;
  }

  // Find the interval
  for (int i = 0; i < TABLE_SIZE - 1; i++) {
    if (input_signal >= table[i].input_signal && input_signal <= table[i + 1].input_signal) {
      // Perform linear interpolation
      double x0 = table[i].input_signal;
      double y0 = table[i].time_for_8_rotations;
      double x1 = table[i + 1].input_signal;
      double y1 = table[i + 1].time_for_8_rotations;

      return y0 + (input_signal - x0) * (y1 - y0) / (x1 - x0);
    }
  }

  return -1;  // Should never reach here
}

// void irInterruptRoutine() {
//   if (currentMillis - lastTick > debounceDelay) {
//     IRCounter++;
//     lastTick = currentMillis;
//   }
// }

void sign_setup() {
  Serial.begin(SERIAL_BAUDRATE);  // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears PWM Motor Control");

  pinMode(IRDPin, INPUT_PULLUP);
  pinMode(statusPin, OUTPUT);
  gearTalon.attach(pwmPin);  // attaches the srvo on pin 3 to the servo object

  changeDirectionDelay = 1000000;  // just a big number
  // attachInterrupt(digitalPinToInterrupt(IRDPin), irInterruptRoutine, FALLING);
}

void sign_loop() {
  currentMillis = millis();

  if (currentMillis - lastChange > changeDirectionDelay * 1000) {
    Direction = -Direction;
    gearTalon.write(NEUTRAL);  // Don't change without stopping first
    Serial.print("Braking. Speed = ");
    Serial.print(talonSpeed);
    Serial.print(" changeDirectionDelay = ");
    Serial.println(changeDirectionDelay, 3);
    delay(stopGearsDuration);       // Don't change instantly. This pauses the code, which isn't great. We could just set a wait state. Let's just see if this works.
    lastChange = currentMillis + stopGearsDuration;
  }

  sensorValue = map(analogRead(PotPin), 4095, 0, 0, maxSpeed);  // scale it to use it with the servo (value between 0 and 90)
  talonSpeed = NEUTRAL - (Direction * sensorValue);
  changeDirectionDelay = interpolate(abs(talonSpeed - NEUTRAL) + NEUTRAL);  // the time period doesn't depend on direction.

  // TODO - need to make an async blink so this doesn't block
  // if (Direction == FORWARD) {
  //   blink(SLOW_BLINK);
  // } else {
  //   blink(FAST_BLINK);
  // }

  gearTalon.write(talonSpeed);
  digitalWrite(statusPin, Direction == FORWARD ? HIGH : LOW);
}

void setup() {
  // simple_blink_setup();
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Booting");

  storm_ota_setup();
  sign_setup();
}

void loop() {
  // simple_blink_loop();
  storm_ota_loop();
  sign_loop();
}
