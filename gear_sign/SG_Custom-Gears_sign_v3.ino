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

#include <ESP32_Servo.h>
#include <SparkFun_APDS9960.h>  // Gesture sensor library


Servo gearservo;  // create servo object to control a servo


// INPUT PINS

int PotPin = 4;     // select the input pin for the analog potentiometer
int IRDPin = 25;    // Digital output of IR sensor
int IRAPin = 26;    // Analog output of IR sensor
int touchPin = 27;  // Capacitive touch sensor pin
int neoPixelPin = 5;  // Control NeoPixels on this pin
int pirPin = 18;    // Goes HIGH for 8 seconds when motion is detected
int gestureIntPin = 23;  // The ADPS-9960 will pull this low when it senses a gesture.  Read the gesture value via I2C

//  OUTPUT PINS
int pwmPin = 16;  // Output pin to control motor speed and direction to Talon SR

// System variables
uint32_t currentMillis = millis();
int sensorValue = 0;  // variable to store the value coming from the POT
int IRState = 1;
int LastIRState = 1;  // Remeber what the prior IR sensor state was
int IRCounter = 0;
int Direction = 1;
int lastPirState = 1;
int pirState = 1;
int stopGears = 0;
int lastStopGears = currentMillis;
int stopGearsDuration = 5000;
int talonSpeed = 90;

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int gesture_isr_flag = 0;

uint32_t lastTick = currentMillis;  // Variable used to debounce the microswitch input
int debounceDelay = 3000; // 100 millisecond debounce delay


void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(IRDPin, INPUT_PULLUP);
  pinMode(pirPin, INPUT_PULLUP);
  gearservo.attach(pwmPin);  // attaches the servo on pin 3 to the servo object
  Serial.begin(115200);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
  Serial.println("Stormgears PWM Motor Control");
  gearservo.write(talonSpeed);

  // Set gesture interrupt pin as input
  pinMode(gestureIntPin, INPUT_PULLUP);

  // Initialize gesture interrupt service routine
  attachInterrupt(digitalPinToInterrupt(gestureIntPin), gestureInterruptRoutine, FALLING);
  attachInterrupt(digitalPinToInterrupt(IRDPin), irInterruptRoutine, FALLING);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    Serial.println(F("Gesture sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }
}

void gestureInterruptRoutine() {
  gesture_isr_flag = 1;
}

void irInterruptRoutine() {
  if (currentMillis - lastTick > debounceDelay) {
    IRCounter++;
    lastTick = currentMillis;
  }
}

void handleGesture() {
  if ( apds.isGestureAvailable() ) {
    switch ( apds.readGesture() ) {
      case DIR_UP:
        Serial.println("UP");
        break;
      case DIR_DOWN:
        Serial.println("DOWN");
        break;
      case DIR_LEFT:
        Serial.println("LEFT");
        break;
      case DIR_RIGHT:
        Serial.println("RIGHT");
        break;
      case DIR_NEAR:
        Serial.println("NEAR");
        break;
      case DIR_FAR:
        Serial.println("FAR");
        break;
      default:
        Serial.println("NONE");
    }
  }
}

void loop() {
  currentMillis = millis();

  //Check if we received a gesture.  TODO:  Use the gestures to interract with the gears or NeoPixels
  if ( gesture_isr_flag == 1 ) {
    detachInterrupt(0);
    handleGesture();
    gesture_isr_flag = 0;
    attachInterrupt(digitalPinToInterrupt(gestureIntPin), gestureInterruptRoutine, FALLING);
  }

  // read the value from the sensor:
  sensorValue = analogRead(PotPin);
  //  PotPin = 512;
  //  Serial.print("sensorValue: ");
  //  Serial.println(sensorValue);
  sensorValue = map(sensorValue, 4095, 0, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  if (stopGears == 1) {
    talonSpeed = 90;  // Stopped position for a servo...
  }
  else {
    talonSpeed = sensorValue / 2 * Direction + 90; // sets the servo position according to the scaled value
  }

  //  Serial.println(sensorValue / 2 * Direction + 90);
  //  IRState = digitalRead(IRDPin);
  pirState = digitalRead(pirPin);
  //  Serial.println(IRState);
  //  IRState = !IRState; // invert the signal for virtical display
  if (lastPirState != pirState) {
    Serial.print("PIR State Changed to: ");
    Serial.println(pirState);
    lastPirState = pirState;
  }
  if (IRCounter == 3) {              //this sets how many spins before it stops and changes direction
    Serial.println("Stop");
    talonSpeed = 90;
    stopGears = 1;
    lastStopGears = currentMillis;
    if (Direction == 1)
      Direction = -1;
    else
      Direction = 1;
    Serial.println(Direction);
    IRCounter = 0;
  }
  gearservo.write(talonSpeed); // sets the servo position according to the scaled value
  if (currentMillis - lastStopGears > stopGearsDuration)  {
    stopGears = 0;
  }
}



