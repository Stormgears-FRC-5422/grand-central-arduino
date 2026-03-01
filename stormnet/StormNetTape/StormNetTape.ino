
#define ledPin 13
#define NUM_LINE_PINS 5 // Number of pins for the line sensor 
byte lineSensorPins[NUM_LINE_PINS] = {A6,A7,A8,A9,A10};
byte lineSensorVal[NUM_LINE_PINS] = {0,0,0,0,0};

void setup() {
  pinMode(ledPin, OUTPUT);          // set the digital pin as output

  for(int i = 0; i < NUM_LINE_PINS; i++) {
    pinMode(lineSensorPins[i], INPUT);
  }

  Serial.begin(115200);             // start serial port at 9600 bps and wait for port to open
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:

  for(int i = 0; i < NUM_LINE_PINS; i++) {
    lineSensorVal[i] = digitalRead(lineSensorPins[i]);
  }

  for(int i = 0; i < NUM_LINE_PINS; i++) {
    Serial.print(lineSensorVal[i]);
    Serial.print("  ");
  }
  Serial.println();

}
