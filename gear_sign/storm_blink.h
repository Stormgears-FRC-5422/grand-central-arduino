#define BLINK_PIN 0
#define BLINK_INTERVAL 500

void blink(int interval) {
  digitalWrite(BLINK_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(interval);                      // wait for a second
  digitalWrite(BLINK_PIN, LOW);   // turn the LED off by making the voltage LOW
  delay(interval);    
}

void simple_blink_setup() {
  pinMode(BLINK_PIN, OUTPUT);
}

void simple_blink_loop() {
  blink(BLINK_INTERVAL);
}

