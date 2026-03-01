//Motor Slave for Stormgears SmartCart

#include <Wire.h>
#define MOTOR_SLAVE 8
#define AUDIO_SLAVE 9
#define LIGHTS_SLAVE 10

const int ledPin =  13;      // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;        // will store last time LED was updated
long blinkinterval = 100;           // interval at which to blink (milliseconds)
const unsigned long int i2cHeartbeatTimeout = 5000; // master must talk to slave within this number of milliseconds or LED will revert to fast pulse
unsigned long int currenti2c = 0; 
unsigned long int previousi2c = 0;

#include <Servo.h>
Servo leftservo, rightservo;  // create servo object to control a servo
#define LEFT_POT_AIN 0
#define RIGHT_POT_AIN 1

unsigned int CmdByte = 0;         // incoming serial byte
unsigned int Left_Pot,Left_PWM,Right_Pot,Right_PWM;

void setup() {
  pinMode(ledPin, OUTPUT); // set the digital pin as output
  leftservo.attach(9); //attach left servo on pin 9 to servo object
  rightservo.attach(10); //attach left servo on pin 10 to servo object
  Wire.begin(MOTOR_SLAVE);        // join i2c bus 
  Wire.onRequest(requestEvent);   // register event
  Wire.onReceive(receiveEvent);   // register event
  Serial.begin(9600); // start serial port at 9600 bps and wait for port to open
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  
  Serial.println();
  Serial.println("Motor Slave Controller Diagnostic System");
  Serial.println("Hit '?' for Help");
  Serial.print("Motor Slave ==> ");
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

//main motor control code here ===============
   
   if (Serial.available() > 0) { //diagnostic menu system starts here
     CmdByte = Serial.read(); // get user command
     Serial.println(char(CmdByte));
     switch(CmdByte){ //act on command
        case 'd': 
           Serial.println();
           Serial.println("   Motor Control Pot and PWM Command Display. Hit any key to exit...");
           Serial.println();
           Serial.println("      Left Pot\t\tLeft PWM\t\tRight Pot\tRight PWM");
           while(Serial.available()==0){
             Left_Pot=analogRead(LEFT_POT_AIN);Right_Pot=analogRead(RIGHT_POT_AIN);
             Left_PWM = map(Left_Pot, 0, 1023, 0, 180); Right_PWM = map(Right_Pot, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
   //=====>        leftservo.write(Left_PWM); rightservo.write(Right_PWM); delay(15);    // sets the servo position according to the scaled value
             Serial.print("      ");
             Serial.print(Left_Pot); //print left pot value
             Serial.print("\t\t");
             Serial.print(Left_PWM); //print left pwm value
             Serial.print("\t\t\t");
             Serial.print(Right_Pot); //print right pot value
             Serial.print("\t\t");
             Serial.print(Right_PWM); //print right pwm value
             Serial.print("\r");
           }
           Serial.read(); //clear the char entered to break out of display mode
           Serial.println();
        break;  
        case 'b':  //bumper test
          //bumper test and display code here
        break;
        case '?':  display_motor_slave_help(); break; //help     
        default:  Serial.println("   Not a recognized command."); break;
     }
     
     Serial.println();
     Serial.print("Motor Slave ==> ");
  }
}

//================================
void display_motor_slave_help(){
   Serial.println();
   Serial.println("===== Motor Slave Help =====");
   Serial.println("    d:  Motor Pot and PWM value display (left and right)"); 
   Serial.println("    b:  Bumper test");
   Serial.println("    ?:  Help"); 
}
//================================
void requestEvent(){ // handle i2c request from master
   blinkinterval=1000;
   previousi2c = currenti2c;
   Left_Pot=analogRead(LEFT_POT_AIN);Right_Pot=analogRead(RIGHT_POT_AIN);
   Left_PWM = map(Left_Pot, 0, 1023, 0, 180); Right_PWM = map(Right_Pot, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
   Wire.write((Left_Pot>>8)&0x0000ffff);  // left pot high byte
   Wire.write((Left_Pot)&0x0000ffff);  // left pot low byte
   Wire.write((Left_PWM)&0x0000ffff);  // left pwm byte
   Wire.write((Right_Pot>>8)&0x0000ffff);  // right pot high byte
   Wire.write((Right_Pot)&0x0000ffff);  // right pot low byte
   Wire.write((Right_PWM)&0x0000ffff);  // right pwm byte
}
//================================
void receiveEvent(int howMany) { // handles i2c write event from master
  blinkinterval=1000;
  previousi2c = currenti2c;
  while (Wire.available()) { // loop until no more bytes available
    char c = Wire.read(); // receive byte as a character
    switch(c) { //act on command
    case 'F': //flash heartbeat LED fast
      blinkinterval=250;
      while (Wire.available()) c=Wire.read(); //clean out i2c command buffer
      break;
    case 'S': //flash heartbeat LED slow  
      blinkinterval=1000;
      while (Wire.available()) c=Wire.read(); //clean out i2c command buffer
      break;
    default:
      while (Wire.available()) c=Wire.read(); //clean out i2c command buffer
      break;
    }

    }
  }
 
