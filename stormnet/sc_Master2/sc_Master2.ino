#include <Wire.h>
#define MOTOR_SLAVE 8
#define AUDIO_SLAVE 9
#define LIGHTS_SLAVE 10

int CmdByte = 0;         // incoming serial byte

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600); // start serial port at 9600 bps and wait for port to open
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  
  Serial.println();
  Serial.println("SmartCart Master Controller Diagnostic System");
  Serial.println("Hit '?' for Help");
  Serial.print("==> ");
}

void loop() { //main user command loop
   if (Serial.available() > 0) {
     CmdByte = Serial.read(); // get user command
     Serial.println(char(CmdByte));
     switch(CmdByte){ //act on command
    
       case 'm': //motor slave command menu    
         display_motor_slave_help();
         CmdByte=0;
         while(CmdByte!='e'){ //stay in submenu until exit command ('e') is given
           if (Serial.available() > 0) { //process letter comman
             CmdByte = Serial.read(); //get user command
             Serial.println(char(CmdByte)); //echo the typed user command to terminal
             motor_slave_commands(CmdByte); //execute the command
             if(CmdByte=='e') display_master_help(); //go back to main menu and display help
           }            
         }         
       break;    
    
       case '?':  display_master_help(); break; //help     
       default:  Serial.println("Not a recognized command."); break;
     }
     
     Serial.println();
     Serial.print("==> ");
  }
}

//================================
void display_master_help(){
   Serial.println();
   Serial.println("===== SmartCart Main Menu Help =====");
   Serial.println("    m:  Motor Slave Command Menu"); 
   Serial.println("    ?:  Help"); 
   Serial.println();
}
//================================
void display_motor_slave_help(){
   Serial.println();
   Serial.println("   ===== Motor Slave Help =====");
   Serial.println("       d:  Motor Pot and PWM value display (left and right)"); 
   Serial.println("       F:  Flash heartbeat LED fast");
   Serial.println("       S:  Flash heartbeat LED slow");
   Serial.println("       e:  Exit to SmartCart Main Menu"); 
   Serial.println("       ?:  Help"); 
   Serial.println();
   Serial.print("   Motor Slave ==> ");
}
void motor_slave_commands(int MtrCmdByte){
   byte L_PotHI, L_PotLO, L_PWM, R_PotHI, R_PotLO, R_PWM;
   switch(MtrCmdByte){ //act on command
             case 'd': 
                Serial.println();
                Serial.println("   Motor Control Pot and PWM Command Display. Hit any key to exit...");
                Serial.println();
                Serial.println("      Left Pot\t\tLeft PWM\t\tRight Pot\tRight PWM");
                while(Serial.available()==0){
                  Wire.requestFrom(MOTOR_SLAVE, 6); //request 4 bytes from MOTOR_SLAVE
                  Serial.print("      ");
                  if (Wire.available()) L_PotHI = Wire.read(); else L_PotHI = 0xff; //set to 0xff if no data available
                  if (Wire.available()) L_PotLO = Wire.read(); else L_PotLO = 0xff; //set to 0xff if no data available
                  Serial.print(((L_PotHI<<8)| L_PotLO)&0x0000ffff); //print left pot value
                  Serial.print("\t\t");
                  if (Wire.available()) L_PWM = Wire.read(); else L_PWM = 0xff; //set to 0xff if no data available
                  Serial.print(L_PWM&0x0000ffff); //print left pwm value
                  Serial.print("\t\t\t");
                  if (Wire.available()) R_PotHI = Wire.read(); else R_PotHI = 0xff; //set to 0xff if no data available
                  if (Wire.available()) R_PotLO = Wire.read(); else R_PotLO = 0xff; //set to 0xff if no data available
                  Serial.print(((R_PotHI<<8)| R_PotLO)&0x0000ffff); //print right pot value
                  Serial.print("\t\t");
                  if (Wire.available()) R_PWM = Wire.read(); else R_PWM = 0xff; //set to 0xff if no data available
                  Serial.print(R_PWM&0x0000ffff); //print right pwm value
                  Serial.print("\r");
                }
                Serial.read(); //clear the char entered to break out of display mode
                Serial.println();
             break;  
             case 'e': break; //exit Motor Slave menu
             case 'F':
              Wire.beginTransmission(MOTOR_SLAVE);
              Wire.write("F");
              Wire.endTransmission();
              Serial.println("   Sent cmd to flash heartbeat LED fast on Motor Slave"); 
              break;
             case 'S':
              Wire.beginTransmission(MOTOR_SLAVE);
              Wire.write("S");
              Wire.endTransmission();
              Serial.println("   Sent cmd to flash heartbeat LED slow on Motor Slave"); 
              break;
             case '?':  display_motor_slave_help(); break; //help     
             default:  Serial.println("   Not a recognized command."); break;
   }
   Serial.println();  
   if((MtrCmdByte!='e')&& (MtrCmdByte!='?')) Serial.print("   Motor Slave ==> ");
}

