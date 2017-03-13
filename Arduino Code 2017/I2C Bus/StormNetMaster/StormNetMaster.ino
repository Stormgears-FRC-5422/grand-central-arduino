#include <Wire.h>
const int I2C_SLAVE_ADDRESS=8;

int CmdByte = 0;         // incoming serial byte
byte lastCommand = -1;

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
//  slave_commands('U');
//  delay(1000);
   if (Serial.available() > 0) {
     CmdByte = Serial.read(); // get user command
     Serial.println(char(CmdByte));
     switch(CmdByte){ //act on command
       case 's': //motor slave command menu
         display_slave_help();
         CmdByte=0;
         while(CmdByte!='e') { //stay in submenu until exit command ('e') is given
           if (Serial.available() > 0) {             //process letter comman
             CmdByte = Serial.read();                //get user command
             if(CmdByte=='e') display_master_help(); //go back to main menu and display help
             else slave_commands(CmdByte);           //execute the command
           }
         }
         break;
       case '?':
         display_master_help();
         break; //help
       default:
         Serial.println("Not a recognized command.");
         break;
     }

     Serial.println();
     Serial.print("==> ");
  }
}

//================================
void display_master_help(){
   Serial.println();
   Serial.println("===== SmartCart Main Menu Help =====");
   Serial.println("    s:  Slave Command Menu");
   Serial.println("    ?:  Help");
   Serial.println();
}

//================================
void display_slave_help(){
   Serial.println();
   Serial.println("===== Stormgears I2C Device Help =====");
   Serial.println("    e:  exit to earlier menu");
   Serial.println("    P:  Ping - read I2C Address");
   Serial.println("    F:  Change LED to FAST flash. Read 'FAST'");
   Serial.println("    S:  Change LED to SLOW flash. Read 'SLOW'");
   Serial.println("    B:  Change LED flash rate directly - pass another long to say how fast (in milliseconds)");
   Serial.println("   \\0:  (or anything unhandled, really) Read unsigned int counter");
   Serial.println();
   Serial.print("> ");
}

void slave_commands(char c){
  Serial.print("sending command ");
  Serial.println(c);
  int tmpValue;  // as needed

  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
    Wire.write(c);
    // Post any arguments - not all commands have them
    switch (c) {
      case 'B':
        // Get the bytes to send from the serial port - we need 4 here
        Serial.readBytes((char*)&tmpValue, 4);
        Serial.print("sending ");
        printData((byte*)&tmpValue, 4);
        Wire.write((char*)&tmpValue, 4);
        break;
      default:
        //Nothing more to send
        ;
    }
  Wire.endTransmission(I2C_SLAVE_ADDRESS);

  // Read the reply.
  switch (c) {
    case 'P':
      readIt(1, true);
      break;
    case 'S':
      readIt(4, false);
      break;
    case 'F':
      readIt(4, false);
      break;
    case 'U':
      readIt(4, true);
      break;
    case '\0':
    default:
      readIt(sizeof(unsigned int), true);
  }

  Serial.print("> ");
}

void readIt(byte count, boolean binary) {
  byte* buffer = new byte[count + 1];
  buffer[count] = 0;
  byte b;

  Wire.requestFrom(I2C_SLAVE_ADDRESS, (int)count);
  for (int i=0; i<count; i++) {
    if (Wire.available()) {
      b = Wire.read();    // receive a byte
      buffer[i] = b;
    }
  }
  if (binary) printData((byte*)buffer, count);
  else Serial.println((char*)buffer);

  delete buffer;
}

void printData(byte* array, uint8_t array_size) {
  Serial.print("[");
  for (int i = 0; i < array_size; i++) {
    Serial.print("0x");
    Serial.print(array[i],HEX);
    if (i != array_size - 1) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}
