//Register address values for node
#define PCA9633_TURN_ON 0x00  // Configure PCA9633 for Mode 1
#define PCA9633_LIDAR 0x02
#define PCA9633_RED 0x03
#define PCA9633_GREEN 0x04
#define PCA9633_BLUE 0x05
#define PCA9633_PWM_ON 0x08

// Set LDRx's of LEDOUT Reg. to 10b for RGB LED, it enables PWM
// Set LDRx of LEDOUT Reg. to 11b for D2 LED, it enables PWM and GRPPWM control
long g_ledout = 0xAA;  // 10 10 10 10  B G R Lidar

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

//Configure PCA9633 LED registers and output values
//usage. (PCA9633 address, RGB value)
void LEDOUT(int addr, int r, int g, int b)
{
//  g_ledout = 0xA9; // reset to 1010 1001 is PWM PWM PWM ON
  
//  //XOR LEDOUT register address for selected color red
//  //This turns the LED on completely for 255, rather than PWM
//  //uses stupid bit tricks to flip from 10 to 01
//  if (r == 255)
//    g_ledout = g_ledout ^ 0x0C;
//
//  if (g == 255)
//    g_ledout = g_ledout ^ 0x30;
//
//  if (b == 255)
//    g_ledout = g_ledout ^ 0xC0;

  //Set PWM register for each color
  PCA9633_WriteRegister(addr, PCA9633_RED, r);
  PCA9633_WriteRegister(addr, PCA9633_GREEN, g);
  PCA9633_WriteRegister(addr, PCA9633_BLUE, b);
//  PCA9633_WriteRegister(addr, PCA9633_LIDAR, b);
  

  //Set LED driver output state
  PCA9633_WriteRegister(addr, PCA9633_PWM_ON, g_ledout);
}

void XSHUT(int addr, boolean on) {
  if (!on) {
    PCA9633_WriteRegister(addr, PCA9633_LIDAR, 255);
    //Set LED driver output state
    PCA9633_WriteRegister(addr, PCA9633_PWM_ON, g_ledout);
  }
  else {
    PCA9633_WriteRegister(addr, PCA9633_LIDAR, 0);
    //Set LED driver output state
    PCA9633_WriteRegister(addr, PCA9633_PWM_ON, 0);
  }
}


