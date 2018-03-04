//Register address values for node
#define PCA9633_MODE1 0x00  // Configure PCA9633 for Mode 1
#define PCA9633_MODE2 0x01  // Configure PCA9633 for Mode 1

#define PCA9633_LIDAR 0x02
#define PCA9633_RED 0x03
#define PCA9633_GREEN 0x04
#define PCA9633_BLUE 0x05
#define PCA9633_PWM_ON 0x08

// Set LDRx's of LEDOUT Reg. to 10b for RGB LED, it enables PWM
// Set LDRx of LEDOUT Reg. to 11b for D2 LED, it enables PWM and GRPPWM control
#define ALL_ON_NOT_LIDAR 0x54
#define ALL_ON_WITH_LIDAR 0x55
#define PWM_ON_NOT_LIDAR 0xA9
#define PWM_ON_WITH_LIDAR 0xAA

#define LEDOUT_XSHUT_ON 0
#define LEDOUT_XSHUT_OFF 255
#define XSHUT_ON_WAIT 25

#define BLACK 0,0,0
#define RED 255,0,0
#define GREEN 0,255,0
#define BLUE 0,0,255
#define YELLOW 255,255,0
#define MAGENTA 255,0,255
#define CYAN 0,255,255
#define WHITE 255,255,255

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
void LEDOUT(int addr, int r, int g, int b, int l, int mode)
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
  PCA9633_WriteRegister(addr, PCA9633_LIDAR, l);
  
  //Set LED driver output state
  PCA9633_WriteRegister(addr, PCA9633_PWM_ON, mode);
}

void XSHUT(int addr, boolean on) {
  if (!on) {
    PCA9633_WriteRegister(addr, PCA9633_LIDAR, 255);
    //Set LED driver output state
    PCA9633_WriteRegister(addr, PCA9633_PWM_ON, ALL_ON_WITH_LIDAR);
  }
  else {
    PCA9633_WriteRegister(addr, PCA9633_LIDAR, 0);
    //Set LED driver output state
    PCA9633_WriteRegister(addr, PCA9633_PWM_ON, ALL_ON_NOT_LIDAR);
  }
}


