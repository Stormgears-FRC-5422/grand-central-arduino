//Register address values for node

#define PCA_SYSTEM_INTERRUPT_CLEAR 0x0B
#define PCA_RESULT_INTERRUPT_STATUS 0x13
#define PCA_RESULT_RANGE_STATUS (0x14 + 10)

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
#define XSHUT_ON_WAIT 100

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

  //Set PWM register for each color
  PCA9633_WriteRegister(addr, PCA9633_RED, r);
  PCA9633_WriteRegister(addr, PCA9633_GREEN, g);
  PCA9633_WriteRegister(addr, PCA9633_BLUE, b); 
  PCA9633_WriteRegister(addr, PCA9633_LIDAR, l);
  
  //Set LED driver output state
  PCA9633_WriteRegister(addr, PCA9633_PWM_ON, mode);
}

/* sketchy - not used.  Just use above method with color codes and mode
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
} */
