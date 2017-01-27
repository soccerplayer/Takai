#include <SPI.h>
#include "accel.h"

//Assign the Chip Select signal to pin 10.
int CS2;
static float offsetx, offsety, offsetz;
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

void measure_accel(float *x, float *y, float *z)
{
  int tmpx, tmpy, tmpz;

  get_accel(&tmpx, &tmpy, &tmpz);
  *x = (float)(tmpx - offsetx) * MODE_4G; //-4g ～ +4gモードの場合
  *y = (float)(tmpy - offsety) * MODE_4G;  //-4g ～ +4gモードの場合
  *z = (float)(tmpz - offsetz) * MODE_4G;  //-4g ～ +4gモードの場合
}

void get_accel(int *x, int *y, int *z)
{
  char values[10];

  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  *x = ((int)values[1] << 8) | (int)values[0];
  //The Y value is stored in values[2] and values[3].
  *y = ((int)values[3] << 8) | (int)values[2];
  //The Z value is stored in values[4] and values[5].
  *z = ((int)values[5] << 8) | (int)values[4];
}

void init_accel(int cs2)
{
  int i, tmpx, tmpy, tmpz;
  float avrx, avry, avrz;

  avrx = avry = avrz = 0;
  
  //Initiate an SPI communication instance.

  //Configure the SPI connection for the ADXL345.
  digitalWrite(cs2, HIGH);

  pinMode(cs2, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  CS2 = cs2;
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
  for (i = 0; i < OFFSET_NUM; i++) {
    get_accel(&tmpx, &tmpy, &tmpz);
    avrx += tmpx;
    avry += tmpy;
    avrz += tmpz;  
  }
  offsetx = avrx / OFFSET_NUM;
  offsety = avry / OFFSET_NUM;
  offsetz = avrz / OFFSET_NUM; 
}

//電圧を０にするらしい
void writeRegister(char registerAddress, char value) {
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS2, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS2, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
//データ読み込み
void readRegister(char registerAddress, int numBytes, char * values) {
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if (numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS2, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for (int i = 0; i < numBytes; i++) {
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS2, HIGH);
}


