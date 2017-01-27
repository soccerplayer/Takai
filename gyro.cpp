#include <SPI.h>
#include "gyro.h"

int L3GD20_CS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;


const byte L3GD20_WHOAMI = 0x0f;
const byte L3GD20_CTRL1 = 0x20;
const byte L3GD20_CTRL2 = 0x21;
const byte L3GD20_CTRL3 = 0x22;
const byte L3GD20_CTRL4 = 0x23;
const byte L3GD20_CTRL5 = 0x24;
const byte L3GD20_X_L = 0x28;
const byte L3GD20_X_H = 0x29;
const byte L3GD20_Y_L = 0x2A;
const byte L3GD20_Y_H = 0x2B;
const byte L3GD20_Z_L = 0x2C;
const byte L3GD20_Z_H = 0x2D;

const byte L3GD20_RW = 0x80;
const byte L3GD20_MS = 0x40;

static float offsetx, offsety, offsetz;
//////write your code///////
void L3GD20_write(byte reg, byte val)
{
  digitalWrite(L3GD20_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(L3GD20_CS, HIGH);
}

byte L3GD20_read(byte reg)
{
  byte ret = 0;

  digitalWrite(L3GD20_CS, LOW);
  SPI.transfer(reg | L3GD20_RW);
  ret = SPI.transfer(0);
  digitalWrite(L3GD20_CS, HIGH);

  return ret;
}

void get_gyro(short *x, short *y, short *z)
{
  short X, Y, Z;

  X = L3GD20_read(L3GD20_X_H);
  *x = X = (X << 8) | L3GD20_read(L3GD20_X_L);
  Y = L3GD20_read(L3GD20_Y_H);
  *y = Y = (Y << 8) | L3GD20_read(L3GD20_Y_L);
  Z = L3GD20_read(L3GD20_Z_H);
  *z = Z = (Z << 8) | L3GD20_read(L3GD20_Z_L);
}

void measure_gyro(float *x, float *y, float *z)
{
  short x_raw, y_raw, z_raw;
  get_gyro(&x_raw, &y_raw, &z_raw);

  *x = (float)(x_raw - offsetx) * 0.00875; // +-250dps
  //x *= 0.0175;// +-500dps
  //x *= 07;  // +-2000dps
  *y = (float)(y_raw - offsety) * 0.00875; // +-250dps
  *z = (float)(z_raw - offsetz) * 0.00875; // +-250dps

}

void init_gyro(int CS1)
{
  digitalWrite(CS1, HIGH);

  pinMode(CS1, OUTPUT);

  L3GD20_CS = CS1;

  int i;
  short tmpx, tmpy, tmpz;
  float avrx, avry, avrz;

  avrx = avry = avrz = 0;

  while (!Serial) {
  }

  Serial.print("L3GD20 ID = ");
  Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show D4

  L3GD20_write(L3GD20_CTRL1, B11001111);
  //   |||||||+ X axis enable
  //   ||||||+- Y axis enable
  //   |||||+-- Z axis enable
  //   ||||+--- PD: 0: power down, 1: active
  //   ||++---- BW1-BW0: cut off [Hz]
  //   ++------ DR1-DR0: ODR 760[HZ]

  L3GD20_write(L3GD20_CTRL2, B00000100);
  L3GD20_write(L3GD20_CTRL5, B00010000);

  for (i = 0; i < OFFSET_NUM; i++)
  {
    get_gyro(&tmpx, &tmpy, &tmpz);
    avrx += tmpx;
    avry += tmpy;
    avrz += tmpz;
  }
  offsetx = avrx / OFFSET_NUM;
  offsety = avry / OFFSET_NUM;
  offsetz = avrz / OFFSET_NUM;
}

