#ifndef GYRO_H_INCLUDE
#define GYRO_H_INCLUDE

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*
*measure_gyro
 *説明:
 *出力値:型、値の範囲、単位
 *入力値:型、引数名、値の範囲、単位
 */

extern int L3GD20_CS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;

#define OFFSET_NUM 1000

extern const byte L3GD20_WHOAMI;
extern const byte L3GD20_CTRL1;
extern const byte L3GD20_CTRL2;
extern const byte L3GD20_CTRL3;
extern const byte L3GD20_CTRL4;
extern const byte L3GD20_CTRL5;
extern const byte L3GD20_X_L;
extern const byte L3GD20_X_H;
extern const byte L3GD20_Y_L;
extern const byte L3GD20_Y_H;
extern const byte L3GD20_Z_L;
extern const byte L3GD20_Z_H;

extern const byte L3GD20_RW;
extern const byte L3GD20_MS;

///////////ユーザ用関数////////////////
/*init_gyro
 *説明:ジャイロセンサの設定
 *出力値:なし
 *入力値:ピン番号
 */
void init_gyro(int CS1);

/*measure_gyro
 *説明:ジャイロセンサを用いて角速度の取得するプログラム
 *出力値:なし      []
 *入力値:float   [rad/s]
 */
void measure_gyro(float *x, float *y, float *z);

/////////////////////////////////////

void get_gyro(short *x, short *y, short *z); //ジャイロの生データ取得
void L3GD20_write(byte reg, byte val);
byte L3GD20_read(byte reg);

#endif

