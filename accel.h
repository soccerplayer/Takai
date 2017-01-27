#ifndef ACCEL_H_INCLUDE
#define ACCEL_H_INCLUDE

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MODE_2G (2.0 / 512.0)
#define MODE_4G (4.0 / 512.0)
#define MODE_8G (8.0 / 512.0)
#define MODE_16G (16.0 / 512.0)
#define OFFSET_NUM 1000 

//Assign the Chip Select signal to pin 10.
extern int CS2;
extern char POWER_CTL;	//Power Control Register
extern char DATA_FORMAT;
extern char DATAX0;	//X-Axis Data 0
extern char DATAX1;	//X-Axis Data 1
extern char DATAY0;	//Y-Axis Data 0
extern char DATAY1;	//Y-Axis Data 1
extern char DATAZ0;	//Z-Axis Data 0
extern char DATAZ1;	//Z-Axis Data 1


/////////////ユーザ用関数///////////////////
/*init_accel
 *説明:加速度センサの設定
 *出力値;なし
 *入力値:ピン番号
 */
void init_accel(int cs2);


/*measure_accel
 *説明:加速度センサの値の取得
 *出力値:なし
 *入力値:float [m/s^2]
 */
void measure_accel(float *x, float *y, float *z);
////////////////////////////////////////////

void get_accel(int *x, int *y, int *z);
void writeRegister(char registerAddress, char value);
void readRegister(char registerAddress, int numBytes, char * values);

#endif

