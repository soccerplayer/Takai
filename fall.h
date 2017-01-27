#ifndef FALL_H_INCLUDE
#define FALL_H_INCLUDE

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ST_START  1
#define ST_UP     2
#define ST_TOP    3
#define ST_DOWN   4
#define ST_LAND   5

#define UP_ALTITUDE 1
#define DOWN_ALTITUDE 2
#define TOP_ALTITUDE 1
#define LAND_ALTITUDE 0

#define PARAPIN A5 //本番はA5バンpin

extern int cur_st;

int check_st(float dtitude);
void release_para(int para_pin, unsigned long heatTime);

#endif
