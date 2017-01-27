#include "fall.h"

int cur_st =  ST_START;

int check_st(float altitude)
{
  if (altitude >= UP_ALTITUDE && cur_st == ST_START) {
    cur_st = ST_UP;
    return (cur_st);
  }
  else if (altitude >= TOP_ALTITUDE && cur_st == ST_UP) {
    cur_st = ST_TOP;
    return(cur_st);
  }
  else if (altitude <= DOWN_ALTITUDE && cur_st == ST_TOP) {
    cur_st = ST_DOWN;
    return (cur_st);
  }
  else if (altitude <= LAND_ALTITUDE && cur_st == ST_DOWN) { 
    cur_st = ST_LAND;
    return (cur_st);
  }
  else {
    return(cur_st);
  }
  
}

void release_para(int para_pin, unsigned long heatTime)
{
  pinMode(para_pin, OUTPUT);
  digitalWrite(para_pin, HIGH);
  delay(heatTime);
  digitalWrite(para_pin, LOW);
}

