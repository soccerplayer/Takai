#include"infrared.h"

 //////write your code///////
float Vcc = 5.0;
float dist;

float measure_infrared(void)
{
  dist = Vcc*analogRead(A1)/1023;
  dist = 26.549 * pow(dist,-1.2091)*0.01;
  return(dist);
}
