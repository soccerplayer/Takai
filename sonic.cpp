#include "sonic.h"

void sonic_init(void)
{
  pinMode(TRIGPIN,OUTPUT);
  pinMode(ECHOPIN,INPUT);
}

float measure_sonic(void)
{
  int dur;
  float dis;

  digitalWrite(TRIGPIN,HIGH);
  delayMicroseconds(CTM);
  digitalWrite(TRIGPIN,LOW);

  dur = pulseIn(ECHOPIN,HIGH);
  dis = (float) dur*0.017;
  dis *=0.01;  
  return(dis);

}





