#ifndef WIRELESS_H_INCLUDED
#define WIRELESS_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void transferData(float data[], int num);
void TransferStr(char *str);
char receiveData(void);
int wirelessAvailable(void);

#endif
