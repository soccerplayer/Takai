#include "wireless.h"


void TransferStr(char *str)
{
  Serial.println(str);  
}

void transferData(float data[], int num)
{
  int i;
  
  for(i = 0; i < num; i++) {
      Serial.print(data[i], 7);
      Serial.print(",");
  }
  
  Serial.print(millis());
  Serial.println();
}

int wirelessAvailable(void)
{
  return(Serial.available());
}

char receiveData(void)
{  
  Serial.read();
}

