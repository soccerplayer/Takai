#include "sd.h"
#include <SD.h>
#include <SPI.h>

void saveLog(char *filename, float *data, int num)
{
  File datafile;
  int i;
  
  datafile = SD.open(filename, FILE_WRITE);

  if(datafile != NULL) {
    if(num < 3){
      for(i = 0; i < num; i++) 
      datafile.print(data[i],7);
      datafile.print(",");  
    }
    else{
    for(i = 0; i < num; i++) 
      datafile.print(data[i]);
      datafile.print(",");    
    }
    datafile.print(millis());
    datafile.println();
    datafile.close();
  }
  
}

