#ifndef SAVELOG_H_INCLUDED
#define SAVELOG_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SD_CSPIN A0

/* saveLog
 * 説明:名前がfilenameのファイルにデータを一つ保存する
 * 出力値 なし
 * 入力値 char *filename :保存するファイルの名前 
 *        float data　　 :保存するデータ
 *        int num        :保存するデータの数
 */
void saveLog(char *filename, float *data, int num);

#endif
