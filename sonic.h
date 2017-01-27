#ifndef SONIC_H_INCLUDED
#define SONIC_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define TRIGPIN   3
#define ECHOPIN   2
#define CTM       10

/*sonic_init
 *説明:超音波センサの初期設定
 *出力値:なし
 *入力値:なし
*/
void sonic_init(void);
/*measure_sonic
 *説明:超音波センサから距離を算出するプログラムです
 *出力値:float 0.02~1.8 [m]
 *入力値:なし
 *
*/
float measure_sonic(void);

#endif
