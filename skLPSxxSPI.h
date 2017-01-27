/*******************************************************************************
*  skLPSxxSPI.h - 大気圧センサLPS331AP/LPS25H用関数ライブラリのインクルードファイル*
*                                                                              *
* ============================================================================ *
*   VERSION  DATE        BY             CHANGE/COMMENT                         *
* ---------------------------------------------------------------------------- *
*   1.00     2015-01-05  きむ茶工房     Create                                 *
* ============================================================================ *
*   Arduino IDE 1.0.5-r2 (Ardino Duemilanove 328/UNO)                          *
*******************************************************************************/
#ifndef skLPSxxSPI_H_INCLUDED
#define skLPSxxSPI_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define LPS331AP        0xBB       // LPS331APのＩＤ
#define LPS25H          0xBD       // LPS25HのＩＤ

// デバイスのレジスタアドレス
#define WHO_AM_I_ADRS   0x0F       // デバイスの識別ID格納レジスタアドレス
#define RES_CONF_ADRS   0x10       // 圧力分解能のモード設定アドレス
#define CTRL_REG1_ADRS  0x20       // 制御レジスタ1の設定レジスタアドレス
#define CTRL_REG2_ADRS  0x21       // 制御レジスタ2の設定レジスタアドレス
#define CTRL_REG3_ADRS  0x22       // 制御レジスタ3の設定レジスタアドレス
#define STATUS_REG_ADRS 0x27       // ステータス･レジスタアドレス
#define OUT_DATA_ADRS   0x28       // 読み出すデータの先頭レジスタアドレス
#define AMP_CTRL_ADRS   0x30       // アナログ･フロント･エンド制御レジスタアドレス

// 圧力分解能のモード設定(内部での平均値読込み回数の設定)
#define RES_CONF_DATA   0b01101001 // B6-4=AVGT(64回) B3-0=AVGP(384回)
// 制御レジスタ1の設定
#define CTRL_REG1_DATA  0b11000000 // B7=PD(1) B6-4=ODR(1Hz) B3-0=(0000)


/*******************************************************************************
*	クラスの定義                                                           *
*******************************************************************************/
class skLPSxxx
{
  private:
    int Who_Am_I_ID ;
    int CS_Pin ;
    float Press ;               // 圧力の値を保存する変数
    float Temp ;                // 温度の値を保存する変数
    
  public:
    skLPSxxx(int id, int cspin) ;
    int   PressureInit() ;
    void  PressureReceive(char reg_adrs, unsigned char *data, char kosu) ;
    void  PressureSend(char reg_adrs, unsigned char *data, char kosu) ;
    void  PressurePD(char mode) ;
    void  PressureRead() ;
    float getPressure(void);
    float getTempreture(void);
    float AltitudeCalc(float pressure_origin, float pressure_dest) ;
} ;

#endif
