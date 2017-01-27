/*******************************************************************************
   skLPSxxSPI - 大気圧センサLPS331AP/LPS25H(SPI接続)用関数ライブラリ
*                                                                              *
     skLPSxxx        - この関数ライブラリを生成する時の初期化処理
     PressureInit    - デバイスの初期化を行う処理
     PressureReceive - デバイスから指定個数のデータを受信する処理
     PressureSend    - デバイスに指定個数のデータを送信する処理
     PressurePD      - デバイスの"パワーダウン"と"動作中"を切り替える処理
     PressureRead    - 圧力･温度を読込み気圧値を計算する処理
     AltitudeCalc    - 気圧値(hPa)から高度を計算する処理
*                                                                              *
  ============================================================================
    VERSION  DATE        BY             CHANGE/COMMENT
  ----------------------------------------------------------------------------
    1.00     2015-01-07  きむ茶工房     Create
  ============================================================================
    Arduino IDE 1.0.5-r2 (Ardino Duemilanove 328/UNO)
*******************************************************************************/
#include <SPI.h>
#include "skLPSxxSPI.h"

/*******************************************************************************
   skLPSxxx(id,cspin)
     この関数ライブラリを生成する時の初期化処理(コンストラクタ)
     cspin   : CSのピン番号を指定します
     id      : デバイスの識別IDを指定します
*******************************************************************************/
skLPSxxx::skLPSxxx(int id, int cspin)
{
  Who_Am_I_ID =  id;
  CS_Pin      = cspin ;
  Press = 0;
  Temp = 0;
  digitalWrite(cspin, HIGH);

  pinMode(cspin, OUTPUT);
}
/*******************************************************************************
   ans = PressureInit()
     デバイスの初期化を行う処理
     デバイスの動作を確かめる為にデバイスIDのチェックを行います。
     分解能のモード(内部平均の数)設定は、デフォルトで動作させます。
     制御レジスタ1は"動作中モード","1Hzの更新速度",Bit0-3=0 で初期化しています
     初期化データを変更する場合は、skLPSxx.hのRES_CONF_DATA/CTRL_REG1_DATAを
     書き換え変更して下さい。
     ans  : 戻り値、0=正常終了
                    6=デバイスのIDチェックエラー
*******************************************************************************/
int skLPSxxx::PressureInit()
{
  int  ans , i ;
  unsigned char data[2] ;

  // デバイスの識別ＩＤをチェックする処理
  PressureReceive(WHO_AM_I_ADRS, &data[0], 1) ;
  Serial.print("LPSxxxxxID = ");
  Serial.println(data[0], HEX);
  // WHO_AM_Iの内容をチェック
  if (data[0] == Who_Am_I_ID) ans = 0 ; // ＩＤは一致した
  else                        ans = 6 ; // ＩＤが一致しない
  // デバイスを初期化する処理
  if (ans == 0) {
    // 圧力分解能のモード設定(圧力=256回 温度=64回 で平均を行う)
    //data[0] = RES_CONF_DATA ;
    //PressureSend(RES_CONF_ADRS,&data[0],1) ;
    // 制御レジスタ1の設定(動作中モード,出力データ速度は1Hz)
    data[0] = CTRL_REG1_DATA ;
    PressureSend(CTRL_REG1_ADRS, &data[0], 1) ;
    delay(1) ;                       // 確実に設定が終了するのを待つ
  }
  return ans ;
}
/*******************************************************************************
   PressureReceive(reg_adrs,*data,kosu)
   デバイスから指定個数のデータを受信する処理
     reg_adrs : 読出すデータのレジスターアドレスを指定する
                連続的に読出す場合は、読出すレジスターの先頭アドレスを指定
*    *data    : 読出したデータの格納先を指定する                               *
     kosu     : 読出すデータのバイト数を指定する
*******************************************************************************/
void skLPSxxx::PressureReceive(char reg_adrs, unsigned char *data, char kosu)
{
  int  i ;
  char sub ;

  if (kosu == 1) sub = reg_adrs | 0x80 ;  // 単発読み出しモード(RW.MS.AD5-0)
  else           sub = reg_adrs | 0xC0 ;  // 連続読み出しモード(RW.MS.AD5-0)

  digitalWrite(CS_Pin, LOW) ;             // SS(CS)ラインをLOWにする
  SPI.transfer(sub) ;                     // レジスタアドレスのデータを発行する
  for (i = 0 ; i < kosu ; i++) {
    *data = SPI.transfer(0x00) ;       // ダミーのデータを送り、データを受信する
    data++ ;
  }
  digitalWrite(CS_Pin, HIGH) ;            // SS(CS)ラインをHIGHにする
}
/*******************************************************************************
   PressureSend(reg_adrs,*data,kosu)
   デバイスに指定個数のデータを送信する処理
     reg_adrs : 書出すデータのレジスターアドレスを指定する
                連続的に書出す場合は、書出すレジスターの先頭アドレスを指定
*    *data    : 書出すデータの格納先を指定する                                 *
     kosu     : 書出すデータのバイト数を指定する
*******************************************************************************/
void skLPSxxx::PressureSend(char reg_adrs, unsigned char *data, char kosu)
{
  int  i ;
  char sub ;

  if (kosu == 1) sub = reg_adrs ;         // 単発書出しモード(RW.MS.AD5-0)
  else           sub = reg_adrs | 0x40 ;  // 連続書出しモード(RW.MS.AD5-0)

  digitalWrite(CS_Pin, LOW) ;             // SS(CS)ラインをLOWにする
  SPI.transfer(sub) ;                     // レジスタアドレスのデータを発行する
  for (i = 0 ; i < kosu ; i++) {
    SPI.transfer(*data) ;              // データを送信する
    data++ ;
  }
  digitalWrite(CS_Pin, HIGH) ;            // SS(CS)ラインをHIGHにする
}
/*******************************************************************************
   ans = PressurePD(mode)
   デバイスの"パワーダウン"と"動作中"を切り替える処理
     mode : 0=パワーダウンモード , 1=動作中モード
*******************************************************************************/
void skLPSxxx::PressurePD(char mode)
{
  unsigned char data[2] ;

  // 制御レジスタ1の現在情報を受信する
  PressureReceive(CTRL_REG1_ADRS, &data[0], 1) ;
  if (mode == 0) {
    data[0] = data[0] & 0x7F ;    // パワーダウンモードにする
  } else {
    data[0] = data[0] | 0x80 ;    // 動作中モードにする
  }
  // 制御レジスタ1の情報を送信する
  PressureSend(CTRL_REG1_ADRS, &data[0], 1) ;
  delay(1) ;                         // 確実に設定が終了するのを待つ
}
/*******************************************************************************
   ans = PressureRead()
   圧力･温度を読込み気圧値を計算する処理
   計算された気圧値(hPa)はPress変数に、温度値(℃)はTemp変数に其々格納されます。
*******************************************************************************/
void skLPSxxx::PressureRead()
{
  unsigned char data[6] ;

  // 圧力と温度データを受信する(このデータはデバイス内部で平均化されています)
  PressureReceive(OUT_DATA_ADRS, &data[0], 5) ;
  // 大気圧の計算を行う
  Press = ((unsigned long)data[2] << 16) + ((unsigned long)data[1] << 8) + data[0] ;
  Press = Press / 4096.0 ;      // hPa
  // 温度の計算を行う
  Temp = (int)(1 + ~((unsigned int)data[4] << 8 | data[3])) * -1 ;
  Temp = 42.5 + (Temp / 480.0) ;// ℃
}

float skLPSxxx::getPressure(void)
{
  return (Press);
}

float skLPSxxx::getTempreture(void)
{
  return (Temp);
}
/*******************************************************************************
   ans = AltitudeCalc(pressure,Difference)
   気圧値(hPa)から高度を計算する処理
     pressure   : 計算した大気圧値を hPa でセットする。
     Difference : 標高の差を指定する
     ans        : 高度値を ｍ で返す。
*******************************************************************************/
float skLPSxxx::AltitudeCalc(float pressure_origin, float pressure_dest)
{
  float h ;

  h = (1 - pow(pressure_dest / pressure_origin, 0.190263)) * 44330.8;

  return h ;
}
