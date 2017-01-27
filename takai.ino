#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "gyro.h"
#include "skLPSxxSPI.h"
#include "accel.h"
#include "motor.h"
#include "fall.h"
#include "wireless.h"
#include "sd.h"
#include "sonic.h"

////////////////////☆無線で送るデータ☆///////////////////////////
////温度t、圧力p、高さh///////////////////////////////////////////////////////////////
///緯度経度latlon、角度angle3、距離distance///////////////////////////////////////////


#define GOAL_LAT 35.515773// 35.515785  134.171798
#define GOAL_LON 134.171783
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)
#define GOAL_RANGE 0

#define accel_cs A3
#define gyro_cs  A1
#define LPS_cs   A2

#define PGAIN 2.0
#define IGAIN 0.01

//#define MODE_TAKAI
//#define MODE_RUN //全部入っている奴
#define MODE_GPS
//#define MODE_MOTOR

float DestLat, DestLon, OriginLat, OriginLon;
float falldata[3];
float rundata[4];

skLPSxxx LPS(LPS25H, LPS_cs); //自分の使う気圧センサーの指定
TinyGPSPlus gps;
SoftwareSerial ss(9, 8); //rx=9,tx=8

void SPI_init()//センサーマイコン間のSPI通信の初期設定
{
  digitalWrite(10, HIGH);//d10pin指定
  pinMode(10, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);//送るbitの指定
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz/8 = 2MHz; (max 10MHz)
}

void sensor_init(void)/*initは初期設定*/
{
  SPI_init();
  init_gyro(gyro_cs);
  LPS.PressureInit() ;
  //init_accel(accel_cs);
}

float PIDcontrol(float command, float current)//PID制御 傾きの度合い
{
  float controlValue;
  float error;
  static float i_error = 0.0;

  error = command - current;
  i_error += error;

  controlValue = PGAIN * error + IGAIN * i_error;
  //Serial.print("error = ");
  //Serial.print(error);
  //Serial.print("\t");
  //Serial.print("I_error = ");
  //Serial.print(i_error);
  //Serial.print("\t");
  //Serial.print("controlValue = ");
  //Serial.println(controlValue);

  return (controlValue);
}

void ReceiveStr(char *str) { //無線送るための奴　transfer 送る奴
  int i;
  char ch;

  for (i = 0; ch != '!';) {
    if (wirelessAvailable()) {
      ch = receiveData();
      str[i] = ch;
      i++;
    }
  }
  str[i - 1] = '\0';
}


float getDt(void) //かかった時間 微小時間 積分などに使用
{
  float time;
  static float lastTime = (float)millis() / 1000.0;
  float currentTime = (float)millis() / 1000.0;

  time = currentTime - lastTime;
  lastTime = (float)millis() / 1000.0;

  return (time);
}

float AngleNormalization(float angle) //角度の範囲を－180～180°の間に固定
{
  if (angle >= 180)
    angle =  angle - 360;
  else if (angle <= -180)
    angle = angle + 360;

  return (angle);
}

void gelay(unsigned long ms)//データの読み込みが終わるまで次のデータを待たせる⇒常に最新のをとるわけでなく順番にとる マイケルハート
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  }
  while (millis() - start < ms);
}

void motor_control(int motorL, int motorR)
{
  if (motorL < 0) {
    digitalWrite(LDIR, HIGH);
    motorL = -motorL;
  }
  else digitalWrite(LDIR, LOW);
  if (motorR < 0) {
    digitalWrite(RDIR, LOW);
    motorR = -motorR;
  }
  else digitalWrite(RDIR, HIGH);
  motorL = constrain(motorL, 0, 255);//この範囲に調整する
  motorR = constrain(motorR, 0, 255);
  analogWrite(LPWM, motorL);
  analogWrite(RPWM, motorR);
}

#ifdef MODE_RUN /*もし定義していたら＝ifdef*/
void setup()//一回だけするもの
{
  //float x, y, z;
  float angle1, angle2, angle3;
  float controlValue;
  float error;
  unsigned long distance;
  static float RC_LPS[2] = {0};
  float p;
  float pressure_origin;
  unsigned long f;  //unsigned long=-分の容量を+に振り分ける


  Serial.begin(9600);
  ss.begin(9600); //ソフトウェアシリアル begin＝基本は9600に設定
  sensor_init();//全部を初期化する関数　上で作ってある
  sonic_init(); //sonic 超音波 sonic.cppに内容
  motor_init();
////////////////////////////////////////////////////////必要そうなら復帰させる
//  LPS.PressureRead();
//  f = millis();
//  RC_LPS[0] = LPS.getPressure();
//  while ((millis() - f) < 10000) {
//    LPS.getPressure();
//    pressure_origin = LPS.getPressure();
//    RC_LPS[1] = 0.99 * RC_LPS[0] + 0.01 * pressure_origin; //誤差を小さくする
//    pressure_origin = RC_LPS[1];
//    RC_LPS[0] = RC_LPS[1];
//  }
      RC_LPS[0] = 0;
/////////////////////////////////////////////////////////////////////////////////////////////
  char str[100];
  Serial.println("check");//この間は動かしてはいけない
  TransferStr("Ready...");//この段階になったら初期設定完了 
  while (strcmp(str, "START") != 0) //スタートを押して積む
  {
    ReceiveStr(str);
  }

  TransferStr("Start!");
  unsigned long time = millis();

  while (1) {
    float h, t, p;
    LPS.PressureRead();
    t = LPS.getTempreture();
    p = LPS.getPressure();
    h = LPS.AltitudeCalc(pressure_origin, p);  //cppの方で高さを出す関数記載 h:高さ
    RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * h;
    RC_LPS[0] = RC_LPS[1];
    
    falldata[0] = t;
    falldata[1] = p;
    falldata[2] = RC_LPS[1];
    transferData(falldata, 3);//wireless.cpp内の関数　データ送信複数個まとめる

    /*
      Serial.print("t=");
      Serial.print(t);
      Serial.print("\t");
      Serial.print("p=");
      Serial.print(p);
      Serial.print("\t");
      Serial.print("RC_LPS[1];=");
      Serial.print(RC_LPS[1]);
      Serial.print("\t");

      Serial.print("\tmeasure_sonic=");
      Serial.println(measure_sonic());

      if (check_st(RC_LPS[1]) == ST_DOWN && measure_sonic() != 0) {
      RC_LPS[1] = (RC_LPS[1] + constrain(measure_sonic(), 0, 1.8)) / 2.0;
      }
    */
    if (check_st(RC_LPS[1]) == ST_LAND) {       //check_st:fall.cpp内関数 ST_LAND=着地
      Serial.println("release");
      release_para(PARAPIN, 1000);
      break;
    }
    if (millis() - time >= 600000) {           //１０分で自動的に切れる　妥当な時間と思われ
      Serial.println("release");
      //release_para(PARAPIN, 1000);
      break;
    }

  }  ////落下検知終了////////////

  delay(10000); //10s
  gelay(2000); //2s データとる
  OriginLat = gps.location.lat(); //緯度経度を取る
  OriginLon = gps.location.lng(); //

  rundata[0] = OriginLat;
  rundata[1] = OriginLon;
  /*
    Serial.print("OriginLat=");
    Serial.print(OriginLat);
    Serial.print("\t");
    Serial.print("OriginLon=");
    Serial.println(OriginLon);
  */
  distance =  (unsigned long)TinyGPSPlus::distanceBetween(  //２点間の距離を求める関数　おそらくライブラリ
                OriginLat,
                OriginLon,
                GOAL_LAT,
                GOAL_LON);

  rundata[2] = distance;
  rundata[3] = 0;  //曲がる角度　下で計算
  transferData(rundata, 4);  //データ送る
  if (distance <= GOAL_RANGE)　//もしゴールに着地していたら
  {
    Serial.println("goal"); //ゴールと送る
    motor_control(0, 0);  //モーターの電圧0 
    while (1);  //　この状態(何もしない状態)をやり続ける
  }

  motor_control(75, 75); //0~255上に範囲記載
  delay(10000);          //10s走る
  motor_control(0, 0);
  gelay(1000);           //1sデータ取得
  DestLat = gps.location.lat();
  DestLon = gps.location.lng();  //現在地の緯度経度取得


  angle1 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, DestLat, DestLon);　　　//北との角度がわかる：少し古い　最新緯度経度
  angle2 = TinyGPSPlus::courseTo(
             DestLat, DestLon, GOAL_LAT, GOAL_LON);　　　　//北との角度　　　　：最新　ゴール緯度経度
  angle3 = angle1 - angle2;                               //どれだけ回転すればいいかの
  angle3 = -DEG2RAD * AngleNormalization(angle3);　　　　//AngleNor～～　180°までで判断し無駄に回転をさせないもの　上に記載
  OriginLat = DestLat;
  OriginLon = DestLon;
  controlValue = PIDcontrol(0, angle3);

  if (controlValue >= 0)
    controlValue = constrain(controlValue, 0, 77);
  else controlValue = constrain(abs(controlValue), 0 , 77);

  motor_control(75 - controlValue, 75 + controlValue);
}

void loop()
{
  float x, y, z;
  float originLat, originLon;
  float angle1, angle2, angle3;
  static float angle = 0;　　　//static 教科書p208 loopが終わっても0に初期化されない  
  float dt;
  float controlValue;
  float error;
  unsigned long distance;

  //measure_gyro(&x, &y, &z);
  dt = getDt();
  //angle += z * dt;
  //angle = DEG2RAD*AngleNormalization(angle);
  angle = 0;
  gelay(2000);

  DestLat = gps.location.lat();
  DestLon = gps.location.lng();
  /*
    Serial.print("DestLat=");
    Serial.print(DestLat, 7);
    Serial.print("\tDestLon=");
    Serial.println(DestLon, 7);
  */
  rundata[0] = DestLat;
  rundata[1] = DestLon;

  distance = (unsigned long)TinyGPSPlus::distanceBetween(
               DestLat,
               DestLon,
               GOAL_LAT,
               GOAL_LON);
  rundata[2] = distance;

  if (distance <= GOAL_RANGE)
  {
    Serial.println("goal");
    motor_control(0, 0);
    while (1);
  }

  angle1 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, DestLat, DestLon);
  angle2 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, GOAL_LAT, GOAL_LON);

  angle3 = angle1 - angle2;
  angle3 = -DEG2RAD * AngleNormalization(angle3);
  
  rundata[3] = angle3;

  transferData(rundata, 4);

  error = angle3 - angle;　　//誤差　角度

  controlValue = PIDcontrol(0, error);   //どれくらい曲がるか
  controlValue = constrain(controlValue, -77, 77);  //constrain -77～77の間に制限
  //Serial.print("controlValue = ");
  //Serial.println(controlValue);
  motor_control(75 - controlValue, 75 + controlValue);　//曲がりの値　モーターで調整
  OriginLat = DestLat;
  OriginLon = DestLon;　　//現在地を元の値へ　ループ
}
#endif

#ifdef MODE_GPS　　//おそらくGPSのみ
int i=1;

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  gelay(0);
}
void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  
  
}
void loop() {
  float LAT;
  float LON;
  unsigned long t;

  gelay(1000);
  LAT = gps.location.lat();
  LON = gps.location.lng();
  //Serial.print("");
  //Serial.print(LAT);
  //Serial.print("\t");
  //Serial.println(LON);
  Serial.print("LAT");
  printFloat(LAT, gps.location.isValid(), 11, 8);
  Serial.print("\t");
  Serial.print("LON");
  printFloat(LON, gps.location.isValid(), 12, 8);
  Serial.print("\n");
  
  t = millis()/1000;
  if(3600*i<t)
  {
    Serial.print(i);
    Serial.println("time");
    i++; 
  }
  delay(1000);
}
#endif

#ifdef MODE_MOTOR
void setup() {
  motor_init();
}

void loop() {
  motor_control(50, -50);
}
#endif

#ifdef MODE_TAKAI
void setup()
{
  //float x, y, z;
  float angle1, angle2, angle3;
  float controlValue;
  float error;
  unsigned long distance;
  static float RC_LPS[2] = {0};
  float p;
  float pressure_origin;
  unsigned long f;


  Serial.begin(9600);
  ss.begin(9600);
  sensor_init();
  sonic_init();
  motor_init();

 /* LPS.PressureRead();
  f = millis();
  RC_LPS[0] = LPS.getPressure();
  while ((millis() - f) < 10000) {
    LPS.getPressure();
    pressure_origin = LPS.getPressure();
    RC_LPS[1] = 0.99 * RC_LPS[0] + 0.01 * pressure_origin;
    pressure_origin = RC_LPS[1];
    RC_LPS[0] = RC_LPS[1];
  }
  RC_LPS[0] = 0;
  char str[100];
  Serial.println("check");
  TransferStr("Ready...");
  while (strcmp(str, "START") != 0) {  //strcmp 比較して～
    ReceiveStr(str);
  }

  TransferStr("Start!");
  unsigned long time = millis();

  while (1) {
    float h, t, p;
    LPS.PressureRead();
    t = LPS.getTempreture();
    p = LPS.getPressure();
    h = LPS.AltitudeCalc(pressure_origin, p);
    RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * h;
    RC_LPS[0] = RC_LPS[1];

    falldata[0] = t;
    falldata[1] = p;
    falldata[2] = RC_LPS[1];
    transferData(falldata, 3);

      Serial.print("t=");
      Serial.print(t);
      Serial.print("\t");
      Serial.print("p=");
      Serial.print(p);
      Serial.print("\t");
      Serial.print("RC_LPS[1];=");
      Serial.print(RC_LPS[1]);
      Serial.print("\t");

      Serial.print("\tmeasure_sonic=");
      Serial.println(measure_sonic());

      if (check_st(RC_LPS[1]) == ST_DOWN && measure_sonic() != 0) {
      RC_LPS[1] = (RC_LPS[1] + constrain(measure_sonic(), 0, 1.8)) / 2.0;
      }
    
    if (check_st(RC_LPS[1]) == ST_LAND) {
      Serial.println("release");
      release_para(PARAPIN, 1000);
      break;
    }
    if (millis() - time >= 600000) {
      Serial.println("release");
      //release_para(PARAPIN, 1000);
      break;
    }

  }
*/
  delay(10000);
  gelay(2000);
  OriginLat = gps.location.lat();
  OriginLon = gps.location.lng();

  rundata[0] = OriginLat;
  rundata[1] = OriginLon;
  /*
    Serial.print("OriginLat=");
    Serial.print(OriginLat);
    Serial.print("\t");
    Serial.print("OriginLon=");
    Serial.println(OriginLon);
  */
  distance =  (unsigned long)TinyGPSPlus::distanceBetween(
                OriginLat,
                OriginLon,
                GOAL_LAT,
                GOAL_LON);

  rundata[2] = distance;
  rundata[3] = 0;
  transferData(rundata, 4);
  if (distance <= GOAL_RANGE)
  {
    Serial.println("goal");
    motor_control(0, 0);
    while (1);
  }

  motor_control(75, 75);
  delay(10000);
  motor_control(0, 0);
  gelay(1000);
  DestLat = gps.location.lat();
  DestLon = gps.location.lng();


  angle1 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, DestLat, DestLon);
  angle2 = TinyGPSPlus::courseTo(
             DestLat, DestLon, GOAL_LAT, GOAL_LON);
  angle3 = angle1 - angle2;
  angle3 = -DEG2RAD * AngleNormalization(angle3);

  OriginLat = DestLat;
  OriginLon = DestLon;
  controlValue = PIDcontrol(0, angle3);

  if (controlValue >= 0)
    controlValue = constrain(controlValue, 0, 77);
  else controlValue = constrain(abs(controlValue), 0 , 77);

  motor_control(75 - controlValue, 75 + controlValue);
}

void loop()
{
  float x, y, z;
  float originLat, originLon;
  float angle1, angle2, angle3;
  static float angle = 0;
  float dt;
  float controlValue;
  float error;
  unsigned long distance;

  //measure_gyro(&x, &y, &z);
  dt = getDt();
  //angle += z * dt;
  //angle = DEG2RAD*AngleNormalization(angle);
  angle = 0;
  gelay(2000);

  DestLat = gps.location.lat();
  DestLon = gps.location.lng();
  /*
    Serial.print("DestLat=");
    Serial.print(DestLat, 7);
    Serial.print("\tDestLon=");
    Serial.println(DestLon, 7);
  */
  rundata[0] = DestLat;
  rundata[1] = DestLon;

  distance = (unsigned long)TinyGPSPlus::distanceBetween(
               DestLat,
               DestLon,
               GOAL_LAT,
               GOAL_LON);
  rundata[2] = distance;

  if (distance <= GOAL_RANGE)
  {
    Serial.println("goal");
    motor_control(0, 0);
    while (1);
  }

  angle1 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, DestLat, DestLon);
  angle2 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, GOAL_LAT, GOAL_LON);

  angle3 = angle1 - angle2;
  angle3 = -DEG2RAD * AngleNormalization(angle3);

  rundata[3] = angle3;　　　　 //最初0だった曲がる角度　

  transferData(rundata, 4);

  error = angle3 - angle;

  controlValue = PIDcontrol(0, error);
  controlValue = constrain(controlValue, -77, 77);
  //Serial.print("controlValue = ");
  //Serial.println(controlValue);
  motor_control(75 - controlValue, 75 + controlValue);
  OriginLat = DestLat;
  OriginLon = DestLon;
}
#endif


