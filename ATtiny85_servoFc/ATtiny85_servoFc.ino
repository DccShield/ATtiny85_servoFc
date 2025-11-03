//--------------------------------------------------------------------------------
// DCC Function Decoder for Smile Function Servo Decoder
// Copyright(C)'2025 Ayanosuke(Maison de DCC) / Desktop Station
// 
// Attiny85 16MHz(PLL) BOD.Enabled 1.8v LTO.dis
//
// DCC電子工作連合
// https://desktopstation.net/tmi/
//
// DCC館
// http://dcc.client.jp/
// http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

#include <arduino.h>
#include "NmraDcc.h"
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below
#include "SoftwareSerial.h"

//各種設定、宣言
//#define SIODBG       // リリース時 コメントアウトすること
#define DECODER_ADDRESS 3
#define DCC_ACK_PIN 0   // Atiny85 PB0(5pin) if defined enables the ACK pin functionality. Comment out to disable.
//                      // Atiny85 DCCin(7pin)
#define O1 0            // Atiny85 PB0(5pin)
#define O2 1            // Atiny85 PB1(6pin) analogwrite
#define O3 3            // Atint85 PB3(2pin)
#define O4 4            // Atiny85 PB4(3pin) analogwrite Servo用

#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0

#define F0 0
#define F1 1
#define F2 2
#define F3 3
#define F4 4
#define F5 5
#define F6 6
#define F7 7
#define F8 8
#define F9 9
#define F10 10
#define F11 11
#define F12 12

#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0

// ファンクション CV変換テーブル

#define CV_zeroDeg 60       // 0deg時のPWMの値
#define CV_ninetyDeg 61     // 90deg時のPWMの値
#define CV_onDeg 62         // on時の角度
#define CV_offDeg 63        // off時の角度
#define CV_initDeg 64       // 起動時の角度
#define CV_onSpeed 65       // off->onに移行する時間
#define CV_offSpeed 66      // on->offに移行する時間
#define CV_sdir 67          // 最後のdir保持用
#define CV_68_function 68   // サーボモータファンクッション番号
#define CV_dummy 69         // dummy
#define MAN_ID_NUMBER 166   // Manufacture ID //
#define MAN_VER_NUMBER  01  // Release Ver CV07 //

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

#if defined(SIODBG)
  //SerialDBG
  SoftwareSerial mySerial(O2, O1); // RX, TX
#endif

//Task Schedule
unsigned long gPreviousL5 = 0;
#if !defined(SIODBG) //////////////////////////////////////////////////
unsigned long gPreviousL10 = 0;
#endif
//進行方向
uint8_t gDirection = 128;

//モータ制御関連の変数
uint32_t gSpeedRef = 1;

//Function State
uint8_t gState_F0 = 0;
uint8_t gState_F1 = 0;
uint8_t gState_F2 = 0;
uint8_t gState_F3 = 0;
uint8_t gState_F4 = 0;
uint8_t gState_F5 = 0;
uint8_t gState_F6 = 0;
uint8_t gState_F7 = 0;
uint8_t gState_F8 = 0;
uint8_t gState_F9 = 0;
uint8_t gState_F10 = 0;
uint8_t gState_F11 = 0; 
uint8_t gState_F12 = 0;
uint8_t gState_F13 = 0;
uint8_t gState_F14 = 0;
uint8_t gState_F15 = 0;
uint8_t gState_F16 = 0;
uint8_t gState_Function = 0;


//CV related
uint8_t gCV1_SAddr = 3;
uint8_t gCVx_LAddr = 3;
uint8_t gCV29_Conf = 0;
uint8_t gCV68_servoAdder = 4; // サーボアドレス

uint8_t zeroDeg = 0;        // 0degのPWM値
uint8_t ninetyDeg = 0;      // 90degのPWM値 ※ HK-5320は0-60deg DM-S0025は0-180deg
uint8_t onDeg = 0;          // ON時の角度
uint8_t offDeg = 0;         // OFF時の角度
uint8_t initDeg = 0;        // 電源切る前の角度
uint8_t onSpeed = 0;        // OFF->ONのスピード
uint8_t offSpeed = 0;       // ON->OFFのスピード
uint8_t gDir = 0;           // ON/OFF 
uint8_t Ndas = 0;
uint8_t sdir = 0;           // gDirの最新値保存用

static float nowDeg = 0;

//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

struct CVPair {
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS}, // CV01
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},               // CV09 The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},          // CV17 XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},          // CV18 YY in the XXYY address
  {CV_29_CONFIG, 2},                                   // CV29 Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
//{CV_zeroDeg ,8},                        // CV60 8で約500us 
//{CV_ninetyDeg ,36},                     // CV61 36で約2400us DM-S0025は36以上に設定すると動きがおかしくなる
  {CV_zeroDeg ,13},                        // CV60 13:880uS GS-1502
  {CV_ninetyDeg ,33},                     // CV61 33:2.14ms GS-1502
  {CV_onDeg ,180},                        // CV62 on時の角度
  {CV_offDeg  ,0},                        // CV63 off時の角度
  {CV_initDeg ,0},                        // CV64 電源切る前の角度
//{CV_onSpeed, 20},                       // CV65 20で2000msec
//{CV_offSpeed, 20},                      // CV66 20で2000msec
  {CV_onSpeed, 5},                       // CV65 5で500msec
  {CV_offSpeed, 5},                      // CV66 5で500msec
  {CV_sdir , 0},                          // CV67 0
  {gCV68_servoAdder , 4},                 // CV68 servo Address 4
  {CV_dummy,0},
};

void(* resetFunc) (void) = 0;  //declare reset function at address 0
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

void resetCVToDefault();
void LightControl();

//------------------------------------------------------------------
// notifyCVResetFactoryDefault()
// CV値を工場出荷状態に設定
//------------------------------------------------------------------
void notifyCVResetFactoryDefault()
{
  //When anything is writen to CV8 reset to defaults.
  resetCVToDefault();
  //Serial.println("Resetting...");
  delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying

  resetFunc();
};

//------------------------------------------------------------------
// CVをデフォルトにリセット
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  for (int j = 0; j < FactoryDefaultCVIndex; j++ ) {
    Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  }
};


//------------------------------------------------------------------
// CV Ack
//------------------------------------------------------------------
void notifyCVAck(void)
{
/*
#if !defined(SIODBG) //////////////////////////////////////////////////
  digitalWrite(O1,ON);
  digitalWrite(O2,ON);
  digitalWrite(O3,ON);
  digitalWrite(O4,ON);

  delay( 6 );

  digitalWrite(O1,OFF);
  digitalWrite(O2,OFF);
  digitalWrite(O3,OFF);
  digitalWrite(O4,OFF);
#endif              //////////////////////////////////////////////////
*/
}


//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
  uint8_t cv_value;

//TCCR1 = 0<<CTC1 | 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
  pinMode(O1, OUTPUT);    // ソフトシリアル用 
  pinMode(O2, OUTPUT);
  pinMode(O3, OUTPUT);
  pinMode(O4, OUTPUT);
//digitalWrite(O4 ,HIGH);   // 起動時にサーボが0deg方向に最大まで行かないように
 
#if defined(SIODBG) //////////////////////////////////////////////////
  mySerial.begin(9600);
  mySerial.println("...");
//mySerial.println("Hello,SFM");
#endif              //////////////////////////////////////////////////

  //DCCの応答用負荷ピン
#if defined(DCCACKPIN)
  //Setup ACK Pin
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, 0);
#endif

#if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
  if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ) {	 //if eeprom has 0xFF then assume it needs to be programmed
    //Serial.println("CV Defaulting due to blank eeprom");
    notifyCVResetFactoryDefault();

  } else {
    //Serial.println("CV Not Defaulting");
  }
#else
  //Serial.println("CV Defaulting Always On Powerup");
  notifyCVResetFactoryDefault();
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0); // Atiny85 7pin(PB2)をDCC_PULSE端子に設定

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100,   FLAGS_MY_ADDRESS_ONLY , 0 );

  //Reset task
  gPreviousL5 = millis();
#if !defined(SIODBG) //////////////////////////////////////////////////
  gPreviousL10 = gPreviousL5;
#endif
  //Init CVs
  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS ) ;
  gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB );
  gCV29_Conf = Dcc.getCV( CV_29_CONFIG );
  gCV68_servoAdder = Dcc.getCV( CV_68_function );

  //Init CVs
  zeroDeg = Dcc.getCV( CV_zeroDeg );
  ninetyDeg = Dcc.getCV( CV_ninetyDeg );
  onDeg = Dcc.getCV( CV_onDeg );
  offDeg = Dcc.getCV( CV_offDeg );
  initDeg = Dcc.getCV( CV_initDeg );
  onSpeed = Dcc.getCV( CV_onSpeed );  
  offSpeed = Dcc.getCV( CV_offSpeed );
  sdir = Dcc.getCV( CV_sdir );

#if defined(SIODBG) //////////////////////////////////////////////////
// mySerial.println(gCV68_servoAdder,HEX);
// mySerial.println(zeroDeg,HEX);
//  mySerial.println(ninetyDeg ,HEX);
//  mySerial.println(onDeg,HEX);
//  mySerial.println(offDeg,HEX);
//  mySerial.println(initDeg,HEX);
//  mySerial.println(onSpeed,HEX);
//  mySerial.println(offSpeed,HEX);
//  mySerial.println(gDir,HEX);
//  mySerial.println(sdir,HEX);
//  mySerial.println("###");
#endif             //////////////////////////////////////////////////
  GTCCR = 1 << PWM1B | 2 << COM1B0;
  TCCR1 = 11 << CS10;
//  gPreviousL5 = millis();

}


//---------------------------------------------------------------------
// Arduino Main Loop
//---------------------------------------------------------------------
void loop() {
  Dcc.process();
  if ( (millis() - gPreviousL5) >= 10) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
  {
    ServoControl();
    gPreviousL5 = millis();
  }
#if !defined(SIODBG) //////////////////////////////////////////////////
  if ( (millis() - gPreviousL10) >= 100) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
  {
    led_control();
    gPreviousL10 = millis();
  }
#endif
}

void led_control()
{
  if(gState_F1 == 0){
    digitalWrite(O1, OFF);
  } else {
    digitalWrite(O1, ON);
  }
  if(gState_F2 == 0){
    digitalWrite(O2, OFF);
  } else {
    digitalWrite(O2, ON);
  }
  if(gState_F3 == 0){
    digitalWrite(O3, OFF);
  } else {
    digitalWrite(O3, ON);
  }
}

//---------------------------------------------------------------------
// 最終値のアクセサリ番号をCV_sdirに書き込み
//---------------------------------------------------------------------
void writeCV()
{
  Dcc.setCV(CV_sdir,gDir);        // 最終値のアクセサリ番号をCV_sdirに書き込み
}
//---------------------------------------------------------------------
// サーボ用信号出力
//---------------------------------------------------------------------
void anaWR()
{
#if defined(SIODBG) //////////////////////////////////////////////////
  char foo;
  foo=map(nowDeg,0,180, zeroDeg, ninetyDeg);
  analogWrite(O4, foo);
  mySerial.println(foo,DEC);
#else
  analogWrite(O4, map(nowDeg,0,180, zeroDeg, ninetyDeg));
#endif             //////////////////////////////////////////////////

//  mySerial.print("A:");
//  mySerial.print(nowDeg,DEC);
//  mySerial.println(foo,DEC);
//#if !defined(SIODBG) //////////////////////////////////////////////////
//  analogWrite(O4, map(nowDeg,0,180, zeroDeg, ninetyDeg));
//  mySerial.print("A:");
//  mySerial.println((int)(nowDeg*1000),DEC);
//#endif             //////////////////////////////////////////////////
}

//---------------------------------------------------------------------
// Servo control Task (100Hz:100ms)
//---------------------------------------------------------------------
void ServoControl()
{
  enum{
      ST_INIT = 0,  // 0
      ST_STANDABY,  // 1
      ST_IDLE,      // 2
      ST_ON,        // 3
      ST_ON_RUN,    // 4
      ST_OFF,       // 5
      ST_OFF_RUN,   // 6
  };
  
  static char state = ST_INIT;        // ステート
  static float delt_deg = 0;          // 10ms辺りの変化量
  static char updown_flg = 0;         // 0:up 1:down

#if defined(SIODBG) /////////////////////////////////////////////////
  static char prestate = 7;     // debug
  if(state != prestate ) {
    mySerial.print("s:");
    mySerial.println(state,DEC);
    prestate = state;
  }

//  static char pregState = 7;     // debug
//  if(gState_Function != pregState ) {
//    mySerial.println(gState_Function,HEX);
//    pregState = gState_Function;
// }
#endif             /////////////////////////////////////////////////

  switch(state){
      case ST_INIT: //0
        if(Ndas == 1){                  // DCCコマンドを受信した？
          if(gState_F4 == sdir ){       // 前回最後のSTR/DIVが同じ？　前回ONの状態でDCC信号が入って来た時に
            state = ST_STANDABY;        // gState_F4 = 0:OFF で入ってきた時の防護策
          }
        }
        break;
      case ST_STANDABY: //1               // 起動時一回だけの処理

//    mySerial.print("1:");               // 起動時の状態確認用
//    mySerial.print(gState_F4,DEC);
//    mySerial.print(":");
//    mySerial.println(sdir,DEC);

        if(gState_F4 == sdir ){ // 前回最後のSTR/DIVが同じ？
          if(gState_F4 == 0){   // Servo off?
            nowDeg = offDeg;     
          } else {                    // Servo on
            nowDeg = onDeg;  
          }
          state = ST_IDLE;
          break;
        } else {
          if(sdir == 0 and gState_F4 == 0){ // sdir = ON ?
              state = ST_OFF;             
          } else {
            state = ST_ON;
          }
        }
        break;
      case ST_IDLE: // 2 
            if(gState_F4 == 0 ){           // Servo O4:OFF
              if(nowDeg == offDeg){   // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              state = ST_OFF;
            } else if(gState_F4 != 0 ){    // Servo O4:ON 10/11 ==0 -> !=0
              if(nowDeg == onDeg){    // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              state = ST_ON;
            }
            break;

      case ST_ON: //3: // ON処理
            if(onDeg - offDeg > 0){  // 上昇か、下降か確認
              updown_flg = UP;       // 上昇
              delt_deg = (float)abs(onDeg - offDeg) / onSpeed / 10; // offDegからonDegまでの移動量を算出
            } else {
              updown_flg = DOWN;       // 下降
              delt_deg = (float)abs(onDeg - offDeg) / offSpeed / 10; // offDegからonDegまでの移動量を算出
            }          
            nowDeg = offDeg;        // 現在の角度を導入
            anaWR();
            state = ST_ON_RUN;
            break;
            
      case ST_ON_RUN: // 4
             if(updown_flg == UP) {
                nowDeg = nowDeg + delt_deg;       // 上昇
                if(nowDeg > onDeg){
                  nowDeg = onDeg;
                  anaWR();
                  gDir = gState_F4;                       // Servo on　になった
                  writeCV();
                  state = ST_IDLE;                  
                } else {
                  anaWR();          
                  state = ST_ON_RUN;                  
                }
              } else {
                nowDeg = nowDeg - delt_deg;       // 下降
  
               if(nowDeg < onDeg){
                  nowDeg = onDeg;
                  anaWR();
                  gDir = gState_F4;                       // Servo on　になった
                  writeCV();
                  state = ST_IDLE;   
                } else {
                  anaWR();              
                 state = ST_ON_RUN;                    
               }
              }
            break;

      case ST_OFF: //5:  // ON->OFF処理
            if(onDeg - offDeg > 0){  // 上昇か、下降か確認
              updown_flg = DOWN;       // 下昇
              delt_deg = (float)abs(onDeg - offDeg) / offSpeed / 10; // offDegからonDegまでの移動量を算出
            } else {
              updown_flg = UP;       // 上降
              delt_deg = (float)abs(onDeg - offDeg) / onSpeed / 10; // offDegからonDegまでの移動量を算出
            }
            nowDeg = onDeg;        // 現在の角度を導入
            anaWR();
            state = ST_OFF_RUN;
            break;
            
      case ST_OFF_RUN: //6
              if(updown_flg == UP) {
                nowDeg = nowDeg + delt_deg;       // 上昇
                if(nowDeg > offDeg){
                  nowDeg = offDeg;
                  anaWR();
                  gDir = gState_F4;                       // Servo off　になった
                  writeCV();
                  state = ST_IDLE;                  
                } else {
                  anaWR();
                  state = ST_OFF_RUN;                  
                }
              } else {
                nowDeg = nowDeg - delt_deg;       // 下降
                if(nowDeg < offDeg){
                  nowDeg = offDeg;
                  anaWR();  
                  gDir = gState_F4;                       // Servo off　になった
                  writeCV();
                  state = ST_IDLE;   
                } else {
                  anaWR();             
                  state = ST_OFF_RUN;                    
                }
              }
            break;
            
      default:
            break;
  }   
}



//---------------------------------------------------------------------------
// DCC速度信号の受信によるイベント 
//---------------------------------------------------------------------------
extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
{
}



//---------------------------------------------------------------------------
//ファンクション信号受信のイベント
//FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
//FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
//前値と比較して変化あったら処理するような作り。
//---------------------------------------------------------------------------
//extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
extern void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  Ndas=1;
  if( FuncGrp == FN_0_4)
  {
    if( gState_F0 != (FuncState & FN_BIT_00))
    {
      //Get Function 0 (FL) state
      gState_F0 = (FuncState & FN_BIT_00);
    }
    if( gState_F1 != (FuncState & FN_BIT_01))
    {
      //Get Function 1 state
      gState_F1 = (FuncState & FN_BIT_01);
    }
    if( gState_F2 != (FuncState & FN_BIT_02))
    {
      gState_F2 = (FuncState & FN_BIT_02);
    }
    if( gState_F3 != (FuncState & FN_BIT_03))
    {
      gState_F3 = (FuncState & FN_BIT_03);
    }
    if( gState_F4 != (FuncState & FN_BIT_04))
    {
      gState_F4 = (FuncState & FN_BIT_04);
    }
  }
#if !defined(SIODBG)
  if( FuncGrp == FN_5_8)
  {
    if( gState_F5 != (FuncState & FN_BIT_05))
    {
      //Get Function 0 (FL) state
      gState_F5 = (FuncState & FN_BIT_05);
    }
    if( gState_F6 != (FuncState & FN_BIT_06))
    {
      //Get Function 1 state
      gState_F6 = (FuncState & FN_BIT_06);
    }
    if( gState_F7 != (FuncState & FN_BIT_07))
    {
      gState_F7 = (FuncState & FN_BIT_07);
    }
    if( gState_F8 != (FuncState & FN_BIT_08))
    {
      gState_F8 = (FuncState & FN_BIT_08);
    }
  }
  if( FuncGrp == FN_9_12)
  {
    if( gState_F9 != (FuncState & FN_BIT_09))
    {
      gState_F9 = (FuncState & FN_BIT_09);
    }
    if( gState_F10 != (FuncState & FN_BIT_10))
    {
      gState_F10 = (FuncState & FN_BIT_10);
    }
    if( gState_F11 != (FuncState & FN_BIT_11))
    {
      gState_F11 = (FuncState & FN_BIT_11);
    }
    if( gState_F12 != (FuncState & FN_BIT_12))
    {
      gState_F12 = (FuncState & FN_BIT_12);
    }
  }
#endif

#if !defined(SIODBG) //////////////////////////////////////////////////
  switch(gCV68_servoAdder){
    case 0:
            gState_Function = gState_F0;
            break;
    case 1:
            gState_Function = gState_F1;
            break;
    case 2:
            gState_Function = gState_F2;
            break;
    case 3:
            gState_Function = gState_F3;
            break;  
    case 4:
            gState_Function = gState_F4;
            break;
    case 5:
            gState_Function = gState_F5;
            break;
    case 6:
            gState_Function = gState_F6;
            break;
    case 7:
            gState_Function = gState_F7;
            break;
    case 8:
            gState_Function = gState_F8;
            break;  
    case 9:
            gState_Function = gState_F9;
            break;
    case 10:
            gState_Function = gState_F10;
            break;
    case 11:
            gState_Function = gState_F11;
            break;
    case 12:
            gState_Function = gState_F12;
            break;
    default:
            break;
  }
#endif             /////////////////////////////////////////////////

}

