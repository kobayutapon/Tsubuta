
// ------------------------------------------
//   Tsubuta Add-on board Ver.2 firmware
// ------------------------------------------
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Wire.h>

// ------- for Standby mode
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#define TSUBUTA_RPIPOWER  3     //  PD3 (O)  Raspberry Pi Power On/Off
#define TSUBUTA_GPI_POFF  4     //  PD4 (I) Shutdown signal from Raspberry Pi

// ---------------------------------------
//     for I2C
// ---------------------------------------
#define I2C_SLAVE_ADDRESS   8


// ------- Command
#define COMMAND_CLEARTIMER        0x00
#define COMMAND_SETINTERVALTIME   0x01
#define COMMAND_POWEROFF          0x02
#define COMMAND_GET_TIME          0x10
#define COMMAND_GET_RUN_TIME      0x11
#define COMMAND_GET_PWRON_CNT     0x12

#define COMMAND_UNKNOWN           0xFF


//#define BATTERY_ALERT_THREAD      3000
#define BATTERY_ALERT_THREAD      30000
//#define BATTERY_ALERT_THREAD      180

bool g_bPowerOn = true;
unsigned long g_PwrOnCount;
unsigned long g_lRunTime;       // Raspberry Pi側の連続通電時間（秒）
unsigned long g_TimerCounter;   // Interval Timer Counter
unsigned long g_WdtCounter;     // Watch dog Timer Counter = Arduino側の連続通電時間（秒）
unsigned long g_IntervalTime = 900;

typedef struct _command_info_ {
  unsigned char m_ucCommand;
  unsigned char m_ucLength;
  bool          m_bReply;
} COMMAND_TABLE;

typedef struct _buffer_tag_ {
  unsigned char   len;
  unsigned char   rdptr;
  unsigned char   wrptr;  
  unsigned char   buff[32];
} CMD_BUFFER;

CMD_BUFFER    g_bufI2CRecive;
CMD_BUFFER    g_bufI2CSend;

typedef struct _i2c_command_tag_ {
  unsigned char   ucCmd;
  char            cParamLen;
  char            cLen;
  unsigned char   ucBuff[16];
} RCVCMD;

volatile RCVCMD g_RcvCmd;

const COMMAND_TABLE   g_cCommandInfo[] = {
  { 0x00 , 0x01 , true },     // Get Baord Information 
  { 0x01 , 0x05 , false },    // Set Interval Timer |
  { 0x02 , 0x01 , false },    // RPi Power Off |
  { 0x10 , 0x01 , true },    // RPi Power Off |
  { 0x11 , 0x01 , true },    // RPi Power Off |
  { 0x12 , 0x01 , true },    // RPi Power Off |
  { 0xFF , 0xFF, false },   
};


// ====================================================
//   I2Cの送受信バッファ回り
// ====================================================
void PutBufferStatus(CMD_BUFFER *pPtr)
{
  Serial.print("len:"); Serial.println(pPtr->len);
  Serial.print("wptr:"); Serial.println(pPtr->wrptr);
  Serial.print("rptr:"); Serial.println(pPtr->rdptr);
}

void InitCommandBuffer(CMD_BUFFER *pPtr)
{
  pPtr->len = 0;
  pPtr->rdptr = 0;
  pPtr->wrptr = 0;
//  memset((void *)pPtr->buff, 0, sizeof(unsigned char)*32 );

  PutBufferStatus(pPtr);
}

bool SetCommandBuffer( CMD_BUFFER *pPtr, unsigned char c )
{
  Serial.print("chr:"); Serial.println(c);
  
  if ( pPtr->len >= 32 ) return false;    // バッファがいっぱいな場合
  pPtr->buff[pPtr->wrptr] = c;
  pPtr->wrptr++;
  pPtr->wrptr &= 31;
  pPtr->len++;
  PutBufferStatus(pPtr);
  
  return true;
  
}

bool GetCommandBuffer( CMD_BUFFER *pPtr, unsigned char *c , int *pLen )
{
  if ( pPtr->len <= 0 ) return false;    // バッファが空な場合

  *pLen = pPtr->len;
  while( pPtr->len > 0 ) {
    *c++ = pPtr->buff[pPtr->rdptr];
    pPtr->rdptr++;
    pPtr->rdptr &= 31;
    pPtr->len--;

    PutBufferStatus(pPtr);
  }
  
  return true;
  
}

bool IsCommandBufferEmpty( CMD_BUFFER *pPtr )
{
  PutBufferStatus(pPtr);
  return ( pPtr->len <= 0 ) ? true : false ;
}

void ClearCommand( RCVCMD *pCmd )
{
  pCmd->ucCmd = 0;
  pCmd->cParamLen = 0;
  pCmd->cLen = -1;
  memset( (void *)pCmd->ucBuff, 0, sizeof(unsigned char)*16 );

}

int GetCmdParamLen( unsigned char ucCmd )
{
  int nRet = -1;
  int nIndex = 0;
  
  while(g_cCommandInfo[nIndex].m_ucCommand != COMMAND_UNKNOWN ) {
    if ( g_cCommandInfo[nIndex].m_ucCommand == ucCmd ) {
      nRet = g_cCommandInfo[nIndex].m_ucLength - 1;
      break;
    }
    nIndex++;
  }

  return nRet;
}


void ParseCommand( RCVCMD *pCmd )
{
  unsigned char ctmp;
  unsigned long ltmp;
  
  switch( pCmd->ucCmd ) {
    case COMMAND_CLEARTIMER:
      g_TimerCounter = 1;
      Serial.print("Current Timer Count:");
      Serial.println(g_TimerCounter);
      g_TimerCounter = 1;
      Serial.print("Cleared Timer Count:");
      Serial.println(g_TimerCounter);
      break;

    case COMMAND_SETINTERVALTIME:
      Serial.println("CMD:SETINTERVAL");
      ltmp = pCmd->ucBuff[0]; ltmp <<= 8;
      ltmp |= pCmd->ucBuff[1]; ltmp <<= 8;
      ltmp |= pCmd->ucBuff[2]; ltmp <<= 8;
      ltmp |= pCmd->ucBuff[3];
      g_IntervalTime = ltmp;
      Serial.print("Interval Time:");
      Serial.println(g_IntervalTime);
      break;

    case COMMAND_GET_TIME:
      Serial.println("CMD:GET_TIME");
      SetCommandBuffer(&g_bufI2CSend, (g_WdtCounter >> 24) & 0xff );
      SetCommandBuffer(&g_bufI2CSend, (g_WdtCounter >> 16) & 0xff);
      SetCommandBuffer(&g_bufI2CSend, (g_WdtCounter >>  8) & 0xff);
      SetCommandBuffer(&g_bufI2CSend, (g_WdtCounter >>  0) & 0xff);
      break;

    case COMMAND_GET_RUN_TIME:
      Serial.println("CMD:GET_RUN_TIME");
      SetCommandBuffer(&g_bufI2CSend, (g_lRunTime >> 24) & 0xff );
      SetCommandBuffer(&g_bufI2CSend, (g_lRunTime >> 16) & 0xff);
      SetCommandBuffer(&g_bufI2CSend, (g_lRunTime >>  8) & 0xff);
      SetCommandBuffer(&g_bufI2CSend, (g_lRunTime >>  0) & 0xff);
      break;

    case COMMAND_GET_PWRON_CNT:
      SetCommandBuffer(&g_bufI2CSend, (g_PwrOnCount >> 24) & 0xff );
      SetCommandBuffer(&g_bufI2CSend, (g_PwrOnCount >> 16) & 0xff);
      SetCommandBuffer(&g_bufI2CSend, (g_PwrOnCount >>  8) & 0xff);
      SetCommandBuffer(&g_bufI2CSend, (g_PwrOnCount >>  0) & 0xff);
      break;      
      
    default:
      break;  
  }
}


// --------------------------------
//   Interupt handler
// --------------------------------
// Interrupt handler for I2C

bool g_bRequestEvent;

void receiveEvent( int howMany  )
{
  int c;
  int len;
  
  while(Wire.available()) {
    g_bRequestEvent = false;
    c = Wire.read();
    Serial.print("I2C:Rcv ");
    Serial.println(c, HEX);
    if ( g_RcvCmd.cLen < 0 ) {
      g_RcvCmd.ucCmd = (unsigned char)c;
      if ( (g_RcvCmd.cParamLen = GetCmdParamLen(g_RcvCmd.ucCmd)) >= 0 ) {
        g_RcvCmd.cLen = 0;
      }
      Serial.print("Command :");
      Serial.println(g_RcvCmd.ucCmd,HEX);
      Serial.print("Param Len :");
      Serial.println(g_RcvCmd.cParamLen,HEX);
      
      if ( g_RcvCmd.cParamLen == 0x00 ) {
        Serial.println("Parse command");
        ParseCommand((RCVCMD *)&g_RcvCmd);
        ClearCommand((RCVCMD *)&g_RcvCmd);        
      }
    } else {
      g_RcvCmd.ucBuff[g_RcvCmd.cLen] = c;
      g_RcvCmd.cLen++;
      Serial.print("Rcv Len :");
      Serial.println(g_RcvCmd.cLen,HEX);
      if ( g_RcvCmd.cLen >= g_RcvCmd.cParamLen ) {
        // パラメータがすべてそろったのでコマンド解析を行う
        Serial.println("Parse command");
        ParseCommand((RCVCMD *)&g_RcvCmd);
        ClearCommand((RCVCMD *)&g_RcvCmd);
      }
    }
  }  
}

void requestEvent( )
{
  unsigned char c;
  unsigned char buff[32];
  int len;
  
  Serial.println("I2C:Send Request ");
  if ( g_bufI2CSend.len > 0 ) {
    if ( true == GetCommandBuffer( &g_bufI2CSend, buff, &len )) {
      Wire.write(buff, len );
    }
  }
}


void PowerOn()
{
#if 0
  Serial.println("P1");
    delay(10);
  for( int i=0; i<100; i++ ) {
    digitalWrite( TSUBUTA_RPIPOWER, HIGH );
    digitalWrite( TSUBUTA_RPIPOWER, LOW );
    delay(20);
  }

  Serial.println("P2");
    delay(10);
  for( int i=0; i<100; i++ ) {
    digitalWrite( TSUBUTA_RPIPOWER, HIGH );
    digitalWrite( TSUBUTA_RPIPOWER, LOW );
    delay(10);
  }
  
  Serial.println("P3");
    delay(10);
  for( int i=0; i<50; i++ ) {
    digitalWrite( TSUBUTA_RPIPOWER, HIGH );
    digitalWrite( TSUBUTA_RPIPOWER, LOW );
    delay(5);
  }

  Serial.println("P4");
    delay(10);
  for( int i=0; i<50; i++ ) {
    digitalWrite( TSUBUTA_RPIPOWER, HIGH );
    digitalWrite( TSUBUTA_RPIPOWER, LOW );
    delay(1);
  }
  
  Serial.println("P5");
    delay(10);
  for( int i=0; i<50; i++ ) {
    digitalWrite( TSUBUTA_RPIPOWER, HIGH );
    digitalWrite( TSUBUTA_RPIPOWER, LOW );
  }
  
#endif

  Serial.println("P6");
  delay(10);
  digitalWrite( TSUBUTA_RPIPOWER, HIGH );
  g_bPowerOn = true;  
}

// ----------------------------------------------

void setup() {
  // put your setup code here, to run once:
  // シリアルポート初期化:
  Serial.begin(115200);
  Serial.println("Tsubuta add-on board.");
  Serial.println(MCUSR,HEX);

  Serial.println("Initialize GPIO.");
  pinMode( TSUBUTA_RPIPOWER, OUTPUT );          // Raspberry Pi On/Off
  pinMode( TSUBUTA_GPI_POFF, INPUT  );          // Interrupt signal from Raspberry Pi
  pinMode( 10, OUTPUT );                        // BOOTMODE
  pinMode( 9, OUTPUT );                         // BATTERY ALERT(to RPi)
  digitalWrite( TSUBUTA_RPIPOWER, LOW );
  digitalWrite( 9, LOW );

#if 1
  // Set I2C
  // TWIのライブラリの中で5Vにプルアップされてしまう。ここでPullUpをやめているけど、
  // 瞬間的に５Vかかって壊れる可能性あるのでライブラリの中を修正したほうが良い。
  // \hardware\arduino\avr\libraries\Wire\utility\twi.cの
  // activate internal pullups for twi.
  //  digitalWrite(SDA, 1);
  //  digitalWrite(SCL, 1);
  // の部分をコメントアウトしてプルアップ無効化すること
  Serial.println("Initialize I2C.");
  Wire.begin(I2C_SLAVE_ADDRESS);
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  
  Wire.onRequest( requestEvent );
  Wire.onReceive( receiveEvent );
#endif

  // Setup System 
  Serial.println("Buffer initialize.");
  InitCommandBuffer(&g_bufI2CRecive);
  InitCommandBuffer(&g_bufI2CSend);
  ClearCommand((RCVCMD*)&g_RcvCmd);
  
  Serial.println("Get Bootmode.");
#if 1
  digitalWrite( 10, HIGH );
  delay(100);
  int v=0;
  v = analogRead(A7);    
  digitalWrite( 10, LOW );
  Serial.println(v);

  // 817    BOOTMODE4
  // 765    BOOTMODE3
  // 680    BOOTMODE2
  
  // 509    BOOTMODE1
  // 0      BOOTMODE0

  if ( v < 500 ) {
//    g_IntervalTime = 900;     // BOOTMODE0 - 15min
//    g_IntervalTime = 844;     // BOOTMODE0 - 15min  
    g_IntervalTime = 563;     // BOOTMODE0 - 10min  
  } 
  else if ( v >= 500 && v < 600 ) {
//    g_IntervalTime = 1687;     // BOOTMODE1 - 30min 1800sec
    g_IntervalTime = 1125;     // BOOTMODE1 - 30min 1800sec
  }
  else if ( v >= 600 && v < 720 ) {
    g_IntervalTime = 3375; // 3600;     // BOOTMODE2 - 1h
  }
  else if ( v >= 720 && v < 790 ) {
    g_IntervalTime = 3375 * 12 ; //3600 * 12;     // BOOTMODE3 - 12h
  }
  else if ( v >= 790  ) {
    g_IntervalTime = 3375 * 24;     // BOOTMODE3 - 24h
  }
#else
  g_IntervalTime = 563;     // BOOTMODE0 - 10min  
#endif

  g_WdtCounter = 0;
  g_lRunTime = 0;
  g_TimerCounter = 1;
  g_PwrOnCount = 1;
  setup_watchdog(6);    // 250msec timer

  // GPIO Output setting
  Serial.println("RPI P.ON.");
  delay(100);
  PowerOn();

}

void loop() {
  // put your main code here, to run repeatedly:

  if ( g_bPowerOn == true ) {
    Serial.print("Runtime :");
    Serial.println(g_lRunTime);
    if ( g_lRunTime > BATTERY_ALERT_THREAD ) {
      Serial.println("Battery down");
      digitalWrite( 9, HIGH );      
    }
    int n = digitalRead(  TSUBUTA_GPI_POFF );
    if ( n == HIGH ) {
      digitalWrite( 9, LOW );
      digitalWrite( TSUBUTA_RPIPOWER, LOW );    
      g_bPowerOn = false;
    }
    delay(500);
  } else {
    system_sleep();
    
    if ( g_TimerCounter == 0 ) {
      Serial.begin(115200);
      Serial.println("Resume.");
      PowerOn();
      g_PwrOnCount++;
    }
  }
}

// --------------------------------
//   for Standby mode Function
// --------------------------------

// Sleep Function
void system_sleep() {                   // システム停止 
  cbi(ADCSRA,ADEN);                     // ADC 回路電源をOFF (ADC使って無くても120μA消費するため）
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // パワーダウンモード
//  set_sleep_mode(SLEEP_MODE_PWR_SAVE);  // パワーダウンモード
  sleep_enable();
  sleep_mode();                         // ここでスリープに入る
  sleep_disable();                      // WDTがタイムアップしたらここから動作再開 
  wdt_reset();
  sbi(ADCSRA,ADEN);                     // ADC ON
}  //system_sleep

// 割り込みモードでウォッチドッグタイマーを設定。引数による動作は時間は下記
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {           // 
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}  //setup_watchdog

ISR(WDT_vect) {                         // WDTがタイムアップした時に実行される処理
  g_WdtCounter++; 
//  if ( 0 == ( g_WdtCounter % 4 )) {
    if ( g_bPowerOn == true ) {
      g_lRunTime++;           // 連続通電時間は１秒ごとにカウント
    }
    g_TimerCounter++;
    g_TimerCounter %= g_IntervalTime;
//  }        
}




