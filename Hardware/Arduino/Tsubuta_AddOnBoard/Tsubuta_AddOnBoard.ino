
// ------------------------------------------
//   Tsubuta Add-on board Ver.2 firmware
// ------------------------------------------
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Wire.h>

// --------------------------------------
//   Version Information
// --------------------------------------
#define VERSION 0x0001

#define INTERVAL_TIME   60*15   // default interval : 15min

// --------------------------------------
//   EEPROM defenision
// --------------------------------------
//    00h              -- Version. if 0xff, value is not set.
//    01h              -- Power On Mode : 0 - auto power off / 1 : auto power on  
//    02h/03h/04h/05h  -- Interval Timer Count
//    06h              -- GPIO0 Setting 0:Out / 1:In / 2:In-Pullup
//    07h              -- GPIO1 Setting 0:Out / 1:In / 2:In-Pullup
//    08h              -- GPIO2 Setting 0:Out / 1:In / 2:In-Pullup
//    09h              -- GPIO3 Setting 0:Out / 1:In / 2:In-Pullup

// ---------------------------------------
//     for I2C
// ---------------------------------------
#define I2C_SLAVE_ADDRESS   8


// ---------------------------------------
//     for UART
// ---------------------------------------
#define UART_BAUDRATE     115200

// ---------------------------------------
//     ピン定義
// ---------------------------------------
#define TSUBUTA_UART_RXD  0     //  PD0
#define TSUBUTA_UART_TXD  1     //  PD1

#define TSUBUTA_RPIPOWER  3     //  PD3 (O)  Raspberry Pi Power On/Off
#define TSUBUTA_GPI_POFF  4     //  PD4 (I) Shutdown signal from Raspberry Pi

#define TSUBUTA_GPIO0     5     //  PD5  PWM
#define TSUBUTA_GPIO1     6     //  PD6  PWM
#define TSUBUTA_GPIO2     7     //  PD7
#define TSUBUTA_GPIO3     8     //  PB0

#define TSUBUTA_RPI_INT   9     //  PB1 (O) Interrupt signal to Raspberry Pi 
#define TSUBUTA_RTC_INT   10    //  PB2 (I) Interrupt signal from DS1307(RTC Clock)

#define TSUBUTA_AD0       0     //  PC0
#define TSUBUTA_AD1       1     //  PC1
#define TSUBUTA_AD2       2     //  PC2
#define TSUBUTA_AD3       3     //  PC3

#define LED_PIN     13

// ------- Command
#define COMMAND_GETINFO           0x00
#define COMMAND_SETINTERVALTIME   0x01
#define COMMAND_POWEROFF          0x02
#define COMMAND_LOAD_SETTING      0x0E
#define COMMAND_SAVE_SETTING      0x0F

#define COMMAND_SETGPIO1          0x10
#define COMMAND_SETGPIO2          0x11
#define COMMAND_SETGPIO3          0x12
#define COMMAND_SETGPIO4          0x13

#define COMMAND_SET_GPO1          0x20
#define COMMAND_SET_GPO2          0x21
#define COMMAND_SET_GPO3          0x22
#define COMMAND_SET_GPO4          0x23

#define COMMAND_GET_GPI1          0x30
#define COMMAND_GET_GPI2          0x31
#define COMMAND_GET_GPI3          0x32
#define COMMAND_GET_GPI4          0x33

#define COMMAND_GET_ADC1          0x40
#define COMMAND_GET_ADC2          0x41
#define COMMAND_GET_ADC3          0x42
#define COMMAND_GET_ADC4          0x43

#define COMMAND_UNKNOWN           0xFF


// ------- for Standby mode
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

enum {
  STATE_STANDBY,    // Standby mode
  STATE_SLEEP,      // run state when wdt is on.
  STATE_RUN,        // Full run
};

enum {
  RPI_POWER_ON,
  RPI_POWER_OFF
};

typedef struct config_info {
  unsigned char mEEPROMVersion;
  unsigned char mAutoPowerOnMode;
  unsigned long mIntervalTime;
  int           m_GPIO1mode;
  int           m_GPIO2mode;
  int           m_GPIO3mode;
  int           m_GPIO4mode;

  int           m_bPowerStatus;   // RPIの電源ON/OFF状態
  unsigned long m_lRunTime;       // Arduino側の連続通電時間（秒）
  unsigned long m_TimerCounter;   // Interval Timer Counter
  unsigned long m_WdtCounter;     // Watch dog Timer Counter

  bool          m_bPowerOffEventSignaled;   // RPiからのイベント入ったかフラグ
  bool          m_RTCEventSignaled;         // RTCからのイベント入ったかフラグ
  bool          m_PowerKeyEventSignaled;
  
} CONFIG_TSUBUTA;



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

const int GPIO_PinMappping[] = { 5, 6, 7, 8 };
const int ADC_PinMappping[] = { 0, 1, 2, 3 };

const COMMAND_TABLE   g_cCommandInfo[] = {
  { 0x00 , 0x01 , true },     // Get Baord Information 
  { 0x01 , 0x05 , false },    // Set Interval Timer |
  { 0x02 , 0x01 , false },    // RPi Power Off |
  { 0x0E , 0x01 , false },    // Load Settings from EEPROM |
  { 0x0F , 0x01 , false },    // Save settings to EEPROM |
  { 0x10 , 0x02 , false },    // Set GPIO1 State|
  { 0x11 , 0x02 , false },    // Set GPIO2 State|
  { 0x12 , 0x02 , false },    // Set GPIO3 State|
  { 0x13 , 0x02 , false },    // Set GPIO4 State|
  { 0x20 , 0x02 , false },    // Set GPO1 H/L |
  { 0x21 , 0x02 , false },    // Set GPO2 H/L |
  { 0x22 , 0x02 , false },    // Set GPO3 H/L |
  { 0x23 , 0x02 , false },    // Set GPO4 H/L |
  { 0x30 , 0x02 , true },     // Get GPI1 |
  { 0x31 , 0x02 , true },     // Get GPI2 |
  { 0x32 , 0x02 , true },     // Get GPI3 |
  { 0x33 , 0x02 , true },     // Get GPI4 |
  { 0x40 , 0x01 , true },     // Get ADC1 |
  { 0x41 , 0x01 , true },     // Get ADC2 |
  { 0x42 , 0x01 , true },     // Get ADC3 |
  { 0x43 , 0x01 , true },     //| Get ADC4 |
  { 0xFF , 0xFF, false },   
};



volatile CONFIG_TSUBUTA  g_stSystemSetting;

// 割り込み内でのイベント管理用
volatile bool g_bPushPowerButton;           // GPIOに割り当てられたPower Onボタンが押されたフラグ
volatile bool g_bRequestPowerOffFromRpi;    // RPiからの電源OFF要求(GPIO L->H)

void  system_sleep();                   // システム停止 
void  setup_watchdog(int ii);
void  setPowDownMode();                  // CPUをパワーダウンモードに設定

void  receiveEvent( int howMany  );
void  requestEvent( );

void  initSystemSettingInfo( void );
bool  isAutoPowerOn();
bool  isTimerDone();
bool  isRaspberryPiPowerOn();
void  setRaspberryPiPower( int nMode );


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
  memset((void *)pPtr->buff, 0, sizeof(unsigned char)*32 );

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


//  ---- I2C Command Function
int  SetGPIOMode( int pin, int mode )
{
  int PinMode = -1;
  
  if ( mode == 0x00 ) {
    PinMode = INPUT;
  } else if ( mode == 0x01 ) {
    PinMode = INPUT_PULLUP;
  } else if ( mode == 0x02 ) {
    PinMode = OUTPUT;
  }
  if ( PinMode >= 0 ) {
    pinMode(pin, PinMode );
    if (PinMode == OUTPUT ) {
      digitalWrite(pin, LOW);
    }
  }
  return PinMode;
}


void ParseCommand( RCVCMD *pCmd )
{
  unsigned char ctmp;
  unsigned long ltmp;
  
  switch( pCmd->ucCmd ) {
    case COMMAND_GETINFO:
      Serial.println("CMD:GETINFO");
      SetCommandBuffer(&g_bufI2CSend, g_stSystemSetting.mEEPROMVersion);
      break;

    case COMMAND_SETINTERVALTIME:
      Serial.println("CMD:SETINTERVAL");
      ltmp = pCmd->ucBuff[0]; ltmp <<= 8;
      ltmp |= pCmd->ucBuff[1]; ltmp <<= 8;
      ltmp |= pCmd->ucBuff[2]; ltmp <<= 8;
      ltmp |= pCmd->ucBuff[3];
      g_stSystemSetting.mIntervalTime = ltmp;
      Serial.print("Interval Time:");
      Serial.println(g_stSystemSetting.mIntervalTime);
      break;

    case COMMAND_LOAD_SETTING:
      Serial.println("CMD:LOADSETTING");
      LoadSystemSettings();
      break;

    case COMMAND_SAVE_SETTING:
      Serial.println("CMD:SAVESETTING");
      SaveSystemSettings();
      break;

    case COMMAND_SETGPIO1:
      Serial.println("CMD:SETGPIO1_CNF");
      ltmp = SetGPIOMode(5, pCmd->ucBuff[0] );
      if ( ltmp >= 0 ) {
        g_stSystemSetting.m_GPIO1mode = ltmp;
      }
      break;
      
    case COMMAND_SETGPIO2:
      Serial.println("CMD:SETGPIO2_CNF");
      ltmp = SetGPIOMode(6, pCmd->ucBuff[0] );
      if ( ltmp >= 0 ) {
        g_stSystemSetting.m_GPIO1mode = ltmp;
      }
      break;
      
    case COMMAND_SETGPIO3:
      Serial.println("CMD:SETGPIO3_CNF");
      ltmp = SetGPIOMode(7, pCmd->ucBuff[0] );
      if ( ltmp >= 0 ) {
        g_stSystemSetting.m_GPIO1mode = ltmp;
      }
      break;
      
    case COMMAND_SETGPIO4:
      Serial.println("CMD:SETGPIO4_CNF");
      ltmp = SetGPIOMode(8, pCmd->ucBuff[0] );
      if ( ltmp >= 0 ) {
        g_stSystemSetting.m_GPIO1mode = ltmp;
      }
      break;
      
      break;

    case COMMAND_SET_GPO1:
    case COMMAND_SET_GPO2:
    case COMMAND_SET_GPO3:
    case COMMAND_SET_GPO4:
      ctmp = pCmd->ucCmd - COMMAND_SET_GPO1;
      Serial.print("CMD:SET_GPO_STATE"); Serial.println(ctmp);
      if ( pCmd->ucBuff[0] == 0x00 ) {
        digitalWrite( GPIO_PinMappping[ctmp], LOW );
      } else {
        digitalWrite( GPIO_PinMappping[ctmp], HIGH );
      }
      break;

    case COMMAND_GET_GPI1:
    case COMMAND_GET_GPI2:
    case COMMAND_GET_GPI3:
    case COMMAND_GET_GPI4:
      ctmp = pCmd->ucCmd - COMMAND_SET_GPO1;
      Serial.print("CMD:GET_GPI_STATE"); Serial.println(ctmp);
      ltmp = digitalRead( GPIO_PinMappping[ctmp] );
      SetCommandBuffer(&g_bufI2CSend, (unsigned char)ltmp );
      break;

    case COMMAND_GET_ADC1:
    case COMMAND_GET_ADC2:
    case COMMAND_GET_ADC3:
    case COMMAND_GET_ADC4:
      ctmp = pCmd->ucCmd - COMMAND_GET_ADC1;
      Serial.print("CMD:GETADC_STATE"); Serial.println(ctmp);
      ltmp = analogRead( ADC_PinMappping[ctmp] );
      Serial.print("ADC:");
      Serial.println(ltmp, HEX );
      SetCommandBuffer(&g_bufI2CSend, (unsigned char)( (ltmp >> 8 ) & 0xff ));
      SetCommandBuffer(&g_bufI2CSend, (unsigned char)(  ltmp & 0xff ));
    
      break;

    default:
      break;  
  }
}


// ---------------------------------------------------------------
//    システム設定を行う
//   ここは後でEEPROMから設定を行うようにする
//   一番最初でEEPROM未設定か設定がおかしかったら初期化すること
// ---------------------------------------------------------------
void dumpEEPROM( void )
{
  Serial.println("--------");
  for( int i=0; i<10; i++ ) {
    Serial.println(EEPROM.read(i), HEX);
  }
}

void initSystemSettingInfo( void )
{
  g_stSystemSetting.mEEPROMVersion = VERSION;
  g_stSystemSetting.mAutoPowerOnMode = 0x01;
  g_stSystemSetting.mIntervalTime = INTERVAL_TIME ;
  g_stSystemSetting.m_GPIO1mode = INPUT_PULLUP;
  g_stSystemSetting.m_GPIO2mode = INPUT;
  g_stSystemSetting.m_GPIO3mode = INPUT;
  g_stSystemSetting.m_GPIO4mode = INPUT;

  g_stSystemSetting.m_bPowerStatus = false;
  g_stSystemSetting.m_lRunTime = 0;
  g_stSystemSetting.m_TimerCounter = 0;
  g_stSystemSetting.m_WdtCounter = 0;
  
  g_stSystemSetting.m_bPowerOffEventSignaled = false;
  g_stSystemSetting.m_RTCEventSignaled = false;
  g_stSystemSetting.m_PowerKeyEventSignaled = false;
  
}

void PutSystemSettings( void )
{
  unsigned long tmp;
  Serial.print("VERSION:"); Serial.println(EEPROM.read(0), HEX );
  Serial.print("AUTO-PW-ON:"); Serial.println(EEPROM.read(1), HEX );
  Serial.print("GPIO1:"); Serial.println(EEPROM.read(6), HEX ); delay(10);
  Serial.print("GPIO2:"); Serial.println(EEPROM.read(7), HEX ); delay(10);
  Serial.print("GPIO3:"); Serial.println(EEPROM.read(8), HEX ); delay(10);
  Serial.print("GPIO4:"); Serial.println(EEPROM.read(9), HEX ); delay(10);
  tmp = EEPROM.read(2) & 0xff; tmp <<= 8;
  tmp |= EEPROM.read(3) & 0xff; tmp <<= 8;
  tmp |= EEPROM.read(4) & 0xff; tmp <<= 8;
  tmp |= EEPROM.read(5) & 0xff;
  Serial.print("INTERVAL:"); Serial.println( tmp ); delay(300);

  
}
void LoadSystemSettings( void )
{
  unsigned long tmp;
  if ( EEPROM.read(0) == 0xFF ) {
    Serial.println("EEPROM not defined.USe default."); delay(10);
    initSystemSettingInfo();
    SaveSystemSettings();
    return;
  }
  g_stSystemSetting.mEEPROMVersion = EEPROM.read(0);
  g_stSystemSetting.mAutoPowerOnMode = EEPROM.read(1);
  if ( g_stSystemSetting.mAutoPowerOnMode != 0 && g_stSystemSetting.mAutoPowerOnMode != 1 ) {
    Serial.println("Illigal value:AUTO-POWER-ON. Use default value."); delay(10);
    g_stSystemSetting.mAutoPowerOnMode = 0x01;
  }
  tmp = EEPROM.read(2) & 0xff; tmp <<= 8;
  tmp |= EEPROM.read(3) & 0xff; tmp <<= 8;
  tmp |= EEPROM.read(4) & 0xff; tmp <<= 8;
  tmp |= EEPROM.read(5) & 0xff;
  
  g_stSystemSetting.mIntervalTime = tmp;
  
  g_stSystemSetting.m_GPIO1mode = EEPROM.read(6);
  g_stSystemSetting.m_GPIO2mode = EEPROM.read(7);
  g_stSystemSetting.m_GPIO3mode = EEPROM.read(8);
  g_stSystemSetting.m_GPIO4mode = EEPROM.read(9);
  
}
void SaveSystemSettings( void )
{

  EEPROM.write(0, VERSION);
  EEPROM.write(1, (g_stSystemSetting.mAutoPowerOnMode == 0)?0:1);
  EEPROM.write(2, (g_stSystemSetting.mIntervalTime >> 24 ) & 0xFF );
  EEPROM.write(3, (g_stSystemSetting.mIntervalTime >> 16 ) & 0xFF );
  EEPROM.write(4, (g_stSystemSetting.mIntervalTime >>  8 ) & 0xFF );
  EEPROM.write(5, (g_stSystemSetting.mIntervalTime ) & 0xFF );
  EEPROM.write(6, g_stSystemSetting.m_GPIO1mode & 0xff );
  EEPROM.write(7, g_stSystemSetting.m_GPIO2mode & 0xff );
  EEPROM.write(8, g_stSystemSetting.m_GPIO3mode & 0xff );
  EEPROM.write(9, g_stSystemSetting.m_GPIO4mode & 0xff );

}

void ClearSystemSettings( void )
{
  initSystemSettingInfo();
  SaveSystemSettings();  
}

// ----------------------------------------------
// 自動パワーON設定かをチェック
bool isAutoPowerOn()
{
  return (g_stSystemSetting.mAutoPowerOnMode == 0x00 ) ? false:true;
}

// ----------------------------------------------
// 寝ている時間がInterval Timer設定を経過したかをチェック
bool  isTimerDone()
{
  if ( g_stSystemSetting.mIntervalTime == 0 ) return false;
  if ( g_stSystemSetting.m_TimerCounter > g_stSystemSetting.mIntervalTime - 1) {
    return true;
  } else {
    return false;
  }
}


// ----------------------------------------------
// Raspberry Pi側の電源がONしているかをチェック
bool isRaspberryPiPowerOn()
{
  return g_stSystemSetting.m_bPowerStatus;  
}

// ----------------------------------------------
// RPiの電源ON/OFFを行う
void setRaspberryPiPower( int nMode )
{
  if ( nMode  == RPI_POWER_ON ) {
    digitalWrite( TSUBUTA_RPIPOWER, HIGH);
    Serial.begin(UART_BAUDRATE);
    Wire.begin(I2C_SLAVE_ADDRESS);
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
    
    g_stSystemSetting.m_bPowerStatus = true;

  } else {
    digitalWrite( TSUBUTA_RPIPOWER, LOW);    
    g_stSystemSetting.m_bPowerStatus = false;
  }
}

// ----------------------------------------------
// GPIのチェックを行う
void checkPowerOffFromMainCPU()
{
  if ( HIGH == digitalRead(  TSUBUTA_GPI_POFF ) ) {
    g_stSystemSetting.m_bPowerOffEventSignaled = true;
  }
}
void checkSystemInput(  )
{

  if ( LOW == digitalRead(  TSUBUTA_GPIO0 ) ) {
    g_stSystemSetting.m_PowerKeyEventSignaled = true;
  } else {
    g_stSystemSetting.m_PowerKeyEventSignaled = false;    
  }
}


void setup() {
  
  // シリアルポート初期化:
  Serial.begin(UART_BAUDRATE);
  Serial.println("Tsubuta add-on board.");

  // Setup System 
  InitCommandBuffer(&g_bufI2CRecive);
  InitCommandBuffer(&g_bufI2CSend);
  ClearCommand((RCVCMD*)&g_RcvCmd);


//  initSystemSettingInfo();
  LoadSystemSettings();
  PutSystemSettings();

  // Set GPIO Settings
  pinMode( TSUBUTA_RPIPOWER, OUTPUT );          // Raspberry Pi On/Off
  pinMode( TSUBUTA_GPI_POFF, INPUT  );          // Interrupt signal from Raspberry Pi
  pinMode( TSUBUTA_RPI_INT,  OUTPUT  );         // Interrupt signal to Raspberry Pi
  pinMode( TSUBUTA_RTC_INT,  INPUT  );          // Interrupt signal from RTC
  
  pinMode( TSUBUTA_GPIO0,  g_stSystemSetting.m_GPIO1mode  );    
  pinMode( TSUBUTA_GPIO1,  g_stSystemSetting.m_GPIO2mode  );    
  pinMode( TSUBUTA_GPIO2,  g_stSystemSetting.m_GPIO3mode  );    
  pinMode( TSUBUTA_GPIO3,  g_stSystemSetting.m_GPIO4mode  );    

  // GPIO Output setting
  digitalWrite( TSUBUTA_RPIPOWER, LOW );
  digitalWrite( TSUBUTA_RPI_INT, LOW );
  

  // Set I2C
  // TWIのライブラリの中で5Vにプルアップされてしまう。ここでPullUpをやめているけど、
  // 瞬間的に５Vかかって壊れる可能性あるのでライブラリの中を修正したほうが良い。
  // \hardware\arduino\avr\libraries\Wire\utility\twi.cの
  // activate internal pullups for twi.
  //  digitalWrite(SDA, 1);
  //  digitalWrite(SCL, 1);
  // の部分をコメントアウトしてプルアップ無効化すること
  Wire.begin(I2C_SLAVE_ADDRESS);
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  
  Wire.onRequest( requestEvent );
  Wire.onReceive( receiveEvent );

  // Raspberry Pi Power On チェック 
  if ( isAutoPowerOn() == true ) {
    Serial.println("Auto Power On.");
    setRaspberryPiPower(RPI_POWER_ON);
  } else {
    Serial.println("Auto Power Off.");
    setRaspberryPiPower(RPI_POWER_OFF);    
  }
  delay(100);
  setup_watchdog(4);  // Watch dog timer : 250msec
  Serial.println("Initialize done.");
  
}


void SystemSleep()
{
  
  if (true == isRaspberryPiPowerOn()) return;
  
  while( true ) {
    system_sleep();
    checkSystemInput();
    if ( true == isTimerDone() ) {
      setRaspberryPiPower(RPI_POWER_ON);
      Serial.println(g_stSystemSetting.m_TimerCounter);
      g_stSystemSetting.m_TimerCounter = 0;
      Serial.println("Resume:Timer");
      return;
    }
    if ( true == g_stSystemSetting.m_PowerKeyEventSignaled ) {
      setRaspberryPiPower(RPI_POWER_ON);
      g_stSystemSetting.m_PowerKeyEventSignaled = false;
      Serial.println("Resume:Power Button");
      return;
    }
  }
}

void SystemRun()
{
  while(true) {
    checkPowerOffFromMainCPU();
    if ( true == g_stSystemSetting.m_bPowerOffEventSignaled ) {
      Serial.println("Power off from Raspberry Pi. 5sec wait...");
      Serial.println("Enter to standby mode.");
      Serial.println(g_stSystemSetting.m_TimerCounter);
      delay(250);
      setRaspberryPiPower(RPI_POWER_OFF); 
      g_stSystemSetting.m_bPowerOffEventSignaled = false;
    }
    return;    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
#if 0
  if ( IsCommandBufferEmpty(&g_bufI2CRecive) == false ) {
    
  }
#endif
  if (true == isRaspberryPiPowerOn()) {
    SystemRun();
  } else {
    SystemSleep();
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
      if ( g_RcvCmd.cParamLen == 0x00 ) {
        Serial.println("Parse command");
        ParseCommand((RCVCMD *)&g_RcvCmd);
        ClearCommand((RCVCMD *)&g_RcvCmd);        
      }
    } else {
      g_RcvCmd.ucBuff[g_RcvCmd.cLen] = c;
      g_RcvCmd.cLen++;
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

//  if (true == GetCommandBuffer( &g_bufI2CSend, &c)) {
//    Wire.write( c );
//    Serial.println(c, HEX);
//  }
  if ( g_bufI2CSend.len > 0 ) {
    if ( true == GetCommandBuffer( &g_bufI2CSend, buff, &len )) {
      Wire.write(buff, len );
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
  sleep_enable();
  sleep_mode();                         // ここでスリープに入る
  sleep_disable();                      // WDTがタイムアップしたらここから動作再開 
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

void setPowDownMode(){                  // CPUをパワーダウンモードに設定
  // CPU Sleep Modes          参考にしたプログラムに入っていたがこのルーチンは使用しない
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  // パワーダウンモードへ設定
  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode
}  //  setPowDownMode

ISR(WDT_vect) {                         // WDTがタイムアップした時に実行される処理
  g_stSystemSetting.m_WdtCounter++; 
  if ( 0 == ( g_stSystemSetting.m_WdtCounter % 4 )) {
    g_stSystemSetting.m_lRunTime++;           // 連続通電時間は１秒ごとにカウント
    g_stSystemSetting.m_TimerCounter++;
    g_stSystemSetting.m_TimerCounter %= ( g_stSystemSetting.mIntervalTime + 1);
  }        
}







