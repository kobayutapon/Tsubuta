
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

// ---------------------------------------------------------------
//    システム設定を行う
//   ここは後でEEPROMから設定を行うようにする
//   一番最初でEEPROM未設定か設定がおかしかったら初期化すること
// ---------------------------------------------------------------
void initSystemSettingInfo( void )
{
  g_stSystemSetting.mEEPROMVersion = 0xff;
  g_stSystemSetting.mAutoPowerOnMode = 0x01;
  g_stSystemSetting.mIntervalTime = 10 ;
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

#if 1
  if ( LOW == digitalRead(  TSUBUTA_GPIO0 ) ) {
    g_stSystemSetting.m_PowerKeyEventSignaled = true;
  } else {
    g_stSystemSetting.m_PowerKeyEventSignaled = false;    
  }
#endif
  
}


void setup() {
  // Setup System 
  initSystemSettingInfo();

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
  
  // シリアルポート初期化:
  Serial.begin(UART_BAUDRATE);
  Serial.println("Tsubuta add-on board.");

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

  if (true == isRaspberryPiPowerOn()) {
    SystemRun();
  } else {
    SystemSleep();
  }
  
#if 0
  checkSystemInput();
//  Serial.print(g_stSystemSetting.m_lRunTime);
//  Serial.print(":");
//  Serial.println(g_stSystemSetting.m_TimerCounter);
  
  if ( false == isRaspberryPiPowerOn()) {
    system_sleep(); // RPiの電源が入っていなかったらねる。

    // 経過時間をチェックする。タイマーが入っていたら電源ONにする
    if ( true == isTimerDone() ) {
      setRaspberryPiPower(RPI_POWER_ON);
      g_stSystemSetting.m_TimerCounter = 0;
      Serial.println("Resume.");
    }

    // パワーキーが押されて復帰していたなら電源を入れる
    if ( true == g_stSystemSetting.m_PowerKeyEventSignaled ) {
      setRaspberryPiPower(RPI_POWER_ON);
      g_stSystemSetting.m_PowerKeyEventSignaled = false;
      Serial.println("Resume.");
    }
    
  } else {
    // Raspberry PiからのPower Off制御が入ったら電源を切る
    checkPowerOffFromMainCPU();
    if ( true == g_stSystemSetting.m_bPowerOffEventSignaled ) {
      Serial.println("Power off from Raspberry Pi. 5sec wait...");
      Serial.println("Enter to standby mode.");
      delay(250);
      setRaspberryPiPower(RPI_POWER_OFF); 
      g_stSystemSetting.m_bPowerOffEventSignaled = false;
    }
    if ( true == g_stSystemSetting.m_PowerKeyEventSignaled ) {
    }
  }
#endif
}


// --------------------------------
//   Interupt handler
// --------------------------------
// Interrupt handler for I2C
unsigned long   g_ulI2Ccommand;

void receiveEvent( int howMany  )
{
  while(Wire.available()) {
    g_ulI2Ccommand = Wire.read();
  }

  Serial.print("I2C Recive:");
  Serial.println(g_ulI2Ccommand);
  
}

void requestEvent( )
{
  Serial.println("I2C Send:");
  Wire.write(0x00);
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







