# つぶ田 Add-On ボード仕様
## Hardware
* Arduino互換　ATMega328P
* 外部I/F
 I2C/UART(Arduino)/UART(Raspberry Pi)

### EEPROM 仕様
|Address|Description|
|:------|-----|
|00h              | Version. if 0xff, value is not set.|
|01h              | Power On Mode : 0 - auto power off / 1 : auto power on |  
|02h/03h/04h/05h  | Interval Timer Count |
|06h              | GPIO0 Setting 0:Out / 1:In / 2:In-Pullup |
|07h              | GPIO1 Setting 0:Out / 1:In / 2:In-Pullup |
|08h              | GPIO2 Setting 0:Out / 1:In / 2:In-Pullup |
|09h              | GPIO3 Setting 0:Out / 1:In / 2:In-Pullup |

### GPIO Pin assign
|Port|Pin No.|Description|
|:---|-------|-----------|
|TSUBUTA_UART_RXD  | 0 |  PD0 |
|TSUBUTA_UART_TXD  | 1 |  PD1 |
|TSUBUTA_RPIPOWER  | 3 |  PD3 (O)  Raspberry Pi Power On/Off |
|TSUBUTA_GPI_POFF  | 4 |  PD4 (I) Shutdown signal from Raspberry Pi |
|TSUBUTA_GPIO0     | 5 |  PD5  PWM |
|TSUBUTA_GPIO1     | 6 |  PD6  PWM |
|TSUBUTA_GPIO2     | 7 |  PD7 |
|TSUBUTA_GPIO3     | 8 |  PB0 |
|TSUBUTA_RPI_INT   | 9 |  PB1 (O) Interrupt signal to Raspberry Pi |
|TSUBUTA_RTC_INT   | 10 |  PB2 (I) Interrupt signal from DS1307(RTC Clock) |
|TSUBUTA_AD0       | A0  |  PC0 |
|TSUBUTA_AD1       | A1  |  PC1 |
|TSUBUTA_AD2       | A2  |  PC2 |
|TSUBUTA_AD3       | A3  |  PC3 |

## Firmware仕様


### I/F
* I2C
Slave Address 0x08  
Raspberry Piと接続。電源は3.3V。  

### プログラム仕様
* I2C Command  
1バイトで送受信する。

|Command|Parameter|Length|Decsription|
|:------|---------|------|-----------|
| 0x00 | N/A | 0x01 | Get Baord Information |
| 0x01 | Time(4Byte)| 0x05 | Set Interval Timer |
| 0x02 | N/A | 0x01 | RPi Power Off |
| 0x0F | N/A | 0x01 | Save settings to EEPROM |
| 0x10 | State ( 1Byte) |0x02| Set GPIO1 State|
| 0x11 | State ( 1Byte) |0x02| Set GPIO2 State|
| 0x12 | State ( 1Byte) |0x02| Set GPIO3 State|
| 0x13 | State ( 1Byte) |0x02| Set GPIO4 State|
| 0x20 | H/L ( 1Byte) | 0x02 | Set GPO1 H/L |
| 0x21 | H/L ( 1Byte) | 0x02 | Set GPO2 H/L |
| 0x22 | H/L ( 1Byte) | 0x02 | Set GPO3 H/L |
| 0x23 | H/L ( 1Byte) | 0x02 | Set GPO4 H/L |
| 0x30 | H/L ( 1Byte) | 0x02 | Get GPI1 |
| 0x31 | H/L ( 1Byte) | 0x02 | Get GPI2 |
| 0x32 | H/L ( 1Byte) | 0x02 | Get GPI3 |
| 0x33 | H/L ( 1Byte) | 0x02 | Get GPI4 |
| 0x40 | N/A | 0x01 | Get ADC1 |
| 0x41 | N/A | 0x01 | Get ADC2 |
| 0x42 | N/A | 0x01 | Get ADC3 |
| 0x43 | N/A | 0x01 | Get ADC4 |
