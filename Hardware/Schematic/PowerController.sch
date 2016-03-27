EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Raspberry_PI_B+
LIBS:ds1307_pcf8583
LIBS:PowerController-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA328-P IC2
U 1 1 559BD818
P 4850 3200
F 0 "IC2" H 4100 4450 40  0000 L BNN
F 1 "ATMEGA328-P" H 5250 1800 40  0000 L BNN
F 2 "Housings_DIP:DIP-28_W7.62mm_LongPads" H 4850 3200 30  0000 C CIN
F 3 "" H 4850 3200 60  0000 C CNN
	1    4850 3200
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 559BD994
P 6500 2700
F 0 "Y1" H 6500 2850 50  0000 C CNN
F 1 "16MHz" H 6500 2550 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 6500 2700 60  0001 C CNN
F 3 "" H 6500 2700 60  0000 C CNN
	1    6500 2700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR01
U 1 1 559BDA51
P 7600 4250
F 0 "#PWR01" H 7600 4000 50  0001 C CNN
F 1 "GND" H 7600 4100 50  0000 C CNN
F 2 "" H 7600 4250 60  0000 C CNN
F 3 "" H 7600 4250 60  0000 C CNN
	1    7600 4250
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BCE Q1
U 1 1 559BDDE4
P 7800 2100
F 0 "Q1" H 8100 2150 50  0000 R CNN
F 1 "Q_NPN_BCE" H 8400 2050 50  0000 R CNN
F 2 "Power_Integrations:TO-220" H 8000 2200 29  0001 C CNN
F 3 "" H 7800 2100 60  0000 C CNN
	1    7800 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 559BDEFA
P 7900 2400
F 0 "#PWR02" H 7900 2150 50  0001 C CNN
F 1 "GND" H 7900 2250 50  0000 C CNN
F 2 "" H 7900 2400 60  0000 C CNN
F 3 "" H 7900 2400 60  0000 C CNN
	1    7900 2400
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 559BE1CB
P 3000 2000
F 0 "#PWR03" H 3000 1850 50  0001 C CNN
F 1 "+5V" H 3000 2140 50  0000 C CNN
F 2 "" H 3000 2000 60  0000 C CNN
F 3 "" H 3000 2000 60  0000 C CNN
	1    3000 2000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR04
U 1 1 559BE1F3
P 7500 3000
F 0 "#PWR04" H 7500 2850 50  0001 C CNN
F 1 "+5V" H 7500 3140 50  0000 C CNN
F 2 "" H 7500 3000 60  0000 C CNN
F 3 "" H 7500 3000 60  0000 C CNN
	1    7500 3000
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 559BE211
P 7500 3350
F 0 "R1" V 7580 3350 50  0000 C CNN
F 1 "10K" V 7500 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7430 3350 30  0001 C CNN
F 3 "" H 7500 3350 30  0000 C CNN
	1    7500 3350
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P2
U 1 1 559BE3DF
P 8000 3750
F 0 "P2" H 8000 4000 50  0000 C CNN
F 1 "CONN_AVR_UART" V 8100 3750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 8000 3750 60  0001 C CNN
F 3 "" H 8000 3750 60  0000 C CNN
	1    8000 3750
	1    0    0    -1  
$EndComp
Text Label 8200 4650 0    60   ~ 0
RPPWR
$Comp
L +5V #PWR05
U 1 1 559BE978
P 10450 2000
F 0 "#PWR05" H 10450 1850 50  0001 C CNN
F 1 "+5V" H 10450 2140 50  0000 C CNN
F 2 "" H 10450 2000 60  0000 C CNN
F 3 "" H 10450 2000 60  0000 C CNN
	1    10450 2000
	1    0    0    -1  
$EndComp
Text Label 10450 4550 0    60   ~ 0
RPPWR_GND
Text Label 8500 3300 0    60   ~ 0
RPPWR_GND
$Comp
L GND #PWR06
U 1 1 559BF7C3
P 3700 4950
F 0 "#PWR06" H 3700 4700 50  0001 C CNN
F 1 "GND" H 3700 4800 50  0000 C CNN
F 2 "" H 3700 4950 60  0000 C CNN
F 3 "" H 3700 4950 60  0000 C CNN
	1    3700 4950
	1    0    0    -1  
$EndComp
Text Label 8500 2600 0    60   ~ 0
PWR_OFF
$Comp
L CONN_01X04 P3
U 1 1 559BF366
P 8950 1250
F 0 "P3" H 8950 1500 50  0000 C CNN
F 1 "CONN_I2C" V 9050 1250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 8950 1250 60  0001 C CNN
F 3 "" H 8950 1250 60  0000 C CNN
	1    8950 1250
	0    -1   -1   0   
$EndComp
NoConn ~ 10050 2800
NoConn ~ 10050 2900
NoConn ~ 10050 3100
NoConn ~ 10050 3200
NoConn ~ 10050 3300
NoConn ~ 9550 2800
NoConn ~ 9550 2400
$Comp
L CONN_01X02 P1
U 1 1 559BFAD4
P 1450 1750
F 0 "P1" H 1450 1900 50  0000 C CNN
F 1 "CONN_POWER" V 1550 1750 50  0000 C CNN
F 2 "Connect:AK300-2" H 1450 1750 60  0001 C CNN
F 3 "" H 1450 1750 60  0000 C CNN
	1    1450 1750
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR07
U 1 1 559BFB3E
P 2300 1350
F 0 "#PWR07" H 2300 1200 50  0001 C CNN
F 1 "+5V" H 2300 1490 50  0000 C CNN
F 2 "" H 2300 1350 60  0000 C CNN
F 3 "" H 2300 1350 60  0000 C CNN
	1    2300 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 559BFB64
P 2300 2050
F 0 "#PWR08" H 2300 1800 50  0001 C CNN
F 1 "GND" H 2300 1900 50  0000 C CNN
F 2 "" H 2300 2050 60  0000 C CNN
F 3 "" H 2300 2050 60  0000 C CNN
	1    2300 2050
	1    0    0    -1  
$EndComp
Text Label 7900 1550 0    60   ~ 0
RPPWR_GND
$Comp
L PWR_FLAG #FLG09
U 1 1 559D2ABE
P 1900 1400
F 0 "#FLG09" H 1900 1495 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 1580 50  0000 C CNN
F 2 "" H 1900 1400 60  0000 C CNN
F 3 "" H 1900 1400 60  0000 C CNN
	1    1900 1400
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 559D30F5
P 6650 2200
F 0 "R2" V 6730 2200 50  0000 C CNN
F 1 "1.8K" V 6650 2200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6580 2200 30  0001 C CNN
F 3 "" H 6650 2200 30  0000 C CNN
	1    6650 2200
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 559D3164
P 7200 2450
F 0 "R3" V 7280 2450 50  0000 C CNN
F 1 "3.3K" V 7200 2450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7130 2450 30  0001 C CNN
F 3 "" H 7200 2450 30  0000 C CNN
	1    7200 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 559D32CB
P 7200 3000
F 0 "#PWR010" H 7200 2750 50  0001 C CNN
F 1 "GND" H 7200 2850 50  0000 C CNN
F 2 "" H 7200 3000 60  0000 C CNN
F 3 "" H 7200 3000 60  0000 C CNN
	1    7200 3000
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 559E5E29
P 6800 2550
F 0 "C2" H 6825 2650 50  0000 L CNN
F 1 "22p" H 6825 2450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6838 2400 30  0001 C CNN
F 3 "" H 6800 2550 60  0000 C CNN
	1    6800 2550
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 559E6391
P 2100 1750
F 0 "C1" H 2125 1850 50  0000 L CNN
F 1 "22u" H 2125 1650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2138 1600 30  0001 C CNN
F 3 "" H 2100 1750 60  0000 C CNN
	1    2100 1750
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X20 P5
U 1 1 559E6A92
P 9800 3050
F 0 "P5" H 9800 4100 50  0000 C CNN
F 1 "CONN_02X20" V 9800 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x20" H 9800 2100 60  0001 C CNN
F 3 "" H 9800 2100 60  0000 C CNN
	1    9800 3050
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P4
U 1 1 559E8062
P 10900 2500
F 0 "P4" H 10900 2700 50  0000 C CNN
F 1 "CONN_UART" V 11000 2500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 10900 2500 60  0001 C CNN
F 3 "" H 10900 2500 60  0000 C CNN
	1    10900 2500
	1    0    0    -1  
$EndComp
NoConn ~ 10050 3400
NoConn ~ 10050 3600
NoConn ~ 10050 3800
NoConn ~ 10050 3900
NoConn ~ 10050 4000
NoConn ~ 9550 2900
NoConn ~ 9550 3000
NoConn ~ 9550 3100
NoConn ~ 9550 3200
NoConn ~ 9550 3400
NoConn ~ 9550 3500
NoConn ~ 9550 3700
NoConn ~ 9550 3600
NoConn ~ 9550 3800
NoConn ~ 9550 3900
$Comp
L R R4
U 1 1 559EAFE4
P 9250 1900
F 0 "R4" V 9330 1900 50  0000 C CNN
F 1 "10K" V 9250 1900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9180 1900 30  0001 C CNN
F 3 "" H 9250 1900 30  0000 C CNN
	1    9250 1900
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 559EB065
P 9450 1900
F 0 "R5" V 9530 1900 50  0000 C CNN
F 1 "10K" V 9450 1900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9380 1900 30  0001 C CNN
F 3 "" H 9450 1900 30  0000 C CNN
	1    9450 1900
	1    0    0    -1  
$EndComp
Text Label 8500 2300 0    60   ~ 0
A_SCL
Text Label 8500 2200 0    60   ~ 0
A_SDA
Text Label 6400 3350 0    60   ~ 0
A_SDA
Text Label 6400 3450 0    60   ~ 0
A_SCL
$Comp
L C C3
U 1 1 559E5E60
P 6800 2850
F 0 "C3" H 6825 2950 50  0000 L CNN
F 1 "22p" H 6825 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6838 2700 30  0001 C CNN
F 3 "" H 6800 2850 60  0000 C CNN
	1    6800 2850
	0    1    1    0   
$EndComp
$Comp
L CONN_02X03 P6
U 1 1 56488F63
P 6100 1200
F 0 "P6" H 6100 1400 50  0000 C CNN
F 1 "ICSP" H 6100 1000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 6100 0   60  0001 C CNN
F 3 "" H 6100 0   60  0000 C CNN
	1    6100 1200
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR011
U 1 1 56489253
P 6450 1000
F 0 "#PWR011" H 6450 850 50  0001 C CNN
F 1 "+5V" H 6450 1140 50  0000 C CNN
F 2 "" H 6450 1000 60  0000 C CNN
F 3 "" H 6450 1000 60  0000 C CNN
	1    6450 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 56489330
P 6450 1450
F 0 "#PWR012" H 6450 1200 50  0001 C CNN
F 1 "GND" H 6450 1300 50  0000 C CNN
F 2 "" H 6450 1450 60  0000 C CNN
F 3 "" H 6450 1450 60  0000 C CNN
	1    6450 1450
	1    0    0    -1  
$EndComp
Text Label 4750 1300 0    60   ~ 0
RST
Text Label 7100 3300 3    60   ~ 0
RST
$Comp
L R R6
U 1 1 564B4DF7
P 7350 2100
F 0 "R6" V 7430 2100 50  0000 C CNN
F 1 "10K" V 7350 2100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7280 2100 30  0001 C CNN
F 3 "" H 7350 2100 30  0000 C CNN
	1    7350 2100
	0    1    1    0   
$EndComp
$Comp
L DS1307Z IC1
U 1 1 56CD8159
P 8000 5650
F 0 "IC1" H 7700 6100 50  0000 L BNN
F 1 "DS1307Z" H 7700 5150 50  0000 L BNN
F 2 "ds1307_pcf8583:ds1307_pcf8583-SOIC8" H 8000 5800 50  0001 C CNN
F 3 "" H 8000 5650 60  0000 C CNN
	1    8000 5650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 56CD85B6
P 7350 6050
F 0 "#PWR013" H 7350 5800 50  0001 C CNN
F 1 "GND" H 7350 5900 50  0000 C CNN
F 2 "" H 7350 6050 50  0000 C CNN
F 3 "" H 7350 6050 50  0000 C CNN
	1    7350 6050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 56CD86A7
P 7350 5250
F 0 "#PWR014" H 7350 5100 50  0001 C CNN
F 1 "+5V" H 7350 5390 50  0000 C CNN
F 2 "" H 7350 5250 50  0000 C CNN
F 3 "" H 7350 5250 50  0000 C CNN
	1    7350 5250
	1    0    0    -1  
$EndComp
Text Label 8850 5350 0    60   ~ 0
A_SCL
Text Label 8850 5550 0    60   ~ 0
A_SDA
$Comp
L Battery BT1
U 1 1 56CD9864
P 8900 6150
F 0 "BT1" H 9000 6200 50  0000 L CNN
F 1 "CR2303" H 9000 6100 50  0000 L CNN
F 2 "Connect:CR2032H" V 8900 6190 50  0001 C CNN
F 3 "" V 8900 6190 50  0000 C CNN
	1    8900 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 56CD998E
P 8900 6300
F 0 "#PWR015" H 8900 6050 50  0001 C CNN
F 1 "GND" H 8900 6150 50  0000 C CNN
F 2 "" H 8900 6300 50  0000 C CNN
F 3 "" H 8900 6300 50  0000 C CNN
	1    8900 6300
	1    0    0    -1  
$EndComp
Text Label 8850 5750 0    60   ~ 0
A_SQW
Text Label 6500 2400 0    60   ~ 0
A_SQW
$Comp
L GND #PWR016
U 1 1 56CE2232
P 3750 2750
F 0 "#PWR016" H 3750 2500 50  0001 C CNN
F 1 "GND" H 3750 2600 50  0000 C CNN
F 2 "" H 3750 2750 50  0000 C CNN
F 3 "" H 3750 2750 50  0000 C CNN
	1    3750 2750
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG017
U 1 1 559D2B66
P 1900 2300
F 0 "#FLG017" H 1900 2395 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 2480 50  0000 C CNN
F 2 "" H 1900 2300 60  0000 C CNN
F 3 "" H 1900 2300 60  0000 C CNN
	1    1900 2300
	1    0    0    1   
$EndComp
$Comp
L Crystal Y2
U 1 1 56CEF583
P 6850 5650
F 0 "Y2" H 6850 5800 50  0000 C CNN
F 1 "32KHz" H 6850 5500 50  0000 C CNN
F 2 "Crystals:Crystal_Round_Horizontal_3mm" H 6850 5650 50  0001 C CNN
F 3 "" H 6850 5650 50  0000 C CNN
	1    6850 5650
	0    -1   -1   0   
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 56CF126F
P 5300 1000
F 0 "SW2" H 5450 1110 50  0000 C CNN
F 1 "SW_PUSH" H 5300 920 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVPBF" H 5300 1000 50  0001 C CNN
F 3 "" H 5300 1000 50  0000 C CNN
	1    5300 1000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR018
U 1 1 56CF135A
P 5300 700
F 0 "#PWR018" H 5300 450 50  0001 C CNN
F 1 "GND" H 5300 550 50  0000 C CNN
F 2 "" H 5300 700 50  0000 C CNN
F 3 "" H 5300 700 50  0000 C CNN
	1    5300 700 
	-1   0    0    1   
$EndComp
$Comp
L CONN_02X08 P7
U 1 1 56CF23CB
P 1950 3650
F 0 "P7" H 1950 4100 50  0000 C CNN
F 1 "CONN_02X08" V 1950 3650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x08" H 1950 2450 50  0001 C CNN
F 3 "" H 1950 2450 50  0000 C CNN
	1    1950 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 56CF2418
P 2350 4150
F 0 "#PWR019" H 2350 3900 50  0001 C CNN
F 1 "GND" H 2350 4000 50  0000 C CNN
F 2 "" H 2350 4150 50  0000 C CNN
F 3 "" H 2350 4150 50  0000 C CNN
	1    2350 4150
	1    0    0    -1  
$EndComp
Text Label 6050 2950 0    60   ~ 0
AD0
Text Label 6050 3050 0    60   ~ 0
AD1
Text Label 6050 3150 0    60   ~ 0
AD2
Text Label 6050 3250 0    60   ~ 0
AD3
Text Label 5950 4100 0    60   ~ 0
PWR_OFF
Text Label 5950 4200 0    60   ~ 0
GPIO0
Text Label 5950 4300 0    60   ~ 0
GPIO1
Text Label 5950 4400 0    60   ~ 0
GPIO2
Text Label 1500 3300 2    60   ~ 0
GPIO3
Text Label 1500 3400 2    60   ~ 0
GPIO2
Text Label 1500 3500 2    60   ~ 0
GPIO1
Text Label 1500 3600 2    60   ~ 0
GPIO0
Text Label 1500 3700 2    60   ~ 0
AD0
Text Label 1500 3800 2    60   ~ 0
AD1
Text Label 1500 3900 2    60   ~ 0
AD2
Text Label 1500 4000 2    60   ~ 0
AD3
NoConn ~ 6050 3900
NoConn ~ 10050 2300
NoConn ~ 10050 2600
NoConn ~ 10050 3550
Text Label 6350 2100 0    60   ~ 0
GPIO3
Text Label 7050 1650 0    60   ~ 0
RPPWR
$Comp
L CONN_02X02 P8
U 1 1 56D2BB2A
P 2900 3450
F 0 "P8" H 2900 3600 50  0000 C CNN
F 1 "CONN_02X02" H 2900 3300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02" H 2900 2250 50  0001 C CNN
F 3 "" H 2900 2250 50  0000 C CNN
	1    2900 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4400 3700 4400
Connection ~ 3700 4400
Wire Wire Line
	5850 2700 6300 2700
Wire Wire Line
	6300 2700 6300 2550
Wire Wire Line
	6300 2550 6650 2550
Wire Wire Line
	6300 2850 6650 2850
Wire Wire Line
	6300 2850 6300 2800
Wire Wire Line
	6300 2800 5850 2800
Wire Wire Line
	7600 3900 7600 4250
Wire Wire Line
	5850 3700 7800 3700
Wire Wire Line
	7800 3800 5850 3800
Wire Wire Line
	7800 3900 7600 3900
Wire Wire Line
	7900 2300 7900 2400
Wire Wire Line
	3000 2100 3950 2100
Connection ~ 3750 2100
Wire Wire Line
	5850 3550 6050 3550
Wire Wire Line
	6050 3550 6050 3600
Wire Wire Line
	6050 3600 7800 3600
Wire Wire Line
	7500 3500 7500 3600
Connection ~ 7500 3600
Wire Wire Line
	5850 4000 6400 4000
Wire Wire Line
	6400 4000 6400 4650
Wire Wire Line
	6400 4650 8200 4650
Wire Wire Line
	10450 2200 10050 2200
Wire Wire Line
	10450 2000 10450 2200
Wire Wire Line
	10050 2100 10450 2100
Connection ~ 10450 2100
Wire Wire Line
	10050 2700 10450 2700
Connection ~ 10450 2700
Wire Wire Line
	10450 3000 10050 3000
Connection ~ 10450 3000
Wire Wire Line
	9100 2500 9550 2500
Wire Wire Line
	9100 1450 9100 4000
Wire Wire Line
	8500 3300 9550 3300
Connection ~ 9100 3300
Wire Wire Line
	3000 2100 3000 2000
Wire Wire Line
	3950 4300 3700 4300
Wire Wire Line
	8500 2600 9550 2600
Wire Wire Line
	8800 2100 9550 2100
Wire Wire Line
	8800 1450 8800 2100
Wire Wire Line
	8500 2200 9550 2200
Wire Wire Line
	8900 2200 8900 1450
Wire Wire Line
	8500 2300 9550 2300
Wire Wire Line
	9000 2300 9000 1450
Wire Wire Line
	1900 1550 2300 1550
Wire Wire Line
	2300 1550 2300 1350
Wire Wire Line
	1900 1950 2300 1950
Wire Wire Line
	2300 1950 2300 2050
Wire Wire Line
	7900 1900 7900 1550
Wire Wire Line
	5850 2200 6500 2200
Wire Wire Line
	7500 3200 7500 3000
Wire Wire Line
	3700 4300 3700 4950
Wire Wire Line
	1900 1700 1650 1700
Wire Wire Line
	1650 1800 1900 1800
Wire Wire Line
	1900 1800 1900 2300
Connection ~ 1900 1950
Wire Wire Line
	6800 2200 7600 2200
Wire Wire Line
	7200 2300 7200 2200
Wire Wire Line
	7200 2600 7200 3000
Wire Wire Line
	9550 2700 7600 2700
Wire Wire Line
	7600 2700 7600 2200
Wire Wire Line
	1900 1400 1900 1700
Connection ~ 1900 1550
Wire Wire Line
	2100 1600 2100 1550
Connection ~ 2100 1550
Wire Wire Line
	2100 1900 2100 1950
Connection ~ 2100 1950
Wire Wire Line
	10050 2400 10700 2400
Wire Wire Line
	10700 2500 10050 2500
Connection ~ 9100 2500
Wire Wire Line
	10450 3500 10050 3500
Connection ~ 10450 3500
Wire Wire Line
	10450 3700 10050 3700
Wire Wire Line
	10450 2600 10450 3700
Connection ~ 10450 3650
Wire Wire Line
	9100 4000 9550 4000
Wire Wire Line
	8800 1600 9450 1600
Wire Wire Line
	9250 1600 9250 1750
Connection ~ 8800 1600
Wire Wire Line
	9450 1600 9450 1750
Connection ~ 9250 1600
Wire Wire Line
	9250 2050 9250 2200
Connection ~ 9250 2200
Wire Wire Line
	9450 2050 9450 2300
Connection ~ 9450 2300
Connection ~ 7200 2200
Connection ~ 9000 2300
Connection ~ 8900 2200
Wire Wire Line
	5850 3350 6400 3350
Wire Wire Line
	5850 3450 6400 3450
Connection ~ 7200 2850
Wire Wire Line
	6950 2850 7200 2850
Connection ~ 6500 2850
Connection ~ 6500 2550
Wire Wire Line
	6950 2550 7050 2550
Wire Wire Line
	7050 2550 7050 2850
Connection ~ 7050 2850
Wire Wire Line
	6350 1200 6600 1200
Wire Wire Line
	6600 1200 6600 1900
Wire Wire Line
	6600 1900 6200 1900
Wire Wire Line
	6200 1900 6200 2400
Wire Wire Line
	6200 2400 5850 2400
Wire Wire Line
	5850 2600 6100 2600
Wire Wire Line
	6100 2600 6100 1700
Wire Wire Line
	6100 1700 5700 1700
Wire Wire Line
	5700 1700 5700 1200
Wire Wire Line
	5700 1200 5850 1200
Wire Wire Line
	5750 1650 6050 1650
Wire Wire Line
	6050 1650 6050 2500
Wire Wire Line
	6050 2500 5850 2500
Wire Wire Line
	6350 1100 6450 1100
Wire Wire Line
	6450 1100 6450 1000
Wire Wire Line
	6350 1300 6450 1300
Wire Wire Line
	6450 1300 6450 1450
Wire Wire Line
	5750 1650 5750 1100
Wire Wire Line
	5750 1100 5850 1100
Wire Wire Line
	4750 1300 5850 1300
Wire Wire Line
	7100 3600 7100 3300
Connection ~ 7100 3600
Wire Wire Line
	7600 2100 7500 2100
Wire Wire Line
	7200 5550 7600 5550
Wire Wire Line
	7600 5750 7200 5750
Wire Wire Line
	7600 5950 7350 5950
Wire Wire Line
	7350 5950 7350 6050
Wire Wire Line
	7600 5350 7350 5350
Wire Wire Line
	7350 5350 7350 5250
Wire Wire Line
	8500 5550 8850 5550
Wire Wire Line
	8500 5350 8850 5350
Wire Wire Line
	8500 5950 8900 5950
Wire Wire Line
	8900 5950 8900 6000
Wire Wire Line
	8500 5750 8850 5750
Wire Wire Line
	5850 2300 6300 2300
Wire Wire Line
	6300 2300 6300 2400
Wire Wire Line
	6300 2400 6500 2400
Wire Wire Line
	3950 2400 3750 2400
Wire Wire Line
	3750 2400 3750 2100
Wire Wire Line
	3950 2700 3750 2700
Wire Wire Line
	3750 2700 3750 2750
Wire Wire Line
	7200 5550 7200 5500
Wire Wire Line
	7200 5500 6850 5500
Wire Wire Line
	6850 5800 7200 5800
Wire Wire Line
	7200 5800 7200 5750
Connection ~ 5300 1300
Wire Wire Line
	2200 3300 2350 3300
Wire Wire Line
	2350 3300 2350 4150
Wire Wire Line
	2200 3400 2350 3400
Connection ~ 2350 3400
Wire Wire Line
	2200 3500 2350 3500
Connection ~ 2350 3500
Wire Wire Line
	2200 3600 2350 3600
Connection ~ 2350 3600
Wire Wire Line
	2200 3700 2350 3700
Connection ~ 2350 3700
Wire Wire Line
	2200 3800 2350 3800
Connection ~ 2350 3800
Wire Wire Line
	2200 3900 2350 3900
Connection ~ 2350 3900
Wire Wire Line
	2200 4000 2350 4000
Connection ~ 2350 4000
Wire Wire Line
	5850 2950 6050 2950
Wire Wire Line
	5850 3050 6050 3050
Wire Wire Line
	5850 3150 6050 3150
Wire Wire Line
	5850 3250 6050 3250
Wire Wire Line
	5850 4100 5950 4100
Wire Wire Line
	5850 4200 5950 4200
Wire Wire Line
	5850 4300 5950 4300
Wire Wire Line
	5850 4400 5950 4400
Wire Wire Line
	1700 3300 1500 3300
Wire Wire Line
	1700 3400 1500 3400
Wire Wire Line
	1700 3500 1500 3500
Wire Wire Line
	1700 3600 1500 3600
Wire Wire Line
	1700 3700 1500 3700
Wire Wire Line
	1700 3800 1500 3800
Wire Wire Line
	1700 3900 1500 3900
Wire Wire Line
	1700 4000 1500 4000
Wire Wire Line
	5850 3900 6050 3900
Wire Wire Line
	10700 2600 10450 2600
Wire Wire Line
	10450 3650 10450 4550
Wire Wire Line
	5850 2100 6350 2100
Wire Wire Line
	7200 2100 7050 2100
Wire Wire Line
	7050 2100 7050 1650
$Comp
L GND #PWR020
U 1 1 56D2BC5E
P 3200 3750
F 0 "#PWR020" H 3200 3500 50  0001 C CNN
F 1 "GND" H 3200 3600 50  0000 C CNN
F 2 "" H 3200 3750 50  0000 C CNN
F 3 "" H 3200 3750 50  0000 C CNN
	1    3200 3750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR021
U 1 1 56D2BCA2
P 2600 3150
F 0 "#PWR021" H 2600 3000 50  0001 C CNN
F 1 "+5V" H 2600 3290 50  0000 C CNN
F 2 "" H 2600 3150 50  0000 C CNN
F 3 "" H 2600 3150 50  0000 C CNN
	1    2600 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3400 3200 3400
Wire Wire Line
	3200 3400 3200 3750
Wire Wire Line
	3150 3500 3200 3500
Connection ~ 3200 3500
Wire Wire Line
	2650 3400 2600 3400
Wire Wire Line
	2600 3150 2600 3500
Wire Wire Line
	2600 3500 2650 3500
Connection ~ 2600 3400
$EndSCHEMATC
