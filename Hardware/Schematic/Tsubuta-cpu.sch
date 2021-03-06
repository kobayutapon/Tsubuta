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
LIBS:switches
LIBS:Tsubuta_Rpi-HAT-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
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
L ATMEGA328P-AU U1
U 1 1 5A3A25DE
P 5200 3500
F 0 "U1" H 4450 4750 50  0000 L BNN
F 1 "ATMEGA328P-AU" H 5600 2100 50  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 5200 3500 50  0001 C CIN
F 3 "" H 5200 3500 50  0001 C CNN
	1    5200 3500
	1    0    0    -1  
$EndComp
NoConn ~ 4300 3000
$Comp
L +5V #PWR3
U 1 1 5A3A261C
P 4050 2200
F 0 "#PWR3" H 4050 2050 50  0001 C CNN
F 1 "+5V" H 4050 2340 50  0000 C CNN
F 2 "" H 4050 2200 50  0001 C CNN
F 3 "" H 4050 2200 50  0001 C CNN
	1    4050 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2700 4050 2700
Wire Wire Line
	4050 2700 4050 2200
Wire Wire Line
	4300 2500 4050 2500
Connection ~ 4050 2500
Wire Wire Line
	4300 2400 4050 2400
Connection ~ 4050 2400
$Comp
L GND #PWR4
U 1 1 5A3A2643
P 4050 4850
F 0 "#PWR4" H 4050 4600 50  0001 C CNN
F 1 "GND" H 4050 4700 50  0000 C CNN
F 2 "" H 4050 4850 50  0001 C CNN
F 3 "" H 4050 4850 50  0001 C CNN
	1    4050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 4500 4050 4500
Wire Wire Line
	4050 4500 4050 4850
Wire Wire Line
	4300 4600 4050 4600
Connection ~ 4050 4600
Wire Wire Line
	4300 4700 4050 4700
Connection ~ 4050 4700
Wire Wire Line
	4300 3750 4100 3750
Wire Wire Line
	4300 3850 4100 3850
Text GLabel 4100 3750 0    60   Output ~ 0
3V3_ENB
Text GLabel 4100 3850 0    60   Input ~ 0
BOOTMODE
Wire Wire Line
	6200 4700 6450 4700
Wire Wire Line
	6200 4600 6450 4600
Wire Wire Line
	6200 4500 6450 4500
Wire Wire Line
	6200 4400 6450 4400
Wire Wire Line
	6200 4300 6450 4300
Wire Wire Line
	6200 4200 6450 4200
Wire Wire Line
	6200 4100 6450 4100
Wire Wire Line
	6200 4000 6450 4000
Wire Wire Line
	6200 3850 6450 3850
Wire Wire Line
	6200 3750 6450 3750
Wire Wire Line
	6200 3650 6450 3650
Wire Wire Line
	6200 3550 6450 3550
Wire Wire Line
	6200 3450 6450 3450
Wire Wire Line
	6200 3350 6450 3350
Wire Wire Line
	6200 3250 6450 3250
Wire Wire Line
	6200 3100 6450 3100
Wire Wire Line
	6200 3000 6450 3000
Wire Wire Line
	6200 2900 6450 2900
Wire Wire Line
	6200 2800 6450 2800
Wire Wire Line
	6200 2700 6450 2700
Wire Wire Line
	6200 2600 6450 2600
Wire Wire Line
	6200 2500 6450 2500
Wire Wire Line
	6200 2400 6450 2400
Text GLabel 6450 2400 2    60   BiDi ~ 0
GPIO3
Text GLabel 6450 2500 2    60   Output ~ 0
ARD_INT_5V
Text GLabel 6450 2600 2    60   Output ~ 0
#ARD_SS
Text GLabel 6450 2700 2    60   Output ~ 0
ARD_MOSI
Text GLabel 6450 2800 2    60   Input ~ 0
ARD_MISO
Text GLabel 6450 2900 2    60   Output ~ 0
ARD_SCK
Text GLabel 6450 3000 2    60   BiDi ~ 0
ARD_XTAL1
Text GLabel 6450 3100 2    60   BiDi ~ 0
ARD_XTAL2
Text GLabel 6450 3250 2    60   Input ~ 0
ADC0
Text GLabel 6450 3350 2    60   Input ~ 0
ADC1
Text GLabel 6450 3450 2    60   Input ~ 0
ADC2
Text GLabel 6450 3550 2    60   Input ~ 0
ADC3
Text GLabel 6450 3650 2    60   BiDi ~ 0
ARD_SDA
Text GLabel 6450 3750 2    60   BiDi ~ 0
ARD_SCL
Text GLabel 6450 3850 2    60   Input ~ 0
#RST
Text GLabel 6450 4000 2    60   Input ~ 0
ARD_RX
Text GLabel 6450 4100 2    60   Output ~ 0
ARD_TX
Text GLabel 6450 4200 2    60   Input ~ 0
RPI_INT
Text GLabel 6450 4300 2    60   Input ~ 0
RPPWR
Text GLabel 6450 4400 2    60   Output ~ 0
PWR_OFF
Text GLabel 6450 4500 2    60   BiDi ~ 0
GPIO0
Text GLabel 6450 4600 2    60   BiDi ~ 0
GPIO1
Text GLabel 6450 4700 2    60   BiDi ~ 0
GPIO2
$Comp
L Conn_02x03_Odd_Even J3
U 1 1 5A3A3D1C
P 9750 1750
F 0 "J3" H 9800 1950 50  0000 C CNN
F 1 "CN_ICSP" H 9800 1550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 9750 1750 50  0001 C CNN
F 3 "" H 9750 1750 50  0001 C CNN
	1    9750 1750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR9
U 1 1 5A3A3DEB
P 10300 1500
F 0 "#PWR9" H 10300 1350 50  0001 C CNN
F 1 "+5V" H 10300 1640 50  0000 C CNN
F 2 "" H 10300 1500 50  0001 C CNN
F 3 "" H 10300 1500 50  0001 C CNN
	1    10300 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 1650 10300 1650
Wire Wire Line
	10300 1650 10300 1500
$Comp
L GND #PWR10
U 1 1 5A3A3E2C
P 10300 2050
F 0 "#PWR10" H 10300 1800 50  0001 C CNN
F 1 "GND" H 10300 1900 50  0000 C CNN
F 2 "" H 10300 2050 50  0001 C CNN
F 3 "" H 10300 2050 50  0001 C CNN
	1    10300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 1850 10300 1850
Wire Wire Line
	10300 1850 10300 2050
Wire Wire Line
	10050 1750 10300 1750
Text GLabel 10300 1750 2    60   Input ~ 0
ARD_MOSI
Wire Wire Line
	9550 1650 9250 1650
Wire Wire Line
	9550 1750 9250 1750
Text GLabel 9250 1650 0    60   Output ~ 0
ARD_MISO
Text GLabel 9250 1750 0    60   Input ~ 0
ARD_SCK
Wire Wire Line
	9550 1850 8200 1850
$Comp
L R_Small R1
U 1 1 5A3A4206
P 8350 1700
F 0 "R1" H 8380 1720 50  0000 L CNN
F 1 "10K" H 8380 1660 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 8350 1700 50  0001 C CNN
F 3 "" H 8350 1700 50  0001 C CNN
	1    8350 1700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR5
U 1 1 5A3A4227
P 8350 1500
F 0 "#PWR5" H 8350 1350 50  0001 C CNN
F 1 "+5V" H 8350 1640 50  0000 C CNN
F 2 "" H 8350 1500 50  0001 C CNN
F 3 "" H 8350 1500 50  0001 C CNN
	1    8350 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 1500 8350 1600
Wire Wire Line
	8350 1800 8350 2000
Connection ~ 8350 1850
Text GLabel 8200 1850 0    60   Output ~ 0
#RST
$Comp
L C_Small C1
U 1 1 5A3A4339
P 8350 2100
F 0 "C1" H 8360 2170 50  0000 L CNN
F 1 "0.1uF" H 8360 2020 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8350 2100 50  0001 C CNN
F 3 "" H 8350 2100 50  0001 C CNN
	1    8350 2100
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J1
U 1 1 5A3A44A4
P 8950 2500
F 0 "J1" H 8950 2700 50  0000 C CNN
F 1 "Conn_01x04" H 8950 2200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 8950 2500 50  0001 C CNN
F 3 "" H 8950 2500 50  0001 C CNN
	1    8950 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 2400 8350 2200
Wire Wire Line
	8750 2500 8350 2500
Wire Wire Line
	8750 2600 8350 2600
$Comp
L GND #PWR6
U 1 1 5A3A464C
P 8350 2800
F 0 "#PWR6" H 8350 2550 50  0001 C CNN
F 1 "GND" H 8350 2650 50  0000 C CNN
F 2 "" H 8350 2800 50  0001 C CNN
F 3 "" H 8350 2800 50  0001 C CNN
	1    8350 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 2700 8350 2700
Wire Wire Line
	8350 2700 8350 2800
Text GLabel 8350 2500 0    60   Output ~ 0
ARD_RX
Text GLabel 8350 2600 0    60   Input ~ 0
ARD_TX
$Comp
L SW_Push SW1
U 1 1 5A3A47C4
P 9400 2150
F 0 "SW1" H 9450 2250 50  0000 L CNN
F 1 "SW_RST" H 9400 2090 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 9400 2350 50  0001 C CNN
F 3 "" H 9400 2350 50  0001 C CNN
	1    9400 2150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 1950 9400 1850
Connection ~ 9400 1850
$Comp
L GND #PWR7
U 1 1 5A3A4881
P 9400 2450
F 0 "#PWR7" H 9400 2200 50  0001 C CNN
F 1 "GND" H 9400 2300 50  0000 C CNN
F 2 "" H 9400 2450 50  0001 C CNN
F 3 "" H 9400 2450 50  0001 C CNN
	1    9400 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2350 9400 2450
$Comp
L Crystal Y1
U 1 1 5A3A4A49
P 8900 3650
F 0 "Y1" H 8900 3800 50  0000 C CNN
F 1 "16MHz" H 8900 3500 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 8900 3650 50  0001 C CNN
F 3 "" H 8900 3650 50  0001 C CNN
	1    8900 3650
	0    1    1    0   
$EndComp
$Comp
L C_Small C2
U 1 1 5A3A4BFD
P 9400 3450
F 0 "C2" H 9410 3520 50  0000 L CNN
F 1 "22pF" H 9410 3370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9400 3450 50  0001 C CNN
F 3 "" H 9400 3450 50  0001 C CNN
	1    9400 3450
	0    1    1    0   
$EndComp
$Comp
L C_Small C3
U 1 1 5A3A4C2C
P 9400 3850
F 0 "C3" H 9410 3920 50  0000 L CNN
F 1 "22pF" H 9410 3770 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9400 3850 50  0001 C CNN
F 3 "" H 9400 3850 50  0001 C CNN
	1    9400 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	8400 3850 9300 3850
Wire Wire Line
	9300 3450 8400 3450
Wire Wire Line
	8900 3500 8900 3450
Connection ~ 8900 3450
Wire Wire Line
	8900 3800 8900 3850
Connection ~ 8900 3850
Wire Wire Line
	9500 3450 9750 3450
Wire Wire Line
	9750 3450 9750 3850
Wire Wire Line
	9750 3850 9500 3850
$Comp
L GND #PWR8
U 1 1 5A3A4E0F
P 10100 4000
F 0 "#PWR8" H 10100 3750 50  0001 C CNN
F 1 "GND" H 10100 3850 50  0000 C CNN
F 2 "" H 10100 4000 50  0001 C CNN
F 3 "" H 10100 4000 50  0001 C CNN
	1    10100 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 4000 10100 3650
Wire Wire Line
	10100 3650 9750 3650
Connection ~ 9750 3650
Text GLabel 8400 3450 0    60   BiDi ~ 0
ARD_XTAL1
Text GLabel 8400 3850 0    60   BiDi ~ 0
ARD_XTAL2
Wire Wire Line
	8750 2400 8350 2400
$EndSCHEMATC
