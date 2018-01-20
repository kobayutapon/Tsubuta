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
Sheet 2 4
Title "Tsubuta Add-on Ver.2"
Date "2017-12-20"
Rev ""
Comp "GDG Shinshu"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RPi_GPIO J2
U 1 1 5516AE26
P 4600 2350
AR Path="/5516AE26" Ref="J2"  Part="1" 
AR Path="/5515D395/5516AE26" Ref="J2"  Part="1" 
F 0 "J2" H 5350 2600 60  0000 C CNN
F 1 "RPi_GPIO" H 5350 2500 60  0000 C CNN
F 2 "RPi_Hat:Pin_Header_Straight_2x20" H 4600 2350 60  0001 C CNN
F 3 "" H 4600 2350 60  0000 C CNN
	1    4600 2350
	1    0    0    -1  
$EndComp
Text Notes 4900 4650 0    60   Italic 0
Thru-Hole Connector
Wire Wire Line
	6300 2450 6500 2450
Wire Wire Line
	6500 2450 6500 1900
Wire Wire Line
	6300 2350 6500 2350
Connection ~ 6500 2350
Wire Wire Line
	6300 2550 6500 2550
Wire Wire Line
	6500 2550 6500 4600
$Comp
L GND #PWR2
U 1 1 5A3A0269
P 6500 4600
F 0 "#PWR2" H 6500 4350 50  0001 C CNN
F 1 "GND" H 6500 4450 50  0000 C CNN
F 2 "" H 6500 4600 50  0001 C CNN
F 3 "" H 6500 4600 50  0001 C CNN
	1    6500 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 2950 6500 2950
Connection ~ 6500 2950
Wire Wire Line
	6300 3250 6500 3250
Connection ~ 6500 3250
Wire Wire Line
	6300 3750 6500 3750
Connection ~ 6500 3750
Wire Wire Line
	6300 3950 6500 3950
Connection ~ 6500 3950
Text GLabel 6500 1900 1    60   Input ~ 0
RPI_5V
Wire Wire Line
	6300 2650 6700 2650
Wire Wire Line
	6300 2750 6700 2750
Text GLabel 6700 2650 2    60   Output ~ 0
RPI_TXD
Text GLabel 6700 2750 2    60   Input ~ 0
RPI_RXD
NoConn ~ 6300 2850
NoConn ~ 6300 3050
NoConn ~ 6300 3150
NoConn ~ 6300 3350
NoConn ~ 6300 3450
NoConn ~ 6300 3550
NoConn ~ 6300 3650
NoConn ~ 6300 3850
NoConn ~ 6300 4050
NoConn ~ 6300 4150
NoConn ~ 6300 4250
Wire Wire Line
	4400 2750 4050 2750
Wire Wire Line
	4050 2750 4050 4500
Wire Wire Line
	4400 4250 4050 4250
Connection ~ 4050 4250
Wire Wire Line
	4400 3550 4050 3550
Connection ~ 4050 3550
$Comp
L GND #PWR1
U 1 1 5A3A2240
P 4050 4500
F 0 "#PWR1" H 4050 4250 50  0001 C CNN
F 1 "GND" H 4050 4350 50  0000 C CNN
F 2 "" H 4050 4500 50  0001 C CNN
F 3 "" H 4050 4500 50  0001 C CNN
	1    4050 4500
	1    0    0    -1  
$EndComp
NoConn ~ 4400 4150
NoConn ~ 4400 4050
NoConn ~ 4400 3950
NoConn ~ 4400 3850
NoConn ~ 4400 3750
NoConn ~ 4400 3650
NoConn ~ 4400 3450
NoConn ~ 4400 3350
NoConn ~ 4400 3250
NoConn ~ 4400 3150
NoConn ~ 4400 3050
NoConn ~ 7600 1850
Wire Wire Line
	4400 2950 3850 2950
Wire Wire Line
	4400 2850 3850 2850
Wire Wire Line
	4400 2450 3850 2450
Wire Wire Line
	4400 2550 3850 2550
Wire Wire Line
	4400 2650 3850 2650
Text GLabel 3850 2450 0    60   BiDi ~ 0
RPI_SDA
Text GLabel 3850 2550 0    60   Output ~ 0
RPI_SCL
Text GLabel 3850 2650 0    60   Output ~ 0
RPI_INT
Text GLabel 3850 2850 0    60   Output ~ 0
PWR_OFF
Text GLabel 3850 2950 0    60   Input ~ 0
ARD_INT_3V3
NoConn ~ 4400 2350
NoConn ~ 4800 6800
$EndSCHEMATC
