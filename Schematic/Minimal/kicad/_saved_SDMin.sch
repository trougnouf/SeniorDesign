EESchema Schematic File Version 2
LIBS:SDMin-rescue
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
LIBS:stm32
LIBS:generic
LIBS:SDMin-cache
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
L STM32L476R U?
U 1 1 56B8B935
P 4900 3850
F 0 "U?" H 4950 3800 60  0000 C CNN
F 1 "STM32L476R" H 4900 3900 60  0000 C CNN
F 2 "" H 4600 3850 60  0000 C CNN
F 3 "" H 4600 3850 60  0000 C CNN
	1    4900 3850
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 56B8E511
P 4350 1900
F 0 "C?" H 4360 1970 50  0000 L CNN
F 1 ".1uF" H 4360 1820 50  0000 L CNN
F 2 "" H 4350 1900 50  0000 C CNN
F 3 "" H 4350 1900 50  0000 C CNN
	1    4350 1900
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 56B8E649
P 4650 1900
F 0 "C?" H 4660 1970 50  0000 L CNN
F 1 ".1uF" H 4660 1820 50  0000 L CNN
F 2 "" H 4650 1900 50  0000 C CNN
F 3 "" H 4650 1900 50  0000 C CNN
	1    4650 1900
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 56B8E66D
P 4950 1900
F 0 "C?" H 4960 1970 50  0000 L CNN
F 1 ".1uF" H 4960 1820 50  0000 L CNN
F 2 "" H 4950 1900 50  0000 C CNN
F 3 "" H 4950 1900 50  0000 C CNN
	1    4950 1900
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 56B8E695
P 5250 1900
F 0 "C?" H 5260 1970 50  0000 L CNN
F 1 ".1uF" H 5260 1820 50  0000 L CNN
F 2 "" H 5250 1900 50  0000 C CNN
F 3 "" H 5250 1900 50  0000 C CNN
	1    5250 1900
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 56B8E6C1
P 5550 1900
F 0 "C?" H 5560 1970 50  0000 L CNN
F 1 ".1uF" H 5560 1820 50  0000 L CNN
F 2 "" H 5550 1900 50  0000 C CNN
F 3 "" H 5550 1900 50  0000 C CNN
	1    5550 1900
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 56B8E759
P 4750 1550
F 0 "#PWR?" H 4750 1400 50  0001 C CNN
F 1 "+3V3" H 4750 1690 50  0000 C CNN
F 2 "" H 4750 1550 50  0000 C CNN
F 3 "" H 4750 1550 50  0000 C CNN
	1    4750 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B8E779
P 4550 1550
F 0 "#PWR?" H 4550 1300 50  0001 C CNN
F 1 "GND" H 4550 1400 50  0000 C CNN
F 2 "" H 4550 1550 50  0000 C CNN
F 3 "" H 4550 1550 50  0000 C CNN
	1    4550 1550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4550 1900 4550 1550
Wire Wire Line
	4750 1900 4750 1550
Wire Wire Line
	4850 1900 4850 1700
Wire Wire Line
	4250 1700 5450 1700
Connection ~ 4550 1700
Wire Wire Line
	5150 1700 5150 1900
Connection ~ 4850 1700
Wire Wire Line
	5450 1700 5450 1900
Connection ~ 5150 1700
Wire Wire Line
	4250 1900 4250 1700
Wire Wire Line
	4150 1900 4150 1600
Wire Wire Line
	4150 1600 5650 1600
Connection ~ 4750 1600
Wire Wire Line
	4450 1900 4450 1600
Connection ~ 4450 1600
Wire Wire Line
	5050 1600 5050 1900
Wire Wire Line
	5350 1600 5350 1900
Connection ~ 5050 1600
Wire Wire Line
	5650 1600 5650 1900
Connection ~ 5350 1600
$Comp
L Lightware_SF10/A U?
U 1 1 56B8F17D
P 2450 3650
F 0 "U?" H 2650 3250 60  0000 C CNN
F 1 "Lightware_SF10/A" H 2500 4050 60  0000 C CNN
F 2 "" H 2450 3650 60  0000 C CNN
F 3 "" H 2450 3650 60  0000 C CNN
	1    2450 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 3650 2750 3650
Wire Wire Line
	3900 3750 2750 3750
$EndSCHEMATC
