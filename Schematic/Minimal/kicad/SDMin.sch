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
LIBS:diodesinc
LIBS:components
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
$Comp
L Battery BT?
U 1 1 56B971E8
P 2500 850
F 0 "BT?" H 2600 900 50  0000 L CNN
F 1 "Battery" H 2600 800 50  0000 L CNN
F 2 "" V 2500 890 50  0000 C CNN
F 3 "" V 2500 890 50  0000 C CNN
	1    2500 850 
	1    0    0    -1  
$EndComp
$Comp
L MCP73811/2 U?
U 1 1 56B97525
P 1650 800
F 0 "U?" H 1500 600 60  0000 C CNN
F 1 "MCP73811/2" H 1700 1000 60  0000 C CNN
F 2 "" H 1700 650 60  0000 C CNN
F 3 "" H 1700 650 60  0000 C CNN
	1    1650 800 
	1    0    0    -1  
$EndComp
$Comp
L USB_B P?
U 1 1 56B97931
P 700 800
F 0 "P?" H 900 600 50  0000 C CNN
F 1 "USB_B" H 650 1000 50  0000 C CNN
F 2 "" V 650 700 50  0000 C CNN
F 3 "" V 650 700 50  0000 C CNN
	1    700  800 
	0    -1   -1   0   
$EndComp
$Comp
L PAM2301 U?
U 1 1 56B97A68
P 1250 1500
F 0 "U?" H 1600 1750 60  0000 C CNN
F 1 "PAM2301" H 1050 1750 60  0000 C CNN
F 2 "" H 1250 1500 60  0000 C CNN
F 3 "" H 1250 1500 60  0000 C CNN
	1    1250 1500
	1    0    0    -1  
$EndComp
$Comp
L AAT1217 U?
U 1 1 56B98419
P 1150 2350
F 0 "U?" H 950 2200 60  0000 C CNN
F 1 "AAT1217" H 1100 2600 60  0000 C CNN
F 2 "" H 1150 2350 60  0000 C CNN
F 3 "" H 1150 2350 60  0000 C CNN
	1    1150 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B99846
P 1000 700
F 0 "#PWR?" H 1000 450 50  0001 C CNN
F 1 "GND" H 1000 550 50  0000 C CNN
F 2 "" H 1000 700 50  0000 C CNN
F 3 "" H 1000 700 50  0000 C CNN
	1    1000 700 
	-1   0    0    1   
$EndComp
$Comp
L C_Small C?
U 1 1 56B99A60
P 1100 900
F 0 "C?" H 1110 970 50  0000 L CNN
F 1 "1uF" H 1110 820 50  0000 L CNN
F 2 "" H 1100 900 50  0000 C CNN
F 3 "" H 1100 900 50  0000 C CNN
	1    1100 900 
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 56B9A3C3
P 2250 800
F 0 "C?" H 2260 870 50  0000 L CNN
F 1 "1uF" H 2260 720 50  0000 L CNN
F 2 "" H 2250 800 50  0000 C CNN
F 3 "" H 2250 800 50  0000 C CNN
	1    2250 800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9A75A
P 2050 900
F 0 "#PWR?" H 2050 650 50  0001 C CNN
F 1 "GND" H 2050 750 50  0000 C CNN
F 2 "" H 2050 900 50  0000 C CNN
F 3 "" H 2050 900 50  0000 C CNN
	1    2050 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9A7B4
P 2250 900
F 0 "#PWR?" H 2250 650 50  0001 C CNN
F 1 "GND" H 2250 750 50  0000 C CNN
F 2 "" H 2250 900 50  0000 C CNN
F 3 "" H 2250 900 50  0000 C CNN
	1    2250 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9A80E
P 2500 1000
F 0 "#PWR?" H 2500 750 50  0001 C CNN
F 1 "GND" H 2500 850 50  0000 C CNN
F 2 "" H 2500 1000 50  0000 C CNN
F 3 "" H 2500 1000 50  0000 C CNN
	1    2500 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9B33C
P 550 1400
F 0 "#PWR?" H 550 1150 50  0001 C CNN
F 1 "GND" H 550 1250 50  0000 C CNN
F 2 "" H 550 1400 50  0000 C CNN
F 3 "" H 550 1400 50  0000 C CNN
	1    550  1400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 56B9B3BD
P 550 1300
F 0 "C?" H 560 1370 50  0000 L CNN
F 1 "10uF" H 560 1220 50  0000 L CNN
F 2 "" H 550 1300 50  0000 C CNN
F 3 "" H 550 1300 50  0000 C CNN
	1    550  1300
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L?
U 1 1 56B9BC26
P 2100 1350
F 0 "L?" H 2100 1450 50  0000 C CNN
F 1 "4.7uH" H 2100 1300 50  0000 C CNN
F 2 "" H 2100 1350 50  0000 C CNN
F 3 "" H 2100 1350 50  0000 C CNN
	1    2100 1350
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 56B9BD07
P 2400 1500
F 0 "C?" H 2410 1570 50  0000 L CNN
F 1 "10uF" H 2410 1420 50  0000 L CNN
F 2 "" H 2400 1500 50  0000 C CNN
F 3 "" H 2400 1500 50  0000 C CNN
	1    2400 1500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 56B9BD61
P 2450 1350
F 0 "#PWR?" H 2450 1200 50  0001 C CNN
F 1 "+3V3" H 2450 1490 50  0000 C CNN
F 2 "" H 2450 1350 50  0000 C CNN
F 3 "" H 2450 1350 50  0000 C CNN
	1    2450 1350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9BDEF
P 2400 1600
F 0 "#PWR?" H 2400 1350 50  0001 C CNN
F 1 "GND" H 2400 1450 50  0000 C CNN
F 2 "" H 2400 1600 50  0000 C CNN
F 3 "" H 2400 1600 50  0000 C CNN
	1    2400 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9BF4B
P 1250 1900
F 0 "#PWR?" H 1250 1650 50  0001 C CNN
F 1 "GND" H 1250 1750 50  0000 C CNN
F 2 "" H 1250 1900 50  0000 C CNN
F 3 "" H 1250 1900 50  0000 C CNN
	1    1250 1900
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R?
U 1 1 56B9D677
P 1700 2200
F 0 "R?" H 1730 2220 50  0000 L CNN
F 1 "1M" H 1730 2160 50  0000 L CNN
F 2 "" H 1700 2200 50  0000 C CNN
F 3 "" H 1700 2200 50  0000 C CNN
	1    1700 2200
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R?
U 1 1 56B9D795
P 650 2400
F 0 "R?" H 680 2420 50  0000 L CNN
F 1 "332k" H 680 2360 50  0000 L CNN
F 2 "" H 650 2400 50  0000 C CNN
F 3 "" H 650 2400 50  0000 C CNN
	1    650  2400
	-1   0    0    1   
$EndComp
$Comp
L C_Small C?
U 1 1 56B9D80B
P 1750 2600
F 0 "C?" H 1760 2670 50  0000 L CNN
F 1 "4.7uF" H 1760 2520 50  0000 L CNN
F 2 "" H 1750 2600 50  0000 C CNN
F 3 "" H 1750 2600 50  0000 C CNN
	1    1750 2600
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 56B9D872
P 1900 2150
F 0 "C?" H 1910 2220 50  0000 L CNN
F 1 "4.7uF" H 1910 2070 50  0000 L CNN
F 2 "" H 1900 2150 50  0000 C CNN
F 3 "" H 1900 2150 50  0000 C CNN
	1    1900 2150
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L?
U 1 1 56B9D90C
P 850 1950
F 0 "L?" H 850 2050 50  0000 C CNN
F 1 "4.7uF" H 850 1900 50  0000 C CNN
F 2 "" H 850 1950 50  0000 C CNN
F 3 "" H 850 1950 50  0000 C CNN
	1    850  1950
	1    0    0    -1  
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
Wire Wire Line
	3900 3650 2750 3650
Wire Wire Line
	3900 3750 2750 3750
Wire Wire Line
	1000 1000 1200 1000
Wire Wire Line
	1100 800  1100 700 
Wire Wire Line
	1100 700  1000 700 
Wire Wire Line
	1200 1000 1200 700 
Wire Wire Line
	1200 700  1250 700 
Connection ~ 1100 1000
Wire Wire Line
	1200 800  1250 800 
Connection ~ 1200 800 
Wire Wire Line
	1200 900  1250 900 
Connection ~ 1200 900 
Connection ~ 2150 700 
Wire Wire Line
	2900 2000 2900 700 
Wire Wire Line
	2900 1200 550  1200
Wire Wire Line
	650  1200 650  1450
Connection ~ 2500 700 
Connection ~ 650  1350
Wire Wire Line
	2050 900  2050 900 
Connection ~ 2250 700 
Connection ~ 650  1200
Wire Wire Line
	1850 1450 2350 1450
Wire Wire Line
	2350 1450 2350 1350
Wire Wire Line
	2350 1350 2450 1350
Wire Wire Line
	2400 1400 2400 1350
Connection ~ 2400 1350
Wire Wire Line
	2900 700  2050 700 
Wire Wire Line
	1200 2000 2900 2000
Wire Wire Line
	1600 2000 1600 2200
Connection ~ 2900 1200
Wire Wire Line
	1800 2200 1800 2400
Wire Wire Line
	1800 2400 1600 2400
Wire Wire Line
	1200 2000 1200 1950
Wire Wire Line
	1200 1950 1100 1950
Connection ~ 1600 2000
Wire Wire Line
	600  1950 500  1950
Wire Wire Line
	500  1950 500  2200
Wire Wire Line
	500  2200 700  2200
Wire Wire Line
	1900 2000 1900 2050
Connection ~ 1900 2000
$Comp
L GND #PWR?
U 1 1 56B9DE65
P 1900 2250
F 0 "#PWR?" H 1900 2000 50  0001 C CNN
F 1 "GND" H 1900 2100 50  0000 C CNN
F 2 "" H 1900 2250 50  0000 C CNN
F 3 "" H 1900 2250 50  0000 C CNN
	1    1900 2250
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 56B9DED1
P 1000 2600
F 0 "R?" H 1030 2620 50  0000 L CNN
F 1 "1.02M" H 1030 2560 50  0000 L CNN
F 2 "" H 1000 2600 50  0000 C CNN
F 3 "" H 1000 2600 50  0000 C CNN
	1    1000 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 2300 1600 2600
Wire Wire Line
	1100 2600 1650 2600
Wire Wire Line
	700  2400 700  2600
Wire Wire Line
	650  2600 900  2600
Wire Wire Line
	550  2300 700  2300
Wire Wire Line
	650  2500 650  2600
Connection ~ 700  2600
$Comp
L GND #PWR?
U 1 1 56B9E33E
P 550 2550
F 0 "#PWR?" H 550 2300 50  0001 C CNN
F 1 "GND" H 550 2400 50  0000 C CNN
F 2 "" H 550 2550 50  0000 C CNN
F 3 "" H 550 2550 50  0000 C CNN
	1    550  2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	550  2300 550  2550
Connection ~ 650  2300
Connection ~ 1600 2600
$Comp
L GND #PWR?
U 1 1 56B9E54E
P 1850 2600
F 0 "#PWR?" H 1850 2350 50  0001 C CNN
F 1 "GND" H 1850 2450 50  0000 C CNN
F 2 "" H 1850 2600 50  0000 C CNN
F 3 "" H 1850 2600 50  0000 C CNN
	1    1850 2600
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 56B9E5BF
P 2050 2500
F 0 "#PWR?" H 2050 2350 50  0001 C CNN
F 1 "+5V" H 2050 2640 50  0000 C CNN
F 2 "" H 2050 2500 50  0000 C CNN
F 3 "" H 2050 2500 50  0000 C CNN
	1    2050 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 2500 1600 2500
Connection ~ 1600 2500
$Comp
L +5V #PWR?
U 1 1 56B9E8E5
P 2750 3350
F 0 "#PWR?" H 2750 3200 50  0001 C CNN
F 1 "+5V" H 2750 3490 50  0000 C CNN
F 2 "" H 2750 3350 50  0000 C CNN
F 3 "" H 2750 3350 50  0000 C CNN
	1    2750 3350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 56B9E93E
P 2750 3450
F 0 "#PWR?" H 2750 3200 50  0001 C CNN
F 1 "GND" H 2750 3300 50  0000 C CNN
F 2 "" H 2750 3450 50  0000 C CNN
F 3 "" H 2750 3450 50  0000 C CNN
	1    2750 3450
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
