EESchema Schematic File Version 4
EELAYER 30 0
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
L Memory_EEPROM:24LC1025 U?
U 1 1 5EB9A07C
P 1650 1300
F 0 "U?" H 1750 1550 50  0000 C CNN
F 1 "24LC1025" H 1400 1050 50  0000 C CNN
F 2 "" H 1650 1300 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21941B.pdf" H 1650 1300 50  0001 C CNN
	1    1650 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EB9BE5C
P 2400 1200
F 0 "R?" V 2607 1200 50  0000 C CNN
F 1 "330R" V 2516 1200 50  0000 C CNN
F 2 "" V 2330 1200 50  0001 C CNN
F 3 "~" H 2400 1200 50  0001 C CNN
	1    2400 1200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5EB9C71C
P 2400 1300
F 0 "R?" V 2515 1300 50  0000 C CNN
F 1 "330R" V 2606 1300 50  0000 C CNN
F 2 "" V 2330 1300 50  0001 C CNN
F 3 "~" H 2400 1300 50  0001 C CNN
	1    2400 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 1600 1650 1750
$Comp
L power:GND #PWR?
U 1 1 5EB9D239
P 750 1800
F 0 "#PWR?" H 750 1550 50  0001 C CNN
F 1 "GND" H 755 1627 50  0000 C CNN
F 2 "" H 750 1800 50  0001 C CNN
F 3 "" H 750 1800 50  0001 C CNN
	1    750  1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1400 2050 1750
Wire Wire Line
	2050 1750 1650 1750
Connection ~ 1650 1750
Wire Wire Line
	2050 1200 2250 1200
Wire Wire Line
	2050 1300 2250 1300
Wire Wire Line
	2550 1200 2650 1200
$Comp
L power:VCC #PWR?
U 1 1 5EB9E9D8
P 1650 950
F 0 "#PWR?" H 1650 800 50  0001 C CNN
F 1 "VCC" H 1667 1123 50  0000 C CNN
F 2 "" H 1650 950 50  0001 C CNN
F 3 "" H 1650 950 50  0001 C CNN
	1    1650 950 
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5EBA9178
P 1000 950
F 0 "#PWR?" H 1000 800 50  0001 C CNN
F 1 "VCC" H 1017 1123 50  0000 C CNN
F 2 "" H 1000 950 50  0001 C CNN
F 3 "" H 1000 950 50  0001 C CNN
	1    1000 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1750 1650 1750
Wire Wire Line
	2550 1300 2650 1300
Text GLabel 2650 1200 2    50   BiDi ~ 0
SDA
Text GLabel 2650 1300 2    50   Input ~ 0
SCL
Text Notes 600  650  0    50   ~ 0
Mémoire n°1
Wire Wire Line
	1650 1000 1650 950 
Wire Wire Line
	1000 950  1000 1200
Wire Wire Line
	750  1800 750  1750
Connection ~ 750  1750
Wire Wire Line
	750  1750 750  1400
Wire Notes Line
	600  700  2900 700 
Wire Notes Line
	2900 700  2900 2050
Wire Notes Line
	2900 2050 600  2050
Wire Notes Line
	600  2050 600  700 
$Comp
L Memory_EEPROM:24LC1025 U?
U 1 1 5EBC2B3A
P 1650 3100
F 0 "U?" H 1750 3350 50  0000 C CNN
F 1 "24LC1025" H 1400 2850 50  0000 C CNN
F 2 "" H 1650 3100 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21941B.pdf" H 1650 3100 50  0001 C CNN
	1    1650 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC2B40
P 2400 3000
F 0 "R?" V 2607 3000 50  0000 C CNN
F 1 "330R" V 2516 3000 50  0000 C CNN
F 2 "" V 2330 3000 50  0001 C CNN
F 3 "~" H 2400 3000 50  0001 C CNN
	1    2400 3000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC2B46
P 2400 3100
F 0 "R?" V 2515 3100 50  0000 C CNN
F 1 "330R" V 2606 3100 50  0000 C CNN
F 2 "" V 2330 3100 50  0001 C CNN
F 3 "~" H 2400 3100 50  0001 C CNN
	1    2400 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 3400 1650 3550
$Comp
L power:GND #PWR?
U 1 1 5EBC2B4D
P 750 3600
F 0 "#PWR?" H 750 3350 50  0001 C CNN
F 1 "GND" H 755 3427 50  0000 C CNN
F 2 "" H 750 3600 50  0001 C CNN
F 3 "" H 750 3600 50  0001 C CNN
	1    750  3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3200 2050 3550
Wire Wire Line
	2050 3550 1650 3550
Connection ~ 1650 3550
Wire Wire Line
	2050 3000 2250 3000
Wire Wire Line
	2050 3100 2250 3100
Wire Wire Line
	2550 3000 2650 3000
$Comp
L power:VCC #PWR?
U 1 1 5EBC2B59
P 1650 2750
F 0 "#PWR?" H 1650 2600 50  0001 C CNN
F 1 "VCC" H 1667 2923 50  0000 C CNN
F 2 "" H 1650 2750 50  0001 C CNN
F 3 "" H 1650 2750 50  0001 C CNN
	1    1650 2750
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5EBC2B5F
P 1000 2750
F 0 "#PWR?" H 1000 2600 50  0001 C CNN
F 1 "VCC" H 1017 2923 50  0000 C CNN
F 2 "" H 1000 2750 50  0001 C CNN
F 3 "" H 1000 2750 50  0001 C CNN
	1    1000 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  3550 1650 3550
Wire Wire Line
	2550 3100 2650 3100
Text GLabel 2650 3000 2    50   BiDi ~ 0
SDA
Text GLabel 2650 3100 2    50   Input ~ 0
SCL
Text Notes 600  2450 0    50   ~ 0
Mémoire n°2
Wire Wire Line
	1650 2800 1650 2750
Wire Wire Line
	1000 2750 1000 3000
Wire Wire Line
	750  3600 750  3550
Connection ~ 750  3550
Wire Wire Line
	750  3550 750  3100
Wire Notes Line
	600  2500 2900 2500
Wire Notes Line
	2900 2500 2900 3850
Wire Notes Line
	2900 3850 600  3850
Wire Notes Line
	600  3850 600  2500
$Comp
L Memory_EEPROM:24LC1025 U?
U 1 1 5EBC4AD7
P 1650 4850
F 0 "U?" H 1750 5100 50  0000 C CNN
F 1 "24LC1025" H 1400 4600 50  0000 C CNN
F 2 "" H 1650 4850 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21941B.pdf" H 1650 4850 50  0001 C CNN
	1    1650 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC4ADD
P 2400 4750
F 0 "R?" V 2607 4750 50  0000 C CNN
F 1 "330R" V 2516 4750 50  0000 C CNN
F 2 "" V 2330 4750 50  0001 C CNN
F 3 "~" H 2400 4750 50  0001 C CNN
	1    2400 4750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC4AE3
P 2400 4850
F 0 "R?" V 2515 4850 50  0000 C CNN
F 1 "330R" V 2606 4850 50  0000 C CNN
F 2 "" V 2330 4850 50  0001 C CNN
F 3 "~" H 2400 4850 50  0001 C CNN
	1    2400 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 5150 1650 5300
$Comp
L power:GND #PWR?
U 1 1 5EBC4AEA
P 750 5350
F 0 "#PWR?" H 750 5100 50  0001 C CNN
F 1 "GND" H 755 5177 50  0000 C CNN
F 2 "" H 750 5350 50  0001 C CNN
F 3 "" H 750 5350 50  0001 C CNN
	1    750  5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4950 2050 5300
Wire Wire Line
	2050 5300 1650 5300
Connection ~ 1650 5300
Wire Wire Line
	2050 4750 2250 4750
Wire Wire Line
	2050 4850 2250 4850
Wire Wire Line
	2550 4750 2650 4750
$Comp
L power:VCC #PWR?
U 1 1 5EBC4AF6
P 1650 4500
F 0 "#PWR?" H 1650 4350 50  0001 C CNN
F 1 "VCC" H 1667 4673 50  0000 C CNN
F 2 "" H 1650 4500 50  0001 C CNN
F 3 "" H 1650 4500 50  0001 C CNN
	1    1650 4500
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5EBC4AFC
P 1000 4500
F 0 "#PWR?" H 1000 4350 50  0001 C CNN
F 1 "VCC" H 1017 4673 50  0000 C CNN
F 2 "" H 1000 4500 50  0001 C CNN
F 3 "" H 1000 4500 50  0001 C CNN
	1    1000 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  5300 1650 5300
Wire Wire Line
	2550 4850 2650 4850
Text GLabel 2650 4750 2    50   BiDi ~ 0
SDA
Text GLabel 2650 4850 2    50   Input ~ 0
SCL
Text Notes 600  4200 0    50   ~ 0
Mémoire n°3
Wire Wire Line
	1650 4550 1650 4500
Wire Wire Line
	1000 4500 1000 4750
Wire Wire Line
	750  5350 750  5300
Connection ~ 750  5300
Wire Notes Line
	600  4250 2900 4250
Wire Notes Line
	2900 4250 2900 5600
Wire Notes Line
	2900 5600 600  5600
Wire Notes Line
	600  5600 600  4250
$Comp
L Memory_EEPROM:24LC1025 U?
U 1 1 5EBC9DFB
P 1650 6700
F 0 "U?" H 1750 6950 50  0000 C CNN
F 1 "24LC1025" H 1400 6450 50  0000 C CNN
F 2 "" H 1650 6700 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21941B.pdf" H 1650 6700 50  0001 C CNN
	1    1650 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC9E01
P 2400 6600
F 0 "R?" V 2607 6600 50  0000 C CNN
F 1 "330R" V 2516 6600 50  0000 C CNN
F 2 "" V 2330 6600 50  0001 C CNN
F 3 "~" H 2400 6600 50  0001 C CNN
	1    2400 6600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC9E07
P 2400 6700
F 0 "R?" V 2515 6700 50  0000 C CNN
F 1 "330R" V 2606 6700 50  0000 C CNN
F 2 "" V 2330 6700 50  0001 C CNN
F 3 "~" H 2400 6700 50  0001 C CNN
	1    2400 6700
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 7000 1650 7150
$Comp
L power:GND #PWR?
U 1 1 5EBC9E0E
P 750 7200
F 0 "#PWR?" H 750 6950 50  0001 C CNN
F 1 "GND" H 755 7027 50  0000 C CNN
F 2 "" H 750 7200 50  0001 C CNN
F 3 "" H 750 7200 50  0001 C CNN
	1    750  7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 6800 2050 7150
Wire Wire Line
	2050 7150 1650 7150
Connection ~ 1650 7150
Wire Wire Line
	2050 6600 2250 6600
Wire Wire Line
	2050 6700 2250 6700
Wire Wire Line
	2550 6600 2650 6600
$Comp
L power:VCC #PWR?
U 1 1 5EBC9E1A
P 1650 6350
F 0 "#PWR?" H 1650 6200 50  0001 C CNN
F 1 "VCC" H 1667 6523 50  0000 C CNN
F 2 "" H 1650 6350 50  0001 C CNN
F 3 "" H 1650 6350 50  0001 C CNN
	1    1650 6350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5EBC9E20
P 1000 6350
F 0 "#PWR?" H 1000 6200 50  0001 C CNN
F 1 "VCC" H 1017 6523 50  0000 C CNN
F 2 "" H 1000 6350 50  0001 C CNN
F 3 "" H 1000 6350 50  0001 C CNN
	1    1000 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  7150 1650 7150
Wire Wire Line
	2550 6700 2650 6700
Text GLabel 2650 6600 2    50   BiDi ~ 0
SDA
Text GLabel 2650 6700 2    50   Input ~ 0
SCL
Text Notes 600  6050 0    50   ~ 0
Mémoire n°4
Wire Wire Line
	1650 6400 1650 6350
Wire Wire Line
	1000 6350 1000 6600
Wire Wire Line
	750  7200 750  7150
Wire Notes Line
	600  6100 2900 6100
Wire Notes Line
	2900 6100 2900 7450
Wire Notes Line
	2900 7450 600  7450
Wire Notes Line
	600  7450 600  6100
Wire Wire Line
	1250 6800 1000 6800
Connection ~ 1000 6800
Wire Wire Line
	1000 6800 1000 7000
Wire Wire Line
	1250 6700 1000 6700
Connection ~ 1000 6700
Wire Wire Line
	1000 6700 1000 6800
Wire Wire Line
	1250 6600 1000 6600
Connection ~ 1000 6600
Wire Wire Line
	1000 6600 1000 6700
Wire Wire Line
	750  5300 750  4950
Wire Wire Line
	1250 4850 1000 4850
Wire Wire Line
	1250 4750 1000 4750
Connection ~ 1000 4750
Wire Wire Line
	1000 4750 1000 4850
Wire Wire Line
	1250 4950 750  4950
Wire Wire Line
	1250 3000 1000 3000
Connection ~ 1000 3000
Wire Wire Line
	1000 3000 1000 3200
Wire Wire Line
	1250 3200 1000 3200
Wire Wire Line
	1250 3100 750  3100
Wire Wire Line
	1250 1400 750  1400
Connection ~ 750  1400
Wire Wire Line
	750  1400 750  1300
Wire Wire Line
	1250 1300 750  1300
Wire Wire Line
	1250 1200 1000 1200
$EndSCHEMATC
