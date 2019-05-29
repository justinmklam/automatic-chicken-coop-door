EESchema Schematic File Version 4
LIBS:auto-chicken-coop-door-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Automatic Chicken Coop Door"
Date "2019-04-27"
Rev "A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Chicken~Coop~Door:OLED_Display_128x68 U4
U 1 1 5CC7AC9B
P 5050 6100
F 0 "U4" H 4950 6650 50  0000 C CNN
F 1 "OLED_Display_128x68" H 5050 6100 50  0000 C CNN
F 2 "" H 5050 6100 50  0001 C CNN
F 3 "" H 5050 6100 50  0001 C CNN
	1    5050 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4750 4100 4450
Wire Wire Line
	4100 4450 4900 4450
Wire Wire Line
	3800 5050 3800 4150
Wire Wire Line
	3800 4150 4900 4150
Wire Wire Line
	3900 4250 4900 4250
Wire Wire Line
	3900 4950 3900 4250
Wire Wire Line
	4900 4350 4000 4350
Wire Wire Line
	4000 4350 4000 4850
Wire Wire Line
	3150 5050 3800 5050
Wire Wire Line
	3150 4950 3900 4950
Wire Wire Line
	4000 4850 3150 4850
Wire Wire Line
	3150 4750 4100 4750
$Comp
L Chicken~Coop~Door:Sparkfun_Dual_Motor_Driver_TB6612FNG U3
U 1 1 5CC93726
P 2750 5700
F 0 "U3" H 2750 5750 50  0000 C CNN
F 1 "Sparkfun_Dual_Motor_Driver_TB6612FNG" H 2750 5650 50  0000 C CNN
F 2 "" H 2750 5700 50  0001 C CNN
F 3 "" H 2750 5700 50  0001 C CNN
	1    2750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 5050 2350 5050
Wire Wire Line
	1850 5150 2350 5150
$Comp
L power:GND #PWR0101
U 1 1 5CD026BA
P 2350 5450
F 0 "#PWR0101" H 2350 5200 50  0001 C CNN
F 1 "GND" H 2355 5277 50  0000 C CNN
F 2 "" H 2350 5450 50  0001 C CNN
F 3 "" H 2350 5450 50  0001 C CNN
	1    2350 5450
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0102
U 1 1 5CD03543
P 2100 4750
F 0 "#PWR0102" H 2100 4600 50  0001 C CNN
F 1 "VCC" H 2117 4923 50  0000 C CNN
F 2 "" H 2100 4750 50  0001 C CNN
F 3 "" H 2100 4750 50  0001 C CNN
	1    2100 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 5350 1850 5150
Wire Wire Line
	1300 5350 1850 5350
Wire Wire Line
	1850 4850 1850 5050
Wire Wire Line
	1300 4850 1850 4850
$Comp
L Motor:Motor_DC M1
U 1 1 5CC7CC5F
P 1300 5050
F 0 "M1" H 1458 5046 50  0000 L CNN
F 1 "Motor_DC" H 1458 4955 50  0000 L CNN
F 2 "" H 1300 4960 50  0001 C CNN
F 3 "~" H 1300 4960 50  0001 C CNN
	1    1300 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 5650 5300 5650
Wire Wire Line
	5500 4750 5500 5750
Wire Wire Line
	5500 5750 5300 5750
Text GLabel 5400 5150 0    50   Input ~ 0
SDA
Text GLabel 5500 5300 2    50   Input ~ 0
SCL
Text GLabel 3300 3300 0    50   Input ~ 0
SDA
Text GLabel 3300 3200 0    50   Input ~ 0
SCL
$Comp
L power:VCC #PWR0103
U 1 1 5CD79418
P 3700 3000
F 0 "#PWR0103" H 3700 2850 50  0001 C CNN
F 1 "VCC" H 3717 3173 50  0000 C CNN
F 2 "" H 3700 3000 50  0001 C CNN
F 3 "" H 3700 3000 50  0001 C CNN
	1    3700 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5CD7EE85
P 3800 3800
F 0 "#PWR0104" H 3800 3550 50  0001 C CNN
F 1 "GND" H 3805 3627 50  0000 C CNN
F 2 "" H 3800 3800 50  0001 C CNN
F 3 "" H 3800 3800 50  0001 C CNN
	1    3800 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4750 5400 5650
$Comp
L power:VCC #PWR0105
U 1 1 5CDDF751
P 5700 5850
F 0 "#PWR0105" H 5700 5700 50  0001 C CNN
F 1 "VCC" H 5717 6023 50  0000 C CNN
F 2 "" H 5700 5850 50  0001 C CNN
F 3 "" H 5700 5850 50  0001 C CNN
	1    5700 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5CDE402A
P 5700 5950
F 0 "#PWR0106" H 5700 5700 50  0001 C CNN
F 1 "GND" H 5705 5777 50  0000 C CNN
F 2 "" H 5700 5950 50  0001 C CNN
F 3 "" H 5700 5950 50  0001 C CNN
	1    5700 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 5950 5300 5950
Wire Wire Line
	5300 5850 5700 5850
$Comp
L Chicken~Coop~Door:Adafruit_Solar_Charger_390 U5
U 1 1 5CDFCF9D
P 7050 1600
F 0 "U5" H 7133 2065 50  0000 C CNN
F 1 "Adafruit_Solar_Charger_390" H 7133 1974 50  0000 C CNN
F 2 "" H 7000 1000 50  0001 C CNN
F 3 "" H 7000 1000 50  0001 C CNN
	1    7050 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:Solar_Cells SC1
U 1 1 5CDFE7D9
P 8200 1450
F 0 "SC1" H 8308 1496 50  0000 L CNN
F 1 "Solar_Cells" H 8308 1405 50  0000 L CNN
F 2 "" V 8200 1510 50  0001 C CNN
F 3 "~" V 8200 1510 50  0001 C CNN
	1    8200 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 1400 7900 1400
Wire Wire Line
	7900 1400 7900 1250
Wire Wire Line
	7900 1250 8200 1250
Wire Wire Line
	7500 1500 7900 1500
Wire Wire Line
	7900 1500 7900 1650
Wire Wire Line
	7900 1650 8200 1650
$Comp
L power:GND #PWR0107
U 1 1 5CD7E121
P 7300 3400
F 0 "#PWR0107" H 7300 3150 50  0001 C CNN
F 1 "GND" H 7305 3227 50  0000 C CNN
F 2 "" H 7300 3400 50  0001 C CNN
F 3 "" H 7300 3400 50  0001 C CNN
	1    7300 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell BT1
U 1 1 5CE10F7E
P 6550 2200
F 0 "BT1" H 6300 2350 50  0000 L CNN
F 1 "Battery_Cell" H 6050 2150 50  0000 L CNN
F 2 "" V 6550 2260 50  0001 C CNN
F 3 "~" V 6550 2260 50  0001 C CNN
	1    6550 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2000 6850 2000
Wire Wire Line
	6950 2000 6950 2300
Wire Wire Line
	6950 2300 6550 2300
Wire Wire Line
	6300 3400 7300 3400
$Comp
L Chicken~Coop~Door:Arduino_Pro_Mini U1
U 1 1 5CC9C424
P 5600 3950
F 0 "U1" H 6100 3150 50  0000 C CNN
F 1 "Arduino_Pro_Mini" H 6100 3050 50  0000 C CNN
F 2 "" H 6350 3050 50  0001 C CNN
F 3 "" H 6350 3050 50  0001 C CNN
	1    5600 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3300 7200 3300
Wire Wire Line
	7200 2000 7200 2550
Wire Wire Line
	7300 2000 7300 2450
Connection ~ 7300 3400
$Comp
L power:VCC #PWR0108
U 1 1 5CDEBA25
P 6500 3600
F 0 "#PWR0108" H 6500 3450 50  0001 C CNN
F 1 "VCC" H 6650 3700 50  0000 C CNN
F 2 "" H 6500 3600 50  0001 C CNN
F 3 "" H 6500 3600 50  0001 C CNN
	1    6500 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3600 6500 3600
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5CE4B214
P 5650 2600
F 0 "J1" V 5712 2844 50  0000 L CNN
F 1 "Programming_Headers" V 5803 2844 50  0000 L CNN
F 2 "" H 5650 2600 50  0001 C CNN
F 3 "~" H 5650 2600 50  0001 C CNN
	1    5650 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 2800 5350 3050
Wire Wire Line
	5450 2800 5450 3050
Wire Wire Line
	5550 3050 5550 2800
Wire Wire Line
	5650 2800 5650 3050
Wire Wire Line
	5750 3050 5750 2800
Wire Wire Line
	5850 2800 5850 3050
$Comp
L power:VCC #PWR0109
U 1 1 5CCADED1
P 8250 3100
F 0 "#PWR0109" H 8250 2950 50  0001 C CNN
F 1 "VCC" H 8267 3273 50  0000 C CNN
F 2 "" H 8250 3100 50  0001 C CNN
F 3 "" H 8250 3100 50  0001 C CNN
	1    8250 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5CC76C6D
P 8250 3250
F 0 "R1" H 8320 3296 50  0000 L CNN
F 1 "10k" H 8320 3205 50  0000 L CNN
F 2 "" V 8180 3250 50  0001 C CNN
F 3 "~" H 8250 3250 50  0001 C CNN
	1    8250 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3400 8500 3400
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5CE9FFC0
P 8700 3400
F 0 "J2" H 8780 3392 50  0000 L CNN
F 1 "Conn_Left" H 8780 3301 50  0000 L CNN
F 2 "" H 8700 3400 50  0001 C CNN
F 3 "~" H 8700 3400 50  0001 C CNN
	1    8700 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5CEC3DBE
P 8500 3500
F 0 "#PWR0110" H 8500 3250 50  0001 C CNN
F 1 "GND" H 8505 3327 50  0000 C CNN
F 2 "" H 8500 3500 50  0001 C CNN
F 3 "" H 8500 3500 50  0001 C CNN
	1    8500 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4350 7600 4350
Wire Wire Line
	7600 4350 7600 3400
Connection ~ 8250 3400
$Comp
L power:VCC #PWR0111
U 1 1 5CED7845
P 8250 4150
F 0 "#PWR0111" H 8250 4000 50  0001 C CNN
F 1 "VCC" H 8267 4323 50  0000 C CNN
F 2 "" H 8250 4150 50  0001 C CNN
F 3 "" H 8250 4150 50  0001 C CNN
	1    8250 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5CED784B
P 8250 4300
F 0 "R2" H 8320 4346 50  0000 L CNN
F 1 "10k" H 8320 4255 50  0000 L CNN
F 2 "" V 8180 4300 50  0001 C CNN
F 3 "~" H 8250 4300 50  0001 C CNN
	1    8250 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4450 8500 4450
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5CED7852
P 8700 4450
F 0 "J3" H 8780 4442 50  0000 L CNN
F 1 "Conn_Middle" H 8780 4351 50  0000 L CNN
F 2 "" H 8700 4450 50  0001 C CNN
F 3 "~" H 8700 4450 50  0001 C CNN
	1    8700 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5CED7858
P 8500 4550
F 0 "#PWR0112" H 8500 4300 50  0001 C CNN
F 1 "GND" H 8505 4377 50  0000 C CNN
F 2 "" H 8500 4550 50  0001 C CNN
F 3 "" H 8500 4550 50  0001 C CNN
	1    8500 4550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0113
U 1 1 5CED8F7B
P 8250 5050
F 0 "#PWR0113" H 8250 4900 50  0001 C CNN
F 1 "VCC" H 8267 5223 50  0000 C CNN
F 2 "" H 8250 5050 50  0001 C CNN
F 3 "" H 8250 5050 50  0001 C CNN
	1    8250 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5CED8F81
P 8250 5200
F 0 "R3" H 8320 5246 50  0000 L CNN
F 1 "10k" H 8320 5155 50  0000 L CNN
F 2 "" V 8180 5200 50  0001 C CNN
F 3 "~" H 8250 5200 50  0001 C CNN
	1    8250 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 5350 8500 5350
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5CED8F88
P 8700 5350
F 0 "J4" H 8780 5342 50  0000 L CNN
F 1 "Conn_Right" H 8780 5251 50  0000 L CNN
F 2 "" H 8700 5350 50  0001 C CNN
F 3 "~" H 8700 5350 50  0001 C CNN
	1    8700 5350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5CED8F8E
P 8500 5450
F 0 "#PWR0114" H 8500 5200 50  0001 C CNN
F 1 "GND" H 8505 5277 50  0000 C CNN
F 2 "" H 8500 5450 50  0001 C CNN
F 3 "" H 8500 5450 50  0001 C CNN
	1    8500 5450
	1    0    0    -1  
$EndComp
Connection ~ 8250 4450
Wire Wire Line
	6300 4550 7600 4550
Wire Wire Line
	7600 4550 7600 5350
Connection ~ 8250 5350
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 5CEF524D
P 9700 3400
F 0 "J5" H 9618 3617 50  0000 C CNN
F 1 "Conn_Left" H 9618 3526 50  0000 C CNN
F 2 "" H 9700 3400 50  0001 C CNN
F 3 "~" H 9700 3400 50  0001 C CNN
	1    9700 3400
	-1   0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5CEF807B
P 10200 3450
F 0 "SW1" V 10154 3598 50  0000 L CNN
F 1 "SW_Left" V 10245 3598 50  0000 L CNN
F 2 "" H 10200 3650 50  0001 C CNN
F 3 "~" H 10200 3650 50  0001 C CNN
	1    10200 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 3400 10050 3400
Wire Wire Line
	10050 3400 10050 3250
Wire Wire Line
	10050 3250 10200 3250
Wire Wire Line
	10200 3650 10050 3650
Wire Wire Line
	10050 3650 10050 3500
Wire Wire Line
	10050 3500 9900 3500
Wire Wire Line
	6300 4450 8250 4450
Wire Wire Line
	7600 3400 8250 3400
Wire Wire Line
	7600 5350 8250 5350
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 5CF1A170
P 9700 4450
F 0 "J6" H 9618 4667 50  0000 C CNN
F 1 "Conn_Middle" H 9618 4576 50  0000 C CNN
F 2 "" H 9700 4450 50  0001 C CNN
F 3 "~" H 9700 4450 50  0001 C CNN
	1    9700 4450
	-1   0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5CF1A176
P 10200 4500
F 0 "SW2" V 10154 4648 50  0000 L CNN
F 1 "SW_Middle" V 10245 4648 50  0000 L CNN
F 2 "" H 10200 4700 50  0001 C CNN
F 3 "~" H 10200 4700 50  0001 C CNN
	1    10200 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 4450 10050 4450
Wire Wire Line
	10050 4450 10050 4300
Wire Wire Line
	10050 4300 10200 4300
Wire Wire Line
	10200 4700 10050 4700
Wire Wire Line
	10050 4700 10050 4550
Wire Wire Line
	10050 4550 9900 4550
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 5CF1B8E0
P 9700 5350
F 0 "J7" H 9618 5567 50  0000 C CNN
F 1 "Conn_Right" H 9618 5476 50  0000 C CNN
F 2 "" H 9700 5350 50  0001 C CNN
F 3 "~" H 9700 5350 50  0001 C CNN
	1    9700 5350
	-1   0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 5CF1B8E6
P 10200 5400
F 0 "SW3" V 10154 5548 50  0000 L CNN
F 1 "SW_Right" V 10245 5548 50  0000 L CNN
F 2 "" H 10200 5600 50  0001 C CNN
F 3 "~" H 10200 5600 50  0001 C CNN
	1    10200 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 5350 10050 5350
Wire Wire Line
	10050 5350 10050 5200
Wire Wire Line
	10050 5200 10200 5200
Wire Wire Line
	10200 5600 10050 5600
Wire Wire Line
	10050 5600 10050 5450
Wire Wire Line
	10050 5450 9900 5450
Wire Wire Line
	2100 4750 2100 4850
$Comp
L Timer_RTC:DS3231M U2
U 1 1 5CC70780
P 3800 3400
F 0 "U2" H 4300 3100 50  0000 C CNN
F 1 "DS3231M" H 4150 3000 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm" H 3800 2800 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS3231.pdf" H 4070 3450 50  0001 C CNN
	1    3800 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3500 4300 3500
$Comp
L Chicken~Coop~Door:Buck_Boost_module U6
U 1 1 5CEF5879
P 3750 2150
F 0 "U6" H 3750 1885 50  0000 C CNN
F 1 "Buck_Boost_XL6009E1" H 3750 1976 50  0000 C CNN
F 2 "" H 3900 2100 50  0001 C CNN
F 3 "" H 3900 2100 50  0001 C CNN
	1    3750 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 2250 5550 2550
Wire Wire Line
	5550 2550 7200 2550
Connection ~ 7200 2550
Wire Wire Line
	7200 2550 7200 3300
Wire Wire Line
	4100 2100 5650 2100
Wire Wire Line
	5650 2100 5650 2450
Wire Wire Line
	5650 2450 7300 2450
Connection ~ 7300 2450
Wire Wire Line
	7300 2450 7300 3400
Wire Wire Line
	3400 2250 2250 2250
Wire Wire Line
	2250 2250 2250 4750
Wire Wire Line
	2250 4750 2350 4750
Wire Wire Line
	1950 4950 2350 4950
Wire Wire Line
	3400 2100 1950 2100
Wire Wire Line
	1950 2100 1950 4950
Wire Wire Line
	2100 4850 2350 4850
$Comp
L Transistor_BJT:TIP122 Q1
U 1 1 5CF1402B
P 4750 2450
F 0 "Q1" H 5050 2450 50  0000 C CNN
F 1 "TIP122" H 5050 2350 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4950 2375 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/TI/TIP120.pdf" H 4750 2450 50  0001 L CNN
	1    4750 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2250 4150 2250
Wire Wire Line
	4550 2450 4550 3250
Wire Wire Line
	4550 3250 4750 3250
Wire Wire Line
	4150 2650 4850 2650
Wire Wire Line
	4150 2250 4150 2650
Wire Wire Line
	4850 2250 5550 2250
Wire Wire Line
	4650 3500 4650 3750
Wire Wire Line
	4650 3750 4900 3750
Wire Wire Line
	4750 3250 4750 3850
Wire Wire Line
	4750 3850 4900 3850
$EndSCHEMATC
