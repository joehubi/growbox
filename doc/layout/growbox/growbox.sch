EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "growbox"
Date "28.06.2021"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Module:Arduino_UNO_R3 A?
U 1 1 60DA1571
P 3700 3750
F 0 "A?" H 3350 4700 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 4450 4600 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 3700 3750 50  0001 C CIN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 3700 3750 50  0001 C CNN
	1    3700 3750
	1    0    0    -1  
$EndComp
Text GLabel 2100 2400 0    50   Input ~ 0
D5
Wire Wire Line
	3200 3750 2950 3750
Text GLabel 2100 2600 0    50   Input ~ 0
D6
Wire Wire Line
	2800 3850 3200 3850
Text GLabel 2150 2800 0    50   Input ~ 0
GND
Wire Wire Line
	2500 2800 2500 4950
Wire Wire Line
	3600 4950 3600 4850
Text GLabel 9000 2950 2    50   Input ~ 0
L1_230V
Text Notes 8900 2850 0    50   ~ 0
Ruck Rohrventilator\nEM125L-EC-02\n46W\n0.4 A_max
Text GLabel 9000 3100 2    50   Input ~ 0
N_230V
Text GLabel 9000 3250 2    50   Input ~ 0
PWM_in
Text GLabel 9000 3400 2    50   Input ~ 0
PWM_gnd
Text Notes 2550 2100 2    50   ~ 0
NodeMCU ESP8266\nJoy-IT\n
Wire Wire Line
	2100 2400 2950 2400
Wire Wire Line
	2100 2600 2800 2600
Wire Wire Line
	2500 4950 3600 4950
Wire Wire Line
	2150 2800 2500 2800
Text GLabel 2150 2200 0    50   Input ~ 0
3.3V
Text Label 2200 2400 0    50   ~ 0
serial_rx
Text Label 2200 2600 0    50   ~ 0
serial_tx
Wire Wire Line
	3600 4950 4750 4950
Wire Wire Line
	8800 5000 8800 3400
Wire Wire Line
	8800 3400 9000 3400
Connection ~ 3600 4950
Wire Wire Line
	9000 3250 8650 3250
Wire Wire Line
	8650 3250 8650 5100
Wire Wire Line
	2950 5100 2950 4050
Wire Wire Line
	2950 4050 3200 4050
Text Notes 8300 1350 0    50   ~ 0
Heizung\n(interne Regelung)\n700 W
Text GLabel 8350 1450 2    50   Input ~ 0
L1_230V
Text GLabel 8350 1600 2    50   Input ~ 0
N_230V
Text GLabel 4900 1500 2    50   Input ~ 0
L1_230V
Text GLabel 4900 1650 2    50   Input ~ 0
N_230V
Text Notes 4900 1400 0    50   ~ 0
Netzspannung 230V
Wire Wire Line
	2950 5100 8650 5100
Wire Wire Line
	8800 5000 6100 5000
Wire Wire Line
	4750 5000 4750 4950
$Comp
L Analog_Switch:DG308AxJ Schalter
U 1 1 60DC265B
P 7150 2250
F 0 "Schalter" H 7150 2517 50  0000 C CNN
F 1 "Ch1" H 7150 2426 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 7150 2150 50  0001 C CNN
F 3 "http://pdf.datasheetcatalog.com/datasheets/70/494502_DS.pdf" H 7150 2250 50  0001 C CNN
	1    7150 2250
	1    0    0    -1  
$EndComp
Text GLabel 6200 1900 2    50   Input ~ 0
5V
Text GLabel 6200 2050 2    50   Input ~ 0
GND
Text Notes 6200 1800 0    50   ~ 0
Relais-Modul\nHLRELM-4
$Comp
L Analog_Switch:DG308AxJ Schalter?
U 1 1 60DC9B1C
P 7150 2850
F 0 "Schalter?" H 7150 3117 50  0000 C CNN
F 1 "Ch2" H 7150 3026 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 7150 2750 50  0001 C CNN
F 3 "http://pdf.datasheetcatalog.com/datasheets/70/494502_DS.pdf" H 7150 2850 50  0001 C CNN
	1    7150 2850
	1    0    0    -1  
$EndComp
$Comp
L Analog_Switch:DG308AxJ Schalter?
U 1 1 60DD957E
P 7150 3400
F 0 "Schalter?" H 7150 3667 50  0000 C CNN
F 1 "Ch3" H 7150 3576 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 7150 3300 50  0001 C CNN
F 3 "http://pdf.datasheetcatalog.com/datasheets/70/494502_DS.pdf" H 7150 3400 50  0001 C CNN
	1    7150 3400
	1    0    0    -1  
$EndComp
$Comp
L Analog_Switch:DG308AxJ Schalter?
U 1 1 60DDFA74
P 7150 4000
F 0 "Schalter?" H 7150 4267 50  0000 C CNN
F 1 "Ch4" H 7150 4176 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 7150 3900 50  0001 C CNN
F 3 "http://pdf.datasheetcatalog.com/datasheets/70/494502_DS.pdf" H 7150 4000 50  0001 C CNN
	1    7150 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2950 7450 2950
Wire Wire Line
	7450 2950 7450 2850
Wire Wire Line
	6850 2250 6550 2250
Wire Wire Line
	4700 2250 4700 1500
Wire Wire Line
	4700 1500 4900 1500
Wire Wire Line
	6850 2850 6550 2850
Wire Wire Line
	6550 2850 6550 2250
Connection ~ 6550 2250
Wire Wire Line
	6550 2250 4700 2250
Wire Wire Line
	6850 3400 6550 3400
Wire Wire Line
	6550 3400 6550 2850
Connection ~ 6550 2850
Wire Wire Line
	6850 4000 6550 4000
Wire Wire Line
	6550 4000 6550 3400
Connection ~ 6550 3400
Wire Wire Line
	3150 2450 7150 2450
Wire Wire Line
	3100 2500 6400 2500
Wire Wire Line
	6400 2500 6400 3050
Wire Wire Line
	6400 3050 7150 3050
Wire Wire Line
	3200 3550 3050 3550
Wire Wire Line
	3200 3650 3000 3650
Wire Wire Line
	3000 2600 6250 2600
Wire Wire Line
	6250 2600 6250 4200
Wire Wire Line
	6250 4200 7150 4200
Text Notes 9550 4150 0    50   ~ 0
LED inkl. Trafo\n57W
Text GLabel 9550 4300 2    50   Input ~ 0
L1_230V
Text GLabel 9550 4500 2    50   Input ~ 0
N_230V
Wire Wire Line
	8400 3400 8400 4300
Wire Wire Line
	8400 4300 9550 4300
Wire Wire Line
	7450 3400 8400 3400
Text Notes 9550 4950 0    50   ~ 0
Ventilator\n5W
Text GLabel 9550 5050 2    50   Input ~ 0
L1_230V
Text GLabel 9550 5200 2    50   Input ~ 0
N_230V
Wire Wire Line
	9550 5050 8900 5050
Wire Wire Line
	8900 5050 8900 4750
Wire Wire Line
	8900 4750 7450 4750
Wire Wire Line
	7450 4750 7450 4000
Wire Wire Line
	4900 1650 4800 1650
Wire Wire Line
	4800 1650 4800 5550
Wire Wire Line
	9550 4500 9350 4500
Wire Wire Line
	9000 3100 8200 3100
Wire Wire Line
	8200 3100 8200 5550
Connection ~ 8200 5550
Wire Wire Line
	8200 5550 9350 5550
Wire Wire Line
	9450 5200 9450 5550
Wire Wire Line
	9450 5550 9350 5550
Wire Wire Line
	9450 5200 9550 5200
Connection ~ 9350 5550
Wire Wire Line
	9350 4500 9350 5550
$Comp
L Sensor_Temperature:DS18B20 U?
U 1 1 60DB0579
P 3500 5750
F 0 "U?" H 3270 5796 50  0000 R CNN
F 1 "DS18B20" H 3270 5705 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2500 5500 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 3350 6000 50  0001 C CNN
	1    3500 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3750 4550 3750
Wire Wire Line
	4550 3750 4550 5750
Wire Wire Line
	4550 5750 3800 5750
$Comp
L Sensor_Temperature:DS18B20 U?
U 1 1 60DBB026
P 2500 5750
F 0 "U?" H 2270 5796 50  0000 R CNN
F 1 "DS18B20" H 2270 5705 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 1500 5500 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 2350 6000 50  0001 C CNN
	1    2500 5750
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Temperature:DS18B20 U?
U 1 1 60DBD17C
P 1500 5750
F 0 "U?" H 1270 5796 50  0000 R CNN
F 1 "DS18B20" H 1270 5705 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 500 5500 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 1350 6000 50  0001 C CNN
	1    1500 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6050 3500 6200
Wire Wire Line
	3500 6200 2500 6200
Wire Wire Line
	700  6200 700  4950
Connection ~ 2500 4950
Wire Wire Line
	2500 6050 2500 6200
Connection ~ 2500 6200
Wire Wire Line
	2500 6200 1500 6200
Wire Wire Line
	1500 6050 1500 6200
Connection ~ 1500 6200
Wire Wire Line
	1500 6200 700  6200
Wire Wire Line
	2800 5750 2800 5400
Wire Wire Line
	2800 5400 4500 5400
Wire Wire Line
	4500 5400 4500 3850
Wire Wire Line
	4500 3850 4200 3850
Wire Wire Line
	4200 3950 4450 3950
Wire Wire Line
	4450 3950 4450 5350
Wire Wire Line
	4450 5350 1850 5350
Wire Wire Line
	1850 5350 1850 5750
Wire Wire Line
	1850 5750 1800 5750
Text GLabel 5500 4200 2    50   Input ~ 0
GND
Text GLabel 5500 3900 2    50   Input ~ 0
5V
Text GLabel 5500 4050 2    50   Input ~ 0
0...3V
Text Notes 5350 3800 0    50   ~ 0
DEBO CAP-SENS\nBodenfeuchte
Connection ~ 5300 5000
Wire Wire Line
	5300 5000 4750 5000
$Comp
L Sensor:DHT11 U?
U 1 1 60DDB8AE
P 10300 1850
F 0 "U?" H 10057 1896 50  0000 R CNN
F 1 "Luftfeuchte&Temp_DHT11" H 10150 2100 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 10300 1450 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 10450 2100 50  0001 C CNN
	1    10300 1850
	1    0    0    -1  
$EndComp
$Comp
L Sensor:DHT11 U?
U 1 1 60DB4B4F
P 10300 1050
F 0 "U?" H 10057 1096 50  0000 R CNN
F 1 "Luftfeuchte&Temp_DHT11" H 10150 1300 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 10300 650 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 10450 1300 50  0001 C CNN
	1    10300 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2350 4950 2350
Wire Wire Line
	4950 2350 4950 5950
Wire Wire Line
	3800 2350 3800 2750
Text GLabel 5950 6500 2    50   Input ~ 0
VCC3V
Text Notes 5850 6450 0    50   ~ 0
Real Time Clock\nZS-042\nI2C
Text GLabel 5950 6650 2    50   Input ~ 0
GND
Text GLabel 5950 6800 2    50   Input ~ 0
SDA
Text GLabel 5950 6950 2    50   Input ~ 0
SDL
Wire Wire Line
	5950 6650 4750 6650
Wire Wire Line
	4750 6650 4750 5000
Connection ~ 4750 5000
Wire Wire Line
	4200 4450 4300 4450
Wire Wire Line
	4300 4450 4300 6800
Wire Wire Line
	4300 6800 5050 6800
Wire Wire Line
	5950 6950 5400 6950
Wire Wire Line
	4250 6950 4250 4550
Wire Wire Line
	4250 4550 4200 4550
$Comp
L Device:R 10k
U 1 1 60FA353C
P 5050 6200
F 0 "10k" H 5100 6200 50  0000 L CNN
F 1 "R" H 5100 6150 50  0000 L CNN
F 2 "" V 4980 6200 50  0001 C CNN
F 3 "~" H 5050 6200 50  0001 C CNN
	1    5050 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 6500 5750 6500
Wire Wire Line
	5750 6500 5750 5950
Wire Wire Line
	4950 5950 5050 5950
Wire Wire Line
	5050 6800 5050 6350
Connection ~ 5050 6800
Wire Wire Line
	5050 6800 5950 6800
Wire Wire Line
	5050 6050 5050 5950
Connection ~ 5050 5950
Wire Wire Line
	5050 5950 5400 5950
$Comp
L Device:R 10k
U 1 1 60FE43A9
P 5400 6200
F 0 "10k" H 5450 6200 50  0000 L CNN
F 1 "R" H 5450 6150 50  0000 L CNN
F 2 "" V 5330 6200 50  0001 C CNN
F 3 "~" H 5400 6200 50  0001 C CNN
	1    5400 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6950 5400 6350
Connection ~ 5400 6950
Wire Wire Line
	5400 6950 4250 6950
Wire Wire Line
	5400 6050 5400 5950
Connection ~ 5400 5950
Wire Wire Line
	5400 5950 5750 5950
Wire Wire Line
	3800 2350 3800 2300
Connection ~ 3800 2350
Wire Wire Line
	2950 2400 2950 2300
Wire Wire Line
	2950 2300 3250 2300
Connection ~ 2950 2400
Wire Wire Line
	3550 2300 3800 2300
Connection ~ 3800 2300
Wire Wire Line
	3800 2300 3800 2100
$Comp
L Device:R 10k
U 1 1 60FF26ED
P 3400 2300
F 0 "10k" V 3450 2450 50  0000 L CNN
F 1 "R" H 3450 2250 50  0000 L CNN
F 2 "" V 3330 2300 50  0001 C CNN
F 3 "~" H 3400 2300 50  0001 C CNN
	1    3400 2300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 2100 3800 2100
$Comp
L Device:R 10k
U 1 1 6102CC5F
P 3400 2100
F 0 "10k" V 3450 2250 50  0000 L CNN
F 1 "R" H 3450 2050 50  0000 L CNN
F 2 "" V 3330 2100 50  0001 C CNN
F 3 "~" H 3400 2100 50  0001 C CNN
	1    3400 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2800 2100 2800 2600
Wire Wire Line
	2800 2100 3250 2100
Connection ~ 2800 2600
Wire Wire Line
	3800 2100 3800 1900
Wire Wire Line
	3800 1900 2700 1900
Wire Wire Line
	2700 1900 2700 2200
Wire Wire Line
	2700 2200 2150 2200
Connection ~ 3800 2100
Text Label 750  4950 0    50   ~ 0
GND
Text Label 7600 5000 0    50   ~ 0
GND
Text Label 4950 5400 0    50   ~ 0
3V
Wire Wire Line
	5300 4200 5500 4200
Wire Wire Line
	5300 4200 5300 5000
Wire Wire Line
	3900 2750 5300 2750
Wire Wire Line
	4200 4050 5500 4050
Wire Wire Line
	5500 3900 5300 3900
Wire Wire Line
	5300 3900 5300 2750
Wire Wire Line
	4800 5550 8200 5550
Wire Notes Line
	9500 4750 10050 4750
Wire Notes Line
	10050 5350 9500 5350
Wire Notes Line
	9500 4750 9500 5350
Wire Notes Line
	10050 4750 10050 5350
Wire Notes Line
	9500 3950 9500 4600
Wire Notes Line
	9500 4600 10150 4600
Wire Notes Line
	10150 4600 10150 3950
Wire Notes Line
	10150 3950 9500 3950
Wire Notes Line
	8850 3500 8850 2450
Wire Notes Line
	8850 2450 9700 2450
Wire Notes Line
	9700 2450 9700 3500
Wire Notes Line
	9700 3500 8850 3500
Wire Notes Line
	6150 1600 6150 4300
Wire Notes Line
	6150 4300 7600 4300
Wire Notes Line
	7600 4300 7600 1600
Wire Notes Line
	7600 1600 6150 1600
Wire Notes Line
	5350 3600 5350 4300
Wire Notes Line
	5350 4300 6050 4300
Wire Notes Line
	6050 4300 6050 3600
Wire Notes Line
	6050 3600 5350 3600
Connection ~ 6100 5000
Wire Wire Line
	5300 5000 6100 5000
Wire Wire Line
	5300 2750 5950 2750
Wire Wire Line
	5950 2750 5950 1900
Wire Wire Line
	5950 1900 6200 1900
Connection ~ 5300 2750
Wire Wire Line
	6100 2050 6100 5000
Wire Wire Line
	6100 2050 6200 2050
Wire Notes Line
	4850 1250 5700 1250
Wire Notes Line
	5700 1250 5700 1800
Wire Notes Line
	5700 1800 4850 1800
Wire Notes Line
	4850 1800 4850 1250
Wire Notes Line
	2600 2900 2600 1900
Wire Notes Line
	2600 1900 1800 1900
Wire Notes Line
	1800 1900 1800 2900
Wire Notes Line
	1800 2900 2600 2900
Wire Notes Line
	5800 6200 5800 7050
Wire Notes Line
	6550 7050 6550 6200
Wire Notes Line
	5800 6200 6550 6200
Wire Notes Line
	6550 7050 5800 7050
Wire Wire Line
	700  4950 2500 4950
Wire Notes Line
	8250 1100 8250 1700
Wire Notes Line
	9050 1700 9050 1100
Wire Notes Line
	8250 1700 9050 1700
Wire Notes Line
	8250 1100 9050 1100
Wire Wire Line
	2950 2400 2950 3750
Wire Wire Line
	2800 2600 2800 3850
Wire Wire Line
	6350 3600 7150 3600
Wire Wire Line
	6350 2550 6350 3600
Wire Wire Line
	3050 2550 6350 2550
Wire Wire Line
	3000 2600 3000 3650
Wire Wire Line
	3050 2550 3050 3550
Wire Wire Line
	3200 3450 3100 3450
Wire Wire Line
	3100 2500 3100 3450
Wire Wire Line
	3150 3350 3200 3350
Wire Wire Line
	3150 2450 3150 3350
$EndSCHEMATC
