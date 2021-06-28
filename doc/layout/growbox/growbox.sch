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
F 0 "A?" H 3700 4931 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 3700 4840 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 3700 3750 50  0001 C CIN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 3700 3750 50  0001 C CNN
	1    3700 3750
	1    0    0    -1  
$EndComp
Text GLabel 2100 2400 0    50   Input ~ 0
D5
Wire Wire Line
	3200 3650 2950 3650
Wire Wire Line
	2950 3650 2950 2400
Text GLabel 2100 2600 0    50   Input ~ 0
D6
Wire Wire Line
	2800 2600 2800 3750
Wire Wire Line
	2800 3750 3200 3750
Text GLabel 2150 2800 0    50   Input ~ 0
GND
Wire Wire Line
	2500 2800 2500 4950
Wire Wire Line
	3600 4950 3600 4850
Text GLabel 9000 2950 2    50   Input ~ 0
L1_230V
Text Notes 8850 2800 0    50   ~ 0
Ruck Rohrventilator\nEM125L-EC-02\n46W\n0.4 A_max
Text GLabel 9000 3100 2    50   Input ~ 0
N_230V
Text GLabel 9000 3250 2    50   Input ~ 0
PWM_in
Text GLabel 9000 3400 2    50   Input ~ 0
PWM_gnd
Text Notes 1950 2050 0    50   ~ 0
NodeMCU ESP8266\nJoy-IT\nhttps://joy-it.net/de/products/SBC-NodeMCU\n
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
Wire Wire Line
	3800 2200 2150 2200
Wire Wire Line
	3800 2200 3800 2750
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
Text Notes 8850 1200 0    50   ~ 0
Heizung\n700 W
Text GLabel 8850 1350 2    50   Input ~ 0
L1_230V
Text GLabel 8850 1500 2    50   Input ~ 0
N_230V
Text GLabel 4900 1500 2    50   Input ~ 0
L1_230V
Text GLabel 4900 1650 2    50   Input ~ 0
N_230V
Text Notes 4900 1400 0    50   ~ 0
Netz
Wire Wire Line
	2950 5100 8650 5100
Wire Wire Line
	8800 5000 5950 5000
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
Text GLabel 6650 1900 2    50   Input ~ 0
5V
Text GLabel 6650 2050 2    50   Input ~ 0
GND
Text Notes 6900 1800 0    50   ~ 0
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
	7500 2250 8300 2250
Wire Wire Line
	8300 2250 8300 1350
Wire Wire Line
	8300 1350 8850 1350
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
	3200 3150 3150 3150
Wire Wire Line
	3150 3150 3150 2450
Wire Wire Line
	3150 2450 7150 2450
Wire Wire Line
	3200 3250 3100 3250
Wire Wire Line
	3100 3250 3100 2500
Wire Wire Line
	3100 2500 6400 2500
Wire Wire Line
	6400 2500 6400 3050
Wire Wire Line
	6400 3050 7150 3050
Wire Wire Line
	3200 3350 3050 3350
Wire Wire Line
	3050 3350 3050 2550
Wire Wire Line
	3050 2550 6350 2550
Wire Wire Line
	6350 2550 6350 3600
Wire Wire Line
	6350 3600 7150 3600
Wire Wire Line
	3200 3450 3000 3450
Wire Wire Line
	3000 3450 3000 2600
Wire Wire Line
	3000 2600 6250 2600
Wire Wire Line
	6250 2600 6250 4200
Wire Wire Line
	6250 4200 7150 4200
Wire Wire Line
	3900 2750 5900 2750
Wire Wire Line
	5900 2750 5900 1900
Wire Wire Line
	5900 1900 6650 1900
Wire Wire Line
	6600 2050 5950 2050
Wire Wire Line
	5950 2050 5950 5000
Connection ~ 5950 5000
Wire Wire Line
	5950 5000 4750 5000
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
	4800 5550 8050 5550
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
	8850 1500 8050 1500
Wire Wire Line
	8050 1500 8050 5550
Connection ~ 8050 5550
Wire Wire Line
	8050 5550 8200 5550
Wire Wire Line
	9450 5200 9450 5550
Wire Wire Line
	9450 5550 9350 5550
Wire Wire Line
	9450 5200 9550 5200
Connection ~ 9350 5550
Wire Wire Line
	9350 4500 9350 5550
$EndSCHEMATC
